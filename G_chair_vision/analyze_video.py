#!/usr/bin/env python3
"""
analyze_video.py — 用真实 GTA5 录像对比 ROI 策略，标定光流参数。

对同一段视频，用多种 ROI 计算光流特征，输出 CSV + 对比图，
看哪种 ROI 的"速度代理"最平滑、最不受车身晃动污染。

画面特点（已确认）：第三人称追尾视角，车在画面下方正中，无 HUD。
所以 ROI 必须【排除中间的车】，只取车两侧的路面。

用法:
  ./.venv/bin/python analyze_video.py \
      --video ../GTA5_drive_video.mp4 --start 25 --duration 60 --step 2
"""

import argparse
import os

import cv2
import numpy as np

PROC_W, PROC_H = 320, 180

# ROI 策略: (名称, y0比例, x0比例, x1比例, 是否排除中间车)
STRATEGIES = [
    ('center_old', 0.35, 0.15, 0.85, False),   # 原方案: 包含车（有问题）
    ('sides',      0.40, 0.03, 0.97, True),    # 车两侧路面
    ('lower_full', 0.55, 0.03, 0.97, False),   # 更靠下（近处路面）
    ('upper_road', 0.35, 0.03, 0.97, True),    # 稍高，含远处路面
]


def roi_mask(shape, y0f, x0f, x1f, exclude_car, car_half=0.13):
    h, w = shape
    mask = np.zeros((h, w), dtype=np.uint8)
    y0 = int(h * y0f)
    x0, x1 = int(w * x0f), int(w * x1f)
    mask[y0:, x0:x1] = 1
    if exclude_car:
        cx = w // 2
        mask[y0:, cx - int(w * car_half):cx + int(w * car_half)] = 0
    return mask.astype(bool)


def flow_features(prev_gray, gray, mask):
    flow = cv2.calcOpticalFlowFarneback(prev_gray, gray, None,
                                        0.5, 3, 15, 3, 5, 1.2, 0)
    mag = np.sqrt(flow[:, :, 0] ** 2 + flow[:, :, 1] ** 2)
    fy = flow[:, :, 1]
    fx = flow[:, :, 0]
    return dict(
        fy_med=float(np.median(fy[mask])),
        fx_med=float(np.median(fx[mask])),
        fy_p75=float(np.percentile(fy[mask], 75)),
        fy_p90=float(np.percentile(fy[mask], 90)),
        fx_p75=float(np.percentile(fx[mask], 75)),
        fx_p25=float(np.percentile(fx[mask], 25)),
        mag_mean=float(np.mean(mag[mask])),
        mag_p75=float(np.percentile(mag[mask], 75)),
        fy_std=float(np.std(fy[mask])),
    )


def main():
    ap = argparse.ArgumentParser(description='GTA5 录像光流 ROI 对比')
    ap.add_argument('--video', default='../GTA5_drive_video.mp4')
    ap.add_argument('--start', type=float, default=25.0)
    ap.add_argument('--duration', type=float, default=60.0)
    ap.add_argument('--step', type=int, default=2, help='每 N 帧取样')
    ap.add_argument('--debug-dir', default=None, help='保存 ROI 可视化帧')
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print('无法打开 %s' % args.video)
        return
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total = cap.get(cv2.CAP_PROP_FRAME_COUNT)
    print('视频 %.0fx%.0f @ %.1ffps, 共 %.0f 帧' %
          (cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
           fps, total))

    cap.set(cv2.CAP_PROP_POS_MSEC, args.start * 1000)
    masks = {name: roi_mask((PROC_H, PROC_W), y0, x0, x1, excl)
             for name, y0, x0, x1, excl in STRATEGIES}

    rows = []
    prev_gray = None
    idx = 0
    target = int(args.duration * fps / args.step)
    if args.debug_dir:
        os.makedirs(args.debug_dir, exist_ok=True)

    while len(rows) < target:
        ok, frame = cap.read()
        if not ok:
            break
        if idx % args.step != 0:
            idx += 1
            continue
        idx += 1
        t = args.start + len(rows) * args.step / fps
        proc = cv2.resize(frame, (PROC_W, PROC_H))
        gray = cv2.cvtColor(proc, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        if prev_gray is not None:
            row = {'t': t}
            for name, mask in masks.items():
                f = flow_features(prev_gray, gray, mask)
                row[name + '_fy'] = f['fy_med']
                row[name + '_fy75'] = f['fy_p75']
                row[name + '_fy90'] = f['fy_p90']
                row[name + '_fx'] = f['fx_med']
                row[name + '_fx75'] = f['fx_p75']
                row[name + '_fx25'] = f['fx_p25']
                row[name + '_mag'] = f['mag_mean']
                row[name + '_mag75'] = f['mag_p75']
                row[name + '_std'] = f['fy_std']
            rows.append(row)
        prev_gray = gray.copy()

        if args.debug_dir and len(rows) % 30 == 1:
            dbg = proc.copy()
            for name, mask in masks.items():
                ov = dbg.copy()
                ov[mask] = (0.5 * ov[mask] + 0.5 * np.array([0, 255, 0])).astype(np.uint8)
                cv2.imwrite(os.path.join(args.debug_dir, '%s_%04d.jpg' %
                                         (name, len(rows))), ov)

    cap.release()
    if not rows:
        print('没有取到帧')
        return

    keys = list(rows[0].keys())
    out_csv = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'video_flow.csv')
    with open(out_csv, 'w') as fh:
        fh.write(','.join(keys) + '\n')
        for r in rows:
            fh.write(','.join('%.5f' % r[k] for k in keys) + '\n')
    print('已保存 %s (%d 行)' % (out_csv, len(rows)))

    t = np.array([r['t'] for r in rows])
    print('\n%-12s %10s %10s %10s %10s %10s' %
          ('ROI', 'fy中位', 'fy_p75', 'fy_p90', 'fx75-25', 'mag_p75'))
    for name, *_ in STRATEGIES:
        g = lambda k: np.array([r[name + '_' + k] for r in rows])
        print('%-12s %10.3f %10.3f %10.3f %10.3f %10.3f' %
              (name, np.median(g('fy')), np.median(g('fy75')),
               np.median(g('fy90')), np.median(g('fx75') - g('fx25')),
               np.median(g('mag75'))))

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
        for name, *_ in STRATEGIES:
            axs[0].plot(t, [r[name + '_fy75'] for r in rows], label=name, lw=0.8)
        axs[0].set_ylabel('fy p75 (px/frame)')
        axs[0].set_title('GTA5 video: vertical flow p75 by ROI (forward speed proxy)')
        axs[0].legend(fontsize=8)
        for name, *_ in STRATEGIES:
            axs[1].plot(t, [r[name + '_fx75'] - r[name + '_fx25'] for r in rows],
                        label=name, lw=0.8)
        axs[1].set_ylabel('fx p75-p25 (lateral flow spread)')
        axs[1].set_xlabel('video time (s)')
        axs[1].legend(fontsize=8)
        fig.tight_layout()
        p = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'video_flow.png')
        fig.savefig(p, dpi=110)
        print('[plot] %s' % p)
    except ImportError:
        pass


if __name__ == '__main__':
    main()
