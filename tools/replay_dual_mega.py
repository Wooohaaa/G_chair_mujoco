#!/usr/bin/env python3
"""把 GTA5 录制遥测离线生成为老双 Mega 4轴协议帧，并给出安全统计。

默认只生成帧文件，不碰串口。加 --serial PORT1 PORT2 才实际发送。
关键：老固件步数是相对上电位置的无符号值，0mm 遥测对应 BASE 步（偏置），
      因此先斜坡升到 BASE，再叠加 ±位移，最后斜坡回 0。

用法:
  ./.venv/bin/python tools/replay_dual_mega.py --dry-run
  sudo ./.venv/bin/python tools/replay_dual_mega.py --serial /dev/ttyUSB1 /dev/ttyUSB2
"""
import argparse
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'G_chair_sim'))

from config import TELEMETRY_CHANNELS
from motion_pipeline import MotionPipeline

DT = 0.02
# 缸号 -> 板/通道（实测 HARDWARE_MAP.md）
from dual_mega_map import CYL_TO_BOARD


def make_frames(disp_mm, base_steps, scale, max_mm):
    """输入 (n,6) 缸位移mm，输出两板帧序列 [(bytes,bytes), ...]。"""
    frames = []
    for d in disp_mm:
        vals = np.clip(d * scale, -max_mm, max_mm)
        steps = np.clip(np.round(base_steps + vals / 0.005), 0, 38400).astype(int)
        boards = []
        for bi in range(2):
            ch = [0, 0, 0, 0]
            for cyl, (board, channel) in CYL_TO_BOARD.items():
                if board == bi:
                    ch[channel] = int(round(steps[cyl] * 1.7))  # 老固件除数1.7
            boards.append(bytes([255, 255]) + b''.join(
                v.to_bytes(2, 'big') for v in ch))
        frames.append(tuple(boards))
    return frames


def ramp(target_mm, n, up=True):
    """从 0mm 平滑过渡到 target_mm（或反向），保证与回放首/末帧连续。"""
    arr = []
    for k in range(n):
        v = k / (n - 1) if up else (n - 1 - k) / (n - 1)
        arr.append(np.asarray(target_mm, dtype=float) * v)
    return arr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', default=os.path.join(ROOT, 'G_chair_mujoco',
                                                  'output', 'raw_telemetry.csv'))
    ap.add_argument('--seconds', type=float, default=10.0)
    ap.add_argument('--start-frame', type=int, default=0,
                    help='从第几帧开始（用于选取录制中最剧烈的片段）')
    ap.add_argument('--scale', type=float, default=0.5)
    ap.add_argument('--max-mm', type=float, default=25.0)
    ap.add_argument('--base-steps', type=int, default=6000)   # 30mm 偏置
    ap.add_argument('--out', default=os.path.join(ROOT, 'output', 'dual_mega_frames.json'))
    ap.add_argument('--serial', nargs=2, metavar=('PORT1', 'PORT2'))
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    if not os.path.exists(args.csv):
        print('找不到', args.csv); return 1
    data = np.loadtxt(args.csv, delimiter=',', skiprows=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    n = int(args.seconds / DT)
    data = data[args.start_frame:args.start_frame + n]
    if len(data) < n:
        print('数据不足: 从%d帧起需要%d帧, 实际可用%d帧'
              % (args.start_frame, n, len(data))); return 1
    print('选取片段: 第 %d~%d 帧 (%.1f~%.1fs)'
          % (args.start_frame, args.start_frame + n,
             args.start_frame * DT, (args.start_frame + n) * DT))

    mp = MotionPipeline()
    disp = []
    for row in data:
        tele = {k: float(row[j]) for j, k in enumerate(TELEMETRY_CHANNELS)}
        _, d = mp.pose_and_disp(tele, DT)
        disp.append(d)
    disp = np.array(disp)
    raw = make_frames(disp, args.base_steps, args.scale, args.max_mm)
    t0 = np.clip(disp[0] * args.scale, -args.max_mm, args.max_mm)
    t1 = np.clip(disp[-1] * args.scale, -args.max_mm, args.max_mm)
    # 从 0 步（上电位置）平滑升到回放首帧，回放结束后平滑回 0
    base_mm = -args.base_steps * 0.005   # 0步对应的“相对中位mm”
    up = [base_mm + (t0 - base_mm) * (k / 149) for k in range(150)]
    down = [t1 + (base_mm - t1) * (k / 149) for k in range(150)]
    seq = make_frames(up, args.base_steps, 1.0, 1000) + raw + \
          make_frames(down, args.base_steps, 1.0, 1000)

    all_steps = []
    for d in np.vstack([up, np.clip(disp * args.scale, -args.max_mm, args.max_mm), down]):
        all_steps.append(np.clip(args.base_steps + d / 0.005, 0, 38400))
    all_steps = np.array(all_steps)
    jump = np.max(np.abs(np.diff(all_steps, axis=0)), axis=0)

    print('=' * 60)
    print('老双 Mega 回放预览: %s' % os.path.basename(args.csv))
    print('  回放 %.1fs (%d帧) + 升降斜坡各 3.0s，共 %d 帧/板'
          % (args.seconds, n, len(seq)))
    print('  缩放 %.2f, 限幅 +/-%.1fmm, 偏置 %d步(%.1fmm)'
          % (args.scale, args.max_mm, args.base_steps, args.base_steps * 0.005))
    print('  缸位移范围(mm): %s' %
          ' '.join('[%+.1f,%+.1f]' % (x, y) for x, y in
                   zip(all_steps.min(axis=0) * 0.005, all_steps.max(axis=0) * 0.005)))
    vmm = jump.max() * 0.005 * 50   # 步/帧 -> mm/s
    print('  最大单帧跳变: %.2fmm (%.0fmm/s @50Hz)' % (jump.max() * 0.005, vmm))
    safe = (vmm <= 110 and all_steps.max() <= 38400 and all_steps.min() >= 0)
    print('  安全判定: %s' % ('OK (<=110mm/s, 行程内)' if safe
                            else '❌ 超出限制，请调小 scale/max-mm'))
    print('=' * 60)
    if not safe:
        print('安全检查未通过，拒绝发送。')
        return 1

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w') as f:
        json.dump({'meta': {'seconds': args.seconds, 'scale': args.scale,
                            'base_steps': args.base_steps, 'max_mm': args.max_mm},
                   'frames': [[b[0].hex(), b[1].hex()] for b in seq]}, f)
    print('帧已保存:', args.out)

    if args.serial:
        import serial
        s1, s2 = [serial.Serial(p, 115200, timeout=0.1) for p in args.serial]
        for s in (s1, s2):
            s.dtr = False; s.rts = True; time.sleep(0.15)
            s.dtr = True; s.rts = False; time.sleep(0.15)
            s.reset_input_buffer()
        time.sleep(1.0)
        print('开始发送（10s回放 + 6s斜坡）...')
        t0 = time.time(); next_t = t0
        for i, (f1, f2) in enumerate(seq):
            s1.write(f1); s2.write(f2)
            next_t += DT
            time.sleep(max(0, next_t - time.time()))
        s1.write(bytes([255, 255]) + b'\x00\x00' * 4)
        s2.write(bytes([255, 255]) + b'\x00\x00' * 4)
        time.sleep(0.3)
        s1.close(); s2.close()
        print('完成，已固定回0，耗时 %.1fs' % (time.time() - t0))
    return 0


if __name__ == '__main__':
    sys.exit(main())
