#!/usr/bin/env python3
"""
video_mujoco_compare.py — GTA5 视频 vs MuJoCo 平台 并排对比

左边：GTA5 原画面；右边：用该画面推导的遥测驱动的 MuJoCo 平台。
取视频中间一段（默认从 400s 起 30s），输出 mp4 供逐帧对比准确度与运动效果。

数据链与实机完全一致：
  画面 → 光流/p75 → VisionMotion → 25浮点遥测 → PoseFromMotion+FilterChain
       → IK → MuJoCo 闭环平台

用法:
  ../G_chair_mujoco/.venv/bin/python video_mujoco_compare.py \
      --video ../GTA5_drive_video.mp4 --start 400 --duration 30 \
      --out video_compare.mp4
"""

import argparse
import os
import sys

import cv2
import imageio.v2 as imageio
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'G_chair_mujoco'))
sys.path.insert(0, os.path.join(HERE, '..', 'G_chair_sim'))

import mujoco
from sim import GChairSim, OUTPUT
from screen_telemetry import VisionMotion, build_telemetry
from pose_from_motion import PoseFromMotion
from filters import FilterChain
from config import FILTERS


def telemetry_to_pose(tele, dt, mapper, filt):
    pf = mapper.apply_filters(mapper.compute(tele, dt), filt, dt)
    return np.array([
        pf['sway'] / 1000.0, pf['surge'] / 1000.0, pf['heave'] / 1000.0,
        np.radians(pf['roll']), np.radians(pf['pitch']), np.radians(pf['yaw']),
    ])


def main():
    ap = argparse.ArgumentParser(description='GTA5 视频 vs MuJoCo 并排对比')
    ap.add_argument('--video', default='../GTA5_drive_video.mp4')
    ap.add_argument('--start', type=float, default=400.0, help='视频起始秒数')
    ap.add_argument('--duration', type=float, default=30.0)
    ap.add_argument('--out', default='video_compare.mp4')
    ap.add_argument('--out-fps', type=int, default=15, help='输出帧率 (每 N 个视频帧取1)')
    ap.add_argument('--width', type=int, default=640, help='每半边宽度')
    ap.add_argument('--height', type=int, default=360, help='每半边高度')
    # 相机: 默认=GTA5 追尾视角（在座椅背侧 +Y、斜上方俯视椅背）
    ap.add_argument('--cam-azimuth', type=float, default=90.0,
                    help='相机方位角 (90=正后方对着椅背)')
    ap.add_argument('--cam-elevation', type=float, default=20.0,
                    help='相机仰角 (正=斜上方俯视)')
    ap.add_argument('--cam-distance', type=float, default=1.9)
    ap.add_argument('--cam-lookat-z', type=float, default=0.55)
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print('无法打开 %s' % args.video)
        return
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    cap.set(cv2.CAP_PROP_POS_MSEC, args.start * 1000)
    dt = 1.0 / fps
    skip = max(1, int(round(fps / args.out_fps)))
    print('视频 %.1ffps, 段 %.0f~%.0fs, 每 %d 帧取1 -> %.0ffps 输出' %
          (fps, args.start, args.start + args.duration, skip, fps / skip))

    sim = GChairSim()
    motion = VisionMotion()
    mapper = PoseFromMotion()
    filt = FilterChain(FILTERS)
    renderer = mujoco.Renderer(sim.model, height=args.height, width=args.width)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0, 0, args.cam_lookat_z]
    cam.distance = args.cam_distance
    cam.elevation = args.cam_elevation
    cam.azimuth = args.cam_azimuth
    print('相机: 方位 %.0f°, 仰角 %.0f°, 距离 %.1f' %
          (cam.azimuth, cam.elevation, cam.distance))

    out_frames = []
    log = {'t': [], 'cmd': [], 'act': []}
    n = int(args.duration * fps)
    for i in range(n):
        ok, frame = cap.read()
        if not ok:
            print('视频读完')
            break
        t = args.start + i / fps
        m, raw, speed, valid = motion.process(frame, i / fps)
        tele = build_telemetry(m, speed, 0.0, valid, t)
        pose = telemetry_to_pose(tele, dt, mapper, filt)
        sim.command_pose(pose, dt)
        sim.step()
        act = sim.platform_pose()

        log['t'].append(t)
        log['cmd'].append(pose)
        log['act'].append(act)

        if i % skip == 0:
            left = cv2.resize(frame, (args.width, args.height))
            renderer.update_scene(sim.data, camera=cam)
            rgb = renderer.render()
            right = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            combo = np.hstack([left, right])
            cv2.putText(combo, 't=%.1fs' % t, (10, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(combo, 'surge %.2fG' % m['accel_z'], (10, 54),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
            cv2.putText(combo, 'GTA5', (args.width - 100, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(combo, 'MuJoCo', (args.width * 2 - 140, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            out_frames.append(combo)

    cap.release()
    renderer.close()
    print('采集 %d 帧对比图' % len(out_frames))

    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or '.', exist_ok=True)
    imageio.mimsave(args.out, out_frames, fps=args.out_fps, codec='libx264',
                    quality=8, macro_block_size=2)
    print('[video] %s' % args.out)

    # 曲线 + CSV
    t = np.array(log['t'])
    cmd = np.array(log['cmd'])
    act = np.array(log['act'])
    pe = np.linalg.norm((act[:, :3] - cmd[:, :3]) * 1000, axis=1)
    re = np.degrees(np.linalg.norm(act[:, 3:] - cmd[:, 3:], axis=1))
    print('位置跟踪 RMS/MAX: %.2f / %.2f mm; 姿态 RMS/MAX: %.2f / %.2f deg' %
          (np.sqrt(np.mean(pe ** 2)), pe.max(),
           np.sqrt(np.mean(re ** 2)), re.max()))

    csv_path = os.path.join(os.path.dirname(os.path.abspath(args.out)), 'compare_log.csv')
    arr = np.c_[t, cmd, act]
    header = 't,' + ','.join('cmd_' + s for s in 'x y z roll pitch yaw'.split()) + \
             ',' + ','.join('act_' + s for s in 'x y z roll pitch yaw'.split())
    np.savetxt(csv_path, arr, delimiter=',', header=header, comments='')
    print('[csv] %s' % csv_path)

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(2, 1, figsize=(13, 6), sharex=True)
        axs[0].plot(t, cmd[:, 2] * 1000, '--', lw=0.8, label='cmd heave')
        axs[0].plot(t, act[:, 2] * 1000, lw=0.9, label='act heave')
        axs[0].set_ylabel('heave (mm)'); axs[0].legend(fontsize=8)
        axs[0].set_title('Video-driven MuJoCo tracking (%.0f~%.0fs)' %
                         (args.start, args.start + args.duration))
        axs[1].plot(t, np.degrees(cmd[:, 3]), '--', lw=0.8, label='cmd roll')
        axs[1].plot(t, np.degrees(act[:, 3]), lw=0.9, label='act roll')
        axs[1].set_ylabel('roll (deg)'); axs[1].set_xlabel('video time (s)')
        axs[1].legend(fontsize=8)
        fig.tight_layout()
        p = os.path.join(os.path.dirname(os.path.abspath(args.out)), 'compare_plot.png')
        fig.savefig(p, dpi=110)
        print('[plot] %s' % p)
    except ImportError:
        pass


if __name__ == '__main__':
    main()
