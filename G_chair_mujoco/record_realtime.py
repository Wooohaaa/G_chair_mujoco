#!/usr/bin/env python3
"""
record_realtime.py — 用真实 UDP 遥测驱动 MuJoCo 并离屏渲染成 mp4。

从 sim_udp.py 复用数据链（UDP/TCP 接收 → 姿态映射 → IK → MuJoCo），
每帧渲染一帧画面，录成视频，供反复观看演示效果。

用法:
  ./.venv/bin/python record_realtime.py --duration 30 --out output/real_gta5_demo.mp4
"""
import argparse
import os
import sys
import time

import numpy as np
import mujoco
import imageio.v2 as imageio

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'G_chair_sim'))

from sim import GChairSim, OUTPUT
from udp_telemetry import UdpTelemetryReceiver
from pose_from_motion import PoseFromMotion
from filters import FilterChain
from config import FILTERS


def main():
    ap = argparse.ArgumentParser(description='真实数据驱动 MuJoCo 录制')
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--port', type=int, default=4124)
    ap.add_argument('--duration', type=float, default=30.0)
    ap.add_argument('--out', default='output/real_gta5_demo.mp4')
    ap.add_argument('--fps', type=int, default=25)
    ap.add_argument('--width', type=int, default=960)
    ap.add_argument('--height', type=int, default=540)
    ap.add_argument('--wait', type=float, default=10.0)
    args = ap.parse_args()

    sim = GChairSim()
    mapper = PoseFromMotion()
    filt = FilterChain(FILTERS)
    rx = UdpTelemetryReceiver(args.host, args.port)
    rx.start()

    print('[udp] 监听 %s:%d, 等待首帧...' % (args.host, args.port))
    t0 = time.time()
    while time.time() - t0 < args.wait and rx.get() is None:
        time.sleep(0.02)
    if rx.get() is None:
        print('[udp] 未收到数据'); rx.stop(); return
    print('[udp] 收到数据, 开始录制 %.0fs' % args.duration)

    renderer = mujoco.Renderer(sim.model, height=args.height, width=args.width)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0, 0, 0.55]
    cam.distance = 1.9
    cam.elevation = 20
    cam.azimuth = 90
    cam.lookat[:] = [0, 0, 0.6]

    dt_ctrl = 1.0 / 50.0
    frames = []
    t_start = time.time()
    n_ctrl = int(args.duration / dt_ctrl)
    render_every = max(1, int(round(args.fps * dt_ctrl)))

    try:
        for k in range(n_ctrl):
            t = k * dt_ctrl
            item = rx.get(max_age=1.0)
            if item is None:
                tele = dict.fromkeys(['accel_x', 'accel_z', 'accel_y',
                                      'rot_x', 'rot_y', 'rot_z'], 0.0)
            else:
                tele = item[0]
            pf = mapper.apply_filters(mapper.compute(tele, dt_ctrl), filt, dt_ctrl)
            import math
            pose = np.array([pf['sway']/1000, pf['surge']/1000, pf['heave']/1000,
                             math.radians(pf['roll']), math.radians(pf['pitch']),
                             math.radians(pf['yaw'])])
            sim.command_pose(pose, dt_ctrl)
            sim.step()

            if k % render_every == 0:
                cam.azimuth = 90 + 8 * np.sin(2 * np.pi * 0.05 * t)
                renderer.update_scene(sim.data, camera=cam)
                frames.append(renderer.render().copy())

            # 按真实时间播放
            lag = (t + dt_ctrl) - (time.time() - t_start)
            if lag > 0:
                time.sleep(lag)
    except KeyboardInterrupt:
        print('\n中断')
    finally:
        renderer.close()
        rx.stop()

    if not frames:
        print('无帧'); return
    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or '.', exist_ok=True)
    imageio.mimsave(args.out, frames, fps=args.fps, codec='libx264',
                    quality=8, macro_block_size=2)
    print('[video] 已保存 %s (%d 帧, %.1fs)' % (args.out, len(frames), len(frames)/args.fps))


if __name__ == '__main__':
    main()
