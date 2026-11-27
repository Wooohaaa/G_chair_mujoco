#!/usr/bin/env python3
"""
benchmark.py — 测量视觉节点的真实处理帧率和延迟分解。

测三段:
  1. 各阶段耗时 (读帧/缩放灰度/光流/映射+滤波)
  2. 纯算法最大吞吐 (脱离采集源，喂固定帧)
  3. 端到端延迟 (模拟采集→出UDP)

用法:
  # 本机 / RK3566 通用, 有摄像头或视频都行
  python3 benchmark.py --source /dev/video9 --frames 200
  python3 benchmark.py --mock --frames 200
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from screen_telemetry import (VisionMotion, PROC_W, PROC_H,
                              build_telemetry, MockCapture)


def pct(a, q):
    return float(np.percentile(a, q))


def run(args):
    if args.mock:
        cap = MockCapture()
        print('源: 合成路面')
    else:
        cap = cv2.VideoCapture(args.source)
        if not cap.isOpened():
            print('无法打开 %s' % args.source)
            return
        print('源: %s' % args.source)

    proc_size = (256, 144) if args.fast else (PROC_W, PROC_H)
    motion = VisionMotion(use_yolo=args.yolo, fast_flow=args.fast,
                          proc_size=proc_size)
    print('模式: %s, 分辨率 %s' % ('FAST' if args.fast else '默认', proc_size))
    n = args.frames
    t_read, t_pre, t_flow, t_map, t_total = [], [], [], [], []
    prev = None

    for i in range(n):
        t0 = time.perf_counter()
        if args.mock:
            ok, frame = cap.read()
        else:
            ok, frame = cap.read()
        t1 = time.perf_counter()

        proc = cv2.resize(frame, motion.proc_size)
        gray = cv2.cvtColor(proc, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        t2 = time.perf_counter()

        m, raw, speed, valid = motion.process(frame, i / 30.0)
        t3 = time.perf_counter()

        tele = build_telemetry(m, speed, 0.0, valid, i / 30.0)
        payload = ' '.join('%.4f' % tele[k] for k in
                           ['accel_x'] * 25)  # 占位，下面用真顺序
        t4 = time.perf_counter()

        t_read.append((t1 - t0) * 1000)
        t_pre.append((t2 - t1) * 1000)
        t_flow.append((t3 - t2) * 1000)
        t_map.append((t4 - t3) * 1000)
        t_total.append((t4 - t0) * 1000)

    if not args.mock:
        cap.release()

    print('\n========== 处理耗时 (ms), %d 帧 ==========' % n)
    for name, arr in [('读帧', t_read), ('缩放+灰度', t_pre),
                      ('光流+滤波(process)', t_flow), ('映射+打包', t_map),
                      ('端到端', t_total)]:
        a = np.array(arr)
        print('  %-18s 均值 %7.2f  P50 %7.2f  P95 %7.2f  MAX %7.2f'
              % (name, a.mean(), pct(a, 50), pct(a, 95), a.max()))
    fps = 1000.0 / np.mean(t_total)
    print('\n  纯处理吞吐: %.1f FPS (理论上限)' % fps)
    print('  其中光流占比: %.1f%%' % (100 * np.mean(t_flow) / np.mean(t_total)))


def bench_pure(args):
    """脱离采集源，纯算法最大吞吐。"""
    sys.path.insert(0, os.path.join(HERE, '..', 'G_chair_sim'))
    from config import TELEMETRY_CHANNELS
    motion = VisionMotion()
    frame = np.random.randint(0, 255, (PROC_H, PROC_W, 3), dtype=np.uint8)
    n = args.frames
    dt = []
    for i in range(n):
        t = time.perf_counter()
        m, raw, speed, valid = motion.process(frame, i / 30.0)
        tele = build_telemetry(m, speed, 0.0, valid, i / 30.0)
        payload = ' '.join('%.4f' % tele[k] for k in TELEMETRY_CHANNELS)
        dt.append((time.perf_counter() - t) * 1000)
    a = np.array(dt)
    print('\n========== 纯算法吞吐 (随机帧, %d 次) ==========' % n)
    print('  均值 %.2f ms  P50 %.2f  P95 %.2f  MAX %.2f'
          % (a.mean(), pct(a, 50), pct(a, 95), a.max()))
    print('  理论上限 %.1f FPS' % (1000.0 / a.mean()))


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', default='/dev/video9')
    ap.add_argument('--mock', action='store_true')
    ap.add_argument('--frames', type=int, default=200)
    ap.add_argument('--yolo', action='store_true')
    ap.add_argument('--fast', action='store_true', help='轻量光流(256x144, levels2/win11/it1)')
    ap.add_argument('--pure', action='store_true', help='只测纯算法吞吐')
    args = ap.parse_args()
    if args.pure:
        bench_pure(args)
    else:
        run(args)
