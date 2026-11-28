#!/usr/bin/env python3
"""实时桥：局域网 GTA5 UDP 遥测 -> 映射管线 -> 老双 Mega 4轴帧。

安全设计：
  - 软件限速：每帧最多 110mm/s * 0.02s = 2.2mm（440步），从 0 步平滑起步
  - 绝对步数钳位 0..38400；位移限幅 +/-max_mm；缩放 scale
  - 遥测断流 >0.5s 冻结目标（保持当前位置），并告警
  - Ctrl+C 先斜坡回 0 再关串口；任何异常也走回零流程

默认 --dry-run 不碰串口，只打印运行统计。加 --serial PORT1 PORT2 才实际驱动。
"""
import argparse
import os
import signal
import socket
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'G_chair_sim'))

from config import TELEMETRY_CHANNELS
from motion_pipeline import MotionPipeline
from udp_telemetry import parse_datagram

from dual_mega_map import CYL_TO_BOARD
DT = 0.02
MAX_MM_S = 110.0
MAX_STEP = int(MAX_MM_S * DT / 0.005)   # 440 步/帧


def frames_from_steps(steps):
    out = []
    for bi in range(2):
        ch = [0, 0, 0, 0]
        for cyl, (board, channel) in CYL_TO_BOARD.items():
            if board == bi:
                ch[channel] = int(round(steps[cyl] * 1.7))
        out.append(bytes([255, 255]) + b''.join(
            max(0, min(65535, v)).to_bytes(2, 'big') for v in ch))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=4124)
    ap.add_argument('--serial', nargs=2, metavar=('PORT1', 'PORT2'))
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--scale', type=float, default=0.5)
    ap.add_argument('--max-mm', type=float, default=25.0)
    ap.add_argument('--base-steps', type=int, default=6000)
    ap.add_argument('--seconds', type=float, default=0, help='0=持续运行')
    ap.add_argument('--hz', type=float, default=50.0)
    args = ap.parse_args()

    if not args.dry_run and not args.serial:
        ap.error('必须指定 --serial PORT1 PORT2 或 --dry-run')
    if args.base_steps * 0.005 + args.max_mm > 95:
        print('偏置+限幅超出物理半行程，拒绝启动'); return 1

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', args.port))
    sock.settimeout(0.005)

    s1 = s2 = None
    if args.serial:
        import serial
        s1, s2 = [serial.Serial(p, 115200, timeout=0.1) for p in args.serial]
        for s in (s1, s2):
            s.dtr = False; s.rts = True; time.sleep(0.15)
            s.dtr = True; s.rts = False; time.sleep(0.15)
            s.reset_input_buffer()
        time.sleep(0.8)

    mp = MotionPipeline()
    applied = np.zeros(6)          # 当前实际下发步数，从0起步
    target = np.zeros(6)
    tele = None
    last_tele_t = time.time()
    stopping = False
    n = 0
    stale_warned = False
    t0 = time.time()
    next_t = t0

    def on_signal(sig, frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    print('实时桥启动: UDP :%d -> %s, scale=%.2f, 限幅±%.1fmm, 偏置%d步'
          % (args.port, args.serial if args.serial else 'DRY-RUN',
             args.scale, args.max_mm, args.base_steps), flush=True)
    try:
        while True:
            if stopping or (args.seconds and time.time() - t0 >= args.seconds):
                target = np.zeros(6)   # 平滑回零
            # 收最新遥测
            try:
                while True:
                    data, addr = sock.recvfrom(2048)
                    t = parse_datagram(data)
                    if t is not None:
                        tele = t
                        last_tele_t = time.time()
            except socket.timeout:
                pass
            if tele is not None and time.time() - last_tele_t > 0.5:
                if not stale_warned:
                    print('[warn] 遥测断流 >0.5s，保持当前位置', flush=True)
                    stale_warned = True
            elif time.time() - last_tele_t <= 0.5:
                stale_warned = False

            if tele is not None and not stopping:
                _, disp = mp.pose_and_disp(tele, DT)
                v = np.clip(disp * args.scale, -args.max_mm, args.max_mm)
                target = np.clip(np.round(args.base_steps + v / 0.005), 0, 38400)

            # 软件限速：每帧最多 MAX_STEP 步
            delta = target - applied
            applied += np.clip(delta, -MAX_STEP, MAX_STEP)
            applied = np.clip(applied, 0, 38400)

            if s1:
                f1, f2 = frames_from_steps(applied.astype(int))
                s1.write(f1); s2.write(f2)
            n += 1
            if n % 100 == 0:
                print('帧 %d, 目标步 %s, 实际步 %s'
                      % (n, np.round(target).astype(int).tolist(),
                         np.round(applied).astype(int).tolist()), flush=True)

            # 回零完成后退出
            if (stopping or (args.seconds and time.time() - t0 >= args.seconds)) \
                    and np.max(np.abs(applied)) == 0:
                break
            next_t += 1.0 / args.hz
            time.sleep(max(0, next_t - time.time()))
    finally:
        zero = frames_from_steps(np.zeros(6))
        if s1:
            s1.write(zero[0]); s2.write(zero[1])
            time.sleep(0.2)
            s1.close(); s2.close()
        sock.close()
        print('已固定回0并关闭，共 %d 帧' % n, flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
