#!/usr/bin/env python3
"""
latency_test.py — 端到端延迟测量：发送端把发送时刻打进 time 通道，
接收端用 (收到时刻 - 发送时刻) 得到真实延迟。

用法:
  接收: python3 latency_test.py recv --port 4130
  发送: python3 latency_test.py send --ip 127.0.0.1 --port 4130 --source /dev/video9
"""
import argparse, os, socket, sys, time
import numpy as np
import cv2
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'G_chair_sim'))
from screen_telemetry import VisionMotion, build_telemetry, PROC_W, PROC_H
from config import TELEMETRY_CHANNELS


def send(args):
    motion = VisionMotion()
    cap = None if args.mock else cv2.VideoCapture(args.source)
    if cap and not cap.isOpened():
        print('无法打开', args.source); return
    if args.tcp:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((args.ip, args.port))
        print('TCP 连接到 %s:%d' % (args.ip, args.port), flush=True)
    else:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    remote = (args.ip, args.port)
    print('发送 %d 帧' % args.frames, flush=True)
    for i in range(args.frames):
        if cap:
            ok, f = cap.read()
            if not ok: break
        else:
            f = np.random.randint(0, 255, (PROC_H, PROC_W, 3), np.uint8)
        m, raw, speed, valid = motion.process(f, i / 30.0)
        tele = build_telemetry(m, speed, 0.0, valid, i / 30.0)
        t_send = time.time()          # 发前打时间戳
        tele['time'] = t_send
        payload = ' '.join('%.6f' % tele[k] for k in TELEMETRY_CHANNELS)
        if args.tcp:
            s.sendall(payload.encode() + b'\n')
        else:
            s.sendto(payload.encode(), remote)
        if i % 50 == 0: print('sent', i, flush=True)
    s.close()


def recv(args):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM if args.tcp else socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('0.0.0.0', args.port))
    if args.tcp:
        s.listen(1)
        s.settimeout(args.timeout)   # 防止 accept 永久阻塞
        try:
            conn, _ = s.accept()
        except socket.timeout:
            print('等待连接超时'); s.close(); return
        conn.settimeout(args.timeout)
        src = conn
    else:
        s.settimeout(args.timeout)
        src = s
    lat = []
    t0 = time.time()
    buf = b''
    while time.time() - t0 < args.timeout:
        try:
            if args.tcp:
                chunk = conn.recv(8192)
                if not chunk: break
                buf += chunk
                if b'\n' not in buf: continue
                data, buf = buf.split(b'\n', 1)
            else:
                data, _ = src.recvfrom(8192)
        except socket.timeout:
            break
        v = data.decode('ascii', 'ignore').split()
        if len(v) < 25:
            continue
        try:
            t_send = float(v[24])
        except ValueError:
            continue
        t_recv = time.time()
        lat.append((t_recv - t_send) * 1000.0)
    s.close()
    if not lat:
        print('未收到数据'); return
    a = np.array(lat)
    print('\n========== 端到端延迟 (%d 帧) ==========' % len(a))
    print('  P50 %.1f ms  P95 %.1f ms  MAX %.1f ms' %
          (np.percentile(a, 50), np.percentile(a, 95), a.max()))


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='mode')
    ps = sub.add_parser('send')
    ps.add_argument('--source', default='/dev/video9')
    ps.add_argument('--mock', action='store_true')
    ps.add_argument('--ip', default='127.0.0.1')
    ps.add_argument('--port', type=int, default=4130)
    ps.add_argument('--frames', type=int, default=300)
    ps.add_argument('--tcp', action='store_true')
    pr = sub.add_parser('recv')
    pr.add_argument('--port', type=int, default=4130)
    pr.add_argument('--timeout', type=float, default=30.0)
    pr.add_argument('--tcp', action='store_true')
    args = ap.parse_args()
    if args.mode == 'send': send(args)
    elif args.mode == 'recv': recv(args)
