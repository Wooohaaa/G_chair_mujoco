#!/usr/bin/env python3
"""
record_paired.py — 录制对齐的真值数据集（视频 + 真实UDP遥测）

【用途】
解决"视频提取姿态"优化缺真值的问题：采集卡插在仿真电脑上，
同时录 GTA5 画面 + 抓游戏机发来的 25 浮点 UDP 遥测，统一时钟，天然对齐。
产出的数据集用于：算相关系数、对比视觉推导 vs 真实遥测、评判每次算法改动。

【数据链】
  GTA5(mod) ──UDP :4124──> 本机 ─┬─> UDP接收线程 → 遥测CSV(带时间戳)
  HDMI采集卡(/dev/videoN) ───────┴─> 视频采集线程 → 视频mp4(同时刻起)

【用法】
  # 列出摄像头找出采集卡
  python3 record_paired.py --list

  # 录 60 秒（采集卡在 /dev/video2，按实际改）
  python3 record_paired.py --video /dev/video2 --duration 60 --out /tmp/opencode/paired_run1

  # 边录边看画面（可选）
  python3 record_paired.py --video /dev/video2 --duration 60 --preview

【产物】
  <out>_video.mp4       录制视频（30fps）
  <out>_telemetry.csv   真实遥测（每行带 t 相对秒数、25通道）
  <out>_meta.json       时间基准、帧率、对齐信息
"""
import argparse
import json
import os
import sys
import threading
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'G_chair_sim'))
from udp_telemetry import UdpTelemetryReceiver
from config import TELEMETRY_CHANNELS


def list_cameras():
    """列出可用摄像头及其分辨率。"""
    print('=== 检测摄像头 ===')
    for i in range(8):
        c = cv2.VideoCapture(i)
        if not c.isOpened():
            c.release()
            continue
        ok, frame = c.read()
        w = int(c.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(c.get(cv2.CAP_PROP_FRAME_HEIGHT))
        # 尝试读设备名
        name = ''
        try:
            name = open('/sys/class/video4linux/video%d/name' % i).read().strip()
        except Exception:
            pass
        print('  /dev/video%d  %s  %dx%d  可读=%s' % (i, name, w, h, ok))
        c.release()


def record(args):
    # ---- UDP 接收线程 ----
    rx = UdpTelemetryReceiver(args.host, args.port)
    rx.start()
    tele_rows = []          # [t, 25通道]
    tele_lock = threading.Lock()
    t0 = None               # 全局时间基准（两线程共享）

    last_recv_time = [0.0]   # 上一帧的接收时刻，用于去重

    def udp_loop():
        nonlocal t0
        while not stop.is_set():
            item = rx.get(max_age=0.5)
            if item is None:
                time.sleep(0.002)
                continue
            tele, age = item
            now = time.time()
            # 去重：UdpTelemetryReceiver.get() 返回缓存的最新帧，
            # 同一帧会被反复读到。用接收器内部的 recv_time 判断是否新帧。
            recv_stamp = now - age
            if recv_stamp <= last_recv_time[0]:
                time.sleep(0.002)
                continue
            last_recv_time[0] = recv_stamp
            with tele_lock:
                if t0 is None:
                    t0 = recv_stamp
                t = recv_stamp - t0
                row = [tele.get(k, 0.0) for k in TELEMETRY_CHANNELS]
                tele_rows.append([t] + row)

    # ---- 视频采集 ----
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print('无法打开视频设备 %s' % args.video)
        rx.stop()
        return
    if args.width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    print('视频: %dx%d @ %.1ffps' % (w, h, fps))

    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or '.', exist_ok=True)
    video_path = args.out + '_video.mp4'
    writer = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))

    out_csv = args.out + '_telemetry.csv'
    out_meta = args.out + '_meta.json'

    print('[rec] 开始录制 %ds → %s*' % (args.duration, args.out))
    print('[rec] 请开始开车，录制中...', flush=True)

    stop = threading.Event()
    th = threading.Thread(target=udp_loop, daemon=True)
    th.start()

    video_t0 = None
    frames = 0
    start = time.time()
    try:
        while time.time() - start < args.duration:
            ok, frame = cap.read()
            if not ok:
                print('[rec] 视频读取失败')
                break
            now = time.time()
            with tele_lock:
                if t0 is None:
                    t0 = now
                    video_t0 = now
                elif video_t0 is None:
                    video_t0 = now
            writer.write(frame)
            frames += 1
            if args.preview:
                cv2.imshow('record (q to quit)', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            if frames % int(fps) == 0:
                print('  %.0fs / %.0fs  视频帧=%d 遥测帧=%d' %
                      (time.time() - start, args.duration, frames, len(tele_rows)),
                      flush=True)
    except KeyboardInterrupt:
        print('\n[rec] 中断')
    finally:
        stop.set()
        th.join(timeout=1.0)
        rx.stop()
        cap.release()
        writer.release()
        if args.preview:
            cv2.destroyAllWindows()

    # ---- 保存遥测 ----
    with open(out_csv, 'w') as fh:
        fh.write('t,' + ','.join(TELEMETRY_CHANNELS) + '\n')
        for row in tele_rows:
            fh.write(','.join('%.6f' % x for x in row) + '\n')

    meta = {
        'video_file': video_path,
        'telemetry_file': out_csv,
        'video_frames': frames,
        'video_fps': fps,
        'video_size': [w, h],
        'telemetry_frames': len(tele_rows),
        'duration': args.duration,
        't_basis': 'both threads use time.time(); t0 set at first frame/telemetry',
        'note': 't in telemetry.csv is seconds relative to recording start; '
                'video frame k timestamp ≈ k/fps (one-time offset < 1 frame)',
    }
    with open(out_meta, 'w') as fh:
        json.dump(meta, fh, indent=2, ensure_ascii=False)

    print('\n[rec] 完成:')
    print('  视频 %d 帧 → %s' % (frames, video_path))
    print('  遥测 %d 帧 → %s' % (len(tele_rows), out_csv))
    print('  元信息 → %s' % out_meta)

    # 快速统计
    if len(tele_rows) > 10:
        a = np.array(tele_rows)
        print('\n  遥测通道范围（确认抓到真实数据）:')
        for k in ['accel_x', 'accel_z', 'rot_y']:
            j = 1 + TELEMETRY_CHANNELS.index(k)
            print('    %-9s min%+.3f max%+.3f std%.3f' %
                  (k, a[:, j].min(), a[:, j].max(), a[:, j].std()))


def main():
    ap = argparse.ArgumentParser(description='录制对齐的视频+UDP真值数据集')
    ap.add_argument('--list', action='store_true', help='列出摄像头后退出')
    ap.add_argument('--video', default='/dev/video2', help='采集卡设备')
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--port', type=int, default=4124, help='游戏机UDP端口')
    ap.add_argument('--duration', type=float, default=60.0)
    ap.add_argument('--width', type=int, default=0)
    ap.add_argument('--height', type=int, default=0)
    ap.add_argument('--out', default='/tmp/opencode/paired')
    ap.add_argument('--preview', action='store_true')
    args = ap.parse_args()
    if args.list:
        list_cameras()
    else:
        record(args)


if __name__ == '__main__':
    main()
