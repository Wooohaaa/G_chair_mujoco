#!/usr/bin/env python3
"""
udp_sender_test.py — UDP 遥测发送端(测试/模板, 纯标准库, 可拷到游戏PC运行)。

把模拟赛车数据按 25 个空格分隔浮点数的格式, 以 50Hz 发到仿真PC。
格式与工程自带的 DCS_To_FlyPT_Mover.lua 完全相同。

用法:
  python udp_sender_test.py --ip 192.168.1.50 --port 4124 --duration 30
先在仿真PC运行 sim_udp.py, 再运行本脚本。
"""

import argparse
import math
import socket
import time

# 与 config.TELEMETRY_CHANNELS 顺序一致
CHANNELS = [
    'accel_x', 'accel_z', 'accel_y', 'speed_x', 'speed_z', 'speed_y',
    'rot_y', 'rot_x', 'rot_z', 'heading', 'bank', 'pitch', 'airspeed',
    'gear_front', 'gear_turn', 'gear_left', 'gear_right', 'altitude',
    'flaps', 'gear', 'speedbrakes', 'canopy', 'stall', 'aoa', 'time',
]


def make_frame(t):
    """按场景生成一帧模拟赛车遥测 (起步/匀速/急刹/右转/左转/颠簸)。"""
    ax = ay = az = rx = ry = rz = 0.0
    speed = 0.0
    if t < 12:                       # 直线起步
        az = min(t / 4.0, 1.0) * 1.2
        speed = min(t * 2.5, 20.0)
    elif t < 24:                     # 匀速 + 轻微颠簸
        speed = 20.0
        ay = 0.05 * math.sin(3.0 * t)
        ax = 0.03 * math.sin(2.0 * t)
    elif t < 32:                     # 急刹
        az = -1.5 * min((t - 24) / 2.0, 1.0)
        rz = 0.1 * math.sin(2.0 * (t - 24))
        speed = max(20.0 - 1.5 * (t - 24) * 10, 0.0)
    elif t < 44:                     # 右转
        s = 0.8 * min((t - 32) / 2.0, 1.0)
        speed = 12.0; ax = s * 1.2; rx = -s * 0.15; ry = s * 0.2; az = -0.2 * s
    elif t < 56:                     # 左转
        s = -0.8 * min((t - 44) / 2.0, 1.0)
        speed = 12.0; ax = s * 1.2; rx = -s * 0.15; ry = s * 0.2; az = -0.2 * abs(s)
    else:                            # 颠簸
        speed = 15.0
        ay = 0.3 * math.sin(8.0 * (t - 56)) * min(t - 56, 1.0)
        ax = 0.1 * math.sin(6.0 * (t - 56))
        rz = 0.05 * math.sin(7.0 * (t - 56))

    d = dict.fromkeys(CHANNELS, 0.0)
    d.update(accel_x=ax, accel_z=az, accel_y=ay,
             speed_x=speed * math.sin(ry * t), speed_z=speed, speed_y=0.0,
             rot_y=ry, rot_x=rx, rot_z=rz,
             airspeed=speed, gear_front=1.0, gear_left=1.0, gear_right=1.0,
             gear=1.0, time=t, aoa=az * 0.1)
    return d


def main():
    ap = argparse.ArgumentParser(description='UDP 遥测发送端 (25 浮点数)')
    ap.add_argument('--ip', default='127.0.0.1', help='仿真PC 的 IP')
    ap.add_argument('--port', type=int, default=4124)
    ap.add_argument('--duration', type=float, default=30.0)
    ap.add_argument('--hz', type=float, default=50.0)
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    remote = (args.ip, args.port)
    dt = 1.0 / args.hz
    print('发送 -> udp://%s:%d @ %.0fHz, %ds (Ctrl+C 停止)' %
          (args.ip, args.port, args.hz, args.duration))
    t0 = time.time()
    try:
        while (time.time() - t0) < args.duration:
            t = time.time() - t0
            frame = make_frame(t)
            payload = ' '.join('%.4f' % frame[k] for k in CHANNELS).encode('ascii')
            sock.sendto(payload, remote)
            time.sleep(max(0, dt - ((time.time() - t0) % dt)))
    except KeyboardInterrupt:
        print('\n停止发送')
    sock.close()


if __name__ == '__main__':
    main()
