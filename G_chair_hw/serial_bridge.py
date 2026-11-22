#!/usr/bin/env python3
"""
serial_bridge.py — 把 UDP 遥测转成两块 Mega2560 能吃的串口帧。

数据链:
  游戏PC --UDP 25浮点--> 仿真PC --本脚本--> COM口1(1号板) + COM口2(2号板)

帧格式与 servoController.ino / FlyPT Mover 完全一致:
  <255><255> + 4 个 16-bit 大端值  (115200, 8N1)

复用 G_chair_sim 的 遥测→姿态→滤波→IK→行程映射, 所以数值与 FlyPT 链路一致。

用法:
  # 先不开串口, 只看要发什么
  ./.venv/bin/python serial_bridge.py --dry-run --port 4124

  # 真机 (Linux 下串口名如 /dev/ttyUSB0; Windows 下 COM3)
  ./.venv/bin/python serial_bridge.py --port1 /dev/ttyUSB0 --port2 /dev/ttyUSB1

  # 将来换成一块 ESP32 (单帧 6 轴)
  ./.venv/bin/python serial_bridge.py --single-board --port1 /dev/ttyUSB0

需要 pyserial: ./.venv/bin/pip install pyserial
"""

import argparse
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_ROOT = os.path.join(HERE, '..')
sys.path.insert(0, SIM_ROOT)
sys.path.insert(0, os.path.join(SIM_ROOT, 'G_chair_sim'))

from config import TELEMETRY_CHANNELS, SIM
from udp_telemetry import UdpTelemetryReceiver
from motion_pipeline import MotionPipeline

START = bytes([255, 255])
DT = SIM['dt']            # 0.02s = 50Hz, 与原链路一致


def zero_telemetry():
    """断流时的安全帧: 全 0 → 平台回中位。"""
    d = dict.fromkeys(TELEMETRY_CHANNELS, 0.0)
    d['_scenario'] = '断流回中'
    d['_scenario_idx'] = -1
    return d


def build_frames_mover(disp_mm):
    """
    按 FlyPT Mover 实际配置生成两帧:
      1号板: Axis1a Axis2a Axis3a Axis4a
      2号板: Axis5a Axis6a Axis4a Axis3a
    disp_mm 为 6 轴相对中位位移 (mm), 范围 ±95。
    """
    from controller import DualBoardController
    ctl = DualBoardController()
    f1 = bytearray(START)
    f2 = bytearray(START)
    order1 = [0, 1, 2, 3]
    order2 = [4, 5, 3, 2]
    for i in order1:
        v = ctl.board1.displacement_to_flypt_16bit(disp_mm[i])
        f1 += bytes([(v >> 8) & 0xFF, v & 0xFF])
    for i in order2:
        v = ctl.board1.displacement_to_flypt_16bit(disp_mm[i])
        f2 += bytes([(v >> 8) & 0xFF, v & 0xFF])
    return bytes(f1), bytes(f2)


def build_frame_single(disp_mm):
    """单板(如 ESP32-S3) 用: 一帧带 6 轴。协议需与板载固件约定。"""
    from controller import DualBoardController
    ctl = DualBoardController()
    f = bytearray(START)
    for i in range(6):
        v = ctl.board1.displacement_to_flypt_16bit(disp_mm[i])
        f += bytes([(v >> 8) & 0xFF, v & 0xFF])
    return bytes(f)


def open_serial(path, baud):
    import serial
    return serial.Serial(path, baudrate=baud, bytesize=8, parity='N',
                         stopbits=1, timeout=0)


def run(args):
    rx = UdpTelemetryReceiver(args.host, args.port)
    rx.start()
    print('[udp] 监听 %s:%d' % (args.host, args.port))

    # 仿真/实机共用的映射管线（默认加速度驱动映射，--flypt 切回原链路）
    motion = MotionPipeline(accel_model=not args.flypt,
                            lpf_tau=args.lpf, washout_tau=args.washout,
                            max_speed=args.max_speed)
    print('[motion] %s, lpf=%.2f, washout=%.2f' %
          ('加速度驱动(对称)' if not args.flypt else '原FlyPT映射',
           args.lpf, args.washout))

    ser1 = ser2 = None
    if not args.dry_run:
        ser1 = open_serial(args.port1, args.baud)
        if not args.single_board:
            ser2 = open_serial(args.port2, args.baud)
        print('[serial] 已打开 %s%s' % (args.port1, ('' if args.single_board else ' + ' + args.port2)))

    print('[bridge] 开始转发 (Ctrl+C 停止, --dry-run 不实际写串口)')
    t0 = time.time()
    frames = 0
    lost = 0
    try:
        while (args.duration <= 0) or (time.time() - t0 < args.duration):
            item = rx.get(max_age=args.stale)
            if item is None:
                lost += 1
                disp = motion.displacements_mm(zero_telemetry(), DT)  # 回中位
            else:
                disp = motion.displacements_mm(item[0], DT)
            disp = np.asarray(disp, dtype=float)
            disp = np.clip(disp, -95.0, 95.0)

            if args.single_board:
                frame = build_frame_single(disp)
                if ser1:
                    ser1.write(frame)
            else:
                f1, f2 = build_frames_mover(disp)
                if ser1:
                    ser1.write(f1)
                if ser2:
                    ser2.write(f2)

            frames += 1
            if args.verbose and frames % 50 == 0:
                print(' t=%.1fs 帧=%d 断流=%d  disp=%s' %
                      (time.time() - t0, frames, lost, np.round(disp, 1)), flush=True)
            time.sleep(max(0, DT - ((time.time() - t0) % DT)))
    except KeyboardInterrupt:
        print('\n[bridge] 停止')
    finally:
        rx.stop()
        for s in (ser1, ser2):
            if s:
                s.close()


def main():
    ap = argparse.ArgumentParser(description='UDP 遥测 → Mega2560 串口帧桥接')
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--port', type=int, default=4124, help='UDP 监听端口')
    ap.add_argument('--stale', type=float, default=0.5, help='断流判定秒数')
    ap.add_argument('--port1', default='/dev/ttyUSB0', help='1号板串口')
    ap.add_argument('--port2', default='/dev/ttyUSB1', help='2号板串口')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--single-board', action='store_true', help='单板模式: 一帧 6 轴')
    ap.add_argument('--dry-run', action='store_true', help='不写串口, 只验证链路')
    ap.add_argument('--verbose', action='store_true')
    ap.add_argument('--duration', type=float, default=0.0, help='运行秒数, 0=一直运行')
    ap.add_argument('--flypt', action='store_true',
                    help='用原始 FlyPT 映射(默认用修好的加速度驱动映射)')
    ap.add_argument('--lpf', type=float, default=0.15, help='遥测低通 tau 秒 (0=关)')
    ap.add_argument('--washout', type=float, default=2.5, help='洗出高通 tau 秒 (0=关)')
    ap.add_argument('--max-speed', type=float, default=110.0,
                    help='最大缸速 mm/s (主控能力128, 默认110; 0=不限速)')
    args = ap.parse_args()
    run(args)


if __name__ == '__main__':
    main()
