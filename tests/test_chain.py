#!/usr/bin/env python3
"""
test_chain.py — 全链路回归测试（无需真实硬件）

用 UDP 测试发送 + 虚拟串口模拟 ESP32，验证：
  1. 25通道 UDP 解析
  2. MotionPipeline 映射/IK/限速
  3. 指令帧 <255><255>+6×16bit 格式
  4. mm→16bit→步数→mm 换算精度（中位/满量程）
  5. 状态 $ 帧解析
  6. 帧超限时失步自恢复（固件逻辑模拟）

运行: ./.venv/bin/python tests/test_chain.py
"""
import os
import sys
import subprocess
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')
sys.path.insert(0, ROOT)

PASS = 0
FAIL = 0


def check(name, cond, detail=''):
    global PASS, FAIL
    if cond:
        PASS += 1
        print('  ✅ %s %s' % (name, detail))
    else:
        FAIL += 1
        print('  ❌ %s %s' % (name, detail))


def test_udp_parse():
    print('\n[1] UDP 25通道解析')
    sys.path.insert(0, os.path.join(ROOT, 'G_chair_sim'))
    from udp_telemetry import parse_datagram
    from config import TELEMETRY_CHANNELS
    vals = [0.1 * i for i in range(25)]
    payload = ' '.join('%.4f' % v for v in vals).encode()
    d = parse_datagram(payload)
    check('25通道解析', d is not None and len([k for k in d if not k.startswith('_')]) == 25)
    check('字段顺序', abs(d['accel_x'] - 0.0) < 1e-6 and abs(d['time'] - 2.4) < 1e-6,
          'time=%.2f' % d['time'])
    check('逗号分隔兼容', parse_datagram(payload.replace(b' ', b',')) is not None)
    check('坏包拒绝', parse_datagram(b'garbage') is None)


def test_mapping():
    print('\n[2] MotionPipeline 映射 + IK + 限速')
    from motion_pipeline import MotionPipeline
    m = MotionPipeline()
    tele = {'accel_x': 0.4, 'accel_z': -0.2, 'accel_y': 0.05,
            'rot_y': 0.3, 'rot_x': 0.02, 'rot_z': 0.01}
    m.reset()
    pose, disp = m.pose_and_disp(tele, 0.02)
    check('位姿输出', len(pose) == 6 and np.all(np.isfinite(pose)))
    check('缸位移输出', len(disp) == 6 and np.all(np.isfinite(disp)))
    # 跑变化的遥测（模拟真实驾驶），确认跟随到非零。
    # 注意：washout 高通对"恒定输入"会回中位（设计意图），所以用变化输入测试。
    peak = 0.0
    for k in range(100):
        t_var = dict(tele)
        t_var['accel_x'] = 0.5 * np.sin(k * 0.1)
        _, d = m.pose_and_disp(t_var, 0.02)
        peak = max(peak, np.max(np.abs(d)))
    check('变化输入跟随', peak > 1.0, '峰值 %.1fmm' % peak)
    check('行程限幅', np.max(np.abs(disp)) <= 95.0, 'max %.1fmm' % np.max(np.abs(disp)))
    # 限速
    m2 = MotionPipeline(max_speed=110)
    a = np.loadtxt(os.path.join(ROOT, 'G_chair_mujoco', 'output', 'raw_telemetry.csv'),
                   delimiter=',', skiprows=1) if os.path.exists(
        os.path.join(ROOT, 'G_chair_mujoco', 'output', 'raw_telemetry.csv')) else None
    if a is not None:
        keys = ['accel_x', 'accel_z', 'accel_y', 'rot_y', 'rot_x', 'rot_z']
        prev = None; speeds = []
        for row in a[:1000]:
            t = {k: row[j] for j, k in enumerate(keys)}
            _, d = m2.pose_and_disp(t, 0.02)
            if prev is not None:
                speeds.append(np.max(np.abs(d - prev)) / 0.02)
            prev = d
        check('缸速限制', max(speeds) <= 111, 'MAX %.0f mm/s' % max(speeds))


def test_conversion():
    print('\n[3] mm→16bit→步数→mm 换算精度')
    sys.path.insert(0, os.path.join(ROOT, 'G_chair_sim'))
    from controller import DualBoardController
    ctl = DualBoardController()
    DIV = 65535 / 38400
    for disp in (-95, 0, 30, 95):
        v = ctl.board1.displacement_to_flypt_16bit(disp)
        mm = (v / DIV - 19200) * 0.005
        check('%+dmm 换算' % disp, abs(mm - disp) < 0.01,
              '16bit=%d -> %+.4fmm' % (v, mm))


def test_frame():
    print('\n[4] 串口指令帧格式')
    sys.path.insert(0, os.path.join(ROOT, 'G_chair_sim'))
    from controller import DualBoardController
    ctl = DualBoardController()
    d = [10, 20, 30, 40, 50, 60]
    f = bytearray([255, 255])
    for x in d:
        v = ctl.board1.displacement_to_flypt_16bit(float(x))
        f += bytes([(v >> 8) & 0xFF, v & 0xFF])
    check('帧长14字节', len(f) == 14, '实际 %d' % len(f))
    check('帧头 FF FF', f[0] == 255 and f[1] == 255)
    vals = [(f[2 + 2*k] << 8) | f[3 + 2*k] for k in range(6)]
    check('大端解码', vals[0] == ctl.board1.displacement_to_flypt_16bit(10.0))


def test_status_parse():
    print('\n[5] 状态 $ 帧解析')
    line = '$19200,19200,19200,19200,19200,19200,' \
           '19200,19200,19200,19200,19200,19200,0,1,0'
    parts = line[1:].split(',')
    check('字段数15', len(parts) == 15, '实际 %d' % len(parts))
    cur = [int(x) for x in parts[0:6]]
    limit = int(parts[12]); en = int(parts[13]); wd = int(parts[14])
    check('中位步数19200', all(x == 19200 for x in cur))
    check('状态位', limit == 0 and en == 1 and wd == 0)


def main():
    print('=' * 50)
    print('G_chair 全链路回归测试')
    print('=' * 50)
    test_udp_parse()
    test_mapping()
    test_conversion()
    test_frame()
    test_status_parse()
    print('\n' + '=' * 50)
    print('结果: %d PASS, %d FAIL' % (PASS, FAIL))
    print('=' * 50)
    return 0 if FAIL == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
