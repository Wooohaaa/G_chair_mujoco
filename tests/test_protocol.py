#!/usr/bin/env python3
"""test_protocol.py — 协议/换算正确性回归测试（保护中位精度与帧格式）。"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'G_chair_sim'))

from controller import DualBoardController
from config import TELEMETRY_CHANNELS

PASS = FAIL = 0
DIV = 65535 / 38400


def check(name, cond, detail=''):
    global PASS, FAIL
    if cond:
        PASS += 1; print('  ✅ %s %s' % (name, detail))
    else:
        FAIL += 1; print('  ❌ %s %s' % (name, detail))


def main():
    print('=' * 50)
    print('协议/换算回归测试')
    print('=' * 50)
    ctl = DualBoardController()

    print('\n[1] 位移 ↔ 16bit ↔ 步数')
    for mm in (-95, -50, 0, 30, 50, 95):
        v = ctl.board1.displacement_to_flypt_16bit(mm)
        steps = v / DIV
        real = (steps - 19200) * 0.005
        check('%+dmm' % mm, abs(real - mm) < 0.01,
              '16bit=%d 步=%.0f 还原=%+.4fmm' % (v, steps, real))

    print('\n[2] 16bit 往返')
    for v in (341, 32768, 43007, 65194):
        mm = ctl.board1.flypt_16bit_to_displacement(v)
        v2 = ctl.board1.displacement_to_flypt_16bit(mm)
        check('v=%d' % v, abs(v2 - v) <= 1, '往返=%d' % v2)

    print('\n[3] 帧格式')
    f = bytearray([255, 255])
    for d in [10, 20, 30, 40, 50, 60]:
        v = ctl.board1.displacement_to_flypt_16bit(float(d))
        f += bytes([(v >> 8) & 0xFF, v & 0xFF])
    check('帧长14', len(f) == 14, '%d' % len(f))
    check('帧头', f[0] == 255 and f[1] == 255)
    dec = [(f[2+2*k] << 8) | f[3+2*k] for k in range(6)]
    check('大端', dec[0] == ctl.board1.displacement_to_flypt_16bit(10.0))

    print('\n[4] 25通道定义')
    check('通道数25', len(TELEMETRY_CHANNELS) == 25, '%d' % len(TELEMETRY_CHANNELS))
    check('关键通道顺序', TELEMETRY_CHANNELS[:3] == ['accel_x', 'accel_z', 'accel_y']
          and TELEMETRY_CHANNELS[6] == 'rot_y' and TELEMETRY_CHANNELS[24] == 'time')

    print('\n' + '=' * 50)
    print('结果: %d PASS, %d FAIL' % (PASS, FAIL))
    print('=' * 50)
    return 0 if FAIL == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
