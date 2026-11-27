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

    print('\n[5] 状态帧解析（新旧固件兼容）')
    from host_app import parse_status_frame
    base = '$' + ','.join(str(i) for i in range(12)) + ',5,1,0'
    for name, tail, exp_latch, exp_estop in [
        ('旧15字段', '', 0, 0),
        ('旧急停16字段', ',1', 0, 1),
        ('新16字段锁存', ',L3', 3, 0),
        ('新急停17字段', ',1,L12', 12, 1),
    ]:
        hw = {'limit': 0, 'enabled': 0, 'watchdog': 0, 'latch': 0, 'frames': 0,
              'cur': [], 'tgt': [], 'estop': 0}
        ok = parse_status_frame(base + tail, hw)
        check(name, ok and hw['limit'] == 5 and hw['enabled'] == 1
              and hw['latch'] == exp_latch and hw['estop'] == exp_estop
              and hw['cur'] == list(range(6))
              and hw['tgt'] == list(range(6, 12)),
              'limit=%d enabled=%d latch=%d estop=%d cur0=%d tgt0=%d'
              % (hw['limit'], hw['enabled'], hw['latch'], hw['estop'],
                 hw['cur'][0], hw['tgt'][0]))
    check('非$帧忽略', not parse_status_frame('garbage', {'latch': 0}))

    print('\n[6] 录制日志分析（新增实际位置列）')
    import subprocess
    import tempfile
    import numpy as np
    t = np.arange(0, 5, 0.02)
    legs = np.column_stack([10 * np.sin(t)] * 6)
    cur = legs + np.column_stack([0.5 * np.sin(3 * t + i) for i in range(6)])
    data = np.column_stack([t, legs, np.zeros((len(t), 6)),
                            np.zeros((len(t), 6)), cur,
                            np.zeros((len(t), 3))])
    hdr = ','.join(['t'] + ['leg%d' % i for i in range(6)] +
                   ['pose%d' % i for i in range(6)] +
                   ['speed%d' % i for i in range(6)] +
                   ['cur%d' % i for i in range(6)] +
                   ['hwlimit', 'hwlatch', 'hwenabled'])
    with tempfile.NamedTemporaryFile('w', suffix='.csv', delete=False) as f:
        f.write(hdr + '\n')
        np.savetxt(f, data, delimiter=',')
        tmp = f.name
    try:
        r = subprocess.run([sys.executable, os.path.join(ROOT, 'tools', 'analyze_run.py'), tmp],
                           capture_output=True, text=True)
        check('实际位置跟踪分析', r.returncode == 0
              and '缸跟踪 RMS' in r.stdout and '0.50' in r.stdout,
              [l.strip() for l in r.stdout.splitlines() if '最大误差' in l][:1])
    finally:
        os.remove(tmp)
    # --compare 双文件对比
    with tempfile.NamedTemporaryFile('w', suffix='.csv', delete=False) as f:
        f.write(hdr + '\n')
        np.savetxt(f, data, delimiter=',')
        tmp = f.name
    try:
        r = subprocess.run([sys.executable, os.path.join(ROOT, 'tools', 'analyze_run.py'),
                            '--compare', tmp, tmp], capture_output=True, text=True)
        check('--compare 对比', r.returncode == 0 and '变化(第2份相对第1份)' in r.stdout)
    finally:
        os.remove(tmp)
    # 旧格式 19 列应优雅跳过
    with tempfile.NamedTemporaryFile('w', suffix='.csv', delete=False) as f:
        f.write(hdr.split(',hwlimit')[0] + '\n')
        np.savetxt(f, data[:, :19], delimiter=',')
        tmp = f.name
    try:
        r = subprocess.run([sys.executable, os.path.join(ROOT, 'tools', 'analyze_run.py'), tmp],
                           capture_output=True, text=True)
        check('旧格式兼容', r.returncode == 0 and '旧格式日志' in r.stdout)
    finally:
        os.remove(tmp)

    print('\n' + '=' * 50)
    print('结果: %d PASS, %d FAIL' % (PASS, FAIL))
    print('=' * 50)
    return 0 if FAIL == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
