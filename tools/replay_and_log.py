#!/usr/bin/env python3
"""replay_and_log.py — 把原始25通道遥测回放通过映射管线，生成可分析的运行日志。

record/capture_raw.py 产出的是【原始遥测】(25通道, 无时间列)；
analyze_run.py 需要【上位机运行日志】(t + leg0..5 + pose0..5 + speed0..5 [+ cur..])。
本工具把前者喂给与实机同一套 MotionPipeline，写出后者，使"录制→分析"闭环正确。

用法: ./.venv/bin/python tools/replay_and_log.py [raw.csv] [out_log.csv] [帧数]
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'G_chair_sim'))

from config import TELEMETRY_CHANNELS
from motion_pipeline import MotionPipeline

DT = 0.02


def main():
    if '--help' in sys.argv or '-h' in sys.argv:
        print('用法: replay_and_log.py [raw.csv] [out_log.csv] [帧数]')
        return 0
    src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        ROOT, 'G_chair_mujoco', 'output', 'raw_telemetry.csv')
    dst = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
        ROOT, 'output', 'replay_log.csv')
    limit = int(sys.argv[3]) if len(sys.argv) > 3 else 100000
    if not os.path.exists(src):
        print('找不到 %s' % src)
        return 1

    with open(src, encoding='utf-8', errors='ignore') as f:
        header = f.readline().strip().split(',')
    # capture_raw.py 输出25列(无t)，record_paired.py 输出26列(t在首列)
    offset = 1 if header and header[0].strip() == 't' else 0
    data = np.loadtxt(src, delimiter=',', skiprows=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < offset + len(TELEMETRY_CHANNELS):
        print('字段数不足: 需要%d列, 实际%d列' % (offset + len(TELEMETRY_CHANNELS),
                                              data.shape[1]))
        return 1

    mp = MotionPipeline()
    rows = []
    prev = None
    for i, row in enumerate(data[:limit]):
        tele = {k: float(row[offset + j]) for j, k in enumerate(TELEMETRY_CHANNELS)}
        pose, disp = mp.pose_and_disp(tele, DT)
        speed = [0.0] * 6 if prev is None else [abs(float(a) - b) / DT
                                                for a, b in zip(disp, prev)]
        prev = [float(x) for x in disp]
        rows.append([round(i * DT, 3)] + prev + [float(x) for x in pose] + speed +
                    [float('nan')] * 6 + [0.0, 0.0, 0.0])

    os.makedirs(os.path.dirname(os.path.abspath(dst)), exist_ok=True)
    hdr = ','.join(['t'] + ['leg%d' % i for i in range(6)] +
                   ['pose%d' % i for i in range(6)] +
                   ['speed%d' % i for i in range(6)] +
                   ['cur%d' % i for i in range(6)] +
                   ['hwlimit', 'hwlatch', 'hwenabled'])
    np.savetxt(dst, np.array(rows), delimiter=',', header=hdr, comments='')
    print('回放 %d 帧 → %s (%.1fs)' % (len(rows), dst, len(rows) * DT))
    return 0


if __name__ == '__main__':
    sys.exit(main())
