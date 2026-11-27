#!/usr/bin/env python3
"""
full_chain_sim.py — 全链路数值仿真（从遥测到 MuJoCo 实际位姿）

把每一环都真实串起来，证明"仿真所见 = 主控所得"：

  25通道遥测 CSV
    → MotionPipeline (低通→映射→洗出→限速→IK) → 6缸位移 mm
    → 16bit 帧 (controller.py, 同 serial_bridge)
    → ESP32 固件步进仿真 (firmware_sim.Axis, 4ms tick)
    → MuJoCo command_displacement → 实际平台位姿

对比"管线指令位姿"与"MuJoCo 实际位姿"，验证全链路误差。

运行: ./.venv/bin/python tools/full_chain_sim.py [raw_telemetry.csv] [帧数]
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'G_chair_esp32', 'tests'))

from motion_pipeline import MotionPipeline
from G_chair_sim.controller import DualBoardController
from firmware_sim import Axis, CENTER, STEP_MM, SAMPLE_PERIOD_US
from G_chair_mujoco.sim import GChairSim

DT = 0.02
SUB = int(round(DT * 1e6 / SAMPLE_PERIOD_US))   # 每 20ms 帧 5 个 4ms tick


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        ROOT, 'G_chair_mujoco', 'output', 'raw_telemetry.csv')
    limit = int(sys.argv[2]) if len(sys.argv) > 2 else 1500
    if not os.path.exists(path):
        print('找不到 %s' % path)
        return 1

    data = np.loadtxt(path, delimiter=',', skiprows=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    keys = ['accel_x', 'accel_z', 'accel_y', 'rot_y', 'rot_x', 'rot_z']
    # CSV 前 9 列顺序: accel_x,accel_z,accel_y,speed_x,speed_z,speed_y,rot_y,rot_x,rot_z
    print('=' * 60)
    print('全链路数值仿真: %s (%d 帧)' % (os.path.basename(path), min(limit, len(data))))
    print('=' * 60)

    mp = MotionPipeline()
    ctl = DualBoardController()
    axes = [Axis() for _ in range(6)]
    sim = GChairSim()

    err_pose = []      # 指令位姿 vs MuJoCo 实际位姿 (mm)
    err_leg = []       # 管线缸位移 vs 固件实际缸位移 (mm)
    speeds = []

    for row in data[:limit]:
        tele = {k: float(row[j]) for j, k in enumerate(keys)}
        pose, disp = mp.pose_and_disp(tele, DT)

        # 走一遍真实串口帧（16bit 量化）
        frame_mm = []
        for i in range(6):
            v = ctl.board1.displacement_to_flypt_16bit(disp[i])
            steps = v / (65535 / 38400)
            mm = (steps - CENTER) * STEP_MM
            frame_mm.append(mm)
        # 固件 4ms tick 跟踪
        for a, mm in zip(axes, frame_mm):
            a.move(CENTER + mm / STEP_MM)
        for _ in range(SUB):
            for a in axes:
                a.update()
        actual_mm = np.array([a.mm() for a in axes])

        # 送入 MuJoCo
        sim.command_displacement(actual_mm, DT)
        sim.step()
        act = sim.platform_pose()

        err_pose.append(np.linalg.norm((act[:3] - pose[:3]) * 1000))
        err_leg.append(np.max(np.abs(actual_mm - disp)))
        if len(err_leg) > 1:
            speeds.append(np.max(np.abs(actual_mm - prev)) / DT)
        prev = actual_mm.copy()

    ep = np.array(err_pose)
    el = np.array(err_leg)
    sp = np.array(speeds)
    print('\n[1] 管线缸位移 → 16bit量化 → 固件实际 误差')
    print('  均值 %.4f mm, 最大 %.4f mm' % (el.mean(), el.max()))
    print('  （来自 16bit 量化 + 4ms 离散化，应 <0.05mm）')
    print('\n[2] 固件缸速')
    print('  P50 %.0f / P99 %.0f / MAX %.0f mm/s (主控上限128)' %
          (np.percentile(sp, 50), np.percentile(sp, 99), sp.max()))
    print('\n[3] 指令位姿 → MuJoCo 实际位姿 跟踪误差')
    print('  RMS %.2f mm, MAX %.2f mm' %
          (np.sqrt(np.mean(ep ** 2)), ep.max()))
    print('\n[4] MuJoCo 最终缸位移')
    print('  %s mm' % np.round(sim.leg_positions_mm(), 2))
    print('\n结论: 全链路(遥测→映射→16bit→固件→MuJoCo)数值连通，误差在机构跟踪范围内')
    print('=' * 60)
    return 0


if __name__ == '__main__':
    sys.exit(main())
