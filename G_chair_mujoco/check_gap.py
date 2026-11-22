#!/usr/bin/env python3
"""量测 6 条缸顶端 tip_i 与平台铰点 plat_i 的实际间隙（connect 约束违反量）。"""
import os
import sys
import numpy as np
import mujoco
from sim import GChairSim, OUTPUT

HERE = os.path.dirname(os.path.abspath(__file__))


def gap_mm(sim):
    m, d = sim.model, sim.data
    tip = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, 'tip_%d' % i) for i in range(6)]
    plat = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, 'plat_%d' % i) for i in range(6)]
    g = np.array([np.linalg.norm(d.site_xpos[tip[i]] - d.site_xpos[plat[i]])
                  for i in range(6)]) * 1000.0
    return g


def run(csv_path=None):
    sim = GChairSim()
    m = sim.model
    # connect 约束参数
    eq = m.eq_data[0]
    print('connect solref=%s solimp=%s' % (m.eq_solref[0], m.eq_solimp[0]))

    # 6 个执行器的力上限
    print('actuator forcerange:', m.actuator_forcerange[0])
    print('actuator kp/kv/ctrlrange:', m.actuator_gainprm[0], m.actuator_biasprm[0])

    gaps = []
    legmax = []
    if csv_path and os.path.exists(csv_path):
        data = np.loadtxt(csv_path, delimiter=',', skiprows=1)
        t = data[:, 0]
        cmd = data[:, 1:7]
        print('用 %s 驱动, %d 帧, t=%.1f~%.1f' % (csv_path, len(t), t[0], t[-1]))
    else:
        # 无 CSV: 跑 demo 场景
        from sim import make_scenario
        sc = make_scenario('demo', 24)
        n = int(24 / 0.02)
        cmd = np.array([sc(k * 0.02) for k in range(n)])
        print('用 demo 场景驱动, %d 帧' % n)

    for pose in cmd:
        sim.command_pose(pose, 0.02)
        sim.step()
        gaps.append(gap_mm(sim))
        legmax.append(np.max(np.abs(sim.leg_positions_mm())))

    gaps = np.array(gaps)
    print('\n=== 缸顶-平台铰点 间隙 (mm) ===')
    print('  整体 MAX %.3f  平均 %.3f  P99 %.3f' %
          (gaps.max(), gaps.mean(), np.percentile(gaps, 99)))
    for i in range(6):
        print('  腿%d: MAX %.3f  平均 %.3f' % (i, gaps[:, i].max(), gaps[:, i].mean()))
    print('  最大缸位移 %.1f mm' % max(legmax))
    bad = gaps > 1.0
    print('  >1mm 的帧数: %d / %d' % (bad.any(axis=1).sum(), len(gaps)))
    i = gaps.max()
    idx = np.unravel_index(gaps.argmax(), gaps.shape)
    print('  最坏: 帧%d 腿%d -> %.3f mm' % (idx[0], idx[1], i))


if __name__ == '__main__':
    csv = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(__file__), '..', 'G_chair_vision', 'compare_log.csv')
    run(csv)
