#!/usr/bin/env python3
"""
stream_smooth_check.py — 对比"原始/合成跳变遥测"与"平滑的真实赛车遥测"
回答: 真实赛车数据来了, MuJoCo 链会不会正确运行?
"""
import math
import os
import sys

import numpy as np

from sim import GChairSim, OUTPUT

DT_CTRL = 0.02


def run_stream(smooth=False, duration=30.0):
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'G_chair_sim'))
    from pipeline import SimulationPipeline

    pipe = SimulationPipeline()
    sim = GChairSim()
    # 一阶低通状态, 模拟真实遥测经过车载滤波/平滑后的连续信号
    lp = {}
    tau = 0.15  # 150ms 平滑时间常数
    a = DT_CTRL / (tau + DT_CTRL)

    cmd_log, act_log, t_log = [], [], []
    n = int(duration / DT_CTRL)
    for k in range(n):
        t = k * DT_CTRL
        tele = pipe.telemetry_gen.generate()
        if smooth:
            for key in ('accel_x', 'accel_y', 'accel_z', 'rot_x', 'rot_y', 'rot_z'):
                lp[key] = lp.get(key, tele[key]) * (1 - a) + tele[key] * a
                tele[key] = lp[key]
        pf = pipe.pose_mapper.apply_filters(
            pipe.pose_mapper.compute(tele, pipe.dt), pipe.filter_chain, pipe.dt)
        cmd = np.array([pf['sway']/1000, pf['surge']/1000, pf['heave']/1000,
                        math.radians(pf['roll']), math.radians(pf['pitch']),
                        math.radians(pf['yaw'])])
        sim.command_pose(cmd, DT_CTRL)
        sim.step()
        cmd_log.append(cmd); act_log.append(sim.platform_pose()); t_log.append(t)
    cmd, act, t = map(np.array, (cmd_log, act_log, t_log))
    pos_err = np.linalg.norm((act[:, :3]-cmd[:, :3])*1000, axis=1)
    rot_err = np.degrees(np.linalg.norm(act[:, 3:]-cmd[:, 3:], axis=1))
    return t, cmd, act, pos_err, rot_err


def main():
    print('=' * 60)
    print('真实赛车遥测对接测试  (30s, 50Hz, 含起步/匀速/急刹/转弯/颠簸)')
    print('=' * 60)
    for smooth in (False, True):
        t, cmd, act, pe, re = run_stream(smooth=smooth, duration=30.0)
        tag = '平滑(真实车载滤波后)' if smooth else '原始合成(含场景跳变)'
        print('\n[%s]' % tag)
        print('  位置误差  RMS %.2f mm / MAX %.2f mm' % (np.sqrt(np.mean(pe**2)), pe.max()))
        print('  姿态误差  RMS %.2f deg / MAX %.2f deg' % (np.sqrt(np.mean(re**2)), re.max()))
        i = pe.argmax()
        print('  最大误差发生在 t=%.2fs (场景边界附近=%s)'
              % (t[i], '是' if any(abs(t[i]-b) < 0.6 for b in (12, 24, 32)) else '否'))
        # 该时刻前后一步指令跳变
        if i > 0:
            jump = np.max(np.abs((cmd[i]-cmd[i-1]) * np.array([1000,1000,1000,57.3,57.3,57.3])))
            print('  该步指令跳变 %.1f mm/deg' % jump)


if __name__ == '__main__':
    main()
