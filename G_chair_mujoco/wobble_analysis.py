#!/usr/bin/env python3
"""
wobble_analysis.py — 诊断 MuJoCo 运行时"晃动"的来源, 并跑一段真实赛车数据流仿真。

三个实验:
  A. 中位保持 (cmd=0): 看机构本身的重力下垂/残余抖动
  B. 阶跃响应: 看执行器+软约束的过冲/振铃 (晃动的主因之一)
  C. 数据流 (G_chair_sim pipeline, 可加噪声=真实遥测): 看指令跳变/跟踪/超程

用法:
  ./.venv/bin/python wobble_analysis.py
"""

import math
import os
import sys

import numpy as np

from sim import GChairSim, OUTPUT

DT_CTRL = 0.02          # 50 Hz 控制周期 (与 sim.py 默认一致)
RNG = np.random.default_rng(42)


def settle_stats(sim, seconds=6.0):
    """保持当前指令, 记录平台位姿, 返回稳态偏压与峰峰抖动。"""
    n = int(round(seconds / DT_CTRL))
    rec = []
    for _ in range(n):
        sim.step()
        rec.append(sim.platform_pose())
    rec = np.array(rec)
    # 后 2/3 视为稳态
    tail = rec[int(n * 0.33):]
    bias = tail.mean(axis=0)
    ptp = tail.max(axis=0) - tail.min(axis=0)
    return rec, bias, ptp


def exp_neutral(sim):
    print('\n[A] 中位保持 (命令全 0) — 机构自身行为')
    sim.reset()
    sim.command_pose(np.zeros(6), DT_CTRL)
    rec, bias, ptp = settle_stats(sim, seconds=6.0)
    print('  稳态平台偏压  : x/y/z = %.2f / %.2f / %.2f mm' % tuple(bias[:3] * 1000))
    print('  稳态姿态偏压  : r/p/y = %.3f / %.3f / %.3f deg' % tuple(np.degrees(bias[3:])))
    print('  稳态峰峰抖动  : x/y/z = %.2f / %.2f / %.2f mm' % tuple(ptp[:3] * 1000))
    print('  稳态姿态抖动  : r/p/y = %.3f / %.3f / %.3f deg' % tuple(np.degrees(ptp[3:])))
    print('  稳态缸位移    : %s mm' % np.round(sim.leg_positions_mm(), 2))
    return rec, bias, ptp


def step_response(sim, amp_mm=30.0):
    print('\n[B] 阶跃响应 (heave +%.0f mm, 保持 3s) — 过冲/振铃' % amp_mm)
    sim.reset()
    # 先稳定在中位 2s
    sim.command_pose(np.zeros(6), DT_CTRL)
    for _ in range(int(2.0 / DT_CTRL)):
        sim.step()
    base = sim.platform_pose()[2] * 1000.0

    cmd = np.zeros(6)
    cmd[2] = amp_mm / 1000.0
    sim.command_pose(cmd, DT_CTRL)

    n = int(3.0 / DT_CTRL)
    resp = []
    for _ in range(n):
        sim.step()
        resp.append(sim.platform_pose()[2] * 1000.0 - base)
    resp = np.array(resp)
    t = np.arange(n) * DT_CTRL
    peak = resp.max()
    overshoot = (peak - amp_mm) / amp_mm * 100.0
    # 进入 ±5% 带宽并首次保持在其中的时刻
    band = 0.05 * amp_mm
    settled_idx = n
    for i in range(n):
        if np.all(np.abs(resp[i:] - amp_mm) <= band):
            settled_idx = i
            break
    settle_t = t[settled_idx] if settled_idx < n else float('nan')
    # 统计过冲后的振荡次数 (穿越目标线的次数)
    cross = np.sum(np.diff(np.sign(resp - amp_mm)) != 0)
    print('  目标 %.1f mm, 峰值 %.1f mm, 过冲 %.1f%%' % (amp_mm, peak, overshoot))
    print('  ±5%% 稳定时间: %.2f s, 穿越目标次数: %d (振铃)' % (settle_t, cross))
    return t, resp


def add_noise(telemetry, t):
    """给干净的合成遥测加上真实车载传感器的噪声/抖动。"""
    noisy = dict(telemetry)
    # 加速度通道: 白噪声 + 低频漂移 (模拟 IMU 噪声与路面振动)
    for key in ('accel_x', 'accel_y', 'accel_z'):
        noisy[key] = telemetry[key] + RNG.normal(0, 0.012) + 0.006 * math.sin(2 * math.pi * 2.3 * t)
    # 角速度通道: 陀螺噪声
    for key in ('rot_x', 'rot_y', 'rot_z'):
        noisy[key] = telemetry[key] + RNG.normal(0, 0.004)
    return noisy


def exp_stream(use_noise=True, duration=60.0):
    print('\n[C] 数据流仿真 (%s赛车遥测, 25通道 → 姿态映射 → 滤波 → IK → MuJoCo)'
          % ('带噪声' if use_noise else '干净'))
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'G_chair_sim'))
    from pipeline import SimulationPipeline
    from telemetry import TelemetryGenerator

    pipe = SimulationPipeline()
    sim = GChairSim()

    # 直接复用 pipeline 的遥测生成器, 但额外注入噪声
    gen = pipe.telemetry_gen
    log = {'t': [], 'cmd': [], 'act': [], 'leg_cmd': [], 'leg_act': [], 'scn': []}
    n = int(round(duration / DT_CTRL))
    jump_events = []   # 指令单步跳变 > 阈值
    prev_c = None

    for k in range(n):
        t = k * DT_CTRL
        tele = gen.generate()
        if use_noise:
            tele = add_noise(tele, t)
        # 复刻 pipeline.run_step 的前半段 (遥测→姿态→滤波)
        pose_raw = pipe.pose_mapper.compute(tele, pipe.dt)
        pose_filt = pipe.pose_mapper.apply_filters(pose_raw, pipe.filter_chain, pipe.dt)
        cmd = np.array([
            pose_filt['sway'] / 1000.0, pose_filt['surge'] / 1000.0,
            pose_filt['heave'] / 1000.0,
            math.radians(pose_filt['roll']), math.radians(pose_filt['pitch']),
            math.radians(pose_filt['yaw']),
        ])
        disp, lim = sim.command_pose(cmd, DT_CTRL)   # IK 用当前 MuJoCo 几何
        sim.step()
        act = sim.platform_pose()
        leg_act = sim.leg_positions_mm()

        log['t'].append(t)
        log['cmd'].append(cmd)
        log['act'].append(act)
        log['leg_cmd'].append(lim)
        log['leg_act'].append(leg_act)
        log['scn'].append(tele.get('_scenario', ''))
        if prev_c is not None:
            d = np.max(np.abs(cmd - prev_c) * np.array([1000, 1000, 1000,
                       57.3, 57.3, 57.3]))
            if d > 12.0:   # 单 20ms 步内跳变 >12mm/12deg
                jump_events.append((t, d, tele.get('_scenario', '')))
        prev_c = cmd.copy()

    for key in ('t', 'cmd', 'act', 'leg_cmd', 'leg_act'):
        log[key] = np.array(log[key])

    cmd, act = log['cmd'], log['act']
    pos_err = np.linalg.norm((act[:, :3] - cmd[:, :3]) * 1000, axis=1)
    rot_err = np.degrees(np.linalg.norm(act[:, 3:] - cmd[:, 3:], axis=1))
    print('  帧数 %d, 时长 %.1fs' % (len(log['t']), log['t'][-1]))
    print('  跟踪误差 : 位置 RMS %.2f / MAX %.2f mm ; 姿态 RMS %.2f / MAX %.2f deg'
          % (np.sqrt(np.mean(pos_err**2)), pos_err.max(),
             np.sqrt(np.mean(rot_err**2)), rot_err.max()))
    print('  缸位移 MAX : 指令 %.1f mm / 实际 %.1f mm (行程 ±95mm)'
          % (np.max(np.abs(log['leg_cmd'])), np.max(np.abs(log['leg_act']))))
    sat = np.sum(np.max(np.abs(log['leg_cmd']), axis=1) > 95)
    print('  超程帧数   : %d / %d' % (sat, len(log['t'])))
    print('  单步指令跳变 >12mm/deg 的事件数: %d' % len(jump_events))
    for e in jump_events[:8]:
        print('     t=%.2fs 跳变 %.1f  (%s)' % e)
    if len(jump_events) > 8:
        print('     ... 共 %d 次' % len(jump_events))

    # 逐自由度指令的抖动: 稳态段前后差分统计
    print('  指令信号平滑度 (|Δ|/步 均值, mm或deg):')
    for i, name in enumerate(['sway', 'surge', 'heave', 'roll', 'pitch', 'yaw']):
        scale = 1000 if i < 3 else 57.2958
        d = np.abs(np.diff(cmd[:, i])) * scale
        print('     %-6s  mean %.2f  p99 %.2f  max %.2f' %
              (name, d.mean(), np.percentile(d, 99), d.max()))
    return log, jump_events


def make_plots(rec, bias, ptp, step, log):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('[plot] 跳过 (无 matplotlib)')
        return
    os.makedirs(OUTPUT, exist_ok=True)

    # 图1: 中位保持 + 阶跃
    fig, axs = plt.subplots(1, 2, figsize=(12, 4))
    t0 = np.arange(len(rec)) * DT_CTRL
    axs[0].plot(t0, rec[:, 2] * 1000, lw=1)
    axs[0].axhline(0, color='r', ls=':', lw=0.8)
    axs[0].set_title('A: Neutral hold (cmd=0)\nsteady bias z=%.2fmm, p2p=%.2fmm'
                     % (bias[2] * 1000, ptp[2] * 1000))
    axs[0].set_xlabel('t (s)'); axs[0].set_ylabel('heave (mm)')
    ts, rs = step
    axs[1].plot(ts, rs, lw=1)
    axs[1].axhline(30, color='r', ls=':', lw=0.8)
    axs[1].set_title('B: Step response (heave +30mm)\novershoot %.1f%%, ringing'
                     % ((rs.max() - 30) / 30 * 100))
    axs[1].set_xlabel('t (s)'); axs[1].set_ylabel('heave (mm)')
    fig.tight_layout()
    p = os.path.join(OUTPUT, 'wobble_mechanism.png')
    fig.savefig(p, dpi=110); plt.close(fig)
    print('[plot] %s' % p)

    # 图2: 数据流指令 vs 实际
    fig, axs = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
    t = log['t']
    axs[0].plot(t, log['cmd'][:, 2] * 1000, '--', lw=0.8, label='cmd heave')
    axs[0].plot(t, log['act'][:, 2] * 1000, lw=0.9, label='act heave')
    axs[0].set_ylabel('heave (mm)'); axs[0].legend(fontsize=8)
    axs[0].set_title('C: Streaming telemetry (noisy) — cmd vs MuJoCo actual')
    axs[1].plot(t, np.degrees(log['cmd'][:, 3]), '--', lw=0.8, label='cmd roll')
    axs[1].plot(t, np.degrees(log['act'][:, 3]), lw=0.9, label='act roll')
    axs[1].plot(t, np.degrees(log['cmd'][:, 5]), ':', lw=0.8, label='cmd yaw')
    axs[1].plot(t, np.degrees(log['act'][:, 5]), lw=0.9, label='act yaw')
    axs[1].set_ylabel('angle (deg)'); axs[1].set_xlabel('t (s)')
    axs[1].legend(fontsize=8, loc='upper right')
    for ax in axs:
        for sc, txt in [(12, '匀速'), (24, '急刹'), (32, '右转'), (44, '左转'), (56, '颠簸')]:
            ax.axvline(sc, color='grey', ls=':', lw=0.5)
    fig.tight_layout()
    p = os.path.join(OUTPUT, 'wobble_stream.png')
    fig.savefig(p, dpi=110); plt.close(fig)
    print('[plot] %s' % p)


def main():
    sim = GChairSim()
    rec, bias, ptp = exp_neutral(sim)
    step = step_response(sim)
    log, jumps = exp_stream(use_noise=True, duration=60.0)
    make_plots(rec, bias, ptp, step, log)


if __name__ == '__main__':
    main()
