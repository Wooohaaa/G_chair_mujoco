#!/usr/bin/env python3
"""
sim_udp.py — 用另一台电脑发来的 UDP 遥测驱动本机 MuJoCo 仿真。

数据链:
  游戏PC --UDP(25个空格分隔浮点数, 端口默认4124)--> 本机
      --> 姿态映射(PoseFromMotion) --> 滤波(FilterChain) --> IK --> MuJoCo

用法:
  ./.venv/bin/python sim_udp.py --realtime --plot           # 收数据并实时跑
  ./.venv/bin/python sim_udp.py --viewer                    # 打开 MuJoCo 查看器
  ./.venv/bin/python sim_udp.py --host 0.0.0.0 --port 4124  # 指定监听

无数据时自动回中位(安全), 收到数据后自动跟随。
"""

import argparse
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.join(HERE, '..', 'G_chair_sim')
sys.path.insert(0, SIM_DIR)

from sim import GChairSim, OUTPUT, plot_log

from udp_telemetry import UdpTelemetryReceiver
from pose_from_motion import PoseFromMotion
from filters import FilterChain
from config import FILTERS

ZERO_TELE = {}   # 空 dict → PoseFromMotion 各通道取 0 → 平台平滑回中位


def telemetry_to_pose(tele, dt, mapper, filt):
    """遥测 dict → [x,y,z,roll,pitch,yaw] (m, rad), 复用 G_chair_sim 的映射/滤波。"""
    pf = mapper.apply_filters(mapper.compute(tele, dt), filt, dt)
    return np.array([
        pf['sway'] / 1000.0, pf['surge'] / 1000.0, pf['heave'] / 1000.0,
        math.radians(pf['roll']), math.radians(pf['pitch']), math.radians(pf['yaw']),
    ])


def run(args):
    dt_ctrl = 1.0 / args.ctrl_hz
    sim = GChairSim()
    mapper = PoseFromMotion()
    filt = FilterChain(FILTERS)

    rx = UdpTelemetryReceiver(args.host, args.port)
    rx.start()
    print('[udp] 监听 %s:%d, 等待首帧遥测...' % (args.host, args.port))

    # 等待第一帧, 最多 args.wait 秒
    t0 = time.time()
    first = None
    while time.time() - t0 < args.wait:
        first = rx.get()
        if first is not None:
            break
        time.sleep(0.01)
    if first is None:
        print('[udp] %.1fs 内未收到数据, 仍将运行(回中位), 可按 Ctrl+C 退出' % args.wait)
    else:
        print('[udp] 收到首帧! 开始驱动 MuJoCo (Ctrl+C 退出)')

    viewer = None
    if args.viewer:
        try:
            from mujoco import viewer as mj_viewer
            viewer = mj_viewer.launch_passive(sim.model, sim.data)
            print('[viewer] 已打开 MuJoCo 查看器')
        except Exception as e:
            print('[viewer] 打开失败: %s' % e)
            viewer = None

    log = {k: [] for k in ('t', 'cmd', 'act', 'leg_cmd', 'leg_act', 'sat')}
    frames = 0
    lost = 0
    wall0 = time.time()
    n = int(args.duration / dt_ctrl) if args.duration > 0 else 10 ** 9

    try:
        k = 0
        while k < n:
            t = k * dt_ctrl
            item = rx.get(max_age=args.stale)
            if item is None:
                cmd = telemetry_to_pose(ZERO_TELE, dt_ctrl, mapper, filt)
                lost += 1
            else:
                tele, _age = item
                cmd = telemetry_to_pose(tele, dt_ctrl, mapper, filt)

            disp, lim = sim.command_pose(cmd, dt_ctrl)
            sim.step()
            act = sim.platform_pose()
            log['t'].append(t)
            log['cmd'].append(cmd)
            log['act'].append(act)
            log['leg_cmd'].append(lim)
            log['leg_act'].append(sim.leg_positions_mm())
            log['sat'].append(float(np.max(np.abs(disp)) > sim.p.range * 1000.0))
            frames += 1
            k += 1

            if viewer is not None:
                if not viewer.is_running():
                    break
                viewer.sync()
                lag = t - (time.time() - wall0)
                if lag > 0:
                    time.sleep(min(lag, dt_ctrl))
            elif args.realtime:
                lag = (t + dt_ctrl) - (time.time() - wall0)
                if lag > 0:
                    time.sleep(lag)
    except KeyboardInterrupt:
        print('\n[interrupted]')
    finally:
        if viewer is not None:
            viewer.close()
        rx.stop()

    if frames:
        t = np.array(log['t'])
        cmd = np.array(log['cmd']); act = np.array(log['act'])
        leg = np.array(log['leg_act']); sat = np.array(log['sat'])
        pe = np.linalg.norm((act[:, :3] - cmd[:, :3]) * 1000, axis=1)
        re = np.degrees(np.linalg.norm(act[:, 3:] - cmd[:, 3:], axis=1))
        print('\n================ UDP 仿真结果 ================')
        print(' %s' % rx.stats())
        print(' 帧数 %d, 无数据帧 %d, 时长 %.1fs' % (frames, lost, t[-1]))
        print(' 位置跟踪 RMS/MAX : %.2f / %.2f mm' % (np.sqrt(np.mean(pe**2)), pe.max()))
        print(' 姿态跟踪 RMS/MAX : %.2f / %.2f deg' % (np.sqrt(np.mean(re**2)), re.max()))
        print(' 实际缸位移 MAX    : %.1f mm (行程 ±%.0f mm)' % (np.max(np.abs(leg)), sim.p.range*1000))
        print(' 超程帧数          : %d' % int(sat.sum()))
        print('=============================================\n')

        if args.save_csv or args.plot:
            os.makedirs(OUTPUT, exist_ok=True)
        if args.save_csv:
            path = os.path.join(OUTPUT, 'udp_log.csv')
            arr = np.c_[t, cmd, act, np.array(log['leg_cmd']), leg]
            header = ('t,' + ','.join('cmd_' + s for s in 'x y z roll pitch yaw'.split()) +
                      ',' + ','.join('act_' + s for s in 'x y z roll pitch yaw'.split()) +
                      ',' + ','.join('legcmd_%d' % i for i in range(6)) +
                      ',' + ','.join('legact_%d' % i for i in range(6)))
            np.savetxt(path, arr, delimiter=',', header=header, comments='')
            print('[csv] %s' % path)
        if args.plot:
            track = {'pos_rms_mm': np.sqrt(np.mean(pe**2)), 'rot_rms_deg': np.sqrt(np.mean(re**2))}
            plot_log(t, cmd, act, np.array(log['leg_cmd']), leg, track)


def main():
    ap = argparse.ArgumentParser(description='UDP 遥测驱动 MuJoCo 仿真')
    ap.add_argument('--host', default='0.0.0.0', help='监听地址 (0.0.0.0=允许任意网卡)')
    ap.add_argument('--port', type=int, default=4124, help='监听 UDP 端口 (与 DCS Lua 一致)')
    ap.add_argument('--wait', type=float, default=10.0, help='等待首帧的秒数')
    ap.add_argument('--stale', type=float, default=0.5, help='超过该秒数视为断流(回中位)')
    ap.add_argument('--ctrl-hz', type=float, default=50.0)
    ap.add_argument('--duration', type=float, default=0.0, help='0=一直运行直到 Ctrl+C')
    ap.add_argument('--realtime', action='store_true', help='按真实时间播放')
    ap.add_argument('--viewer', action='store_true', help='打开 MuJoCo 交互查看器')
    ap.add_argument('--plot', action='store_true')
    ap.add_argument('--save-csv', action='store_true')
    args = ap.parse_args()
    run(args)


if __name__ == '__main__':
    main()
