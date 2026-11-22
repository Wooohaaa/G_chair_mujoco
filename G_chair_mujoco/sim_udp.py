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
import socket
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.join(HERE, '..', 'G_chair_sim')
sys.path.insert(0, SIM_DIR)

from sim import GChairSim, OUTPUT, plot_log

from udp_telemetry import UdpTelemetryReceiver, parse_datagram
from pose_from_motion import PoseFromMotion
from filters import FilterChain
from config import FILTERS
from motion_map import accel_driven_pose, Washout6D

ZERO_TELE = {}   # 空 dict → PoseFromMotion 各通道取 0 → 平台平滑回中位


class AccelLowPass:
    """对加速度/角速度遥测先做一阶低通，压掉 GTA5 碰撞尖峰。

    只滤波遥测输入 (G / rad/s)，不改 Mapper/FilterChain，所以 FlyPT 链路语义不变。
    tau 越大越平滑但延迟越高；0.15s 是之前诊断实测的折中值。
    """
    def __init__(self, tau):
        self.tau = tau
        self.v = {}

    def update(self, tele, dt):
        if self.tau <= 0:
            return tele
        a = dt / (self.tau + dt)
        out = dict(tele)
        for k in ('accel_x', 'accel_z', 'accel_y', 'rot_x', 'rot_y', 'rot_z'):
            x = float(tele.get(k, 0.0))
            prev = self.v.get(k, x)
            v = (1 - a) * prev + a * x
            self.v[k] = v
            out[k] = v
        return out


def pose_from_telemetry(tele, dt, mapper, filt, lp=None, wo=None, accel_model=False):
    """统一入口：按 accel_model 选择原始 FlyPT 映射或加速度驱动映射。"""
    if accel_model:
        if lp is not None:
            tele = lp.update(tele, dt)
        return accel_driven_pose(tele, dt, wo)
    return telemetry_to_pose(tele, dt, mapper, filt, lp)


def telemetry_to_pose(tele, dt, mapper, filt, lp=None):
    """遥测 dict → [x,y,z,roll,pitch,yaw] (m, rad), 复用 G_chair_sim 的映射/滤波。"""
    if lp is not None:
        tele = lp.update(tele, dt)
    pf = mapper.apply_filters(mapper.compute(tele, dt), filt, dt)
    return np.array([
        pf['sway'] / 1000.0, pf['surge'] / 1000.0, pf['heave'] / 1000.0,
        math.radians(pf['roll']), math.radians(pf['pitch']), math.radians(pf['yaw']),
    ])


class TcpTelemetrySource:
    """TCP 客户端：配合 adb forward，板子作 server，后台线程收包。"""

    def __init__(self, host, port):
        self.remote = (host, port)
        self.sock = None
        self.buf = b''
        self.latest = None
        self.recv_time = 0.0
        self.frames = 0
        self.bad = 0
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(0.2)
        self.sock.connect(self.remote)
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print('[tcp] 已连接 %s:%d' % self.remote)

    def _loop(self):
        while self._running:
            try:
                chunk = self.sock.recv(8192)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            self.buf += chunk
            while b'\n' in self.buf:
                line, self.buf = self.buf.split(b'\n', 1)
                tele = parse_datagram(line)
                with self._lock:
                    if tele is None:
                        self.bad += 1
                    else:
                        self.latest = tele
                        self.recv_time = time.time()
                        self.frames += 1

    def get(self, max_age=None):
        with self._lock:
            if self.latest is None:
                return None
            age = time.time() - self.recv_time
            if max_age is not None and age > max_age:
                return None
            return dict(self.latest), age

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self.sock:
            self.sock.close()

    def stats(self):
        return '收包 %d, 坏包 %d' % (self.frames, self.bad)


def run(args):
    dt_ctrl = 1.0 / args.ctrl_hz
    sim = GChairSim()
    mapper = PoseFromMotion()
    filt = FilterChain(FILTERS)
    lp = AccelLowPass(args.lpf) if args.lpf > 0 else None
    if lp is not None:
        print('[lpf] 遥测低通 tau=%.2fs (抑制尖峰)' % args.lpf)
    wo = Washout6D(args.washout) if args.washout > 0 else None
    if args.accel_model:
        print('[motion] 加速度驱动模型 (roll<-accel_x, 左右对称)')
    else:
        print('[motion] 原始 FlyPT 映射 (PoseFromMotion)')

    if args.tcp:
        rx = TcpTelemetrySource(args.host, args.port)
    else:
        rx = UdpTelemetryReceiver(args.host, args.port)
    rx.start()
    print('[%s] 等待首帧遥测...' % ('tcp' if args.tcp else 'udp'))

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
        print('[%s] 收到首帧! 开始驱动 MuJoCo (Ctrl+C 退出)'
              % ('tcp' if args.tcp else 'udp'))

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
                cmd = pose_from_telemetry(ZERO_TELE, dt_ctrl, mapper, filt,
                                          lp, wo, args.accel_model)
                lost += 1
            else:
                tele, _age = item
                cmd = pose_from_telemetry(tele, dt_ctrl, mapper, filt,
                                          lp, wo, args.accel_model)

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
    ap.add_argument('--host', default='0.0.0.0', help='UDP 监听地址 / TCP 连接目标')
    ap.add_argument('--port', type=int, default=4124, help='端口 (与 DCS Lua 一致)')
    ap.add_argument('--tcp', action='store_true', help='TCP 客户端模式 (配合 adb forward)')
    ap.add_argument('--lpf', type=float, default=0.15,
                    help='遥测加速度低通时间常数秒 (0=关闭, 默认 0.15 抑制碰撞尖峰)')
    ap.add_argument('--washout', type=float, default=2.5,
                    help='洗出高通时间常数秒 (0=关闭, 默认 2.5; 恒定加速度后回中位)')
    ap.add_argument('--accel-model', action='store_true', default=True,
                    help='加速度驱动映射 (roll<-accel_x, 左右对称; 默认开启)')
    ap.add_argument('--no-accel-model', dest='accel_model', action='store_false',
                    help='用原始 FlyPT PoseFromMotion 映射(可能左右不对称)')
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
