#!/usr/bin/env python3
"""
sim.py — G_chair 六轴 Stewart 平台 MuJoCo 仿真主程序

功能:
  * 由目标位姿 (平移 m + 姿态 rad) 经逆运动学得到 6 缸位移, 用 position 执行器驱动
  * 内置多种演示运动, 也可接入原工程 flypt 数据链 (--pipeline)
  * MuJoCo 交互查看 (--viewer) / 离屏渲染动画 (--render out.mp4)
  * 静态位姿跟踪校验与工作空间/行程检查 (--validate)

示例:
  python sim.py --viewer                         # 打开交互查看器, 播放演示动作
  python sim.py --render out.mp4 --duration 20   # 离屏渲染 20s 动画
  python sim.py --scenario roll --viewer         # 单独测试横滚
  python sim.py --pipeline --viewer              # 用 G_chair_sim 遥测→姿态链驱动
  python sim.py --validate                       # 输出校验报告
"""

import argparse
import math
import os
import sys
import time

import numpy as np
import mujoco

from params import PARAMS
from gchair_kin import make_rig

HERE = os.path.dirname(os.path.abspath(__file__))
OUTPUT = os.path.join(HERE, 'output')


def quat_to_euler(q):
    w, x, y, z = q
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    yaw = math.atan2(R[1, 0], R[0, 0])
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    return roll, pitch, yaw


class GChairSim:
    def __init__(self, params=PARAMS, xml=None):
        self.p = params
        self.rig = make_rig(params)
        self.xml = xml or os.path.join(HERE, 'model.xml')
        if not os.path.exists(self.xml):
            import build_model
            with open(self.xml, 'w', encoding='utf-8') as fh:
                fh.write(build_model.build_xml(params))
        self.model = mujoco.MjModel.from_xml_path(self.xml)
        self.data = mujoco.MjData(self.model)
        self.steps_per_ctrl = 10
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def command_pose(self, pose, dt_ctrl=0.02):
        """pose = [x, y, z, roll, pitch, yaw]  (m, rad)。返回 (原始位移mm, 限幅后mm)。"""
        x, y, z, roll, pitch, yaw = pose
        disp = self.rig.inverse_kinematics(x * 1000.0, y * 1000.0, z * 1000.0,
                                           roll, pitch, yaw)
        lim = np.clip(disp, -self.p.range * 1000.0, self.p.range * 1000.0)
        self.data.ctrl[:] = lim / 1000.0
        self.steps_per_ctrl = max(1, int(round(dt_ctrl / self.model.opt.timestep)))
        return disp, lim

    def command_displacement(self, disp_mm, dt_ctrl=0.02):
        """直接设置 6 缸位移(mm)，跳过内部 IK。

        用于"仿真=实机"统一：缸位移由共用的 MotionPipeline 用同一套实物几何算好，
        仿真和实机拿到的是同一组数字。这里只负责把位移下发到 MuJoCo 执行器。
        """
        lim = np.clip(np.asarray(disp_mm, dtype=float),
                      -self.p.range * 1000.0, self.p.range * 1000.0)
        self.data.ctrl[:] = lim / 1000.0
        self.steps_per_ctrl = max(1, int(round(dt_ctrl / self.model.opt.timestep)))
        return lim

    def step(self):
        for _ in range(self.steps_per_ctrl):
            mujoco.mj_step(self.model, self.data)

    def advance(self, seconds):
        n = int(round(seconds / self.model.opt.timestep))
        for _ in range(n):
            mujoco.mj_step(self.model, self.data)

    def platform_pose(self):
        b = self.data.body('platform')
        roll, pitch, yaw = quat_to_euler(b.xquat)
        return np.array([b.xpos[0], b.xpos[1], b.xpos[2] - self.p.neutral_height(),
                         roll, pitch, yaw])

    def platform_pos_raw(self):
        return self.data.body('platform').xpos.copy()

    def leg_positions_mm(self):
        return np.array([float(self.data.joint('leg_%d' % i).qpos[0])
                         for i in range(6)]) * 1000.0


# ============================ 运动场景 ============================

def make_scenario(name, duration):
    """返回 f(t) -> [x,y,z,roll,pitch,yaw] (m, rad)。"""
    w = 2.0 * math.pi

    def ramp(t, T=1.5):
        return min(1.0, t / T) if T > 0 else 1.0

    def neutral(t):
        return np.zeros(6)

    def heave(t):
        return np.array([0, 0, 0.028 * math.sin(w * 0.25 * t), 0, 0, 0])

    def roll(t):
        return np.array([0, 0, 0, math.radians(4.5) * math.sin(w * 0.25 * t), 0, 0])

    def pitch(t):
        return np.array([0, 0, 0, 0, math.radians(4.5) * math.sin(w * 0.25 * t), 0])

    def yaw(t):
        return np.array([0, 0, 0, 0, 0, math.radians(6.0) * math.sin(w * 0.20 * t)])

    def sway(t):
        return np.array([0.03 * math.sin(w * 0.25 * t), 0, 0, 0, 0, 0])

    def surge(t):
        return np.array([0, 0.03 * math.sin(w * 0.25 * t), 0, 0, 0, 0])

    def combined(t):
        r = ramp(t)
        return r * np.array([
            0.020 * math.sin(w * 0.20 * t),
            0.020 * math.sin(w * 0.17 * t + 1.0),
            0.018 * math.sin(w * 0.25 * t),
            math.radians(3.0) * math.sin(w * 0.23 * t + 0.5),
            math.radians(3.0) * math.sin(w * 0.19 * t),
            math.radians(4.0) * math.sin(w * 0.13 * t),
        ])

    def demo(t):
        """分阶段: 中位 -> 升沉 -> 横滚 -> 俯仰 -> 偏航 -> 混合 -> 回中"""
        seg = 4.0
        idx = int(t // seg)
        lt = t - idx * seg
        fade = math.sin(math.pi * min(1.0, lt / seg)) ** 2
        out = np.zeros(6)
        if idx == 1:
            out[2] = 0.03 * fade
        elif idx == 2:
            out[3] = math.radians(5.0) * fade
        elif idx == 3:
            out[4] = math.radians(5.0) * fade
        elif idx == 4:
            out[5] = math.radians(7.0) * fade
        elif idx == 5:
            out[:] = combined(lt)
        elif idx >= 6:
            out[:] = 0.0
        return out

    return {
        'neutral': neutral, 'heave': heave, 'roll': roll, 'pitch': pitch,
        'yaw': yaw, 'sway': sway, 'surge': surge, 'combined': combined,
        'demo': demo,
    }.get(name, demo)


# ============================ 运行 ============================

def pose_from_pipeline(pipe):
    """运行一次 G_chair_sim 管线, 返回其期望位姿 [x,y,z,roll,pitch,yaw]。"""
    pipe.run_step()
    pf = pipe.history['pose_filtered'][-1]
    return np.array([
        pf['sway'] / 1000.0, pf['surge'] / 1000.0, pf['heave'] / 1000.0,
        math.radians(pf['roll']), math.radians(pf['pitch']), math.radians(pf['yaw']),
    ])


def run(args):
    sim = GChairSim()
    p = sim.p
    dt_ctrl = 1.0 / args.ctrl_hz
    duration = args.duration

    pipeline = None
    if args.pipeline:
        sys.path.insert(0, os.path.join(HERE, '..', 'G_chair_sim'))
        from pipeline import SimulationPipeline
        pipeline = SimulationPipeline()
        duration = min(duration, 60.0)
        scenario = None
        print('[pipeline] 使用 G_chair_sim 遥测→姿态→滤波 链驱动 MuJoCo')
    else:
        scenario = make_scenario(args.scenario, duration)
        print('[scenario] %s, duration=%.1fs, ctrl=%.0fHz' % (args.scenario, duration, args.ctrl_hz))

    viewer = None
    if args.viewer:
        try:
            from mujoco import viewer as mj_viewer
            viewer = mj_viewer.launch_passive(sim.model, sim.data)
            print('[viewer] 已打开 MuJoCo 交互查看器 (关闭窗口或按 Ctrl+C 退出)')
        except Exception as e:
            print('[viewer] 无法打开查看器: %s' % e)
            viewer = None

    renderer = None
    writer = None
    frames_t = 0.0
    if args.render:
        renderer = mujoco.Renderer(sim.model, height=args.height, width=args.width)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [0, 0, 0.55]
        cam.distance = 2.4
        cam.elevation = -15
        cam.azimuth = 135
        os.makedirs(os.path.dirname(os.path.abspath(args.render)), exist_ok=True)
        if args.render.endswith('.gif'):
            writer = 'gif'
        else:
            writer = 'mp4'
        frames = []
        print('[render] %dx%d -> %s' % (args.width, args.height, args.render))

    log = {k: [] for k in ['t', 'cmd', 'act', 'leg_cmd', 'leg_act', 'sat']}
    n_ctrl = int(round(duration / dt_ctrl))
    wall0 = time.time()
    render_every = max(1, int(round(args.fps * dt_ctrl)))

    try:
        for k in range(n_ctrl):
            t = k * dt_ctrl
            if pipeline is not None:
                cmd = pose_from_pipeline(pipeline)
            else:
                cmd = np.asarray(scenario(t), dtype=float)
            disp, lim = sim.command_pose(cmd, dt_ctrl)
            sim.step()
            act = sim.platform_pose()
            leg_act = sim.leg_positions_mm()

            log['t'].append(t)
            log['cmd'].append(cmd)
            log['act'].append(act)
            log['leg_cmd'].append(lim)
            log['leg_act'].append(leg_act)
            log['sat'].append(float(np.max(np.abs(disp)) > p.range * 1000.0))

            if renderer is not None and (k % render_every == 0):
                cam.azimuth = 135 + 18 * math.sin(2 * math.pi * 0.05 * t)
                renderer.update_scene(sim.data, camera=cam)
                frames.append(renderer.render().copy())

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
        if renderer is not None:
            renderer.close()
        if args.render and frames:
            if writer == 'gif':
                import imageio.v2 as imageio
                imageio.mimsave(args.render, frames, fps=args.fps)
            else:
                import imageio.v2 as imageio
                imageio.mimsave(args.render, frames, fps=args.fps, codec='libx264',
                                quality=8, macro_block_size=2)
            print('[render] 已保存 %s (%d frames)' % (args.render, len(frames)))

    report(sim, log, args, pipeline is not None)
    return sim, log


def report(sim, log, args, used_pipeline=False):
    p = sim.p
    t = np.array(log['t'])
    if len(t) == 0:
        return
    cmd = np.array(log['cmd'])
    act = np.array(log['act'])
    leg_cmd = np.array(log['leg_cmd'])
    leg_act = np.array(log['leg_act'])
    sat = np.array(log['sat'])

    pos_err = np.linalg.norm((act[:, :3] - cmd[:, :3]) * 1000.0, axis=1)
    rot_err = np.degrees(np.linalg.norm(act[:, 3:] - cmd[:, 3:], axis=1))
    track = {
        'pos_rms_mm': float(np.sqrt(np.mean(pos_err ** 2))),
        'pos_max_mm': float(np.max(pos_err)),
        'rot_rms_deg': float(np.sqrt(np.mean(rot_err ** 2))),
        'rot_max_deg': float(np.max(rot_err)),
        'leg_max_mm': float(np.max(np.abs(leg_act))),
        'leg_cmd_max_mm': float(np.max(np.abs(leg_cmd))),
        'sat_frames': int(sat.sum()),
        'frames': int(len(t)),
        'mode': 'pipeline' if used_pipeline else args.scenario,
    }
    print('\n================ 仿真结果 ================')
    print(' 模式            : %s' % track['mode'])
    print(' 时长/帧数       : %.1fs / %d' % (t[-1], track['frames']))
    print(' 位置跟踪 RMS/MAX: %.2f / %.2f mm' % (track['pos_rms_mm'], track['pos_max_mm']))
    print(' 姿态跟踪 RMS/MAX: %.2f / %.2f deg' % (track['rot_rms_deg'], track['rot_max_deg']))
    print(' 实际缸位移 MAX  : %.1f mm  (行程 ±%.0f mm)' % (track['leg_max_mm'], p.range * 1000))
    print(' 指令缸位移 MAX  : %.1f mm' % (track['leg_cmd_max_mm']))
    print(' 超程帧数        : %d' % track['sat_frames'])
    print('==========================================\n')

    if args.save_csv or args.plot:
        os.makedirs(OUTPUT, exist_ok=True)
    if args.save_csv:
        path = os.path.join(OUTPUT, 'sim_log.csv')
        arr = np.c_[t, cmd, act, leg_cmd, leg_act]
        header = ('t,' + ','.join('cmd_' + s for s in 'x y z roll pitch yaw'.split()) +
                  ',' + ','.join('act_' + s for s in 'x y z roll pitch yaw'.split()) +
                  ',' + ','.join('legcmd_%d' % i for i in range(6)) +
                  ',' + ','.join('legact_%d' % i for i in range(6)))
        np.savetxt(path, arr, delimiter=',', header=header, comments='')
        print('[csv] 已保存 %s' % path)

    if args.plot:
        plot_log(t, cmd, act, leg_cmd, leg_act, track)


def setup_mpl():
    import matplotlib
    matplotlib.use('Agg')
    matplotlib.rcParams['font.sans-serif'] = ['DejaVu Sans']
    matplotlib.rcParams['axes.unicode_minus'] = False
    return matplotlib


def plot_log(t, cmd, act, leg_cmd, leg_act, track):
    setup_mpl()
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    axs[0].plot(t, cmd[:, 2] * 1000, '--', label='cmd heave')
    axs[0].plot(t, act[:, 2] * 1000, label='act heave')
    axs[0].set_ylabel('heave (mm)')
    axs[0].legend(loc='upper right', fontsize=8)
    axs[1].plot(t, np.degrees(cmd[:, 3]), '--', label='cmd roll')
    axs[1].plot(t, np.degrees(act[:, 3]), label='act roll')
    axs[1].plot(t, np.degrees(cmd[:, 4]), ':', label='cmd pitch')
    axs[1].plot(t, np.degrees(act[:, 4]), label='act pitch')
    axs[1].set_ylabel('angle (deg)')
    axs[1].legend(loc='upper right', fontsize=8)
    for i in range(6):
        axs[2].plot(t, leg_cmd[:, i], '--', lw=0.7)
        axs[2].plot(t, leg_act[:, i], lw=0.8)
    axs[2].set_ylabel('leg displacement (mm)')
    axs[2].set_xlabel('time (s)')
    axs[2].axhline(95, color='r', lw=0.6, ls=':')
    axs[2].axhline(-95, color='r', lw=0.6, ls=':')
    fig.suptitle('G_chair MuJoCo  track RMS %.1fmm / %.1fdeg' %
                 (track['pos_rms_mm'], track['rot_rms_deg']))
    fig.tight_layout()
    path = os.path.join(OUTPUT, 'sim_tracking.png')
    fig.savefig(path, dpi=110)
    plt.close(fig)
    print('[plot] 已保存 %s' % path)


# ============================ 校验 ============================

def validate(args):
    from validate import run_validation
    run_validation(args)


def main():
    ap = argparse.ArgumentParser(description='G_chair Stewart 平台 MuJoCo 仿真')
    ap.add_argument('--scenario', default='demo',
                    choices=['neutral', 'heave', 'roll', 'pitch', 'yaw',
                             'sway', 'surge', 'combined', 'demo'])
    ap.add_argument('--duration', type=float, default=24.0)
    ap.add_argument('--ctrl-hz', type=float, default=50.0)
    ap.add_argument('--viewer', action='store_true', help='打开 MuJoCo 交互查看器')
    ap.add_argument('--render', default=None, help='离屏渲染输出路径 (.mp4/.gif)')
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--realtime', action='store_true', help='按真实时间播放')
    ap.add_argument('--pipeline', action='store_true',
                    help='接入 G_chair_sim 遥测→姿态→滤波链')
    ap.add_argument('--plot', action='store_true', help='保存跟踪曲线')
    ap.add_argument('--save-csv', action='store_true', help='保存 CSV 日志')
    ap.add_argument('--validate', action='store_true', help='运行静态校验')
    args = ap.parse_args()

    if args.validate:
        validate(args)
    else:
        run(args)


if __name__ == '__main__':
    main()
