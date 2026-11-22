"""
visualize.py — 3D 可视化 + 2D 图表
使用 matplotlib 实现:
  1. 3D 动画: Stewart 平台实时运动
  2. 腿长位移曲线
  3. 姿态曲线
  4. 滤波对比
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')  # 非交互模式, 保存文件
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d import Axes3D
from config import RIG, SIM, OUTPUT_DIR
import os

# 中文字体
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


def create_output_dir():
    os.makedirs(OUTPUT_DIR, exist_ok=True)


def plot_displacements(history, save_path):
    """腿长位移曲线"""
    fig, axes = plt.subplots(2, 1, figsize=(14, 8))
    times = history['time']
    disp = np.array(history['displacements'])
    ctrl = np.array(history['arduino_positions'])
    colors = ['#e74c3c', '#e67e22', '#f1c40f', '#2ecc71', '#3498db', '#9b59b6']
    names = ['Axis1a', 'Axis2a', 'Axis3a', 'Axis4a', 'Axis5a', 'Axis6a']

    # 上: FlyPT输出位移
    ax = axes[0]
    for i in range(6):
        ax.plot(times, disp[:, i], color=colors[i], label=names[i], linewidth=1.2)
    ax.axhline(y=RIG['range'], color='red', linestyle='--', alpha=0.5, label='行程上限(+95mm)')
    ax.axhline(y=-RIG['range'], color='red', linestyle='--', alpha=0.5, label='行程下限(-95mm)')
    ax.axhline(y=0, color='gray', linewidth=0.5)
    ax.set_ylabel('位移 (mm)')
    ax.set_title('FlyPT 逆运动学输出 (6轴电缸位移)')
    ax.legend(loc='upper right', fontsize=8, ncol=3)
    ax.grid(True, alpha=0.3)

    # 下: Arduino 控制器跟踪
    ax = axes[1]
    for i in range(6):
        ax.plot(times, ctrl[:, i], color=colors[i], label=names[i], linewidth=1.2, alpha=0.7)
    ax.axhline(y=RIG['range'], color='red', linestyle='--', alpha=0.5)
    ax.axhline(y=-RIG['range'], color='red', linestyle='--', alpha=0.5)
    ax.axhline(y=0, color='gray', linewidth=0.5)
    ax.set_ylabel('位移 (mm)')
    ax.set_xlabel('时间 (s)')
    ax.set_title('Arduino 控制器实际跟踪位移')
    ax.legend(loc='upper right', fontsize=8, ncol=3)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    plt.close()


def plot_pose(history, save_path):
    """姿态曲线"""
    fig, axes = plt.subplots(2, 1, figsize=(14, 8))
    times = history['time']
    pose_raw = history['pose_raw']
    pose_filt = history['pose_filtered']

    # 上: 线性位移
    ax = axes[0]
    for i, (key, label, color) in enumerate([
        ('sway', 'Sway 横向 (mm)', '#e74c3c'),
        ('surge', 'Surge 纵向 (mm)', '#e67e22'),
        ('heave', 'Heave 垂直 (mm)', '#2ecc71'),
    ]):
        ax.plot(times, [p[key] for p in pose_filt], color=color, label=label, linewidth=1.5)
    ax.axhline(y=0, color='gray', linewidth=0.5)
    ax.set_ylabel('位移 (mm)')
    ax.set_title('平台线性位移 (滤波后)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 下: 旋转角度
    ax = axes[1]
    for i, (key, label, color) in enumerate([
        ('roll', 'Roll 滚转 (°)', '#3498db'),
        ('pitch', 'Pitch 俯仰 (°)', '#9b59b6'),
        ('yaw', 'Yaw 偏航 (°)', '#e67e22'),
    ]):
        ax.plot(times, [p[key] for p in pose_filt], color=color, label=label, linewidth=1.5)
    ax.axhline(y=0, color='gray', linewidth=0.5)
    ax.set_ylabel('角度 (°)')
    ax.set_xlabel('时间 (s)')
    ax.set_title('平台旋转姿态 (滤波后)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    plt.close()


def plot_filter_comparison(history, save_path):
    """滤波前后对比"""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    times = history['time']
    pose_raw = history['pose_raw']
    pose_filt = history['pose_filtered']

    dofs = [
        ('sway', 'Sway (mm)', '#e74c3c'),
        ('surge', 'Surge (mm)', '#e67e22'),
        ('heave', 'Heave (mm)', '#2ecc71'),
    ]

    for ax, (key, label, color) in zip(axes, dofs):
        raw_data = [p[key] for p in pose_raw]
        filt_data = [p[key] for p in pose_filt]
        ax.plot(times, raw_data, color=color, alpha=0.4, label='原始', linewidth=1)
        ax.plot(times, filt_data, color=color, linewidth=1.5, label='滤波后')
        ax.axhline(y=0, color='gray', linewidth=0.5)
        ax.set_ylabel(label)
        ax.set_title(f'{label} 滤波效果对比')
        ax.legend()
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel('时间 (s)')
    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    plt.close()


def create_3d_animation(pipeline, history, save_path, fps=15):
    """创建3D平台运动动画 (GIF)"""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # 获取平台几何
    base_pts = pipeline.rig.get_base_points()
    plat_pts_neutral = pipeline.rig.plat_points.copy()

    # 预设图形对象
    base_lines = []
    leg_lines = []
    plat_lines = []

    base_colors = ['#3498db'] * 6
    leg_colors = ['#e74c3c', '#e67e22', '#f1c40f', '#2ecc71', '#3498db', '#9b59b6']
    plat_colors = ['#2ecc71'] * 6

    for i in range(6):
        # 底座标记
        b, = ax.plot([base_pts[i, 0]], [base_pts[i, 1]], [base_pts[i, 2]],
                     'o', color=base_colors[i], markersize=8)
        base_lines.append(b)

        # 腿
        l, = ax.plot([], [], [], color=leg_colors[i], linewidth=3)
        leg_lines.append(l)

        # 平台铰点
        p, = ax.plot([], [], [], 's', color=plat_colors[i], markersize=6)
        plat_lines.append(p)

    # 平台面
    plat_face, = ax.plot([], [], [], 'k-', alpha=0.3, linewidth=1)

    # 坐标轴
    ax.set_xlim(-400, 400)
    ax.set_ylim(-400, 400)
    ax.set_zlim(0, 1100)
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_title('G_chair Stewart 六轴平台 3D 运动仿真')

    # 场景标签
    scenario_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes,
                              fontsize=10, verticalalignment='top')

    frame_skip = int(max(1, len(history['time']) // int(SIM['duration'] * fps)))
    total_frames = int(len(history['time']) // frame_skip) + 1

    def update(frame):
        idx = frame * frame_skip
        if idx >= len(history['time']):
            idx = len(history['time']) - 1

        pose = history['pose_filtered'][idx]
        disp = history['displacements'][idx]

        # 计算平台位姿
        x = pose['sway']
        y = pose['surge']
        z = pose['heave']
        roll = np.radians(pose['roll'])
        pitch = np.radians(pose['pitch'])
        yaw = np.radians(pose['yaw'])

        plat_world = pipeline.rig.get_plat_world(x, y, z, roll, pitch, yaw)

        # 更新腿
        for i in range(6):
            leg_lines[i].set_data(
                [base_pts[i, 0], plat_world[i, 0]],
                [base_pts[i, 1], plat_world[i, 1]]
            )
            leg_lines[i].set_3d_properties(
                [base_pts[i, 2], plat_world[i, 2]]
            )
            plat_lines[i].set_data([plat_world[i, 0]], [plat_world[i, 1]])
            plat_lines[i].set_3d_properties([plat_world[i, 2]])

        # 平台面 (六边形)
        plat_face.set_data(
            list(plat_world[:, 0]) + [plat_world[0, 0]],
            list(plat_world[:, 1]) + [plat_world[0, 1]]
        )
        plat_face.set_3d_properties(
            list(plat_world[:, 2]) + [plat_world[0, 2]]
        )

        # 场景标签
        scenario_text.set_text(
            f'场景: {history["scenario"][idx]}\n'
            f't = {history["time"][idx]:.1f}s\n'
            f'位移: {disp[0]:+.1f}, {disp[1]:+.1f}, {disp[2]:+.1f}, {disp[3]:+.1f}, {disp[4]:+.1f}, {disp[5]:+.1f} mm'
        )

        return leg_lines + plat_lines + base_lines + [plat_face, scenario_text]

    anim = FuncAnimation(
        fig, update, frames=total_frames,
        interval=1000 / fps, blit=False, repeat=True
    )

    # 保存为 GIF
    anim.save(save_path, writer=PillowWriter(fps=fps))
    plt.close(fig)


def plot_workspace_validation(rig, save_path):
    """工作空间验证图"""
    fig = plt.figure(figsize=(12, 5))

    # 子图1: 可达空间截面
    ax1 = fig.add_subplot(121, projection='3d')
    # 随机采样并过滤可达点
    n = 500
    accessible_pts = []
    for _ in range(n):
        x = np.random.uniform(-30, 30)
        y = np.random.uniform(-30, 30)
        z = np.random.uniform(-15, 15)
        roll = np.random.uniform(-0.1, 0.1)
        pitch = np.random.uniform(-0.1, 0.1)
        yaw = np.random.uniform(-0.1, 0.1)
        disp = rig.inverse_kinematics(x, y, z, roll, pitch, yaw)
        if np.all(np.abs(disp) <= rig.range):
            accessible_pts.append([x, y, z])

    if accessible_pts:
        pts = np.array(accessible_pts)
        ax1.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c='cyan', s=2, alpha=0.5)

    # 中位平台
    mid = rig.get_plat_world(0, 0, 0, 0, 0, 0)
    ax1.plot(mid[:, 0], mid[:, 1], mid[:, 2], 'r-', linewidth=2, label='中位平台')

    ax1.set_xlabel('X (mm)')
    ax1.set_ylabel('Y (mm)')
    ax1.set_zlabel('Z (mm)')
    ax1.set_title('可达工作空间 (随机采样)')
    ax1.legend()

    # 子图2: 腿长位移分布
    ax2 = fig.add_subplot(122)
    all_disp = []
    for _ in range(n):
        x = np.random.uniform(-30, 30)
        y = np.random.uniform(-30, 30)
        z = np.random.uniform(-15, 15)
        roll = np.random.uniform(-0.1, 0.1)
        pitch = np.random.uniform(-0.1, 0.1)
        yaw = np.random.uniform(-0.1, 0.1)
        disp = rig.inverse_kinematics(x, y, z, roll, pitch, yaw)
        all_disp.append(disp)

    all_disp = np.array(all_disp)
    for i in range(6):
        ax2.hist(all_disp[:, i], bins=50, alpha=0.5, label=f'Axis{i+1}a')
    ax2.axvline(x=rig.range, color='red', linestyle='--', label='行程上限')
    ax2.axvline(x=-rig.range, color='red', linestyle='--', label='行程下限')
    ax2.set_xlabel('腿长位移 (mm)')
    ax2.set_ylabel('频次')
    ax2.set_title('腿长位移分布')
    ax2.legend(fontsize=7)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    plt.close()
