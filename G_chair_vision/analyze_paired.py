#!/usr/bin/env python3
"""
analyze_paired.py — 分析配对数据集，评判视觉提取姿态的准确性

输入：record_paired.py 产出的真值数据集，或"视觉输出CSV + 真实遥测CSV"
功能：
  1. 算每个通道的【相关系数】【最佳时间延迟】【归一化误差】
  2. 找出视觉算法的短板通道（为优化指方向）
  3. 生成对比曲线图

用法：
  # 分析一次配对录制（真值 vs 视觉需同时录，这里演示用已有数据）
  python3 analyze_paired.py --vision video_telemetry.csv --truth ../G_chair_mujoco/output/raw_telemetry.csv

  # 若数据来自 record_paired，自动按 t 对齐
  python3 analyze_paired.py --paired /tmp/opencode/paired_run

说明：
  - vision CSV: 有 t 列 + 25通道（screen_telemetry.py --csv 产出）
  - truth CSV:  有 t 列 + 25通道（record_paired.py 或 capture_raw.py 产出）
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'G_chair_sim'))
from config import TELEMETRY_CHANNELS

KEYS = ['accel_x', 'accel_z', 'accel_y', 'rot_x', 'rot_y', 'rot_z']


def load_csv(path, dt=0.02):
    """读 CSV，返回 (t数组, {通道:值数组})。

    有 t 列就用它；没有则用帧号×dt 生成时间轴（capture_raw.py 的旧格式）。
    """
    if not os.path.exists(path):
        return None, None
    data = np.genfromtxt(path, delimiter=',', names=True)
    if data.dtype.names is None:
        return None, None
    names = data.dtype.names
    if 't' in names:
        t = np.asarray(data['t'], dtype=float)
    else:
        # 没有时间列：用帧号×dt（旧 capture_raw.py 格式）
        n = len(data[names[0]])
        t = np.arange(n) * dt
        print('  [%s] 无 t 列，按帧号×%.3fs 生成时间轴' % (os.path.basename(path), dt))
    ch = {}
    for k in TELEMETRY_CHANNELS:
        if k in names:
            ch[k] = np.asarray(data[k], dtype=float)
    return t, ch


def best_lag(a, b, max_lag=30):
    """找 b 相对 a 的最佳延迟(帧)。正=b滞后a。"""
    if len(a) < 10 or len(b) < 10:
        return 0, 0.0
    a = (a - a.mean()) / (a.std() + 1e-9)
    b = (b - b.mean()) / (b.std() + 1e-9)
    best_lag_v, best_c = 0, -2.0
    for lag in range(-max_lag, max_lag + 1):
        if lag >= 0:
            aa, bb = a[lag:], b[:len(b) - lag]
        else:
            aa, bb = a[:len(a) + lag], b[-lag:]
        n = min(len(aa), len(bb))
        if n < 10:
            continue
        c = float(np.mean(aa[:n] * bb[:n]))
        if c > best_c:
            best_c, best_lag_v = c, lag
    return best_lag_v, best_c


def analyze(vision_path, truth_path, dt=0.02):
    tv, cv = load_csv(vision_path, dt)
    tt, ct = load_csv(truth_path, dt)
    if tv is None or tt is None:
        print('读取失败:', vision_path, truth_path)
        return

    print('视觉帧 %d (%.1fs), 真值帧 %d (%.1fs)' %
          (len(tv), tv[-1] - tv[0], len(tt), tt[-1] - tt[0]))
    print('\n%-9s %8s %8s %8s %10s  %s' %
          ('通道', '视觉std', '真值std', '幅度比', '相关系数', '判定'))
    results = {}
    for k in KEYS:
        if k not in cv or k not in ct:
            continue
        a, b = cv[k], ct[k]
        # 重采样到同长度（取公共长度）
        n = min(len(a), len(b))
        a, b = a[:n], b[:n]
        std_a, std_b = a.std(), b.std()
        ratio = std_a / (std_b + 1e-9)
        lag, corr = best_lag(a, b)
        if corr > 0.6 and 0.6 < ratio < 1.6:
            verdict = 'OK'
        elif corr <= 0.3:
            verdict = '*** 相关性弱 ***'
        elif ratio > 1.6:
            verdict = '*** 高估 ***'
        elif ratio < 0.6:
            verdict = '*** 低估 ***'
        else:
            verdict = '一般'
        print('%-9s %8.3f %8.3f %8.2f %10.3f  %s (延迟%+d帧/%+.0fms)' %
              (k, std_a, std_b, ratio, corr, verdict, lag, lag * dt * 1000))
        results[k] = (std_a, std_b, ratio, corr, lag)

    # 排名：找最需要改进的通道
    print('\n=== 需要优先改进的通道（相关系数最低）===')
    for k, (_, _, _, corr, _) in sorted(results.items(), key=lambda x: x[1][3])[:3]:
        print('  %s: 相关系数 %.3f' % (k, corr))

    # 给出可执行的增益建议：相关性够高时，按幅值比的倒数缩放 motion_map 增益
    print('\n=== 增益重标定建议（供 --fast 模式/换分辨率后使用）===')
    gain_map = {'accel_x': ['roll', 'sway'],
                'accel_z': ['pitch', 'surge'],
                'accel_y': ['heave'],
                'rot_y': ['yaw']}
    defaults = {'roll': 35.0, 'pitch': 30.0, 'yaw': 10.0,
                'sway': 100.0, 'surge': -100.0, 'heave': 50.0}
    any_tip = False
    for k, gains in gain_map.items():
        if k not in results:
            continue
        _, _, ratio, corr, _ = results[k]
        if corr > 0.5 and ratio > 0.2:
            factor = 1.0 / ratio
            for g in gains:
                new = defaults[g] * factor
                print('  %s 幅值比 %.2f (相关 %.2f) → %s 增益 %.1f → %.1f' %
                      (k, ratio, corr, g, defaults[g], new))
            any_tip = True
    if not any_tip:
        print('  相关性不足或幅值异常，暂不建议自动调增益（先改算法/分辨率）')

    # 画图
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(len(KEYS), 1, figsize=(13, 14), sharex=True)
        n = min(len(tv), len(tt))
        for ax, k in zip(axs, KEYS):
            if k not in cv or k not in ct:
                continue
            ax.plot(tv[:n], cv[k][:n], lw=0.7, alpha=0.8, label='vision')
            ax.plot(tt[:n], ct[k][:n], lw=0.7, alpha=0.8, label='truth')
            ax.set_ylabel(k)
            ax.legend(fontsize=7, loc='upper right')
        axs[-1].set_xlabel('t (s)')
        fig.suptitle('Vision vs Truth: paired telemetry')
        fig.tight_layout()
        p = os.path.join(HERE, 'paired_analysis.png')
        fig.savefig(p, dpi=110)
        print('\n[plot] %s' % p)
    except ImportError:
        pass


def main():
    ap = argparse.ArgumentParser(description='分析配对数据集的相关性')
    ap.add_argument('--vision', default='video_telemetry.csv', help='视觉输出CSV')
    ap.add_argument('--truth', default='../G_chair_mujoco/output/raw_telemetry.csv',
                    help='真实遥测CSV(带t列)')
    ap.add_argument('--dt', type=float, default=0.02)
    args = ap.parse_args()
    # 尊重调用者的工作目录：相对路径按 cwd 解析，不再强制加 HERE
    analyze(os.path.abspath(args.vision), os.path.abspath(args.truth),
            args.dt)


if __name__ == '__main__':
    main()
