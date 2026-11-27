#!/usr/bin/env python3
"""
analyze_run.py — 分析录制的上位机运行日志，量化问题并给出调参建议。

读取 output/host_app_log.csv（t, leg0..5, pose0..5, speed0..5），输出：
  - 行程占用 / 超程帧
  - 缸速分布 / 超限帧（主控硬件上限 128mm/s）
  - 左右转对称性（用 pose3=roll 的方向分布）
  - 体感幅度是否够（roll/pitch 峰值）
  - 给具体的增益/限速调整建议

用法: ./.venv/bin/python tools/analyze_run.py [csv路径]
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')


def pct(a, q):
    return float(np.percentile(a, q))


def compare(paths):
    """并排比较多份录制，用于验证调参改进效果。"""
    print('=' * 72)
    print('运行对比: %s' % '  vs  '.join(os.path.basename(p) for p in paths))
    print('=' * 72)
    rows = []
    for p in paths:
        d = np.loadtxt(p, delimiter=',', skiprows=1)
        if d.ndim == 1:
            d = d.reshape(1, -1)
        if d.shape[1] < 19:
            print('  %s: 字段不足, 跳过' % os.path.basename(p))
            continue
        legs, pose, speed = d[:, 1:7], d[:, 7:13], d[:, 13:19]
        roll = np.degrees(pose[:, 3])
        rows.append((os.path.basename(p),
                     np.max(np.abs(legs)), np.percentile(speed, 99),
                     np.max(np.abs(roll)), np.percentile(speed, 95)))
    if not rows:
        return 1
    print('  %-28s %10s %10s %10s %10s' %
          ('文件', '缸位移MAX', '缸速P99', 'roll MAX', '缸速P95'))
    for name, lm, s99, rm, s95 in rows:
        print('  %-28s %9.1fmm %9.0fmm/s %9.2f° %9.0fmm/s' %
              (name, lm, s99, rm, s95))
    if len(rows) == 2:
        a, b = rows[0], rows[1]
        print('\n  变化(第2份相对第1份):')
        print('    缸位移MAX %+.1fmm, 缸速P99 %+.0fmm/s, roll MAX %+.2f°' %
              (b[1] - a[1], b[2] - a[2], b[3] - a[3]))
    print('=' * 72)
    return 0


def usage():
    print('用法:')
    print('  analyze_run.py [csv路径]              分析单份录制日志')
    print('  analyze_run.py --compare <a.csv> <b.csv>   对比调参前后')
    print('  默认分析 output/host_app_log.csv')


def main():
    if '--help' in sys.argv or '-h' in sys.argv:
        usage()
        return 0
    if len(sys.argv) > 2 and sys.argv[1] == '--compare':
        return compare(sys.argv[2:])
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        ROOT, 'output', 'host_app_log.csv')
    if not os.path.exists(path):
        print('找不到 %s' % path)
        return 1

    data = np.loadtxt(path, delimiter=',', skiprows=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 19:
        print('字段数不足（需要19列: t + leg0..5 + pose0..5 + speed0..5），实际 %d'
              % data.shape[1])
        return 1

    t = data[:, 0]
    legs = data[:, 1:7]          # mm
    pose = data[:, 7:13]         # x,y,z(m) + roll,pitch,yaw(rad)
    speed = data[:, 13:19]       # mm/s
    roll_deg = np.degrees(pose[:, 3])
    pitch_deg = np.degrees(pose[:, 4])
    yaw_deg = np.degrees(pose[:, 5])

    print('=' * 56)
    print('运行分析报告: %s' % os.path.basename(path))
    print('=' * 56)
    print('时长 %.1fs, %d 帧, 平均 %.1fHz' %
          (t[-1] - t[0], len(t), len(t) / max(t[-1] - t[0], 0.01)))

    print('\n[1] 行程占用')
    print('  最大缸位移 %.1f mm (行程 ±95)' % np.max(np.abs(legs)))
    over = (np.abs(legs) > 95).any(axis=1).sum()
    print('  超程帧 %d / %d' % (over, len(t)))
    for i in range(6):
        print('    缸%d: %+7.1f ~ %+7.1f mm' % (i, legs[:, i].min(), legs[:, i].max()))

    print('\n[2] 缸速 / 主控能力 (128mm/s)')
    print('  P50 %.0f  P95 %.0f  P99 %.0f  MAX %.0f mm/s' %
          (pct(speed, 50), pct(speed, 95), pct(speed, 99), speed.max()))
    clip = (speed >= 127).any(axis=1).sum()
    print('  顶到限速帧 %d / %d (%.1f%%)' % (clip, len(t), 100.0 * clip / len(t)))

    print('\n[3] 体感幅度')
    print('  roll  P5 %.2f / P95 %.2f / MAX %.2f deg' %
          (pct(roll_deg, 5), pct(roll_deg, 95), np.max(np.abs(roll_deg))))
    print('  pitch P5 %.2f / P95 %.2f / MAX %.2f deg' %
          (pct(pitch_deg, 5), pct(pitch_deg, 95), np.max(np.abs(pitch_deg))))
    print('  yaw   P5 %.2f / P95 %.2f / MAX %.2f deg' %
          (pct(yaw_deg, 5), pct(yaw_deg, 95), np.max(np.abs(yaw_deg))))

    print('\n[4] 左右对称性（按 roll 符号分组）')
    L = roll_deg > 0.3
    R = roll_deg < -0.3
    print('  正向帧 %d, 负向帧 %d' % (L.sum(), R.sum()))
    if L.sum() > 10 and R.sum() > 10:
        print('  正向 roll 均值 %+.2f / 负向 %+.2f deg, 幅值比 %.2f' %
              (roll_deg[L].mean(), roll_deg[R].mean(),
               abs(roll_deg[L].mean()) / max(abs(roll_deg[R].mean()), 1e-6)))

    print('\n[5] 建议')
    tips = []
    if np.max(np.abs(legs)) > 90:
        tips.append('缸位移接近满行程 → 减小对应平移/角度增益')
    if clip / len(t) > 0.05:
        tips.append('超过5%%的帧顶到限速 → 若嫌动作软可降低 max_speed 要求；若嫌不够快需换更高速电缸')
    if np.max(np.abs(roll_deg)) < 2.0:
        tips.append('roll 峰值 <2° → 体感偏弱，可加大 Web 界面的 roll 增益（当前默认35度/G）')
    if np.max(np.abs(roll_deg)) > 5.8:
        tips.append('roll 峰值接近限幅6° → 可能过于剧烈，可减小 roll 增益')
    if not tips:
        tips.append('各项指标在合理范围，当前参数可用')
    for i, tip in enumerate(tips, 1):
        print('  %d. %s' % (i, tip))
    print('=' * 56)
    return 0


if __name__ == '__main__':
    sys.exit(main())
