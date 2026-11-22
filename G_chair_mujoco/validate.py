"""
validate.py — 静态位姿校验 / 工作空间与行程检查

校验内容:
  1. 机构雅可比条件数 (奇异/接近奇异检查)
  2. 位姿包线内各极值点: 命令位姿 -> IK 缸位移 -> MuJoCo 收敛后实际位姿
     统计位置/姿态跟踪误差, 检查是否超行程
  3. 随机位姿抽样: 可达性与行程占用统计

输出:
  output/validation_report.txt
  output/validation.png
"""

import math
import os

import numpy as np

from sim import GChairSim, OUTPUT


def envelope_poses():
    """Mover 允许范围: sway/surge ±50mm, heave ±30mm, roll/pitch ±6°, yaw ±8°。"""
    Z = lambda mm: mm / 1000.0
    D = math.radians
    poses = [('neutral', np.zeros(6))]
    for s in (+1, -1):
        poses += [
            ('heave%+d' % (30 * s), np.array([0, 0, Z(30 * s), 0, 0, 0])),
            ('sway%+d' % (50 * s), np.array([Z(50 * s), 0, 0, 0, 0, 0])),
            ('surge%+d' % (50 * s), np.array([0, Z(50 * s), 0, 0, 0, 0])),
            ('roll%+d' % (6 * s), np.array([0, 0, 0, D(6 * s), 0, 0])),
            ('pitch%+d' % (6 * s), np.array([0, 0, 0, 0, D(6 * s), 0])),
            ('yaw%+d' % (8 * s), np.array([0, 0, 0, 0, 0, D(8 * s)])),
        ]
    poses.append(('combined', np.array([Z(40), Z(-40), Z(25), D(4), D(-4), D(6)])))
    return poses


def condition_number(sim):
    p = sim.p
    B = p.base_anchors()
    P = p.platform_anchors()
    H = p.neutral_height()
    J = np.zeros((6, 6))
    for i in range(6):
        d = np.array([P[i, 0], P[i, 1], H]) - B[i]
        u = d / np.linalg.norm(d)
        r = np.array([P[i, 0], P[i, 1], 0.0])
        J[i, :3] = u
        J[i, 3:] = np.cross(r, u)
    s = np.linalg.svd(J, compute_uv=False)
    return float(s[0] / s[-1]), s


def run_validation(args=None):
    os.makedirs(OUTPUT, exist_ok=True)
    sim = GChairSim()
    p = sim.p
    settle = 1.5
    cond, sv = condition_number(sim)

    rows = []
    for name, cmd in envelope_poses():
        sim.reset()
        disp, lim = sim.command_pose(cmd, dt_ctrl=0.02)
        sim.data.ctrl[:] = lim / 1000.0
        sim.advance(settle)
        act = sim.platform_pose()
        pos_err = np.linalg.norm((act[:3] - cmd[:3]) * 1000.0)
        rot_err = math.degrees(np.linalg.norm(act[3:] - cmd[3:]))
        legs = sim.leg_positions_mm()
        # 用实际缸位移做解析正运动学, 与 MuJoCo 实际位姿比较, 验证机构模型一致性
        fk = sim.rig.forward_kinematics(legs, seed=np.r_[cmd[:3] * 1000.0, cmd[3:]])
        fk_pos = np.linalg.norm(fk[:3] / 1000.0 - act[:3]) * 1000.0
        fk_rot = math.degrees(np.linalg.norm(fk[3:] - act[3:]))
        sat = bool(np.max(np.abs(disp)) > p.range * 1000.0)
        rows.append((name, np.max(np.abs(disp)), np.max(np.abs(legs)),
                     pos_err, rot_err, sat, fk_pos, fk_rot))

    names = [r[0] for r in rows]
    cmd_leg = np.array([r[1] for r in rows])
    act_leg = np.array([r[2] for r in rows])
    pos_err = np.array([r[3] for r in rows])
    rot_err = np.array([r[4] for r in rows])
    sat = np.array([r[5] for r in rows])

    # 随机抽样检查可达性
    rng = np.random.default_rng(0)
    n_rand = 500
    reach = 0
    max_disp = 0.0
    for _ in range(n_rand):
        cmd = np.array([
            rng.uniform(-0.050, 0.050), rng.uniform(-0.050, 0.050),
            rng.uniform(-0.030, 0.030), math.radians(rng.uniform(-6, 6)),
            math.radians(rng.uniform(-6, 6)), math.radians(rng.uniform(-8, 8))])
        disp = sim.rig.inverse_kinematics(cmd[0] * 1000, cmd[1] * 1000, cmd[2] * 1000,
                                          cmd[3], cmd[4], cmd[5])
        m = np.max(np.abs(disp))
        max_disp = max(max_disp, m)
        if m <= p.range * 1000:
            reach += 1

    lines = []
    lines.append('G_chair Stewart 平台 MuJoCo 校验报告')
    lines.append('=' * 52)
    lines.append('几何: 底座半径 %.3f m, 平台半径 %.3f m' % (p.base_radius, p.plat_radius))
    lines.append('      中位腿长 %.3f m, 中位高度 %.4f m, 行程 ±%.0f mm'
                 % (p.leg_middle, p.neutral_height(), p.range * 1000))
    lines.append('机构雅可比: cond=%.2f, 奇异值=%s' % (cond, np.round(sv, 3)))
    lines.append('')
    lines.append('%-10s %10s %10s %10s %10s %6s' %
                 ('pose', 'IK腿max', '实际腿max', '位置err', '姿态err', '超程'))
    lines.append('%-10s %10s %10s %10s %10s %6s' %
                 ('', '(mm)', '(mm)', '(mm)', '(deg)', ''))
    for r in rows:
        lines.append('%-10s %10.2f %10.2f %10.2f %10.3f %6s' %
                     (r[0], r[1], r[2], r[3], r[4], 'YES' if r[5] else '-'))
    lines.append('')
    lines.append('静态跟踪: 位置 RMS=%.2f mm, MAX=%.2f mm; 姿态 RMS=%.3f deg, MAX=%.3f deg'
                 % (np.sqrt(np.mean(pos_err ** 2)), pos_err.max(),
                    np.sqrt(np.mean(rot_err ** 2)), rot_err.max()))
    lines.append('随机位姿可达率: %d/%d = %.1f%%  (包线内, 行程 ±%.0f mm)'
                 % (reach, n_rand, 100.0 * reach / n_rand, p.range * 1000))
    lines.append('随机抽样最大缸位移: %.1f mm' % max_disp)
    lines.append('MuJoCo vs 解析FK: 位置 MAX=%.2f mm, 姿态 MAX=%.3f deg  (机构一致性)'
                 % (max(r[6] for r in rows), max(r[7] for r in rows)))

    ok = reach == n_rand and not sat.any() and pos_err.max() < 6.0 and rot_err.max() < 0.6
    lines.append('')
    lines.append('结论: %s' % ('PASS (机构稳定, 包线内可达, 跟踪误差在阈值内)'
                              if ok else 'CHECK (见上表, 可能存在超程或跟踪偏差)'))
    report = '\n'.join(lines)
    print('\n' + report + '\n')
    with open(os.path.join(OUTPUT, 'validation_report.txt'), 'w', encoding='utf-8') as fh:
        fh.write(report + '\n')

    plot_validation(p, names, cmd_leg, act_leg, pos_err, rot_err, cond, reach, n_rand)
    return ok


def plot_validation(p, names, cmd_leg, act_leg, pos_err, rot_err, cond, reach, n_rand):
    from sim import setup_mpl
    setup_mpl()
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(2, 2, figsize=(13, 9))
    x = np.arange(len(names))
    axs[0, 0].bar(x - 0.2, cmd_leg, 0.4, label='IK command', color='C0')
    axs[0, 0].bar(x + 0.2, act_leg, 0.4, label='MuJoCo actual', color='C1')
    axs[0, 0].axhline(95, color='r', ls=':', lw=0.8)
    axs[0, 0].axhline(-95, color='r', ls=':', lw=0.8)
    axs[0, 0].set_xticks(x)
    axs[0, 0].set_xticklabels(names, rotation=55, ha='right', fontsize=7)
    axs[0, 0].set_ylabel('max leg displacement (mm)')
    axs[0, 0].set_title('Leg displacement (stroke ±95 mm)')
    axs[0, 0].legend(fontsize=8)

    axs[0, 1].bar(x, pos_err, color='C2')
    axs[0, 1].set_xticks(x)
    axs[0, 1].set_xticklabels(names, rotation=55, ha='right', fontsize=7)
    axs[0, 1].set_ylabel('position error (mm)')
    axs[0, 1].set_title('Position tracking error')

    axs[1, 0].bar(x, rot_err, color='C3')
    axs[1, 0].set_xticks(x)
    axs[1, 0].set_xticklabels(names, rotation=55, ha='right', fontsize=7)
    axs[1, 0].set_ylabel('orientation error (deg)')
    axs[1, 0].set_title('Orientation tracking error')

    axs[1, 1].axis('off')
    txt = ('Geometry\n'
           '  base/plat radius: %.2f / %.2f m\n'
           '  mid leg / height: %.3f / %.4f m\n'
           '  stroke: +/-95 mm\n\n'
           'Mechanism\n'
           '  Jacobian condition: %.2f  (>100 near singular)\n'
           '  random pose reach: %d/%d = %.1f%%\n\n'
           'Static tracking\n'
           '  position MAX: %.2f mm\n'
           '  orientation MAX: %.3f deg\n'
           % (p.base_radius, p.plat_radius, p.leg_middle, p.neutral_height(), cond,
              reach, n_rand, 100.0 * reach / n_rand,
              pos_err.max(), rot_err.max()))
    axs[1, 1].text(0.02, 0.98, txt, va='top', fontsize=10, family='monospace')
    fig.suptitle('G_chair MuJoCo validation', fontsize=14)
    fig.tight_layout()
    path = os.path.join(OUTPUT, 'validation.png')
    fig.savefig(path, dpi=120)
    plt.close(fig)
    print('[plot] 已保存 %s' % path)
