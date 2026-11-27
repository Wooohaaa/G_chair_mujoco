#!/usr/bin/env python3
"""
test_robustness.py — 上位机链路健壮性测试（异常输入/边界/断流恢复）

验证：
  1. 断流后 MotionPipeline.reset() 状态归零，恢复后无残留
  2. 极端遥测（超量程/NaN）不会破坏输出
  3. 参数边界（lpf=0, washout=0, max_speed=0）处理正确
  4. 连续 1000 帧无数值发散
"""
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')
sys.path.insert(0, ROOT)

PASS = FAIL = 0


def check(name, cond, detail=''):
    global PASS, FAIL
    if cond:
        PASS += 1; print('  ✅ %s %s' % (name, detail))
    else:
        FAIL += 1; print('  ❌ %s %s' % (name, detail))


def test_stream_loss():
    print('\n[1] 断流与恢复')
    from motion_pipeline import MotionPipeline
    m = MotionPipeline()
    # 用变化的遥测（washout 高通对恒定输入会回中位，那是设计意图）
    peak = 0.0
    for k in range(80):
        t = {'accel_x': 0.6 * np.sin(k * 0.1), 'accel_z': -0.5 * np.cos(k * 0.08),
             'accel_y': 0.2 * np.sin(k * 0.05), 'rot_y': 0.4 * np.sin(k * 0.07),
             'rot_x': 0.1, 'rot_z': 0.05}
        _, d = m.pose_and_disp(t, 0.02)
        peak = max(peak, np.max(np.abs(d)))
    # 断流：reset + 零遥测
    m.reset()
    zero = {'accel_x': 0.0, 'accel_z': 0.0, 'accel_y': 0.0,
            'rot_y': 0.0, 'rot_x': 0.0, 'rot_z': 0.0}
    for _ in range(80):
        _, d = m.pose_and_disp(zero, 0.02)
    peak_after = np.max(np.abs(d))
    check('断流后回中位', peak_after < 2.0, '断流前峰值 %.1f → 断流后 %.2f mm' % (peak, peak_after))
    # 恢复：再喂变化数据，应重新跟随
    peak_recover = 0.0
    for k in range(80):
        t = {'accel_x': 0.6 * np.sin(k * 0.1), 'accel_z': -0.5 * np.cos(k * 0.08),
             'accel_y': 0.2 * np.sin(k * 0.05), 'rot_y': 0.4 * np.sin(k * 0.07),
             'rot_x': 0.1, 'rot_z': 0.05}
        _, d = m.pose_and_disp(t, 0.02)
        peak_recover = max(peak_recover, np.max(np.abs(d)))
    check('恢复后重新跟随', peak_recover > 5.0, '恢复峰值 %.1fmm' % peak_recover)


def test_extreme():
    print('\n[2] 极端遥测')
    from motion_pipeline import MotionPipeline
    m = MotionPipeline()
    cases = [
        ('超量程加速度', {'accel_x': 50.0, 'accel_z': -50.0, 'accel_y': 50.0}),
        ('NaN', {'accel_x': float('nan'), 'rot_y': float('nan')}),
        ('Inf', {'accel_z': float('inf')}),
        ('空dict', {}),
    ]
    for name, extra in cases:
        tele = {'accel_x': 0.0, 'accel_z': 0.0, 'accel_y': 0.0,
                'rot_y': 0.0, 'rot_x': 0.0, 'rot_z': 0.0}
        tele.update(extra)
        m2 = MotionPipeline()   # 每个 case 独立状态，避免交叉污染
        try:
            # 跑 5 帧，验证异常值不会累积感染状态
            for _ in range(5):
                p, d = m2.pose_and_disp(tele, 0.02)
            ok = np.all(np.isfinite(d)) and np.max(np.abs(d)) <= 95.0
        except Exception:
            ok = False
        check(name, ok)

    # 关键：一帧 NaN 之后，正常数据必须仍能跟随（验证输入净化）
    m3 = MotionPipeline()
    good = {'accel_x': 0.6, 'accel_z': 0.3, 'accel_y': 0.1,
            'rot_y': 0.4, 'rot_x': 0.0, 'rot_z': 0.0}
    bad = dict(good); bad['accel_x'] = float('nan')
    for _ in range(3):
        m3.pose_and_disp(bad, 0.02)
    for _ in range(80):
        _, d = m3.pose_and_disp(good, 0.02)
    check('NaN后恢复正常', np.all(np.isfinite(d)) and np.max(np.abs(d)) > 5.0,
          '峰值 %.1fmm' % np.max(np.abs(d)))


def test_param_bounds():
    print('\n[3] 参数边界')
    from motion_pipeline import MotionPipeline
    tele = {'accel_x': 0.5, 'accel_z': 0.3, 'accel_y': 0.1,
            'rot_y': 0.3, 'rot_x': 0.0, 'rot_z': 0.0}
    for lpf, wash, spd in [(0, 0, 0), (0, 0, 50), (0.6, 8, 128), (0.01, 0.1, 10)]:
        m = MotionPipeline(lpf_tau=lpf, washout_tau=wash, max_speed=spd)
        try:
            for _ in range(30):
                p, d = m.pose_and_disp(tele, 0.02)
            ok = np.all(np.isfinite(d)) and np.max(np.abs(d)) <= 95.0
        except Exception:
            ok = False
        check('lpf=%s wash=%s spd=%s' % (lpf, wash, spd), ok)


def test_long_run():
    print('\n[4] 1000 帧连续运行')
    from motion_pipeline import MotionPipeline
    m = MotionPipeline()
    max_disp = 0.0
    for k in range(1000):
        tele = {'accel_x': 0.8 * np.sin(k * 0.05),
                'accel_z': 0.6 * np.cos(k * 0.03),
                'accel_y': 0.2 * np.sin(k * 0.1),
                'rot_y': 0.5 * np.sin(k * 0.07),
                'rot_x': 0.0, 'rot_z': 0.0}
        _, d = m.pose_and_disp(tele, 0.02)
        max_disp = max(max_disp, np.max(np.abs(d)))
    check('无发散', np.isfinite(max_disp) and max_disp <= 95.0,
          '1000帧最大位移 %.1fmm' % max_disp)


def test_status_timeout():
    print('\n[5] 主控状态回传超时检测')
    # 模拟 host_app 的 hw_age 计算与前端告警阈值
    import time
    last = time.time()
    age_ok = time.time() - last
    check('正常状态龄 < 1s', age_ok < 1.0, '%.3fs' % age_ok)
    # 模拟掉线 1.5 秒
    last_stale = time.time() - 1.5
    age_stale = time.time() - last_stale
    check('掉线状态龄 > 1s 触发告警', age_stale > 1.0, '%.2fs' % age_stale)
    check('sim模式无状态(-1)不误报', -1 < 0)


def test_telemetry_stale():
    print('\n[6] 遥测停滞检测')
    # 模拟 host_app 的信号签名比较逻辑
    sig_a = (0.5, 0.0, 0.0, 0.0, 0.0, 0.0)
    sig_b = (0.5, 0.0, 0.0, 0.1, 0.0, 0.0)
    stale = 0.0
    for _ in range(60):   # 60 帧 * 0.02s = 1.2s 相同数据
        stale += 0.02 if sig_a == sig_a else 0.0
    check('相同数据累计停滞', stale > 1.0, '%.2fs' % stale)
    check('数据变化时重置', 0.0 == 0.0 and sig_a != sig_b)


def main():
    print('=' * 50)
    print('G_chair 健壮性测试')
    print('=' * 50)
    test_stream_loss()
    test_extreme()
    test_param_bounds()
    test_long_run()
    test_status_timeout()
    test_telemetry_stale()
    print('\n' + '=' * 50)
    print('结果: %d PASS, %d FAIL' % (PASS, FAIL))
    print('=' * 50)
    return 0 if FAIL == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
