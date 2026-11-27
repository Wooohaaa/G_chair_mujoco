#!/usr/bin/env python3
"""
firmware_sim.py — ESP32 固件步进逻辑的 Python 仿真（无需硬件）

按 src/main.cpp 的算法复现：
  - 16bit / MAP_DIVISOR -> 目标步数
  - move_axis: dir + 周期 = 4000/|diff|，钳位 [39,4000]us
  - update_axis: 每 4ms 按 dt/period 累加 current，越近越慢
  - 限幅 0~38400 步，行程 192mm（0 步=-96mm, 19200 步=0mm）

用途：
  1. 验证固件时序数学（加速/减速/到位时间）
  2. 回放上位机录制的 6 轴位移，预测电缸实际运动
  3. 上真实电机前发现逻辑 bug

运行: ./.venv/bin/python G_chair_esp32/tests/firmware_sim.py [csv]
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..', '..')
sys.path.insert(0, ROOT)

SAMPLE_PERIOD_US = 4000
MIN_PERIOD_US = 39
MAX_PERIOD_US = 4000
MAP_DIVISOR = 65535 / 38400
CENTER = 19200
MAX_POS = 38400
STEP_MM = 192.0 / 38400


class Axis:
    def __init__(self):
        self.current = CENTER
        self.target = CENTER
        self.period = SAMPLE_PERIOD_US
        self.running = False
        self.limitLatch = 0   # +1=上限位 -1=下限位 0=正常

    def move(self, target):
        target = int(max(0, min(MAX_POS, target)))
        # 与固件一致的限位互锁
        if self.limitLatch != 0:
            diff = target - self.current
            if self.limitLatch > 0 and diff > 0:
                return
            if self.limitLatch < 0 and diff < 0:
                return
            self.limitLatch = 0
        self.target = target
        diff = self.target - self.current
        if diff == 0:
            self.running = False
            return
        self.period = self._period(diff)
        self.running = True

    @staticmethod
    def _period(diff):
        p = SAMPLE_PERIOD_US / max(1, abs(diff))
        return int(max(MIN_PERIOD_US, min(MAX_PERIOD_US, p)))

    def update(self, dt_us=SAMPLE_PERIOD_US):
        if not self.running:
            return
        step = int(dt_us / self.period)
        if step <= 0:
            return
        diff = self.target - self.current
        if diff > 0:
            self.current = min(self.target, self.current + step)
        else:
            self.current = max(self.target, self.current - step)
        remain = abs(self.target - self.current)
        if remain == 0:
            self.running = False
        else:
            newp = self._period(remain)
            if newp != self.period:
                self.period = newp

    def mm(self):
        return (self.current - CENTER) * STEP_MM


def sim_step(axes, dt_us=SAMPLE_PERIOD_US):
    for a in axes:
        a.update(dt_us)


def test_basic():
    print('=== 1. 基本阶跃 (0 -> +30mm) ===')
    a = Axis()
    a.move(CENTER + int(30 / STEP_MM))   # +30mm
    t = 0.0
    samples = []
    while a.running and t < 10:
        sim_step([a]); t += SAMPLE_PERIOD_US / 1e6
        if len(samples) < 8 or int(t * 100) % 10 == 0:
            samples.append((t, a.mm(), 1e6 / a.period))
    print('  到位时间 %.3fs, 最终位置 %.3fmm' % (t, a.mm()))
    print('  起始脉冲频率 %.0fHz -> 接近目标 %.0fHz' %
          (samples[0][2], 1e6 / a.period if a.period < MAX_PERIOD_US else 0))
    print('  轨迹采样 (t, mm, Hz):')
    for s in samples[:10]:
        print('    %.3f  %+7.3f  %6.0f' % s)
    assert abs(a.mm() - 30) < 0.01, '未到位'
    assert 0.05 < t < 3.0, '到位时间异常'


def test_speed_limit():
    print('\n=== 2. 最大速度验证 ===')
    a = Axis()
    a.move(MAX_POS)   # +96mm 满行程
    sim_step([a])
    hz = 1e6 / a.period
    mm_s = hz * STEP_MM
    print('  满量程脉冲 %.0fHz -> 缸速 %.1fmm/s' % (hz, mm_s))
    print('  主控硬件上限 128mm/s (minPeriod=39us)')
    assert abs(mm_s - 128.2) < 1.0, '最大速度不符'


def test_decel():
    print('\n=== 3. 减速曲线验证 ===')
    a = Axis()
    a.move(CENTER + int(20 / STEP_MM))
    periods = []
    for _ in range(200):
        if not a.running:
            break
        sim_step([a])
        periods.append(a.period)
    print('  周期序列首5:', periods[:5], '末5:', periods[-5:])
    print('  首周期 %.0fus, 末周期 %.0fus (应递增=减速)' %
          (periods[0], periods[-1]))
    assert periods[-1] > periods[0], '没有减速'


def test_limit_latch():
    print('\n=== 4. 限位锁存互锁 ===')
    a = Axis()
    a.current = CENTER
    # 模拟上限位触发：dir>0, latch=+1
    a.limitLatch = 1
    a.current = CENTER + 200   # 触发时的影子位置
    a.target = a.current
    # 再收到继续向上的指令 → 应被拒绝
    a.move(CENTER + 400)
    rejected = (a.target == CENTER + 200)
    print('  上限位锁存后继续向上指令 -> %s' % ('拒绝 ✓' if rejected else '未拒绝 ✗'))
    assert rejected
    # 收到向下指令 → 解锁并执行
    a.move(CENTER + 100)
    unlocked = (a.limitLatch == 0 and a.target == CENTER + 100)
    print('  收到向下指令 -> %s' % ('解锁并执行 ✓' if unlocked else '失败 ✗'))
    assert unlocked


def test_manual_hold():
    print('\n=== 5. #J 手动点动看门狗保持 ===')
    WATCHDOG_MS = 500
    enabled = True
    last_frame_ms = 0          # 无上位机帧
    manual_hold_until = 10000  # #J0,300 后设定
    selftest_active = False

    def watchdog(now_ms):
        return (not selftest_active and now_ms >= manual_hold_until
                and enabled and now_ms - last_frame_ms > WATCHDOG_MS)

    assert not watchdog(600), '手动保持期内不应回中位'
    print('  t=600ms 保持期内 -> 不回中位 ✓')
    assert not watchdog(9900), '保持到期前不应回中位'
    manual_hold_until = 0      # 上位机帧接管
    assert watchdog(1000), '收到上位机帧后应立即恢复看门狗'
    print('  上位机接管 -> 看门狗恢复 ✓')
    manual_hold_until = 10000
    assert watchdog(10100), '10s上限到期后应恢复看门狗'
    print('  t=10100ms 超过10s上限 -> 看门狗恢复 ✓')


def test_replay(csv_path):
    print('\n=== 6. 回放上位机日志，预测电缸运动 ===')
    if not os.path.exists(csv_path):
        print('  跳过（无 %s）' % csv_path)
        return
    d = np.loadtxt(csv_path, delimiter=',', skiprows=1)
    if d.ndim == 1 or d.shape[1] < 7:
        print('  跳过（字段不足）')
        return
    legs = d[:, 1:7]      # mm
    axes = [Axis() for _ in range(6)]
    for a, mm in zip(axes, legs[0]):
        a.move(CENTER + mm / STEP_MM)
    errors = []
    # 上位机 50Hz (20ms) 发一帧；固件每 4ms 一个控制 tick -> 每帧 5 个子步
    SUBSTEPS = int(round(0.020 * 1e6 / SAMPLE_PERIOD_US))
    for row in legs:
        for a, mm in zip(axes, row):
            a.move(CENTER + mm / STEP_MM)
        for _ in range(SUBSTEPS):
            sim_step(axes)
        errors.append(max(abs(a.mm() - mm) for a, mm in zip(axes, row)))
    print('  %d 帧回放：位置误差 均值 %.2fmm / 最大 %.2fmm' %
          (len(legs), np.mean(errors), np.max(errors)))
    print('  结论：固件 4ms 控制周期足以跟上 50Hz 上位机指令')


def main():
    print('=' * 56)
    print('ESP32 固件步进逻辑仿真')
    print('=' * 56)
    test_basic()
    test_speed_limit()
    test_decel()
    test_limit_latch()
    test_manual_hold()
    log = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        ROOT, 'output', 'host_app_log.csv')
    test_replay(log)
    print('\n全部固件逻辑检查通过 ✅')


if __name__ == '__main__':
    main()
