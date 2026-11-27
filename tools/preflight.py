#!/usr/bin/env python3
"""
preflight.py — 上真实电机前的 GO/NO-GO 检查汇总

把所有能自动验证的项目跑一遍，输出简洁的"可否上实机"结论 + 人工检查清单。
不替代 POWERON_CHECKLIST.md 的实物步骤，只确认"代码层面已就绪"。

运行: ./.venv/bin/python tools/preflight.py
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')
PY = sys.executable


def run(cmd, cwd=None):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=300,
                           cwd=cwd or ROOT)
        return r.returncode == 0, r.stdout + r.stderr
    except Exception as e:
        return False, str(e)


def main():
    print('=' * 60)
    print('G_chair 上实机前检查 (Preflight)')
    print('=' * 60)
    checks = []

    # 1. 链路测试
    ok, out = run([PY, 'tests/test_chain.py'])
    line = [l for l in out.splitlines() if '结果:' in l]
    checks.append(('全链路测试(19项)', ok, line[-1] if line else '无输出'))

    # 2. 健壮性测试
    ok, out = run([PY, 'tests/test_robustness.py'])
    line = [l for l in out.splitlines() if '结果:' in l]
    checks.append(('健壮性测试(18项)', ok, line[-1] if line else '无输出'))

    # 2b. 协议换算测试
    ok, out = run([PY, 'tests/test_protocol.py'])
    line = [l for l in out.splitlines() if '结果:' in l]
    checks.append(('协议/换算测试(26项)', ok, line[-1] if line else '无输出'))

    # 2c. Web/WS 接口测试
    ok, out = run([PY, 'tests/test_web.py'])
    line = [l for l in out.splitlines() if '结果:' in l]
    checks.append(('Web/WS接口测试(6项)', ok, line[-1] if line else '无输出'))

    # 3. 上位机自检
    ok, out = run([PY, 'host_app.py', '--check'])
    checks.append(('上位机自检', ok, '映射/帧格式 OK' if ok else '失败'))

    # 4. MuJoCo 校验
    ok, out = run([PY, 'sim.py', '--validate'], cwd=os.path.join(ROOT, 'G_chair_mujoco'))
    line = [l for l in out.splitlines() if '结论:' in l]
    checks.append(('MuJoCo 机构校验', ok, line[-1] if line else '无输出'))

    # 5. 固件逻辑仿真
    ok, out = run([PY, 'G_chair_esp32/tests/firmware_sim.py'])
    line = [l for l in out.splitlines() if '全部固件逻辑检查通过' in l]
    checks.append(('固件逻辑仿真', ok and bool(line), '步进/限速/减速 OK' if line else '失败'))

    # 6. 全链路数值仿真
    ok, out = run([PY, 'tools/full_chain_sim.py'])
    line = [l for l in out.splitlines() if '结论:' in l]
    checks.append(('全链路数值仿真', ok and bool(line), line[-1] if line else '失败'))

    # 6b. 视觉节点 mock（默认 + --fast），验证可选回退链路可运行
    vis_args = ['G_chair_vision/screen_telemetry.py', '--mock', '--ip', '127.0.0.1',
                '--duration', '2', '--hz', '20']
    ok1, out1 = run([PY] + vis_args)
    ok2, out2 = run([PY] + vis_args + ['--fast', '--proc-size', '256x144'])
    f1 = [l for l in out1.splitlines() if '共发送' in l]
    f2 = [l for l in out2.splitlines() if '共发送' in l]
    checks.append(('视觉节点mock(默认+fast)',
                   ok1 and ok2 and bool(f1) and bool(f2),
                   '%s / %s' % (f1[-1] if f1 else '失败', f2[-1] if f2 else '失败')))

    print()
    for name, ok, detail in checks:
        print('  %s %-22s %s' % ('✅' if ok else '❌', name, detail))
    print()
    all_ok = all(c[1] for c in checks)
    if all_ok:
        print('结论: ✅ 代码层面全部就绪，可以进入实物接线/上电阶段')
        print('\n人工必做（按 G_chair_esp32/POWERON_CHECKLIST.md）:')
        print('  1. 共地: ESP32 GND ↔ 伺服 24V-')
        print('  2. 信号线串 2.2kΩ 电阻')
        print('  3. 伺服参数(日鼎 PN-01/02/04/41, 位置模式)')
        print('  4. 串口监视器发 #T 自检, 看电机是否逐轴点动')
        print('  5. 空载 → 单缸带载 → 6 缸逐个验证')
        print('  6. 若 3.3V 不驱动 → 补 74HCT245 电平转换')
    else:
        print('结论: ❌ 有检查未通过，先修复后再上实机')
    print('=' * 60)
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
