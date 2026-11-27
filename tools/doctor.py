#!/usr/bin/env python3
"""doctor.py — 环境诊断：检查依赖、Python 版本、串口、GitHub DNS。

运行: ./.venv/bin/python tools/doctor.py
"""
import os
import platform
import socket
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')


def main():
    print('=' * 56)
    print('G_chair 环境诊断')
    print('=' * 56)
    print('Python: %s @ %s' % (platform.python_version(), sys.executable))
    print('系统: %s %s' % (platform.system(), platform.release()))
    print('工作目录: %s' % os.getcwd())

    print('\n[依赖]')
    mods = [('numpy', 'numpy'), ('mujoco', 'mujoco'), ('serial', 'pyserial'),
            ('websockets', 'websockets'), ('matplotlib', 'matplotlib'),
            ('cv2', 'opencv-python-headless')]
    for mod, pkg in mods:
        try:
            m = __import__(mod)
            print('  ✅ %-12s %s' % (pkg, getattr(m, '__version__', '?')))
        except ImportError:
            print('  ❌ %-12s 未安装 (pip install %s)' % (pkg, pkg))

    print('\n[串口设备]')
    import glob
    ports = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*') +
                   glob.glob('/dev/cu.usbmodem*'))
    print('  %s' % (ports if ports else '未检测到（实机模式需要）'))

    print('\n[ADB 设备]')
    try:
        r = subprocess.run(['adb', 'devices'], capture_output=True, text=True, timeout=5)
        devs = [l.split()[0] for l in r.stdout.splitlines()
                if l.strip() and 'device' in l and not l.startswith('List')]
        print('  %s' % (devs if devs else '未连接 RK3566'))
    except FileNotFoundError:
        print('  未安装 adb')

    print('\n[GitHub DNS]')
    try:
        ip = socket.gethostbyname('github.com')
        print('  github.com -> %s' % ip)
        # 测试实际可达性
        r = subprocess.run(['curl', '-sI', '--max-time', '5', 'https://github.com'],
                           capture_output=True, text=True, timeout=8)
        print('  HTTPS: %s' % ('OK' if r.returncode == 0 else '不通'))
        if r.returncode != 0:
            print('  修复: 见 HANDOFF.md（写入 20.205.243.166 到 /etc/hosts）')
    except Exception as e:
        print('  查询失败: %s' % e)

    print('\n[核心文件]')
    for f in ['host_app.py', 'motion_pipeline.py', 'run.sh', 'PROTOCOL.md']:
        print('  %s %s' % ('✅' if os.path.exists(os.path.join(ROOT, f)) else '❌', f))
    print('=' * 56)


if __name__ == '__main__':
    main()
