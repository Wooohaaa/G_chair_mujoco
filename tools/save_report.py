#!/usr/bin/env python3
"""save_report.py — 运行全量验证并把证据存成带时间戳的报告文件。

运行: ./.venv/bin/python tools/save_report.py
输出: output/reports/report_YYYYMMDD_HHMMSS.txt
"""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')


def main():
    out_dir = os.path.join(ROOT, 'output', 'reports')
    os.makedirs(out_dir, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    path = os.path.join(out_dir, 'report_%s.txt' % stamp)

    r = subprocess.run(['bash', os.path.join(ROOT, 'run.sh'), 'verify'],
                       capture_output=True, text=True, cwd=ROOT, timeout=600)
    content = ('G_chair 全量验证报告\n时间: %s\n%s\n'
               '退出码: %d\n' % (time.strftime('%Y-%m-%d %H:%M:%S'),
                               '=' * 50, r.returncode))
    content += r.stdout + r.stderr
    with open(path, 'w') as f:
        f.write(content)
    print(content[-800:])
    print('报告已保存: %s' % path)
    return r.returncode


if __name__ == '__main__':
    sys.exit(main())
