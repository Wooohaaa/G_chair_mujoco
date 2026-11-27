#!/usr/bin/env python3
"""抓真实 UDP 遥测原始 25 通道到 CSV，用于诊断左右转符号。"""
import os, sys, time
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'G_chair_sim'))
from udp_telemetry import UdpTelemetryReceiver
from config import TELEMETRY_CHANNELS

rx = UdpTelemetryReceiver('0.0.0.0', 4124)
rx.start()
print('监听 4124，等待数据...', flush=True)
t0 = time.time()
while time.time() - t0 < 10 and rx.get() is None:
    time.sleep(0.02)
if rx.get() is None:
    print('无数据'); rx.stop(); raise SystemExit

rows = []
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
print('开始抓 %.0fs，请在游戏里做左右转弯...' % dur, flush=True)
t0 = time.time()
while time.time() - t0 < dur:
    item = rx.get()
    if item:
        tele, _ = item
        rows.append([tele.get(k, 0.0) for k in TELEMETRY_CHANNELS])
    time.sleep(0.01)
rx.stop()

# 第二个参数可指定输出路径，避免覆盖基准数据集 raw_telemetry.csv
out = (sys.argv[2] if len(sys.argv) > 2 else
       os.path.join(HERE, 'output', 'raw_telemetry.csv'))
a = np.array(rows)
os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
np.savetxt(out, a, delimiter=',', header=','.join(TELEMETRY_CHANNELS), comments='')
print('保存 %s (%d 帧)' % (out, len(rows)))

if len(a):
    print('\n=== 各通道范围 ===')
    for i, k in enumerate(TELEMETRY_CHANNELS[:9]):
        print('  %-10s min %+7.3f  max %+7.3f  正负都有:%s' %
              (k, a[:, i].min(), a[:, i].max(),
               'YES' if a[:, i].min() < -0.01 and a[:, i].max() > 0.01 else 'NO'))
