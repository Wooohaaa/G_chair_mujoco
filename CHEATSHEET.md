# 速查卡（打印/贴墙）

## 日常一条命令

```bash
cd /home/chiehyu/G_chair/G_chair/sim

./run.sh preflight          # 上实机前检查（全绿才接线）
./run.sh host-sim --mock    # 无硬件演示（浏览器 localhost:8765）
./run.sh host-real --serial /dev/ttyUSB0    # 接真实 ESP32
./run.sh host-wifi --esp32-ip <板子IP>      # WiFi 直连
./run.sh verify             # 全量验证（65项测试，出证据）
./run.sh report             # 同上并存成带时间戳的报告文件
./run.sh test-proto         # 协议/换算测试（中位精度）
./run.sh test-web           # Web接口测试
```

## 实机接线（只接一根先测）

| ESP32 | 2.2kΩ | 日鼎 DB25 |
|---|---|---|
| GPIO4 STEP0 | → | 8 脉冲+ |
| GPIO17 DIR0 | → | 20 方向+ |
| GPIO12 ENA | → | 2 使能 |
| GND | 直连 | 14 (24V-) **共地！** |

## 串口命令（115200 监视器）

`#T` 自检逐轴点动 ｜ `#N` 中位 ｜ `#E/#D` 使能/禁止 ｜ `#H` 帮助

## 关键数值

- 中位 16bit=32768 → 19200 步 → 0mm（误差 <0.001mm）
- 行程 ±95mm，物理半行程 96mm，0.005mm/步
- 最高缸速 128mm/s（39µs 脉冲），默认限速 110
- 上位机 50Hz，固件 4ms tick，状态回传 20Hz，看门狗 500ms

## 出问题先查

1. **完全不动** → 共地 → 使能 → 3.3V电平(补74HCT245) → 伺服参数
2. **方向反** → DIR 极性
3. **Web无数据** → `./run.sh verify` → 串口是否选对 → `--check`
4. **左右不对称** → Web 调 roll 增益，或 `./run.sh analyze output/host_app_log.csv`
5. **推送失败** → `gh api user` 检查；见 HANDOFF.md 的 DNS 修复

## 文档地图

- 总览 `SYSTEM_OVERVIEW.md` ｜ 状态 `STATUS.md` ｜ 交接 `HANDOFF.md`
- 协议 `PROTOCOL.md` ｜ 上位机 `HOST_APP.md` ｜ 变更 `CHANGELOG.md`
- 实机 `QUICKSTART.md` ｜ 固件 `G_chair_esp32/` ｜ 视觉 `G_chair_vision/`
