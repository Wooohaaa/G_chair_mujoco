# ESP32-S3 WiFi UDP 直连模式（可选）

让 ESP32 直接通过 WiFi 接收上位机的 6 轴指令帧，**甩掉 USB 串口线**。
RMT 脉冲仍由硬件产生，WiFi 接收不影响运动精度（这正是双核+RMT 的价值）。

> 默认仍是串口模式（`pio run`）；WiFi 模式是可选编译开关，不影响现有稳定链路。

## 1. 配置 WiFi

编辑 `include/wifi_config.h`：

```c
#define WIFI_SSID       "你的WiFi名"
#define WIFI_PASSWORD   "你的密码"
#define WIFI_UDP_PORT   4125
```

> ⚠️ 不要把真实密码提交到公开仓库。建议生成 `wifi_config_local.h`（加入 .gitignore）
> 并在编译 flag 里 `-include wifi_config_local.h` 覆盖。

## 2. 编译烧录 WiFi 版

```bash
cd sim/G_chair_esp32
./.venv/bin/pio run -e esp32-s3-wifi -t upload
```

烧录后串口监视器会打印 ESP32 的 IP，例如 `[wifi] 已连接 IP=192.168.1.50`。
两版占用（实测）：

| 版本 | Flash | RAM |
|---|---|---|
| 串口版 | 8.0% | 5.9% |
| WiFi 版 | 20.6% | 13.8% |

## 3. 上位机切到 WiFi 输出

```bash
cd sim
./G_chair_mujoco/.venv/bin/python host_app.py --mode real \
    --output wifi --esp32-ip 192.168.1.50 --esp32-udp 4125
```

上位机把 `<255><255>+6×16bit` 帧用 UDP 发到 ESP32 的 4125 端口。
ESP32 收到后逐字节喂给同一个 `parse_byte()`，复用全部映射/限位/看门狗逻辑。

## 4. 与串口模式的异同

| | 串口模式 | WiFi 模式 |
|---|---|---|
| 物理连接 | USB 线 | 无线（同一 WiFi） |
| 指令帧 | `<255><255>+6×16bit` | **完全相同**（只是走 UDP） |
| 解析/映射/脉冲 | 同一套代码 | 同一套代码 |
| 状态回传 | USB 串口 `$` 帧 | 同一 UDP 端口回传 `$` 帧，上位机显示状态龄 |
| 依赖 | pyserial | 网络连通 |

## 5. 验证记录（2026-11-26 ~ 27）

- 三个 PlatformIO 环境均编译 **SUCCESS**（串口版 / WiFi 版 / 急停版）
- 上位机 `--output wifi` + mock 数据实测：6 秒发出 **245 帧**，每帧 **14 字节**、帧头 **ff ff** ✓
- 状态回传闭环（2026-11-27）：固件在 WiFi 模式把 `$` 帧经 UDP 发回上位机，
  模拟实测 60 帧/3s、状态龄 0.05s；上位机 Web 显示限位/看门狗，无"未连接"误报 ✓

## 6. 注意事项

1. 上位机与 ESP32 必须在同一网段；IP 以串口监视器打印的为准。
2. WiFi 掉线时上位机仍持续发（UDP 无连接），ESP32 看门狗 500ms 无帧自动回中位。
3. 若 WiFi 环境不稳定，优先用串口模式（更可靠）。
4. 上位机在 `esp32_udp` 端口同时收 ESP32 的 `$` 状态帧；
   实测 60 帧/3s、状态龄 0.05s，限位/看门狗/断线告警在 Web 界面可见。
