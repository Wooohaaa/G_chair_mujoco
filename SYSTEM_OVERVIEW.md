# G_chair 六轴体感平台 · 系统总览

> 这是整个工程的**地图**。所有细节文档都在本文末尾索引。
> 📌 **当前状态/待实测/下一步**：`STATUS.md`

## 一、系统是什么

用一块 **ESP32-S3** 替代原工程的**两块 Mega2560 + 无源转接板**，
驱动 Stewart 六轴平台的 6 个伺服电缸，实现赛车体感座椅。

数据来自 **GTA5**（通过 ScriptHookV mod 发 UDP），上位机做"加速度→姿态→缸位移"映射，
ESP32 只负责把 6 缸位移转成精确步进脉冲（硬件 RMT，不受 WiFi/软件干扰）。

```
┌─────────────┐   UDP 25浮点   ┌──────────────────────────────────┐
│ 游戏电脑     │  :4124        │  仿真/上位机电脑  (host_app.py)    │
│ GTA5 + mod   │ ────────────→ │                                  │
│ ----------- │               │  UdpTelemetry → MotionPipeline    │
│ GchairTelemetry.cs          │  (低通→加速度映射→洗出→限速→IK)   │
└─────────────┘               │         ↓ 6缸位移(mm)              │
                              │  输出: 串口 / WiFi UDP / MuJoCo    │
                              └──────────────┬───────────────────┘
                                             │ <255><255>+6×16bit
                              ┌──────────────┴──────────────┐
                              │  ESP32-S3 主控 (单板)         │
                              │  6路RMT脉冲 + 限位 + 看门狗   │
                              └──────────────┬──────────────┘
                                             │ 差分脉冲 24V
                              ┌──────────────┴──────────────┐
                              │  6 × 日鼎伺服驱动 → 电缸      │
                              └─────────────────────────────┘
```

## 二、核心设计原则

1. **主控是执行层，不参与算法**。所有映射/滤波/IK 在上位机（`motion_pipeline.py`）。
   换板不动体感参数。
2. **仿真=实机**。两者共用 `MotionPipeline` 和同一套实物几何 `PARAMS`；
   同一遥测产生完全相同的缸位移（旧实现差 49.5mm，已修）。
3. **安全分层**。硬件急停（常闭切 24V）是主保护；固件限位、看门狗、使能互锁是第二道。
4. **精确换算**。16bit→步数用 65535/38400（修原 1.7 的 +0.37mm 偏置），
   全链路 ±95mm 误差 <0.001mm。

## 三、快速开始

```bash
cd sim
./run.sh verify                        # 一键全量验证(测试+自检+校验+三版固件编译)
./run.sh fwsim                         # 固件步进逻辑仿真(上真实电机前)
./run.sh fullchain                     # 全链路数值仿真(遥测→映射→16bit→固件→MuJoCo)
./run.sh preflight                     # 上实机前 GO/NO-GO 汇总 + 人工清单
./run.sh doctor                        # 环境诊断(依赖/串口/ADB/GitHub DNS)
./run.sh record 60 output/session.csv  # 录制真实遥测
./run.sh deploy-board                  # 推送视觉节点到 RK3566(需 ADB)
./run.sh test                          # 19 项回归测试，确认链路正常
./run.sh analyze output/host_app_log.csv  # 分析录制日志，给调参建议
./run.sh host-sim --mock               # 无硬件看 Web 界面 (http://localhost:8765)
./run.sh host-real --serial /dev/ttyUSB0   # 接真实 ESP32
./run.sh host-wifi --esp32-ip 192.168.1.50 # WiFi 直连（可选）
./run.sh build                         # 编译 ESP32 两版固件
```

## 四、目录/文件索引

### 上位机（核心）
| 文件 | 说明 |
|---|---|
| `host_app.py` / `host_app.html` | 一键上位机 + Web 监控 + 在线调参（`HOST_APP.md`） |
| `motion_pipeline.py` | 共用映射管线：低通→映射→洗出→限速→IK（`set_params` 可运行时调参） |
| `speed_limiter.py` | 自适应限速（只削尖峰、几何一致、正常段零延迟） |
| `motion_map.py` | 加速度驱动映射（roll←accel_x，修好左右对称） |
| `run.sh` | 一键命令集合 |

### 仿真
| 文件 | 说明 |
|---|---|
| `G_chair_mujoco/` | MuJoCo 模型/IK/FK/校验（`README.md`）；外观按实物增强 |
| `G_chair_vision/` | RK3566 视觉节点、视频分析、并排对比（`README.md`） |
| `output/real_gta5/` | 真实数据回放的诊断文档与曲线 |

### 实机
| 目录 | 说明 |
|---|---|
| `G_chair_gta5/` | GTA5 mod（ScriptHookV + SHVDN）发 UDP（`README.md`） |
| `G_chair_esp32/` | ESP32-S3 固件（`INTEGRATION.md` 总方案、`MINIMAL_SETUP.md` 最小配置、`POWERON_CHECKLIST.md` 上电步骤、`BOARD_SPEC.md` 画板规格、`WIFI_MODE.md` WiFi 模式） |
| `G_chair_hw/` | 串口桥接（`README.md`）、调试面板（`DEBUG_PANEL.md`） |

### 双机 / 参考
| 文件 | 说明 |
|---|---|
| `TWO_PC_SETUP.md` | 游戏机 ↔ 仿真机 网络连接 |
| `FLYPT_GAMES_REFERENCE.md` | FlyPT 支持游戏与数据来源（GTA5 不在其中；本系统不依赖 FlyPT） |
| `tests/test_chain.py` | 全链路回归测试（19 项） |

## 五、已知限制与下一步

| 项 | 状态 |
|---|---|
| 真实电机带载验证 | ⬜ 未做（按 `POWERON_CHECKLIST.md` 逐轴验证） |
| 3.3V 直连伺服是否可行 | ⬜ 待实测；不行补 74HCT245 |
| 急停 GPIO 双保险 | ⬜ 画板时预留（当前硬件急停是常闭按钮切 24V） |
| 闭环反馈（编码器） | ⬜ 未做（当前开环） |
| 滤波器增益在线调 | ⚠️ 已支持 lpf/washout/max_speed；增益常量仍需改代码 |
| YOLO 语义门控 | ⚠️ 接口预留（`YoloGate` TODO），默认纯光流 |

## 六、关键实测数据

| 指标 | 值 |
|---|---|
| 机构雅可比条件数 | 7.73（远离奇异） |
| 静态跟踪 | RMS 1.60mm / MAX 2.04mm |
| 真实数据回放跟踪 | RMS 1.65mm（旧几何时 5.41mm） |
| 缸速限制 | 峰值 1507 → 110mm/s，0 帧超主控能力 128 |
| 16bit 换算精度 | ±95mm 误差 <0.001mm |
| RK3566 视觉帧率 | ~13.7 FPS（光流占 97%）；PC ~97 FPS |
| 全链路测试 | 19 PASS / 0 FAIL |
