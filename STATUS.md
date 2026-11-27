# 工程状态快照（2026-11-27 20:09 更新）

> 本文记录"现在到底做到哪了、哪些验证过、哪些必须实物实测、下一步做什么"。
> 系统全貌见 `SYSTEM_OVERVIEW.md`；快速开始见 `run.sh --help`。

---

## 一、已完成并验证（有证据）

| 项 | 状态 | 证据 |
|---|---|---|
| MuJoCo 模型与机构校验 | ✅ | 条件数 7.73、可达率 500/500、静态跟踪 RMS 1.60mm（`G_chair_mujoco/README.md`） |
| 仿真=实机统一映射 | ✅ | 共用 `MotionPipeline` + 实物几何，回放跟踪 RMS 1.65mm（旧几何时 5.41mm） |
| 左右转对称修复 | ✅ | roll 幅值比 0.43→1.08，`turn_compare_FIXED.png` |
| 自适应限速 | ✅ | 峰值 1507→110mm/s，0 帧超主控 128 能力 |
| 中位换算精确化 | ✅ | ±95mm 误差 <0.001mm（原 1.7 除数有 +0.37mm 偏置） |
| ESP32-S3 固件 | ✅ 编译通过 | 串口/WiFi/急停三环境均 SUCCESS；6路RMT、限位、看门狗、状态回传 |
| 一键上位机 | ✅ 实测 | `host_app.py` Web 监控、在线调参（lpf/washout/限速/增益）、录制、回放均验证 |
| real+serial 完整闭环 | ✅ 实测 | 虚拟串口模拟 ESP32：209 帧、使能=1、状态回传 209 帧、6缸位移正常 |
| real+wifi 输出 | ✅ 实测 | 6 秒 245 帧 UDP，帧长 14、帧头 ffff |
| 调试面板 | ✅ 实测 | 虚拟串口闭环，帧格式正确，状态回传解析正确 |
| 全链路测试 | ✅ | `test_chain.py` 19 PASS + `test_robustness.py` 18 PASS + `test_protocol.py` 34 PASS + `test_web.py` 6 PASS |
| 全链路数值仿真 | ✅ | 16bit量化误差 0.0033mm、缸速 ≤110mm/s、MuJoCo 跟踪 RMS 1.43mm |
| 四种参数模式 | ✅ | 默认/无限速/FlyPT/无滤波 各 800 帧均有限、位移合理（限速模式 15.3mm vs 无限速 53.1mm，证明限速生效） |
| 固件 jog 命令 | ✅ | `#J0,300`=+1.5mm、`#J2,-200`=−1.0mm、超程钳位、非法轴拒绝 |
| Web 手动测试 | ✅ | 9 预设 + 自定义6缸输入，指令保持直到"恢复遥测"，实测 30.0mm 稳定保持 |
| WiFi 状态闭环 | ✅ | 固件 UDP 回传 `$` 帧，实测 60 帧/3s、状态龄 0.05s，无"未连接"误报 |
| RK3566 一键部署 | ✅ | `./run.sh deploy-board` 推送视觉节点（未接板时优雅退出） |
| 调参持久化 | ✅ | 滑块改动自动存 `output/tuning.json`，重启恢复；可一键恢复默认 |
| 视觉→上位机端到端 | ✅ 实测 | mock 视觉 UDP → 上位机 → MuJoCo，251帧0断流 |
| WiFi UDP 模式 | ✅ 编译+模拟 | 上位机 `--output wifi` 6秒发245帧；板子 WiFi 版编译通过 |
| 文档体系 | ✅ | 12+ 份 md，索引见 `SYSTEM_OVERVIEW.md` |

## 二、必须实物实测（代码无法替代）

| 项 | 为什么 | 怎么做 |
|---|---|---|
| **3.3V 直连伺服能否驱动** | 决定要不要补 74HCT245 | 按 `G_chair_esp32/POWERON_CHECKLIST.md` 单缸测试 |
| **共地** | 漏接则伺服不响应 | ESP32 GND ↔ 伺服 24V- |
| **ENA 极性** | 日鼎使能针脚高低有效需确认 | 必要时改 `config.h` 的 `ENABLE_ACTIVE_LOW` |
| **X 轴方向** | DIR 极性可能反 | 改变 `PIN_DIR` 逻辑或伺服方向参数 |
| **RMT 变速实际生效** | P0-1 已代码修，但需波形确认 | 逻辑分析仪看频率随 `|target-current|` 变化 |
| **WiFi 实际连接** | 需真实 SSID/密码和网络 | 填 `wifi_config.h` 后烧录 WiFi 版 |

## 三、明确未做（不是遗漏，是范围外）

| 项 | 说明 |
|---|---|
| 闭环反馈 | 当前开环（与原实物一致）；要闭环需加编码器接口 |
| YOLO 语义门控 | 接口预留（`YoloGate` TODO），默认纯光流。**阻塞点**：需用户实际 RKNN 模型（yolov8n.rknn）的输出张量格式与校准数据才能填 decode/NMS；不阻塞主链路 |
| 多游戏支持 | 只做 GTA5 mod；其它游戏需写各自的数据源 |
| 真实 6 轴同时带载 | 只做过单缸/模拟验证，未全负载跑过 |
| 硬件急停 GPIO | 已做可选编译（`esp32-s3-estop`），但需画板预留 |

## 四、下一步优先级（如果继续）

1. **实物单缸测试**（最高优先）——这是整个工程从"仿真正确"到"真的能动"的最后一公里。
   按 `POWERON_CHECKLIST.md` 逐项打勾。
2. 若 3.3V 不驱动 → 补 74HCT245（十几块），重测。
3. 6 缸逐轴验证后，跑 `host_app.py --mode real` 接真实上位机。
4. 用 `--replay` 回放真实数据，对比不同增益参数，调出满意体感。
5. （可选）画 PCB 时预留急停 GPIO、编码器接口、24V→5V DC-DC。

## 五、关键文件速查

```
run.sh                    一键命令入口
SYSTEM_OVERVIEW.md        系统地图
HOST_APP.md               一键上位机（监控+调参+回放）
STATUS.md                 本文：状态快照
tests/test_chain.py       19 项链路测试
tests/test_robustness.py   18 项健壮性测试
tests/test_protocol.py    34 项协议/状态帧/日志分析/常量一致性测试
tests/test_web.py        6 项 Web/WS 接口+WiFi状态帧测试
tools/preflight.py        上实机前 GO/NO-GO 汇总
tools/full_chain_sim.py   全链路数值仿真
tools/analyze_run.py      录制日志分析 + --compare 调参前后对比
tools/save_report.py      全量验证 + 时间戳报告
tools/doctor.py           环境诊断(依赖/串口/ADB/GitHub DNS)
G_chair_esp32/tests/firmware_sim.py  固件步进逻辑仿真
PROTOCOL.md               线格式/单位/时序唯一权威参考
motion_pipeline.py        共用映射核心（set_params 可在线调参）
G_chair_esp32/            固件 + 硬件文档（INTEGRATION/BOARD_SPEC/MINIMAL_SETUP/
                          POWERON_CHECKLIST/WIFI_MODE/REPLACING_DUAL_MEGA/
                          SCHEDULING_ANALYSIS/WIRING_COMPARISON）
G_chair_vision/           RK3566 视觉节点 + 性能基准
G_chair_gta5/             GTA5 mod
```

## 六、一句话总结

**仿真、算法、通信、固件、上位机、测试全部完成并验证；只差"把线接到真实电缸上"这一步实物测试。**
