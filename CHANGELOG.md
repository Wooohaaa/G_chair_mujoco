# 变更日志

> 按时间倒序记录重要改动。当前状态见 `STATUS.md`，系统地图见 `SYSTEM_OVERVIEW.md`。

## 2026-11-27 晚间补充（20:35）

- 完整验证体系：`preflight`（8 项检查 / 57 测试）、`report`（时间戳证据）、
  `doctor`（环境诊断）、`test_protocol.py`（15 项）、`test_web.py`（5 项）
- 上位机：手动姿态保持 + 自定义 6 缸输入、参数持久化、6DOF 位姿显示、
  遥测停滞/主控掉线告警、WiFi UDP 状态回传闭环
- 固件：`#J<轴>,<步数>` 单轴点动命令、WiFi 模式 `$` 帧经 UDP 回传、
  `#T` 非阻塞自检、三环境编译 SUCCESS
- 视觉：`--fast` 轻量光流（PC 2× 提速）、`analyze_paired` 增益建议、一键部署脚本
- 文档：`SYSTEM_OVERVIEW` / `STATUS` / `HANDOFF` / `QUICKSTART` / `PROTOCOL` /
  `CHEATSHEET` / `CHANGELOG` 共 30 份；GitHub DNS 排障记录

## 2026-11-27（第二轮完善）

### 安全与健壮性
- 上位机新增 **ESP32 状态回传超时告警**（>1s 未更新 → Web 红色告警）
- 新增 **遥测数值停滞检测**（游戏暂停/mod 卡死 → 告警）
- 修复 **无效串口导致启动崩溃**（优雅降级 + Web 告警"未连接"）
- WiFi 模式修复"未连接"误报；新增 **WiFi UDP 状态回传闭环**（固件 `$` 帧经 UDP 回上位机）
- `host_app.py --check` 启动自检；调参参数自动持久化到 `output/tuning.json`

### Web 界面
- 新增 **手动姿态测试**：9 预设 + 自定义 6 缸输入，指令**保持**直到"恢复遥测控制"
- 显示 6DOF 位姿（升沉/roll/pitch/yaw）、状态龄、控制源
- 在线调参扩展到增益（roll/pitch/yaw/sway），重启后保留

### 工具链
- `tools/full_chain_sim.py`：全链路数值仿真（遥测→映射→16bit→固件→MuJoCo），
  实测量化误差 0.0033mm、缸速 ≤110mm/s、跟踪 RMS 1.43mm
- `tools/preflight.py`：上实机前 GO/NO-GO 汇总（一条命令输出全部证据）
- `tools/analyze_run.py --compare`：调参前后并排对比
- `run.sh` 新增 `verify` / `fwsim` / `fullchain` / `preflight` / `record` / `deploy-board` 等命令
- `deploy_board.sh`：一键推送视觉节点到 RK3566

### 视觉节点
- `screen_telemetry.py` 新增 `--fast`（256×144 + 轻量 Farneback），PC 实测 2× 提速（100→208 FPS）
- `analyze_paired.py` 新增可执行增益重标定建议（相关性足够高时才给）
- 修复 `analyze_paired.py` 相对路径解析

## 2026-11-26（第一轮完善）

### 核心修复
- **统一仿真与实机 IK 几何**：`MotionPipeline` 改用 `PARAMS`（实物几何），
  修复同一位姿两套几何差 49.5mm 的问题；新增 `sim.command_displacement()`
- **精确中位换算**：除数从 `1.7` 改为 `65535/38400=1.7078125`，
  修复 +0.37mm 中位偏置与 1mm 满量程误差（全链路 <0.001mm）
- **左右转不对称修复**：`motion_map.py` 改为加速度驱动映射
  （roll←accel_x），幅值比 0.43→1.08
- **自适应限速器**：`speed_limiter.py` 只削尖峰、几何一致、正常段零延迟，
  峰值缸速 1507→110mm/s、0 帧超主控能力
- **输入净化**：NaN/Inf 不再永久污染低通/洗出状态
- **断流 reset**：断流时重置滤波状态，避免 washout 反向瞬态

### 固件
- ESP32-S3 单板固件：6 路 RMT、限位、看门狗、使能互锁、状态 `$` 帧
- 三种编译配置：串口版 / WiFi 版 / 急停 GPIO 版（均编译 SUCCESS）
- 固件自检命令 `#T/#N/#E/#D/#H`；`#T` 改为非阻塞状态机（原 `delay` 阻塞脉冲）
- RMT 周期更新改为 stop→写→start（避免 loop 模式 wait=true 死锁）
- 状态回传改为 snprintf 一次 `Serial.write`（减少阻塞）

### 上位机
- `host_app.py` 一键上位机（real/sim 模式、Web 监控、在线调参、录制、回放）
- `motion_pipeline.py` 共用映射管线（set_params 可运行时调参）
- 修复 CSV 表头字段数不匹配（14 vs 19）
- 上位机断流/掉线/停滞三重告警

### 测试与文档
- `tests/test_chain.py`（19 项）、`tests/test_robustness.py`（18 项）
- 27+ 份文档：SYSTEM_OVERVIEW / STATUS / HANDOFF / QUICKSTART /
  PROTOCOL / HOST_APP / FLYPT_GAMES_REFERENCE / E2E_VISION / 硬件文档 7 份
