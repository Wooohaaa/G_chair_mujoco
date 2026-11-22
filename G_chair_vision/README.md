# G_chair 视觉节点：游戏画面 → 体感运动数据（RK3566）

## 能做什么 / 不能做什么（先看这里）

用 HDMI 采集卡（如 `dev/video9`）把游戏画面送进 RK3566，程序从画面里估算车辆运动，
输出与工程完全相同的 **25 浮点 UDP 遥测**，直接喂给现有的 `sim_udp.py`，再驱动两块 Arduino。

**必须理解的限制**：

1. **YOLO 检测"车"本身拿不到加速度。** 赛车游戏镜头跟随车辆，车在画面里基本居中，
   它的框位置/大小几乎不反映加减速。所以本方案的主信号是**路面光流**（背景相对运动），
   YOLO(RKNN) 只是可选的语义门控（判断是否在游戏画面、前方车辆逼近产生冲击）。
2. **精度明显低于游戏遥测。** 实测本机模拟：遥测链路位置 RMS 约 1.5~1.8mm，
   视觉链路约 7mm。这是信息源本身的差距，不是代码 bug。
3. **帧率低。** RK3566 上 YOLOv8n INT8 纯推理官方 35 FPS，含前后处理实际 12~18 FPS；
   本程序发送默认 20Hz（纯光流模式可更高）。而遥测是 50Hz。
4. **需要标定。** `SPEED_SCALE` 等增益取决于采集卡分辨率、游戏 FOV、镜头畸变，
   换成你的真实游戏画面后必须重新调（下面有标定方法）。

**结论**：这是给"没有遥测接口的游戏"用的降级方案。GTA5 能用 SHVDN 遥测就优先用遥测，
视觉方案作为备用/补充。

## 用真实 GTA5 录像标定的发现（重要）

工程根目录的 `GTA5_drive_video.mp4`（1280×720 / 30fps / 13分49秒，第三人称追尾视角、
无 HUD）用来做了标定，`analyze_video.py` 输出 `video_flow.png` / `video_flow.csv`，
主程序输出 `video_demo_result.png` / `video_telemetry.csv`。

**三个必须修正的问题**：

1. **ROI 不能包含中间的车。** 车身随悬挂晃动会污染光流。已改成 `sides` ROI：
   画面下方 + 排除中间 13% 宽度。实测 p75 纵向光流从 0.537（含车）提升到 0.968。
2. **不能用 median。** GTA5 沥青几乎无纹理，光流中位数被 0 主导（实测 0.003），
   只有车道线/路缘等有纹理像素有有效运动（75 分位 0.5~1.0）。改用 **p75**。
3. **必须先平滑速度再差分。** 直接对 p75 速度差分会把车道线经过的节奏放大成
   3~4 秒周期的伪加速度振荡。现在 `speed_lp` 用 2.0s 时间常数，再差分、再限幅。

**诚实结论**：即使修正后，画面推导的加速度仍比游戏遥测噪声大得多
（`video_demo_result.png` 可见 surge 在 ±0.2~0.8G 波动）。这是信息源上限，不是 bug。
整条链路实测可用：真实视频→UDP→MuJoCo 收包 1328 / 坏包 0 / 无超程，
缸位移最大 58mm；但位置跟踪 RMS 约 5mm（遥测方案约 1.5mm）。

### 复现命令

```bash
cd sim/G_chair_vision
# ROI 对比分析（约 2~3 分钟）
../G_chair_mujoco/.venv/bin/python analyze_video.py \
    --video ../GTA5_drive_video.mp4 --start 25 --duration 90 --step 3
# 主程序跑视频并导出遥测
../G_chair_mujoco/.venv/bin/python screen_telemetry.py \
    --source ../GTA5_drive_video.mp4 --csv video_telemetry.csv --duration 45
```

## 本机自测（不需要摄像头和 RKNN）

```bash
cd sim/G_chair_vision
../G_chair_mujoco/.venv/bin/python screen_telemetry.py --mock --ip 127.0.0.1 --duration 20
# 另一个终端开 MuJoCo 接收
cd ../G_chair_mujoco && ./.venv/bin/python sim_udp.py --realtime
```

`--mock` 生成向前滚动的路面动画（加速→匀速→急刹循环）。已实测：
收包 320/0 坏包、无超程、平台随加减速动作，位置跟踪 RMS 7.08mm。

## RK3566（泰山派）部署

### 0. 通过 ADB 连接板子（立创泰山派已在板端开启 ADB）

电脑端装 adb 后：

```bash
sudo apt install adb
adb devices          # 应看到设备号，状态 device
adb push G_chair_vision /root/gchair_vision
adb push ../G_chair_sim/config.py /root/gchair_vision/config.py   # 25通道定义，必须
adb shell
```

### 0.1 板子没联网？用 ADB + TCP（推荐，已实测）

板子和电脑不同网段、ping 不通时，不需要 UART。ADB 只转发 TCP，所以把视觉节点
切成 TCP server，电脑用 `adb forward` 连过去。

**板子**（先起 server）:

```bash
adb shell 'python3 -u /root/gchair_vision/screen_telemetry.py \
    --mock --transport tcp --port 4124 --duration 25 --hz 20'
```

**电脑**（另一个终端）:

```bash
adb forward tcp:4124 tcp:4124
cd sim/G_chair_mujoco
./.venv/bin/python sim_udp.py --tcp --host 127.0.0.1 --port 4124 --realtime
# 结束后: adb forward --remove-all
```

实测（立创泰山派 / Ubuntu 20.04 aarch64 / Python 3.8 / cv2 4.13 / rknnlite）：
**板子→ADB→电脑→MuJoCo 收包 241、坏包 0、丢帧 0、无超程**，平台正确跟随加减速。
若只想本地验证板子发包，去掉 `--transport tcp` 用 UDP 自发自收：

```bash
adb shell 'python3 /root/gchair_vision/udp_check.py 4124 9 > /tmp/check.log 2>&1 & \
  sleep 1; python3 -u /root/gchair_vision/screen_telemetry.py \
  --mock --ip 127.0.0.1 --duration 7 --hz 20; wait; cat /tmp/check.log'
```

纯 UDP 本地实测：收包 140 帧、25 通道、加速度与角速度均为非零真实数据。

### 1. 环境

```bash
sudo apt install python3-opencv python3-numpy
# RKNN 运行时（模型推理用）
pip install rknn-toolkit-lite2
```

确认采集卡节点：`v4l2-ctl --list-devices`，通常是 `/dev/video9` 之类。

### 2. 纯光流模式（先跑通这个）

不需要 YOLO 模型，帧率更高、更稳：

```bash
python3 screen_telemetry.py \
    --source /dev/video9 \
    --ip 10.148.82.176 --port 4124 \
    --hz 20 --debug-dir ./debug
```

`--debug-dir` 会每 10 帧存一张处理图，先确认 ROI 框住的是路面而不是菜单/HUD。

### 3. 打开 YOLO 语义门控（可选）

```bash
python3 screen_telemetry.py --source /dev/video9 --yolo --model yolov8n.rknn ...
```

模型要求：
- `yolov8n.rknn`，**必须 INT8**（FP16 在 RK3566 只有 3~5 FPS，不可用）
- 固定输入 320 或 640，opset 11~13 导出
- 用 rknn_model_zoo 的 yolov8 例程转换，校准集要用你真实游戏画面
- `YoloGate.analyze()` 里的解码/NMS 是 TODO，需要按 rknn_model_zoo 后处理填

### 4. 接入实机

仿真机正常跑：

```bash
cd sim/G_chair_mujoco
./.venv/bin/python sim_udp.py --realtime
# 要驱动电缸再开一个终端
cd ../G_chair_hw
../G_chair_mujoco/.venv/bin/python -u serial_bridge.py \
    --port1 /dev/ttyUSB0 --port2 /dev/ttyUSB1 --verbose
```

RK3566 只负责发 UDP，步进脉冲仍由两块 Arduino 产生（Linux 不是实时系统，
不能直接用 RK3566 的 GPIO 模拟 15kHz STEP 脉冲，会丢步）。

## 标定方法

1. 用 `--debug-dir` 保存处理帧，确认 ROI 只覆盖路面（避开 HUD、菜单、天空）。
2. 在游戏里做一段已知动作（直线加速→匀速→急刹），记录程序打印的"速度代理"曲线。
3. 调整顶部参数：
   - `SPEED_SCALE`：让速度代理在量级上合理（建议目标范围 0~10）
   - `ACCEL_CLIP`：目前 ±1.5G，如果频繁顶到限幅就调大 `LP_TAU` 或减小增益
   - `LP_TAU`（0.20s）：调大更平滑但延迟更高
   - `WASHOUT_TAU`（2.5s）：调小则恒速时更快回中位，调大则保持体感更久
4. 用 `output/` 里的跟踪曲线对比命令与实际位姿，确认没有异常抖动。

## 输出格式

与 `config.py` 的 `TELEMETRY_CHANNELS` 完全一致（25 个空格分隔浮点数）：
用到的通道是 `accel_x/z/y`（横向/纵向/垂直加速度 G）、`rot_x/y/z`（角速度 rad/s）、
`speed_x/z`、`time`。仿真端零改动。

## 文件

```
sim/G_chair_vision/
├── screen_telemetry.py   视觉节点主程序（光流+可选RKNN+UDP）
└── README.md             本文档
```
