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
