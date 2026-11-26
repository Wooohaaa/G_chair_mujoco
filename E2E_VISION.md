# 端到端验证：视觉节点 → 上位机 → MuJoCo / 实机

验证"游戏画面 → 视觉推算 → 体感平台"这条备选链路（当游戏无遥测接口时用）。
GTA5 有 mod 走 UDP，不需要视觉；但本链路证明：将来支持无遥测游戏时，
视觉节点可直接替换 GTA5 mod，上位机和主控都不用改。

```
摄像头/HDMI → 视觉节点(screen_telemetry.py, 光流) → UDP 25浮点 :4124
            → host_app.py → MotionPipeline → MuJoCo / ESP32
```

## 本机端到端验证（2026-11-26）

用 PC 跑视觉节点的 `--mock`（合成路面）模拟 RK3566：

| 项 | 结果 |
|---|---|
| 视觉节点 | 20Hz 发 UDP 到 4124 ✓ |
| 上位机接收 | 251 帧、0 断流 ✓ |
| 缸位移跟随 | `[-45.4,-26.5,1.2,27.3,-1.0,-40.3] mm` ✓ |

**结论**：视觉节点输出的 25 浮点格式与 GTA5 mod 完全兼容，上位机无需任何改动即可接收。

## RK3566 实机部署

板子通过 ADB 连接（见 `G_chair_vision/README.md`），运行：

```bash
adb shell 'python3 /root/gchair_vision/screen_telemetry.py \
    --source /dev/video9 --ip <上位机IP> --port 4124'
```

然后上位机正常启动 `./run.sh host-sim` 或 `host-real`，自动接收视觉数据。

> 注意：RK3566 纯光流约 13.7 FPS（见 `G_chair_vision/README.md` 性能表），
> 上位机按 50Hz 取最新帧，不影响控制稳定性，但体感细腻度低于游戏遥测。

## 与 GTA5 遥测对比

| | GTA5 mod | 视觉节点 |
|---|---|---|
| 数据源 | 游戏内部状态（精确） | 画面光流推算（噪声大） |
| 帧率 | 50Hz+ | ~14Hz (RK3566) |
| 左右对称 | ✅ 修好 | ✅ 修好 |
| 适用 | GTA5 | 任何能采集画面的游戏 |

两者**输出格式相同**，上位机一视同仁。
