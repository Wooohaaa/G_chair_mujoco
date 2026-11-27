#!/usr/bin/env bash
# 把最新的视觉节点文件推送到 RK3566（泰山派，ADB 连接）
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
VISION="$HERE/G_chair_vision"
DEST=/root/gchair_vision

echo "=== 检查 ADB 设备 ==="
DEV=$(adb devices | awk 'NR>1 && $2=="device" {print $1; exit}')
if [ -z "$DEV" ]; then
  echo "未检测到 ADB 设备。请连接 RK3566 后重试。"
  exit 1
fi
echo "设备: $DEV"

echo "=== 创建目标目录 ==="
adb shell "mkdir -p $DEST"

echo "=== 推送文件 ==="
for f in screen_telemetry.py benchmark.py analyze_video.py analyze_paired.py \
         record_paired.py latency_test.py udp_check.py requirements.txt; do
  if [ -f "$VISION/$f" ]; then
    adb push "$VISION/$f" "$DEST/$f"
  fi
done
# config.py 提供 25 通道定义
adb push "$HERE/G_chair_sim/config.py" "$DEST/config.py"

echo "=== 推送完成 ==="
adb shell "ls -la $DEST | head -15"
echo ""
echo "在板子上运行（示例）："
echo "  adb shell 'cd $DEST && python3 screen_telemetry.py --source /dev/video9 --ip <上位机IP> --port 4124'"
echo "  快速模式: 加 --fast（256x144，约24FPS，需重新标定增益）"
echo "  性能基准: python3 benchmark.py --mock --frames 200 [--fast]"
