#!/usr/bin/env bash
# G_chair 一键启动脚本
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
PY="$HERE/G_chair_mujoco/.venv/bin/python"

usage() {
  echo "用法: ./run.sh <命令> [参数]"
  echo ""
  echo "  host-sim     启动上位机(仿真模式，MuJoCo)  [默认 --mock 可加在后面]"
  echo "  host-real    启动上位机(实机模式，USB串口)"
  echo "  host-wifi    启动上位机(实机模式，WiFi UDP)"
  echo "  panel        启动独立调试面板(手动姿态测试)"
  echo "  test         运行全链路回归测试(19项)"
  echo "  test-robust  运行健壮性测试(12项: 断流/异常/边界/长跑)"
  echo "  build        编译 ESP32 固件(串口/WiFi/急停三版)"
  echo "  validate     运行 MuJoCo 静态校验"
  echo "  simulate     用真实视频录制 MuJoCo 对比视频"
  echo ""
  echo "示例:"
  echo "  ./run.sh host-sim --mock          # 无硬件看界面"
  echo "  ./run.sh host-real --serial /dev/ttyUSB0"
  echo "  ./run.sh host-wifi --esp32-ip 192.168.1.50"
}

CMD="${1:-}"
shift || true

case "$CMD" in
  host-sim)
    exec "$PY" "$HERE/host_app.py" --mode sim "$@"
    ;;
  host-real)
    exec "$PY" "$HERE/host_app.py" --mode real --output serial "$@"
    ;;
  host-wifi)
    exec "$PY" "$HERE/host_app.py" --mode real --output wifi "$@"
    ;;
  panel)
    exec "$PY" "$HERE/G_chair_hw/debug_panel.py" "$@"
    ;;
  test)
    exec "$PY" "$HERE/tests/test_chain.py"
    ;;
  test-robust)
    exec "$PY" "$HERE/tests/test_robustness.py"
    ;;
  build)
    PIO="$HERE/G_chair_mujoco/.venv/bin/pio"
    (cd "$HERE/G_chair_esp32" && "$PIO" run && "$PIO" run -e esp32-s3-wifi && "$PIO" run -e esp32-s3-estop)
    ;;
  validate)
    (cd "$HERE/G_chair_mujoco" && "$PY" sim.py --validate)
    ;;
  simulate)
    (cd "$HERE/G_chair_vision" && "$PY" video_mujoco_compare.py "$@")
    ;;
  ""|-h|--help)
    usage
    ;;
  *)
    echo "未知命令: $CMD"; usage; exit 1
    ;;
esac
