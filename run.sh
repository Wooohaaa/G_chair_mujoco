#!/usr/bin/env bash
# G_chair 一键启动脚本
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
PY="$HERE/G_chair_mujoco/.venv/bin/python"
PIO="$HERE/G_chair_mujoco/.venv/bin/pio"

usage() {
  echo "用法: ./run.sh <命令> [参数]"
  echo ""
  echo "  host-sim     启动上位机(仿真模式，MuJoCo)  [默认 --mock 可加在后面]"
  echo "  host-real    启动上位机(实机模式，USB串口)"
  echo "  host-wifi    启动上位机(实机模式，WiFi UDP)"
  echo "  panel        启动独立调试面板(手动姿态测试)"
  echo "  test         运行全链路回归测试(19项)"
  echo "  test-robust  运行健壮性测试(18项: 断流/异常/边界/长跑/掉线/停滞)"
  echo "  test-proto   运行协议/换算测试(31项: 中位精度/帧格式/状态帧兼容/日志分析)"
  echo "  test-web     运行 Web/WS 接口测试(6项: 手动/调参/录制/WiFi状态帧)"
  echo "  verify       一键全量验证(测试+自检+校验+三版固件编译)"
  echo "  report       全量验证并存成带时间戳的报告文件"
  echo "  doctor       环境诊断(依赖/串口/ADB/GitHub DNS)"
  echo "  fwsim        仿真 ESP32 固件步进逻辑(上真实电机前验证时序)"
  echo "  fullchain    全链路数值仿真(遥测→映射→16bit→固件→MuJoCo)"
  echo "  preflight    上实机前 GO/NO-GO 检查汇总 + 人工清单"
  echo "  deploy-board 推送最新视觉节点到 RK3566 (需 ADB 连接)"
  echo "  record       录制原始 UDP 遥测到 CSV (第2参数为秒数, 默认60)"
  echo "  analyze      分析录制日志, 给调参建议 (第2参数为csv路径)"
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
  test-proto)
    exec "$PY" "$HERE/tests/test_protocol.py"
    ;;
  test-web)
    exec "$PY" "$HERE/tests/test_web.py"
    ;;
  record)
    exec "$PY" "$HERE/G_chair_mujoco/capture_raw.py" "${1:-60}" "${2:-}"
    ;;
  analyze)
    exec "$PY" "$HERE/tools/analyze_run.py" "$@"
    ;;
  fwsim)
    exec "$PY" "$HERE/G_chair_esp32/tests/firmware_sim.py" "$@"
    ;;
  fullchain)
    exec "$PY" "$HERE/tools/full_chain_sim.py" "$@"
    ;;
  preflight)
    exec "$PY" "$HERE/tools/preflight.py"
    ;;
  report)
    exec "$PY" "$HERE/tools/save_report.py"
    ;;
  doctor)
    exec "$PY" "$HERE/tools/doctor.py"
    ;;
  deploy-board)
    exec bash "$HERE/deploy_board.sh"
    ;;
  verify)
    echo "===== 全量验证 ====="
    echo "[链路测试]"; "$PY" "$HERE/tests/test_chain.py" | tail -3
    echo "[健壮性测试]"; "$PY" "$HERE/tests/test_robustness.py" | tail -3
    echo "[协议换算测试]"; "$PY" "$HERE/tests/test_protocol.py" | tail -3
    echo "[Web接口测试]"; "$PY" "$HERE/tests/test_web.py" | tail -3
    echo "[上位机自检]"; "$PY" "$HERE/host_app.py" --check
    echo "[固件逻辑仿真]"; "$PY" "$HERE/G_chair_esp32/tests/firmware_sim.py" | tail -4
    echo "[MuJoCo校验]"; (cd "$HERE/G_chair_mujoco" && "$PY" sim.py --validate 2>&1 | grep "结论:")
    echo "[固件编译]"; (cd "$HERE/G_chair_esp32" && "$PIO" run -e esp32-s3-devkitc-1 2>&1 | grep -o SUCCESS | tail -1 && "$PIO" run -e esp32-s3-wifi 2>&1 | grep -o SUCCESS | tail -1 && "$PIO" run -e esp32-s3-estop 2>&1 | grep -o SUCCESS | tail -1)
    echo "[Git同步]"; (cd "$HERE" && git status -sb | head -1)
    echo "===== 完成 ====="
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
