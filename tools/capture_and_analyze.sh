#!/usr/bin/env bash
# 一键：录制原始遥测 → 回放通过映射管线生成运行日志 → 立即分析
# 用法: ./tools/capture_and_analyze.sh [秒数,默认60] [输出名,默认session_时间戳]
set -e
HERE="$(cd "$(dirname "$0")/.." && pwd)"
cd "$HERE"

DUR="${1:-60}"
NAME="${2:-session_$(date +%Y%m%d_%H%M%S)}"
RAW="output/${NAME}_raw.csv"
LOG="output/${NAME}_log.csv"

echo "=== 录制 ${DUR}s 原始遥测到 $RAW ==="
./run.sh record "$DUR" "$RAW"

echo ""
echo "=== 回放通过映射管线 → $LOG ==="
./run.sh replay-analyze "$RAW" "$LOG"

echo ""
echo "=== 分析 ==="
./run.sh analyze "$LOG"

echo ""
echo "下一步："
echo "  调参后重录: ./tools/capture_and_analyze.sh $DUR ${NAME}_tuned"
echo "  对比:       ./run.sh analyze --compare $LOG output/${NAME}_tuned_log.csv"
