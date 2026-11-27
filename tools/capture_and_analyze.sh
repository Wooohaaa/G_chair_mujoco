#!/usr/bin/env bash
# 一键：录制一段真实遥测 → 立即分析，给出调参建议
# 用法: ./tools/capture_and_analyze.sh [秒数,默认60] [输出名,默认session_时间戳]
set -e
HERE="$(cd "$(dirname "$0")/.." && pwd)"
cd "$HERE"

DUR="${1:-60}"
NAME="${2:-session_$(date +%Y%m%d_%H%M%S)}"
OUT="output/${NAME}.csv"

echo "=== 录制 ${DUR}s 遥测到 $OUT ==="
./run.sh record "$DUR" "$OUT"

echo ""
echo "=== 分析 ==="
./run.sh analyze "$OUT"

echo ""
echo "下一步："
echo "  调参后重录: ./tools/capture_and_analyze.sh $DUR ${NAME}_tuned"
echo "  对比:       ./run.sh analyze --compare $OUT output/${NAME}_tuned.csv"
