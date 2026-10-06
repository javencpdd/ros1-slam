#!/usr/bin/env bash
# m_slice.sh — 按 Z 轴把点云切成水平断面条（逐层查看：地面层/狗身层/天花板管道层）
# 超参:
#   FILE=xxx            图名（不带 .pcd；缺省=自动选 data/maps 最新一张）
#   THICK=0.5           每片厚度（米）
#   ZMIN/ZMAX           切片范围（缺省=自动按点云实际 z 范围外扩 0.5m）
#   OUTDIR=slices       输出子目录（data/maps/<OUTDIR>/）
# 用法: bash m_slice.sh                 → 全高程 0.5m 一片
#       FILE=lite3_ceshi1006 THICK=0.3 bash m_slice.sh
source "$(dirname "$0")/m_env.sh"
FILE=${FILE:-}
if [ -z "$FILE" ]; then
  FILE=$(ls -t /home/ros1-slam/data/maps/*.pcd 2>/dev/null | head -1 | xargs -r basename | sed 's/\.pcd$//')
  echo "[m_slice] 未指定 FILE → 自动选最新: $FILE"
fi
THICK=${THICK:-0.5}
OUTDIR=${OUTDIR:-slices}
exec python3 /home/ros1-slam/scripts/mapping/pcd_slice.py "/home/ros1-slam/data/maps/${FILE}.pcd" "$THICK" "$ZMIN" "$ZMAX" "/home/ros1-slam/data/maps/${OUTDIR}" "${FILE}"
