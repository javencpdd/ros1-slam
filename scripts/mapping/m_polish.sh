#!/usr/bin/env bash
# m_polish.sh — PCD 抛光：去地板/房顶 + 自适应降体素 + 离群去噪（三合一）
# 超参（详见 pcd_polish.py 头注）:
#   FILE=       图名（缺省自动选最新）
#   OUT=        输出名（缺省 <FILE>_polished）
#   ZMIN=/ZMAX= 显式 z 保留区间（优先于自动楼层）
#   FLOOR_KEEP=0.10 ROOF_TRIM=0.15   地板/房顶刮除量
#   FLOORS=auto 多层楼梯检测（实验性）  STORY=3.2 MIN_FLOOR_GAP=2.5
#   VOXEL=      显式体素（缺省自适应至 TARGET=400000 点）
#   SOR_MIN=5   SOR 邻居阈（0=关）  SOR_RADIUS=0.25
# 用法:
#   bash m_polish.sh                                    # 全自动（单层）
#   FILE=lite3_ceshi1006 VOXEL=0.08 bash m_polish.sh    # 指定图+固定体素
#   FLOORS=auto bash m_polish.sh                        # 多层楼梯模式
source "$(dirname "$0")/m_env.sh"
FILE=${FILE:-}
if [ -z "$FILE" ]; then
  FILE=$(ls -t /home/ros1-slam/data/maps/*.pcd 2>/dev/null | head -1 | xargs -r basename | sed 's/\.pcd$//')
  echo "[m_polish] 未指定 FILE → 自动选最新: $FILE"
fi
P="/home/ros1-slam/data/maps/${FILE}.pcd"
[ -f "$P" ] || { echo "[m_polish] ✗ $P 不存在"; exit 1; }
exec python3 /home/ros1-slam/scripts/mapping/pcd_polish.py "$P"
