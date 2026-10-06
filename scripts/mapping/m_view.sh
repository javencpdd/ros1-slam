#!/usr/bin/env bash
# m_view.sh — 把任意建好的 PCD 发布为 Foxglove 可订阅的 latched 话题（看图用，看完 Ctrl-C）
# 超参:
#   FILE=lite3_ceshi1006   PCD 文件名（不带 .pcd；默认最近产物）
#   TOPIC=/cloud_view      发布话题名
#   DECIMATE=10            抽点稀疏化（1=全量；WiFi 看图建议 5–10）
source "$(dirname "$0")/m_env.sh"
# FILE 不指定 = 自动选 data/maps 下最新的一张（按 mtime）
if [ -z "${FILE:-}" ]; then
  FILE=$(ls -t /home/ros1-slam/data/maps/*.pcd 2>/dev/null | head -1 | xargs -r basename | sed 's/\.pcd$//')
  [ -n "$FILE" ] && echo "[m_view] 未指定 FILE → 自动选最新: $FILE"
fi
FILE=${FILE:-lite3}
TOPIC=${TOPIC:-/cloud_view}
DEC=${DECIMATE:-10}
# master 自检：不在则自动起一个（看图独立于采图流程）
if ! timeout 3 rostopic list >/dev/null 2>&1; then
  echo "[m_view] master 未运行——自动起 roscore 11312（看完可用 m_clean.sh 清）"
  nohup roscore -p 11312 >"/tmp/roscore_view_$$.log" 2>&1 &   # $$ 后缀防多用户/多实例日志文件互踩
  for i in $(seq 1 12); do
    timeout 2 rostopic list >/dev/null 2>&1 && break
    sleep 1
  done
fi
P="/home/ros1-slam/data/maps/${FILE}.pcd"
[ -f "$P" ] || { echo "[m_view] ✗ $P 不存在"; ls /home/ros1-slam/data/maps/*.pcd; exit 1; }
exec python3 /home/ros1-slam/scripts/mapping/pcd_latch_pub.py "$P" "$TOPIC" "$DEC"
