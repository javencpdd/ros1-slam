#!/usr/bin/env bash
# m_stop.sh — 采图停机（⚠ interval=-1 全量落盘：SLAM 先停且必须等落盘日志）
# 超参: WAIT=45 落盘等待上限秒
#       NAME=标签     产物重命名（默认=月日_时分，如 lite3_1006_1759.pcd；NAME=mine→lite3_mine.pcd；NAME=keep→保持 lite3.pcd）
#       KEEP_LATEST=1 额外保留一份 lite3.pcd 副本（下游 hdl map_name 默认名兼容；263MB 级大图注意磁盘）
source "$(dirname "$0")/m_env.sh"
WAIT=${WAIT:-45}
echo "[m_stop] 停 SLAM（等待落盘日志，最长 ${WAIT}s）..."
pkill -INT -f run_mapping_online 2>/dev/null
for i in $(seq 1 "$WAIT"); do
  if dmesg 2>/dev/null | tail -0 >/dev/null; then :; fi
  # 落盘完成的判据=进程退出
  pgrep -f run_mapping_online >/dev/null || { echo "[m_stop] faster_lio 已退出（落盘完成）"; break; }
  sleep 1
done
pgrep -f run_mapping_online >/dev/null && echo "[m_stop] ⚠ ${WAIT}s 内 faster_lio 仍在——PCD 可能未写完，检查 3 号窗日志！"
pkill -INT -f livox_ros_driver2 2>/dev/null; sleep 2
pkill -f "roscore -p 11312|rosmaster" 2>/dev/null; pkill -f "roscore" 2>/dev/null
sleep 2
N=$(ps aux | grep -E "roscore|rosmaster|livox|run_mapping" | grep -v grep | wc -l)
[ "$N" -eq 0 ] && echo "[m_stop] 全部退净 ✓" || { echo "[m_stop] ⚠ 残留 $N："; ps aux | grep -E "roscore|livox|run_mapping" | grep -v grep; }
# ---- 产物重命名（NAME/KEEP_LATEST 超参；interval=-1 全量图通常数百 MB）----
STAMP=$(date +%m%d_%H%M)
if [ "${NAME:-}" = "keep" ]; then
  DEST="lite3.pcd"; echo "[m_stop] 产物保持默认名: $DEST"
elif [ -n "${NAME:-}" ]; then
  DEST="lite3_${NAME}.pcd"
else
  DEST="lite3_${STAMP}.pcd"
fi
if [ -f /home/ros1-slam/data/maps/lite3.pcd ] && [ "$DEST" != "lite3.pcd" ]; then
  mv /home/ros1-slam/data/maps/lite3.pcd "/home/ros1-slam/data/maps/$DEST"
  echo "[m_stop] 产物重命名: lite3.pcd → $DEST"
fi
if [ "${KEEP_LATEST:-0}" = "1" ] && [ "$DEST" != "lite3.pcd" ] && [ -f "/home/ros1-slam/data/maps/$DEST" ]; then
  cp "/home/ros1-slam/data/maps/$DEST" /home/ros1-slam/data/maps/lite3.pcd
  echo "[m_stop] 最新副本: lite3.pcd（KEEP_LATEST=1，下游默认名兼容）"
fi
ls -lh /home/ros1-slam/data/maps/lite3*.pcd 2>/dev/null | awk '{print "[m_stop] 产物: "$5, $9}'
