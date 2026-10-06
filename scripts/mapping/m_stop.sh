#!/usr/bin/env bash
# m_stop.sh — 采图停机（⚠ interval=-1 全量落盘：SLAM 先停且必须等落盘日志）
# 超参: WAIT=45 落盘等待上限秒
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
ls -lh /home/ros1-slam/data/maps/lite3.pcd 2>/dev/null | awk '{print "[m_stop] 产物: "$5, $9}'
