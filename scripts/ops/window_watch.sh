#!/usr/bin/env bash
# window_watch.sh — 活体窗就绪看守（v6 自动调度配套件；零 ROS、零节点、零 sudo）
# 判据（在册）：1 分钟 loadavg ≤4（v5.3 §0.11）且 scripts/verify/v_window_preflight.sh rc=0
# 用法： scripts/ops/window_watch.sh [超时小时数，默认 8]
# 出口： rc=0 立旗成功（旗＝/home/ros1-slam/logs/window_open.flag）
#        rc=3 超时未立旗（旗不落，调用方读不到旗＝不可开窗）
# 纪律： 退避轮询（60s 一拍，禁 tight loop）；预检输出随旗落留底，正文只从留底取数（R324 险情兜法）
set -u

INTERVAL=60
TIMEOUT_H="${1:-8}"
FLAG="/home/ros1-slam/logs/window_open.flag"
PREF="/home/ros1-slam/scripts/verify/v_window_preflight.sh"
PREFLOG="/tmp/window_watch_pref.log"
END=$(( $(date +%s) + TIMEOUT_H * 3600 ))

n=0
while [ "$(date +%s)" -lt "$END" ]; do
  n=$((n + 1))
  load1=$(cut -d' ' -f1 /proc/loadavg)
  under=$(awk -v L="$load1" 'BEGIN{print (L <= 4.0) ? 1 : 0}')
  if [ "$under" = "1" ] && bash "$PREF" > "$PREFLOG" 2>&1; then
    {
      echo "WINDOW_OPEN $(date '+%F %H:%M:%S') load1=${load1} poll=${n} preflight_rc=0"
      echo "---- v_window_preflight.sh 输出留底 ----"
      cat "$PREFLOG"
    } > "$FLAG"
    echo "WINDOW_OPEN load1=${load1} poll=${n} → ${FLAG}"
    exit 0
  fi
  sleep "$INTERVAL"
done
echo "TIMEOUT after ${TIMEOUT_H}h（last load1=${load1}）— 未立旗"
exit 3
