#!/usr/bin/env bash
# window_watch.sh — 活体窗就绪看守（window-scheduler 技能通用版；零 ROS、零节点、零 sudo）
# 判据：1 分钟 loadavg ≤ WS_LOAD_THRESHOLD（缺省 4.0）；可选预检脚本 rc=0
# 用法： WS_LOAD_THRESHOLD=4.0 WS_FLAG=/path/window_open.flag WS_PREFLIGHT=/path/preflight.sh \
#          bash window_watch.sh [超时小时，缺省 8]
# 出口： rc=0 立旗成功；rc=3 超时未立旗
# 纪律： 退避轮询（WS_INTERVAL 一拍，禁 tight loop）；预检输出随旗落留底，正文只从留底取数
set -u

INTERVAL="${WS_INTERVAL:-60}"
TIMEOUT_H="${1:-${WS_TIMEOUT_H:-8}}"
THRESH="${WS_LOAD_THRESHOLD:-4.0}"
FLAG="${WS_FLAG:-/home/ros1-slam/logs/window_open.flag}"
PREF="${WS_PREFLIGHT:-}"
PREFLOG="/tmp/window_watch_pref.log"

END=$(( $(date +%s) + TIMEOUT_H * 3600 ))
n=0
while [ "$(date +%s)" -lt "$END" ]; do
  n=$((n + 1))
  load1=$(cut -d' ' -f1 /proc/loadavg)
  under=$(awk -v L="$load1" -v T="$THRESH" 'BEGIN{print (L <= T) ? 1 : 0}')
  if [ "$under" = "1" ]; then
    if [ -n "$PREF" ] && [ -f "$PREF" ]; then
      if ! bash "$PREF" > "$PREFLOG" 2>&1; then
        sleep "$INTERVAL"; continue
      fi
      pref_note="preflight_rc=0"
    else
      pref_note="preflight=skipped"
    fi
    {
      echo "WINDOW_OPEN $(date '+%F %H:%M:%S') load1=${load1} poll=${n} ${pref_note}"
      [ -s "$PREFLOG" ] && { echo "---- preflight 输出留底 ----"; cat "$PREFLOG"; }
    } > "$FLAG"
    echo "WINDOW_OPEN load1=${load1} poll=${n} → ${FLAG}"
    exit 0
  fi
  sleep "$INTERVAL"
done
echo "TIMEOUT after ${TIMEOUT_H}h（last load1=${load1}）— 未立旗"
exit 3
