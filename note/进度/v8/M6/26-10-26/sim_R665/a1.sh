#!/usr/bin/env bash
# R665 A1 阶段：预检先行 → a1.launch（后端参数化）→ 驱动 → 显式清理。
set -o pipefail
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
export ROS_HOSTNAME=127.0.0.1
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_LOG_DIR=/opt/ros1-slam/r665/logs
mkdir -p /opt/ros1-slam/r665/logs

BACKEND="${1:-dwa}"
roslaunch /opt/ros1-slam/r665/inputs/a1.launch backend:="$BACKEND" \
  > /opt/ros1-slam/r665/logs/roslaunch-$BACKEND.log 2>&1 &
RL_PID=$!
trap 'kill -9 $RL_PID 2>/dev/null; pkill -9 -f gzserver 2>/dev/null' EXIT

python3 -B /opt/ros1-slam/r665/inputs/a1_driver.py "$BACKEND"
RC=$?

kill -INT "$RL_PID" 2>/dev/null; sleep 12
kill "$RL_PID" 2>/dev/null
pkill -INT -f gzserver 2>/dev/null; sleep 10
pkill -f gzserver 2>/dev/null; sleep 4
pkill -9 -f gzserver 2>/dev/null; sleep 2
trap - EXIT
LEFT=$(pgrep -af "gzserver|roslaunch|rosmaster|move_base" | grep -v pgrep || true)
if [ -n "$LEFT" ]; then echo "LEFTOVER: $LEFT"; exit 7; fi
echo "CLEAN"
exit $RC
