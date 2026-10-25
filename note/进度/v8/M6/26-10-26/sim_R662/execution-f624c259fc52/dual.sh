#!/usr/bin/env bash
# R662 双轨容器阶段：预检先行（stage 驱动注入）→ dual.launch → 驱动采集 → 显式清理。
set -o pipefail
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:/opt/ros1-slam/ws/src:/opt/ros1-slam/r633/ws/src"
source /opt/ros1-slam/r633/ws/devel/setup.bash --extend 2>/dev/null || true
export ROS_HOSTNAME=127.0.0.1
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_LOG_DIR=/opt/ros1-slam/r662/logs
mkdir -p /opt/ros1-slam/r662/logs

TRACK="${1:-truth}"
roslaunch /opt/ros1-slam/r662/inputs/dual.launch > /opt/ros1-slam/r662/logs/roslaunch-$TRACK.log 2>&1 &
RL_PID=$!
trap 'kill -9 $RL_PID 2>/dev/null; pkill -9 -f gzserver 2>/dev/null' EXIT

python3 -B /opt/ros1-slam/r662/inputs/dual_driver.py "$TRACK"
RC=$?

kill -INT "$RL_PID" 2>/dev/null; sleep 12
kill "$RL_PID" 2>/dev/null
pkill -INT -f gzserver 2>/dev/null; sleep 10
pkill -f gzserver 2>/dev/null; sleep 4
pkill -9 -f gzserver 2>/dev/null; sleep 2
trap - EXIT
LEFT=$(pgrep -af "gzserver|roslaunch|rosmaster|scan_planner" | grep -v pgrep || true)
if [ -n "$LEFT" ]; then echo "LEFTOVER: $LEFT"; exit 7; fi
echo "CLEAN"
exit $RC
