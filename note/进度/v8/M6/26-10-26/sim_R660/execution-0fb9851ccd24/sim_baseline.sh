#!/usr/bin/env bash
# R660a 仿真基线（rospy 驱动＋显式清理序）：清理须在退净检查之前（trap EXIT 时序教训）。
set -o pipefail
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
export ROS_HOSTNAME=127.0.0.1
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_LOG_DIR=/opt/ros1-slam/r660/logs
mkdir -p /opt/ros1-slam/r660/logs

CASE="${1:-stairs}"
if [ "$CASE" = "m1" ]; then
  LAUNCH="lite3_gazebo_sim m1_gazebo.launch"
else
  LAUNCH="lite3_gazebo_sim stairstudy.launch world_file:=stair_lab_stairs_easy.world"
fi

roslaunch $LAUNCH > /opt/ros1-slam/r660/logs/roslaunch-$CASE.log 2>&1 &
RL_PID=$!
trap 'kill -9 $RL_PID 2>/dev/null; pkill -9 -f gzserver 2>/dev/null' EXIT

python3 -B /opt/ros1-slam/r660/inputs/sim_probe_driver.py "$CASE"
RC=$?

# 显式清理序（gzserver 低 RTF 下 INT 关闭慢：INT→宽限→TERM→宽限→KILL）
kill -INT "$RL_PID" 2>/dev/null; sleep 12
kill "$RL_PID" 2>/dev/null
pkill -INT -f gzserver 2>/dev/null; sleep 10
pkill -f gzserver 2>/dev/null; sleep 4
pkill -9 -f gzserver 2>/dev/null; sleep 2
trap - EXIT

LEFT=$(pgrep -af "gzserver|roslaunch|rosmaster" | grep -v pgrep || true)
if [ -n "$LEFT" ]; then echo "LEFTOVER: $LEFT"; exit 7; fi
echo "CLEAN"
exit $RC
