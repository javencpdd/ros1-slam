#!/usr/bin/env bash
# R660a 仿真基线（rospy 驱动版）：bash 管道采集在低 RTF 下不可靠（三跑实证），改 Python。
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
cleanup() {
  [ -n "${RL_PID:-}" ] && kill -INT "$RL_PID" 2>/dev/null; sleep 6
  [ -n "${RL_PID:-}" ] && kill "$RL_PID" 2>/dev/null
  pkill -INT -f gzserver 2>/dev/null; sleep 2
  pkill -f gzserver 2>/dev/null
}
trap cleanup EXIT

python3 -B /opt/ros1-slam/r660/inputs/sim_probe_driver.py "$CASE"
RC=$?

# 退净：容器内不得残留仿真/ROS 进程
LEFT=$(pgrep -af "gzserver|roslaunch|rosmaster|rosgzserver" | grep -v pgrep || true)
if [ -n "$LEFT" ]; then echo "LEFTOVER: $LEFT"; exit 7; fi
echo "CLEAN"
exit $RC
