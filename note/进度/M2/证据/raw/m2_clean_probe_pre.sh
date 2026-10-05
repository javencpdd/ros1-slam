#!/usr/bin/env bash
set -u
source /opt/ros/noetic/setup.bash
[ -f /home/ros1-slam/ws/devel/setup.bash ] && source /home/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11313 GAZEBO_MASTER_URI=http://127.0.0.1:11355
export ROS_HOSTNAME=localhost
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:/home/ros1-slam/ws/devel/lib
T0=$(date +%s)
roslaunch lite3_gazebo_sim m1_gazebo.launch > /tmp/m2_clean_pre.log 2>&1 &
sleep 8
for i in $(seq 1 20); do
  CT=$(timeout 3 rostopic echo -n 1 /clock 2>/dev/null | grep -oE "secs: [0-9]+" | head -1 | awk '{print $2}')
  CPU=$(ps -o pcpu= -p "$(pgrep -f '^gzserver' | head -1)" 2>/dev/null | tr -d ' ')
  echo "t=$(( $(date +%s)-T0 ))s clock_secs=${CT:-none} gzcpu=${CPU:-none}"
  [ -n "$CT" ] && [ "$CT" -ge 3 ] 2>/dev/null && { echo "CLOCK_ADVANCING_AT t=$(( $(date +%s)-T0 ))s sim=${CT}s"; break; }
  sleep 12
done
