#!/usr/bin/env bash
set -u
source /opt/ros/noetic/setup.bash
[ -f /home/ros1-slam/ws/devel/setup.bash ] && source /home/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11313 GAZEBO_MASTER_URI=http://127.0.0.1:11355
export ROS_HOSTNAME=localhost
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:/home/ros1-slam/ws/devel/lib
cleanup() { rosnode kill -a 2>/dev/null; sleep 2; pkill -f gzserver 2>/dev/null; pkill -f "roslaunch.*manual" 2>/dev/null; true; }
trap cleanup EXIT
roslaunch lite3_gazebo_sim m1_gazebo.launch > /tmp/m2_manual_gz.log 2>&1 &
echo "t0=$(date +%s)"
for t in $(seq 1 60); do
  if timeout 2 rostopic list 2>/dev/null | grep -q "^/gazebo/model_states$"; then echo "MODEL_STATES_READY_AT=$((t*4))s(近似)"; break; fi
  sleep 4
done
rostopic list 2>/dev/null | head -25
echo "=== echo one model_states:"
timeout 15 rostopic echo -n 1 /gazebo/model_states 2>&1 | head -12
echo "=== gzserver alive:"; pgrep -af gzserver | grep -v grep | head -2
