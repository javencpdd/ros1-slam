#!/bin/bash
# A2 锁存复验：两次重启 SCAN（fail_safe:=false）＋急停后观察
set +u
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
source /work/scan_priv/devel/setup.bash
export CMAKE_PREFIX_PATH=/work/scan_priv/devel:/opt/ros1-slam/ws/devel:/opt/ros/noetic
export ROS_PACKAGE_PATH=/work/scan_priv/src:/opt/ros1-slam/ws/src:$ROS_PACKAGE_PATH
export ROS_MASTER_URI=http://127.0.0.1:11319 ROS_IP=127.0.0.1 HOME=/work ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs
for i in 2 3; do
  pkill -f '[r]oslaunch scan_planner'; sleep 2
  pkill -f '[s]can_planner_node'; pkill -f '[c]losed_loop_controller'; sleep 1
  rosservice call /gazebo/delete_model '{model_name: stop_wall}' >/dev/null 2>&1
  rosservice call /gazebo/set_model_state '{model_state: {model_name: lite3_planar, pose: {position: {x: 0.0, y: 0.0, z: 0.15}, orientation: {x: 0, y: 0, z: 0, w: 1}}, twist: {linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}}}' >/dev/null 2>&1
  rm -f /work/scan_stop_box.json
  roslaunch scan_planner run.launch navi_mode:=3 controller_mode:=closed_loop is_real_world:=true need_extrinsic:=false fail_safe:=false > /work/scan_latch_$i.log 2>&1 &
  sleep 22
  python3 /work/scan_stop_event.py > /work/scan_latch_ev$i.json 2>&1
  echo "LATCH RUN $i done"
  sleep 2
done
echo LATCH_ALL_DONE
