#!/bin/bash
set +u
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11319 ROS_IP=127.0.0.1 HOME=/work ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs
FLAT=/repo/note/计划书v8/calibration/flat2d
bash /work/flat2d_stop2.sh
rosparam delete /move_base/global_costmap >/dev/null 2>&1
rosparam delete /move_base/local_costmap >/dev/null 2>&1
timeout 8 rosservice call /gazebo/set_model_state '{model_state: {model_name: lite3_planar, pose: {position: {x: 0.0, y: 0.0, z: 0.15}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, reference_frame: world}}' >/dev/null 2>&1
rosparam set use_sim_time true
rosparam set /mux/mux_config /work/mux_flat2d.yaml
python3 /repo/ws/src/60_safety/lite3_safety/scripts/safe_cmd_vel_mux.py __name:=mux > /work/d3_mux.log 2>&1 &
roslaunch $FLAT/flat2d_stack_sense.launch > /work/d3_stack.log 2>&1 &
sleep 16
timeout 40 python3 /work/diag_lethal.py 2>&1 | head -20
echo D3_DONE
