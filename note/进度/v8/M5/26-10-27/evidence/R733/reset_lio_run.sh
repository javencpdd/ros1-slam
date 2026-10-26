#!/usr/bin/env bash
set +u
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11317 ROS_IP=127.0.0.1 GAZEBO_MASTER_URI=http://127.0.0.1:11346 HOME=/work ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs
rosservice call /gazebo/set_model_state "{model_state: {model_name: lite3_planar, pose: {position: {x: 0.0, y: 0.0, z: 0.15}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, reference_frame: world}}" 2>&1 | tail -2
sleep 1
PIDS=$(ps -eo pid,cmd | grep "run_mapping_online" | grep -v grep | awk '{print $1}')
[ -n "$PIDS" ] && kill -INT $PIDS 2>/dev/null
sleep 3
PIDS=$(ps -eo pid,cmd | grep "run_mapping_online" | grep -v grep | grep -v defunct | awk '{print $1}')
[ -n "$PIDS" ] && kill -TERM $PIDS 2>/dev/null
sleep 2
rosparam load /opt/ros1-slam/ws/src/95_simulation/config/faster_lio_sim.yaml
rosparam set use_sim_time true
rosrun faster_lio run_mapping_online > /work/m1_lio4.log 2>&1 &
sleep 12
python3 /work/lio_dual.py /work/lio_dual_11317b.json 2>&1 | tail -2
