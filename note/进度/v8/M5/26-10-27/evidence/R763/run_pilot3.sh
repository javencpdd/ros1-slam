#!/bin/bash
set +u
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11319 ROS_IP=127.0.0.1 HOME=/work ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs
for i in 1 2 3; do
  pkill -f '[r]oslaunch /work/pilot_a1.launch'; sleep 2
  pkill -f '[m]ove_base'; pkill -f '[m]ap_server'; sleep 1
  rosservice call /gazebo/set_model_state '{model_state: {model_name: lite3_planar, pose: {position: {x: 0.0, y: 0.0, z: 0.15}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, reference_frame: world}}' >/dev/null 2>&1
  roslaunch /work/pilot_a1.launch > /work/pilot_run_${i}.launch.log 2>&1 &
  sleep 12
  python3 /work/pilot_driver3.py dwa > /work/pilot_run_${i}.driver.log 2>&1
  cp /work/pilot_a1.json /work/pilot_run_${i}.json
  echo "PILOT RUN $i done"
  sleep 2
done
pkill -f '[r]oslaunch /work/pilot_a1.launch'; sleep 2
pkill -f '[m]ove_base'; pkill -f '[m]ap_server'
echo PILOT3_DONE
