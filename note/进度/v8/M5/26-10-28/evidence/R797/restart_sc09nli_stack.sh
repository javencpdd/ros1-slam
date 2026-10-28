#!/bin/bash
# f64nli 栈：局部 static+stvl、无局部 inflation
set +u
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11319 ROS_IP=127.0.0.1 HOME=/work ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs
pkill -f '[f]lat2d_stack_sc09_f64.launch'
pkill -f '[f]lat2d_stack_sc09_f64nl.launch'
pkill -f '[f]lat2d_stack_sc09_f64nli.launch'
pkill -f '[f]lat2d_stack_sc09.launch'
pkill -f '[f]lat2d_stack_both_stvl.launch'
pkill -f '[f]lat2d_stack_probe.launch'
pkill -f '[f]lat2d_stack_probe3.launch'
pkill -f '[f]lat2d_stack_probe4.launch'
pkill -f '[f]lat2d_stack_probe5.launch'
pkill -f '[f]lat2d_stack_global_stvl.launch'
pkill -f '[f]lat2d_stack_obs_local.launch'
pkill -f '[f]lat2d_stack_stvl.launch'
pkill -f '[f]lat2d_stack_sense.launch'
pkill -f '[f]lat2d_stack.launch'
sleep 1
rosparam delete /move_base/global_costmap >/dev/null 2>&1
rosparam delete /move_base/local_costmap >/dev/null 2>&1
rosparam set use_sim_time true
roslaunch /repo/ws/src/90_platform/v8_platform/calibration/flat2d/flat2d_stack_sc09_f64nli.launch > /work/sc09_f64nli_stack.log 2>&1 &
sleep 10
bash /work/start_mux.sh
bash /work/relay_restart_volume.sh
sleep 8
rosnode list | grep -E "move_base|map_server|map_to_odom|mux|scan_relay"
