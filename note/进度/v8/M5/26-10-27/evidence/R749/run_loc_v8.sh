#!/bin/bash
# 定位双轨 v3：LIO＋NDT 同 run 采 12.57 m 路径；SE(2) 对齐、body→base_link、score/accept
set +u
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11319 ROS_IP=127.0.0.1 HOME=/work ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs
for i in 1; do
  pkill -f '[r]un_mapping_online'; pkill -f 'v8_ndt_odom/[n]dt_odom_node'; pkill -f '[s]afe_cmd_vel_mux'; pkill -f 'python3 /work/[s]can_live_relay'
  sleep 3
  # 清理停车试验的侧墙（幂等）
  rosservice call /gazebo/delete_model '{model_name: corr_north}' >/dev/null 2>&1
  rosservice call /gazebo/delete_model '{model_name: corr_south}' >/dev/null 2>&1
  rosservice call /gazebo/set_model_state '{model_state: {model_name: lite3_planar, pose: {position: {x: 0.0, y: 0.0, z: 0.15}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, reference_frame: world}}' >/dev/null 2>&1
  sleep 1
  rosparam load /opt/ros1-slam/ws/src/95_simulation/config/faster_lio_sim.yaml
  rosparam set use_sim_time true
  rosrun faster_lio run_mapping_online > /work/loc_lio8_${i}.log 2>&1 &
  (
    source /work/ndt_priv/devel/setup.bash
    rosrun v8_ndt_odom ndt_odom_node _map_path:=/work/maps/lite3.pcd _score_max:=1.0 _resolution:=1.0 _global_search:=false > /work/loc_ndt8_${i}.log 2>&1
  ) &
  sleep 14
  python3 /work/loc_dual_v3.py --out /work/loc_v8_${i}.json --csv /work/loc_v8_${i} > /work/loc_v8_${i}.log 2>&1
  echo "LOC RUN $i done: $(tail -1 /work/loc_v8_${i}.log | cut -c1-200)"
  sleep 2
done
pkill -f '[r]un_mapping_online'; pkill -f 'v8_ndt_odom/[n]dt_odom_node'
echo LOC_ALL_DONE
