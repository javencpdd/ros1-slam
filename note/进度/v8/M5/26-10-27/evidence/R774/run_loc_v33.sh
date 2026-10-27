#!/bin/bash
# R774：跳变门改为“自洽≥10 帧 ＋ 分数<0.5×已跟踪”后，pred-on 三次复跑（v33_1..3）
set +u
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11319 ROS_IP=127.0.0.1 HOME=/work ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs

run_one() {
  tag=$1; pred=$2
  pkill -f '[r]un_mapping_online'; pkill -f 'v8_ndt_odom/[n]dt_odom_node'; pkill -f '[s]afe_cmd_vel_mux'; pkill -f 'python3 /work/[s]can_live_relay'
  sleep 3
  rosservice call /gazebo/delete_model '{model_name: corr_north}' >/dev/null 2>&1
  rosservice call /gazebo/delete_model '{model_name: corr_south}' >/dev/null 2>&1
  rosservice call /gazebo/set_model_state '{model_state: {model_name: lite3_planar, pose: {position: {x: 0.0, y: 0.0, z: 0.15}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, reference_frame: world}}' >/dev/null 2>&1
  sleep 1
  rosparam load /opt/ros1-slam/ws/src/95_simulation/config/faster_lio_sim.yaml
  rosparam set use_sim_time true
  rosrun faster_lio run_mapping_online > /work/loc_lio${tag}.log 2>&1 &
  (
    source /work/ndt_priv/devel/setup.bash
    rosrun v8_ndt_odom ndt_odom_node _map_path:=/work/maps/m1_world_synth_v4.pcd _score_max:=1000.0 _resolution:=1.0 _global_search:=false _ext_z:=0.0 _ext_pitch:=0.0 _use_prediction:=$pred > /work/loc_ndt${tag}.log 2>&1
  ) &
  sleep 14
  python3 /work/loc_dual_v3.py --out /work/loc_${tag}.json --csv /work/loc_${tag} > /work/loc_${tag}.log 2>&1
  echo "LOC RUN ${tag} pred=${pred} done: $(tail -1 /work/loc_${tag}.log | cut -c1-260)"
  sleep 2
}

run_one v33_1 true
run_one v33_2 true
run_one v33_3 true
pkill -f '[r]un_mapping_online'; pkill -f 'v8_ndt_odom/[n]dt_odom_node'
rm -f /work/loc_gate.lock
echo LOC_ALL_DONE
