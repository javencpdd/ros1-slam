#!/bin/bash
# R774 续：分辨率/步长调参（新跳变门 + 预测开）
set +u
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11319 ROS_IP=127.0.0.1 HOME=/work ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs
[ -e /work/loc_tune2.lock ] && { echo LOCKED; exit 1; }
touch /work/loc_tune2.lock

run_one() {
  tag=$1; res=$2; iters=$3; step=$4
  pkill -f '[r]un_mapping_online'; pkill -f 'v8_ndt_odom/[n]dt_odom_node'; pkill -f '[s]afe_cmd_vel_mux'; pkill -f 'python3 /work/[s]can_live_relay'
  sleep 3
  rosservice call /gazebo/set_model_state '{model_state: {model_name: lite3_planar, pose: {position: {x: 0.0, y: 0.0, z: 0.15}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, reference_frame: world}}' >/dev/null 2>&1
  sleep 1
  rosparam load /opt/ros1-slam/ws/src/95_simulation/config/faster_lio_sim.yaml
  rosparam set use_sim_time true
  rosrun faster_lio run_mapping_online > /work/loc_lio${tag}.log 2>&1 &
  (
    source /work/ndt_priv/devel/setup.bash
    rosrun v8_ndt_odom ndt_odom_node _map_path:=/work/maps/m1_world_synth_v4.pcd _score_max:=1000.0 _resolution:=$res _max_iterations:=$iters _step_size:=$step _global_search:=false _ext_z:=0.0 _ext_pitch:=0.0 _use_prediction:=true > /work/loc_ndt${tag}.log 2>&1
  ) &
  sleep 14
  python3 /work/loc_dual_v3.py --out /work/loc_${tag}.json --csv /work/loc_${tag} > /work/loc_${tag}.log 2>&1
  echo "LOC RUN ${tag} res=${res} it=${iters} step=${step} done: $(tail -1 /work/loc_${tag}.log | cut -c1-200)"
  sleep 2
}

run_one v36_1 0.5 50 0.05
run_one v37_1 0.35 30 0.1
pkill -f '[r]un_mapping_online'; pkill -f 'v8_ndt_odom/[n]dt_odom_node'
rm -f /work/loc_tune2.lock
echo LOC_ALL_DONE
