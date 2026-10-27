#!/bin/bash
# R784：定位 9 run（3 起点 × 3 重复），LIO＋NDT 同采；NDT 用 res0.5/预测开/新门
set +u
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11319 ROS_IP=127.0.0.1 HOME=/work ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs
[ -e /work/loc9b.lock ] && { echo LOCKED; exit 1; }
touch /work/loc9b.lock

run_one() {
  tag=$1; sx=$2; sy=$3
  pkill -f '[r]un_mapping_online'; pkill -f 'v8_ndt_odom/[n]dt_odom_node'; pkill -f '[s]afe_cmd_vel_mux'; pkill -f 'python3 /work/[s]can_live_relay'
  sleep 3
  timeout 8 rosservice call /gazebo/set_model_state "{model_state: {model_name: lite3_planar, pose: {position: {x: $sx, y: $sy, z: 0.15}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, reference_frame: world}}" >/dev/null 2>&1
  sleep 1
  rosparam load /opt/ros1-slam/ws/src/95_simulation/config/faster_lio_sim.yaml
  rosparam set use_sim_time true
  rosrun faster_lio run_mapping_online > /work/l9_lio_${tag}.log 2>&1 &
  (
    source /work/ndt_priv/devel/setup.bash
    rosrun v8_ndt_odom ndt_odom_node _map_path:=/work/maps/m1_world_synth_v4.pcd _score_max:=1000.0 _resolution:=0.5 _max_iterations:=30 _step_size:=0.1 _global_search:=false _ext_z:=0.0 _ext_pitch:=0.0 _use_prediction:=true _init_x:=$sx _init_y:=$sy > /work/l9_ndt_${tag}.log 2>&1
  ) &
  sleep 14
  python3 /work/loc_dual_v3.py --out /work/l9b_${tag}.json --csv /work/l9b_${tag} --leg 13.0 > /work/l9b_${tag}.log 2>&1
  echo "L9 ${tag} start=($sx,$sy) done: $(tail -1 /work/l9b_${tag}.log | cut -c1-150)"
  sleep 2
}

i=0
for start in "0.0 0.0" "0.8 0.4" "1.4 -0.4"; do
  set -- $start
  for rep in 1 2 3; do
    i=$((i+1))
    run_one "s${i}_r${rep}" $1 $2
  done
done
pkill -f '[r]un_mapping_online'; pkill -f 'v8_ndt_odom/[n]dt_odom_node'
rm -f /work/loc9b.lock
echo L9_ALL_DONE
