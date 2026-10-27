#!/bin/bash
# R786：v7 世界/图（肋＋隔墙＋走廊柱）3 次定位对照
set +u
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11319 ROS_IP=127.0.0.1 HOME=/work ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs
[ -e /work/loc7.lock ] && { echo LOCKED; exit 1; }
touch /work/loc7.lock
run_one() {
  tag=$1
  pkill -f '[r]un_mapping_online'; pkill -f 'v8_ndt_odom/[n]dt_odom_node'; pkill -f '[s]afe_cmd_vel_mux'
  sleep 3
  timeout 8 rosservice call /gazebo/set_model_state '{model_state: {model_name: lite3_planar, pose: {position: {x: 0.0, y: 0.0, z: 0.112}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, reference_frame: world}}' >/dev/null 2>&1
  sleep 4
  rosparam load /opt/ros1-slam/ws/src/95_simulation/config/faster_lio_sim.yaml
  rosparam set use_sim_time true
  rosrun faster_lio run_mapping_online > /work/l7_lio_${tag}.log 2>&1 &
  (
    source /work/ndt_priv/devel/setup.bash
    rosrun v8_ndt_odom ndt_odom_node _map_path:=/work/maps/m1_world_synth_v7.pcd _score_max:=1000.0 _resolution:=0.5 _max_iterations:=30 _step_size:=0.1 _global_search:=false _ext_z:=0.0 _ext_pitch:=0.0 _use_prediction:=true > /work/l7_ndt_${tag}.log 2>&1
  ) &
  sleep 14
  python3 /work/loc_dual_v3.py --out /work/l7_${tag}.json --csv /work/l7_${tag} --leg 13.0 > /work/l7_${tag}.log 2>&1
  echo "L7 ${tag} done: $(tail -1 /work/l7_${tag}.log | cut -c1-120)"
  sleep 2
}
run_one a
run_one b
run_one c
pkill -f '[r]un_mapping_online'; pkill -f 'v8_ndt_odom/[n]dt_odom_node'
rm -f /work/loc7.lock
echo L7_ALL_DONE
