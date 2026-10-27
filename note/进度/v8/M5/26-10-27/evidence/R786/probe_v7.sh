#!/bin/bash
set +u
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11319 ROS_IP=127.0.0.1 HOME=/work ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs
OUT=/work/score_v7
mkdir -p $OUT
P=/work/ndt_priv/devel/lib/v8_ndt_odom/ndt_score_probe
M=/work/maps/m1_world_synth_v7.pcd
rosparam set use_sim_time false
for pose in "0 0" "3.5 0"; do
  set -- $pose; sx=$1; sy=$2
  timeout 8 rosservice call /gazebo/set_model_state "{model_state: {model_name: lite3_planar, pose: {position: {x: $sx, y: $sy, z: 0.112}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, reference_frame: world}}" >/dev/null 2>&1
  sleep 4
  python3 /work/capture_scan.py $OUT/scan_${sx}_1.pcd 20
  sleep 1
  python3 /work/capture_scan.py $OUT/scan_${sx}_2.pcd 20
  echo "== at ($sx,$sy)"
  for i in 1 2; do printf "  truth f$i "; $P $M $OUT/scan_${sx}_$i.pcd $sx $sy 0 0.5 fixed; done
  if [ "$sx" = "0" ]; then
    for pose2 in "0.10 0 0" "0.15 0 0" "0 0 2" "0 0 5" "0 0 180"; do set -- $pose2; printf "  off "; $P $M $OUT/scan_0_1.pcd $1 $2 $3 0.5 fixed; done
  else
    for pose2 in "3.6 0 0" "3.65 0 0" "3.5 0 2" "3.5 0 5" "3.5 0 180"; do set -- $pose2; printf "  off "; $P $M $OUT/scan_3.5_1.pcd $1 $2 $3 0.5 fixed; done
  fi
done
echo PROBE_V7_DONE
