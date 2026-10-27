#!/bin/bash
set +u
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11319 ROS_IP=127.0.0.1 HOME=/work ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs
OUT=/work/score_static2
mkdir -p $OUT
bash /work/flat2d_stop2.sh
pkill -f '[r]un_mapping_online'
sleep 2
timeout 8 rosservice call /gazebo/set_model_state '{model_state: {model_name: lite3_planar, pose: {position: {x: 0.0, y: 0.0, z: 0.15}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, reference_frame: world}}' >/dev/null 2>&1
rosparam set use_sim_time false
sleep 2
for i in 1 2 3 4 5 6; do
  python3 /work/capture_scan.py $OUT/scan_$i.pcd 20
  sleep 1
done
PROBE=/work/ndt_priv/devel/lib/v8_ndt_odom/ndt_score_probe
MAP=/work/maps/m1_world_synth_v4.pcd
: > $OUT/probe_true.txt
for i in 1 2 3 4 5 6; do
  $PROBE $MAP $OUT/scan_$i.pcd 0 0 0 0.5 fixed >> $OUT/probe_true.txt 2>>$OUT/probe.err
done
: > $OUT/probe_offsets.txt
for pose in "0.10 0 0" "0.15 0 0" "0 0 2" "0 0 5" "0 0 180"; do
  set -- $pose
  $PROBE $MAP $OUT/scan_1.pcd $1 $2 $3 0.5 fixed >> $OUT/probe_offsets.txt 2>>$OUT/probe.err
done
: > $OUT/probe_align.txt
$PROBE $MAP $OUT/scan_1.pcd 0 0 0 0.5 align >> $OUT/probe_align.txt 2>>$OUT/probe.err
echo SCORE2_DONE
