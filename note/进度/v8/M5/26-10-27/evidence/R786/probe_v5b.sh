#!/bin/bash
set +u
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11319 ROS_IP=127.0.0.1 HOME=/work ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs
OUT=/work/score_v5
P=/work/ndt_priv/devel/lib/v8_ndt_odom/ndt_score_probe
M=/work/maps/m1_world_synth_v5.pcd
echo "== (0,0) offsets NDT"
for pose in "0 0 0" "0.10 0 0" "0.15 0 0" "0 0 2" "0 0 5" "0 0 180"; do
  set -- $pose; printf "  "; $P $M $OUT/scan_0_1.pcd $1 $2 $3 0.5 fixed
done
echo "== (0,0) offsets resid"
for pose in "0 0 0" "0.10 0 0" "0.15 0 0" "0 0 2" "0 0 5" "0 0 180"; do
  set -- $pose; printf "  "; $P $M $OUT/scan_0_1.pcd $1 $2 $3 0.5 resid
done
echo "== (3.5,0) offsets NDT"
for pose in "3.5 0 0" "3.6 0 0" "3.65 0 0" "3.5 0 2" "3.5 0 5" "3.5 0 180"; do
  set -- $pose; printf "  "; $P $M $OUT/scan_3.5_1.pcd $1 $2 $3 0.5 fixed
done
echo "== (3.5,0) offsets resid"
for pose in "3.5 0 0" "3.6 0 0" "3.65 0 0" "3.5 0 2" "3.5 0 5" "3.5 0 180"; do
  set -- $pose; printf "  "; $P $M $OUT/scan_3.5_1.pcd $1 $2 $3 0.5 resid
done
echo PROBE_V5B_DONE
