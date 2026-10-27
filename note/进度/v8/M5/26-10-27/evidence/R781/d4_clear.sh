#!/bin/bash
set +u
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11319 ROS_IP=127.0.0.1 HOME=/work ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs
FLAT=/repo/note/计划书v8/calibration/flat2d
python3 $FLAT/scan_relay.py > /work/d4_relay.log 2>&1 &
sleep 8
timeout 8 rosservice call /move_base/clear_costmaps >/dev/null 2>&1
sleep 4
echo "--- after clear, cloud live ---"
timeout 40 python3 /work/diag_lethal.py 2>&1 | head -14
sleep 6
echo "--- 6s later ---"
timeout 40 python3 /work/diag_lethal.py 2>&1 | head -6
echo D4_DONE
