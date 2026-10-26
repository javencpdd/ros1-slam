#!/usr/bin/env bash
# 感知静台：深度相机 10 Hz + 探测器阈值 1.0 m；3 次重复。
set +u
source /opt/ros/noetic/setup.bash
export MUJOCO_GL=osmesa
export ROS_MASTER_URI=http://127.0.0.1:11312 ROS_IP=127.0.0.1 ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs HOME=/work PYTHONUNBUFFERED=1
mkdir -p /work/ros /work/logs
for i in 0 1 2; do
  roscore -p 11312 >/dev/null 2>&1 &
  RPID=$!
  sleep 5
  python3 /work/haz_detector.py "/work/haz_det_${i}.json" 1.0 > "/work/haz_det_${i}.log" 2>&1 &
  DPID=$!
  sleep 1
  python3 /work/haz_render_pub.py 10 "/work/haz_frames_${i}.json" > "/work/haz_render_${i}.log" 2>&1
  sleep 1
  python3 /work/haz_metrics.py "/work/haz_frames_${i}.json" "/work/haz_det_${i}.json" > "/work/haz_result_${i}.json" 2>&1
  kill $DPID 2>/dev/null
  kill $RPID 2>/dev/null
  pkill -x rosmaster 2>/dev/null; pkill -x rosout 2>/dev/null
  sleep 1
  echo -n "seed=$i "; cat "/work/haz_result_${i}.json"
done
echo HAZ_DONE
