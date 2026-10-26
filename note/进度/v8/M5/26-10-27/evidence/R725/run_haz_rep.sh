#!/usr/bin/env bash
# 同载体全链：真实 mux + 深度探测器 + MuJoCo Lite3 制动。参数：seed
set +u
SEED=${1:-0}
TAG=${2:-$SEED}
rm -f /work/haz_chain_${TAG}.json /work/haz_chain_det_${TAG}.json /work/haz_chain_result_${TAG}.json
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11316 ROS_IP=127.0.0.1 ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs HOME=/work PYTHONUNBUFFERED=1 MUJOCO_GL=osmesa
mkdir -p /work/ros /work/logs
roscore -p 11316 >/dev/null 2>&1 &
RPID=$!
sleep 5
rosparam set /use_sim_time true
printf 'output_topic: /cmd_vel\noutput_rate_hz: 20\nhold_topic: /lite3/supervisor/hold\nsources:\n  - {name: high, topic: /src/high, priority: 1, timeout_s: 0.5}\n' > /work/m_haz.yaml
rosparam set /mux/mux_config /work/m_haz.yaml
python3 /work/haz_chain_bridge.py "$SEED" "/work/haz_chain_${TAG}.json" 16.0 > /work/haz_chain_${TAG}.blog 2>&1 &
BPID=$!
sleep 3
python3 /repo/ws/src/60_safety/lite3_safety/scripts/safe_cmd_vel_mux.py __name:=mux > /work/haz_mux_${TAG}.log 2>&1 &
MPID=$!
sleep 2
python3 /work/haz_chain_detector.py "/work/haz_chain_det_${TAG}.json" 1.0 > /work/haz_det_${TAG}.log 2>&1 &
DPID=$!
sleep 1
python3 /work/haz_chain_driver.py 30 > /work/haz_drv_${TAG}.log 2>&1 &
wait $BPID 2>/dev/null
sleep 1
python3 /work/haz_chain_metrics.py "/work/haz_chain_${TAG}.json" "/work/haz_chain_det_${TAG}.json" > "/work/haz_chain_result_${TAG}.json" 2>&1
kill $DPID $MPID $RPID 2>/dev/null
pkill -x rosmaster 2>/dev/null; pkill -x rosout 2>/dev/null
sleep 1
cat "/work/haz_chain_result_${TAG}.json"
