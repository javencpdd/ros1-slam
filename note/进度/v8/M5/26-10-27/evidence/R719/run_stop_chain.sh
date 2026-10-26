#!/usr/bin/env bash
# 全链停车测量：真实 mux -> MuJoCo 四足策略 -> 基座停住。参数：seed
set +u
SEED=${1:-0}
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs HOME=/work PYTHONUNBUFFERED=1
mkdir -p /work/ros /work/logs
roscore -p 11311 >/dev/null 2>&1 &
RPID=$!
sleep 6
printf 'output_topic: /cmd_vel\noutput_rate_hz: 20\nhold_topic: /lite3/supervisor/hold\nsources:\n  - {name: high, topic: /src/high, priority: 1, timeout_s: 0.5}\n' > /work/m_stop.yaml
rosparam set /mux/mux_config /work/m_stop.yaml
python3 /repo/ws/src/60_safety/lite3_safety/scripts/safe_cmd_vel_mux.py __name:=mux > /work/mux_stop.log 2>&1 &
MPID=$!
sleep 2
python3 /work/mj_bridge.py "$SEED" "/work/mj_chain_${SEED}.json" 13.5 > /work/mj_bridge_${SEED}.log 2>&1 &
BPID=$!
sleep 2
EXTRA=${2:-0}
VCMD=${3:-0.3}
export MJ_XML=${4:-/work/Lite3_rl_deploy-main/third_party/deep_robotics_model/Lite3/Lite3_mjcf/mjcf/Lite3.xml}
export FRICTION_SCALE=${5:-}
python3 /work/mj_stop_driver.py "$EXTRA" "$VCMD" > "/work/mj_driver_${SEED}.json" 2>/work/mj_driver_${SEED}.err
DPID=$!
wait $BPID 2>/dev/null
sleep 1
python3 /work/stop_metrics.py "/work/mj_chain_${SEED}.json" "/work/mj_driver_${SEED}.json" > "/work/mj_stop_result_${SEED}.json" 2>&1
kill $MPID $RPID 2>/dev/null
pkill -x rosmaster 2>/dev/null
pkill -x rosout 2>/dev/null
sleep 1
cat "/work/mj_stop_result_${SEED}.json"
