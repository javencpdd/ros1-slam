#!/usr/bin/env bash
# w3_t2_run.sh —— 矩阵W3/A5 组合矩阵 T2 驱动（小明 2026-10-07；m2_w2_localcmp.sh 同构，/tmp/matrix_w3 路径）
# 用法: bash w3_t2_run.sh <controller> <runlabel> <planner>
# 判据（stageB 口径）：state=3/outcome=0、dt≤120s(sim)、dev_odom≤0.35m、A0 RTF≥0.25（M2R2 裁决，逐跑必探）
CTRL=$1; LABEL=$2; PLANNER=$3
LOGDIR=/tmp/matrix_w3/logs; mkdir -p "$LOGDIR"
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11313 GAZEBO_MASTER_URI=http://127.0.0.1:11345
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:/home/ros1-slam/ws/devel/lib

cleanup() { rosnode kill -a 2>/dev/null; sleep 3
  pkill -f gzserver 2>/dev/null; pkill -f gzclient 2>/dev/null
  pkill -f "roslaunch.*m2_teb" 2>/dev/null; pkill -f move_base_flex 2>/dev/null
  pkill -f sim_adapter 2>/dev/null; pkill -f m2_cmd_relay 2>/dev/null; sleep 2; }
trap cleanup EXIT

echo "[w3-T2] run=$LABEL planner=$PLANNER controller=$CTRL 起 m2_teb.launch..."
setsid roslaunch lite3_gazebo_sim m2_teb.launch > "$LOGDIR/T2_${LABEL}_launch.log" 2>&1 &
T=0
until [ "$T" -ge 300 ]; do
  if rostopic list 2>/dev/null | grep -qx -- "/clock" && timeout 10 rostopic echo -n 1 /clock >/dev/null 2>&1; then
    echo "[w3-T2] /clock 就绪 t=${T}s"; break
  fi
  sleep 2; T=$((T+2))
done
[ "$T" -lt 300 ] || { echo "[w3-T2] FAIL: /clock 300s 未出"; exit 1; }

python3 /tmp/matrix_w3/w3_t2_exec.py "$CTRL" "$LABEL" "$PLANNER" 2>&1 | tee "$LOGDIR/T2_${LABEL}.txt"
RC=${PIPESTATUS[0]}
echo "[w3-T2] rc=$RC，退净..."
exit $RC
