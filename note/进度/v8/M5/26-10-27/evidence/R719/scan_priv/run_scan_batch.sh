#!/usr/bin/env bash
# 27 独立 run：3 场景 × 3 seed × 3 重复；每次全新 roslaunch。
set +u
mkdir -p /work/scan_runs
run_one() {
  local scen=$1 seed=$2 rep=$3
  local out="/work/scan_runs/${scen}_${seed}_${rep}.json"
  local path="/work/path_${scen}_${seed}.json"
  if [ -f "$out" ]; then echo "SKIP $scen $seed $rep"; return; fi
  P=$(ps -eo pid,cmd | grep "roslaunch scan_planner" | grep -v grep | awk '{print $1}')
  [ -n "$P" ] && kill -INT $P 2>/dev/null
  sleep 3
  pkill -x rosmaster 2>/dev/null; pkill -x rosout 2>/dev/null
  sleep 2
  source /opt/ros/noetic/setup.bash
  source /opt/ros1-slam/ws/devel/setup.bash
  source /work/scan_priv/devel/setup.bash
  export CMAKE_PREFIX_PATH=/work/scan_priv/devel:/opt/ros1-slam/ws/devel:/opt/ros/noetic
  export ROS_PACKAGE_PATH=/work/scan_priv/src:/opt/ros1-slam/ws/src:$ROS_PACKAGE_PATH
  export ROS_MASTER_URI=http://127.0.0.1:11314 ROS_IP=127.0.0.1 HOME=/work ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs
  roscore -p 11314 >/dev/null 2>&1 &
  sleep 5
  roslaunch scan_planner run.launch navi_mode:=3 controller_mode:=closed_loop is_real_world:=false > /work/scan_runs/${scen}_${seed}_${rep}.log 2>&1 &
  sleep 20
  cd /work && timeout 60 python3 scan_timing_probe.py 45 "$out" 0.0 "$path" > /work/scan_runs/${scen}_${seed}_${rep}.tail 2>&1
  echo "DONE $scen $seed $rep $(tail -1 /work/scan_runs/${scen}_${seed}_${rep}.tail)"
}
for scen in straight scurve offset; do
  for seed in 0 1 2; do
    for rep in 0 1 2; do
      run_one $scen $seed $rep
    done
  done
done
P=$(ps -eo pid,cmd | grep "roslaunch scan_planner" | grep -v grep | awk '{print $1}')
[ -n "$P" ] && kill -INT $P 2>/dev/null
sleep 3
pkill -x rosmaster 2>/dev/null; pkill -x rosout 2>/dev/null
echo SCAN_BATCH_DONE
