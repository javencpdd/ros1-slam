#!/usr/bin/env bash
set -uo pipefail
OUT="$1"; DBG="${2:-false}"
mkdir -p "$OUT"
export ROS_MASTER_URI=http://127.0.0.1:11313 GAZEBO_MASTER_URI=http://127.0.0.1:11345
export ROS_HOME="$OUT/ros_home" PYTHONNOUSERSITE=1
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
source /opt/ros1-slam/far_overlay/devel/setup.bash
LOG="$OUT/cloud_run.log"; : > "$LOG"
say(){ printf '%s\n' "$*" | tee -a "$LOG"; }
roslaunch lite3_bringup mode_sim_bt.launch global_backend:=far enable_bt:=false \
  arrival_topic:=/v8/bt/arrived far_debug:=$DBG > "$OUT/launch.log" 2>&1 &
LP=$!
trap 'kill -INT $LP 2>/dev/null; sleep 5; kill -TERM $LP 2>/dev/null; sleep 2; kill -KILL $LP 2>/dev/null; wait $LP 2>/dev/null' EXIT
sleep 30
say "== 云链路四段（30 s 后） =="
python3 "$(dirname "$0")/_cloud_probe.py" 8 2>&1 | tee -a "$LOG"
say "== FAR 图读数 =="
grep -c "Global V-Graph Updated" "$OUT/launch.log" | tee -a "$LOG"
grep -o "Number of global vertices: [0-9]*" "$OUT/launch.log" | tail -2 | tee -a "$LOG"
say "== 目标接受性：发一条 1.0 m 目标再看 =="
timeout 5 rostopic pub -1 /v8/global/goal geometry_msgs/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}' >/dev/null 2>&1
sleep 6
timeout 6 rostopic echo -n1 /global_path/status 2>&1 | head -3 | tee -a "$LOG"
say "== 目标之后 FAR 侧 =="
grep -c "new goal updated" "$OUT/launch.log" | tee -a "$LOG"
grep -c "Path Search Time" "$OUT/launch.log" | tee -a "$LOG"
python3 "$(dirname "$0")/_cloud_probe.py" 4 2>&1 | tee -a "$LOG"
say CLOUD_RUN_DONE
