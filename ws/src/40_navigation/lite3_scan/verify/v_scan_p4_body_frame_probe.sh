#!/usr/bin/env bash
# v_scan_p4_body_frame_probe.sh —— 机体系/地图系归属探针的启动壳（#57／请求单 B9）
#
# 链与 v_scan_p4_cloud_map_probe.sh 同序（roscore → m2_gazebo → m2_mapping → 60_safety → 70_scan），
# 两处差别：**不投 goal**（无轨迹无重规划）＋**逐档原地转机体偏航**（用 set_model_state，不发 /cmd_vel）。
# 零改点：不动 p4_replan.py、不动 SEE_RADIUS/SEE_MIN_PTS/AVOID/到达门、世界件一字不动（运行时搬模型并还原）。
#
# 调用姿势自证：bash ws/src/40_navigation/lite3_scan/verify/v_scan_p4_body_frame_probe.sh
# rc：0=判出机体系（盲区随机体转）1=判出世界/地图系（盲区固定）2=env/锁 9=前置或自锁不过
# 注意：**无判据行 ⇒ 改判 INVALID(9)**（"跑不动"不等于"跑不过"，坑30 同族）
set -u
ROOT=/home/ros1-slam
PROBE=$ROOT/ws/src/40_navigation/lite3_scan/verify/p4_body_frame_probe.py
WORLD=$ROOT/ws/src/95_simulation/lite3_gazebo_sim/worlds/m1_world.world
STAMP=$(date +%Y%m%d_%H%M%S)
OUT=${BF_EVIDENCE:-$ROOT/note/进度/SCAN集成/证据/log/raw/p4_body_frame_probe_$STAMP}
LOG=""; PIDS=(); ROSCORE_PID=""

mkdir -p "$OUT" || { echo "建不出证据目录"; exit 2; }
LOG="$OUT/probe_shell.log"
say() { echo "[$(date '+%T')] $*" | tee -a "$LOG"; }
note() { echo "$*" | tee -a "$LOG"; }

LOCK=${SCAN_HARNESS_LOCK:-/tmp/lite3_scan_harness.lock}
if [ -f "$LOCK" ]; then
  OLD=$(cat "$LOCK" 2>/dev/null)
  if [ -n "$OLD" ] && kill -0 "$OLD" 2>/dev/null; then
    echo "P4 机体系归属探针 = INVALID|已有链上实例在跑（PID=$OLD）⇒ 并发不自证，拒跑"; exit 2
  fi
  note "陈旧锁接管（旧值=$OLD）"
fi
echo $$ > "$LOCK"

cleanup() {
  say "-- teardown --"
  rm -f "$LOCK"
  rosnode kill -a >/dev/null 2>&1; sleep 2
  for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null; done
  [ -n "$ROSCORE_PID" ] && kill "$ROSCORE_PID" 2>/dev/null
  pkill -f 'roslaunch.*m2_gazebo' 2>/dev/null; pkill -f 'roslaunch.*m2_mapping' 2>/dev/null
  pkill -f 'roslaunch.*70_scan' 2>/dev/null; pkill -f 'roslaunch.*60_safety' 2>/dev/null
  pkill -f gzserver 2>/dev/null; pkill -f 'run_mapping_online' 2>/dev/null
  pkill -f 'scan_planner_node' 2>/dev/null; pkill -f 'safe_cmd_vel_mux' 2>/dev/null
  sleep 4
  RES=$(ps -eo comm= | grep -cE '^(gzserver|rosmaster|roscore|rosout|run_mapping_online|scan_planner_node)$' || true)
  PORTS=$(ss -ltnH 2>/dev/null | grep -cE ':(11311|11312|11313|11345)\b' || true)
  note "退净：残留进程(comm 计)=$RES 在册端口监听=$PORTS（都应为 0）"
  sha256sum "$WORLD" > "$OUT/world_after.sha"
  N=$(find "$OUT" -type f | wc -l); note "证据件数=$N"
}
trap cleanup EXIT

set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
[ -f "$ROOT/ws/devel/setup.bash" ] && source "$ROOT/ws/devel/setup.bash"
set -u
[ "${ROS_DISTRO:-}" = "noetic" ] || { note "FATAL ROS_DISTRO=$ROS_DISTRO"; rm -f "$LOCK"; exit 2; }
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:$ROOT/ws/devel/lib
export ROS_MASTER_URI=http://127.0.0.1:11313
wait_topic() {
  local t=$1 deadline=$((SECONDS + ${2:-300}))
  while [ $SECONDS -lt $deadline ]; do
    rostopic list 2>/dev/null | grep -qx "$t" && return 0
    sleep 2
  done
  return 1
}
sha256sum "$WORLD" > "$OUT/world_before.sha"

say "== [1] roscore（11313）=="
roscore > "$OUT/roscore.log" 2>&1 &
ROSCORE_PID=$!
for _ in $(seq 1 30); do rostopic list >/dev/null 2>&1 && break; sleep 1; done
say "== [2] m2_gazebo =="
roslaunch lite3_gazebo_sim m2_gazebo.launch gui:=false > "$OUT/gazebo.log" 2>&1 &
PIDS+=($!)
wait_topic /clock 300 || { echo "P4 机体系归属探针 = INVALID|/clock 未就绪"; exit 9; }
wait_topic /rslidar_points 480 || { echo "P4 机体系归属探针 = INVALID|/rslidar_points 未就绪"; exit 9; }
say "== [3] m2_mapping =="
roslaunch lite3_gazebo_sim m2_mapping.launch > "$OUT/mapping.log" 2>&1 &
PIDS+=($!)
wait_topic /Odometry 360 || { echo "P4 机体系归属探针 = INVALID|/Odometry 未就绪"; exit 9; }
say "== [4] 60_safety =="
roslaunch lite3_safety 60_safety.launch > "$OUT/mux.log" 2>&1 &
PIDS+=($!)
wait_topic /cmd_vel 240 || { echo "P4 机体系归属探针 = INVALID|/cmd_vel 未就绪"; exit 9; }
say "== [5] 70_scan（navi_mode:=1，不投 goal）=="
roslaunch lite3_scan 70_scan.launch navi_mode:=1 > "$OUT/scan.log" 2>&1 &
PIDS+=($!)
wait_topic /LIO/odom_vehicle 240 || { echo "P4 机体系归属探针 = INVALID|odom relay 未起"; exit 9; }
wait_topic /grid_map/occupancy 240 || { echo "P4 机体系归属探针 = INVALID|占据图未就绪"; exit 9; }
say "等 45s 让地图收敛（P1 口径的收敛窗量级）"; sleep 45
note "占据图是否在发：$(rostopic hz -w5 -1 /grid_map/occupancy 2>/dev/null | grep -m1 average || echo 未知)"

say "== [6] 跑机体系/地图系归属探针（偏航档 ${PROBE_YAWS:-0,30,60,90}）=="
BF_OUT="$OUT/bf.json" python3 "$PROBE" > "$OUT/probe_stdout.txt" 2>&1
rc=$?
VERD=$(grep -o 'P4 机体系归属探针 = [A-Z]*|.*' "$OUT/probe_stdout.txt" | tail -1)
if [ -z "$VERD" ]; then
  note "探针未产出判据行（崩溃或提前退出）⇒ 这是**跑不动**不是跑不过，按三态口径改判 INVALID"
  echo "P4 机体系归属探针 = INVALID|探针进程未产出判据行（见 probe_stdout.txt 的 traceback）；原 rc=$rc" | tee -a "$LOG"
  exit 9
fi
echo "$VERD" | tee -a "$LOG"
sed -n '1,10p' "$OUT/probe_stdout.txt" | tee -a "$LOG"
note "分档自证与档位明细见 probe_stdout.txt 全文／机器可读件 bf.json"
exit $rc
