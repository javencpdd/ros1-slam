#!/usr/bin/env bash
# v_scan_bt.sh —— SCAN 规划/跟踪接入 v8 BT 编排的活体验收（2026-11-12）
#
# 链：roscore → m2_gazebo → m2_mapping(faster_lio) → 60_safety(mux) → 70_scan
#     → scan_bt_nav(BT + scan_bt_bridge) → bt_scan_probe（发 /v8/bt/goal）
#
# 与 v_scan_p3.sh 的差别：目标不再由探针直发 /scan_planner/goal，而是发
# /v8/bt/goal 由 BT 的 RequestPath 转发；到达权来自 SCAN 的
# /lite3/scan/arrived → 桥 → /v8/bt/arrived。判据沿用 A7 P3 口径
# （终点 ≤0.35 m，零自设），另加「BT outcome=reached」「SCAN arrived=True」
# 两条本集成必须成立的面。
#
# 退出码：0=PASS；1=FAIL；9=前置不成立（链没起来/topic 不在线）；2=环境不净。
set -u
export ROS_MASTER_URI=http://127.0.0.1:11313
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
RUN=${SCAN_BT_RUN:-/tmp/scan_bt_run}
LOG=$RUN/harness.log
mkdir -p "$RUN"
declare -a PIDS=()
say() { echo "[$(date '+%T')] $*" | tee -a "$LOG"; }
note() { echo "$*" | tee -a "$LOG"; }

set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
set -u
[ "${ROS_DISTRO:-}" = "noetic" ] || { echo "FATAL: ROS_DISTRO 非 noetic"; exit 2; }

wait_topic() {
  local t=$1 deadline=$((SECONDS + ${2:-300}))
  while [ $SECONDS -lt $deadline ]; do
    if rostopic list 2>/dev/null | grep -qx "$t"; then
      timeout 8 rostopic echo -n1 "$t" >/dev/null 2>&1 && return 0
    fi
    sleep 2
  done
  return 1
}

LOCK=${SCAN_BT_LOCK:-/tmp/lite3_scan_bt.lock}
if [ -f "$LOCK" ]; then
  OLD=$(cat "$LOCK" 2>/dev/null)
  if [ -n "$OLD" ] && kill -0 "$OLD" 2>/dev/null; then
    echo "SCAN-BT 结论 = INVALID|已有实例在跑（PID=$OLD）"
    exit 2
  fi
fi
echo $$ > "$LOCK"
cleanup() {
  say "-- teardown --"
  rm -f "$LOCK"
  rosnode kill -a >/dev/null 2>&1; sleep 2
  for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null; done
  pkill -f 'roslaunch.*m2_gazebo' 2>/dev/null; pkill -f 'roslaunch.*m2_mapping' 2>/dev/null
  pkill -f 'roslaunch.*70_scan' 2>/dev/null; pkill -f 'roslaunch.*60_safety' 2>/dev/null
  pkill -f 'roslaunch.*scan_bt_nav' 2>/dev/null; pkill -f 'roslaunch.*bt_console' 2>/dev/null
  pkill -f gzserver 2>/dev/null; pkill -f safe_cmd_vel_mux 2>/dev/null
  pkill -f scan_planner_node 2>/dev/null; pkill -f closed_loop_controller 2>/dev/null
  pkill -f scan_arrival_monitor 2>/dev/null; pkill -f scan_bt_bridge 2>/dev/null
  pkill -f bt_console 2>/dev/null; pkill -f 'topic_tools.*relay' 2>/dev/null
  sleep 3; kill "${ROSCORE_PID:-0}" 2>/dev/null; sleep 2
  note "残留进程: $(pgrep -f 'roscore|rosmaster|gzserver|roslaunch|scan_planner_node|closed_loop_controller|safe_cmd_vel_mux|scan_arrival_monitor|scan_bt_bridge|bt_console' | tr '\n' ' ')(空=净)"
  note "11313 监听: $(ss -lntp 2>/dev/null | grep -c ':11313')"
}
trap cleanup EXIT

say "== [0] 环境预检 =="
L13=$(ss -lntp 2>/dev/null | grep -c ':11313')
note "起前 11313 监听=$L13（须 0，否则先停旧栈）"
[ "$L13" = "0" ] || { echo "SCAN-BT 结论 = INVALID|11313 已被占用（旧栈未停）"; exit 2; }

say "== [1] roscore 11313 =="
roscore -p 11313 > "$RUN/roscore.log" 2>&1 &
ROSCORE_PID=$!
for _ in $(seq 1 30); do rostopic list >/dev/null 2>&1 && break; sleep 1; done

say "== [2] m2_gazebo =="
# SCAN_BT_WORLD：默认 m1_world（平地）；斜坡用 worlds/ramp_scan.world（平滑 10°）。
WORLD=${SCAN_BT_WORLD:-}
SPAWN_ARGS=()
[ -n "${SCAN_BT_SPAWN_X:-}" ] && SPAWN_ARGS+=("spawn_x:=$SCAN_BT_SPAWN_X")
[ -n "${SCAN_BT_SPAWN_Y:-}" ] && SPAWN_ARGS+=("spawn_y:=$SCAN_BT_SPAWN_Y")
[ -n "${SCAN_BT_SPAWN_Z:-}" ] && SPAWN_ARGS+=("spawn_z:=$SCAN_BT_SPAWN_Z")
[ -n "${SCAN_BT_SPAWN_YAW:-}" ] && SPAWN_ARGS+=("spawn_yaw:=$SCAN_BT_SPAWN_YAW")
if [ -n "$WORLD" ]; then
  note "world = $WORLD  spawn = ${SPAWN_ARGS[*]:-default}"
  roslaunch lite3_gazebo_sim m2_gazebo.launch gui:=false "${SPAWN_ARGS[@]}" world:="$WORLD" > "$RUN/gazebo.log" 2>&1 &
else
  roslaunch lite3_gazebo_sim m2_gazebo.launch gui:=false "${SPAWN_ARGS[@]}" > "$RUN/gazebo.log" 2>&1 &
fi
PIDS+=($!)
wait_topic /clock 300 || { echo "SCAN-BT 结论 = INVALID|/clock 未就绪"; exit 9; }
wait_topic /rslidar_points 480 || { echo "SCAN-BT 结论 = INVALID|/rslidar_points 未就绪"; exit 9; }
wait_topic /livox/imu 300 || { echo "SCAN-BT 结论 = INVALID|/livox/imu 未就绪"; exit 9; }

say "== [3] m2_mapping（faster_lio）=="
roslaunch lite3_gazebo_sim m2_mapping.launch > "$RUN/mapping.log" 2>&1 &
PIDS+=($!)
wait_topic /Odometry 360 || { echo "SCAN-BT 结论 = INVALID|/Odometry 未就绪"; exit 9; }

say "== [4] 60_safety（mux）=="
roslaunch lite3_safety 60_safety.launch > "$RUN/mux.log" 2>&1 &
PIDS+=($!)
wait_topic /cmd_vel 240 || { echo "SCAN-BT 结论 = INVALID|/cmd_vel 未就绪"; exit 9; }
# mux 的 speed_cap_required=true，无 /v8/safety/speed_cap 时按 blind 0.1m/s 限速；
# 仿真链没有超声/限速源，显式发 0.5（mux max_linear 也是 0.5）解除盲限速。
rostopic pub -r 2 /v8/safety/speed_cap std_msgs/Float32 "data: 0.5" > "$RUN/speed_cap.log" 2>&1 &
PIDS+=($!)

say "== [5] 70_scan =="
SCAN_ARGS=("navi_mode:=${SCAN_NAVI_MODE:-1}")
[ -n "${SCAN_MAX_VEL:-}" ] && SCAN_ARGS+=("max_vel:=$SCAN_MAX_VEL")
[ -n "${SCAN_CTRL_VX:-}" ] && SCAN_ARGS+=("ctrl_max_vx:=$SCAN_CTRL_VX")
[ -n "${SCAN_CTRL_VY:-}" ] && SCAN_ARGS+=("ctrl_max_vy:=$SCAN_CTRL_VY")
[ -n "${SCAN_ADV_PARAM:-}" ] && SCAN_ARGS+=("advanced_param:=$SCAN_ADV_PARAM")
[ "${SCAN_GROUND_FILTER:-false}" = "true" ] && SCAN_ARGS+=("ground_filter:=true" "cloud_topic:=/LIO/clouds_lidar_ground_filtered")
[ -n "${SCAN_GROUND_MIN_Z:-}" ] && SCAN_ARGS+=("ground_min_z:=$SCAN_GROUND_MIN_Z")
roslaunch lite3_scan 70_scan.launch "${SCAN_ARGS[@]}" > "$RUN/scan.log" 2>&1 &
PIDS+=($!)
wait_topic /LIO/odom_vehicle 240 || { echo "SCAN-BT 结论 = INVALID|/LIO/odom_vehicle 未就绪"; exit 9; }
wait_topic /grid_map/occupancy 240 || { echo "SCAN-BT 结论 = INVALID|/grid_map/occupancy 未就绪"; exit 9; }
wait_topic /lite3/scan/arrived 180 || { echo "SCAN-BT 结论 = INVALID|到达实体话题未就绪"; exit 9; }

say "== [6] scan_bt_nav（BT + 契约桥）=="
BT_ARGS=()
[ -n "${SCAN_BT_ARRIVAL_FLOOR:-}" ] && BT_ARGS+=("arrival_speed_floor_mps:=$SCAN_BT_ARRIVAL_FLOOR")
[ "${SCAN_REFERENCE_PATH:-false}" = "true" ] && BT_ARGS+=("publish_reference_path:=true")
roslaunch lite3_scan scan_bt_nav.launch "${BT_ARGS[@]}" > "$RUN/bt.log" 2>&1 &
PIDS+=($!)
wait_topic /v8/bt/status 120 || { echo "SCAN-BT 结论 = INVALID|/v8/bt/status 未就绪"; exit 9; }
wait_topic /global_path/status 60 || { echo "SCAN-BT 结论 = INVALID|/global_path/status 未就绪"; exit 9; }
note "节点: $(rosnode list 2>/dev/null | grep -cE 'bt_console|scan_bt_bridge|goal_gate|scan_planner_node|closed_loop_controller|scan_arrival_monitor|safe_cmd_vel_mux')/7"

say "== [7] BT 发目标并判到达 =="
PROBE=${SCAN_BT_PROBE:-$(dirname "$0")/bt_scan_probe.py}
note "probe = $PROBE"
python3 "$PROBE" > "$RUN/probe.json" 2>"$RUN/probe.err"
RC=$?
cat "$RUN/probe.json" | tee -a "$LOG"
[ -s "$RUN/probe.err" ] && tail -5 "$RUN/probe.err" | tee -a "$LOG"

case "$RC" in
  0) echo "SCAN-BT 结论 = PASS|BT 编排经 SCAN 规划/跟踪完成到达" | tee -a "$LOG";;
  9) echo "SCAN-BT 结论 = INVALID|前置不成立（见 probe.json）" | tee -a "$LOG";;
  *) echo "SCAN-BT 结论 = FAIL|BT/SCAN 集成未达成（见 probe.json）" | tee -a "$LOG";;
esac
exit "$RC"
