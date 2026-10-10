#!/usr/bin/env bash
# v_scan_p2.sh —— SCAN 集成 A7 P2 判据② harness（主理人直接执行，2026-10-08）
# 判据②：发 goal 后 ≤5s 内 FSM 出轨迹（/planning/bspline），且曲线采样点不穿墙。
# 附带 §C 勘误项的话题级确认：goal 订阅归属必须是 /scan_planner/goal 而非 /move_base_simple/goal。
# 链与退净骨架同 v_scan_p1.sh（复用 m2_smoke.sh 的 wait_topic D-M2-P4 + trap EXIT）。
set -u
export ROS_MASTER_URI=http://127.0.0.1:11313
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
RUN=/tmp/scan_p2_run
LOG=$RUN/harness.log
mkdir -p "$RUN"
declare -a PIDS=()

say() { echo "[$(date '+%T')] $*" | tee -a "$LOG"; }
note() { echo "$*" | tee -a "$LOG"; }

# 环境：catkin 的 1.ros_distro.sh 在未设 ROS_DISTRO 时引用 $ROS_DISTRO，与 set -u 冲突 ⇒ source 段临时关 -u
set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
[ -f /home/ros1-slam/ws/devel/setup.bash ] && source /home/ros1-slam/ws/devel/setup.bash
set -u
[ "${ROS_DISTRO:-}" = "noetic" ] || { echo "FATAL: ROS_DISTRO=$ROS_DISTRO 非 noetic"; exit 2; }
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:/home/ros1-slam/ws/devel/lib   # 缺此行 Mid360 插件不加载

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

cleanup() {
  say "-- teardown --"
  rosnode kill -a >/dev/null 2>&1; sleep 2
  for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null; done
  pkill -f 'roslaunch.*m2_gazebo' 2>/dev/null; pkill -f 'roslaunch.*m2_mapping' 2>/dev/null
  pkill -f 'roslaunch.*70_scan' 2>/dev/null
  pkill -f gzserver 2>/dev/null; pkill -f sim_adapter 2>/dev/null
  pkill -f run_mapping_online 2>/dev/null; pkill -f scan_planner_node 2>/dev/null
  pkill -f closed_loop_controller 2>/dev/null; pkill -f 'topic_tools.*relay' 2>/dev/null
  sleep 3; kill "${ROSCORE_PID:-0}" 2>/dev/null; sleep 2
  note "残留进程: $(pgrep -f 'roscore|rosmaster|gzserver|roslaunch|run_mapping_online|scan_planner_node|closed_loop_controller' | tr '\n' ' ')(空=净)"
  note "11313 监听: $(ss -lntp 2>/dev/null | grep -c ':11313')  11312 监听: $(ss -lntp 2>/dev/null | grep -c ':11312')"
}
trap cleanup EXIT

say "== [0] P0.5 双查 + 基线 =="
# A1（2026-10-10 已裁＝乙＋丙同落）：见证基准每批现取，不沿用写死 PID
WITNESS=""
ps -eo pid,user,lstart,args | awk '/ros2qnx|message_transformer|lite3_ros_bridge|jetson2motion|jetson2app|transfer/ && !/awk/ {print}' > "$RUN/foxy_start.txt" 2>/dev/null
FOXY_N=$(grep -c "[0-9]" "$RUN/foxy_start.txt" 2>/dev/null); FOXY_N=${FOXY_N:-0}
note "foxy 见证基准（A1 乙＝本 boot 现取）：命中 $FOXY_N 行 ⇒ $RUN/foxy_start.txt"
[ "$FOXY_N" = "0" ] && { WITNESS="INVALID-空集"; note "  [INVALID] 基准空集（A1 丙）⇒ 本跑不得写零触碰"; }
sed "s/^/    /" "$RUN/foxy_start.txt" 2>/dev/null | tee -a "$LOG"
note "11312 监听(起前): $(ss -lntp 2>/dev/null | grep -c ':11312') 须=0"
note "ros2qnx: $(pgrep -fa ros2qnx | wc -l) 须=0"
sha256sum /home/ros1-slam/data/maps/lite3.pcd | tee "$RUN/pcd_before.sha"

say "== [1] roscore 11313 =="
roscore -p 11313 > "$RUN/roscore.log" 2>&1 &
ROSCORE_PID=$!
for _ in $(seq 1 30); do rostopic list >/dev/null 2>&1 && break; sleep 1; done

say "== [2] m2_gazebo =="
roslaunch lite3_gazebo_sim m2_gazebo.launch gui:=false > "$RUN/gazebo.log" 2>&1 &
PIDS+=($!)
wait_topic /clock 300 || { echo "FAIL: /clock"; exit 1; }
wait_topic /rslidar_points 480 || { echo "FAIL: /rslidar_points"; exit 1; }
wait_topic /livox/imu 300 || { echo "FAIL: /livox/imu"; exit 1; }

say "== [3] m2_mapping (pcd_save:=false) =="
roslaunch lite3_gazebo_sim m2_mapping.launch > "$RUN/mapping.log" 2>&1 &
PIDS+=($!)
wait_topic /Odometry 360 || { echo "FAIL: /Odometry"; exit 1; }
wait_topic /cloud_registered_body 360 || { echo "FAIL: /cloud_registered_body"; exit 1; }

say "== [4] 70_scan.launch =="
roslaunch lite3_scan 70_scan.launch navi_mode:=1 > "$RUN/scan.log" 2>&1 &
PIDS+=($!)
wait_topic /LIO/clouds_lidar 240 || { echo "FAIL: /LIO/clouds_lidar"; tail -25 "$RUN/scan.log" | tee -a "$LOG"; exit 1; }
wait_topic /grid_map/occupancy 240 || { echo "FAIL: /grid_map/occupancy"; tail -25 "$RUN/scan.log" | tee -a "$LOG"; exit 1; }
note "F9 互斥（节点级）: move_base 节点数=$(rosnode list 2>/dev/null | grep -cE 'move_base|/teb') 须=0"

say "== [5] 判据② 取证 =="
python3 /home/ros1-slam/ws/src/40_navigation/lite3_scan/verify/p2_goal_plan.py > "$RUN/p2.json" 2>"$RUN/p2.err"
RC=$?
note "p2 python rc=$RC"
cat "$RUN/p2.json" | tee -a "$LOG"
[ -s "$RUN/p2.err" ] && { note "stderr 尾:"; tail -6 "$RUN/p2.err" | tee -a "$LOG"; }

say "== [6] 判定（fail-closed） =="
V=$(python3 - "$RUN/p2.json" <<'PY'
import json,sys
try: d=json.load(open(sys.argv[1]))
except Exception as e: print("FAIL|occ.json 不可读(%s)"%type(e).__name__); sys.exit(0)
c=d.get("checks") or {}
if not c: print("FAIL|无任何判据项"); sys.exit(0)
bad=[k for k,v in c.items() if not v]
print(("PASS|" if (d.get("verdict")=="PASS" and not bad) else "FAIL|") + ("全部 %d 项通过" % len(c) if not bad else "未过: "+", ".join(bad)))
PY
)
[ -z "$V" ] && V="FAIL|判定脚本无输出"
note "判定: $V"
sha256sum /home/ros1-slam/data/maps/lite3.pcd > "$RUN/pcd_after.sha"
diff -q "$RUN/pcd_before.sha" "$RUN/pcd_after.sha" >/dev/null 2>&1 && note "样张 sha256 起前=退后 OK" || note "★样张变化"

note "foxy 见证终检（与起前现取基准整行比对）:"
ps -eo pid,user,lstart,args | awk '/ros2qnx|message_transformer|lite3_ros_bridge|jetson2motion|jetson2app|transfer/ && !/awk/ {print}' > "$RUN/foxy_end.txt" 2>/dev/null
if [ -n "$WITNESS" ]; then note "红线见证 = $WITNESS（本跑不作零触碰声称）"
elif [ ! -s "$RUN/foxy_end.txt" ]; then WITNESS="INVALID-退后空集"; note "★[INVALID] 退后空集而起前非空 ⇒ 不作零触碰声称"
elif diff -q "$RUN/foxy_start.txt" "$RUN/foxy_end.txt" >/dev/null 2>&1; then WITNESS="OK"; note "红线见证 = OK（$FOXY_N 行整行逐位一致）"
else WITNESS="DISTURBED"; note "★红线见证 = DISTURBED ⇒ 本跑不作达成声称"; diff "$RUN/foxy_start.txt" "$RUN/foxy_end.txt" | head -4 | sed "s/^/    /" | tee -a "$LOG"
fi
echo "=================================================="
case "$V" in PASS\|*) echo "P2 判据② 结论 = PASS —— ${V#PASS|}"; RC2=0;; *) echo "P2 判据② 结论 = FAIL —— ${V#FAIL|}"; RC2=1;; esac
echo "P2_EXIT=$RC2 WITNESS=${WITNESS:-未检} LOGDIR=$RUN"
exit "$RC2"
