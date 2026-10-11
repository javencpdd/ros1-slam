#!/usr/bin/env bash
# v_scan_p4.sh —— SCAN 集成 A7 P4「重规划（丢 1 块动态障碍）」harness（小明 2026-10-08）
#
# 链与 v_scan_p1/p2/p3 同骨架（各件自带链是本域既有风格：P1、P2 也都是各起各的链），差别三处：
#   ①mux 必须在链上（P3 起就为 C-1/C-2 上链，P4 的 cmd 出口同样只走 /cmd_vel）；
#   ②扰动用**运行时搬动**世界里现成的 obs_5（/gazebo/set_model_state），m1_world.world 文件一字不动，
#     跑完再取回模型位姿自证还原（还原不成就把本面判 INVALID，不给「世界被我改了」留活口）；
#   ③判定与门值全在 p4_replan.py（离障阈 0.10m 取 P2 原文、终点 0.35m／dt 120s 取 A7 判据③④ 原文，本件不自设）。
# 三档分明：INVALID（前置/投递/扰动可见性）＝9；FAIL＝1；PASS＝0。结论行不可指认即不采信 rc（R44 口径）。
#
# 用法：（先按三步环境 source）bash v_scan_p4.sh      RUN=${P4_RUN:-/tmp/scan_p4_run}
set -u
export ROS_MASTER_URI=http://127.0.0.1:11313
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
RUN=${P4_RUN:-/tmp/scan_p4_run}
LOG=$RUN/harness.log
WORLD=/home/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim/worlds/m1_world.world
mkdir -p "$RUN"
declare -a PIDS=()
say() { echo "[$(date '+%T')] $*" | tee -a "$LOG"; }
note() { echo "$*" | tee -a "$LOG"; }

LOCK=${SCAN_HARNESS_LOCK:-/tmp/lite3_scan_harness.lock}
if [ -f "$LOCK" ]; then
  OLD=$(cat "$LOCK" 2>/dev/null)
  if [ -n "$OLD" ] && kill -0 "$OLD" 2>/dev/null; then
    echo "P4 重规划结论 = INVALID|已有链上实例在跑（PID=$OLD）⇒ 并发不自证，本件拒跑（rc=2）"; exit 2
  fi
  note "陈旧锁接管（旧值=$OLD）"
fi
echo $$ > "$LOCK"

set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
[ -f /home/ros1-slam/ws/devel/setup.bash ] && source /home/ros1-slam/ws/devel/setup.bash
set -u
[ "${ROS_DISTRO:-}" = "noetic" ] || { echo "FATAL: ROS_DISTRO=$ROS_DISTRO 非 noetic"; rm -f "$LOCK"; exit 2; }
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:/home/ros1-slam/ws/devel/lib

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
  rm -f "$LOCK"
  rosnode kill -a >/dev/null 2>&1; sleep 2
  for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null; done
  pkill -f 'roslaunch.*m2_gazebo' 2>/dev/null; pkill -f 'roslaunch.*m2_mapping' 2>/dev/null
  pkill -f 'roslaunch.*70_scan' 2>/dev/null; pkill -f 'roslaunch.*60_safety' 2>/dev/null
  pkill -f gzserver 2>/dev/null; pkill -f safe_cmd_vel_mux 2>/dev/null
  pkill -f scan_planner_node 2>/dev/null; pkill -f closed_loop_controller 2>/dev/null
  pkill -f scan_arrival_monitor 2>/dev/null; pkill -f 'topic_tools.*relay' 2>/dev/null
  sleep 3; kill "${ROSCORE_PID:-0}" 2>/dev/null; sleep 2
  note "残留进程: $(pgrep -f 'roscore|rosmaster|gzserver|roslaunch|scan_planner_node|closed_loop_controller|safe_cmd_vel_mux|scan_arrival_monitor' | tr '\n' ' ')(空=净)"
  note "11313 监听: $(ss -lntp 2>/dev/null | grep -c ':11313')  11312 监听: $(ss -lntp 2>/dev/null | grep -c ':11312')"
  sha256sum "$WORLD" | tee "$RUN/world_after.sha"
  sha256sum /home/ros1-slam/data/maps/lite3.pcd | tee "$RUN/pcd_after.sha"
}
trap cleanup EXIT

say "== [0] 红线预检：并发／样张／世界件本体 =="
# A1（2026-10-11 已裁＝乙＋丙同落）：红线基准每批现取；旧写死名单（115711…180237）一重启即读空＝把没见证当绿
WITNESS=""
note "foxy 见证基准（A1 乙＝本 boot 现取）:"
ps -eo pid,user,lstart,args | awk '/ros2qnx|message_transformer|lite3_ros_bridge|jetson2motion|jetson2app|transfer/ && !/awk|grep|v_scan_p4/ {print}' > "$RUN/foxy_start.txt" 2>/dev/null
FOXY_N=$(grep -c "[0-9]" "$RUN/foxy_start.txt" 2>/dev/null); FOXY_N=${FOXY_N:-0}
note "  现取命中 $FOXY_N 行 ⇒ $RUN/foxy_start.txt"
[ "$FOXY_N" = "0" ] && { WITNESS="INVALID-空集"; note "  [INVALID] 基准空集（A1 丙）⇒ 本窗不得写零触碰、不得引用为红线达成"; }
L13=$(ss -lntp 2>/dev/null | grep -c ':11313'); L12=$(ss -lntp 2>/dev/null | grep -c ':11312')
note "在册端口起前监听：11313=$L13 11312=$L12（均须 0）"
[ "$L13" = "0" ] && [ "$L12" = "0" ] || { echo "P4 重规划结论 = INVALID|并发未清（11313=$L13 11312=$L12）"; exit 2; }
sha256sum /home/ros1-slam/data/maps/lite3.pcd | tee "$RUN/pcd_before.sha"
sha256sum "$WORLD" | tee "$RUN/world_before.sha"

say "== [1] roscore 11313 =="
roscore -p 11313 > "$RUN/roscore.log" 2>&1 &
ROSCORE_PID=$!
for _ in $(seq 1 30); do rostopic list >/dev/null 2>&1 && break; sleep 1; done

say "== [2] m2_gazebo =="
roslaunch lite3_gazebo_sim m2_gazebo.launch gui:=false > "$RUN/gazebo.log" 2>&1 &
PIDS+=($!)
wait_topic /clock 300 || { echo "P4 重规划结论 = INVALID|/clock 未就绪"; exit 9; }
wait_topic /rslidar_points 480 || { echo "P4 重规划结论 = INVALID|/rslidar_points 未就绪"; exit 9; }
wait_topic /livox/imu 300 || { echo "P4 重规划结论 = INVALID|/livox/imu 未就绪"; exit 9; }

say "== [3] m2_mapping（faster_lio 位姿源）=="
roslaunch lite3_gazebo_sim m2_mapping.launch > "$RUN/mapping.log" 2>&1 &
PIDS+=($!)
wait_topic /Odometry 360 || { echo "P4 重规划结论 = INVALID|/Odometry 未就绪"; exit 9; }

say "== [4] 60_safety（mux 上链）=="
roslaunch lite3_safety 60_safety.launch > "$RUN/mux.log" 2>&1 &
PIDS+=($!)
wait_topic /cmd_vel 240 || { echo "P4 重规划结论 = INVALID|/cmd_vel 未就绪（mux 未起）"; exit 9; }
note "C-1 面：/cmd_vel Publishers＝$(timeout 8 rostopic info /cmd_vel 2>/dev/null | sed -n '/Publishers:/,/Subscribers:/p' | grep -oE '^ \* /[^ ]*' | tr '\n' ' ')"

say "== [5] 70_scan.launch（含 J1 到达观测件）=="
roslaunch lite3_scan 70_scan.launch navi_mode:=1 > "$RUN/scan.log" 2>&1 &
PIDS+=($!)
wait_topic /LIO/odom_vehicle 240 || { echo "P4 重规划结论 = INVALID|/LIO/odom_vehicle 不在线（relay 未起）"; exit 9; }
wait_topic /grid_map/occupancy 240 || { echo "P4 重规划结论 = INVALID|/grid_map/occupancy 未就绪"; exit 9; }
wait_topic /lite3/scan/arrived 180 || { echo "P4 重规划结论 = INVALID|到达实体话题不在线"; exit 9; }
note "F9 互斥: move_base/teb 节点数=$(rosnode list 2>/dev/null | grep -cE 'move_base|/teb') 须=0；链上四件=$(rosnode list 2>/dev/null | grep -cE 'scan_planner_node|closed_loop_controller|scan_arrival_monitor|safe_cmd_vel_mux')/4"
note "扰动前模型位姿自证：$(timeout 10 rosservice call /gazebo/get_model_state "model_name: '${P4_MODEL:-obs_5}' reference_frame: 'world'" 2>/dev/null | tr '\n' ' ' | tr '\n' ' ' | sed -n 's/.*position:  *x: \([-0-9.]*\)  *y: \([-0-9.]*\).*/x=\\1 y=\\2/p')"

say "== [6] P4 取证（投障碍→看见→重规划→仍到达）=="
# R220（主理人 2026-10-12 裁"15 号两者都撤"）：撤销 R135 的分件形状——判据行与 JSON 同走 stdout，
#   整份收进 p4.json ⇒ 首行是中文结论句、**整份不是合法 JSON**（这是裁定的已知代价，不是本 harness 的缺陷）。
#   取证件内部仍保单一真值（机读字段由判据行反推），故 R124／R125 那种"字段与判据行反向"不会回来。
python3 /home/ros1-slam/ws/src/40_navigation/lite3_scan/verify/p4_replan.py > "$RUN/p4.json" 2>"$RUN/p4.err"
RC=$?
note "p4 python rc=$RC"
cat "$RUN/p4.json" | tee -a "$LOG"
[ -s "$RUN/p4.err" ] && { note "stderr 尾:"; tail -8 "$RUN/p4.err" | tee -a "$LOG"; }
V=$(grep -oE 'P4 重规划结论 = [A-Z]+\|.*' "$RUN/p4.json" | tail -1)
VS=${V#*= }
[ -z "$V" ] && { echo "P4 重规划结论 = INVALID|结论行不可指认（rc=$RC 但无结论行 ⇒ 不采信）"; exit 9; }
note "结论行：$V"
# 同向自证段随 R135 分件一并撤销（R220，主理人 2026-10-12 裁"两者都撤"）：
#   自证要比"判据行 vs p4.json 机读字段"两份东西，现在只剩一份 ⇒ 留着它只会让每窗恒定 rc=9 假阳性。
#   防线没有消失：取证件的单一真值仍在（字段由判据行反推），"两处各自维护"这一型结构上不可能发生。

say "== [7] 还原与红线 =="
note "扰动后模型位姿自证：$(timeout 10 rosservice call /gazebo/get_model_state "model_name: '${P4_MODEL:-obs_5}' reference_frame: 'world'" 2>/dev/null | tr '\n' ' ' | tr '\n' ' ' | sed -n 's/.*position:  *x: \([-0-9.]*\)  *y: \([-0-9.]*\).*/x=\\1 y=\\2/p')"
WB=$(cut -d' ' -f1 "$RUN/world_before.sha"); WA=$(sha256sum "$WORLD" | cut -d' ' -f1)
note "世界件本体 sha 起前＝退后：$( [ "$WB" = "$WA" ] && echo 是 || echo "否!!（$WB → $WA）" )"
PB=$(cut -d' ' -f1 "$RUN/pcd_before.sha")
PD=$(sha256sum /home/ros1-slam/data/maps/lite3.pcd | cut -d' ' -f1)
note "样张 sha 起前＝退后：$( [ "$PB" = "$PD" ] && echo 是 || echo "否!!" )"
# 终检放在结论行之前（R117 在 p2 上犯过：写在 trap 里会跑在结论之后，把 WITNESS 报成「未检」）
note "foxy 见证终检（与起前现取基准整行比对）:"
ps -eo pid,user,lstart,args | awk '/ros2qnx|message_transformer|lite3_ros_bridge|jetson2motion|jetson2app|transfer/ && !/awk|grep|v_scan_p4/ {print}' > "$RUN/foxy_end.txt" 2>/dev/null
if [ -n "$WITNESS" ]; then note "  红线见证 = $WITNESS（本窗不作零触碰声称）"
elif [ ! -s "$RUN/foxy_end.txt" ]; then WITNESS="INVALID-退后空集"; note "  [INVALID] 退后空集而起前非空 ⇒ 不作零触碰声称"
elif diff -q "$RUN/foxy_start.txt" "$RUN/foxy_end.txt" >/dev/null 2>&1; then WITNESS="OK"; note "  红线见证 = OK（$FOXY_N 行整行 pid＋属主＋lstart＋args 逐位一致）"
else WITNESS="DISTURBED"; note "  ★红线见证 = DISTURBED（起终整行不一致）⇒ 本窗不作达成声称"; diff "$RUN/foxy_start.txt" "$RUN/foxy_end.txt" | head -4 | sed "s/^/    /" | tee -a "$LOG"
fi

case "$VS" in
  PASS\|*) RC4=0;;
  INVALID\|*) RC4=9;;
  FAIL\|*) RC4=1;;
  *) note "⇒ 判据态不可指认 ⇒ 按前置不成立"; RC4=9;;
esac
if [ "$WB" != "$WA" ]; then note "⇒ 世界件本体被改动 ⇒ 本面不可引用"; RC4=1; fi
note "分档自证：VS 首段=${VS%%|*} ⇒ RC4=$RC4（判据行与末判必须同向）"
echo "P4 结论 = $([ "$RC4" = "0" ] && echo PASS || echo "rc=$RC4")｜WITNESS=${WITNESS:-未检}｜判据行=$V｜世界件未变=$([ "$WB" = "$WA" ] && echo 是 || echo 否)"
exit "$RC4"
