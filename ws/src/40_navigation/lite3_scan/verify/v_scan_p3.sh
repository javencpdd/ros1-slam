#!/usr/bin/env bash
# v_scan_p3.sh —— SCAN 集成 A7 判据③④⑤「动态闭环跟踪走完」harness（小明 2026-10-08，roadmap C #3 的 P3 波）
#
# 链：roscore → m2_gazebo → m2_mapping（faster_lio）→ **60_safety（mux，/cmd_vel 唯一写者）** → 70_scan → p3_track
#   与 v_scan_p1/p2 同一骨架（复用其 wait_topic D-M2-P4 式与 trap EXIT 退净），差别只有两处：
#   ①本批必须把 mux 起在链上——J3 第七源（/scan_planner/cmd_vel，priority 7）的活体面 C-1/C-2 与 P3 同窗，
#     小张 §4.2 β 的口径就是「SCAN 只在无其他导航源时被选中」，不起 mux 就证不到「谁在写 /cmd_vel」；
#   ②判据实体是小张为 O-1 补的到达观测件（/lite3/scan/arrived），不是 FSM 内部状态、不是 bspline 末点。
# 判定与门值全在 p3_track.py（0.35m／120s／三连照设计原文，本件不自设门）。
#
# 退出码（本件自有层，不与 SM／BR／RD 各层互认）：0=判据③④⑤ 全过；1=判据 FAIL；
#   9=前置不成立（链没起来／goal 找不到／到达实体不在线 ⇒ 不判被试）；2=环境或红线不过（并发未清等）。
set -u
export ROS_MASTER_URI=http://127.0.0.1:11313
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
RUN=${P3_RUN:-/tmp/scan_p3_run}
LOG=$RUN/harness.log
mkdir -p "$RUN"
declare -a PIDS=()
MUX_PID=""
say() { echo "[$(date '+%T')] $*" | tee -a "$LOG"; }
note() { echo "$*" | tee -a "$LOG"; }

set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
[ -f /home/ros1-slam/ws/devel/setup.bash ] && source /home/ros1-slam/ws/devel/setup.bash
set -u
[ "${ROS_DISTRO:-}" = "noetic" ] || { echo "FATAL: ROS_DISTRO=$ROS_DISTRO 非 noetic"; exit 2; }
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
  rosnode kill -a >/dev/null 2>&1; sleep 2
  for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null; done
  [ -n "$MUX_PID" ] && kill "$MUX_PID" 2>/dev/null
  pkill -f 'roslaunch.*m2_gazebo' 2>/dev/null; pkill -f 'roslaunch.*m2_mapping' 2>/dev/null
  pkill -f 'roslaunch.*70_scan' 2>/dev/null; pkill -f 'roslaunch.*60_safety' 2>/dev/null
  pkill -f gzserver 2>/dev/null; pkill -f safe_cmd_vel_mux 2>/dev/null
  pkill -f scan_planner_node 2>/dev/null; pkill -f closed_loop_controller 2>/dev/null
  pkill -f scan_arrival_monitor 2>/dev/null; pkill -f 'topic_tools.*relay' 2>/dev/null
  sleep 3; kill "${ROSCORE_PID:-0}" 2>/dev/null; sleep 2
  note "残留进程: $(pgrep -f 'roscore|rosmaster|gzserver|roslaunch|scan_planner_node|closed_loop_controller|safe_cmd_vel_mux|scan_arrival_monitor' | tr '\n' ' ')(空=净)"
  note "11313 监听: $(ss -lntp 2>/dev/null | grep -c ':11313')  11312 监听: $(ss -lntp 2>/dev/null | grep -c ':11312')"
  sha256sum /home/ros1-slam/data/maps/lite3.pcd | tee "$RUN/pcd_after.sha"
}
trap cleanup EXIT

say "== [0] 红线预检：并发／样张／foxy =="
ps -o pid,user,lstart,etime,comm --no-headers -p 115711,115792,115795,115797,115799,180237 2>/dev/null | tee -a "$LOG"
L13=$(ss -lntp 2>/dev/null | grep -c ':11313'); L12=$(ss -lntp 2>/dev/null | grep -c ':11312')
note "在册端口起前监听：11313=$L13 11312=$L12（均须 0，否则并发未清 ⇒ rc=2 不跑）"
[ "$L13" = "0" ] && [ "$L12" = "0" ] || { echo "P3 判据③ 结论 = INVALID|并发未清（11313=$L13 11312=$L12）"; exit 2; }
sha256sum /home/ros1-slam/data/maps/lite3.pcd | tee "$RUN/pcd_before.sha"

say "== [1] roscore 11313 =="
roscore -p 11313 > "$RUN/roscore.log" 2>&1 &
ROSCORE_PID=$!
for _ in $(seq 1 30); do rostopic list >/dev/null 2>&1 && break; sleep 1; done

say "== [2] m2_gazebo =="
roslaunch lite3_gazebo_sim m2_gazebo.launch gui:=false > "$RUN/gazebo.log" 2>&1 &
PIDS+=($!)
wait_topic /clock 300 || { echo "P3 判据③ 结论 = INVALID|/clock 未就绪"; exit 9; }
wait_topic /rslidar_points 480 || { echo "P3 判据③ 结论 = INVALID|/rslidar_points 未就绪"; exit 9; }
wait_topic /livox/imu 300 || { echo "P3 判据③ 结论 = INVALID|/livox/imu 未就绪"; exit 9; }

say "== [3] m2_mapping（faster_lio 位姿源）=="
roslaunch lite3_gazebo_sim m2_mapping.launch > "$RUN/mapping.log" 2>&1 &
PIDS+=($!)
wait_topic /Odometry 360 || { echo "P3 判据③ 结论 = INVALID|/Odometry 未就绪"; exit 9; }
# /LIO/odom_vehicle 不在这一步：它是 70_scan 内 lio_relay_sim 的三个 relay 之一（odom_body_out 默认值即此名），
#   首版把这条 wait 放在 mapping 段 ⇒ 6 分钟空等后打 INVALID（窗白烧）。这一族的通用形＝wait 的话题必须在其产生段之后。
note "本段只等 /Odometry；/LIO/odom_vehicle 归 [5] 段（relay 在 70_scan 里）"

say "== [4] 60_safety（mux 上链，/cmd_vel 唯一写者）=="
roslaunch lite3_safety 60_safety.launch > "$RUN/mux.log" 2>&1 &
PIDS+=($!)
MUX_PID=$(pgrep -f 'safe_cmd_vel_mux' | head -1)
wait_topic /cmd_vel 240 || { echo "P3 判据③ 结论 = INVALID|/cmd_vel 未就绪（mux 未起）"; exit 9; }
note "mux 属主 PID（起链时取，仅供 C-2 kill 用）：$MUX_PID"
note "C-1 静态面：/cmd_vel Publishers＝$(timeout 8 rostopic info /cmd_vel 2>/dev/null | sed -n '/Publishers:/,$p' | grep -o '/[a-zA-Z0-9_/]*' | tr '\n' ' ')"

say "== [5] 70_scan.launch（navi_mode:=1，含 J1 到达观测件）=="
roslaunch lite3_scan 70_scan.launch navi_mode:=1 > "$RUN/scan.log" 2>&1 &
PIDS+=($!)
wait_topic /LIO/clouds_lidar 240 || { echo "P3 判据③ 结论 = INVALID|/LIO/clouds_lidar 未就绪"; exit 9; }
wait_topic /grid_map/occupancy 240 || { echo "P3 判据③ 结论 = INVALID|/grid_map/occupancy 未就绪"; exit 9; }
wait_topic /LIO/odom_vehicle 180 || { echo "P3 判据③ 结论 = INVALID|/LIO/odom_vehicle 不在线（relay 未起 ⇒ 到达参考位姿源缺失）"; exit 9; }
wait_topic /lite3/scan/arrived 180 || { echo "P3 判据③ 结论 = INVALID|到达实体话题不在线（J1 观测件未起 ⇒ 判据无载体）"; exit 9; }
note "F9 互斥（节点级）: move_base/teb 节点数=$(rosnode list 2>/dev/null | grep -cE 'move_base|/teb') 须=0"
note "链上节点: $(rosnode list 2>/dev/null | grep -cE 'scan_planner_node|closed_loop_controller|scan_arrival_monitor|safe_cmd_vel_mux')/4 须=4"

say "== [6] 判据③④⑤ 取证（三连在同一个 sim 窗内跑，逐跑 goal 重选）=="
python3 /home/ros1-slam/ws/src/40_navigation/lite3_scan/verify/p3_track.py > "$RUN/p3.json" 2>"$RUN/p3.err"
RC=$?
note "p3 python rc=$RC"
cat "$RUN/p3.json" | tee -a "$LOG"
[ -s "$RUN/p3.err" ] && { note "stderr 尾:"; tail -6 "$RUN/p3.err" | tee -a "$LOG"; }
V=$(grep -oE 'P3 判据③ 结论 = [A-Z]+\|.*' "$RUN/p3.json" | tail -1)
[ -z "$V" ] && { echo "P3 判据③ 结论 = INVALID|结论行不可指认（rc=$RC 但无结论行 ⇒ 不采信）"; exit 9; }
note "结论行：$V"

say "== [7] C-1/C-2 mux 活体面（与 P3 同窗）=="
C1=$(timeout 8 rostopic info /cmd_vel 2>/dev/null | sed -n '/Publishers:/,/Subscribers:/p' | grep -c '/safe_cmd_vel_mux')
C1O=$(timeout 8 rostopic info /cmd_vel 2>/dev/null | sed -n '/Publishers:/,/Subscribers:/p' | grep -cE '^ \* /')
note "C-1 车动窗后 /cmd_vel 写者：mux 命中=$C1 写者总数=$C1O（均须恰 1／1，§2.6 硬规则 a）"
HZ_ON=$(timeout 6 rostopic hz /cmd_vel 2>/dev/null | grep -oE 'average rate: [0-9.]+' | tail -1 | awk '{print $3}')
note "C-2 kill 前 /cmd_vel 平均速率＝${HZ_ON:-不可测}（mux 恒 20Hz）"
[ -n "$MUX_PID" ] && kill "$MUX_PID" 2>/dev/null
sleep 3
HZ_OFF=$(timeout 6 rostopic hz /cmd_vel 2>/dev/null | grep -c 'average rate')
note "C-2 kill 后 6s 内速率行数＝$HZ_OFF（须 0 ⇒ /cmd_vel 停发；非 0 说明还有别的写者）"

say "== [8] 退净与红线 =="
[ -s "$RUN/pcd_before.sha" ] && [ -s "$RUN/pcd_after.sha" ] && note "样张 sha 起前＝退后：$( [ "$(cut -d' ' -f1 "$RUN/pcd_before.sha")" = "$(cd "$RUN" && sha256sum /home/ros1-slam/data/maps/lite3.pcd | cut -d' ' -f1)" ] && echo 是 || echo 否 )"
note "teardown 后的监听与残留见 trap 段"

case "$V" in
  PASS\|*) RC3=0;;
  INVALID\|*) RC3=9;;
  *) RC3=1;;
esac
if [ "$RC3" = "0" ] && { [ "$C1" != "1" ] || [ "$C1O" != "1" ] || [ "$HZ_OFF" != "0" ]; }; then
  note "⇒ 判据③④⑤ 过但 C-1/C-2 不过 ⇒ mux 唯一写者／停发未证成 ⇒ 本件整体判 FAIL"
  RC3=1
fi
echo "P3 判据③④⑤ 与 C-1/C-2 联合结论 = $([ "$RC3" = "0" ] && echo PASS || echo "FAIL/INVALID rc=$RC3")｜判据行=$V｜C1写者=$C1/$C1O｜C2停发行数=$HZ_OFF"
exit "$RC3"
