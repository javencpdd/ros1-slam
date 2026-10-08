#!/usr/bin/env bash
# v_scan_task_live.sh —— M5.2 scan_navigate 的活体面 harness（小明 2026-10-08）
#
# 链＝v_scan_p3/p4 同骨架（本域风格：每个 harness 自带链），末段把任务层节点起起来，
#   把它的 screen 输出原样收进 log，然后**把判定交回 v_task_scan_primitive.sh 的段 B**——
#   判据只有一份定义，不在这里另写一套（她 R46／B1 那一族：判据多副本必漂移）。
# goal 取 A7 P3 活体成功过的绝对坐标（默认 -2.72, 0.76，见 raw/scan_p3_run8_jointPASS/p3.json），
#   目的是只测「任务层能不能驱动并验到达」，不测取信；取信面属 P3/P4 的已判范围。
# 用法：bash v_scan_task_live.sh      环境变量：TASK_GOAL_X/TASK_GOAL_Y/TASK_WALL_TIMEOUT/TASK_RUN
set -u
export ROS_MASTER_URI=http://127.0.0.1:11313
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
RUN=${TASK_RUN:-/tmp/scan_task_run}
LOG=$RUN/harness.log
GX=${TASK_GOAL_X:--2.72}; GY=${TASK_GOAL_Y:-0.76}
WALL=${TASK_WALL_TIMEOUT:-600}
mkdir -p "$RUN"
declare -a PIDS=()
say(){ echo "[$(date '+%T')] $*" | tee -a "$LOG"; }
note(){ echo "$*" | tee -a "$LOG"; }
LOCK=${SCAN_HARNESS_LOCK:-/tmp/lite3_scan_harness.lock}
if [ -f "$LOCK" ] && kill -0 "$(cat "$LOCK" 2>/dev/null)" 2>/dev/null; then
  echo "M5.2 活体结论 = INVALID|已有链上实例在跑（PID=$(cat "$LOCK")）⇒ 拒跑"; exit 2
fi
echo $$ > "$LOCK"
set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash; source /home/ros1-slam/ws/devel/setup.bash
set -u
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:/home/ros1-slam/ws/devel/lib
wait_topic(){ local t=$1 d=$((SECONDS + ${2:-300})); while [ $SECONDS -lt $d ]; do rostopic list 2>/dev/null | grep -qx "$t" && { timeout 8 rostopic echo -n1 "$t" >/dev/null 2>&1 && return 0; }; sleep 2; done; return 1; }
cleanup(){ say "-- teardown --"; rm -f "$LOCK"; rosnode kill -a >/dev/null 2>&1; sleep 2
  for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null; done
  pkill -f 'roslaunch.*m2_gazebo' 2>/dev/null; pkill -f 'roslaunch.*m2_mapping' 2>/dev/null
  pkill -f 'roslaunch.*70_scan' 2>/dev/null; pkill -f 'roslaunch.*60_safety' 2>/dev/null
  pkill -f 'roslaunch.*50_task_smach_min' 2>/dev/null; pkill -f gzserver 2>/dev/null
  pkill -f safe_cmd_vel_mux 2>/dev/null; pkill -f scan_planner_node 2>/dev/null
  pkill -f closed_loop_controller 2>/dev/null; pkill -f scan_arrival_monitor 2>/dev/null
  pkill -f 'topic_tools.*relay' 2>/dev/null; sleep 3; kill "${ROSCORE_PID:-0}" 2>/dev/null; sleep 2
  # 残留必须按 comm 判，不能按命令行 -f 判：-f 会把「执行这条检查的 shell 自己」与刚被 kill 的 roslaunch 收尾件数进来
  #   （22:57 那次打了「残留进程: 10033 17031」，事后 ps 查无此件＝自匹配假报）。
  RES=$(ps -eo comm | awk '$1 ~ /^(gzserver|gzclient|rosmaster|roslaunch|rosout|python3)$/ && $1!="python3"' | tr -s '\n' ' ')
  note "残留进程(comm 面)=[$RES]（须空）；沙箱端口=$(ss -lntp 2>/dev/null | grep -cE ':11313|:11345')（须 0）"
  note "11313 监听=$(ss -lntp 2>/dev/null | grep -c ':11313')  11312 监听=$(ss -lntp 2>/dev/null | grep -c ':11312')"
  sha256sum /home/ros1-slam/data/maps/lite3.pcd | tee "$RUN/pcd_after.sha"; }
trap cleanup EXIT

say "== [0] 预检：并发／样张 =="
[ "$(ss -lntp 2>/dev/null | grep -cE ':1131[1-9]|:11345')" = "0" ] || { echo "M5.2 活体结论 = INVALID|并发未清"; exit 2; }
sha256sum /home/ros1-slam/data/maps/lite3.pcd | tee "$RUN/pcd_before.sha"
note "goal 取件＝($GX, $GY)（A7 P3 活体成功坐标）；任务节点墙钟上限 ${WALL}s；use_sim_time:=true（dt 与 A7 判据④ 同基）"
say "== [1..5] 起链（gazebo→mapping→mux→70_scan）=="
roscore -p 11313 > "$RUN/roscore.log" 2>&1 & ROSCORE_PID=$!
for _ in $(seq 1 30); do rostopic list >/dev/null 2>&1 && break; sleep 1; done
roslaunch lite3_gazebo_sim m2_gazebo.launch gui:=false > "$RUN/gazebo.log" 2>&1 & PIDS+=($!)
wait_topic /clock 300 || { echo "M5.2 活体结论 = INVALID|/clock 未就绪"; exit 9; }
wait_topic /rslidar_points 480 || { echo "M5.2 活体结论 = INVALID|/rslidar_points 未就绪"; exit 9; }
roslaunch lite3_gazebo_sim m2_mapping.launch > "$RUN/mapping.log" 2>&1 & PIDS+=($!)
wait_topic /Odometry 360 || { echo "M5.2 活体结论 = INVALID|/Odometry 未就绪"; exit 9; }
roslaunch lite3_safety 60_safety.launch > "$RUN/mux.log" 2>&1 & PIDS+=($!)
wait_topic /cmd_vel 240 || { echo "M5.2 活体结论 = INVALID|/cmd_vel 未就绪"; exit 9; }
roslaunch lite3_scan 70_scan.launch navi_mode:=1 > "$RUN/scan.log" 2>&1 & PIDS+=($!)
wait_topic /LIO/odom_vehicle 240 || { echo "M5.2 活体结论 = INVALID|/LIO/odom_vehicle 未就绪"; exit 9; }
wait_topic /lite3/scan/arrived 180 || { echo "M5.2 活体结论 = INVALID|到达实体不在线"; exit 9; }
note "链上四件=$(rosnode list 2>/dev/null | grep -cE 'scan_planner_node|closed_loop_controller|scan_arrival_monitor|safe_cmd_vel_mux')/4；F9 互斥=$(rosnode list 2>/dev/null | grep -cE 'move_base|/teb')/0"
say "== [6] 任务层节点（mode:=scan）=="
roslaunch lite3_task 50_task_smach_min.launch mode:=scan goal_x:="$GX" goal_y:="$GY" use_sim_time:=true > "$RUN/task.log" 2>&1 &
TP=$!; PIDS+=($TP)
t0=$SECONDS
while [ $((SECONDS - t0)) -lt "$WALL" ]; do
  grep -q 'task outcome' "$RUN/task.log" 2>/dev/null && break
  sleep 3
done
note "任务节点段耗时 $((SECONDS - t0))s；outcome 行：$(grep -o 'task outcome.*' "$RUN/task.log" 2>/dev/null | tail -1)"
note "原语行：$(grep -o '原语 scan_navigate outcome=[a-z]*[^]]*' "$RUN/task.log" 2>/dev/null | tail -1)"
say "== [7] 判定（交回契约件的段 B，判据只此一份）=="
bash /home/ros1-slam/ws/src/50_task/lite3_task/verify/v_task_scan_primitive.sh "$RUN/task.log" | tee -a "$LOG"
RC=${PIPESTATUS[0]}
# 红线复算必须**当场取现值**：旧写法读 $RUN/pcd_after.sha，而那件是 cleanup（trap）才写的 ⇒
#   [7] 跑在它之前，读到空串，把「样张未扰动」打成了「否」。这是假警报，不是被测件问题（22:57 实测）。
NOW_PCD=$(sha256sum /home/ros1-slam/data/maps/lite3.pcd | cut -d' ' -f1)
note "样张起前＝退后：$( [ "$(cut -d' ' -f1 "$RUN/pcd_before.sha")" = "$NOW_PCD" ] && echo 是 || echo "否!! 起=$(cut -d' ' -f1 "$RUN/pcd_before.sha") 今=$NOW_PCD" )"
echo "M5.2 活体结论 = $([ "$RC" = "0" ] && echo PASS || echo "rc=$RC")｜判定件 rc=$RC（0 全过／5 判据 FAIL／9 前置不成立）"
exit "$RC"
