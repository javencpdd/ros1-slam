#!/usr/bin/env bash
# P4 四项验收自动化回归：正常导航 / 障碍安全 / 定位失效 / 取消。
#
# 每项都从同一干净起点开始（teleport → 重启 HDL → 重启装配），避免上一项的残留影响下一项。
# 单项耗时约 3–4 分钟，全套约 15 分钟。判定只看各自 JSON 里的字段，不做二次解释。
#
# 前置：Gazebo（lite3_planar + rslidar）已在运行。
# 用法：bash p4_regression.sh [--only nav|obstacle|loc|cancel]
set -uo pipefail

REPO=/opt/ros1-slam
WS=$REPO/ws
EV_ROOT=${EV_ROOT:-$REPO/note/进度/00_专题/v8_map_loc_2026-11-24/evidence/regression_$(date +%Y%m%d_%H%M%S)}
START_X=${START_X:-5.2}; START_Y=${START_Y:-2.45}
GOAL_X=${GOAL_X:-3.9};  GOAL_Y=${GOAL_Y:-2.45}
# 取消用更长的走廊（同一条 2.8 m 净宽线，只是终点更远），
# 避免目标落进算法位姿偏置的"已到达"窗口里导致车不动
CANCEL_START_X=${CANCEL_START_X:-5.2}; CANCEL_START_Y=${CANCEL_START_Y:-2.45}
CANCEL_GOAL_X=${CANCEL_GOAL_X:-3.9};   CANCEL_GOAL_Y=${CANCEL_GOAL_Y:-2.45}
# scene A 用的实测点云帧（缺省用证据目录里那一份）
FRAME_NPY=${FRAME_NPY:-$REPO/note/进度/00_专题/v8_map_loc_2026-11-24/evidence/collision_205409/frame.npy}


mkdir -p "$EV_ROOT"
exec > >(tee "$EV_ROOT/run.log") 2>&1
say() { printf '%s\n' "$*"; }

source /opt/ros/noetic/setup.bash
source $WS/devel/setup.bash
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://127.0.0.1:11313}

teleport() {
  rosservice call /gazebo/set_model_state "{model_state: {model_name: lite3_planar, pose: {position: {x: $1, y: $2, z: 0.11}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, reference_frame: world}}" >/dev/null 2>&1
  sleep 2
}

place() {
  # 把模型搬到指定点并**核实搬成功了**。
  # 为什么必须核实：`asan_hdl_repro.sh` 只设 `/hdl/init_pos_x/y`，并不搬模型。
  # 一旦模型没在 START，HDL 的初值就与真实位置错位 —— 后面所有"到达"判定跟着错。
  # 实测错位 1.2 m 时 BT 一上来就判 reached，车压根不动（上一轮取消项 INVALID 的真因）。
  local x=$1 y=$2 out d ok
  teleport "$x" "$y"
  out=$(python3 - "$x" "$y" <<'PY'
import math, sys
import rospy
from nav_msgs.msg import Odometry
rospy.init_node('place_check', anonymous=True)
p = rospy.wait_for_message('/sim/odom_gt', Odometry, timeout=8).pose.pose.position
d = math.hypot(p.x - float(sys.argv[1]), p.y - float(sys.argv[2]))
print('%.3f %d' % (d, 1 if d <= 0.15 else 0))
PY
)
  d=${out%% *}; ok=${out##* }
  say "[place] target=($x,$y) err=${d} m ok=${ok}"
  if [ "$ok" != "1" ]; then
    say '[place] 未到位，重试一次'
    teleport "$x" "$y"
  fi
}

kill_v8() {
  local n
  for n in $(rosnode list 2>/dev/null | grep -E '^/v8' || true); do
    rosnode kill "$n" >/dev/null 2>&1 || true
  done
  sleep 2
}

reset_stack() {
  local tag=$1
  # 第 2 个参数非空 ⇒ 装配完不发目标，交给探针自己发。
  # 定位失效与取消两项必须这样：若装配阶段已经把车开走，探针一上来就"已在运动"，
  # 注入点和零速窗口都不再可控。导航项相反，目标本来由装配脚本发，探针只观察。
  local no_goal=${2:-}
  # ★顺序不能反★：先拆旧编排，再搬车，最后起 HDL。
  # 旧编排没拆就搬车，它会继续把车开回上一个目标（实测 60 s 走掉 1.2 m），
  # 而 HDL 已经在 START 初始化了 ⇒ 位姿与真值错位，"到达"判定全歪。
  kill_v8
  place "$START_X" "$START_Y"
  SKIP_GAZEBO=1 MODE=normal DUR=8 WAIT_SCAN=40 INIT_X=$START_X INIT_Y=$START_Y \
    bash $REPO/scripts/verify/asan_hdl_repro.sh > "$EV_ROOT/${tag}_repro.log" 2>&1
  SKIP_PROBE=1 NO_GOAL="$no_goal" START_X=$START_X START_Y=$START_Y GOAL_X=$GOAL_X GOAL_Y=$GOAL_Y \
    EV="$EV_ROOT/${tag}_graph" bash $REPO/scripts/verify/p4_nav_hdl.sh > "$EV_ROOT/${tag}_setup.log" 2>&1
  local n
  n=$(rosnode list 2>/dev/null | grep -cE '^/v8' || true)
  say "[$tag] v8_nodes=$n"
  [ "$n" -ge 9 ] || say "[$tag] WARN 装配不全（$n/9），本项结果不可信"
}

case_nav() {
  say '=== 1 正常导航 ==='
  reset_stack nav
  python3 $REPO/scripts/verify/p4_nav_probe.py --goal-x "$GOAL_X" --goal-y "$GOAL_Y" \
    --duration 40 --out "$EV_ROOT/nav_probe.json" > "$EV_ROOT/nav_probe.log" 2>&1
  sleep 5   # 停稳后再量真实到达误差，避免运动末段的非同步采样
  python3 - "$GOAL_X" "$GOAL_Y" "$EV_ROOT/nav_arrival.json" <<'PY'
import json, math, sys
import rospy
from nav_msgs.msg import Odometry
gx, gy, out = float(sys.argv[1]), float(sys.argv[2]), sys.argv[3]
rospy.init_node('arrival_check', anonymous=True)
e = rospy.wait_for_message('/hdl_odom', Odometry, timeout=8).pose.pose.position
g = rospy.wait_for_message('/sim/odom_gt', Odometry, timeout=8).pose.pose.position
res = {'est_to_goal_m': round(math.hypot(e.x - gx, e.y - gy), 3),
       'gt_to_goal_m': round(math.hypot(g.x - gx, g.y - gy), 3),
       'est_gt_gap_m': round(math.hypot(e.x - g.x, e.y - g.y), 3),
       'verdict': 'PASS' if math.hypot(g.x - gx, g.y - gy) <= 0.15 else 'FAIL'}
print(json.dumps(res, ensure_ascii=False))
open(out, 'w').write(json.dumps(res, ensure_ascii=False) + '\n')
PY
}

case_obstacle() {
  say '=== 2 障碍安全（离线回归，不需重置栈）==='
  # scene A 用的是**实测点云帧**，不给 --frame 会被 skip 记 INVALID。
  # 缺帧时如实判 INVALID，不拿剩下的四个场景冒充五项全过。
  local frame_arg=''
  if [ -n "${FRAME_NPY:-}" ] && [ -f "${FRAME_NPY}" ]; then
    frame_arg="--frame $FRAME_NPY"
  else
    say "[obstacle] WARN 无 --frame（FRAME_NPY=$FRAME_NPY），scene A 会被 skip"
  fi
  # shellcheck disable=SC2086
  python3 $REPO/scripts/verify/collision_regression.py --out "$EV_ROOT/obstacle.json" \
    $frame_arg > "$EV_ROOT/obstacle.log" 2>&1
  tail -5 "$EV_ROOT/obstacle.log"
}

case_loc() {
  say '=== 3 定位失效（只断定位数据）==='
  reset_stack loc 1
  # 闸门 relay：杀它只断 /hdl_odom，master 与导航进程都不受影响。
  # 探针在「首帧非零命令」时下手，不靠固定 sleep（运动窗口只有 2 s 上下）。
  python3 $REPO/scripts/verify/motion_interrupt_probe.py --mode stop --hold-s 24 \
    --wait-motion-s 45 --inject-delay-s 1.0 --action 'rosnode kill /hdl_gate' \
    --out "$EV_ROOT/loc_stop.json" --dump "$EV_ROOT/loc_series.tsv" \
    > "$EV_ROOT/loc_stop.log" 2>&1 &
  local pp=$!
  sleep 6
  python3 $REPO/scripts/verify/goal_until_motion.py --x "$GOAL_X" --y "$GOAL_Y" \
    --timeout-s 45 --out "$EV_ROOT/loc_goal.json" > "$EV_ROOT/loc_goal.log" 2>&1
  wait $pp
  cat "$EV_ROOT/loc_stop.json" 2>/dev/null
}

case_cancel() {
  say '=== 4 取消（scope=all）==='
  # 取消测试要一条「够长、够宽」的走廊：目标是 1.3 m 外的 (3.9, 2.45)，
  # 净宽 2.8 m。太近（0.65 m）会被算法位姿 0.24 m 的偏置判成"已到达"，车根本不动。
  START_X=$CANCEL_START_X START_Y=$CANCEL_START_Y \
    GOAL_X=$CANCEL_GOAL_X GOAL_Y=$CANCEL_GOAL_Y reset_stack cancel 1
  # ★目标必须在车动起来的那一刻停发★：取消之后再进来新目标，系统重新起步是
  # 正确行为，会被误读成"取消没生效"。goal_until_motion 就干这件事。
  python3 $REPO/scripts/verify/motion_interrupt_probe.py --mode cancel --hold-s 26 \
    --wait-motion-s 45 --inject-delay-s 1.5 \
    --out "$EV_ROOT/cancel.json" --dump "$EV_ROOT/cancel_series.tsv" \
    > "$EV_ROOT/cancel.log" 2>&1 &
  local pp=$!
  sleep 6
  python3 $REPO/scripts/verify/goal_until_motion.py --x "$CANCEL_GOAL_X" --y "$CANCEL_GOAL_Y" \
    --timeout-s 45 --out "$EV_ROOT/cancel_goal.json" > "$EV_ROOT/cancel_goal.log" 2>&1
  wait $pp
  cat "$EV_ROOT/cancel.json" 2>/dev/null
  # 活体反控：同一进程里先取消、再喂旧 stamp 路径（必须被拒）、再喂新 stamp（必须被收）
  python3 $REPO/scripts/verify/cancel_replay_probe.py \
    --x "$CANCEL_GOAL_X" --y "$CANCEL_GOAL_Y" --cancel-seq 5 \
    --out "$EV_ROOT/cancel_replay.json" > "$EV_ROOT/cancel_replay.log" 2>&1
  cat "$EV_ROOT/cancel_replay.json" 2>/dev/null
}

# 单项重跑：ONLY=nav|obstacle|loc|cancel（也接受第一个位置参数）
ONLY=${ONLY:-${1:-}}
[ -z "$ONLY" ] || [ "$ONLY" = nav ]      && case_nav
[ -z "$ONLY" ] || [ "$ONLY" = obstacle ] && case_obstacle
[ -z "$ONLY" ] || [ "$ONLY" = loc ]      && case_loc
[ -z "$ONLY" ] || [ "$ONLY" = cancel ]   && case_cancel

say '=== 汇总（机器判定）==='
python3 $REPO/scripts/verify/p4_judge.py --root "$EV_ROOT"
RC=$?
say "EV_ROOT=$EV_ROOT"
say "judge_rc=$RC（0=四项全 PASS，非零=有项未通过）"
exit $RC

