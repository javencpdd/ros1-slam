#!/usr/bin/env bash
# P4 四项验收自动化回归：正常导航 / 障碍安全 / 定位失效 / 取消。
#
# 每项都从同一干净起点开始（teleport → 重启 HDL → 重启装配），避免上一项的残留影响下一项。
# 单项耗时约 3–4 分钟，全套约 15 分钟。判定只看各自 JSON 里的字段，不做二次解释。
#
# 前置：Gazebo（lite3_planar + rslidar）已在运行。
# 用法：bash p4_regression.sh [nav|obstacle|loc|cancel]   或   ONLY=nav bash p4_regression.sh
#       注意：不接受 `--only nav` 这种写法（脚本按裸词比较，传 --only 会四项都不跑）。
# 组合切换（只换后端，其余不变）：
#   GRAPH=<...graph_3d_m1_hdl_bt_far.yaml> EXPECT_NODES=<n> bash p4_regression.sh
#   POSE_TOPIC=/Odometry GRAPH=<...graph_3d_m2_lio_bt_far.yaml> bash p4_regression.sh
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
# ★组合模式★：M2=1 表示"LIO 位姿"组合——底座已由 sim.sh base m2 提供 /Odometry，
# 本脚本不再装载 HDL，而是用一个独立数据闸门 /Odometry → /Odometry_gated（节点名 m2_gate），
# 供"定位断流"用例 kill/重启，从而不杀 LIO 本身。
M2=${M2:-0}
# ★M2 的算法位姿取"实际导航输入" `/v8/local/pose`★（已做 LIO→map 刚体变换），
# 不是原始 `/Odometry`（那是 LIO 自己原点的坐标系，只在断流探针里作为闸门上游使用）。
POSE_TOPIC=${POSE_TOPIC:-$([ "$M2" = "1" ] && echo /v8/local/pose || echo /hdl_odom)}
GATE_IN=${GATE_IN:-$([ "$M2" = "1" ] && echo /Odometry || echo /hdl_odom_raw)}
GATE_OUT=${GATE_OUT:-$([ "$M2" = "1" ] && echo /Odometry_gated || echo /hdl_odom)}
GATE_NAME=${GATE_NAME:-$([ "$M2" = "1" ] && echo m2_gate || echo hdl_gate)}
# ★必须 export★：到达核对那段是内联 python，靠 os.environ 读 POSE_TOPIC；
# 不导出时它永远回退到 /hdl_odom（M2 组合下必然超时，失败日志已证实）。
export POSE_TOPIC
# ★断流锚按组合取★：HDL 锚 /hdl_odom（asan 的 relay 输出），M2 锚闸门输出 /Odometry_gated
ANCHOR_TOPIC=${ANCHOR_TOPIC:-$([ "$M2" = "1" ] && echo "$GATE_OUT" || echo /hdl_odom)}
export ANCHOR_TOPIC
# 断流注入：HDL 组合杀 asan 的 relay；M2 组合调图内闸门服务（两种机制不混用）
if [ "$M2" = "1" ]; then
  INJECT_ACTION="rosservice call /v8/3d/lio_gate/lio_gate/set_enabled '{data: false}'"
else
  INJECT_ACTION="rosnode kill /hdl_gate"
fi


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
  if [ "$M2" = "1" ]; then
    # ★M2 验收模式★：定位输入链（闸门）由图内 `v8.fw.lio_gate` 组件提供，不在本脚本里 rosrun。
    # 就绪判定放在**图起来之后**（见函数末尾），判的是 loc 真正消费的位姿话题。
    say "[$tag] M2 验收模式（闸门由图内 lio_gate 组件提供）"
  else
    SKIP_GAZEBO=1 MODE=normal DUR=8 WAIT_SCAN=40 INIT_X=$START_X INIT_Y=$START_Y \
      bash $REPO/scripts/verify/asan_hdl_repro.sh > "$EV_ROOT/${tag}_repro.log" 2>&1
  fi
  # ★2026-11-25：GRAPH/EXPECT_NODES 透传★（只换后端，不改任何参数）。
  # 用 `env ${VAR:+VAR=$VAR}` 形式：为空时**不注入**，p4_nav_hdl 的默认值照旧生效，
  # 从而保证"不传 GRAPH 时行为与历史完全一致"。POSE_TOPIC 同理（M2 组合传 /Odometry）。
  env ${GRAPH:+GRAPH="$GRAPH"} ${EXPECT_NODES:+EXPECT_NODES="$EXPECT_NODES"} \
      ${POSE_TOPIC:+POSE_TOPIC="$POSE_TOPIC"} \
      SKIP_PROBE=1 NO_GOAL="$no_goal" START_X=$START_X START_Y=$START_Y GOAL_X=$GOAL_X GOAL_Y=$GOAL_Y \
    EV="$EV_ROOT/${tag}_graph" bash $REPO/scripts/verify/p4_nav_hdl.sh > "$EV_ROOT/${tag}_setup.log" 2>&1
  if [ "$M2" = "1" ]; then
    # ★就绪判定：看"实际消费的位姿话题"有没有连续数据★（不是原始 /Odometry）。
    # 闸门没起来/没转发时这里会明确报错，而不是让后续用例集体 INVALID 却看不出原因。
    local ready=0 tick
    for _ in $(seq 1 20); do
      sleep 2
      tick=$(timeout 4 rostopic hz "$POSE_TOPIC" 2>/dev/null | tail -1)
      case "$tick" in *"average rate"*) ready=1; break ;; esac
    done
    if [ "$ready" != "1" ]; then
      say "[$tag] ABORT：定位输入未就绪（$POSE_TOPIC 无连续数据）——拒绝在无定位状态下做验收"
      return 9
    fi
  fi
  local n
  n=$(rosnode list 2>/dev/null | grep -cE '^/v8' || true)
  say "[$tag] v8_nodes=$n"
  [ "$n" -ge 9 ] || say "[$tag] WARN 装配不全（$n/9），本项结果不可信"
}

case_nav() {
  say '=== 1 正常导航 ==='
  reset_stack nav
  python3 $REPO/scripts/verify/p4_nav_probe.py --goal-x "$GOAL_X" --goal-y "$GOAL_Y" \
    --pose-topic "$POSE_TOPIC" \
    --duration 40 --out "$EV_ROOT/nav_probe.json" > "$EV_ROOT/nav_probe.log" 2>&1
  sleep 5   # 停稳后再量真实到达误差，避免运动末段的非同步采样
  python3 - "$GOAL_X" "$GOAL_Y" "$EV_ROOT/nav_arrival.json" <<'PY'
import json, math, os, sys
import rospy
from nav_msgs.msg import Odometry
gx, gy, out = float(sys.argv[1]), float(sys.argv[2]), sys.argv[3]
rospy.init_node('arrival_check', anonymous=True)
e = rospy.wait_for_message(os.environ.get('POSE_TOPIC', '/hdl_odom'), Odometry, timeout=8).pose.pose.position
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

  # ★2b 运动中障碍（在线）★：离线回归只验算法本身，证不了"整条链在运动中真的会停/绕"。
  # 这里让车真的朝目标走，途中在它前方投一个箱子，断言不穿透 + 有响应（停≥0.5s 或横移≥0.10m）。
  say '=== 2b 运动中障碍（在线）==='
  reset_stack obstacle 1
  python3 $REPO/scripts/verify/goal_until_motion.py --x "$GOAL_X" --y "$GOAL_Y" \
    --timeout-s 45 --out "$EV_ROOT/obstacle_goal.json" > "$EV_ROOT/obstacle_goal.log" 2>&1 &
  local gp=$!
  sleep 6
  python3 $REPO/scripts/verify/obstacle_inmotion_probe.py \
    --pose-topic "${POSE_TOPIC:-/hdl_odom}" \
    --out "$EV_ROOT/obstacle_inmotion.json" > "$EV_ROOT/obstacle_inmotion.log" 2>&1
  wait $gp 2>/dev/null
  cat "$EV_ROOT/obstacle_inmotion.json" 2>/dev/null
}

case_loc() {
  say '=== 3 定位失效（只断定位数据）==='
  reset_stack loc 1
  # 闸门 relay：杀它只断 /hdl_odom，master 与导航进程都不受影响。
  # 探针在「首帧非零命令」时下手，不靠固定 sleep（运动窗口只有 2 s 上下）。
  python3 $REPO/scripts/verify/motion_interrupt_probe.py --mode stop --hold-s 24 \
    --wait-motion-s 90 --inject-delay-s 1.0 --anchor-topic "$ANCHOR_TOPIC" \
    --action "$INJECT_ACTION" \
    --out "$EV_ROOT/loc_stop.json" --dump "$EV_ROOT/loc_series.tsv" \
    > "$EV_ROOT/loc_stop.log" 2>&1 &
  local pp=$!
  sleep 6
  python3 $REPO/scripts/verify/goal_until_motion.py --x "$GOAL_X" --y "$GOAL_Y" \
    --timeout-s 45 --out "$EV_ROOT/loc_goal.json" > "$EV_ROOT/loc_goal.log" 2>&1
  wait $pp
  cat "$EV_ROOT/loc_stop.json" 2>/dev/null
  # ★恢复段：把定位闸门重新拉起来，断言车不自行复动★
  # 只验"断流会停"是不够的：恢复瞬间位姿可能跳变、局部规划器误以为离目标很远而重新起步。
  # ★必须先采样、后恢复闸门★：先恢复再采样会漏掉"恢复瞬间窜出去"的那一段。
  python3 "$REPO/scripts/verify/loc_recovery_probe.py" --seconds 10 --pose-topic "$POSE_TOPIC" \
    --out "$EV_ROOT/loc_recovery.json" > "$EV_ROOT/loc_recovery.log" 2>&1 &
  local rp=$!
  sleep 2
  # ★恢复机制按组合★：M2 走闸门服务；Support 走 asan 的 relay（图里没有 lio_gate，
  # 无条件调 M2 的服务在 Support 下必然失败）
  if [ "$M2" = "1" ]; then
    rosservice call /v8/3d/lio_gate/lio_gate/set_enabled '{data: true}' \
      > "$EV_ROOT/loc_recovery_gate.log" 2>&1
  else
    rosrun topic_tools relay /hdl_odom_raw /hdl_odom __name:=hdl_gate \
      > "$EV_ROOT/loc_recovery_relay.log" 2>&1 &
    sleep 2
  fi
  wait $rp 2>/dev/null
  cat "$EV_ROOT/loc_recovery.json" 2>/dev/null

  # ★正控：恢复后从正式任务入口发一个合法新任务，必须能重新起步★
  # 判据按第 7 条：任务身份（meta 到达）+ 有效速度 + 真实位移 + 正常导航（BT reached），
  # 不能只看 cmd_vel 字符串非零（实测过 0.6 m/s 的位移被弱检测漏掉）。
  say '=== 3b 正控：正式入口下发新任务（导航探针验证）==='
  python3 "$REPO/scripts/verify/loc_timeline_probe.py" --seconds 55 \
    --out "$EV_ROOT/loc_rearm_timeline.tsv" > "$EV_ROOT/loc_rearm_timeline.log" 2>&1 &
  local tp=$!
  sleep 2
  python3 "$REPO/scripts/verify/p4_nav_probe.py" --goal-x "$GOAL_X" --goal-y "$GOAL_Y" \
    --pose-topic "$POSE_TOPIC" --duration 45 \
    --out "$EV_ROOT/loc_rearm_nav.json" > "$EV_ROOT/loc_rearm_nav.log" 2>&1 &
  local np=$!
  sleep 3
  timeout 2 rostopic pub -r 5 /v8/3d/global/start geometry_msgs/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: $START_X, y: $START_Y}, orientation: {w: 1.0}}}" >/dev/null 2>&1
  timeout 2 rostopic pub -r 5 /v8/bt/goal geometry_msgs/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y}, orientation: {w: 1.0}}}" >/dev/null 2>&1
  wait $np 2>/dev/null
  wait $tp 2>/dev/null
  python3 - "$EV_ROOT" <<'PYEOF'
import json, os, sys
ev = sys.argv[1]
try:
    nav = json.load(open(os.path.join(ev, 'loc_rearm_nav.json')))
except Exception:
    nav = {}
metas = 0
try:
    with open(os.path.join(ev, 'loc_rearm_timeline.tsv')) as fh:
        next(fh)
        for line in fh:
            parts = line.rstrip('\n').split('\t')
            if len(parts) > 3:
                metas = max(metas, int(parts[3]))
except Exception:
    pass
checks = {
    'task_identity': metas > 0,
    'nonzero_speed': (nav.get('speed_frames') or 0) > 0,
    # ★真实位移必须用真值★：估计算法位姿的位移不能冒充真实运动
    'real_travel': (nav.get('gt_travel_m') or 0) >= 0.30,
    'bt_reached': bool(nav.get('bt_arrived')),
}
res = dict(checks, rearmed=all(checks.values()),
           speed_frames=nav.get('speed_frames'),
           travel_m=nav.get('est_travel_m'), gt_travel_m=nav.get('gt_travel_m'),
           meta_msgs=metas, reason='new task from formal entry')
open(os.path.join(ev, 'loc_rearm.json'), 'w').write(json.dumps(res, ensure_ascii=False) + '\n')
print(json.dumps(res, ensure_ascii=False))
PYEOF
  cat "$EV_ROOT/loc_rearm.json"
}

case_cancel() {
  say '=== 4 取消（scope=all）==='
  # 取消测试要一条「够长、够宽」的走廊：目标是 1.3 m 外的 (3.9, 2.45)，
  # 净宽 2.8 m。太近（0.65 m）会被算法位姿 0.24 m 的偏置判成"已到达"，车根本不动。
  START_X=$CANCEL_START_X START_Y=$CANCEL_START_Y \
    GOAL_X=$CANCEL_GOAL_X GOAL_Y=$CANCEL_GOAL_Y reset_stack cancel 1
  # ★目标必须在车动起来的那一刻停发★：取消之后再进来新目标，系统重新起步是
  # 正确行为，会被误读成"取消没生效"。goal_until_motion 就干这件事。
  # ★注入前必须在动★：历史证据里出现过 moving_at_inject=false —— 那种情况下"取消后不动"
  # 不能证明取消生效。这里先等到真值速度非零，再开始注入窗口。
  local mv=0 vv
  for _ in $(seq 1 60); do
    sleep 1
    vv=$(timeout 3 rostopic echo -n1 /cmd_vel/linear/x 2>/dev/null | head -1)
    if [ -n "$vv" ] && [ "$vv" != "0.0" ]; then mv=1; break; fi
  done
  [ "$mv" = "1" ] || say '[cancel] WARN 注入前未取得运动（本项将按 INVALID 记）'
  python3 $REPO/scripts/verify/motion_interrupt_probe.py --mode cancel --hold-s 26 \
    --wait-motion-s 90 --inject-delay-s 1.5 \
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

say '=== 收尾：拆掉本轮编排 ==='
# 不拆的后果是实测过的：残留的 roslaunch 会继续往 EV 目录里的 graph.log 追加，
# 一次 --only loc 就把它写脏了 1220 行（那份 log 当时已经在仓里）。
kill_v8

say '=== 汇总（机器判定）==='
python3 $REPO/scripts/verify/p4_judge.py --root "$EV_ROOT"
RC=$?
say "EV_ROOT=$EV_ROOT"
say "judge_rc=$RC（0=四项全 PASS，非零=有项未通过）"
exit $RC

