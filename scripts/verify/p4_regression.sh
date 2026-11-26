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
# ★观测窗口必须 ≥ 系统自己的到达预算★（2026-11-26 修）：
# BT 会按路径长度算 `arrival budget`（本走廊 1.30 m ⇒ 65.0 s，日志逐次打印）。
# 原窗口 45 s **短于系统自己的预算** ⇒ 只要任务按预算节奏走完，探针也已经在 45 s 收工，
# 读数必然"没到"（实测 est_travel=0.89 m / 45 s，恰是每步靠 8 s 推进超时攒出来的）。
# 窗口取 140 s＝预算 65 s + 步进式推进（local 每段 0.30 m 就停，要等新航点）的余量；
# 实测同一走廊的完成时间在 60–130 s 之间抖动，窗口短于这个上界会把"慢但正确"判成失败。
NAV_WINDOW=${NAV_WINDOW:-140}
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
  # ★放好之后必须让 HDL 以"这个位姿"重新初始化★（2026-11-27 实测）：
  #   不重初始化时，HDL 沿用启动时的假设，实测 est↔gt 偏 **3.59 m**；
  #   同一机器人同一位置，重初始化后偏 **0.051 m**。
  #   这条不补，后面所有"到达"判定都是在一个错位的坐标系里做的（19 cm 到达差就是这么来的）。
  reinit_and_check_localization "$x" "$y"
}

reinit_and_check_localization() {
  local x=$1 y=$2 off
  # ★按实际定位后端分支★（2026-11-27 裁决）：`LOC_BACKEND=hdl`（缺省）才做 HDL 重初始化；
  # `lio`（Faster-LIO）/`none` 时**不得**用 HDL 就绪当作前置——那是拿另一套后端冒充验收条件。
  case "${LOC_BACKEND:-hdl}" in
    hdl) ;;
    *) say "[place] LOC_BACKEND=${LOC_BACKEND}：跳过 HDL 初始化/收敛检查"; return 0 ;;
  esac
  if ! timeout 5 rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: $x, y: $y, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0.25,0,0, 0,0,0,0.25,0,0, 0,0,0,0,0.25,0, 0,0,0,0,0,0.25]}}" >/dev/null 2>&1; then
    say '[place] /initialpose 发布失败：定位未被重置，拒绝继续'
    return 1
  fi
  sleep 3
  # ★复用共享探针★（2026-11-27 裁决）：`est_gt_ab_probe.py` 按**消息戳**配对并给中位数，
  # 原来的内联实现按数组下标对齐，两侧帧率不同就会把不同时刻的位姿相减。
  off=$(python3 "$(dirname "${BASH_SOURCE[0]}")/est_gt_ab_probe.py" \
        --seconds 6 --out "${EV:-/tmp}/loc_offset.json" 2>/dev/null \
        | python3 -c 'import json,sys
raw=sys.stdin.read().strip().split("\n")[-1]
d=json.loads(raw)
v=d.get("err_median_dxy")
# ★0.0 是合法读数，不能被 `or 9.999` 当成缺失★（2026-11-27 裁决）：只有 None 才算没测到。
print("9.999" if v is None else "%.3f"%v)' 2>/dev/null)
  off=${off:-9.999}
  say "[place] est_gt_offset=${off} m (限 ${LOC_MAX_OFFSET_M:-0.15} m)"
  if awk "BEGIN{exit !($off > ${LOC_MAX_OFFSET_M:-0.15})}"; then
    say '[place] 定位未收敛：拒绝在该状态下跑验收'
    return 1
  fi
  return 0
}

kill_v8() {
  local n pat
  # ★覆盖 FAR 后端（裁决②）★：`far_planner` 与 `v8_global_far` 不在 `/v8` 前缀下，
  # 旧实现只清 `/v8*`，上一轮的规划器会带着旧目标活到下一轮，表现为"新车一启动就被拖走"。
  for n in $(rosnode list 2>/dev/null | grep -E '^/v8|^/far_planner|^/far_planner_node|^/v8_global_far' || true); do
    rosnode kill "$n" >/dev/null 2>&1 || true
  done
  # 等它们真的消失（rosnode kill 只是请求）；超时再按进程名精确补刀，不做宽泛 pkill
  for _ in $(seq 1 10); do
    n=$(rosnode list 2>/dev/null | grep -cE '^/v8|^/far_planner|^/v8_global_far' || true)
    [ "${n:-0}" -eq 0 ] && break
    sleep 1
  done
  for pat in 'far_planner/far_planner' 'run_global_far.py'; do
    pkill -f "$pat" 2>/dev/null || true
  done
  sleep 1
  rosnode list 2>/dev/null | grep -cE '^/v8|^/far_planner|^/v8_global_far' > "${EV_ROOT:-/tmp}/leftover_nodes.txt" 2>/dev/null || true
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
    SKIP_GAZEBO=1 NO_LAUNCH_GAZEBO=1 MODE=normal DUR=8 WAIT_SCAN=40 \
      INIT_X=$START_X INIT_Y=$START_Y \
      bash $REPO/scripts/verify/asan_hdl_repro.sh > "$EV_ROOT/${tag}_repro.log" 2>&1
    local hdl_rc=$?
    # ★返回码必须检查（裁决③）★：asan 失败时以前会继续往下跑，用"节点数量够了"冒充就绪，
    # 于是后面每个用例都在没有定位的情况下测，读数全是 INVALID。
    if [ "$hdl_rc" -ne 0 ]; then
      say "[$tag] ABORT：asan_hdl_repro rc=$hdl_rc（见 ${tag}_repro.log 尾部）"
      tail -15 "$EV_ROOT/${tag}_repro.log" | sed 's/^/    /'
      return 1
    fi
  fi
  # ★2026-11-25：GRAPH/EXPECT_NODES 透传★（只换后端，不改任何参数）。
  # 用 `env ${VAR:+VAR=$VAR}` 形式：为空时**不注入**，p4_nav_hdl 的默认值照旧生效，
  # 从而保证"不传 GRAPH 时行为与历史完全一致"。POSE_TOPIC 同理（M2 组合传 /Odometry）。
  env ${GRAPH:+GRAPH="$GRAPH"} ${EXPECT_NODES:+EXPECT_NODES="$EXPECT_NODES"} \
      ${POSE_TOPIC:+POSE_TOPIC="$POSE_TOPIC"} \
      SKIP_PROBE=1 NO_GOAL="$no_goal" START_X=$START_X START_Y=$START_Y GOAL_X=$GOAL_X GOAL_Y=$GOAL_Y \
    EV="$EV_ROOT/${tag}_graph" bash $REPO/scripts/verify/p4_nav_hdl.sh > "$EV_ROOT/${tag}_setup.log" 2>&1
  local nav_rc=$?
  # ★必须紧贴命令取 rc★（这里以前隔了 40 行，$? 拿到的是中间某条 rostopic 的返回码，
  # 于是"装配失败"被当成"装配成功"往下跑——裁决②）
  if [ "$nav_rc" -ne 0 ]; then
    say "[$tag] ABORT：p4_nav_hdl rc=$nav_rc（见 $EV_ROOT/${tag}_setup.log / ${tag}_graph/graph.log）"
    tail -20 "$EV_ROOT/${tag}_setup.log" | sed 's/^/    /'
    return "$nav_rc"
  fi
  # ★就绪判定：两种组合都做，看"实际消费的位姿话题"有没有连续数据★（不是原始 /Odometry）。
  # 闸门没起来/没转发时这里会明确报错，而不是让后续用例集体 INVALID 却看不出原因。
  # ★判据不能用 `tail -1`★（2026-11-26 修）：`rostopic hz` 的输出是两行——
  #   `average rate: 9.606` ／ `\tmin: … max: … window: N`
  # 取最后一行永远拿到 min/max 那行 ⇒ `case … "average rate"` 恒不匹配 ⇒ 就绪判定**恒假**，
  # 明明 /hdl_odom 9.6 Hz 也判"无连续数据"、四个用例集体 ABORT（现场读数见 05:21 那轮 run.log）。
  # 正确做法是判整段输出里有没有 "average rate"，不能只看某一行。
  local ready=0 out
  for _ in $(seq 1 20); do
    sleep 2
    out=$(timeout 4 rostopic hz "$POSE_TOPIC" 2>/dev/null || true)
    printf '%s' "$out" | grep -q "average rate" && { ready=1; break; }
  done
  if [ "$ready" != "1" ]; then
    say "[$tag] ABORT：定位输入未就绪（$POSE_TOPIC 无连续数据）——拒绝在无定位状态下做验收"
    return 9
  fi
  local n
  n=$(rosnode list 2>/dev/null | grep -cE '^/v8' || true)
  say "[$tag] v8_nodes=$n"
  [ "$n" -ge 9 ] || say "[$tag] WARN 装配不全（$n/9），本项结果不可信"
}

# ★目标只能发一条★（2026-11-26 修）：`/v8/bt/goal` 是 **FIFO 多任务入口**，
# 每收到一条就 `current_seq=++goal_seq` 入队；`rostopic pub -r 5` 三秒＝15 个任务 ⇒
# 同一件事被拆成 15 个 mission：每出队一个就重发全局路径 + `follow path`，
# 且"出队下一个"会把上一个记成 `goal_outcome=reached` ⇒ BT 报到达而车还差 0.6 m
# （实测 est_to_goal=0.595 / gt=0.677，bt=reached）。三处用例失败同源于此。
send_start() {
  timeout 3 rostopic pub -r 5 /v8/3d/global/start geometry_msgs/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: $1, y: $2}, orientation: {w: 1.0}}}" >/dev/null 2>&1
}

send_goal_once() {  # $1=x $2=y $3=输出 json
  python3 "$REPO/scripts/verify/bt_goal_once.py" --x "$1" --y "$2" --out "$3" \
    > "${3%.json}.log" 2>&1
}

case_nav() {
  say '=== 1 正常导航 ==='
  # ★探针先就绪、再下发任务★：先把图起好（不发目标），探针起订阅，再由正式入口发 start+goal。
  # 反序（先发目标再起探针）会漏掉起步段，也算不出连续采样覆盖。
  reset_stack nav 1 || { say '=== 1 正常导航：前置未就绪，用例阻断 ==='; return 9; }
  # ★并行留时间线★（2026-11-26）：导航偶发"只走 0.3–0.9 m 就停"时，只看末端读数分不清
  # 是 path_goal 没推进、local 拒绝、mux 拦了还是 BT 结束任务。这条 TSV 每次都留，
  # 出事时能直接看"谁在停"。成本是每 0.1 s 一条记录，可忽略。
  python3 $REPO/scripts/verify/loc_timeline_probe.py --seconds "$NAV_WINDOW" \
    --out "$EV_ROOT/nav_timeline.tsv" > "$EV_ROOT/nav_timeline.log" 2>&1 &
  local tp=$!
  python3 $REPO/scripts/verify/p4_nav_probe.py --goal-x "$GOAL_X" --goal-y "$GOAL_Y" \
    --pose-topic "$POSE_TOPIC" \
    --duration "$NAV_WINDOW" --out "$EV_ROOT/nav_probe.json" > "$EV_ROOT/nav_probe.log" 2>&1 &
  local np=$!
  sleep 4                      # 给探针完成订阅/采样就绪
  send_start "$START_X" "$START_Y"
  send_goal_once "$GOAL_X" "$GOAL_Y" "$EV_ROOT/nav_goal_once.json" || \
    say '[nav] WARN 目标未被 BT 受理（见 nav_goal_once.log）'
  wait $np 2>/dev/null
  wait $tp 2>/dev/null
  sleep 5
  python3 $REPO/scripts/verify/nav_arrival_check.py "$GOAL_X" "$GOAL_Y" "$EV_ROOT/nav_arrival.json" \
    > "$EV_ROOT/nav_arrival.log" 2>&1
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
  reset_stack obstacle 1 || { say '=== 2b 运动中障碍：前置未就绪，用例阻断 ==='; return 9; }
  send_start "$START_X" "$START_Y"
  send_goal_once "$GOAL_X" "$GOAL_Y" "$EV_ROOT/obstacle_goal_once.json" || \
    say '[obstacle] WARN 目标未被 BT 受理（见 obstacle_goal_once.log）' &
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
  reset_stack loc 1 || { say '=== 3 定位失效：前置未就绪，用例阻断 ==='; return 9; }
  # 闸门 relay：杀它只断 /hdl_odom，master 与导航进程都不受影响。
  # 探针在「首帧非零命令」时下手，不靠固定 sleep（运动窗口只有 2 s 上下）。
  python3 $REPO/scripts/verify/motion_interrupt_probe.py --mode stop --hold-s 24 \
    --wait-motion-s 90 --inject-delay-s 1.0 --anchor-topic "$ANCHOR_TOPIC" \
    --action "$INJECT_ACTION" \
    --out "$EV_ROOT/loc_stop.json" --dump "$EV_ROOT/loc_series.tsv" \
    > "$EV_ROOT/loc_stop.log" 2>&1 &
  local pp=$!
  sleep 6
  send_start "$START_X" "$START_Y"
  send_goal_once "$GOAL_X" "$GOAL_Y" "$EV_ROOT/loc_goal_once.json" || \
    say '[loc] WARN 目标未被 BT 受理（见 loc_goal_once.log）'
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
    --pose-topic "$POSE_TOPIC" --duration "$NAV_WINDOW" \
    --out "$EV_ROOT/loc_rearm_nav.json" > "$EV_ROOT/loc_rearm_nav.log" 2>&1 &
  local np=$!
  sleep 3
  send_start "$START_X" "$START_Y"
  send_goal_once "$GOAL_X" "$GOAL_Y" "$EV_ROOT/loc_rearm_goal_once.json" || \
    say '[3b] WARN 目标未被 BT 受理（见 loc_rearm_goal_once.log）'
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
    GOAL_X=$CANCEL_GOAL_X GOAL_Y=$CANCEL_GOAL_Y reset_stack cancel 1 || {
      say '=== 4 取消：前置未就绪，用例阻断 ==='; return 9; }
  # ★目标只发一条★：取消之后若还有目标进来，系统重新起步是正确行为，会被误读成
  # "取消没生效"——`send_goal_once` 发一条并确认受理，不再连续刷。
  # ★注入前必须在动★：历史证据里出现过 moving_at_inject=false —— 那种情况下"取消后不动"
  # 不能证明取消生效。这里先等到真值速度非零，再开始注入窗口。
  # ★删掉"先等一次 /cmd_vel 非零"的预检★（2026-11-26）：它每秒起一个 rostopic 进程，
  # 冷启动本身就要 ~1 s，等于在"没看"的状态下耗掉最宝贵的第一步窗口；而且它只是打印一句 WARN，
  # 真正的"等运动"由 motion_interrupt_probe 自己做（并且现在还会等"此刻仍在动"）。
  # 实测：该预检报"未取得运动"，而探针在同一轮里 motion_seen=true —— 预检只贡献噪音与延迟。
  python3 $REPO/scripts/verify/motion_interrupt_probe.py --mode cancel --hold-s 26 \
    --wait-motion-s 90 --inject-delay-s 1.5 \
    --out "$EV_ROOT/cancel.json" --dump "$EV_ROOT/cancel_series.tsv" \
    > "$EV_ROOT/cancel.log" 2>&1 &
  local pp=$!
  sleep 6
  send_start "$CANCEL_START_X" "$CANCEL_START_Y"
  send_goal_once "$CANCEL_GOAL_X" "$CANCEL_GOAL_Y" "$EV_ROOT/cancel_goal_once.json" || \
    say '[cancel] WARN 目标未被 BT 受理（见 cancel_goal_once.log）'
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

