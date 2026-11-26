#!/usr/bin/env bash
# 定位失效安全的正反控（真人裁决 2026-11-26）。
#
# 反控（必须停）：
#   A 显式 `localization_ok=false` → 车停、且**恢复后不自行复动**（连续采样见 loc_recovery_probe）
#   B 断流后把**同一目标**再从正式入口发一次：这是"新任务"（BT 会给新代次），允许动——
#     注意区分：被拒的是"旧任务的残留航点/异步结果"，不是"用户新下的目标"。
# 正控（必须能动）：
#   C 恢复后从**正式任务入口** `/v8/bt/goal` 下发一个合法新任务 ⇒ 20 s 内出现非零速度。
#
# 前置：底座与 3D 图已起（由 p4_nav_hdl 起），本脚本只管注入与观测。
set -uo pipefail
REPO=/opt/ros1-slam
EV=${EV:-/tmp/loc_controls_$(date +%H%M%S)}
GOAL_X=${GOAL_X:-3.9}; GOAL_Y=${GOAL_Y:-2.45}
START_X=${START_X:-5.2}; START_Y=${START_Y:-2.45}
mkdir -p "$EV"
source /opt/ros/noetic/setup.bash
source $REPO/ws/devel/setup.bash
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://127.0.0.1:11313}

say() { printf '[ctrl] %s\n' "$*"; }
goal() {  # 正式任务入口
  # ★发一条就够★：/v8/bt/goal 是 FIFO 多任务入口，`-r 5` 会把同一件事刷成十几个 mission
  python3 "$(dirname "${BASH_SOURCE[0]}")/bt_goal_once.py" --x "$1" --y "$2" \
    --out "${EV:-/tmp}/goal_once.json" >/dev/null 2>&1
}
loc_ok() {  # 显式置位/清位定位闸门；$1=data $2=持续秒数
  timeout "${2:-1}" rostopic pub -r 10 /v8/local/localization_ok std_msgs/Bool "data: $1" >/dev/null 2>&1
}

# ---- 0. 先让车动起来 ----
say "启动导航（正式入口发目标）"
( for _ in $(seq 1 12); do goal "$GOAL_X" "$GOAL_Y"; sleep 2; done ) &
GP=$!
for _ in $(seq 1 20); do
  sleep 1
  v=$(timeout 3 rostopic echo -n1 /cmd_vel/linear/x 2>/dev/null | head -1)
  [ -n "$v" ] && [ "$v" != "0.0" ] && { say "已在运动 v=$v"; break; }
done
kill $GP 2>/dev/null || true

# ---- A. 显式 localization_ok=false，2 s 后恢复；期间连续采样 ----
say "注入：localization_ok=false（显式），2 s 后恢复"
python3 $REPO/scripts/verify/loc_recovery_probe.py --seconds 12 --out "$EV/A_recovery.json" \
  > "$EV/A_recovery.log" 2>&1 &
RP=$!
sleep 1
# ★false 必须短促★：之前写 8 s，与随后的 true 并发，闸门根本没恢复（正控因此假失败）
loc_ok false 1
sleep 1
loc_ok true 1
rostopic pub -1 /v8/local/localization_ok std_msgs/Bool "data: true" >/dev/null 2>&1
wait $RP 2>/dev/null
say "A：$(cat "$EV/A_recovery.json" 2>/dev/null)"

# ---- C. 正控：正式入口下发合法新任务，必须能动 ----
say "正控：正式入口再发一个合法目标，20 s 内应出现非零速度"
MOTION=0
( for _ in $(seq 1 10); do goal "$GOAL_X" "$GOAL_Y"; sleep 2; done ) &
GP=$!
for _ in $(seq 1 20); do
  sleep 1
  v=$(timeout 3 rostopic echo -n1 /cmd_vel/linear/x 2>/dev/null | head -1)
  if [ -n "$v" ] && [ "$v" != "0.0" ]; then MOTION=1; say "恢复运动 v=$v"; break; fi
done
kill $GP 2>/dev/null || true
echo "{\"positive_control_motion\": $([ "$MOTION" = "1" ] && echo true || echo false)}" > "$EV/C_positive.json"
cat "$EV/C_positive.json"
say "EV=$EV"
