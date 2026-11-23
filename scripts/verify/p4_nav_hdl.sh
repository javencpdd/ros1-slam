#!/usr/bin/env bash
# P4 验收：用算法位姿（hdl）驱动 3D 导航，全程只读真值用于事后核对。
#
# 前置（本脚本不负责）：
#   - Gazebo m1 世界已起，且 map_to_odom:=false（真定位接入时占位桥必须退位）
#   - hdl 已在跑并发布 /hdl_odom（见 asan_hdl_repro.sh MODE=normal）
#
# 本脚本做四件事：
#   1) 用 cli.py plan 把 graph 渲染成 launch（★必须重新渲染★：carrier 类型改过之后
#      复用旧 launch 会跑到旧组件，表现为 zmq::error_t 与 kinematic carrier loop failed）
#   2) 启动装配，等 7 个实例就绪
#   3) 发 start 与 goal
#   4) 跑 p4_nav_probe.py 采样，判定 PASS/FAIL
#
# 为什么用脚本而不是一行命令：pkill -f "<launch 名>" 会匹配到执行它的 bash 自身
# （cmdline 里含同一个字符串），把自己杀掉，表现为「编排根本没起来」。
set -uo pipefail

WS=${WS:-/opt/ros1-slam/ws}
REPO=/opt/ros1-slam
FW=$WS/src/90_platform/v8_platform/framework
APP=${APP:-$REPO/data/maps/generated/v8_m1_hdl_v1_app.yaml}
GRAPH=${GRAPH:-$FW/fixtures/graphs/graph_3d_m1_hdl_bt.yaml}
DURATION=${DURATION:-45}
EXPECT_NODES=${EXPECT_NODES:-9}
NOTE=$REPO/note/进度/00_专题/v8_map_loc_2026-11-24
EV=${EV:-$NOTE/evidence/p4_$(date +%Y%m%d_%H%M%S)}

# ★起点与目标必须从 task 读，不能硬编码★：换地图后 task 的 start/goal 会被重算
# （改过一次 fill-start 就让它从 (1.15,0.45)->(4.65,0.45) 变成 (0.05,-0.05)->(0.05,3.45)），
# 硬编码会让 BT 收到一个地图外的目标，报 E_PLAN_NOT_READY 却看不出原因。
if [ -z "${GOAL_X:-}" ]; then
  read -r START_X START_Y GOAL_X GOAL_Y <<EOF
$(python3 -c "
import json,os,sys
app=json.load(open('$APP'))
t=json.load(open(os.path.join('$REPO', app['task'])))
s=t['start']['xyz']; g=t['goal']['xyz']
sys.stdout.write('%.4f %.4f %.4f %.4f' % (s[0],s[1],g[0],g[1]))
")
EOF
fi

mkdir -p "$EV"
exec > >(tee "$EV/run.log") 2>&1

say() { printf '%s\n' "$*"; }
gate_fail() { say "GATE_FAIL: $*"; exit 9; }

source /opt/ros/noetic/setup.bash
source $WS/devel/setup.bash
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://127.0.0.1:11313}

LAUNCH=$EV/graph.launch
say "===== P4 EV=$EV ====="
say "graph=$GRAPH"

# ---------- 1. 重新渲染 launch ----------
say "--- [1] 渲染 launch（每次重渲，避免跑到旧组件）---"
( cd "$FW" && python3 cli.py plan fixtures/manifests "$GRAPH" \
    --out-plan "$EV/plan.json" --out-launch "$LAUNCH" ) > "$EV/render.json" 2>&1
grep -q '"ok": true' "$EV/render.json" || { cat "$EV/render.json"; gate_fail "渲染失败"; }
grep -q 'v8.fw.gazebo_carrier3d\|carrier3d' "$LAUNCH" && say "carrier_component_in_launch=$(grep -c 'carrier3d' "$LAUNCH")"
say "launch_lines=$(wc -l < "$LAUNCH")"

# ---------- 2. 清掉上一次的编排节点（不动 Gazebo 与 hdl）----------
say "--- [2] 清理上一次的 v8 编排节点 ---"
for n in $(rosnode list 2>/dev/null | grep -E '^/v8/' | grep -v '^/v8/local/pose' || true); do
  rosnode kill "$n" >/dev/null 2>&1
done
sleep 4
say "v8_nodes_after_clean=$(rosnode list 2>/dev/null | grep -cE '^/v8/3d|^/v8/bt' || true)"

# ---------- 3. 启动装配 ----------
say "--- [3] 启动装配 ---"
# --screen 是必须的：不加的话各节点的 ROS_INFO 只进 rosout，graph.log 是空的，
# 行为树/规划器为什么失败完全看不到（bb-bt-bt-5-stdout.log 实测 0 行）。
setsid roslaunch --screen "$LAUNCH" > "$EV/graph.log" 2>&1 < /dev/null &
for i in $(seq 1 20); do
  sleep 2
  N=$(rosnode list 2>/dev/null | grep -cE '^/v8/3d|^/v8/bt' || true)
  [ "$N" -ge "$EXPECT_NODES" ] && { say "nodes_ready_after=${i}x2s n=$N"; break; }
done
N=$(rosnode list 2>/dev/null | grep -cE '^/v8/3d|^/v8/bt' || true)
say "v8_nodes=$N (expect>=$EXPECT_NODES)"
rosnode list 2>/dev/null | grep -E '^/v8/3d|^/v8/bt' | tr '\n' ' '; say
if [ "$N" -lt "$EXPECT_NODES" ]; then
  say "--- graph.log 尾部 ---"; tail -25 "$EV/graph.log"
  gate_fail "编排未起来（$N/$EXPECT_NODES）"
fi
# ★只数数量会漏判★：曾经 8 个实例只起来 7 个，缺的正是 loc，数量判据没报错，
# 于是后面 E_PLAN_NOT_READY 与 loc_ok 空都被当成下游问题。loc 必须在列表里。
# 实例名是 ns + name = /v8/3d/loc/loc（不是 /v8/3d/loc），这里用前缀匹配避免再猜错一层。
rosnode list 2>/dev/null | grep -qE '^/v8/3d/loc(/|$)' \
  || { rosnode list 2>/dev/null | grep -E '^/v8' | tr '\n' ' '; say; gate_fail "loc 实例缺失（E_PLAN_NOT_READY 的直接原因）"; }
say "loc_instance=present"

# ---------- 4. 探针 + 发目标 ----------
if [ -n "${SKIP_PROBE:-}" ]; then
  say "--- [4] SKIP_PROBE：只发 start/goal，探针由调用方接管 ---"
  timeout 3 rostopic pub -r 5 /v8/3d/global/start geometry_msgs/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: $START_X, y: $START_Y, z: 0.0}, orientation: {w: 1.0}}}" \
    >/dev/null 2>&1
  sleep 1
  timeout 3 rostopic pub -r 5 /v8/bt/goal geometry_msgs/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, orientation: {w: 1.0}}}" \
    >/dev/null 2>&1
  say "goal_sent=$GOAL_X,$GOAL_Y (no probe)"
  say "EV=$EV"
  exit 0
fi
say "--- [4] 起探针并发 start/goal ---"
python3 "$REPO/scripts/verify/p4_nav_probe.py" \
  --goal-x "$GOAL_X" --goal-y "$GOAL_Y" --duration "$DURATION" --out "$EV/probe.json" \
  > "$EV/probe_stdout.log" 2>&1 &
PROBE_PID=$!
sleep 5
timeout 3 rostopic pub -r 5 /v8/3d/global/start geometry_msgs/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: $START_X, y: $START_Y, z: 0.0}, orientation: {w: 1.0}}}" \
  >/dev/null 2>&1
sleep 1
timeout 3 rostopic pub -r 5 /v8/bt/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, orientation: {w: 1.0}}}" \
  >/dev/null 2>&1
say "goal_sent=$GOAL_X,$GOAL_Y"

# ---------- 5. 等待探针 ----------
say "--- [5] 采样 ${DURATION}s ---"
wait $PROBE_PID 2>/dev/null; say "probe_exit=$?"
say "--- probe.json ---"
cat "$EV/probe.json" 2>/dev/null || say "(无 probe.json)"
say "--- probe stderr/stdout 尾 ---"
tail -8 "$EV/probe_stdout.log" 2>/dev/null

# ---------- 6. 轨迹与路径旁证 ----------
say "--- [6] 旁证 ---"
say "path_status=$(timeout 5 rostopic echo -n1 /v8/3d/global/status 2>/dev/null | head -2 | tr '\n' ' ')"
say "loc_ok=$(timeout 5 rostopic echo -n1 /v8/local/localization_ok 2>/dev/null | head -1)"
say "hdl_odom_pos=$(timeout 5 rostopic echo -n1 /hdl_odom/pose/pose/position 2>/dev/null | grep -E 'x:|y:' | tr '\n' ' ')"
say "EV=$EV"
