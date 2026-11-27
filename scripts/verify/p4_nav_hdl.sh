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
# ★计数口径要含 FAR 侧★（2026-11-27 实测）：FAR 图里 global 实例被 include 取代，节点名是
# /v8_global_far 与 /far_planner（都不带 /v8/ 前缀）。只数 ^/v8/3d|^/v8/bt 时 M2 图最多数到 8，
# 却按 EXPECT_NODES=10 判 ⇒ "编排未起来（8/10）" 属于口径造成的假失败。
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
gate_fail() {
  say "GATE_FAIL: $*"
  if [ -n "${LPID:-}" ]; then
    scene_dump
    # 按**本轮进程所有权**清理：只杀自己起的那个会话组
    kill -TERM -"$LPID" 2>/dev/null || kill -TERM "$LPID" 2>/dev/null || true
    sleep 2
    kill -KILL -"$LPID" 2>/dev/null || true
  fi
  exit 9
}

source /opt/ros/noetic/setup.bash
source $WS/devel/setup.bash
# ★FAR 后端需要 overlay，且必须在这里补★：`$WS/devel/setup.bash` 会把 ROS_PACKAGE_PATH
# **重置**成它自己那套，调用方在外面 source 过的 overlay 路径会被抹掉 ⇒ roslaunch 报
# `Resource not found: far_planner`、整图 0 节点（实测两轮都是这条）。
# 只在图里确实引用 far_planner 时追加，support 组合行为完全不变。
# 判据用"overlay 目录存在"而不是 grep 图内容：图里写的是组件名（v8.fw.global_far），
# 字面 far_planner 只出现在 manifest 与被 include 的 launch 里，grep 图必然落空（实测踩到）。
# overlay/src 只含 far_planner 与其 msgs，与主 ws 无重叠，无条件追加是安全的。
if [ -d "$REPO/tooling/far_overlay/src" ]; then
  # ★统一入口（裁决①）★：不要手写追加。overlay 的 setup.bash 会**覆盖**四个变量，
  # 少任何一个都致命：ROS_PACKAGE_PATH（找不到包）/ CMAKE_PREFIX_PATH（找不到 ws 的 C++ 可执行）
  # / LD_LIBRARY_PATH（nodelet dlopen 失败）/ PYTHONPATH（sim_adapter 起不来）。
  # `tooling/far_overlay/env.sh` 就是为此写的：source 两边后显式补回并去重。
  # shellcheck disable=SC1090
  source "$REPO/tooling/far_overlay/env.sh"
  say "[env] far_planner pkg=$(rospack find far_planner 2>/dev/null)"
  FB="$REPO/tooling/far_overlay/devel/lib/far_planner/far_planner"
  if [ -f "$FB" ]; then
    say "[env] far_planner bin=$(readlink -f "$FB") sha256=$(sha256sum "$FB" | cut -c1-16)"
  else
    say "[env] WARN: 未找到 far_planner 可执行（$FB）"
  fi
fi
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
# ★残留 bt_console 会占住 ZMQ 5555/5556★（bt_console.cpp:1566-1568），新实例启动即
# `terminate called after throwing an instance of 'zmq::error_t'  what(): Address already in use`，
# 整图少一个节点、BT 不工作。rosnode kill 未必让进程退出，这里按**进程名**补一刀
# （用 -x 不用 -f：-f 会匹配到执行本脚本的 bash 自身，曾把自己杀掉）。
pkill -x bt_console 2>/dev/null || true
# ★FAR 后端：先清同名残留★（2026-11-27 实测）——上一轮的 `/far_planner` 还在 master 里时，
# 新的同名节点会以 "new node registered with same name" 自我关停，于是门禁看到"节点缺失"，
# 而 graph.log 里只有一行 WARN，很容易被当成环境问题。
# ★所有 3d/bt 实例名都要清★（2026-11-27 实证）：只清 FAR 三个名字不够——
# 上一轮残留的 `/v8/3d/map/map` 在 master 里挂着时，新起的 map 进程 init 即退出
# （exit 1、连 READY 都不打），于是 /v8_support_map 无发布者、就绪判红、整组被清，
# 表面上像"map 节点自己崩了"。这里按前缀把本轮要起的实例全清一遍。
for _n in $(rosnode list 2>/dev/null | grep -E '^/v8/3d|^/v8/bt|^/v8_global_|^/far_planner' || true); do
  rosnode kill "$_n" >/dev/null 2>&1 || true
done
sleep 1
for _p in 'far_planner/far_planner' 'run_global_far.py'; do
  pkill -f "$_p" 2>/dev/null || true
done
sleep 2
sleep 4
say "v8_nodes_after_clean=$(rosnode list 2>/dev/null | grep -cE '^/v8/3d|^/v8/bt|^/v8_global_|^/far_planner' || true)"

# ---------- 3. 启动装配 ----------
say "--- [3] 启动装配 ---"
# --screen 是必须的：不加的话各节点的 ROS_INFO 只进 rosout，graph.log 是空的，
# 行为树/规划器为什么失败完全看不到（bb-bt-bt-5-stdout.log 实测 0 行）。
# ★roslaunch 必须由**同一个 shell** 启动并 wait★：`setsid roslaunch &` 再在子 shell 里
# `wait $!` 是无效的——那个 PID 不是子 shell 的子进程，wait 立刻返回 127，写进 rc 的是
# 一个假码。这里用 `sh -c` 包一层：roslaunch 是那个 sh 的子进程，由它自己 wait 并把真实
# 退出码落盘；setsid 放在最外层保证脱离当前会话。
setsid sh -c 'roslaunch --screen "$1" > "$2" 2>&1; echo $? > "$3"' _ \
  "$LAUNCH" "$EV/graph.log" "$EV/roslaunch.rc" < /dev/null &
LPID=$!
say "roslaunch_pid=$LPID"

# ★失败现场保留（裁决③）★：门禁失败时先别退出——留 6 s 让 roslaunch 报出
# `process has died [pid, exit code, signal]`，同时把节点/URI/进程树/命令行一起落盘。
scene_dump() {
  local out="$EV/scene"; mkdir -p "$out"
  sleep 6
  rosnode list > "$out/nodes.txt" 2>&1 || true
  rosnode info /far_planner > "$out/far_planner_xmlrpc.txt" 2>&1 || true
  rosnode info /v8_global_far > "$out/v8_global_far_xmlrpc.txt" 2>&1 || true
  pgrep -af 'far_planner|run_global_far|roslaunch' > "$out/procs.txt" 2>&1 || true
  ps -ef --forest > "$out/ps_tree.txt" 2>&1 || true
  [ -f "$EV/roslaunch.rc" ] && cp "$EV/roslaunch.rc" "$out/roslaunch.rc" 2>/dev/null || true
  tail -80 "$EV/graph.log" > "$out/graph_tail.txt" 2>&1 || true
  grep -aE 'process has died|exit code|signal' "$EV/graph.log" > "$out/died.txt" 2>&1 || true
  printf 'launch_pid=%s\n' "$LPID" > "$out/launch_pid.txt"
  say "[scene] 已落盘到 $out（died.txt / graph_tail.txt / procs.txt / xmlrpc）"
  say "[scene] died.txt: $(cat "$out/died.txt" 2>/dev/null | head -3 | tr '\n' ' ')"
}
for i in $(seq 1 20); do
  sleep 2
  N=$(rosnode list 2>/dev/null | grep -cE '^/v8/3d|^/v8/bt|^/v8_global_|^/far_planner' || true)
  [ "$N" -ge "$EXPECT_NODES" ] && { say "nodes_ready_after=${i}x2s n=$N"; break; }
done
N=$(rosnode list 2>/dev/null | grep -cE '^/v8/3d|^/v8/bt|^/v8_global_|^/far_planner' || true)
say "v8_nodes=$N (expect>=$EXPECT_NODES)"
rosnode list 2>/dev/null | grep -E '^/v8/3d|^/v8/bt|^/v8_global_|^/far_planner' | tr '\n' ' '; say
# ★就绪判据不止"数够个数"★（2026-11-27）：个数骗过一次——loc 因 KeyError 退出后
# 残留注册还在、别的实例又多起来，个数照样够。这里加一层"身份 + 存活 + 关键接口活数据"，
# 直接复用本脚本已渲染出的 plan.json，不新建平行系统。
if [ -f "$EV/plan.json" ]; then
  python3 "$REPO/scripts/verify/graph_readiness.py" "$EV/plan.json" \
      --window "${READY_WINDOW:-6}" --json-out "$EV/readiness.json" || \
      gate_fail "编排未就绪（身份/存活/活数据，详见 readiness.json）"
else
  say "[readiness] 无 plan.json，跳过身份级判定（仅按个数）"
fi

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
# ★FAR 组合的额外就绪判据（裁决②）★：节点存在 + 有 PID + 唯一发布者 + 有活数据。
# 只看 v8 节点数会漏掉 far_planner/v8_global_far —— 它们不在 /v8 前缀下，之前正是这样
# "数量通过、规划器其实没起来"。
if printf '%s' "$GRAPH" | grep -q 'far'; then
  for n in /far_planner /v8_global_far; do
    rosnode list 2>/dev/null | grep -qx "$n" || gate_fail "FAR 节点缺失：$n（graph.log 见 $EV/graph.log）"
  done
  FPID=$(rosnode info /far_planner 2>/dev/null | grep -oE 'Pid: [0-9]+' | head -1 | cut -d' ' -f2)
  [ -n "${FPID:-}" ] || gate_fail "far_planner 无 PID（节点注册但进程不在）"
  say "far_planner_pid=$FPID"
  for t in /terrain_cloud /v8/3d/global/path_task; do
    P=$(rostopic info "$t" 2>/dev/null | sed -n '/Publishers/,/Subscribers/p' | grep -c '^ \*' || true)
    [ "${P:-0}" -ge 1 ] || gate_fail "$t 无发布者（FAR 适配层未接上）"
  done
  say "far_publishers_ok=1"
fi


# ---------- 3b. FAR 后端：等地形就绪再发目标 ----------
# FAR 要先有地形点云才能建可见性图、才可能出路径；地形没起来就发目标，BT 只会 30 s 超时。
# 支持后端（global planner）不需要这一步，故用渲染出的 launch 里是否含 far_stack 来判定。
if grep -q "far_stack" "$LAUNCH" 2>/dev/null; then
  say "--- [3b] FAR 后端：等 /terrain_cloud 有活数据 ---"
  TER_OK=0
  for i in $(seq 1 30); do
    out=$(timeout 5 rostopic hz /terrain_cloud 2>/dev/null || true)
    if printf '%s' "$out" | grep -q "average rate"; then TER_OK=1; say "terrain_ready_after=${i}x2s"; break; fi
    sleep 2
  done
  [ "$TER_OK" = "1" ] || say "WARN /terrain_cloud 60s 内无数据（FAR 可能无路径）"
fi

# ---------- 4. 探针 + 发目标 ----------
if [ -n "${SKIP_PROBE:-}" ]; then
  if [ -n "${NO_GOAL:-}" ]; then
    say "--- [4] NO_GOAL：只起装配，目标由调用方发（探针需先于目标启动）---"
    say "EV=$EV"
    exit 0
  fi
  say "--- [4] SKIP_PROBE：只发 start/goal，探针由调用方接管 ---"
  timeout 3 rostopic pub -r 5 /v8/3d/global/start geometry_msgs/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: $START_X, y: $START_Y, z: 0.0}, orientation: {w: 1.0}}}" \
    >/dev/null 2>&1
  sleep 1
  # ★只发一条★：/v8/bt/goal 是 FIFO 多任务入口，`-r 5` 三秒=15 个任务，会把同一件事
  # 拆成 15 个 mission（每出队一个就重发全局路径 + follow path，还会把上一个记成 reached）。
  python3 "$REPO/scripts/verify/bt_goal_once.py" --x "$GOAL_X" --y "$GOAL_Y" \
    --out "$EV/goal_once.json" > "$EV/goal_once.log" 2>&1 || \
    say "WARN 目标未被 BT 受理（见 $EV/goal_once.log）"
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
# ★只发一条★（同上）：重复发会把目标刷成多个 mission
python3 "$REPO/scripts/verify/bt_goal_once.py" --x "$GOAL_X" --y "$GOAL_Y" \
  --out "$EV/goal_once.json" > "$EV/goal_once.log" 2>&1 || \
  say "WARN 目标未被 BT 受理（见 $EV/goal_once.log）"
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
