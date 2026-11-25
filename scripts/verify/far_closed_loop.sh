#!/usr/bin/env bash
# FAR / support 闭环验收的一键跑法（容器内运行）。
#
# 为什么要有这个脚本：此前每轮验收都靠手敲一长串命令，踩过四类环境坑并各浪费一轮：
#   ① docker exec 不加 -d 会随主进程退出清进程组 ⇒ 整链必须在**同一次 exec** 内跑完；
#   ② 镜像默认 ROS_MASTER_URI=11313，另起 roscore(11311) 会形成双 master ⇒ 本脚本只认现有 master，不自己起；
#   ③ 独立 overlay 的 setup 会覆盖 ROS_PACKAGE_PATH/CMAKE_PREFIX_PATH ⇒ 显式补回且不重复 prepend；
#   ④ pkill 的模式若出现在本命令自身命令行里会自杀 ⇒ 一律括号模式，且清场与本脚本分离。
#
# 用法（容器内）：bash scripts/verify/far_closed_loop.sh <combo> <scenario> <evidence_dir>
#   combo    : graph_far | graph_support | trad_far
#   scenario : c1（正常导航）
# 退出码：0 探针判定 PASS；5 判定 FAIL/INVALID；9 前置不成立（无 master / 无点云 / 图未起全）。
set -uo pipefail

COMBO="${1:?combo: graph_far|graph_support|trad_far}"
SCENARIO="${2:-c1}"
EV="${3:?evidence dir}"
REPO="${REPO:-/opt/ros1-slam}"
WS="$REPO/ws"
OVERLAY="$REPO/tooling/far_overlay"
GRAPH="$WS/src/90_platform/v8_platform/framework/fixtures/graphs"
mkdir -p "$EV"

log() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*" >&2; }
die() { log "ABORT: $*"; exit 9; }

# ---------- 0. 环境 ----------
if [ "$COMBO" != "graph_support" ]; then
  # ★统一入口★：不要直接 source overlay 的 setup.bash —— 它会覆盖 ROS_PACKAGE_PATH /
  # CMAKE_PREFIX_PATH / PYTHONPATH 三个变量，分别导致"包找不到""C++ 节点找不到"
  # "sim_adapter 导入 livox_laser_simulation 失败 ⇒ 无点云 ⇒ 全链零速"。env.sh 负责补回并自检。
  source "$OVERLAY/env.sh" || die "FAR 环境自检失败（见上）"
else
  source /opt/ros/noetic/setup.bash
  [ -f "$WS/devel/setup.bash" ] && source "$WS/devel/setup.bash"
fi
# ---------- 1. 前置：唯一 master ----------
# ★与项目冒烟同款★（m2_smoke.sh:31）：nav 用 11313、Gazebo 用 11345。
# 之前手搓时两个 URI 混用，是"有时有 odom、有时没有"的直接原因。
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11313}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}"
# ★按"端口是否可应答"判断，而不是数进程★：容器里可能残留一个跑在默认 11311 的 master，
# 数进程会得 1 而实际 11313 没人应答（实测踩过）。这里以 $ROS_MASTER_URI 能否应答为准。
if ! timeout 4 rostopic list >/dev/null 2>&1; then
  log "master($ROS_MASTER_URI) 不可应答，清掉旧的并显式起一个"
  pkill -f "noetic/bin/[r]osmaster" 2>/dev/null || true
  sleep 3
  roscore -p "${ROS_MASTER_URI##*:}" > "$EV/roscore.log" 2>&1 &
  for _ in $(seq 1 20); do
    sleep 2
    timeout 3 rostopic list >/dev/null 2>&1 && break
  done
fi
timeout 5 rostopic list >/dev/null 2>&1 || die "master 无响应：$ROS_MASTER_URI（见 $EV/roscore.log）"

# ---------- 2. 起仿真（只起一次；已起则复用） ----------
# ★M2 底座★：m2_gazebo（传感器）+ m2_mapping（真实 LIO → /Odometry）。
# 真值单独在 /sim/odom_gt，仅供评测；控制链只吃 /Odometry。
log "启动 M2 底座（gazebo + mapping）"
roslaunch lite3_gazebo_sim m2_gazebo.launch > "$EV/gazebo.log" 2>&1 &
GZ_PID=$!
for _ in $(seq 1 30); do
  sleep 2
  if timeout 3 rostopic info /rslidar_points 2>/dev/null | grep -q sim_adapter; then break; fi
done
roslaunch lite3_gazebo_sim m2_mapping.launch > "$EV/mapping.log" 2>&1 &
MAP_PID=$!
# ★照 m2_smoke 的等待口径★：它等的是 /cloud_registered（LIO 管线活着的证据），
# 而 /Odometry 往往要在机器人**动起来之后**才出现（冒烟是在 drive() 之后才读到 odom）。
CLOUD_OK=0
for _ in $(seq 1 24); do
  sleep 5
  if timeout 4 rostopic hz /cloud_registered 2>/dev/null | grep -q "average rate"; then CLOUD_OK=1; break; fi
done
[ "$CLOUD_OK" = "1" ] || die "/cloud_registered 未出（LIO 管线未起，先跑 stageA 判定底座）"
ODOM_OK=0
for _ in $(seq 1 12); do
  sleep 5
  if timeout 4 rostopic hz /Odometry 2>/dev/null | grep -q "average rate"; then ODOM_OK=1; break; fi
done
[ "$ODOM_OK" = "1" ] || die "LIO 未产出 /Odometry（底座不健康，先跑 docker/sim.sh smoke stageA 判定）"
log "LIO 就绪：$(timeout 5 rostopic hz /Odometry 2>&1 | grep 'average rate' | tail -1)"

# ---------- 3. 起图 ----------
case "$COMBO" in
  graph_far)     GY="$GRAPH/graph_3d_m2_lio_bt_far.yaml" ;;   # M2 底座 → 用 LIO 版对照图
  graph_far_m1)  GY="$GRAPH/graph_3d_m1_hdl_bt_far.yaml" ;;
  graph_support) GY="$GRAPH/graph_3d_m1_hdl_bt.yaml" ;;
  trad_far)      GY="";;
  *) die "未知 combo：$COMBO" ;;
esac

if [ -n "$GY" ]; then
  ( cd "$WS/src/90_platform/v8_platform/framework" && \
    python3 cli.py plan fixtures/manifests "$GY" --package-index fixtures/package_index.json \
      --out-plan "$EV/plan.json" --out-launch "$EV/graph.launch" > "$EV/plan.out" 2>&1 ) \
    || die "图渲染失败，见 $EV/plan.out"
  roslaunch "$EV/graph.launch" > "$EV/stack.log" 2>&1 &
  ST_PID=$!
  sleep 28
else
  # trad_far：手工链（far_stack + path_goal + local + mux + BT + relay），全用 M1 的同一批组件
  roslaunch v8_platform far_stack.launch > "$EV/far_stack.log" 2>&1 &
  roslaunch v8_platform run_path_goal.launch > "$EV/path_goal.log" 2>&1 || true
  ST_PID=$!
  sleep 20
fi

# ---------- 4. 节点存活前置 ----------
timeout 10 rosnode list > "$EV/nodes.txt" 2>&1
for want in "local" "bt" "mux" "global"; do
  grep -q "$want" "$EV/nodes.txt" || die "节点缺失：$want（见 $EV/nodes.txt）"
done

# ---------- 5. 跑场景 ----------
rc=0
case "$SCENARIO" in
  c1)
    timeout 180 python3 "$REPO/scripts/verify/far_closed_loop_probe.py" "$EV/probe_c1.json" \
      > "$EV/probe_c1.out" 2>&1 || rc=$?
    ;;
  *) die "未知 scenario：$SCENARIO" ;;
esac

# ---------- 6. 收尾：只清本脚本起的进程 ----------
[ -n "${ST_PID:-}" ] && kill "$ST_PID" 2>/dev/null
[ -n "${GZ_PID:-}" ] && kill "$GZ_PID" 2>/dev/null
sleep 2

# ---------- 7. 关键读数落盘（排查用） ----------
for t in /v8/local/status /v8/3d/mux/status /v8/3d/perception/status /v8/local/localization_status; do
  n=$(echo "${t#/}" | tr '/' '_')
  timeout 4 rostopic echo -n1 "$t" > "$EV/read_$n.txt" 2>&1 || true
done

if [ "$rc" = "0" ]; then log "SCENARIO $SCENARIO rc=0（看 $EV/probe_$SCENARIO.json 的 verdict）"; else log "SCENARIO $SCENARIO rc=$rc"; fi
exit "$rc"
