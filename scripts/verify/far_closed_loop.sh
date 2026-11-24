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
source /opt/ros/noetic/setup.bash
[ -f "$WS/devel/setup.bash" ] && source "$WS/devel/setup.bash"
if [ "$COMBO" != "graph_support" ]; then
  [ -f "$OVERLAY/devel/setup.bash" ] || die "FAR overlay 未构建：$OVERLAY/devel"
  source "$OVERLAY/devel/setup.bash"
fi
# overlay 的 setup 会覆盖这两处，显式补回；用 case 判重，避免重复 prepend 触发
# `multiple files named [x] in package [y]`。
case ":$ROS_PACKAGE_PATH:" in
  *":$WS/.profiles/x86_sim/src:"*) : ;;
  *) export ROS_PACKAGE_PATH="$WS/.profiles/x86_sim/src${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}" ;;
esac
if [ "$COMBO" != "graph_support" ]; then
  case ":$ROS_PACKAGE_PATH:" in
    *":$OVERLAY/src:"*) : ;;
    *) export ROS_PACKAGE_PATH="$OVERLAY/src${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}" ;;
  esac
  case ":$CMAKE_PREFIX_PATH:" in
    *":$OVERLAY/devel:"*) : ;;
    *) export CMAKE_PREFIX_PATH="$OVERLAY/devel${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}" ;;
  esac
fi
case ":$CMAKE_PREFIX_PATH:" in
  *":$WS/devel:"*) : ;;
  *) export CMAKE_PREFIX_PATH="$WS/devel${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}" ;;
esac

# ---------- 1. 前置：唯一 master ----------
MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11313}"
export ROS_MASTER_URI="$MASTER_URI"
N=$(pgrep -c -f "noetic/bin/[r]osmaster" || true)
[ "$N" = "1" ] || die "rosmaster 实例数=$N（必须恰好 1 个；不许双 master）"
timeout 10 rostopic list >/dev/null 2>&1 || die "master 无响应：$MASTER_URI"

# ---------- 2. 起仿真（只起一次；已起则复用） ----------
if ! timeout 5 rostopic type /rslidar_points 2>/dev/null | grep -q PointCloud2; then
  log "启动 Gazebo（m1_gazebo）"
  roslaunch lite3_gazebo_sim m1_gazebo.launch > "$EV/gazebo.log" 2>&1 &
  GZ_PID=$!
  for _ in $(seq 1 40); do
    sleep 1
    if timeout 3 rostopic type /rslidar_points 2>/dev/null | grep -q PointCloud2; then break; fi
  done
else
  GZ_PID=""
fi
timeout 6 rostopic hz /rslidar_points > "$EV/cloud_hz.txt" 2>&1
grep -q "average rate" "$EV/cloud_hz.txt" || die "无点云：/rslidar_points 未发布（查 GAZEBO_PLUGIN_PATH）"
log "点云：$(grep 'average rate' "$EV/cloud_hz.txt" | tail -1)"

# ---------- 3. 起图 ----------
case "$COMBO" in
  graph_far)     GY="$GRAPH/graph_3d_m1_hdl_bt_far.yaml" ;;
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
