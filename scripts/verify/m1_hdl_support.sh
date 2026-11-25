#!/usr/bin/env bash
# M1 + HDL + Support 基线验收入口。
#
# ★架构约束（2026-11-25 真人补）★：本脚本**不实现底座启动**，只调用统一入口
# `docker/sim.sh base`（master + Xvfb 守卫 + m1_gazebo + HDL），自己只负责"装配 + 采样 + 判定"。
# 底座逻辑唯一实现处是 sim.sh；此前每个验收脚本各写一套启动逻辑，重复且各自踩坑
# （Xvfb 死、包路径重复、话题存在但无数据），所以这里收敛。
#
# 保留两种合法装配：本脚本用 **Graph**（cli.py plan 渲染）；传统 launch 装配仍走
# `roslaunch lite3_bringup mode_sim_*.launch`，两者都不删。
#
# 用法：bash scripts/verify/m1_hdl_support.sh <evidence_dir>
# 退出码：0 链路就绪且关键话题有数据；9 前置不成立；5 判定失败
set -uo pipefail

EV="${1:?evidence dir}"
W=${W:-/opt/ros1-slam}
FW=$W/ws/src/90_platform/v8_platform/framework
GRAPH=${GRAPH:-$FW/fixtures/graphs/graph_3d_m1_hdl_bt.yaml}
mkdir -p "$EV"

log() { printf '[m1-hdl %s] %s\n' "$(date +%H:%M:%S)" "$*"; }
die() { log "ABORT: $*"; exit 9; }

PIDS=()
cleanup() {
  log "停装配（graph）并调用 sim.sh down 停底座"
  for p in "${PIDS[@]:-}"; do [ -n "${p:-}" ] && kill "$p" 2>/dev/null || true; done
  bash "$W/docker/sim.sh" down >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

# ---- 0. 与底座保持同一套 URI/环境（配置，不是重复的启动逻辑） ----
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11313}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}"
export ROS_PACKAGE_PATH="$W/ws/.profiles/x86_sim/src:/opt/ros/noetic/share"

# ---- 1. 调统一入口起底座（master / 显示守卫 / m1_gazebo / HDL 都在里面） ----
# ★底座用 no-hdl★：`graph_3d_m1_hdl_bt.yaml` **自带 HDL 组件**（manifest 里 `v8.fw.hdl_localization`
# 是 include），底座再起一份就是双 HDL，互相抢 nodelet 与 TF，实测导致整图起不来
# （carrier3d 拿不到 /gazebo/set_model_state、/v8/local/pose 无数据）。定位由**图**提供，底座只管世界与传感器。
log "调用统一底座入口：docker/sim.sh base no-hdl"
bash "$W/docker/sim.sh" base no-hdl > "$EV/base.out" 2>&1 || { cat "$EV/base.out"; die "底座未就绪"; }
source /opt/ros/noetic/setup.bash
[ -f "$W/ws/devel/setup.bash" ] && source "$W/ws/devel/setup.bash"

# ---- 2. 装配：Graph 渲染 + 启动（只做装配，不含底座） ----
log "渲染并起 3D 图（support 版）"
( cd "$FW" && python3 cli.py plan fixtures/manifests "$GRAPH" \
    --package-index fixtures/package_index.json \
    --out-plan "$EV/plan.json" --out-launch "$EV/graph.launch" ) > "$EV/plan.out" 2>&1 \
  || die "图渲染失败，见 $EV/plan.out"
roslaunch "$EV/graph.launch" > "$EV/graph.log" 2>&1 &
PIDS+=($!)
# 轮询到位姿真的有数据为止（固定 sleep 会偶发假红：carrier3d 等 /gazebo/set_model_state 有时偏慢）
ready=0
for _ in $(seq 1 24); do
  sleep 5
  if timeout 4 rostopic hz /v8/local/pose 2>/dev/null | grep -q "average rate"; then ready=1; break; fi
done
[ "$ready" = "1" ] || log "警告：60s 内 /v8/local/pose 仍无数据（见 $EV/graph.log）"

# ---- 3. 采样（先采后关） ----
{
  for t in /hdl_odom /v8/local/pose /v8/local/obstacles /cmd_vel; do
    echo "== $t 速率:"; timeout 6 rostopic hz "$t" 2>&1 | tail -1
  done
  echo "== /v8/local/localization_status:"; timeout 5 rostopic echo -n1 /v8/local/localization_status 2>&1 | head -3
  echo "== /v8/3d/perception/status（连采 5 条）:"; timeout 8 rostopic echo -n5 /v8/3d/perception/status 2>&1 | grep -o 'state[^,]*' | head -5
  echo "== /v8/local/status:"; timeout 5 rostopic echo -n1 /v8/local/status 2>&1 | tail -2
  echo "== 位姿与真值对照（真值只作评价）:"
  echo "-- /v8/local/pose:"; timeout 4 rostopic echo -n1 /v8/local/pose/pose/position 2>&1 | head -3
  echo "-- /sim/odom_gt:";   timeout 4 rostopic echo -n1 /sim/odom_gt/pose/pose/position 2>&1 | head -3
} > "$EV/readings.txt" 2>&1
log "读数已落盘：$EV/readings.txt"

# ---- 4. 机器判定 ----
fail=0
grep -q "average rate" <(timeout 6 rostopic hz /v8/local/pose 2>&1) || { log "FAIL: /v8/local/pose 无速率"; fail=1; }
grep -q "average rate" <(timeout 6 rostopic hz /v8/local/obstacles 2>&1) || { log "FAIL: /v8/local/obstacles 无速率"; fail=1; }
[ "$fail" = "0" ] && log "BASELINE READY" || log "BASELINE NOT READY"
exit "$fail"
