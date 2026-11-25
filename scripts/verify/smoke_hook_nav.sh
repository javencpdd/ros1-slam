#!/usr/bin/env bash
# 冒烟钩子：在 m2_smoke 的上下文里挂 FAR 导航图并采读数（诊断优先，不判 PASS）。
#
# 为什么用钩子而不是另起底座：实测同一份 m2_gazebo.launch / URDF / 环境变量，
# 由冒烟脚本内部启动时 /livox/imu 稳定 200Hz，由外部另起进程启动时该话题**连出现都不出现**
# （`rostopic list` 无、`rostopic type` 空）。差异只在执行上下文，故改为在冒烟内部挂载。
#
# 用法（由 m2_smoke 调用，也可手动在冒烟上下文里跑）：
#   M2_SMOKE_HOOK=/opt/ros1-slam/scripts/verify/smoke_hook_nav.sh bash docker/sim.sh smoke stageA
set -uo pipefail

EV="${HOOK_EV:-/opt/ros1-slam/note/进度/00_专题/far_p2_2026-11-25/evidence/hook_nav}"
GRAPH=/opt/ros1-slam/ws/src/90_platform/v8_platform/framework/fixtures/graphs/graph_3d_m2_lio_bt_far.yaml
FW=/opt/ros1-slam/ws/src/90_platform/v8_platform/framework
mkdir -p "$EV"
log() { printf '[hook-nav %s] %s\n' "$(date +%H:%M:%S)" "$*"; }

# ---------- 1. 上下文自证：IMU / 点云 到底在不在 ----------
{
  echo "== rostopic list 里的 imu/cloud:"
  timeout 5 rostopic list 2>/dev/null | grep -Ei "imu|rslidar|cloud_registered" || true
  echo "== /livox/imu 速率:"
  timeout 6 rostopic hz /livox/imu 2>&1 | tail -1
  echo "== /rslidar_points 速率:"
  timeout 6 rostopic hz /rslidar_points 2>&1 | tail -1
} > "$EV/context.txt" 2>&1
log "上下文：$(grep -c 'average rate' "$EV/context.txt") 条速率读数（详见 context.txt）"

# ---------- 2. 渲染并启动 FAR 图 ----------
( cd "$FW" && python3 cli.py plan fixtures/manifests "$GRAPH" \
    --package-index fixtures/package_index.json \
    --out-plan "$EV/plan.json" --out-launch "$EV/graph.launch" ) > "$EV/plan.out" 2>&1
if [ $? -ne 0 ]; then log "渲染失败，见 $EV/plan.out"; exit 0; fi
# ★启动前收敛 ROS_PACKAGE_PATH★：冒烟环境里同时含 ws/src 与 sim_ws/src，同一包出现两次，
# roslaunch 解析 include 时会抛 `multiple files named [x] in package [y]`（实测：图一起就退出、
# 一个 /v8/* 话题都不出现）。这里只保留 profile src + overlay src + noetic，杜绝重复。
source /opt/ros1-slam/tooling/far_overlay/env.sh >/dev/null 2>&1 || true
export ROS_PACKAGE_PATH="/opt/ros1-slam/ws/.profiles/x86_sim/src:/opt/ros1-slam/tooling/far_overlay/src:/opt/ros/noetic/share"

roslaunch "$EV/graph.launch" > "$EV/stack.log" 2>&1 &
ST_PID=$!
log "图已起（pid=$ST_PID），等 40 s 让定位/感知收敛"
sleep 40

# ---------- 3. 采关键读数（先采集，不关栈） ----------
{
  echo "== /v8/local/localization_status:"; timeout 5 rostopic echo -n1 /v8/local/localization_status 2>&1 | head -3
  echo "== /v8/3d/perception/status（连采 6 条，看状态轮转）:"
  timeout 8 rostopic echo -n6 /v8/3d/perception/status 2>&1 | grep -o 'state[^,]*' | head -6
  echo "== /v8/local/obstacles 速率:";     timeout 6 rostopic hz /v8/local/obstacles 2>&1 | tail -1
  echo "== /v8/local/pose 速率:";          timeout 6 rostopic hz /v8/local/pose 2>&1 | tail -1
  echo "== /v8/local/status:";             timeout 5 rostopic echo -n1 /v8/local/status 2>&1 | tail -2
  echo "== /cmd_vel 速率:";                timeout 6 rostopic hz /cmd_vel 2>&1 | tail -1
} > "$EV/readings.txt" 2>&1
# ---------- 3b. 帧戳推进性（stale_frame 的真相） ----------
{
  echo "== /clock 两采样（间隔 1.5s）:"
  timeout 4 rostopic echo -n1 /clock/clock/secs 2>&1 | tail -1
  sleep 1.5
  timeout 4 rostopic echo -n1 /clock/clock/secs 2>&1 | tail -1
  echo "== /rslidar_points 两采样帧戳:"
  timeout 4 rostopic echo -n1 /rslidar_points/header/stamp 2>&1 | tail -2
  sleep 1.5
  timeout 4 rostopic echo -n1 /rslidar_points/header/stamp 2>&1 | tail -2
  echo "== 该话题的发布者/订阅者:"
  timeout 4 rostopic info /rslidar_points 2>&1 | head -8
} > "$EV/stamps.txt" 2>&1

log "读数已落盘：$EV/readings.txt（帧戳见 stamps.txt）"

kill "$ST_PID" 2>/dev/null || true
exit 0
