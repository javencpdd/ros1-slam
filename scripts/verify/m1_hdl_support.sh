#!/usr/bin/env bash
# M1 + HDL + Support 基线验收入口（真实验收入口：任一前置不成立即非零退出）。
#
# 依据 2026-11-23 那次"HDL support PASS / FAR INVALID"的结构复原：
#   `app_world.launch`（只起世界，不起机器人）＋ 3D 图（图内自带 carrier，载体带 livox 雷达与 IMU 传感器）
#   ＋ HDL 地图定位（`/hdl_odom`）。M2 那条 `m2_gazebo + m2_mapping` 是**扩展组合**，不替代本基线。
#
# 与旧手搓的四处差别（每处都曾有实测代价）：
#   ①显示：先过 `xvfb_guard_ensure`（Xvfb 死了必须重起，否则 gzserver 静默起不来、传感器不创建）；
#   ②就绪判据：用 `rostopic list` 判"话题是否存在"，不用 `rostopic type`（无发布者时返回空会假红）；
#   ③包路径：收敛成 profile src + noetic，杜绝 `ws/src` 与副本重复导致 `multiple files named`；
#   ④清理：EXIT trap 只收本脚本起的进程，先采读数再关栈。
#
# 用法：bash scripts/verify/m1_hdl_support.sh <evidence_dir>
# 退出码：0 链路就绪并产出读数；9 前置不成立；5 读数异常
set -uo pipefail

EV="${1:?evidence dir}"
W=/opt/ros1-slam
FW=$W/ws/src/90_platform/v8_platform/framework
GRAPH=$FW/fixtures/graphs/graph_3d_m1_hdl_bt.yaml
MAP=m1_world_synth_v1
mkdir -p "$EV"

log() { printf '[m1-hdl %s] %s\n' "$(date +%H:%M:%S)" "$*"; }
die() { log "ABORT: $*"; exit 9; }

PIDS=()
cleanup() {
  log "清理本次启动的进程（${#PIDS[@]} 个）"
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done
  sleep 2
}
trap cleanup EXIT INT TERM

# ---------- 0. 环境（support 不需要 FAR overlay） ----------
source /opt/ros/noetic/setup.bash
[ -f "$W/ws/devel/setup.bash" ] && source "$W/ws/devel/setup.bash"
. "$W/docker/xvfb_guard.sh"
xvfb_guard_ensure "${DISPLAY:-:99}" || die "显示服务不可用"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11313}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}"
export ROS_PACKAGE_PATH="$W/ws/.profiles/x86_sim/src:/opt/ros/noetic/share"
export GAZEBO_PLUGIN_PATH="$W/sim_ws/devel/lib:$W/ws/devel/lib:/opt/ros/noetic/lib"
export GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models:$W/sim_ws/src:$W/sim_ws/src/lite3_gazebo_sim/worlds"
export LIBGL_ALWAYS_SOFTWARE=1

# ---------- 1. master ----------
if ! timeout 4 rostopic list >/dev/null 2>&1; then
  log "起 master（$ROS_MASTER_URI）"
  roscore -p "${ROS_MASTER_URI##*:}" > "$EV/roscore.log" 2>&1 &
  PIDS+=($!)
  for _ in $(seq 1 20); do sleep 2; timeout 3 rostopic list >/dev/null 2>&1 && break; done
fi
timeout 5 rostopic list >/dev/null 2>&1 || die "master 无应答"

wait_topic() {  # $1=话题 $2=超时秒
  local t="$1" n="$2" i=0
  while [ "$i" -lt "$n" ]; do
    rostopic list 2>/dev/null | grep -qx "$t" && return 0
    sleep 1; i=$((i+1))
  done
  return 1
}

# ---------- 2. 世界 + 图（图内 spawn 载体，带雷达与 IMU） ----------
# ★底座改为 m1_gazebo★：`app_world` 只起世界、机器人由图内 carrier3d spawn，实测该路径下
# IMU 传感器不创建（`/rslidar_points` 有、`/livox/imu` 无）。`m1_gazebo.launch` 用 spawn_model
# 起 `lite3_planar.urdf.xacro`，雷达与 IMU 传感器同源创建（该文件的插件路径本轮已补）。
log "起 m1_gazebo（m1 世界 + 机器人 + 传感器）"
roslaunch lite3_gazebo_sim m1_gazebo.launch > "$EV/world.log" 2>&1 &
PIDS+=($!)
wait_topic /clock 60 || die "/clock 未出（世界没起来：先查显示与 GAZEBO_* 环境）"

log "渲染并起 3D 图（support 版）"
( cd "$FW" && python3 cli.py plan fixtures/manifests "$GRAPH" \
    --package-index fixtures/package_index.json \
    --out-plan "$EV/plan.json" --out-launch "$EV/graph.launch" ) > "$EV/plan.out" 2>&1 \
  || die "图渲染失败，见 $EV/plan.out"
roslaunch "$EV/graph.launch" > "$EV/graph.log" 2>&1 &
PIDS+=($!)

wait_topic /rslidar_points 90 || die "/rslidar_points 未出（载体未 spawn 或雷达插件未加载）"
wait_topic /livox/imu 60 || die "/livox/imu 未出（IMU 传感器未创建 ⇒ 先确认 Xvfb 与 spawn 成功）"
log "传感器就绪"

# ---------- 3. HDL 地图定位（真实坐标变换，不用恒等 TF） ----------
log "起 HDL（map=$MAP，外参取 URDF 仿真安装几何 xyz=(0,0,0.34) rpy=(0,15°,0)）"
roslaunch "$W/ws/.profiles/x86_sim/src/lite3_localization/launch/hdl_localization.launch" \
  tf_x:=0 tf_y:=0 tf_z:=0.34 tf_pitch:=0.2618 map_name:=$MAP imu_topic:=/livox/imu \
  > "$EV/hdl.log" 2>&1 &
PIDS+=($!)
wait_topic /hdl_odom 120 || die "/hdl_odom 未出（HDL 未收敛：查 $EV/hdl.log 与地图 $MAP）"
log "HDL 就绪：/hdl_odom"

# ---------- 4. 先采读数，再关栈 ----------
{
  for t in /hdl_odom /v8/local/pose /v8/local/obstacles /cmd_vel; do
    echo "== $t 速率:"; timeout 6 rostopic hz "$t" 2>&1 | tail -1
  done
  echo "== /v8/local/localization_status:"; timeout 5 rostopic echo -n1 /v8/local/localization_status 2>&1 | head -3
  echo "== /v8/3d/perception/status（连采 5 条）:"; timeout 8 rostopic echo -n5 /v8/3d/perception/status 2>&1 | grep -o 'state[^,]*' | head -5
  echo "== /v8/local/status:"; timeout 5 rostopic echo -n1 /v8/local/status 2>&1 | tail -2
} > "$EV/readings.txt" 2>&1
log "读数已落盘：$EV/readings.txt"

# ---------- 5. 机器判定（缺失即非零） ----------
fail=0
grep -q "average rate" <(timeout 6 rostopic hz /hdl_odom 2>&1) || { log "FAIL: /hdl_odom 无速率"; fail=1; }
grep -q "average rate" <(timeout 6 rostopic hz /v8/local/pose 2>&1) || { log "FAIL: /v8/local/pose 无速率"; fail=1; }
[ "$fail" = "0" ] && log "BASELINE READY（HDL+support 链路就绪）" || log "BASELINE NOT READY"
exit "$fail"
