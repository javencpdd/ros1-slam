#!/usr/bin/env bash
# v_scan_p4_selfmask_probe.sh —— 机体自遮挡探针的启动壳（R82；主理人 2026-10-10 提「球形 blind 换长方体」）
#
# 链比在册的 v_scan_p4_boxface_probe.sh **短一半**：本件只问传感器侧的事实（Q1 自回波有没有、Q2 球壳里多少点、
# Q3 换盒能留多少点），全部答案都在 `/rslidar_points` 原始流里 ⇒ **不起 faster-lio、不起 safety、不起 70_scan、
# 不投 goal、不发 /cmd_vel**。机体靠 SetModelState 搬（障碍物 obs_5 一字不动＝主理人口径），跑完搬回起始位。
#
# 调用姿势自证：bash ws/src/40_navigation/lite3_scan/verify/v_scan_p4_selfmask_probe.sh
# rc：0=测量成立（Q1 可答）1=站全废 2=env/锁 9=前置不成立或探针未产出判据行
set -u
ROOT=/home/ros1-slam
PROBE=$ROOT/ws/src/40_navigation/lite3_scan/verify/p4_selfmask_probe.py
WORLD=$ROOT/ws/src/95_simulation/lite3_gazebo_sim/worlds/m1_world.world
STAMP=$(date +%Y%m%d_%H%M%S)
OUT=${SM_EVIDENCE:-$ROOT/note/进度/SCAN集成/证据/log/raw/r82_selfmask_$STAMP}
LOG="$OUT/probe_shell.log"; PIDS=(); ROSCORE_PID=""

mkdir -p "$OUT" || { echo "建不出证据目录"; exit 2; }
say() { echo "[$(date '+%T')] $*" | tee -a "$LOG"; }
note() { echo "$*" | tee -a "$LOG"; }

LOCK=${SCAN_HARNESS_LOCK:-/tmp/lite3_scan_harness.lock}
if [ -f "$LOCK" ]; then
  OLD=$(cat "$LOCK" 2>/dev/null)
  if [ -n "$OLD" ] && kill -0 "$OLD" 2>/dev/null; then
    echo "P4 自遮挡探针 = INVALID|已有链上实例在跑（PID=$OLD）⇒ 并发不自证，拒跑"; exit 2
  fi
  note "陈旧锁接管（旧值=$OLD）"
fi
echo $$ > "$LOCK"

cleanup() {
  say "-- teardown --"
  rm -f "$LOCK"
  rosnode kill -a >/dev/null 2>&1; sleep 2
  for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null; done
  [ -n "$ROSCORE_PID" ] && kill "$ROSCORE_PID" 2>/dev/null
  pkill -f 'roslaunch.*m2_gazebo' 2>/dev/null; pkill -f gzserver 2>/dev/null
  pkill -f 'sim_adapter' 2>/dev/null
  sleep 4
  RES=$(ps -eo comm= | grep -cE '^(gzserver|rosmaster|roscore|rosout)$' || true)
  PORTS=$(ss -ltnH 2>/dev/null | grep -cE ':(11311|11312|11313|11345)\b' || true)
  note "退净：残留进程(comm 计)=$RES 在册端口监听=$PORTS（都应为 0）"
  sha256sum "$WORLD" > "$OUT/world_after.sha"
  N=$(find "$OUT" -type f | wc -l); note "证据件数=$N"
}
trap cleanup EXIT

set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
[ -f "$ROOT/ws/devel/setup.bash" ] && source "$ROOT/ws/devel/setup.bash"
set -u
[ "${ROS_DISTRO:-}" = "noetic" ] || { note "FATAL ROS_DISTRO=$ROS_DISTRO"; rm -f "$LOCK"; exit 2; }
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:$ROOT/ws/devel/lib
export ROS_MASTER_URI=http://127.0.0.1:11313
wait_topic() {
  local t=$1 deadline=$((SECONDS + ${2:-300}))
  while [ $SECONDS -lt $deadline ]; do
    rostopic list 2>/dev/null | grep -qx "$t" && return 0
    sleep 2
  done
  return 1
}
sha256sum "$WORLD" > "$OUT/world_before.sha"

say "== [1] roscore（11313）=="
roscore > "$OUT/roscore.log" 2>&1 &
ROSCORE_PID=$!
for _ in $(seq 1 30); do rostopic list >/dev/null 2>&1 && break; sleep 1; done
say "== [2] m2_gazebo（只起仿真＋机体＋sim_adapter；**不起建图与规划**）=="
roslaunch lite3_gazebo_sim m2_gazebo.launch gui:=false > "$OUT/gazebo.log" 2>&1 &
PIDS+=($!)
wait_topic /clock 300 || { echo "P4 自遮挡探针 = INVALID|/clock 未就绪"; exit 9; }
wait_topic /rslidar_points 480 || { echo "P4 自遮挡探针 = INVALID|/rslidar_points 未就绪"; exit 9; }
say "等 20s 让物理与传感器稳定（本件不看地图，不等收敛）"; sleep 20
note "原始流频率：$(timeout 12 rostopic hz -w 5 /rslidar_points 2>/dev/null | grep -m1 average || echo 未知)"

say "== [3] 跑自遮挡探针（障碍物不动，搬机体扫近面距离）=="
SM_OUT="$OUT" python3 "$PROBE" > "$OUT/probe_stdout.txt" 2>&1
rc=$?
VERD=$(grep -o 'P4 自遮挡探针 = [A-Z]*|.*' "$OUT/probe_stdout.txt" | tail -1)
if [ -z "$VERD" ]; then
  note "探针未产出判据行（崩溃或提前退出）⇒ 这是**跑不动**不是跑不过，按三态口径改判 INVALID"
  echo "P4 自遮挡探针 = INVALID|探针进程未产出判据行（见 probe_stdout.txt 的 traceback）；原 rc=$rc" | tee -a "$LOG"
  exit 9
fi
echo "$VERD" | tee -a "$LOG"
note "分档自证：判据行首段与 rc 必须同向（PASS↔0／FAIL↔1／INVALID↔9）；本次 rc=$rc"
grep -E '^ *(机体|注入体|躯干盒|landmark|站|Q1|Q2)' "$OUT/probe_stdout.txt" | tee -a "$LOG"
exit $rc
