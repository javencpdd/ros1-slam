#!/usr/bin/env bash
# v_scan_p4_inject_probe.sh —— P4 注入回读探针的启动/退净壳（#55 O-9 根因核查）
#
# 起的最小链：只有 m2_gazebo（gzserver＋世界），**不起 SCAN、不起规划器、不起 mux**
# ⇒ 本件只回答「obs_5 被 set_model_state 搬走后，到底在不在那里」，不掺建图与规划。
# 零改点：不动 p4_replan.py、不动任何门值、世界件 m1_world.world 一字不动（只运行时搬模型并还原）。
#
# 调用姿势自证（在册教训：漏 source 三步会把「跑不动」报成隔离违规）：
#   bash ws/src/40_navigation/lite3_scan/verify/v_scan_p4_inject_probe.sh
# rc：0=PASS 1=FAIL 2=前置不成立（并发锁/env）9=探针判 INVALID
set -u
ROOT=/home/ros1-slam
PROBE=$ROOT/ws/src/40_navigation/lite3_scan/verify/p4_inject_probe.py
WORLD=$ROOT/ws/src/95_simulation/lite3_gazebo_sim/worlds/m1_world.world
STAMP=$(date +%Y%m%d_%H%M%S)
OUT=${PROBE_EVIDENCE:-$ROOT/note/进度/SCAN集成/证据/log/raw/p4_inject_probe_$STAMP}
LOG=""
PIDS=()

mkdir -p "$OUT" || { echo "建不出证据目录 $OUT"; exit 2; }
LOG="$OUT/probe_shell.log"
say() { echo "[$(date '+%T')] $*" | tee -a "$LOG"; }
note() { echo "$*" | tee -a "$LOG"; }

LOCK=${SCAN_HARNESS_LOCK:-/tmp/lite3_scan_harness.lock}
if [ -f "$LOCK" ]; then
  OLD=$(cat "$LOCK" 2>/dev/null)
  if [ -n "$OLD" ] && kill -0 "$OLD" 2>/dev/null; then
    echo "P4 注入探针 = INVALID|已有链上实例在跑（PID=$OLD）⇒ 并发不自证，本件拒跑（rc=2）"; exit 2
  fi
  note "陈旧锁接管（旧值=$OLD）"
fi
echo $$ > "$LOCK"

cleanup() {
  say "-- teardown --"
  rm -f "$LOCK"
  rosnode kill -a >/dev/null 2>&1; sleep 2
  for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null; done
  pkill -f 'roslaunch.*m2_gazebo' 2>/dev/null
  pkill -f gzserver 2>/dev/null
  sleep 3
  # 残留按 comm 计数，不用无锚 pgrep -f（坑39/40：会自匹配出根本不存在的 PID）
  RES=$(ps -eo comm= | grep -cE '^(gzserver|rosmaster|roscore|rosout)$' || true)
  PORTS=$(ss -ltnH 2>/dev/null | grep -cE ':(11311|11312|11313|11345)\b' || true)
  note "退净：残留进程(comm 计)=$RES 在册端口监听=$PORTS（都应为 0）"
  sha256sum "$WORLD" > "$OUT/world_after.sha"
}
trap cleanup EXIT

set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
[ -f "$ROOT/ws/devel/setup.bash" ] && source "$ROOT/ws/devel/setup.bash"
set -u
[ "${ROS_DISTRO:-}" = "noetic" ] || { echo "FATAL: ROS_DISTRO=$ROS_DISTRO 非 noetic"; rm -f "$LOCK"; exit 2; }
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:$ROOT/ws/devel/lib
export ROS_MASTER_URI=http://127.0.0.1:11313

sha256sum "$WORLD" > "$OUT/world_before.sha"
say "== [1] 起最小链：只有 gazebo（gui:=false）=="
roslaunch lite3_gazebo_sim m2_gazebo.launch gui:=false > "$OUT/gazebo.log" 2>&1 &
PIDS+=($!)

D=$((SECONDS + 180)); READY=0
while [ $SECONDS -lt $D ]; do
  if rosservice list 2>/dev/null | grep -qx '/gazebo/get_model_state'; then READY=1; break; fi
  sleep 3
done
[ "$READY" = 1 ] || { say "P4 注入探针 = INVALID|180s 内 /gazebo/get_model_state 未出现 ⇒ 世界未起"; exit 9; }
say "服务就绪，等 8s 让物理世界稳定"; sleep 8

say "== [2] 跑回读探针 =="
PROBE_OUT="$OUT/probe.json" python3 "$PROBE" > "$OUT/probe_stdout.txt" 2>&1
rc=$?
grep -o 'P4 注入探针 = [A-Z]*|.*' "$OUT/probe_stdout.txt" | tail -1 | tee -a "$LOG"
RC4=$([ $rc = 0 ] && echo 0 || echo $rc)
say "探针 rc=$rc"
note "分档自证：判据行首段与 rc 必须同向（PASS↔0／FAIL↔1／INVALID↔9）"
head -30 "$OUT/probe.json" 2>/dev/null | tee -a "$LOG" >/dev/null
NFILES=$(find "$OUT" -type f | wc -l); note "证据件数=$NFILES"
exit $rc
