#!/usr/bin/env bash
# B5 §4.5 验收第 4 条 · support 分支零回归（在 ros1-slam-sim 容器内执行）
#   bash b5_support_zero.sh <输出目录>
#
# 只验「far 分支的到达判据没有漏到 support 分支」，不做运动：
#   1. `~arrival_topic` 为空（配置面）：support 分支不该给它值；
#   2. `/v8/bt/arrived` 上发布者与订阅者都为 0（行为面）：bt_console 只在 `arrival_topic`
#      非空时才建这个订阅（`bt_console.cpp:1286-1289`），support 分支连订阅都不该有；
#   3. 启动日志里的 `pose-based arrival mode`（`bt_console.cpp:1283`）只作旁证——
#      实测该行进文件的延迟可达上百秒，拿它当门会假红（第一轮就是这么红的）。
#
# A 链行为面的逐项对照不在这里做：B0 基线用的是 `bash docker/sim.sh smoke stageB`
# （`SUMMARY marks pass=9 fail=0`／`bt_outcome=reached dt=11.13s`），原样重跑一次才能逐项比，
# 见同目录 `support_zero_smoke/`。
set -uo pipefail

OUT="${1:?输出目录}"
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$OUT"
LOG="$OUT/support_zero.log"
: > "$LOG"

export ROS_MASTER_URI=http://127.0.0.1:11313
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
export ROS_HOME="$OUT/ros_home"
export PYTHONNOUSERSITE=1
mkdir -p "$ROS_HOME"

source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
source /opt/ros1-slam/far_overlay/devel/setup.bash

say() { printf '%s\n' "$*" | tee -a "$LOG"; }

python3 - <<'PY' >> "$LOG" 2>&1 || { say "SUP_PRECHECK=PORT_BUSY"; exit 9; }
import socket
for port in (11313, 11345):
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', port))
print('SUP_PRECHECK=PORTS_FREE')
PY
grep -q 'SUP_PRECHECK=PORTS_FREE' "$LOG" || exit 9

LAUNCH_PID=""
stop_pid() {
    local pid="$1" sig="$2"
    [ -n "$pid" ] || return 0
    kill -0 "$pid" 2>/dev/null || return 0
    kill "-$sig" "$pid" 2>/dev/null || true
    local _i
    for _i in $(seq 1 40); do
        kill -0 "$pid" 2>/dev/null || return 0
        sleep 0.2
    done
}
cleanup() {
    stop_pid "$LAUNCH_PID" INT
    stop_pid "$LAUNCH_PID" TERM
    stop_pid "$LAUNCH_PID" KILL
    [ -n "$LAUNCH_PID" ] && wait "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT

# global_backend 不传＝缺省 support；arrival_topic 也不传＝空
roslaunch lite3_bringup mode_sim_bt.launch enable_bt:=true > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!

python3 - <<'PY' >> "$LOG" 2>&1 || { say "SUP_BOOT=MASTER_TIMEOUT"; exit 5; }
import socket, time, xmlrpc.client
socket.setdefaulttimeout(1.0)
master = xmlrpc.client.ServerProxy('http://127.0.0.1:11313')
end = time.monotonic() + 120
while time.monotonic() < end:
    try:
        ok, _v, ready = master.getParam('/bt_runtime_preflight', '/use_sim_time')
        if ok == 1 and ready is True:
            print('SUP_BOOT=MASTER_READY')
            break
    except (OSError, xmlrpc.client.Error):
        pass
    time.sleep(0.2)
else:
    raise SystemExit(1)
PY
grep -q 'SUP_BOOT=MASTER_READY' "$LOG" || exit 5

# 等 bt_console 真的起来：`/v8/bt/goal` 有订阅者即为起（BT 是它唯一的订阅者）
for _ in $(seq 1 600); do
    BT_GOAL_SUBS=$(timeout 10 rostopic info /v8/bt/goal 2>/dev/null | awk '
        /^Subscribers:/ {insub=1; next}
        insub && /^[[:space:]]*\*/ {n++}
        END {print n+0}')
    [ "${BT_GOAL_SUBS:-0}" -ge 1 ] && break
    sleep 0.5
done
say "SUP_BT_GOAL_SUBSCRIBERS=${BT_GOAL_SUBS:-0}"

# ★ 判定用活体状态，不用启动日志 ★ 实测 bt_console 的 stdout 进文件有上百秒的缓冲延迟：
#   本轮 `pose-based arrival mode` 那一行最后确实在文件里（`grep -c` = 1），但在脚本读取的
#   那一刻还没落盘，于是被判成 NO_POSE_BASED_MODE。判据挂在「写入时机」上就会假红。
#   活体判据两条：
#     ① `~arrival_topic` 为空（配置面）：support 分支不该给它值；
#     ② `/v8/bt/arrived` 上发布者与订阅者都为 0（行为面）：bt_console 只在
#        `arrival_topic` 非空时才建这个订阅（`bt_console.cpp:1286-1289`）。
ARRIVAL_PARAM=$(timeout 10 rosparam get /bt_console/arrival_topic 2>/dev/null \
    | tr -d '\r' | tr -d "\"'" | tr -d '[:space:]')
INFO=$(timeout 15 rostopic info /v8/bt/arrived 2>/dev/null)
PUBS=$(printf '%s\n' "$INFO" | awk '
    /^Publishers:/ {inpub=1; next}
    /^Subscribers:/ {inpub=0}
    inpub && /^[[:space:]]*\*/ {n++}
    END {print n+0}')
SUBS=$(printf '%s\n' "$INFO" | awk '
    /^Subscribers:/ {insub=1; next}
    insub && /^[[:space:]]*\*/ {n++}
    END {print n+0}')
MODE_POSE=$(grep -c 'pose-based arrival mode' "$OUT/launch.log")
MODE_SIGNAL=$(grep -c 'external arrival signal mode' "$OUT/launch.log")
say "SUP_ARRIVAL_PARAM=${ARRIVAL_PARAM:-<unset>}"
say "SUP_ARRIVED_PUBLISHERS=$PUBS SUP_ARRIVED_SUBSCRIBERS=$SUBS"
say "SUP_LOG_ADVISORY pose_based=$MODE_POSE signal=$MODE_SIGNAL"

rosnode list > "$OUT/nodes.txt" 2>&1 || true
say "SUP_NODES=$(wc -l < "$OUT/nodes.txt")"

RC=0
[ -z "${ARRIVAL_PARAM}" ] || { say "SUP_FAIL=ARRIVAL_PARAM_SET:$ARRIVAL_PARAM"; RC=1; }
[ "$PUBS" -eq 0 ] || { say 'SUP_FAIL=ARRIVED_PUBLISHER_PRESENT'; RC=1; }
[ "$SUBS" -eq 0 ] || { say 'SUP_FAIL=ARRIVED_SUBSCRIBER_PRESENT'; RC=1; }
[ "$RC" -eq 0 ] && say 'SUP_VERDICT=PASS'

cleanup
LAUNCH_PID=""

python3 - <<'PY' >> "$LOG" 2>&1
import socket
for port in (11313, 11345):
    with socket.socket() as sock:
        sock.settimeout(1.0)
        assert sock.connect_ex(('127.0.0.1', port)) != 0, 'port still listening: %s' % port
print('SUP_CLEANUP_PORTS_FREE')
PY
say "SUP_DONE rc=$RC"
exit "$RC"
