#!/usr/bin/env bash
# B5 §4.5 验收第 5 条 · 到达预算读数（在 ros1-slam-sim 容器内执行）
#   bash b5_budget.sh <输出目录>
#
# 待验事实（计划 §4.5【连带发现】）：`WaitArrival` 在 `onStart` 里取**当时那条
# `/global_path`** 的长度并冻结成 `budget_`（`bt_console.cpp:449-476`）。FAR 单航点模式下
# 适配器发的是「位姿→航点」两点路径，若这一跳只有一两米，预算就按首段定死，而整条路线
# 可能长得多 ⇒ `elapsed > budget_` 判 FAILURE，与路走得好不好无关。
#
# 口径（真人裁：先读数值，不默认整段真跑）：把三个量摆在一起——
#   目标的直线距离（`_budget_probe.py` 的 goal_dist）、
#   BT 实际拿到的那条路径长度（path_len）、
#   冻结预算（日志里 `[bt] arrival budget %.1f s (path %.2f m, ...)`）。
# ★ 那行只在 `budget_ > timeout_` 时打印（`bt_console.cpp:471-475`）★
#   所以「没有这行」＝「预算就是树里的 60 s」，同样是读数。
set -uo pipefail

OUT="${1:?输出目录}"
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$OUT"
LOG="$OUT/budget.log"
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

python3 - <<'PY' >> "$LOG" 2>&1 || { say "BUD_PRECHECK=PORT_BUSY"; exit 9; }
import socket
for port in (11313, 11345):
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', port))
print('BUD_PRECHECK=PORTS_FREE')
PY
grep -q 'BUD_PRECHECK=PORTS_FREE' "$LOG" || exit 9

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

roslaunch lite3_bringup mode_sim_bt.launch \
    global_backend:=far enable_bt:=true \
    arrival_topic:=/v8/bt/arrived > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!

python3 - <<'PY' >> "$LOG" 2>&1 || { say "BUD_BOOT=MASTER_TIMEOUT"; exit 5; }
import socket, time, xmlrpc.client
socket.setdefaulttimeout(1.0)
master = xmlrpc.client.ServerProxy('http://127.0.0.1:11313')
end = time.monotonic() + 120
while time.monotonic() < end:
    try:
        ok, _v, ready = master.getParam('/bt_runtime_preflight', '/use_sim_time')
        if ok == 1 and ready is True:
            print('BUD_BOOT=MASTER_READY')
            break
    except (OSError, xmlrpc.client.Error):
        pass
    time.sleep(0.2)
else:
    raise SystemExit(1)
PY
grep -q 'BUD_BOOT=MASTER_READY' "$LOG" || exit 5

# BT 的到达模式用「活体」判：far 分支里 bt_console 会订阅 `/v8/bt/arrived`
# （`bt_console.cpp:1286-1289`），support 分支根本不订阅。**不要** grep 启动日志当门——
# 实测 bt_console 的 stdout 进文件有上百秒的缓冲延迟，会把「BT 正常」读成「BT 没起」。
BT_SUBS=0
for _ in $(seq 1 900); do
    BT_SUBS=$(timeout 10 rostopic info /v8/bt/arrived 2>/dev/null | awk '
        /^Subscribers:/ {insub=1; next}
        insub && /^[[:space:]]*\*/ {n++}
        END {print n+0}')
    [ "${BT_SUBS:-0}" -ge 1 ] && break
    sleep 0.5
done
say "BUD_ARRIVED_SUBSCRIBERS=${BT_SUBS:-0}"

# FAR 必须先建起可见图，否则目标会被 `!is_graph_init_` 守卫丢掉，读数就成了「没发出去」
# 这里只作**旁证**（打印读数），硬的就绪门在探针里：它等到地形云真有障碍格才发目标。
for _ in $(seq 1 600); do
    [ "$(grep -c 'Global V-Graph Updated' "$OUT/launch.log")" -ge 3 ] && break
    sleep 0.2
done
say "BUD_GRAPH_PRINTS=$(grep -c 'Global V-Graph Updated' "$OUT/launch.log")"

python3 "$HERE/_budget_probe.py" /v8/bt/goal > "$OUT/probe.log" 2>&1
PROBE_RC=$?
say "BUD_PROBE_RC=$PROBE_RC"
cat "$OUT/probe.log" >> "$LOG"

# 冻结预算的原文（可能一行都没有，那本身就是「等于树里的 60 s」）
# 两处都取：探针从 `/rosout` 实时截（准），这里从文件补（可能迟到，只作旁证）。
say "BUD_BUDGET_LINES_BEGIN"
grep -o '\[bt\] arrival budget[^"]*' "$OUT/launch.log" | tee -a "$LOG" || true
say "BUD_BUDGET_LINES_END"
say "BUD_BT_MODE_LOG=$(grep -c 'external arrival signal mode' "$OUT/launch.log")"
say "BUD_ARRIVAL_TIMEOUTS=$(grep -c 'arrival timeout after' "$OUT/launch.log")"
say "BUD_ARRIVALS=$(grep -c '\[bt\] arrived' "$OUT/launch.log")"

cleanup
LAUNCH_PID=""

python3 - <<'PY' >> "$LOG" 2>&1
import socket
for port in (11313, 11345):
    with socket.socket() as sock:
        sock.settimeout(1.0)
        assert sock.connect_ex(('127.0.0.1', port)) != 0, 'port still listening: %s' % port
print('BUD_CLEANUP_PORTS_FREE')
PY
say "BUD_DONE rc=$PROBE_RC"
exit "$PROBE_RC"
