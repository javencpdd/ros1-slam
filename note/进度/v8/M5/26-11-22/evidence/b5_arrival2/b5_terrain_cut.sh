#!/usr/bin/env bash
# B5 §4.4 验收 2 · 只断云（在 ros1-slam-sim 容器内执行）
#   bash b5_terrain_cut.sh <输出目录>
#
# 与 §4.4 验收 1（杀 FAR 进程）的区别：本条只断「喂给 FAR 的地形」，航点与状态两个通道
# 的代码路径都不动。若判据其实盯着「点云有没有来」，两条会得出不同结果——这条就是用来
# 证明判据落在航点/状态这一层（`waypoint_timeout_s`）。
#
# 注入点：`/rslidar_points` 的发布者是 `/sim_adapter`（`rosnode info` 现读只发 /rosout
# 与 /rslidar_points 两条），杀它等于精确断云，不连带仿真时钟与 odom。
set -uo pipefail

OUT="${1:?输出目录}"
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$OUT"
LOG="$OUT/terrain_cut.log"
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

python3 - <<'PY' >> "$LOG" 2>&1 || { say "CUT_PRECHECK=PORT_BUSY"; exit 9; }
import socket
for port in (11313, 11345):
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', port))
print('CUT_PRECHECK=PORTS_FREE')
PY
grep -q 'CUT_PRECHECK=PORTS_FREE' "$LOG" || exit 9

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
    global_backend:=far enable_bt:=false \
    arrival_topic:=/v8/bt/arrived \
    far_debug:=false > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!

python3 - <<'PY' >> "$LOG" 2>&1 || { say "CUT_BOOT=MASTER_TIMEOUT"; exit 5; }
import socket, time, xmlrpc.client
socket.setdefaulttimeout(1.0)
master = xmlrpc.client.ServerProxy('http://127.0.0.1:11313')
end = time.monotonic() + 90
while time.monotonic() < end:
    try:
        ok, _v, ready = master.getParam('/bt_runtime_preflight', '/use_sim_time')
        if ok == 1 and ready is True:
            print('CUT_BOOT=MASTER_READY')
            break
    except (OSError, xmlrpc.client.Error):
        pass
    time.sleep(0.2)
else:
    raise SystemExit(1)
PY
grep -q 'CUT_BOOT=MASTER_READY' "$LOG" || exit 5

python3 "$HERE/_cut_driver.py" > "$OUT/cut.log" 2>&1
CUT_RC=$?
say "CUT_DRIVER_RC=$CUT_RC"
cat "$OUT/cut.log" >> "$LOG"

cleanup
LAUNCH_PID=""

python3 - <<'PY' >> "$LOG" 2>&1
import socket
for port in (11313, 11345):
    with socket.socket() as sock:
        sock.settimeout(1.0)
        assert sock.connect_ex(('127.0.0.1', port)) != 0, 'port still listening: %s' % port
print('CUT_CLEANUP_PORTS_FREE')
PY
say "CUT_DONE rc=$CUT_RC"
exit "$CUT_RC"
