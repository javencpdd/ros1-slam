#!/usr/bin/env bash
# B5 §4.5 到达验收（v2）· 在 ros1-slam-sim 容器内执行
#   bash b5_arrival2.sh <输出目录>
#
# 与 v1（`../b5_arrival/b5_arrival.sh`）的差别，全部来自 v1 读数的误读复盘：
#   1. 记录器**先起**，任何 goal 之前就订阅好（v1 是发完 goal 才起观察节点，
#      一次性到达帧早于观察者，于是 `ARRIVED_TRUE=0` 与 `PATH_UNIQ_STAMPS=0`
#      被读成「判据正确」，其实是观察者比事件晚到）。
#   2. 加了**正向对照** C1：0.2 m 目标必须出现 arrived=true。
#      没有这一条，C2／C4 的零与「通道根本发不出 true」分不开。
#   3. C4 的顺序改正：先抬位姿、回读确认，再发目标（v1 先发目标，目标当场被判到达，
#      之后才抬位姿，等于没测）。
#   4. （v3）记录器的 READY 门加第 4 条：`/terrain_cloud` 里必须有障碍格。失败轮的
#      `[local] obstacles ok: 0` × 27 条说明障碍链路整场全空，此时发目标不是「判据失真」，
#      是「上游还没起步」，两者不能记成同一件事。
#   5. （v3）驱动前先看 `far_planner` 有没有段错误退出，命中即 `ARRIVAL_FAR_DEAD` 退 8。
#
# 链路：mode_sim_bt.launch global_backend:=far enable_bt:=false
#   ⇒ 仿真 + 位姿源 + far_stack + path_goal + local_planner 全在，BT 不来抢 /v8/local/goal。
# 判据面：/v8/bt/arrived（适配器按 |pose−goal|_3d ≤ hypot(0.25,0.30) 自判，latch 发布）。
#
# ★ 单层仿真里没有第二层楼板 ★ C2／C4 只证明「不误判」，不证明「判对层」；
#   「到对了楼层」要 support_map 按 (x,y) 取 z（押后项 D1）与真实双层图，见计划 §4.5。
#
# 所有等待一律按墙钟（time.monotonic）：仿真 /use_sim_time 为真，实测墙钟只推进约 0.44 倍。
set -uo pipefail

OUT="${1:?输出目录}"
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$OUT"
LOG="$OUT/arrival.log"
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

# 端口必须空闲：别的进程占着 11313/11345 时，后面的判据全会串味
python3 - <<'PY' >> "$LOG" 2>&1 || { say "ARRIVAL_PRECHECK=PORT_BUSY"; exit 9; }
import socket
for port in (11313, 11345):
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', port))
print('ARRIVAL_PRECHECK=PORTS_FREE')
PY
grep -q 'ARRIVAL_PRECHECK=PORTS_FREE' "$LOG" || exit 9

LAUNCH_PID=""
RECORDER_PID=""
# 收尾必须能自己走完：v2 第一次跑卡在 `wait` 上——记录器是 `disable_signals=True` 起的，
# SIGINT 到不了它的主循环，于是 `wait` 一直不返回。这里 INT→TERM→KILL 逐级升级。
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
    stop_pid "$RECORDER_PID" TERM
    stop_pid "$LAUNCH_PID" INT
    stop_pid "$RECORDER_PID" KILL
    stop_pid "$LAUNCH_PID" TERM
    stop_pid "$LAUNCH_PID" KILL
    [ -n "$RECORDER_PID" ] && wait "$RECORDER_PID" 2>/dev/null || true
    [ -n "$LAUNCH_PID" ] && wait "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT

roslaunch lite3_bringup mode_sim_bt.launch \
    global_backend:=far enable_bt:=false \
    arrival_topic:=/v8/bt/arrived \
    far_debug:=${B5_FAR_DEBUG:-false} > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!

python3 - <<'PY' >> "$LOG" 2>&1 || { say "ARRIVAL_BOOT=MASTER_TIMEOUT"; exit 5; }
import socket, time, xmlrpc.client
socket.setdefaulttimeout(1.0)
master = xmlrpc.client.ServerProxy('http://127.0.0.1:11313')
end = time.monotonic() + 90
while time.monotonic() < end:
    try:
        ok, _v, ready = master.getParam('/bt_runtime_preflight', '/use_sim_time')
        if ok == 1 and ready is True:
            print('ARRIVAL_BOOT=MASTER_READY')
            break
    except (OSError, xmlrpc.client.Error):
        pass
    time.sleep(0.2)
else:
    raise SystemExit(1)
PY
grep -q 'ARRIVAL_BOOT=MASTER_READY' "$LOG" || exit 5

# 记录器先起：它要见证 C1 那一帧一次性的 arrived=true
python3 "$HERE/_recorder.py" "$OUT/events.ndjson" > "$OUT/recorder.log" 2>&1 &
RECORDER_PID=$!
for _ in $(seq 1 700); do
    grep -q 'RECORDER_READY\|RECORDER_NOT_READY' "$OUT/recorder.log" && break
    sleep 0.2
done
grep -q 'RECORDER_READY' "$OUT/recorder.log" || { say 'ARRIVAL_RECORDER=NOT_READY'; exit 6; }
say "ARRIVAL_RECORDER=READY $(grep -o 'status=[0-9]* pose=[0-9]* arrived_conn=[0-9-]* path_conn=[0-9-]* reach=[0-9]* terrain_frames=[0-9]* obs=[0-9]*' "$OUT/recorder.log" | head -1)"

# 上游崩了就不能再往下发目标：`far_planner` 段错误（exit -11）时主循环根本不进，
# 目标会被 `!is_graph_init_` 守卫静默丢弃，最后记成「判据没测出来」——那是把
# 一个进程级故障读成判据问题。这里先看一眼，命中就如实报死，不再产出误导读数。
if grep -q 'far_planner-12] process has died' "$OUT/launch.log"; then
    say "ARRIVAL_FAR_DEAD $(grep -o 'exit code -[0-9]*' "$OUT/launch.log" | head -1)"
    exit 8
fi

python3 "$HERE/_driver.py" > "$OUT/driver.log" 2>&1
DRIVER_RC=$?
say "ARRIVAL_DRIVER_RC=$DRIVER_RC"
cat "$OUT/driver.log" >> "$LOG"

# 让收尾的 false／最后几条事件落盘，再停记录器
sleep 1
cleanup
RECORDER_PID=""
LAUNCH_PID=""

python3 "$HERE/_analyze.py" "$OUT/events.ndjson" 2>&1 | tee -a "$LOG"

python3 - <<'PY' >> "$LOG" 2>&1
import socket
for port in (11313, 11345):
    with socket.socket() as sock:
        sock.settimeout(1.0)
        assert sock.connect_ex(('127.0.0.1', port)) != 0, 'port still listening: %s' % port
print('ARRIVAL_CLEANUP_PORTS_FREE')
PY
say "ARRIVAL_DONE rc=$DRIVER_RC"
exit "$DRIVER_RC"
