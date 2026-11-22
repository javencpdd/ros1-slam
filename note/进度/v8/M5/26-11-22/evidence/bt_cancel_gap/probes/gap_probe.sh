#!/usr/bin/env bash
# gap_probe.sh —— 「第二写者取消后重发，BT 多久才接」的 8 秒窗探针运行器（在 ros1-slam-sim 容器内跑）
#   bash gap_probe.sh <输出目录>            # 默认三轮：direct ×2（留失败证据）＋ bt ×1（反控）
#   ROUNDS=direct bash gap_probe.sh <输出目录>
#
# 为什么复制一份而不改 b2_probes.sh：那个目录是 `T-20261122-02` 的**历史证据**，冻结不动。
# 本件的栈起法与它同形（同 launch、同就绪门、同端口退净），差别只在轮次表与事后统计。
#
# ★每轮各起一套独立栈★：上一轮的取消会把适配器 `halted` 置真、把 BT 的
#   `cancel_requested` 与队列都搅进状态，串在同一套栈上读数没有意义（B2 第一版踩过，留档在
#   b2_probes/run1）。
#
# 事后统计（都写进 probes.log，作为探针 stdout 的旁证）：
#   · `arrival timeout after` 条数 —— direct 轮应 ≥1，说明 mission 是**跑满预算**才结束的；
#   · `goal queued` 条数与最大 depth —— 证明目标是「入队」而不是「被拒」；
#   · `dequeued` 条数 —— 若在 8 秒窗内为 0、收尾后才连打，正是缺口形状。
set -uo pipefail

OUT="${1:?输出目录}"
HERE="$(cd "$(dirname "$0")" && pwd)"
READY_GATE="$HERE/../../b2_probes/_ready_gate.py"
[ -f "$READY_GATE" ] || { echo "前置缺失：$READY_GATE 不在位"; exit 9; }
mkdir -p "$OUT"
LOG="$OUT/probes.log"
: > "$LOG"

export ROS_MASTER_URI=http://127.0.0.1:11313
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
export ROS_HOME="$OUT/ros_home"
export PYTHONNOUSERSITE=1
export LIBGL_ALWAYS_SOFTWARE=1
mkdir -p "$ROS_HOME"
[ -n "${DISPLAY:-}" ] || export DISPLAY=:99

source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
source /opt/ros1-slam/far_overlay/devel/setup.bash

say() { printf '%s\n' "$*" | tee -a "$LOG"; }

LAUNCH_PID=""
stop_pid() {
    local pid="$1" sig="$2"
    [ -n "$pid" ] || return 0
    kill -0 "$pid" 2>/dev/null || return 0
    kill "-$sig" "$pid" 2>/dev/null || true
    local _i
    for _i in $(seq 1 50); do
        kill -0 "$pid" 2>/dev/null || return 0
        sleep 0.2
    done
}
cleanup() {
    stop_pid "$LAUNCH_PID" INT
    stop_pid "$LAUNCH_PID" TERM
    stop_pid "$LAUNCH_PID" KILL
    [ -n "$LAUNCH_PID" ] && wait "$LAUNCH_PID" 2>/dev/null || true
    LAUNCH_PID=""
}
trap cleanup EXIT

wait_ports_free() {
    python3 - <<'PY' >> "$LOG" 2>&1
import socket, time
deadline = time.monotonic() + 40
while time.monotonic() < deadline:
    busy = []
    for port in (11313, 11345):
        with socket.socket() as sock:
            sock.settimeout(0.5)
            if sock.connect_ex(('127.0.0.1', port)) == 0:
                busy.append(port)
    if not busy:
        print('GAP_PORTS_FREE')
        raise SystemExit(0)
    time.sleep(0.5)
print('GAP_PORTS_STILL_BUSY')
raise SystemExit(1)
PY
}

run_round() {  # $1 轮名  $2 模式
    local name="$1" mode="$2"
    local out="$OUT/round_$name"
    mkdir -p "$out"
    say "=== round_$name (mode=$mode) ==="

    python3 - <<'PY' >> "$LOG" 2>&1 || { say "ROUND_$name=PORT_BUSY"; return 9; }
import socket
for port in (11313, 11345):
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', port))
print('GAP_PRECHECK=PORTS_FREE')
PY

    roslaunch lite3_bringup mode_sim_bt.launch \
        global_backend:=far \
        arrival_topic:=/v8/bt/arrived > "$out/launch.log" 2>&1 &
    LAUNCH_PID=$!

    python3 - <<'PY' >> "$LOG" 2>&1 || { say "ROUND_$name=MASTER_TIMEOUT"; cleanup; return 5; }
import socket, time, xmlrpc.client
socket.setdefaulttimeout(1.0)
master = xmlrpc.client.ServerProxy('http://127.0.0.1:11313')
end = time.monotonic() + 90
while time.monotonic() < end:
    try:
        ok, _v, ready = master.getParam('/bt_runtime_preflight', '/use_sim_time')
        if ok == 1 and ready is True:
            print('GAP_BOOT=MASTER_READY')
            break
    except (OSError, xmlrpc.client.Error):
        pass
    time.sleep(0.2)
else:
    raise SystemExit(1)
PY
    if ! grep -q 'GAP_BOOT=MASTER_READY' "$LOG"; then
        cleanup
        return 5
    fi

    if grep -q 'far_planner-12] process has died' "$out/launch.log"; then
        say "ROUND_$name=FAR_DEAD $(grep -o 'exit code -[0-9]*' "$out/launch.log" | head -1)"
        cleanup
        return 8
    fi

    python3 "$READY_GATE" > "$out/ready_gate.log" 2>&1
    say "ROUND_$name READY_GATE=$(grep -o 'READY_GATE=[A-Z_]*' "$out/ready_gate.log" | head -1)"
    if ! grep -q 'READY_GATE=READY' "$out/ready_gate.log"; then
        say "ROUND_$name=NOT_READY"
        cleanup
        return 6
    fi

    local t0 t1
    t0=$(date +%s)
    python3 "$HERE/_regoal_second_writer.py" --mode "$mode" --tag "$name" \
        > "$out/second_writer.log" 2>&1
    local rc=$?
    t1=$(date +%s)
    say "ROUND_$name PROBE_RC=$rc ELAPSED_S=$((t1 - t0))"

    cleanup
    wait_ports_free || { say "ROUND_$name=PORT_NOT_FREE"; return 7; }

    # 事后统计必须在 cleanup 之后做：roslaunch 的 output="screen" 是走管道的，
    # 栈还活着时 bt_console 那一堆 stdout 还在缓冲里（坑9.9：实测有上百秒延迟）。
    # 第一版把这四行放在 cleanup 之前，于是三轮全读到 0——那是「没落盘」不是「没发生」。
    sleep 1
    local timeout_n queued_n depth_max dequeued_n
    timeout_n=$(grep -c 'arrival timeout after' "$out/launch.log" 2>/dev/null || true)
    queued_n=$(grep -c 'goal queued' "$out/launch.log" 2>/dev/null || true)
    depth_max=$(grep -a -o 'depth=[0-9]*' "$out/launch.log" 2>/dev/null | sed 's/depth=//' | sort -n | tail -1)
    dequeued_n=$(grep -c 'dequeued' "$out/launch.log" 2>/dev/null || true)
    say "ROUND_$name STATS arrival_timeout=$((timeout_n)) goal_queued=$((queued_n)) depth_max=${depth_max:-0} dequeued=$((dequeued_n))"
    return 0
}

ROUNDS="${ROUNDS:-direct,direct,bt}"
declare -a RC=()
i=0
for mode in ${ROUNDS//,/ }; do
    i=$((i + 1))
    name="$mode$i"
    run_round "$name" "$mode"
    RC+=("$mode=$?")
done

say "GAP_PROBES_DONE ${RC[*]}"
exit 0
