#!/usr/bin/env bash
# B2 补证据 · 在 ros1-slam-sim 容器内执行
#   bash b2_probes.sh <输出目录>
#
# 为什么要补：B2 当时的四个探针只 `print` 不落盘，脚本还丢在 `.git/` 下（未入库）。
# 于是第 5／6／8／9 条验收的读数只剩验证记录里的汇总表，原始 stdout 一份都没有，
# 等于自报。本轮把脚本收进仓、把 stdout 落到文件，逐条对上卡面。
#
# ★每个探针各起一套独立栈★（round_accept／round_drift／round_stale）
#   第一版把三个探针串在同一套栈上跑，结果全被状态污染：
#     · `far_accept` 的取消把 `state.halted` 置真，之后 `far_stale` 的航点超时门
#       按规定在 halted 时不报（`far_node.py` 的 `stale()` 首句就是 `if ... or self.halted`），
#       于是 `STALE_REPORTED=False` —— 看起来像「超时门不生效」，其实是上一轮留下的 halted；
#     · 同理 `far_accept` 自己的「取消后重发」窗口也会被前序状态影响。
#   这与 `far_stale.py` 文件头自己写的「与取消门分开跑」是同一件事，第一版违反了它。
#
# 三个探针对应的卡面条目：
#   round_accept → far_accept.py     第 5（路径帧数＋z）／6（取消三项）／9（取消后重发）
#   round_drift  → _cancel_drift.py  第 8（不许重新起步＋不再接纳＋位置漂移）
#   round_stale  → far_stale.py      第 9（航点超时＋进程被杀）★会杀 far_planner，单独一轮★
#
# 所有等待一律墙钟（time.monotonic）：仿真 /use_sim_time 为真，墙钟只推进约 0.44 倍。
set -uo pipefail

OUT="${1:?输出目录}"
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$OUT"
LOG="$OUT/probes.log"
: > "$LOG"

export ROS_MASTER_URI=http://127.0.0.1:11313
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
export ROS_HOME="$OUT/ros_home"
export PYTHONNOUSERSITE=1
export LIBGL_ALWAYS_SOFTWARE=1
mkdir -p "$ROS_HOME"

# gzserver 要 GL 上下文。entrypoint 已起 Xvfb :99，但它没 export 给 docker exec 的 shell。
if [ -z "${DISPLAY:-}" ]; then
    export DISPLAY=:99
fi

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

# 端口退净才算这一轮结束：下一轮起 master 前 11313 还占着，新栈会连回旧 master
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
        print('PROBES_PORTS_FREE')
        raise SystemExit(0)
    time.sleep(0.5)
print('PROBES_PORTS_STILL_BUSY')
raise SystemExit(1)
PY
}

run_round() {  # 名字 探针文件
    local name="$1" probe="$2"
    local out="$OUT/round_$name"
    mkdir -p "$out"
    say "=== round_$name ==="

    python3 - <<'PY' >> "$LOG" 2>&1 || { say "ROUND_$name=PORT_BUSY"; return 9; }
import socket
for port in (11313, 11345):
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', port))
print('PROBES_PRECHECK=PORTS_FREE')
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
            print('PROBES_BOOT=MASTER_READY')
            break
    except (OSError, xmlrpc.client.Error):
        pass
    time.sleep(0.2)
else:
    raise SystemExit(1)
PY
    if ! grep -q 'PROBES_BOOT=MASTER_READY' "$LOG"; then
        cleanup
        return 5
    fi

    # 上游崩了就别再发目标：`far_planner` 段错误时主循环不进，目标被 !is_graph_init_ 静默丢弃，
    # 最后会被记成「判据没测出来」——那是把进程级故障读成判据问题。
    if grep -q 'far_planner-12] process has died' "$out/launch.log"; then
        say "ROUND_$name=FAR_DEAD $(grep -o 'exit code -[0-9]*' "$out/launch.log" | head -1)"
        cleanup
        return 8
    fi

    # 就绪门：地形云里必须有障碍格，否则后面的「没有路径」是上游没起步，不是判据失效
    python3 "$HERE/_ready_gate.py" > "$out/ready_gate.log" 2>&1
    say "ROUND_$name READY_GATE=$(grep -o 'READY_GATE=[A-Z_]*' "$out/ready_gate.log" | head -1)"
    if ! grep -q 'READY_GATE=READY' "$out/ready_gate.log"; then
        say "ROUND_$name=NOT_READY"
        cleanup
        return 6
    fi

    local t0 t1
    t0=$(date +%s)
    python3 "$HERE/$probe" > "$out/${probe%.py}.log" 2>&1
    local rc=$?
    t1=$(date +%s)
    say "ROUND_$name PROBE_RC=$rc ELAPSED_S=$((t1 - t0))"

    cleanup
    wait_ports_free || { say "ROUND_$name=PORT_NOT_FREE"; return 7; }
    return "$rc"
}

RC_ACCEPT=0
RC_DRIFT=0
RC_STALE=0
RC_REGOAL=0

# 只跑其中几轮：ROUNDS=regoal bash b2_probes.sh <输出目录>
ROUNDS="${ROUNDS:-accept,drift,stale,regoal}"
want() { case ",$ROUNDS," in *",$1,"*) return 0 ;; *) return 1 ;; esac; }

want accept && { run_round accept far_accept.py      || RC_ACCEPT=$?; }
want drift  && { run_round drift  _cancel_drift.py   || RC_DRIFT=$?; }
want stale  && { run_round stale  far_stale.py       || RC_STALE=$?; }
want regoal && { run_round regoal _regoal_via_bt.py  || RC_REGOAL=$?; }

say "PROBES_DONE accept=$RC_ACCEPT drift=$RC_DRIFT stale=$RC_STALE regoal=$RC_REGOAL"
exit 0
