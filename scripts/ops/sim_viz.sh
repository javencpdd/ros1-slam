#!/usr/bin/env bash
# =============================================================================
# sim_viz.sh —— 主机侧仿真可视化（sim-viz 容器 + 宿主 X/GPU）
#
#   scripts/ops/sim_viz.sh doctor          # 只读自检：X、xhost、GPU、镜像、容器、GL
#   scripts/ops/sim_viz.sh start [world]   # 起 sim-viz，容器内起 gzserver + gzclient
#   scripts/ops/sim_viz.sh status          # 容器/进程/窗口
#   scripts/ops/sim_viz.sh stop            # 停 gzclient/gzserver 与容器，撤销 xhost
#
# 认证：容器以 root 连宿主 X，用 `xhost +si:localuser:root`（一次性、可撤销），
# 不再挂 Xauthority cookie（源路径在图形会话建立前不存在，会让容器起不来）。
# =============================================================================
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
CONTAINER=${SIM_VIZ_CONTAINER:-ros1-slam-sim-viz}
XDISPLAY=${X_DISPLAY:-:1}
WORLD=${WORLD:-$ROOT/ws/src/95_simulation/lite3_gazebo_sim/worlds/m1_world_v2.world}
IMAGE=${SIM_IMAGE:-ros1-slam:noetic-sim}
MASTER=${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}

die() { echo "[sim_viz] $*" >&2; exit 2; }

in_container() { docker exec "$CONTAINER" bash -lc "$1"; }

container_path() {
    case "$1" in
        "$ROOT"/*) printf '/opt/ros1-slam/%s' "${1#"$ROOT"/}" ;;
        /opt/ros1-slam/*) printf '%s' "$1" ;;
        *) die "world 必须位于仓库内或已是容器路径：$1" ;;
    esac
}

doctor() {
    echo "=== sim-viz doctor ==="
    echo "DISPLAY                = ${XDISPLAY}"
    [ -S "/tmp/.X11-unix/X${XDISPLAY#:}" ] \
        && echo "X socket               = /tmp/.X11-unix/X${XDISPLAY#:} 存在" \
        || echo "X socket               = 缺失（宿主无 X 或 DISPLAY 不对）"
    if command -v xhost >/dev/null; then
        DISPLAY=$XDISPLAY xhost 2>/dev/null | sed 's/^/xhost                  = /'
    fi
    if command -v nvidia-smi >/dev/null; then
        nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>/dev/null \
            | sed 's/^/GPU                    = /'
    else
        echo "GPU                    = nvidia-smi 缺失"
    fi
    docker info --format '{{json .Runtimes}}' 2>/dev/null \
        | python3 -c "import sys,json; print('docker runtimes        = ' + ','.join(sorted(json.load(sys.stdin))))" 2>/dev/null || true
    docker image inspect "$IMAGE" >/dev/null 2>&1 \
        && echo "image                  = $IMAGE 在位" \
        || echo "image                  = $IMAGE 缺失（需 docker compose --profile viz build sim-viz）"
    if docker inspect "$CONTAINER" >/dev/null 2>&1; then
        echo "container              = $(docker inspect -f '{{.State.Status}}' "$CONTAINER")"
    else
        echo "container              = 未创建"
    fi
    if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" = "true" ]; then
        in_container 'export DISPLAY='"$XDISPLAY"'; if command -v glxinfo >/dev/null; then glxinfo -B 2>/dev/null | grep -m1 "OpenGL renderer"; else echo "glxinfo 缺失"; fi'
    fi
}

start() {
    local world=${1:-$WORLD}
    [ -f "$world" ] || die "world 不存在：$world"
    local cworld
    cworld=$(container_path "$world")
    command -v xhost >/dev/null || die "缺少 xhost"
    DISPLAY=$XDISPLAY xhost +si:localuser:root >/dev/null
    ( cd "$ROOT/docker" && docker compose --profile viz up -d sim-viz >/dev/null )
    sleep 2
    in_container 'export GAZEBO_MASTER_URI='"$MASTER"'; export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/opt/ros1-slam/ws/src/95_simulation:/opt/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim/worlds; export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:/opt/ros1-slam/ws/src/95_simulation; pgrep -x gzserver >/dev/null || (setsid gzserver '"$cworld"' >/tmp/gzserver.log 2>&1 &)'
    sleep 5
    in_container 'export DISPLAY='"$XDISPLAY"'; export GAZEBO_MASTER_URI='"$MASTER"'; export LIBGL_ALWAYS_SOFTWARE=0; pgrep -x gzclient >/dev/null || (setsid gzclient >/tmp/gzclient.log 2>&1 &)'
    sleep 3
    status
}

status() {
    echo "=== sim-viz status ==="
    docker inspect -f 'container = {{.State.Status}} (exit {{.State.ExitCode}})' "$CONTAINER" 2>/dev/null || echo "container = 未创建"
    docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true || return 0
    in_container 'echo "gzserver  = $(pgrep -x gzserver | tr "\n" " ")"'
    in_container 'echo "gzclient  = $(pgrep -x gzclient | tr "\n" " ")"'
    local windows
    windows=$(DISPLAY=$XDISPLAY xwininfo -root -tree 2>/dev/null | grep -i gazebo | head -2 || true)
    if [ -n "$windows" ]; then
        printf '%s\n' "$windows"
    else
        echo "窗口      = 未找到 gazebo 窗口"
    fi
}

stop() {
    if docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true; then
        for name in gzclient gzserver; do
            in_container "for pid in \$(pgrep -x $name); do kill \$pid; done" || true
        done
        sleep 2
        ( cd "$ROOT/docker" && docker compose --profile viz stop sim-viz >/dev/null )
    fi
    DISPLAY=$XDISPLAY xhost -si:localuser:root >/dev/null 2>&1 || true
    echo "[sim_viz] stopped; xhost 已撤销 root"
}

case "${1:-doctor}" in
    doctor) doctor ;;
    start) shift; start "${1:-$WORLD}" ;;
    status) status ;;
    stop) stop ;;
    *) echo "usage: $0 {doctor|start [world]|status|stop}" >&2; exit 2 ;;
esac
