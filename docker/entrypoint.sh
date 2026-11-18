#!/usr/bin/env bash
# =============================================================================
# ros1-slam 容器入口
# 职责：装 ROS 环境 → 装 devel → 落网络口径 → exec "$@"；不写源码空间。
# 网络口径由环境变量决定，不写死 IP（原 scripts/env.sh 硬编码 192.168.1.103）
# =============================================================================
# ★ 不要用 set -u：ROS 的 setup.bash / catkin devel setup.bash 里有未定义变量引用，
#   开了 -u 会在 source 阶段直接退出。
set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
WS_ROOT="${WS_ROOT:-/opt/ros1-slam}"
WS_DIR="${WS_DIR:-${WS_ROOT}/ws}"

# ---- 1. ROS 基础环境 ----
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# ---- 2. devel space（构建视图与顶层 CMake 只由 target helper 维护）----
# 未编译时不要静默失败：明确提示先编（原 scripts/env.sh 的 devel 守卫精神）
if [ -f "${WS_DIR}/devel/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "${WS_DIR}/devel/setup.bash"
else
    echo "[entrypoint] 警告：${WS_DIR}/devel/setup.bash 不存在 —— 尚未编译。"
    echo "[entrypoint]       进入容器后执行：bash /opt/ros1-slam/docker/build.sh"
fi

# ---- 2.5 清掉分仓前的旧仿真插件（2026-11-16）----
# 11-14 物理分仓把仿真源码从 ws/src 搬到 sim_ws/src，但 ws/devel/lib 里留着 11-02 编的
# liblivox_laser_simulation.so。它与 sim_ws 那份符号集完全相同、源码同一份，**唯一差别是
# 编译期烧进 .so 的 __FILE__ 路径**：旧件指向 ws/src/95_simulation（已不存在）⇒ 插件
# Load() 读 CSV 失败就 return，话题照 advertise 但一条数据不发（静默降级，查起来最费时）。
# 路径优先级已在 sim.sh/viz.sh/Dockerfile 改成 sim_ws 优先；这里再做一次「点名删旧件」，
# 让两件事互为兜底。只删这一枚具名文件，不动 devel 卷里其它产物。
_LEGACY_PLUGIN="${WS_DIR}/devel/lib/liblivox_laser_simulation.so"
if [ -f "${_LEGACY_PLUGIN}" ] && [ -f "${WS_ROOT}/sim_ws/devel/lib/liblivox_laser_simulation.so" ]; then
    # 2026-11-18（M10）：加 || true。容器可能以非 root（HOST_UID）运行，若该产物
    # 属主是 root（历史构建留下的），rm 会失败；set -e 下会把整个 entrypoint 打断，
    # 表现成"容器起来就退出"。删不掉只说明它是旧的 root 产物，不该阻断启动。
    rm -f "${_LEGACY_PLUGIN}" 2>/dev/null && echo "[entrypoint] 已清掉分仓前旧插件 ${_LEGACY_PLUGIN}（sim_ws 版本在位，不受影响）" || true
fi

# ---- 3. 目录与运行时环境变量 ----
mkdir -p "${ROS_HOME:-${WS_ROOT}/.ros}" \
         "${ROS_LOG_DIR:-${WS_ROOT}/logs}" \
         "${WS_ROOT}/data/maps" \
         "${WS_ROOT}/data/waypoints"

export ROS_HOME="${ROS_HOME:-${WS_ROOT}/.ros}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${WS_ROOT}/logs}"
export PYTHONNOUSERSITE=1
export ROS_PYTHON_VERSION=3

# ---- 4.5 无头渲染（仿真用）----
# gzserver 在没有 X 的容器里需要一块虚拟屏；USE_XVFB=1 时起 Xvfb :99
if [ "${USE_XVFB:-0}" = "1" ] && command -v Xvfb >/dev/null 2>&1; then
    rm -f /tmp/.X99-lock
    Xvfb :99 -screen 0 1280x1024x24 -nolisten tcp >/tmp/xvfb.log 2>&1 &
    export DISPLAY=:99
    sleep 1
    echo "[entrypoint] Xvfb :99 已起（DISPLAY=${DISPLAY}）"
fi

# ---- 5. ROS 网络口径 ----
# 容器内默认自闭环（127.0.0.1/11311）；开发多容器网络通过变量覆盖。
export ROS_MASTER_URI="${ROS1_MASTER_URI:-http://127.0.0.1:11311}"
if [ -n "${ROS1_IP:-}" ]; then
    export ROS_IP="${ROS1_IP}"
    unset ROS_HOSTNAME
else
    # 未给 IP 时用容器 hostname（compose 网络中可被 DNS 解析）
    export ROS_HOSTNAME="${ROS_HOSTNAME:-$(hostname)}"
fi

echo "[entrypoint] ROS_DISTRO=${ROS_DISTRO}  MASTER=${ROS_MASTER_URI}  IP=${ROS_IP:-${ROS_HOSTNAME}}"

# ---- 6. 交棒 ----
exec "$@"
