#!/usr/bin/env bash
# =============================================================================
# ros1-slam 容器入口
# 职责：装 ROS 环境 → 装 devel → 兜底 catkin 初始化 → 落网络口径 → exec "$@"
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

# ---- 2. catkin 顶层 CMakeLists 兜底 ----
# 用 -v ./ws/src 挂宿主机源码时，镜像里那份 src/CMakeLists.txt 会被覆盖掉，
# 这里在容器启动时补建（幂等，已存在则跳过；会写进宿主机 ws/src，属预期）
if [ ! -f "${WS_DIR}/src/CMakeLists.txt" ] && [ -d "${WS_DIR}/src" ]; then
    echo "[entrypoint] ws/src/CMakeLists.txt 缺失 → catkin_init_workspace"
    # 失败不能让容器起不来（比如 src 被只读挂载），只提示
    ( cd "${WS_DIR}/src" && catkin_init_workspace ) \
        || echo "[entrypoint] 警告：catkin_init_workspace 失败，请检查 ws/src 挂载是否可写"
fi

# ---- 3. devel space ----
# 未编译时不要静默失败：明确提示先编（原 scripts/env.sh 的 devel 守卫精神）
if [ -f "${WS_DIR}/devel/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "${WS_DIR}/devel/setup.bash"
else
    echo "[entrypoint] 警告：${WS_DIR}/devel/setup.bash 不存在 —— 尚未编译。"
    echo "[entrypoint]       进入容器后执行：bash /opt/ros1-slam/docker/build.sh"
fi

# ---- 4. 目录与运行时环境变量 ----
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
# 容器内默认自闭环（127.0.0.1/11311）；真机/多容器时用 ROS1_MASTER_URI / ROS1_IP 覆盖
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
