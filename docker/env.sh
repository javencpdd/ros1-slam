#!/usr/bin/env bash
# =============================================================================
# 容器内"完整 ROS1 沙箱环境" —— 替代直接 source scripts/env.sh 的正确姿势
#
# 用法（进入容器后）：
#     source /opt/ros1-slam/docker/env.sh          # 走 compose 里给的默认口径
#     ROS1_MASTER_URI=http://192.168.1.103:11312 ROS1_IP=192.168.1.103 \
#         source /opt/ros1-slam/docker/env.sh      # 真机/直连 LiDAR
#
# 为什么需要它：
#   项目自带 scripts/env.sh 里 ROS_MASTER_URI / ROS_IP / 路径全写死成真机值
#   （192.168.1.103、/home/ros1-slam/...），在容器里要么指错要么指空。
#   本脚本：先吃原 env.sh（保留它的 devel 守卫、foxy 清理、PYTHONNOUSERSITE），
#   再用容器环境变量把网络口径覆写回来；路径部分由镜像里的家目录软链接兜住。
# =============================================================================

ROS1_MASTER_URI_DEFAULT="${ROS1_MASTER_URI:-http://127.0.0.1:11311}"
ROS1_IP_VALUE="${ROS1_IP:-}"

# ---- 1. 项目原环境入口（含 devel 守卫 + foxy 清理）----
# shellcheck disable=SC1091
if ! source /home/ros1-slam/scripts/env.sh 2>/dev/null; then
    echo "[env.sh] 提示：scripts/env.sh 返回非 0 ——多半是 ws/devel 未编译，先跑 build.sh"
fi

# ---- 2. 覆写网络口径（必须在 source 原 env.sh 之后）----
export ROS_MASTER_URI="${ROS1_MASTER_URI_DEFAULT}"
if [ -n "${ROS1_IP_VALUE}" ]; then
    export ROS_IP="${ROS1_IP_VALUE}"
    unset ROS_HOSTNAME
else
    # ★ 原 env.sh 已经把 ROS_IP 写死成 192.168.1.103，这里必须清掉，
    #   否则容器里会拿着宿主机的 IP 去注册，外部连不进来
    unset ROS_IP
    export ROS_HOSTNAME="${ROS_HOSTNAME:-$(hostname)}"
fi

# ---- 3. 其余运行时量 ----
export ROS_HOME="${ROS_HOME:-/opt/ros1-slam/.ros}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/opt/ros1-slam/logs}"
export PYTHONNOUSERSITE=1
export ROS_DISTRO=noetic
export ROS_VERSION=1
export ROS_PYTHON_VERSION=3

mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}" /opt/ros1-slam/data/maps

echo "[env.sh] MASTER=${ROS_MASTER_URI}  SELF=${ROS_IP:-${ROS_HOSTNAME}}  LOG=${ROS_LOG_DIR}"
