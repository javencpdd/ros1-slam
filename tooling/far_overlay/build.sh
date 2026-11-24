#!/usr/bin/env bash
# FAR 独立构建：只建 far_planner + visibility_graph_msg，不进 ws 主闭包。
#
# ★必须 --extend★：不加 --extend 时本 overlay 生成的 setup.sh 会把 ROS_PACKAGE_PATH
# 直接覆盖成「本 overlay + /opt/ros」，把 ws 的包全挤掉 —— 实测表现为
# `rospack find v8_platform` 报 not found、roslaunch 说 far_stack.launch 不是 launch 文件。
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DEVEL="${WS_DEVEL:-/opt/ros1-slam/ws/devel}"
source /opt/ros/noetic/setup.bash
[ -f "$WS_DEVEL/setup.bash" ] && source "$WS_DEVEL/setup.bash"
cd "$HERE"
catkin_make --extend "$WS_DEVEL" -j"${JOBS:-4}" "$@"
echo "[far] built: $HERE/devel"
