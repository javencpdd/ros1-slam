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
# ★2026-11-25 修（FAR 后端一直跑不起来的真因）★
# `roslaunch` 解析 `type="far_planner"` 时只在**包源码目录**里找可执行，而 overlay 是 devel 空间构建，
# 二进制落在 `devel/lib/far_planner/far_planner` ⇒ 启动时报
#   `ERROR: cannot launch node of type [far_planner/far_planner]:
#    Cannot locate node of type [far_planner] in package [far_planner]`
# ⇒ FAR 节点从未启动，BT 只能等 30 s 超时（日志里连续 `global path timeout`），FAR 组合一直 INVALID。
# 这里在构建后把 devel 里的可执行补成包内软链（幂等，不覆盖真实文件）。
for pkgdir in "$HERE"/devel/lib/*/; do
  [ -d "$pkgdir" ] || continue
  pkg=$(basename "$pkgdir")
  [ -d "$HERE/src/$pkg" ] || continue
  for exe in "$pkgdir"*; do
    [ -f "$exe" ] && [ -x "$exe" ] || continue
    name=$(basename "$exe")
    case "$name" in *.so|*.a|*.pyc) continue ;; esac
    if [ ! -e "$HERE/src/$pkg/$name" ]; then
      ln -s "$exe" "$HERE/src/$pkg/$name"
      echo "[far] linked src/$pkg/$name -> $exe"
    fi
  done
done

echo "[far] built: $HERE/devel"
