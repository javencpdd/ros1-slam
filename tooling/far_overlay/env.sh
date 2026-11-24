#!/usr/bin/env bash
# FAR overlay 的**唯一推荐入口**（source 本文件，不要直接 source overlay 的 setup.bash）。
#
# 为什么需要它：独立 overlay 的 setup.bash 会**覆盖**三个环境变量，实测每一个都会单独致命：
#   ROS_PACKAGE_PATH   → 覆盖后 `rospack find v8_platform` 报 not found，roslaunch 说
#                        `far_stack.launch` 不是 launch 文件（实质是找不到包）。
#   CMAKE_PREFIX_PATH  → 覆盖后 `find_node` 找不到 ws 的 C++ 可执行，报
#                        `Cannot locate node of type [bt_console]`。
#   PYTHONPATH         → 覆盖后 `import livox_laser_simulation` 失败 ⇒ `sim_adapter` 直接退出
#                        ⇒ `/rslidar_points` **没有发布者** ⇒ perception 永远 blind ⇒ 全链恒零速。
#                        （这条最隐蔽：ROS 层一切正常，只是没人发点云。）
#
# 2026-11-25 实测：`catkin_make --extend` 在本容器的 catkin 版本上会把 --extend 透传给 make 而失败，
# 所以不依赖 catkin 生成的链式 setup，改成这里显式补回 + 去重（重复 prepend 会触发
# `multiple files named [x] in package [y]`）。
#
# 用法：source tooling/far_overlay/env.sh
# 可选：FAR_WS_DEVEL 覆盖主 ws 的 devel 路径。

FAR_WS_DEVEL="${FAR_WS_DEVEL:-/opt/ros1-slam/ws/devel}"
FAR_WS_SRC="${FAR_WS_SRC:-/opt/ros1-slam/ws/.profiles/x86_sim/src}"
FAR_OVERLAY_DIR="${FAR_OVERLAY_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"

source /opt/ros/noetic/setup.bash
[ -f "$FAR_WS_DEVEL/setup.bash" ] && source "$FAR_WS_DEVEL/setup.bash"
[ -f "$FAR_OVERLAY_DIR/devel/setup.bash" ] && source "$FAR_OVERLAY_DIR/devel/setup.bash"

# 前置去重：把某条路径加到冒号分隔变量的最前面，已存在则跳过
_far_prepend() {  # 变量名 路径
  local var="$1" path="$2" cur="${!1:-}"
  case ":$cur:" in
    *":$path:"*) printf -v "$var" '%s' "$cur" ;;
    *) printf -v "$var" '%s' "$path${cur:+:$cur}" ;;
  esac
  export "$var"
}

# ★先剔除冗余的 ws/src★：ws/devel 的 setup 会把 `ws/src` 也塞进 ROS_PACKAGE_PATH，而 profile src
# 是同一批包的符号链接视图 ⇒ 同一个包出现两次 ⇒ roslaunch 报
# `multiple files named [x.launch] in package [y]`（HDL 启动就是这么失败的）。
_far_strip() {  # 变量名 要剔除的路径
  local var="$1" drop="$2" out="" p IFS=:
  for p in ${!var}; do
    [ "$p" = "$drop" ] && continue
    out="${out:+$out:}$p"
  done
  printf -v "$var" '%s' "$out"
  export "$var"
}
_far_strip ROS_PACKAGE_PATH "$(dirname "$(dirname "$FAR_WS_SRC")")/../src"
_far_strip ROS_PACKAGE_PATH "/opt/ros1-slam/ws/src"

_far_prepend ROS_PACKAGE_PATH "$FAR_WS_SRC"
_far_prepend ROS_PACKAGE_PATH "$FAR_OVERLAY_DIR/src"
_far_prepend CMAKE_PREFIX_PATH "$FAR_WS_DEVEL"
_far_prepend CMAKE_PREFIX_PATH "$FAR_OVERLAY_DIR/devel"
_far_prepend PYTHONPATH "$FAR_WS_DEVEL/lib/python3/dist-packages"
_far_prepend PYTHONPATH "$FAR_OVERLAY_DIR/devel/lib/python3/dist-packages"

unset -f _far_prepend

# 自检：三处都必须真的可用，否则尽早失败而不是等到车不动再查
_far_fail() { echo "[far-env] $*" >&2; return 1; }
rospack find v8_platform >/dev/null 2>&1 || _far_fail "找不到 v8_platform（ROS_PACKAGE_PATH 未补回）"
rospack find far_planner >/dev/null 2>&1 || _far_fail "找不到 far_planner（overlay 未构建？）"
python3 -c 'import livox_laser_simulation' 2>/dev/null || _far_fail "import livox_laser_simulation 失败（PYTHONPATH 未补回，sim_adapter 会退出）"
echo "[far-env] ok: v8_platform / far_planner / livox_laser_simulation 均可解析"
