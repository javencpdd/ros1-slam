#!/usr/bin/env bash
# =============================================================================
# catkin_build.sh —— 镜像构建期的工作区编译（三步，顺序不能换）
# 由 Dockerfile builder 阶段调用；也可在容器内手工跑做全量重编。
#
# 为什么必须是三步（2026-10-17 实测，反复踩过）：
#   livox_laser_simulation 是第三方副本，按项目红线"一字不改"，而它的
#   CMakeLists.txt 里缺
#       add_dependencies(livox_laser_simulation ${${PROJECT_NAME}_EXPORTED_TARGETS})
#   ⇒ 库目标与自己的 *_generate_messages_cpp 目标之间没有依赖关系，
#     make 会先编译 .cpp、后生成消息头，于是必挂：
#       livox_points_plugin.cpp:18: fatal error:
#       livox_laser_simulation/CustomMsg.h: No such file or directory
#   ★ 这与 -j 无关（-j1 一样挂），纯粹是目标排序，不是并行竞态。
#   解法：① configure → ② 预生成所有消息头 → ③ 正式编译。
#
#   ① 为什么直接调 cmake 而不是跑一遍 catkin_make：
#     catkin_make = cmake + make。① 的目的只是把 build/ 配出来（让 ② 能枚举到
#     *_generate_messages_cpp 目标），不需要跑 make。早先版本真的跑了一遍
#     catkin_make -j1，白跑 577 秒后才交给 ③ —— 那条命令逐字抄自 catkin_make
#     自己的日志输出，行为一致。
#
# 环境变量：ROS_DISTRO(默认 noetic) / WS_DIR(默认 /opt/ros1-slam/ws) / JOBS(默认 4)
# 注：JOBS 只管 ③。①只跑 cmake、②只生成消息头，都是轻量步；
#     ③ 之前所有消息头已在 ② 生成完毕，所以并行安全。
# =============================================================================
set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
WS_DIR="${WS_DIR:-/opt/ros1-slam/ws}"
JOBS="${JOBS:-4}"

# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# -DROS_EDITION=ROS1          : livox_ros_driver2 的 ROS1/ROS2 分支开关，必传
# -DPYTHON_EXECUTABLE         : noetic 用 python3
# -DCATKIN_WHITELIST_PACKAGES=: 必须显式传空 —— catkin_make --pkg 会把它写进
#                               CMakeCache，不清之后每次都只编那几个包（catkin 已知坑）
CMAKE_ARGS="-DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS1 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_WHITELIST_PACKAGES="

# ---- ① configure：只跑 cmake，不跑 make ----
echo "[build] ① configure（只跑 cmake，不跑 make —— 省掉一遍白跑的全量编译）"
mkdir -p "${WS_DIR}/build"
# shellcheck disable=SC2086
cd "${WS_DIR}/build" && cmake "${WS_DIR}/src" ${CMAKE_ARGS} \
    -DCATKIN_DEVEL_PREFIX="${WS_DIR}/devel" \
    -DCMAKE_INSTALL_PREFIX="${WS_DIR}/install" \
    -G "Unix Makefiles"

# ---- ② 预生成所有消息头 ----
# ★ 目标名只能从 `make help` 枚举。别去 build/CMakeFiles 目录名里猜 ——
#   实测那里一个 generate_messages 目录都没有（猜错会让 grep 返回 1，
#   在 set -eo pipefail 下直接把脚本干掉，表现为"① 成功后莫名 exit 1"）。
# ★ `|| TGTS=""` 不能省：grep 无匹配时返回 1，同样会触发 set -e。
TGTS=$(make help 2>/dev/null | grep -oE '\.\.\. [A-Za-z0-9_]+_generate_messages_cpp' | awk '{print $2}' | sort -u | tr '\n' ' ') || TGTS=""
echo "[build] ② 预生成消息头，共 $(echo ${TGTS} | wc -w) 个目标"
if [ -z "${TGTS}" ]; then
    echo "[build] ② 警告：没枚举到消息生成目标 → ③ 退化为串行编译（JOBS=1）"
    JOBS=1
else
    # shellcheck disable=SC2086
    make -j1 ${TGTS} >/dev/null || echo "[build] ② 有目标未建成（继续，由 ③ 暴露真实错误）"
fi

# ---- ③ 正式编译（本步的成败才是最终结论）----
# ★ ③ 可以放心并行（JOBS 默认 4）：② 已经把**所有**包的
#   *_generate_messages_cpp 目标都 make 过了，消息头全部就位，
#   当初 -j4 必挂的竞态根源（库目标抢跑在自己的消息生成之前）已经不存在。
#   早先版本在这里也用 -j1，白白多花 380 秒。
cd "${WS_DIR}"
echo "[build] ③ 正式编译 -j${JOBS}"
# shellcheck disable=SC2086
catkin_make ${CMAKE_ARGS} -j"${JOBS}"

if [ ! -f "${WS_DIR}/devel/setup.bash" ]; then
    echo "[FAIL] catkin_make 失败，往上翻日志定位具体包"
    exit 1
fi
echo "[build] 完成"
