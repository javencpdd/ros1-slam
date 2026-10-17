#!/usr/bin/env bash
# =============================================================================
# 容器内重新编译 catkin 工作区（改完 ws/src 后跑这个）
#
#   bash /opt/ros1-slam/docker/build.sh            # 增量编译（全量范围）
#   bash /opt/ros1-slam/docker/build.sh --clean    # 删 build/devel 后全量重编
#   JOBS=2 bash /opt/ros1-slam/docker/build.sh     # Jetson 内存紧张时降并发
#
# ★ 为什么没有 --slim：
#   瘦身只在镜像构建时做（Dockerfile 的 SLIM=1，靠 CATKIN_IGNORE 实现，
#   且只作用于 builder 阶段 COPY 进来的副本）。这里不做，是因为 ws/src 是
#   bind-mount 的宿主机目录，在容器里打 CATKIN_IGNORE 会写回你的工作区，
#   污染 git 状态、也可能撞上项目的隔离纪律。
#   增量编译本身很快（只重编改动的包），不需要瘦身。
# =============================================================================
# ★ 同 entrypoint：不要用 set -u，ROS 的 setup.bash 有未定义变量引用
set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
WS_DIR="${WS_DIR:-/opt/ros1-slam/ws}"
# ★ 默认串行：livox_laser_simulation（第三方副本，不能改）没有把自己的库目标
#   add_dependencies 到 *_generate_messages_cpp，并行编译会抢跑，
#   报 fatal error: livox_laser_simulation/CustomMsg.h: No such file or directory。
#   增量构建时消息头已在 devel 里，不会再竞态，可以 JOBS=2~4 提速。
JOBS="${JOBS:-4}"

CLEAN=0
while [ $# -gt 0 ]; do
    case "$1" in
        --clean) CLEAN=1; shift ;;
        *) echo "[build] 未知参数: $1（只认 --clean）"; exit 2 ;;
    esac
done

# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ "${CLEAN}" = "1" ]; then
    echo "[build] 清理 build/ devel/"
    rm -rf "${WS_DIR}/build" "${WS_DIR}/devel"
fi

if [ ! -f "${WS_DIR}/src/CMakeLists.txt" ]; then
    echo "[build] catkin_init_workspace"
    ( cd "${WS_DIR}/src" && catkin_init_workspace )
fi

# livox_ros_driver2 上游只有 package_ROS1.xml / package_ROS2.xml，没有 package.xml
# ⇒ catkin/rospack 看不见它。这里幂等补上（与 Dockerfile builder 阶段同一处理）。
if [ ! -f "${WS_DIR}/src/00_drivers/livox_ros_driver2/package.xml" ] \
   && [ -f "${WS_DIR}/src/00_drivers/livox_ros_driver2/package_ROS1.xml" ]; then
    echo "[build] livox_ros_driver2: package_ROS1.xml -> package.xml"
    cp "${WS_DIR}/src/00_drivers/livox_ros_driver2/package_ROS1.xml" \
       "${WS_DIR}/src/00_drivers/livox_ros_driver2/package.xml"
fi

cd "${WS_DIR}"
echo "[build] catkin_make -j${JOBS}（ROS_EDITION=ROS1）"
# ★ -DCATKIN_WHITELIST_PACKAGES= 必须显式传空：catkin_make --pkg 会把它写进
#   CMakeCache，不清的话之后每次都只编那几个包（catkin 已知坑）。
catkin_make \
    -DCMAKE_BUILD_TYPE=Release \
    -DROS_EDITION=ROS1 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DCATKIN_WHITELIST_PACKAGES= \
    -j"${JOBS}"

# shellcheck disable=SC1091
source "${WS_DIR}/devel/setup.bash"
echo "[build] 完成。仿真关键件："
for p in livox_ros_driver2 livox_laser_simulation lite3_gazebo_sim faster_lio lite3_slam lite3_drivers; do
    if rospack find "$p" >/dev/null 2>&1; then
        echo "  [OK]   $p"
    else
        echo "  [MISS] $p  ← 仿真关键件，必须修"
    fi
done
echo "[build] 产物："
ls -1 "${WS_DIR}/devel/lib/liblite3_planar_drive.so" \
      "${WS_DIR}/devel/lib/liblivox_laser_simulation.so" \
      "${WS_DIR}/devel/lib/livox_ros_driver2/livox_ros_driver2_node" 2>/dev/null \
    || echo "  [MISS] 插件/节点缺失（GAZEBO_PLUGIN_PATH=${WS_DIR}/devel/lib）"
