#!/usr/bin/env bash
# =============================================================================
# 容器内跑仿真的统一入口（Gazebo headless + 项目冒烟判据）
#
#   bash /opt/ros1-slam/docker/sim.sh smoke            # m2_smoke.sh（缺省 stageA/B/C 全跑）
#   bash /opt/ros1-slam/docker/sim.sh smoke stageA     # 只跑 Stage A（推荐先跑这个）
#   bash /opt/ros1-slam/docker/sim.sh gazebo           # 只起 Gazebo 场地，前台挂住看日志
#   bash /opt/ros1-slam/docker/sim.sh probe            # 环境自检：GL/Xvfb/插件/端口
#
# 设计要点：
#   · 不依赖外部 roscore —— m2_smoke.sh 自己 export ROS_MASTER_URI=127.0.0.1:11313，
#     roslaunch 会在本容器自起 master；GAZEBO_MASTER_URI=127.0.0.1:11345。
#   · gzserver 需要 GL 上下文：本脚本确保 DISPLAY 指向 Xvfb（entrypoint 已起 :99）。
#   · 用的是 CPU 光线投射（xacro 里 sensor type="ray"，非 gpu_ray）⇒ 不需要 NVIDIA GPU。
# =============================================================================
set -eo pipefail

WS=/opt/ros1-slam
SIM_PKG=$WS/sim_ws/src/lite3_gazebo_sim
MODE="${1:-smoke}"
STAGE="${2:-}"

# ---- 0. 环境 ----
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
[ -f "$WS/ws/devel/setup.bash" ] && source "$WS/ws/devel/setup.bash"
[ -f "$WS/sim_ws/devel/setup.bash" ] && source "$WS/sim_ws/devel/setup.bash"

# ★ 2026-11-16 修（顺序有讲究，别把 ws 放回第一位）：
#   11-14 物理分仓后，sim 的插件编在 sim_ws/devel/lib，而 ws/devel/lib 里还留着一份
#   11-02 的旧产物 liblivox_laser_simulation.so。两者符号集完全相同、源码同一份，
#   **唯一差别是编译期烧进 .so 的 __FILE__ 路径**：旧件指向 ws/src/95_simulation（分仓后
#   已不存在）⇒ 插件 Load() 里读 CSV 失败后直接 return，话题照 advertise 但零数据。
#   实测＝旧件优先时 /livox/lidar 无消息，sim_ws 优先时 10.05Hz。
#   ⇒ sim_ws 必须排在前面，否则 m1 系列世界的仿真雷达是空转的。
export GAZEBO_PLUGIN_PATH=$WS/sim_ws/devel/lib:$WS/ws/devel/lib:/opt/ros/noetic/lib
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:$WS/sim_ws/src:$SIM_PKG/worlds
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:$WS/sim_ws/src
export LIBGL_ALWAYS_SOFTWARE=1

# Xvfb 兜底（2026-11-25 改为**存活检测**，不再只看 DISPLAY 是否为空）
# 事故：镜像 DISPLAY=:99 但 Xvfb 已死、锁文件残留 ⇒ 旧逻辑跳过兜底 ⇒ gzserver 无显示
#       ⇒ 模型 spawn 失败、传感器不创建 ⇒ /livox/imu 连话题都不出现（冒烟"时好时坏"的真因）。
# 判据与三种场景（正常/已死/残留锁）由 docker/xvfb_guard.sh 实现，并有
# scripts/verify/test_xvfb_guard.sh 的三连＋反控守着。
. "$(dirname "${BASH_SOURCE[0]}")/xvfb_guard.sh"
xvfb_guard_ensure "${DISPLAY:-:99}" || echo "[sim] 警告：显示服务不可用，Gazebo 可能起不来" >&2

probe() {
    echo "=== 仿真环境自检 ==="
    echo "DISPLAY            = ${DISPLAY:-<未设置>}"
    echo "LIBGL_ALWAYS_SOFTWARE = ${LIBGL_ALWAYS_SOFTWARE}"
    command -v gzserver >/dev/null && echo "gzserver           = $(command -v gzserver) ($(gzserver --version 2>/dev/null | head -1))" || echo "gzserver           = 缺失（请用 PROFILE=sim 构建）"
    command -v Xvfb    >/dev/null && echo "Xvfb               = ok" || echo "Xvfb               = 缺失"
    echo "glxinfo(软渲染)    = $(glxinfo -B 2>/dev/null | grep -m1 'OpenGL renderer' || echo 'n/a')"
    echo "--- 模型 ---"
    ls -d /usr/share/gazebo-11/models/sun /usr/share/gazebo-11/models/ground_plane 2>/dev/null || echo "sun/ground_plane 模型缺失（离线会去联网拉，必挂）"
    echo "--- 插件 .so ---"
    ls -1 "${WS}/ws/devel/lib/liblite3_planar_drive.so" 2>/dev/null || echo "liblite3_planar_drive.so 缺失（未编译？）"
    ls -1 "${WS}/ws/devel/lib/liblivox_laser_simulation.so" 2>/dev/null || echo "liblivox_laser_simulation.so 缺失（未编译？）"
    ls -1 "${WS}/ws/devel/lib/livox_ros_driver2/livox_ros_driver2_node" 2>/dev/null || echo "livox_ros_driver2_node 缺失（未编译？）"
    echo "--- 端口（应为空）---"
    (ss -lnt 2>/dev/null || netstat -lnt 2>/dev/null) | grep -E '1131[123]|11345' || echo "11311/11312/11313/11345 均无监听 ✔"
}

case "$MODE" in
    probe)  probe ;;
    clean)
        # 对齐 m2_smoke.sh 的退净判据：把本域（11313/11345）的进程与端口清干净。
        # 反复起停后残留的重复 gzserver/rosmaster 会让旁观者（rviz/gzclient）
        # 连到另一个图上，表现为"ROS Time 0.00 / No TF data / 黑屏"。
        echo "[sim] 清理仿真域进程"
        pkill -f gzclient     2>/dev/null || true
        pkill -f gzserver     2>/dev/null || true
        pkill -f rviz         2>/dev/null || true
        pkill -f "roslaunch.*(m2_|mode_sim_)" 2>/dev/null || true
        pkill -f "rosmaster"  2>/dev/null || true
        pkill -f run_mapping  2>/dev/null || true
        pkill -f sim_adapter  2>/dev/null || true
        sleep 3
        echo "[sim] 残留进程："
        ps -eo pid,stat,comm,args 2>/dev/null | grep -E "gzserver|gzclient|rosmaster|roslaunch|run_mapping|rviz" | grep -v grep | head -10 || echo "  （无）"
        echo "[sim] 端口："
        (ss -lnt 2>/dev/null || netstat -lnt 2>/dev/null) | grep -E "1131[123]|11345" || echo "  11311/11312/11313/11345 均无监听 ✔"
        ;;
    gazebo)
        export ROS_MASTER_URI=http://127.0.0.1:11313
        export GAZEBO_MASTER_URI=http://127.0.0.1:11345
        exec roslaunch lite3_gazebo_sim m2_gazebo.launch gui:=false
        ;;
    smoke)
        # ★ 不要在这里 export ROS_MASTER_URI —— m2_smoke.sh:29 自己会设成 127.0.0.1:11313，
        #   在外面先设反而可能与它的口径打架（它设的才是判据域）。
        if [ -n "$STAGE" ]; then
            exec bash "$SIM_PKG/verify/m2_smoke.sh" "$STAGE"
        else
            exec bash "$SIM_PKG/verify/m2_smoke.sh" all
        fi
        ;;
    *)
        echo "用法: bash $0 {probe|gazebo|smoke [stageA|stageB|stageC|all]}"
        exit 2
        ;;
esac
