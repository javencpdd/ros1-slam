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
SIM_PKG=$WS/ws/src/95_simulation/lite3_gazebo_sim
MODE="${1:-smoke}"
STAGE="${2:-}"

# ---- 0. 环境 ----
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
[ -f "$WS/ws/devel/setup.bash" ] && source "$WS/ws/devel/setup.bash"

export GAZEBO_PLUGIN_PATH=$WS/ws/devel/lib:/opt/ros/noetic/lib
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:$WS/ws/src/95_simulation:$SIM_PKG/worlds
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:$WS/ws/src/95_simulation
export LIBGL_ALWAYS_SOFTWARE=1

# Xvfb 兜底：entrypoint 没起（USE_XVFB=0 直跑时）就在这里起
if [ -z "${DISPLAY:-}" ] && command -v Xvfb >/dev/null 2>&1; then
    rm -f /tmp/.X99-lock
    Xvfb :99 -screen 0 1280x1024x24 -nolisten tcp >/tmp/xvfb.log 2>&1 &
    export DISPLAY=:99
    sleep 1
    echo "[sim] 已补起 Xvfb :99"
fi

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
