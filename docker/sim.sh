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

# ---- M1 导航底座（唯一入口；验收脚本必须调它，不得各自重写） ----
# 为什么收敛到 sim.sh：此前每个验收脚本各写一套 "起 master / 起 gazebo / 等话题 / 起 HDL"，
# 逻辑重复且各自踩坑（Xvfb 死、包路径重复、话题存在但无数据）。底座只应有一处实现。
BASE_PIDFILE=/tmp/sim_base.pids
base_up() {
    want_hdl="${1:-hdl}"
    . "$(dirname "${BASH_SOURCE[0]}")/xvfb_guard.sh"
    xvfb_guard_ensure "${DISPLAY:-:99}" || { echo "[base] 显示服务不可用"; return 1; }
    export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11313}"
    export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}"
    : > "$BASE_PIDFILE"
    if ! timeout 4 rostopic list >/dev/null 2>&1; then
        echo "[base] 起 master"
        roscore -p "${ROS_MASTER_URI##*:}" >/tmp/base_roscore.log 2>&1 &
        echo $! >> "$BASE_PIDFILE"
        for _ in $(seq 1 20); do sleep 2; timeout 3 rostopic list >/dev/null 2>&1 && break; done
    fi
    timeout 5 rostopic list >/dev/null 2>&1 || { echo "[base] master 无应答"; return 1; }
    echo "[base] 起 m1_gazebo（世界 + 机器人 + 传感器）"
    roslaunch lite3_gazebo_sim m1_gazebo.launch >/tmp/base_gazebo.log 2>&1 &
    echo $! >> "$BASE_PIDFILE"
    _wait_topic /clock 60 || { echo "[base] /clock 未出（查 /tmp/base_gazebo.log）"; return 1; }
    _wait_topic /rslidar_points 90 || { echo "[base] /rslidar_points 未出"; return 1; }
    _wait_topic /livox/imu 60 || { echo "[base] /livox/imu 未出（IMU 传感器未创建）"; return 1; }
    # 服务面就绪：图里的 carrier3d 要 /gazebo/set_model_state，它由 gazebo_ros_api_plugin 提供。
    # 只等话题不等服务 ⇒ carrier3d 起手 20s 超时后自杀、整图起不来（实测：/cmd_vel、/v8/local/pose 全无）。
    for _ in $(seq 1 30); do
        rosservice list 2>/dev/null | grep -q "/gazebo/set_model_state" && break
        sleep 1
    done
    rosservice list 2>/dev/null | grep -q "/gazebo/set_model_state" \
        || { echo "[base] /gazebo/set_model_state 未就绪（carrier3d 会超时自杀）"; return 1; }
    if [ "$want_hdl" = "hdl" ]; then
        echo "[base] 起 HDL（map=m1_world_synth_v1，外参取 URDF 仿真几何）"
        roslaunch "$WS/ws/.profiles/x86_sim/src/lite3_localization/launch/hdl_localization.launch" \
            tf_x:=0 tf_y:=0 tf_z:=0.34 tf_pitch:=0.2618 \
            map_name:=m1_world_synth_v1 imu_topic:=/livox/imu >/tmp/base_hdl.log 2>&1 &
        echo $! >> "$BASE_PIDFILE"
    fi
    echo "[base] BASE READY（pid 记录在 $BASE_PIDFILE）"
}

base_down() {
    if [ -f "$BASE_PIDFILE" ]; then
        while read -r pid; do [ -n "$pid" ] && kill "$pid" 2>/dev/null || true; done < "$BASE_PIDFILE"
        rm -f "$BASE_PIDFILE"
    fi
    echo "[base] 已停（只停本入口起的进程）"
}

_wait_topic() {  # $1=话题 $2=超时秒；用 rostopic list 判存在性（type 对无发布者会返回空）
    local t="$1" n="$2" i=0
    while [ "$i" -lt "$n" ]; do
        rostopic list 2>/dev/null | grep -qx "$t" && return 0
        sleep 1; i=$((i+1))
    done
    return 1
}

case "$MODE" in
    probe)  probe ;;
    base)   shift; base_up "${1:-hdl}" ;;
    down)   base_down ;;
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
        echo "用法: bash $0 {probe|base [hdl|no-hdl]|down|gazebo|smoke [stageA|stageB|stageC|all]}"
        exit 2
        ;;
esac
