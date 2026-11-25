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

# ---- 底座（唯一入口）----
# ★2026-11-25 收敛★：不再在本文件里自写"起 master / 起 gazebo / 起 HDL"，
# 一律复用历史已验证脚本 `scripts/verify/asan_hdl_repro.sh`（它自己带
# `roslaunch m1_gazebo.launch map_to_odom:=false`、nodelet 装载 HDL、relay 出 /hdl_odom，并有闸门）。
# 本文件只负责：显示守卫 → 孤儿清理 → 调它 → **按活数据**判定就绪。
BASE_PIDFILE=/tmp/sim_base.pids

_wait_hz() {  # $1=话题 $2=超时秒；判"有没有活数据"，不看话题是否注册
    # ★不能写 `timeout 8 rostopic hz | grep -q`★：本文件是 `set -eo pipefail`，
    # grep -q 提前退出会给 rostopic 发 SIGPIPE，整条管道判假 —— 恒假。
    # 这与 asan_hdl_repro.sh:33-37 记录的是同一个坑，照它的写法先取变量再判。
    local t="$1" n="$2" i=0 out
    while [ "$i" -lt "$n" ]; do
        out=$(timeout 8 rostopic hz "$t" 2>/dev/null || true)
        printf '%s' "$out" | grep -q "average rate" && return 0
        sleep 2; i=$((i+2))
    done
    return 1
}

_pub_count() {  # 某话题的发布者数量
    rostopic info "$1" 2>/dev/null | sed -n '/Publishers/,$p' | grep -c '^ \*' || true
}

_check_gazebo_ros() {  # Gazebo 的 ROS 面：节点 + 服务（只看进程会漏判"孤儿 gzserver"）
    rosnode list 2>/dev/null | grep -qx "/gazebo" \
        || { echo "[base] FAIL: master 上没有 /gazebo 节点（gzserver 未注册/已脱管，见 orphan_gzserver.sh）"; return 1; }
    rosservice list 2>/dev/null | grep -q "/gazebo/set_model_state" \
        || { echo "[base] FAIL: /gazebo/set_model_state 不存在（carrier3d 会超时自杀）"; return 1; }
    return 0
}

_check_sensors() {
    _wait_hz /clock 40          || { echo "[base] FAIL: /clock 无数据"; return 1; }
    _wait_hz /rslidar_points 60 || { echo "[base] FAIL: /rslidar_points 无数据"; return 1; }
    _wait_hz /livox/imu 60      || { echo "[base] FAIL: /livox/imu 无数据"; return 1; }
    local n; n=$(_pub_count /livox/imu)
    [ "$n" = "1" ] || { echo "[base] FAIL: /livox/imu 发布者=$n（应为 1：URDF Gazebo 插件）"; return 1; }
    return 0
}

base_up() {
    local kind="${1:-m1}"
    . "$(dirname "${BASH_SOURCE[0]}")/xvfb_guard.sh"
    xvfb_guard_ensure "${DISPLAY:-:99}" || { echo "[base] 显示服务不可用"; return 1; }
    export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11313}"
    export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}"
    : > "$BASE_PIDFILE"

    echo "[base] 孤儿清理"
    bash "$WS/scripts/verify/orphan_gzserver.sh" || { echo "[base] 孤儿清理失败"; return 1; }

    case "$kind" in
      m1)
        echo "[base] 复用历史脚本 asan_hdl_repro.sh（含 map_to_odom:=false 与 HDL 装载）"
        MODE=normal bash "$WS/scripts/verify/asan_hdl_repro.sh" > /tmp/base_m1.log 2>&1
        local rc=$?
        [ "$rc" = "0" ] || { echo "[base] asan_hdl_repro rc=$rc（见 /tmp/base_m1.log 尾部）"; tail -12 /tmp/base_m1.log; return 1; }
        _check_gazebo_ros || return 1
        _check_sensors || return 1
        _wait_hz /hdl_odom 90 || { echo "[base] FAIL: /hdl_odom 无数据（HDL 未收敛，见 /tmp/base_m1.log）"; return 1; }
        echo "[base] BASE READY（m1 + HDL；/livox/imu 单发布者）"
        ;;
      m2)
        echo "[base] 起 m2 底座（m2_gazebo + m2_mapping；/Odometry 为算法位姿）"
        roslaunch lite3_gazebo_sim m2_gazebo.launch > /tmp/base_m2_gz.log 2>&1 &
        echo $! >> "$BASE_PIDFILE"
        _check_gazebo_ros || return 1
        _check_sensors || return 1
        roslaunch lite3_gazebo_sim m2_mapping.launch > /tmp/base_m2_map.log 2>&1 &
        echo $! >> "$BASE_PIDFILE"
        _wait_hz /Odometry 120 || { echo "[base] FAIL: /Odometry 无数据（见 /tmp/base_m2_map.log）"; return 1; }
        echo "[base] BASE READY（m2 + LIO；/Odometry 为算法位姿）"
        ;;
      *)
        echo "[base] 未知组合：$kind（可选 m1|m2）"; return 1 ;;
    esac
}

base_down() {
    if [ -f "$BASE_PIDFILE" ]; then
        while read -r pid; do [ -n "$pid" ] && kill "$pid" 2>/dev/null || true; done < "$BASE_PIDFILE"
        rm -f "$BASE_PIDFILE"
    fi
    echo "[base] 已停（只停本入口记录的进程）"
}

case "$MODE" in
    probe)  probe ;;
    base)   shift; base_up "${1:-m1}" ;;
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
        echo "用法: bash $0 {probe|base [m1|m2]|down|gazebo|smoke [stageA|stageB|stageC|all]}"
        exit 2
        ;;
esac
