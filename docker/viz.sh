#!/usr/bin/env bash
# =============================================================================
# viz.sh —— 仿真过程可视化入口（在容器内跑）
#
#   bash /opt/ros1-slam/docker/viz.sh gzclient     # Gazebo 3D 客户端：看世界/伪狗/雷达
#   bash /opt/ros1-slam/docker/viz.sh rviz-map     # rviz 看建图过程（点云+里程计）
#   bash /opt/ros1-slam/docker/viz.sh rviz-nav     # rviz 看导航（代价图+路径）
#   bash /opt/ros1-slam/docker/viz.sh check        # 自检：X / GL / 显示是否就绪
#
# 三种可视化路径，按场景选：
#   A. 宿主机 X 直通（本脚本默认）—— 宿主机有 X + GPU 时最快，用真 GPU 渲染。
#      配合 docker compose --profile viz up -d sim-viz 使用。
#   B. Foxglove 网页端 —— 不需要 X，任意机器浏览器打开 https://foxglove.dev 连
#      rosbridge(9090)。项目自带 lite3_bringup/mapping_viz.launch 就是为它设计的
#      （/map_snapshot 30s 一条全图 + URDF 模型）。
#   C. 容器内 Xvfb + VNC —— 无 X 的远程机器用，见 README 4.6。
#
# ★ 项目纪律：m2_smoke.sh 是判据载体，改它要换代际。所以可视化**不要**去改它，
#   而是另起一个 GUI 进程旁观同一套话题（本脚本就是干这个的）。
# =============================================================================
set -eo pipefail

WS=/opt/ros1-slam
MODE="${1:-check}"

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
[ -f "$WS/ws/devel/setup.bash" ] && source "$WS/ws/devel/setup.bash"

export GAZEBO_PLUGIN_PATH=$WS/ws/devel/lib:/opt/ros/noetic/lib
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:$WS/ws/src/95_simulation:$WS/ws/src/95_simulation/lite3_gazebo_sim/worlds
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:$WS/ws/src/95_simulation
# 仿真域口径与 m2_smoke.sh 保持一致
export ROS_MASTER_URI="${OBSERVE_URI:-${ROS_MASTER_URI:-http://127.0.0.1:11313}}"
export GAZEBO_MASTER_URI="${OBSERVE_GAZEBO_URI:-${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}}"

# ---- 宿主 X 模式（VIZ_HOST_X=1）----
# 默认走容器内 Xvfb 软渲染；要直通宿主机 X（有桌面/GPU 时更流畅）就设 VIZ_HOST_X=1。
# ★ 旁观 smoke 时务必在**同一个容器**里跑（另一个 docker compose exec）：
#   sim 容器的 ROS_IP 是 127.0.0.1，跨容器时 master 会把 127.0.0.1 通告给别人，连不上。
use_host_x() {
    export DISPLAY="${X_DISPLAY:-:1}"
    export XAUTHORITY=/tmp/.Xauthority
    # ★ 默认强制软件渲染（LIBGL_ALWAYS_SOFTWARE=1）：
    #   实测这台机器宿主机 X 自己就是 llvmpipe（glxinfo: Accelerated: no），
    #   容器里也没有 NVIDIA 容器运行时 ⇒ 若放任它去试硬件，只会得到一堆
    #   "libGL error: failed to open nvidia-drm: .../nvidia-drm_dri.so" 噪音，
    #   且个别程序会因上下文建不起来而黑屏。锁死 swrast 反而最稳。
    #   真要在别的机器上走 GPU：mount /dev/dri + 装 NVIDIA 容器运行时，
    #   然后 VIZ_GL_SOFTWARE=0 再跑。
    export LIBGL_ALWAYS_SOFTWARE="${VIZ_GL_SOFTWARE:-1}"
    if [ ! -S "/tmp/.X11-unix/X${DISPLAY#:}" ]; then
        echo "[viz] 警告：/tmp/.X11-unix/X${DISPLAY#:} 不存在 —— 宿主机 X 没挂进来？"
    fi
    if [ ! -f /tmp/.Xauthority ]; then
        echo "[viz] 警告：/tmp/.Xauthority 不存在 —— 认证 cookie 没挂进来，会报 Authorization required"
    fi
}
[ "${VIZ_HOST_X:-0}" = "1" ] && use_host_x

check() {
    echo "=== 可视化环境自检 ==="
    echo "DISPLAY               = ${DISPLAY:-<未设置>}"
    if [ -n "${DISPLAY:-}" ] && [ -S "/tmp/.X11-unix/X${DISPLAY#:}" ]; then
        echo "X 套接字              = /tmp/.X11-unix/X${DISPLAY#:} 存在 ✔"
    else
        echo "X 套接字              = 未找到（宿主机 X 没挂进来？见 README 4.6）"
    fi
    if command -v glxinfo >/dev/null 2>&1; then
        R=$(timeout 15 glxinfo -B 2>/dev/null | grep -m1 'OpenGL renderer' || true)
        if [ -n "$R" ]; then
            echo "OpenGL renderer       = ${R#*: }"
        else
            echo "OpenGL renderer       = 取不到 —— 真实错误："
            timeout 15 glxinfo -B 2>&1 | head -2 | sed 's/^/                         /'
            echo "                         ⇒ 九成是 X 认证。看 README 4.6 的坑表"
            echo "                           （cookie 文件要对上 DISPLAY 的编号；"
            echo "                            本机桌面 :1 的 cookie 在 /run/user/1000/gdm/Xauthority）"
        fi
    else
        echo "OpenGL renderer       = 无 glxinfo（镜像未装 mesa-utils）"
    fi
    for c in gzclient rviz; do
        printf "%-21s = %s\n" "$c" "$(command -v $c || echo 'MISSING（镜像未装，见 WITH_RVIZ / PROFILE=sim）')"
    done
    echo "--- rviz 配置（项目自带）---"
    for f in "$WS/ws/src/10_slam/faster_lio/rviz_cfg/loam_livox.rviz" \
             "$WS/ws/src/30_3rd/plan_manage/launch/default.rviz"; do
        [ -f "$f" ] && echo "  [有] $f" || echo "  [缺] $f"
    done
    echo "--- 当前话题（需要仿真已在跑）---"
    timeout 20 rostopic list 2>/dev/null | head -12 || echo "  取不到话题 —— 仿真没起？先跑 sim.sh gazebo 或 smoke"
}

case "$MODE" in
    check) check ;;
    gzclient)
        # m2_gazebo.launch 的 gui 参数默认 false，这里显式打开；
        # 若 Gazebo 已在跑（gzserver 已起），单起 gzclient 连同一个 GAZEBO_MASTER_URI 即可
        if timeout 10 rostopic list >/dev/null 2>&1; then
            echo "[viz] 检测到 ROS master 已在跑 —— 直接起 gzclient 旁观"
            exec gzclient
        else
            echo "[viz] 没有检测到 master —— 连 Gazebo 一起起（gui:=true）"
            exec roslaunch lite3_gazebo_sim m2_gazebo.launch gui:=true
        fi
        ;;
    rviz-raw)
        # 只看雷达原始点云：fixed frame = rslidar，只要场地起着就能看
        echo "[viz] rviz 原始点云视图（Fixed Frame=rslidar，订阅 /rslidar_points）"
        echo "[viz] 注意：/livox/lidar 是 livox_laser_simulation/CustomMsg，rviz 显示不了，故用 /rslidar_points"
        exec rviz -d "$WS/docker/rviz_sim_raw.rviz"
        ;;
    rviz-map)
        # 看建图过程：需要 faster_lio 在跑，否则 /cloud_registered 不存在，
        # rviz 只会显示一个 Axes（2026-10-17 用户实际踩到的现象）。
        if ! timeout 8 rostopic list 2>/dev/null | grep -qx "/cloud_registered"; then
            echo "[viz] /cloud_registered 不存在 —— 先起建图（faster_lio）"
            setsid nohup roslaunch lite3_gazebo_sim m2_mapping.launch \
                > /tmp/mapping_viz.log 2>&1 < /dev/null &
            for i in $(seq 1 30); do
                sleep 2
                if timeout 8 rostopic list 2>/dev/null | grep -qx "/cloud_registered"; then
                    echo "[viz] /cloud_registered 已上线（等待 ${i} 轮）"
                    break
                fi
            done
        else
            echo "[viz] /cloud_registered 已在 —— 直接开 rviz"
        fi
        echo "[viz] rviz 建图视图（Fixed Frame=camera_init，订阅 /cloud_registered + /cloud_registered_body）"
        exec rviz -d "$WS/docker/rviz_sim_map.rviz"
        ;;
    rviz-nav)
        # 导航视图：项目自带 plan_manage 的配置，需要 SCAN 规划器在跑（SLIM=1 镜像里没编）
        if ! timeout 8 rostopic list 2>/dev/null | grep -qx "/planning/bspline"; then
            echo "[viz] 警告：/planning/bspline 不存在 —— SCAN 规划器没跑（SLIM=1 镜像不含 30_3rd）"
            echo "[viz]       本视图会只有 Axes。要用它请用 SLIM=0 全量镜像并起 SCAN 链"
        fi
        exec rviz -d "$WS/ws/src/30_3rd/plan_manage/launch/default.rviz"
        ;;
    *)
        echo "用法: bash $0 {check|gzclient|rviz-raw|rviz-map|rviz-nav}"
        exit 2
        ;;
esac
