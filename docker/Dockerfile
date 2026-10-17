# =============================================================================
# ros1-slam 隔离部署沙箱 —— Docker 开发环境
# （不写 # syntax= 指令：那会触发 BuildKit 联网拉 frontend，离线/Jetson 内网会卡住）
# -----------------------------------------------------------------------------
# 技术栈：ROS1 Noetic (Ubuntu 20.04 focal) + catkin_make + C++17 + Python3
# 目标机：Jetson tegra194 / aarch64（JetPack 5.x = Ubuntu 20.04）与 x86_64 开发机
#
# 关键约定（不要改，改了编译/运行都会崩）：
#   1. 工作区绝对路径固定为 /opt/ros1-slam/ws
#      —— catkin 的 devel space 把绝对路径烧进了 setup.bash / .cmake 里，
#         编译路径和运行路径必须一致，否则 source devel 失效。
#   2. 用户与家目录固定为 ros1-slam:/home/ros1-slam
#      —— 项目内 scripts/env.sh、scripts/mapping/m_*.sh、.bashrc 里有大量
#         硬编码的 /home/ros1-slam/... 路径，靠家目录里的软链接接住。
#   3. livox_ros_driver2 的 CMakeLists 用 if(ROS_EDITION STREQUAL "ROS1") 分流，
#      必须传 -DROS_EDITION=ROS1，否则走 ROS2 分支必然编译失败。
#   4. Livox-SDK2 必须先安装到 /usr/local/lib/liblivox_lidar_sdk_static.a。
#
# 构建（构建上下文必须是仓库根目录）：
#   docker build -f docker/Dockerfile -t ros1-slam:noetic .
#   docker build -f docker/Dockerfile -t ros1-slam:noetic-sim --build-arg PROFILE=sim .
#
# ★ 基础镜像选择（2026-10-17 实测）：
#   宿主机直连 registry-1.docker.io → 000（超时，被墙）；走 daemon 代理能通但极慢，
#   `docker build` 会在 "load metadata" 阶段 DeadlineExceeded。
#   两条可行路径：
#     A. 零拉取（推荐，本机实测可用）：宿主机已有 fishros2/ros:noetic-desktop-full
#        —— arch=amd64，含 Gazebo 11.10.2 + 234 个 ros-noetic 包 + libpcl-dev /
#        libeigen3-dev / libtbb-dev / libyaml-cpp-dev / libapr1-dev / libboost-all-dev。
#        ★ 注意它缺 libgoogle-glog-dev —— 而 faster_lio 的 cmake/packages.cmake 里
#          find_package(Glog REQUIRED)，不装就编不过。本 Dockerfile 已装。
#        ★ 它还缺 xvfb / mesa-utils / libgl1-mesa-glx / libosmesa6 —— gzserver 无头
#          渲染必需，已在 PROFILE=sim 分支装。
#        ★ 它的 ENTRYPOINT 是 ["/bin/bash"]，直接 docker run <img> bash -lc '...' 会被
#          前置成 `bash bash -lc` 而报 cannot execute binary file —— 需 --entrypoint bash。
#     B. 官方镜像：ROS_BASE_IMAGE=ros + 走代理（HTTP_PROXY/HTTPS_PROXY build-arg），
#        拉取要耐心等（数分钟级）。
# =============================================================================

ARG ROS_BASE_IMAGE=ros
ARG ROS_BASE_TAG=noetic-ros-base-focal

# =============================================================================
# Stage 1 · deps —— 系统依赖 + ROS 依赖（这一层最重，命中缓存最快）
# =============================================================================
FROM ${ROS_BASE_IMAGE}:${ROS_BASE_TAG} AS deps

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    PYTHONNOUSERSITE=1

# ---- 0. ROS apt 源与密钥修复（2026-10-17 实测必需，别删）----
# fishros2/ros 镜像自带的源在这台机器上全残：
#   · mirrors.tuna.tsinghua.edu.cn/ros{,2}/ubuntu  -> 403 Forbidden
#   · packages.ros.org                            -> 可达，但签名 key F42ED6FBAB17C654 已过期
#                                                  （EXPKEYSIG Open Robotics）
# 实测可用：USTC 的 ros 镜像（HTTP 200）+ keyserver.ubuntu.com:80 能刷新该 key（续到 2030-06-01）。
# 为什么非要修：镜像里缺 ros-noetic-move-base-flex / teb-local-planner / costmap-2d /
#              nav-core / move-base / global-planner / base-local-planner / map-server，
#              而 mbf_costmap_nav 是 Stage B 导航的必需件 —— 不修源就装不上，Stage B 直接跑不了。
# 官方 ros: 基础镜像自带 packages.ros.org/ros/ubuntu（本机可达、key 正常），此步自动跳过。
RUN set -eux; \
    if [ -f /etc/apt/sources.list.d/ros-fish.list ]; then \
        printf '%s\n' "deb [arch=$(dpkg --print-architecture)] https://mirrors.ustc.edu.cn/ros/ubuntu/ focal main" \
            > /etc/apt/sources.list.d/ros-fish.list; \
        echo "[fix] ROS apt 源已切到 USTC（原 tuna 403 / packages.ros.org key 过期）"; \
    fi; \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys F42ED6FBAB17C654 \
        || echo "[warn] OSRF key 刷新失败；若 apt 报 EXPKEYSIG，见 docker/README.md 6.2"

# ---- 0.5 依赖缓存挂载点提示 ----
# 下面 apt-get 走 USTC 国内源（实测 0.2~0.3s），不需要、也不要配 HTTP_PROXY：
# ★ 构建容器里的 127.0.0.1 是容器自己的 loopback，把宿主机代理
#   http://127.0.0.1:7897 传进来只会得到 Connection refused（实测踩过）。

# ---- 0.8 构建期代理自检（2026-10-17 踩坑后加，能省你半小时）----
# 坑：把宿主机代理 http://127.0.0.1:<port> 传进 docker build ——
#     构建容器里的 127.0.0.1 是**容器自己的回环**，那个端口没人监听
#     ⇒ apt 全部 Connection refused ⇒ 满屏 "Unable to locate package"，
#       看起来像"镜像/源坏了"，其实是代理指错了地方。
# 结论：国内源（USTC）直连 0.2~0.3s，默认不要配代理。
#       真要用代理（比如换回官方 ros: 基础镜像走 archive.ubuntu.com），
#       必须填**宿主机局域网 IP**，或给 build 加 network: host。
RUN set -eu; \
    for v in HTTP_PROXY HTTPS_PROXY http_proxy https_proxy; do \
        val="${!v:-}"; \
        case "$val" in \
            127.0.0.1:*|localhost:*) \
                echo "[proxy][FATAL] ${v}=${val} 指向本机回环——构建容器内该端口不存在。"; \
                echo "[proxy][FATAL] apt 会全部 Connection refused。请置空（国内源直连），"; \
                echo "[proxy][FATAL] 或改用宿主机局域网 IP，或 build 加 --network=host。"; \
                exit 1;; \
            "") ;; \
            *) echo "[proxy] ${v}=${val}";; \
        esac; \
    done; \
    echo "[proxy] 未配置构建期代理 —— 国内源直连"

# ---- 1. 系统级构建/运行依赖 ----
# libpcl-dev        : faster_lio / livox_ros_driver2 / pcd2grid / ndt_omp 都要 PCL>=1.8
# libeigen3-dev     : faster_lio(Sophus/IKFoM) / fast_gicp / kindr
# libgoogle-glog-dev: faster_lio cmake/packages.cmake 里 find_package(Glog REQUIRED)
# libyaml-cpp-dev   : faster_lio find_package(yaml-cpp REQUIRED)
# libtbb-dev        : faster_lio 并行稀疏体素（ivox）需要 TBB
# libapr1-dev       : livox_ros_driver2 pkg_check_modules(APR apr-1)
# libpcap-dev       : 00_drivers/lslidar_driver 的 include/lslidar_driver/input.h 要 pcap.h
#                     （2026-10-17 实测：漏了它，编译推进到 lslidar_driver 才报出来）
# libnlopt-dev      : 30_3rd SCAN 规划器 bspline_opt 需要 NLopt
# libboost-all-dev  : livox_ros_driver2 find_package(Boost COMPONENTS system thread chrono)
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake pkg-config ninja-build \
        git ca-certificates wget curl \
        libpcl-dev libeigen3-dev libgoogle-glog-dev libyaml-cpp-dev \
        libtbb-dev libapr1-dev libpcap-dev libboost-all-dev \
        libnlopt-dev libnlopt-cxx-dev libomp-dev \
        libgtest-dev \
        python3-pip python3-dev python3-setuptools python3-wheel \
        python3-numpy python3-yaml python3-scipy python3-opencv \
        tmux screen nano vim-tiny less procps psmisc \
        net-tools iputils-ping iproute2 dnsutils lsof \
        acl \
    && rm -rf /var/lib/apt/lists/*

# ★ `acl`（getfacl／setfacl）为什么在依赖面里（2026-10-17 x86 侧实测）：
#   门禁串成员 VAR＝scripts/verify/v_evidence_reachability.sh 要用 getfacl 判 effective 权限，
#   缺它时该件直接 `[前置缺失] getfacl 不可用 ⇒ 无法判 effective` 报 **rc=9**，
#   异机批末串因此少一件。补装后同一调用式现跑＝`VAR VERDICT: PASS=8 FAIL=0`／rc=0。
#   （Lite3 侧本来就有 acl——JetPack 桌面基线带的；本机基座 fishros2/ros 没带。）

# ---- 1.2 ROS Noetic 依赖（按 ws/src 各包 package.xml 反推）----
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-catkin \
        ros-${ROS_DISTRO}-roscpp ros-${ROS_DISTRO}-rospy ros-${ROS_DISTRO}-rostest \
        ros-${ROS_DISTRO}-std-msgs ros-${ROS_DISTRO}-sensor-msgs \
        ros-${ROS_DISTRO}-geometry-msgs ros-${ROS_DISTRO}-nav-msgs \
        ros-${ROS_DISTRO}-visualization-msgs ros-${ROS_DISTRO}-std-srvs \
        ros-${ROS_DISTRO}-tf ros-${ROS_DISTRO}-tf2 ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-eigen-conversions \
        ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-pcl-conversions \
        ros-${ROS_DISTRO}-rosbag ros-${ROS_DISTRO}-roslaunch \
        ros-${ROS_DISTRO}-robot-state-publisher ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-costmap-2d ros-${ROS_DISTRO}-nav-core \
        ros-${ROS_DISTRO}-global-planner ros-${ROS_DISTRO}-base-local-planner \
        ros-${ROS_DISTRO}-move-base ros-${ROS_DISTRO}-move-base-flex \
        ros-${ROS_DISTRO}-teb-local-planner \
        ros-${ROS_DISTRO}-dwa-local-planner \
        ros-${ROS_DISTRO}-spatio-temporal-voxel-layer \
        ros-${ROS_DISTRO}-sob-layer \
        ros-${ROS_DISTRO}-smach ros-${ROS_DISTRO}-smach-ros \
        ros-${ROS_DISTRO}-map-server \
    && rm -rf /var/lib/apt/lists/*

# ★ 上面三枚（dwa / spatio-temporal-voxel-layer / sob-layer）为什么不能省（2026-10-17 x86 侧实测）：
#   它们由**本项目自己的 costmap 配置**点名，却不在任何 package.xml 里声明 ⇒
#   没有任何工具（rosdep / catkin / 本 Dockerfile 的反推清单）能替你发现它们。
#   真身：40_navigation/lite3_navigation/config/{global,local}_costmap_params.yaml 的 plugins 行
#         {name: stvl_obstacle_layer, type: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"}
#         {name: sob_layer,  type: "sob_layer::SobLayer"}
#         与 mbf_navigation.yaml 的 controllers: [{name: DWAPlannerROS, ...}]
#   缺装时的症状不是"少一层障碍"，而是**整条导航链起不来**：
#     terminate called after throwing an instance of 'pluginlib::LibraryLoadException'
#       what(): the class spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer with base class
#               type costmap_2d::Layer does not exist. Declared types are costmap_2d::{Inflation,
#               Obstacle,Static,Voxel}Layer
#     [move_base_flex-9] process has died [exit code -6]
#   ⇒ mbf_costmap_nav 直接 abort，合成动作 /move_base_flex/move_base 的 goal 面根本不 advertise，
#     异机侧照 Lite3 在册式子写的 goal 客户端会 `wait_for_server` 超时——**看起来像"异机接口差"，
#     实际是缺包**。★修法是补装依赖，不是去改那两份 costmap 配置（costmap_common.yaml 属红线件）。

# ---- 1.3 可选：仿真域（Gazebo 11）—— PROFILE=sim 时安装 ----
# ★ 仿真能不能跑，关键就在这一层。三个不能省的点：
#   ① m1/m2_gazebo.launch 里 headless="false"（不传 gzserver -r）⇒ gzserver 会初始化
#      Ogre 渲染引擎，容器内无 X、无 GPU 时必须靠 Mesa 软件渲染 + Xvfb，否则直接
#      "Unable to open display" / GL context 创建失败。
#   ② 世界件用了 model://sun 与 model://ground_plane ⇒ 必须有本地模型库
#      （gazebo11-common 提供 /usr/share/gazebo-11/models），否则 Gazebo 会去联网
#      拉 model database，离线环境必挂。
#   ③ 本项目的雷达是 <sensor type="ray">（CPU 光线投射），不是 gpu_ray
#      ⇒ 仿真本身不需要 NVIDIA GPU / --gpus，纯 CPU + 软件渲染即可。
ARG PROFILE=core
RUN if [ "${PROFILE}" = "sim" ] || [ "${PROFILE}" = "dev" ]; then \
        apt-get update && apt-get install -y --no-install-recommends \
            gazebo11 libgazebo11-dev gazebo11-common \
            ros-${ROS_DISTRO}-gazebo-ros-pkgs ros-${ROS_DISTRO}-gazebo-ros-control \
            ros-${ROS_DISTRO}-gazebo-plugins ros-${ROS_DISTRO}-gazebo-msgs \
            ros-${ROS_DISTRO}-gazebo-ros \
            libgl1-mesa-glx libgl1-mesa-dri libglu1-mesa libosmesa6 mesa-utils \
            xvfb xauth x11vnc \
            ros-${ROS_DISTRO}-rosbridge-suite \
        && rm -rf /var/lib/apt/lists/* ; \
    fi

# Gazebo 资源/模型/插件搜索路径（离线跑仿真的前提：模型只能从本地来）
ENV GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/opt/ros1-slam/ws/src/95_simulation:/opt/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim/worlds \
    GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:/opt/ros1-slam/ws/src/95_simulation \
    GAZEBO_PLUGIN_PATH=/opt/ros1-slam/ws/devel/lib:/opt/ros/noetic/lib \
    LIBGL_ALWAYS_SOFTWARE=1 \
    USE_XVFB=0

# ---- 1.4 可选：rviz 可视化 —— WITH_RVIZ=1 时安装 ----
ARG WITH_RVIZ=0
RUN if [ "${WITH_RVIZ}" = "1" ]; then \
        apt-get update && apt-get install -y --no-install-recommends \
            ros-${ROS_DISTRO}-rviz ros-${ROS_DISTRO}-rqt ros-${ROS_DISTRO}-rqt-graph \
            ros-${ROS_DISTRO}-rqt-tf-tree ros-${ROS_DISTRO}-rqt-plot \
        && rm -rf /var/lib/apt/lists/* ; \
    fi

# ---- 1.5 rosdep 初始化（便于后续按需装包）----
RUN rosdep init || true && rosdep update || true

# ---- 1.6 T1 环境锚对齐（异机基线）----
# 目标串真身＝Lite3 侧在册值（随导出包到达的 deps/versions.tsv，契约＝av 稿 §2.4）。
# 放在 deps 层最末：前面的 apt 层全部保住缓存，只有本层之后的阶段重跑。
#
# ① 点版本串：/etc/os-release 的 PRETTY_NAME 由 base-files 这一个包生成。基座停在
#    20.04.4，focal-updates 的 base-files 11ubuntu5.8 写出来就是 "Ubuntu 20.04.6 LTS"
#    ＝在册值。★只 --only-upgrade 它一个，不跑 dist-upgrade —— 后者会让整片 ROS
#    依赖面跟着漂移，基线反而更不可控。
# ② numpy／scipy 两串：在册值是「版本＋路径」两道闸（1.19.4 与 1.5.3 都要落在
#    /usr/local/lib/python3.8/dist-packages/…/__init__.py）。apt 面那一对
#    （python3-numpy 1.17.4／python3-scipy 1.3.3）在 /usr/lib/python3/dist-packages，
#    ★不卸载也不升级，让 /usr/local 在 sys.path 上自然遮蔽它。
#    已实测：装上之后 cv2 4.2.0 / rospy / cv_bridge 全部仍可 import。
# ③ gazebo11 那串 ★装不出来，本件不假装对齐：在册值 11.11.0-1~focal 在
#    OSRF ubuntu-stable 与 USTC ros 镜像的 pool 里都已不存在，focal 只有 11.15.1
#    （同目录另有 11.9.0，那是 buster 包，跨发行版不可用）。★不为此从源码编 Gazebo，
#    差值如实登记回报。
ARG PIP_INDEX_URL=https://mirrors.aliyun.com/pypi/simple
RUN apt-get update \
    && apt-get install -y --only-upgrade base-files \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir -i "${PIP_INDEX_URL}" "numpy==1.19.4" "scipy==1.5.3" \
    && python3 -c 'import numpy, scipy; print("[T1] numpy", numpy.__version__, numpy.__file__); print("[T1] scipy", scipy.__version__, scipy.__file__)' \
    && grep PRETTY_NAME /etc/os-release

# =============================================================================
# Stage 2 · livox-sdk —— 编译 Livox-SDK2（第三方源码，原样编译不改）
# =============================================================================
FROM deps AS livox-sdk
ARG JOBS=4
COPY opt/3rdparty/Livox_SDK2 /opt/src/Livox_SDK2
RUN mkdir -p /opt/src/Livox_SDK2/build \
    && cd /opt/src/Livox_SDK2/build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && make -j"${JOBS}" \
    && make install \
    && ldconfig \
    && rm -rf /opt/src/Livox_SDK2/build
# ★ 硬校验：livox_ros_driver2 的 find_library 会在 /usr/local/lib 找这个静态库
RUN test -f /usr/local/lib/liblivox_lidar_sdk_static.a \
    && echo "[OK] liblivox_lidar_sdk_static.a installed" \
    || { echo "[FAIL] Livox-SDK2 静态库缺失，检查 opt/3rdparty/Livox_SDK2 是否完整"; exit 1; }

# =============================================================================
# Stage 3 · builder —— 编译 catkin 工作区
# =============================================================================
FROM livox-sdk AS builder
# JOBS 只管 catkin_build.sh 的 ③ 正式编译，默认 4。
# ★ 早先以为"并行会竞态、必须 -j1"是**误判**：真正原因是
#   livox_laser_simulation（第三方副本，按项目红线"一字不改"）的 CMakeLists.txt 缺
#   add_dependencies(livox_laser_simulation ${${PROJECT_NAME}_EXPORTED_TARGETS})，
#   库目标与自己的 *_generate_messages_cpp 目标之间没有依赖关系 ⇒ make 先编 .cpp
#   后生成消息头，必报 fatal error: livox_laser_simulation/CustomMsg.h。
#   这与 -j 无关（-j1 一样挂）。catkin_build.sh 用"先预生成所有消息头、再正式编译"
#   解决了它，所以 ③ 可以安全并行。（-j1 也能过，但 builder 阶段要多花约 380 秒。）
ARG JOBS=4
# ---- 瘦身编译开关 ----
# SLIM=1：只编"跑仿真"真正需要的包。实现方式是在 builder 阶段给非仿真包打 CATKIN_IGNORE。
#   ★ 为什么不用 catkin_make --pkg：实测（2026-10-17）--pkg 传下去
#     CATKIN_WHITELIST_PACKAGES 变成空串，36 个包照样全编，白名单完全无效。
#   ★ 为什么不会污染项目源码：builder 阶段的 ws/src 是 COPY 进来的副本，
#     不是 compose 里 bind-mount 的那份；打标记只发生在镜像层内。
#   ★ 跳过的是这些（省掉最重的几坨）：
#     · fast_gicp —— thirdparty 里带 nvbio / Sophus / Eigen / pybind11，20_localization 一层 3085 个文件基本都是它
#     · hdl_localization / hdl_global_localization / ndt_omp —— 定位候选件，Stage C 才用
#     · 30_3rd/* —— SCAN 规划器五包（要 nlopt）
#     · 30_perception/libraries、deeprobotics_local_height_map —— 高程图方向（本已搁置，且上游闭源缺源码）
#     · mockamap / map_generator / local_sensing / pcd2grid —— 仿真周边与离线工具
#   注意：livox_laser_simulation 的 package.xml 里 build_depend 了 livox_ros_driver2，
#         所以仿真**仍然要编 Livox 驱动 + Livox-SDK2**，这一块省不掉（stage 2 保留）。
ARG SLIM=0
ARG SLIM_SKIP="20_localization/fast_gicp 20_localization/hdl_localization 20_localization/hdl_global_localization 20_localization/ndt_omp 30_3rd 30_perception/libraries 30_perception/deeprobotics_local_height_map 95_simulation/mockamap 95_simulation/map_generator 95_simulation/local_sensing 10_slam/pcd2grid"

# ARGs 转成 ENV，后面 RUN 里用 bash -lc 单引号包裹时也能正常展开
ENV WS_ROOT=/opt/ros1-slam \
    WS_DIR=/opt/ros1-slam/ws \
    JOBS=${JOBS} \
    SLIM=${SLIM} \
    SLIM_SKIP=${SLIM_SKIP} \
    CPATH=/opt/ros/noetic/include

# ★ 为什么加 CPATH（2026-10-17 实测，别删）：
#   lite3_gazebo_sim/CMakeLists.txt 只写了 find_package(gazebo REQUIRED) 和
#   include_directories(${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})，
#   没有把 tf 声明成 catkin 组件 ⇒ /opt/ros/noetic/include 不在编译路径上
#   ⇒ src/lite3_planar_drive.cpp:14 #include <tf/transform_broadcaster.h> 直接
#     fatal error: No such file or directory（头文件明明装在 /opt/ros/noetic/include/tf/ 下）。
#   这里用 CPATH 补上 ROS 总包含目录，**不改项目源码**：
#     · 项目有隔离纪律与门禁（第三方包"一字不改"、判据载体不可擅动），
#       动 CMakeLists 属于改被测件，可能触发代际/验收问题；
#     · CPATH 只作用于 builder 阶段，runtime 阶段不继承，运行期零影响。
#   "正规"修法（需要你权衡后再动）：
#     lite3_gazebo_sim/CMakeLists.txt 里把 find_package(catkin REQUIRED COMPONENTS tf ...)
#     补上，并把 tf 加进 package.xml 的 build_depend。

# 只拷源码空间：build/ devel/ 由本阶段生成
COPY ws/src ${WS_DIR}/src

# ---- 源码补齐（两个上游/仓库的既有约定，镜像内完成，不改项目源码）----
# ① livox_ros_driver2 上游只提供 package_ROS1.xml / package_ROS2.xml，没有 package.xml
#    ⇒ catkin/rospack 根本看不见这个包（实测 rospack find 报 not found），
#      而 livox_laser_simulation 的 package.xml 里 build_depend 了它。
#      这是 livox_ros_driver2 README 里写明的构建前置步骤，这里在镜像内补上。
# ② SLIM=1 时给非仿真包打 CATKIN_IGNORE（只作用于本阶段 COPY 进来的副本）
RUN set -eu; \
    if [ ! -f "${WS_DIR}/src/00_drivers/livox_ros_driver2/package.xml" ] \
       && [ -f "${WS_DIR}/src/00_drivers/livox_ros_driver2/package_ROS1.xml" ]; then \
        cp "${WS_DIR}/src/00_drivers/livox_ros_driver2/package_ROS1.xml" \
           "${WS_DIR}/src/00_drivers/livox_ros_driver2/package.xml"; \
        echo "[fix] livox_ros_driver2: package_ROS1.xml -> package.xml（上游构建前置步骤）"; \
    fi; \
    if [ "${SLIM}" = "1" ]; then \
        n=0; for d in ${SLIM_SKIP}; do \
            if [ -d "${WS_DIR}/src/${d}" ] && [ ! -f "${WS_DIR}/src/${d}/CATKIN_IGNORE" ]; then \
                touch "${WS_DIR}/src/${d}/CATKIN_IGNORE"; n=$((n+1)); \
            fi; \
        done; \
        echo "[build] SLIM=1 —— 已跳过 ${n} 个非仿真包"; \
    fi

# catkin 顶层 CMakeLists.txt（仓库里没入库，这里生成；宿主机挂 src 时由 entrypoint 兜底重建）
RUN bash -lc 'source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd ${WS_DIR}/src \
    && if [ ! -f CMakeLists.txt ]; then catkin_init_workspace; fi'

# 编译（逻辑在 docker/catkin_build.sh，抽成文件是为了能被 bash -n 静态校验）
# 为什么是三步、顺序不能换：
#   livox_laser_simulation 是第三方副本，按项目红线"一字不改"，而它的 CMakeLists.txt 缺
#   add_dependencies(livox_laser_simulation ${${PROJECT_NAME}_EXPORTED_TARGETS})，
#   库目标与自己的 *_generate_messages_cpp 目标之间没有依赖关系 ⇒ make 会先编 .cpp
#   后生成消息头，必报：
#     livox_points_plugin.cpp:18: fatal error: livox_laser_simulation/CustomMsg.h
#   ★ 这与 -j 无关（-j1 同样挂），是目标排序问题，不是并行竞态。
#   解法：① configure → ② 预生成所有消息头 → ③ 正式编译。
#   -DROS_EDITION=ROS1         : livox_ros_driver2 的 ROS1/ROS2 分支开关（必传！）
#   -DPYTHON_EXECUTABLE        : noetic 用 python3
#   -DCATKIN_WHITELIST_PACKAGES= : 显式清空白名单（catkin_make --pkg 会写进 CMakeCache，
#                                 不清之后每次都只编那几个包 —— catkin 已知坑）
COPY docker/catkin_build.sh /usr/local/bin/catkin_build.sh
RUN chmod 0755 /usr/local/bin/catkin_build.sh \
    && /usr/local/bin/catkin_build.sh

# 编译产物自检：仿真关键件必须齐
RUN bash -lc 'source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source ${WS_DIR}/devel/setup.bash \
    && for p in livox_ros_driver2 livox_laser_simulation lite3_gazebo_sim faster_lio lite3_slam lite3_drivers lite3_bringup lite3_navigation; do \
         rospack find "$p" >/dev/null 2>&1 && echo "[OK]   $p" || echo "[MISS] $p（仿真关键件，必须修）"; \
       done \
    && ls -1 ${WS_DIR}/devel/lib/liblite3_planar_drive.so \
    && ls -1 ${WS_DIR}/devel/lib/liblivox_laser_simulation.so \
    && ls -1 ${WS_DIR}/devel/lib/livox_ros_driver2/livox_ros_driver2_node'

# =============================================================================
# Stage 4 · runtime —— 最终运行镜像
# 注意：这里 FROM livox-sdk 而不是 FROM deps —— 否则 --build-arg PROFILE=sim
#       装的 gazebo 包与 WITH_RVIZ=1 装的 rviz 只存在于 deps 层，runtime 层会丢。
# =============================================================================
FROM livox-sdk AS runtime

# ★ 变量名必须避开 UID/GID —— bash 启动时会把 UID/EUID 强制写成只读内置变量，
#   传 ARG UID=1000 在 RUN 里 ${UID} 恒等于 0，chown 会全部落到 root 上。
ARG HOST_UID=1000
ARG HOST_GID=1000
ARG USERNAME=ros1-slam

ENV WS_ROOT=/opt/ros1-slam \
    WS_DIR=/opt/ros1-slam/ws \
    ROS_HOME=/opt/ros1-slam/.ros \
    ROS_LOG_DIR=/opt/ros1-slam/logs \
    ROS1_MASTER_URI=http://127.0.0.1:11311 \
    PYTHONNOUSERSITE=1 \
    TERM=xterm-256color

# ---- 4.1 接住编译产物（Livox SDK 已在 /usr/local，随 livox-sdk 层继承）----
COPY --from=builder ${WS_DIR} ${WS_DIR}
RUN ldconfig

# ---- 4.2 建用户 + 家目录软链接（接住项目里的 /home/ros1-slam/... 硬编码）----
RUN groupadd -g "${HOST_GID}" -o "${USERNAME}" 2>/dev/null || true \
    && useradd -m -u "${HOST_UID}" -g "${HOST_GID}" -o -s /bin/bash "${USERNAME}" 2>/dev/null || true \
    && mkdir -p "${WS_ROOT}/logs" "${WS_ROOT}/.ros" "${WS_ROOT}/data/maps" "${WS_ROOT}/data/waypoints" \
    && for d in ws scripts docker data note opt; do \
         ln -sfn "${WS_ROOT}/${d}" "/home/${USERNAME}/${d}"; \
       done \
    && ln -sfn "${WS_ROOT}/logs" "/home/${USERNAME}/logs" \
    && chown -R "${HOST_UID}:${HOST_GID}" "${WS_ROOT}" "/home/${USERNAME}"

# ---- 4.3 入口 ----
COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
COPY docker/env.sh        /usr/local/bin/ros1_env.sh
RUN chmod 0755 /usr/local/bin/entrypoint.sh /usr/local/bin/ros1_env.sh

WORKDIR ${WS_ROOT}
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash", "-lc", "sleep infinity"]

# 常用端口说明（docker run 时 -p 暴露）：
#   11311/11312/11313 : ROS Master（本项目默认 11312）
#   11345             : Gazebo master
#   56301/56401  UDP  : Livox Mid-360 点云/IMU 上行
