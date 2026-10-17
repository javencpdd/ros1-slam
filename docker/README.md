# ros1-slam · Docker 开发环境

在容器里复刻 `lite3`（Jetson tegra194 / aarch64）上的 ROS1 Noetic 隔离部署沙箱，
用于**开发、仿真验证与门禁跑批**，不替代真机部署。

---

## 一、项目关键结论（为什么镜像这么写）

| 项 | 结论 | 对 Docker 的影响 |
|---|---|---|
| 技术栈 | ROS1 **Noetic** / Ubuntu 20.04 focal，catkin_make，C++14/17，Python3 | 基础镜像 `ros:noetic-ros-base-focal` |
| 架构 | 目标机 tegra194 **aarch64**（JetPack 5 = Ubuntu 20.04），开发机常见 amd64 | 同一 tag 两种架构都有；**Jetson 上必须本地构建，不能拿 amd64 镜像** |
| 构建系统 | `catkin_make`（不是 catkin_tools / colcon） | `ws/src` 缺顶层 `CMakeLists.txt`，需 `catkin_init_workspace` |
| 关键编译开关 | `livox_ros_driver2` 用 `if(ROS_EDITION STREQUAL "ROS1")` 分流 | **必须传 `-DROS_EDITION=ROS1`**，否则走 ROS2 分支必挂 |
| 第三方原生依赖 | Livox-SDK2 → `/usr/local/lib/liblivox_lidar_sdk_static.a` | 必须在编译 ws 之前单独编译安装 |
| 硬编码路径 | `scripts/env.sh`、`.bashrc`、`scripts/mapping/m_*.sh` 里写死 `/home/ros1-slam/...` | 镜像内建同名用户 + 家目录**软链接**接住 |
| 硬编码网络 | `ROS_MASTER_URI=http://192.168.1.103:11312`、`ROS_IP=192.168.1.103` | 全部改成环境变量注入，容器内默认 `127.0.0.1:11311` |
| 数据库 / 缓存 / MQ | **没有**。本项目是机器人算法栈，状态只存在于 ROS 话题、TF、`data/maps/*.pcd`、日志文件 | 不需要 mysql/redis/kafka；compose 里的 `redis` 只是可选模板（`--profile extras`） |
| 真正的"外部依赖" | ROS Master（11312）、Gazebo master（11345）、Livox Mid-360（192.168.1.201，UDP 56301/56401） | 端口映射 + 真机时 `network_mode: host` |
| 排除项 | `30_perception/deeprobotics_local_height_map` 带 `CATKIN_IGNORE`（闭源、0 代码件） | catkin 自动跳过，无需处理 |

**模块层**（`ws/src/`）：`00_drivers`（Livox/lslidar/qnx 桥）→ `10_slam`（faster_lio 建图）→
`20_localization`（hdl/ndt/fast_gicp）→ `30_3rd`（SCAN 规划器五包）+ `30_perception` →
`40_navigation`（mbf 联邦式 + lite3_scan 壳）→ `50_task`（SMACH）→ `60_safety`（`/cmd_vel` 唯一写者）→
`70_bringup`（TOP 编排）→ `90_supervisor` → `95_simulation`（Gazebo）。

**入口文件**：`scripts/env.sh`（环境入口）、`scripts/mapping/m_start.sh`（采图三窗一键起）、
`ws/src/70_bringup/lite3_bringup/launch/mode_*.launch`（TOP 编排入口）、
`ws/src/95_simulation/lite3_gazebo_sim/launch/m*_gazebo.launch`（仿真入口）。

---

## 二、产出文件

| 文件 | 作用 |
|---|---|
| `../.dockerignore` | 构建上下文忽略表（放仓库根，构建上下文就是仓库根） |
| `Dockerfile` | 四阶段：deps → livox-sdk → builder → runtime |
| `docker-compose.yml` | `roscore` + `slam`（bridge）+ `slam-host`（host 网络）+ 可选 `redis` |
| `entrypoint.sh` | 装 ROS 环境、兜底 catkin 初始化、注入网络口径 |
| `env.sh` | 容器内完整环境入口（先吃项目原 `env.sh`，再覆写网络口径） |
| `build.sh` | 容器内增量/全量编译脚本 |
| `docker/sim.sh` | 仿真统一入口：`probe` / `gazebo` / `smoke [stage]` 三模式 |
| `docker/viz.sh` | 可视化入口：`check` / `gzclient` / `rviz-map` / `rviz-nav` |
| `docker/lint.sh` | 构建前静态自检（Dockerfile RUN 引号配平 + `bash -n` + compose 解析） |
| `docker/catkin_build.sh` | 镜像构建期的工作区编译（三步：configure → 预生成消息头 → 正式编译） |
| `.env.example` | compose 变量模板 |

---

## 三、命令速查

### 0 · 前置

```bash
cd /home/jack/ysc-code/ros1-slam
cp docker/.env.example docker/.env
# 让容器写出的文件归属你自己，而不是 root
sed -i "s/^HOST_UID=.*/HOST_UID=$(id -u)/;s/^HOST_GID=.*/HOST_GID=$(id -g)/" docker/.env
```

### 1 · 构建镜像（构建上下文必须是仓库根）

```bash
# 运行栈（默认）
docker build -f docker/Dockerfile -t ros1-slam:noetic .

# 含 Gazebo 的仿真栈
docker build -f docker/Dockerfile -t ros1-slam:noetic-sim --build-arg PROFILE=sim .

# Jetson 上内存吃紧时降并发
docker build -f docker/Dockerfile -t ros1-slam:noetic --build-arg JOBS=2 .
```

> 首次全量编译较慢：x86 约 10–20 分钟，Jetson Xavier NX 约 40–90 分钟。
> `deps` 层被缓存后，改代码重编只重跑 builder 层。

### 2 · 启动 / 停止

```bash
cd docker

docker compose up -d --build        # 构建并后台启动（roscore + slam）
docker compose ps                   # 看状态
docker compose down                 # 停止并移除容器（数据卷保留）
docker compose down -v              # 连数据卷一起删（下次要重编）

# 真机形态：host 网络，直连 LiDAR
docker compose --profile hostnet up -d slam-host

# 可选外部件
docker compose --profile extras up -d
```

单容器直跑（不用 compose）：

```bash
docker run -d --name ros1-slam-dev \
  --network host \
  --shm-size=2g \
  -e ROS1_MASTER_URI=http://192.168.1.103:11312 \
  -e ROS1_IP=192.168.1.103 \
  -v "$PWD/ws/src:/opt/ros1-slam/ws/src" \
  -v "$PWD/scripts:/opt/ros1-slam/scripts" \
  -v "$PWD/docker:/opt/ros1-slam/docker" \
  -v "$PWD/data:/opt/ros1-slam/data" \
  -v ros1_ws_build:/opt/ros1-slam/ws/build \
  -v ros1_ws_devel:/opt/ros1-slam/ws/devel \
  ros1-slam:noetic
```

### 3 · 进入容器 / 调试

```bash
docker compose exec slam bash                 # 进入（已装好 ROS + devel 环境）
docker exec -it ros1-slam-dev bash            # 直跑方式

# 容器内：装完整环境
source /opt/ros1-slam/docker/env.sh

# 容器内：改完 ws/src 后增量重编
bash /opt/ros1-slam/docker/build.sh
bash /opt/ros1-slam/docker/build.sh --clean   # 全量重编

# 容器内：看包定位 / 话题
rospack find faster_lio
rostopic list && rostopic hz /Odometry
rosnode list
```

### 4 · 看日志

```bash
docker compose logs -f slam                   # 容器 stdout
docker compose logs -f --tail=200 roscore
docker exec slam bash -lc 'ls -lt /opt/ros1-slam/logs | head'   # ROS_LOG_DIR
docker exec slam bash -lc 'tail -f /opt/ros1-slam/logs/latest/*.log'
```

### 5 · 跑测试 / 门禁

```bash
docker compose exec slam bash -lc '
  source /opt/ros1-slam/docker/env.sh
  cd /opt/ros1-slam
  bash scripts/verify/v_sh_py_syntax.sh <被检件清单>; echo "rc=$?"
  bash scripts/verify/v_harness_generation_table.sh --check
  bash scripts/verify/v3_isolation.sh; echo "rc=$?"          # 期望 0
  bash scripts/verify/v_deferred_clause_scan.sh              # rc=1 只是提示
'
```

> 门禁件依赖 `rospack find` / `catkin_make`，**必须先 source 环境**，
> 否则会把"跑不动"误判成"隔离违规"（项目 README 里点名的在册教训）。

### 6 · 起仿真 / 起编排（容器里）

```bash
source /opt/ros1-slam/docker/env.sh
roslaunch lite3_gazebo_sim m2_gazebo.launch          # 仿真场地
roslaunch lite3_bringup mode_sim_teb.launch goal_x:=3.0 goal_y:=0.0
```

---

## 四、仿真（Gazebo）能不能在 Docker 里跑？

**结论：能，已验证跑通。** 2026-10-17 在本机（Docker 29.9.1 / x86_64）实机验证：
镜像构建成功 → 容器起 Gazebo → 全部关键话题上线 → 三项判据指标达标。
详见本节末「6.8 实机验证记录」。

### 4.0 先算账：跑仿真到底要不要全量编译？

**要编译，但"全量"只做一次，而且可以瘦身。**

| 场景 | 要不要编 | 说明 |
|---|---|---|
| 首次构建镜像 | **要，且这是唯一一次重活** | 仿真的核心是 C++ 插件：`liblite3_planar_drive.so`、`liblivox_laser_simulation.so`、`faster_lio`。<br>而且 `lite3_gazebo_sim/CMakeLists.txt` 里 `find_package(gazebo REQUIRED)` ⇒ **必须用 `PROFILE=sim` 的镜像编，`core` 镜像根本编不出这些包** |
| 之后每次跑仿真 | **不用** | 编译产物在命名卷 `ros1_ws_devel` 里，容器停了、重建了都不丢 |
| 改了 `ws/src` 之后 | 增量即可 | `bash /opt/ros1-slam/docker/build.sh`（改 C++ 才需要；改 Python/yaml 直接生效） |

**瘦身首编（推荐，省一半以上时间）**

```bash
docker build -f docker/Dockerfile -t ros1-slam:noetic-sim \
  --build-arg PROFILE=sim --build-arg SLIM=1 .

# 或者进容器后再编：
docker compose exec sim bash /opt/ros1-slam/docker/build.sh --slim
```

`SLIM=1` 只编白名单包（依赖由 catkin 自动带上），省掉的是三坨：

- `20_localization/fast_gicp` —— 自带 nvbio / Sophus / Eigen 三套 thirdparty，**源码量最大**
- `30_3rd/*` —— SCAN 规划器五包（要 nlopt）
- 真机侧 `lslidar_*` / `message_transformer`(ros2qnx) / `pcd2grid` / `kindr` 等

白名单：`livox_laser_simulation lite3_gazebo_sim faster_lio lite3_slam lite3_drivers lite3_bringup lite3_navigation lite3_localization`

**省不掉的**：`livox_laser_simulation/package.xml` 里写了 `<build_depend>livox_ros_driver2</build_depend>`
⇒ **仿真仍然要编 Livox 驱动 + Livox-SDK2**（Dockerfile 的 stage 2 别删）。

**一个 catkin 已知坑**：`--pkg` 白名单会写进 `CMakeCache.txt`。
`build.sh` 的非 slim 分支已显式传 `-DCATKIN_WHITELIST_PACKAGES=` 清掉；如果你手工跑过
`catkin_make --pkg`，下次务必先清，否则之后每次都只编那几个包。

### 4.1 先看清仿真到底要什么（从源码查出来的，不是猜的）

| 查证点 | 证据 | 含义 |
|---|---|---|
| 雷达传感器类型 | `urdf/lite3_planar.urdf.xacro:71` —— `<sensor type="ray">` | **CPU 光线投射，不是 `gpu_ray`** ⇒ **不需要 NVIDIA GPU，不用 `--gpus`** |
| Gazebo GUI | `m1/m2_gazebo.launch` —— `gui:=false` | 只起 `gzserver`，不起 `gzclient` ⇒ 不需要把宿主机 X 接进来 |
| 渲染开关 | `m1/m2_gazebo.launch` —— `headless="false"` | **不传 gzserver 的 `-r`** ⇒ gzserver 仍会初始化 Ogre 渲染引擎 ⇒ **容器内必须有 GL 上下文**：Mesa 软件渲染 + Xvfb（这是最容易踩的空） |
| 世界模型 | `worlds/m1_world.world` —— `model://sun`、`model://ground_plane` | 必须有**本地模型库**（`gazebo11-common`），否则 Gazebo 联网拉 model database，离线必挂 |
| ROS/Gazebo 域 | `verify/m2_smoke.sh:29` —— `ROS_MASTER_URI=127.0.0.1:11313`、`GAZEBO_MASTER_URI=127.0.0.1:11345` | **仿真自成一域**：`roslaunch` 在本容器内自起 master(11313)，**不需要外部 roscore 服务** |
| 退净判据 | `m2_smoke.sh:58` —— `PORTS="11311 11312 11313 11345"` | 跑完 11311/11312 必须零监听；仿真容器天然满足（独立网络命名空间） |
| 判据日志 | `m2_smoke.sh:16` —— `PAYLOAD=/tmp/m2_payload` | 容器内 `/tmp`，正常；**跑完要自己归档**，容器一删就没了 |
| 样张 PCD | `m2_smoke.sh:18` —— `/home/ros1-slam/data/maps/lite3.pcd` | 仓库里 `data/maps/*` 被 gitignore，**这份样张大概率不在**；Stage C 会受影响，Stage A/B 不受（有 `-f` 保护） |

### 4.2 三个必须条件（已补进配置）

1. **软件 OpenGL**：`libgl1-mesa-glx libgl1-mesa-dri libglu1-mesa libosmesa6` + `LIBGL_ALWAYS_SOFTWARE=1`
2. **虚拟屏**：`xvfb`，`USE_XVFB=1` 时 entrypoint 起 `Xvfb :99` 并设 `DISPLAY=:99`
3. **本地模型库**：`gazebo11-common`（提供 `/usr/share/gazebo-11/models/{sun,ground_plane}`）
   —— 这三条已写进 `Dockerfile` 的 `PROFILE=sim` 分支。

### 4.3 一条性能红线（最可能让你"跑起来但判不了"的地方）

`m1_world.world` 设 `max_step_size=0.001`、`real_time_update_rate=500` ⇒ **RTF 上限 0.5**。
而 `m2_smoke.sh:33` 的 A0 前置判据是 **RTF < 0.25 判 INVALID（本轮作废重跑，不计 FAIL）**。

> 也就是说：软件渲染 + 容器里跑，RTF 可能掉到 0.25 以下 —— 结果不是"仿真失败"，
> 而是"这一轮判据作废"。如果读到 INVALID，先做这两件事再判被测件：
> ① `real_time_update_rate` 从 500 调回 1000（世界件注释里写的官方口径）；
> ② 给容器更多 CPU（`--cpus`），别让 make/gzserver 抢核。
> 这是**判据载体口径问题**，不是代码 bug —— 别急着改 `m2_smoke.sh`（它是判据载体，改了要换代际）。

### 4.4 跑起来

```bash
cd docker
# ★ 必须显式写服务名 sim：不带 profiles 的 roscore/slam 会被一起拉起，
#   而 slam 与 sim 共用命名卷 ros1_ws_devel（一份 devel 两处写会互相覆盖）
docker compose --profile sim up -d --build sim
docker compose exec sim bash /opt/ros1-slam/docker/sim.sh probe    # 先自检
docker compose exec sim bash /opt/ros1-slam/docker/sim.sh smoke stageA
docker compose exec sim bash /opt/ros1-slam/docker/sim.sh gazebo   # 只起场地看日志
```

`probe` 会打印：DISPLAY、`glxinfo` 软渲染器、gzserver 版本、sun/ground_plane 模型、
`liblite3_planar_drive.so` / `liblivox_laser_simulation.so` 是否存在、端口是否干净。
**任何一个 NOT OK 先修它，别直接跑 smoke。**

### 4.6 可视化：跑测试的时候怎么看过程

**能看，三条路按场景选。** 项目里现成的可视化配置就有不少
（`faster_lio/rviz_cfg/loam_livox.rviz` 看建图、`plan_manage/launch/default.rviz` 看导航），
不用自己配。

> ★ 项目纪律：`m2_smoke.sh` 是**判据载体**，改它要换代际、五连重算。
> 所以可视化**不要**去改它 —— 另起一个 GUI 进程旁观同一套话题就行
> （`viz.sh` 就是干这个的，判据域 11313/11345 不变）。

#### 路线 A · 宿主机 X 直通（推荐）

实测这台机器：X 服务器在 `:1`（`X.Org Foundation` 11.0），套接字 `/tmp/.X11-unix/X1`。
把 X 套接字和**认证 cookie** 一起挂进容器即可，**不需要重建镜像**：

```bash
cd docker && docker compose --profile viz up -d sim-viz
docker compose exec sim-viz bash /opt/ros1-slam/docker/viz.sh check      # 先自检
```

`viz.sh check` 会打印：`DISPLAY`、X 套接字是否存在、`OpenGL renderer`、
gzclient/rviz 是否就位、当前话题。**renderer 那行拿不到值就先修 X，别急着起 GUI。**

然后（仿真另起，见 4.4）：

```bash
docker compose exec sim-viz bash /opt/ros1-slam/docker/viz.sh gzclient    # Gazebo 3D 客户端
docker compose exec sim-viz bash /opt/ros1-slam/docker/viz.sh rviz-map    # rviz 看建图
docker compose exec sim-viz bash /opt/ros1-slam/docker/viz.sh rviz-nav    # rviz 看导航
```

`gzclient` 也可以直接并进仿真启动：`roslaunch lite3_gazebo_sim m2_gazebo.launch gui:=true`
（该 launch 的 `gui` 参数默认 `false`，显式打开即可）。

**踩过的坑（都已写进 compose）**：

| 现象 | 原因 | 处置 |
|---|---|---|
| `Authorization required, but no authorization protocol specified` | 只挂了 X 套接字，没给认证 cookie | compose 已挂 `X_AUTH_FILE`（默认 `/run/user/1000/gdm/Xauthority`） |
| 挂了 cookie 还是报同样的错 | **cookie 文件拿错了**。本机 `/home/jack/.Xauthority` 里只有 `unix:10`/`unix:11` 的 cookie，而桌面 X 是 `:1`，它的 cookie 在 `/run/user/1000/gdm/Xauthority` | 用 `X_AUTH_FILE` 指到对的文件；或直接 `xhost +local:root` 绕过（宿主机执行，可逆） |
| `XAUTHORITY` 被设成容器里不存在的路径 | compose 里写了 `${XAUTHORITY:-...}`，把调用方 shell 的值继承进来了 | 已改为写死 `/tmp/.Xauthority`（与 `HTTP_PROXY` 那次同型的坑） |
| renderer 一直是 `llvmpipe` | 镜像里全局 `LIBGL_ALWAYS_SOFTWARE=1` | `sim-viz` 已覆盖为 `0` |

> 实测本机宿主机 X 给出的就是 `llvmpipe`（`Accelerated: no`）——这是**宿主机 X 会话自身的状态**，
> 容器改不了。GL 上下文可用，gzclient/rviz 能正常开窗，只是帧率有限。
> 想更快只能治宿主机 X（让它加载 NVIDIA GLX）。

#### 路线 B · Foxglove 网页端（不需要 X，任意机器浏览器）

项目自带的 `lite3_bringup/mapping_viz.launch` 就是为它设计的
（`/map_snapshot` 30s 一条全图 + URDF 模型自动跟随 `/Odometry`）：

```bash
docker compose --profile extras up -d rosbridge      # 起 rosbridge(9090)，需 PROFILE=sim 镜像
docker compose exec sim bash -lc '
  source /opt/ros1-slam/docker/env.sh
  export ROS_MASTER_URI=http://127.0.0.1:11313
  roslaunch lite3_bringup mapping_viz.launch'
```

然后浏览器开 https://foxglove.dev → 新建布局 → `Foxglove WebSocket` 连
`ws://<宿主机IP>:9090`，固定参考系 `camera_init`，加 3D 面板订阅 `/map_snapshot`。

#### 路线 C · 容器内 VNC（无 X 的远程机器）

镜像已装 `x11vnc`；容器里起一块虚拟屏再开 VNC：

```bash
docker compose exec sim bash -lc '
  export DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1
  Xvfb :99 -screen 0 1280x1024x24 & sleep 2
  x11vnc -display :99 -forever -shared -rfbport 5900 &'
```

宿主机（或任意能连到 5900 的机器）用 VNC 客户端连 `<宿主机IP>:5900`。
软渲染下帧率有限，适合"看个大概"，细看请用路线 A。

#### 三条路怎么选

| 场景 | 路线 |
|---|---|
| 就坐在这台机器前，有桌面 | **A**（真 GPU，最流畅） |
| 远程 SSH、想用浏览器看 | **B**（零 X 依赖，还能分享布局） |
| 远程 SSH、只有 VNC 客户端 | **C**（软渲染，凑合） |

#### 跑批时的建议搭配

`m2_smoke.sh` 本身**不需要 GUI**，可以无头跑判据；想同时看过程就：
另开一个终端跑路线 A 的 `gzclient` / `rviz-map` 旁观 —— 两者互不影响，
也不会碰到判据载体。注意跑完按 `m2_smoke.sh` 的退净判据确认 11311/11312 零监听。


### 4.5 没跑通的兜底判断顺序

`/clock` 出不来 → gzserver 没起来 → 看 `probe` 的 GL/模型两行（99% 是这俩）
→ `/rslidar_points` 出不来 → `liblivox_laser_simulation.so` 没加载，看 `GAZEBO_PLUGIN_PATH`
→ `/livox/imu` 出不来 → IMU 插件 `update_rate=200` 在软渲染下喂不饱，属 RTF 问题
→ 全出但判 INVALID → RTF<0.25，按 4.3 处理
→ GUI 起不来 / 花屏 → 看 `viz.sh check` 的 `OpenGL renderer` 那行，按 4.6 选路线

---

## 五、注意事项（坑位清单）

1. **架构必须一致**：aarch64 机器上必须用 aarch64 镜像。跨架构跑要么起不来，
   要么 qemu 慢到不可用。核对：`docker image inspect ros1-slam:noetic --format '{{.Architecture}}'`。
2. **catkin devel 空间烧死绝对路径**：`ws` 在容器里的路径固定为 `/opt/ros1-slam/ws`。
   换了路径就要全量重编，别挪。
3. **`-DROS_EDITION=ROS1` 不能漏**：漏了 `livox_ros_driver2` 走 ROS2 分支，
   报 `find_library liblivox_lidar_sdk_shared.so REQUIRED` 找不到。
4. **文件权限**：容器默认以 root 跑，写进 `ws/src` 的文件在宿主机上是 root 属主。
   把 `.env` 里的 `HOST_UID/HOST_GID` 设成 `id -u`/`id -g`，并在 compose 里放开 `user:` 那行。
   （Dockerfile 里变量名必须叫 `HOST_UID` —— bash 把 `UID` 当只读内置变量会覆盖掉传入值。）
5. **依赖缓存**：`deps` 层放在最前面就是为了命中缓存。改 `Dockerfile` 时尽量只动后面几层；
   真正要清缓存：`docker build --no-cache` 或 `docker builder prune`。
6. **端口冲突**：`11311/11312/11313/11345/56301/56401` 是项目在册端口。
   宿主机上若已有 ROS1 或 foxy 栈在跑，会 `bind` 失败。
   排查：`ss -lntup | grep -E '1131[123]|11345'` 与 `ss -lnup | grep 56[34]01`。
   真机形态建议直接 `network_mode: host`，让容器和宿主机共用一套端口。
7. **`network_mode: host` 时 `ports:` 会被忽略**，别指望映射生效。
8. **源码热更新的边界**：`ws/src` 是挂载的，改 Python 节点（rospy）通常直接生效；
   改 C++ / 新增包 / 改 `package.xml` 必须进容器跑 `build.sh` 重编。
   `ws/build` 与 `ws/devel` 用命名卷持久化，容器重建不丢；换镜像后要 `--clean` 重编。
9. **`ws/src/CMakeLists.txt` 仓库里没入库**：镜像构建时会生成，
   但用 `-v` 挂宿主机 `ws/src` 后会被覆盖掉 —— entrypoint 与 build.sh 都有幂等兜底重建
   （会在宿主机 `ws/src/` 下生成一个符号链接，属预期，可加进 `.gitignore`）。
10. **Gazebo 需要 `/dev/shm`**：compose 已给 `shm_size: 2gb`；`docker run` 记得加 `--shm-size=2g`，
    否则 gzserver 会莫名崩。
11. **GUI（rviz / gzclient）**：需 `WITH_RVIZ=1` 构建 + 挂 `DISPLAY` 与 `/tmp/.X11-unix`，
    宿主机先 `xhost +local:root`。SSH 无 X 的环境别开 `rviz:=true`
    （项目里 `required=true` 会拖死 launch —— 已在 `mbf_navigation.launch` 里默认 `rviz:=false`）。
12. **镜像体积**：core 约 4–6 GB，sim（含 Gazebo）约 7–9 GB。`.dockerignore` 已挡掉
    `.git`、证据日志、`ws/build`、`ws/devel`、`data/maps`，别轻易放宽。
13. **`fast_gicp` 的 thirdparty 子模块**（Sophus/nvbio/Eigen/pybind11）若编译报错，
    多半是子模块没同步；本项目是"原样复制"的副本，一般不触发。真出问题可临时
    `touch ws/src/20_localization/fast_gicp/CATKIN_IGNORE` 跳过该包。
14. **真机红线不变**：容器化不改变项目三条硬规则 —— `ros2qnx` 仅真机 mode；
    采图全程不发 `/cmd_vel`；隔离目录内禁出现开发侧家目录路径字符串。
    门禁 `v3_isolation.sh` 在容器里同样要跑绿。

---

## 六、排错：`DeadlineExceeded` 拉不到基础镜像 / `apt-get` 失败

### 6.1 先说实测结论（2026-10-17 在这台机器上量到的）

| 探测 | 命令 | 结果 | 含义 |
|---|---|---|---|
| 宿主机直连 Docker Hub | `curl https://registry-1.docker.io/v2/` | **000（10s 超时）** | 直连被墙 |
| 宿主机走代理 | `curl -x http://127.0.0.1:7897 https://registry-1.docker.io/v2/` | **401** | 代理通（401 是 `/v2/` 未带 token 的正常响应） |
| Docker daemon 代理 | `docker info \| grep -i proxy` | **已配 `http://127.0.0.1:7897`** | daemon 侧没问题 |
| 实际拉取 | `timeout 90 docker pull ros:noetic-ros-base-focal` | 层在下载，90s 没拉完 | **通，但极慢** |

⇒ **根因不是"没配代理"，而是基础镜像体积大 + 代理链路慢，超过构建期限。**
报错里的 `load metadata` 是 BuildKit 的期限提示，不是"镜像不存在"——
tag 不存在时报的会是 `manifest unknown`。

**这台机器上最快的解法：用本地已有镜像，一次都不拉。**

```bash
docker images | grep -i ros      # 本机已有：fishros2/ros:noetic-desktop-full  4.77GB
```

`.env` 已默认指向它（`ROS_BASE_IMAGE=fishros2/ros` / `ROS_BASE_TAG=noetic-desktop-full`）。
实测这个镜像里**已经有**：`/opt/ros/noetic`、Gazebo **11.10.2**（`gazebo11`+`gazebo11-common`+`gazebo11-dev`）、
**234 个 `ros-noetic-*` 包**、`libpcl-dev`、`libeigen3-dev`、`libtbb-dev`、`libyaml-cpp-dev`、
`libapr1-dev`、`libboost-all-dev`、`libgl1-mesa-dri`、`python3-numpy/yaml`、`rosdep`。

它**缺**的（本 Dockerfile 都已补装，别删）：

| 缺什么 | 谁要 |
|---|---|
| `libgoogle-glog-dev` | `faster_lio/cmake/packages.cmake` 的 `find_package(Glog REQUIRED)` —— **不装直接编不过** |
| `xvfb` | gzserver 无头渲染要虚拟屏 |
| `mesa-utils`（`glxinfo`）、`libgl1-mesa-glx`、`libosmesa6` | GL 上下文 / 软件渲染 |
| `libnlopt-dev`、`libnlopt-cxx-dev` | `30_3rd` SCAN 五包（`SLIM=1` 时用不到） |
| `libomp-dev` | OpenMP（`ndt_omp` 等） |

> ⚠️ **`fishros2/ros:*` 的 `ENTRYPOINT` 是 `["/bin/bash"]`**。直接
> `docker run fishros2/ros:noetic-desktop-full bash -lc '...'` 会被前置成 `bash bash -lc`
> 而报 `cannot execute binary file`，要加 `--entrypoint bash`。
> （本项目自己的 Dockerfile 会重设 ENTRYPOINT，不受影响。）

### 6.2 给 Docker daemon 配代理（治本，推荐）

```bash
sudo mkdir -p /etc/systemd/system/docker.service.d
printf '%s\n' '[Service]' 'Environment="HTTP_PROXY=http://127.0.0.1:7897"' 'Environment="HTTPS_PROXY=http://127.0.0.1:7897"' 'Environment="NO_PROXY=localhost,127.0.0.1,::1"' | sudo tee /etc/systemd/system/docker.service.d/http-proxy.conf
sudo systemctl daemon-reload && sudo systemctl restart docker
```

> **粘贴前先自查文件有没有被粘坏**（聊天里复制多行常丢换行）：
> `cat /etc/systemd/system/docker.service.d/http-proxy.conf` 必须是 4 行；
> 静态校验：`systemd-analyze verify /etc/systemd/system/docker.service.d/http-proxy.conf`

确认生效（这才是可信的确认，别信"命令跑完了"）：

```bash
systemctl show docker --property=Environment | tr ' ' '\n' | grep -i proxy
docker info | grep -i proxy
```

### 6.3 换镜像源（不想动 daemon 代理时）

`.env` 里改一行即可，Dockerfile 已支持：

```bash
ROS_BASE_IMAGE=docker.mirrors.ustc.edu.cn/library/ros
```

再拉一次验证：`docker pull docker.mirrors.ustc.edu.cn/library/ros:noetic-ros-base-focal`

### 6.4 如果宿主机已经有别的基础镜像

`docker images` 里若已有 `ubuntu:20.04` 或某个 `ros:*`，可把 `ROS_BASE_IMAGE` 指过去复用。
注意：**换成非 ros 官方镜像时，`/opt/ros/noetic` 与 `rosdep` 都不存在**，编译会挂在
`source /opt/ros/noetic/setup.bash` —— 那种情况请回到 6.2/6.3 把官方镜像拉下来。

### 6.5 镜像拉到了，但 `apt-get update` 卡住

同样是代理问题，但作用在**构建期**。用 `.env` 里的三项（已由 compose 透传）：

```bash
HTTP_PROXY=http://127.0.0.1:7897
HTTPS_PROXY=http://127.0.0.1:7897
NO_PROXY=localhost,127.0.0.1,::1
```

`HTTP_PROXY` 等是 BuildKit 预置 ARG，只影响构建过程，**不会烧进镜像层**
（可用 `docker history ros1-slam:noetic | grep -i proxy` 自查，应该查不到）。

### 6.6 一次到位的构建命令（上面都配好后）

```bash
cd docker && SLIM=1 docker compose --profile sim build sim
# 或者完全手工：
docker build -f docker/Dockerfile -t ros1-slam:noetic-sim \
  --build-arg PROFILE=sim --build-arg SLIM=1 \
  --build-arg HTTP_PROXY=http://127.0.0.1:7897 \
  --build-arg HTTPS_PROXY=http://127.0.0.1:7897 .
```

### 6.7 顺手修掉的两个编排问题（本轮已改）

- **`--profile sim` 会把 `slam` 也拉起来**：不带 `profiles` 的服务在 `--profile` 下依然启用，
  而 `slam` 与 `sim` 共用命名卷 `ros1_ws_devel`，两份容器写同一份 devel 会互相覆盖。
  ⇒ 现在统一要求写服务名：`docker compose --profile sim up -d sim`。
- **`slam` 原来占了 11313/11345**：那是仿真域端口（`m2_smoke.sh:29` 写死），
  两个服务同时起会 `port is already allocated`。已从 `slam` 的 `ports` 里摘掉。

### 6.8 实机验证记录（2026-10-17，Docker 29.9.1 / x86_64 / 31GB）

**镜像构建**：`SLIM=1 PROFILE=sim` + 本地 `fishros2/ros:noetic-desktop-full` 基础镜像
⇒ `ros1-slam:noetic-sim`（7.53 GB）构建成功，8 个关键包 `rospack find` 全 OK，
`liblite3_planar_drive.so` / `liblivox_laser_simulation.so` / `livox_ros_driver2_node` 均在位。

**容器内 `sim.sh probe`**：

| 检查项 | 结果 |
|---|---|
| 软渲染 | `OpenGL renderer string: llvmpipe (LLVM 12.0.0, 256 bits)` |
| gzserver | `/usr/bin/gzserver`，Gazebo **11.15.1** |
| 模型库 | `sun` / `ground_plane` 均在 `/usr/share/gazebo-11/models/` |
| 端口 | 11311/11312/11313/11345 起跑前均无监听 |

**起 `m2_gazebo.launch` 后的话题**：`/clock`、`/livox/imu`、`/livox/lidar`、
`/rslidar_points`、`/sim/odom_gt`、`/tf`、`/tf_static`、`/joint_states`、`/gazebo/*`、`/cmd_vel`。

**三项判据指标**：

| 指标 | 判据门槛 | 实测 | 结论 |
|---|---|---|---|
| `/rslidar_points` | ≥ 5 Hz | **10.003 Hz** | 达标 |
| `/livox/imu` | ≥ 100 Hz | **200.000 Hz** | 达标 |
| RTF（`/gazebo/performance_metrics`） | ≥ 0.25（低于则判 INVALID） | **0.396** | 达标，不会作废 |

**收尾**：容器已 `down` 移除，宿主机 11311/11312/11313/11345 均无监听，
无残留容器。镜像保留（7.53 GB），重启秒起。

**未跑**：`m2_smoke.sh` 全量三阶段（Stage A/B/C）没跑——那需要跑完驱动相位与退净判据，
属验收批跑，请在容器里按 4.4 的命令自行执行。
