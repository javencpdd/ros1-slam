# ros1-slam · Docker 开发与仿真

> 最新更新时间：2026-11-05 11:49:54 CST。本轮按 Dockerfile、Compose、脚本和 launch 静态核对；未构建镜像、未启动容器或复跑算法验收。

x86 的 ROS1/Noetic、Gazebo classic 与回归开发环境。Lite3 采用原生 ROS1，地图和现场配置各机独立。当前旧算法、地图准备和真机排程仍按[执行队列](../note/计划书v8/23_v8任务队列.md)保持取消；本文不授权启动硬件。架构与能力边界见[项目架构](../note/架构/项目架构与目录.md)和[进度](../note/进度/README.md)。

## 一、文件与运行形态

| 文件 | 作用 |
| --- | --- |
| [Dockerfile](Dockerfile) | `deps → livox-sdk → builder → runtime`；Noetic、catkin，仿真用 `PROFILE=sim` |
| [docker-compose.yml](docker-compose.yml) | 开发、无头/桌面仿真、现场模板和 extras |
| [.env.example](.env.example) | Compose 变量模板，`.env` 属本机配置 |
| [entrypoint.sh](entrypoint.sh)、[env.sh](env.sh) | ROS、工作区、网络和可选 Xvfb 环境 |
| [catkin_build.sh](catkin_build.sh) | configure → 预生成消息头 → 正式编译 |
| [build.sh](build.sh) | 容器内增量构建，仅接受可选 `--clean`，**没有 `--slim`** |
| [sim.sh](sim.sh) | `probe`、`gazebo`、`smoke [stageA/stageB/stageC/all]`、`clean` |
| [viz.sh](viz.sh) | `check`、`gzclient`、`rviz-raw`、`rviz-map`、`rviz-nav` |
| [foxglove/](foxglove/README.md) | rosbridge 协议、话题和手写布局 |
| [lint.sh](lint.sh) | Dockerfile RUN、Shell 语法与 Compose 静态检查 |

| 服务 | 用途与配置 |
| --- | --- |
| `roscore` + `slam` | bridge 开发域，master 缺省 `http://roscore:11312`；容器名 `ros1-slam-roscore`、`ros1-slam-dev` |
| `sim` | CPU 无头仿真、Mesa/Xvfb；11313/11345；不请求 NVIDIA GPU |
| `sim-viz` | 宿主 X 桌面仿真；同样发布11313/11345，当前 Compose 显式请求 NVIDIA GPU |
| `slam-host` | host 网络现场模板，须核本机 master/IP 和单项许可 |
| `rosbridge`、`redis` | extras；Redis 非算法依赖。独立 rosbridge 缺省连接开发 `roscore`，不能直接读取 sim 的 loopback ROS 图 |

ROS 服务共用 `ws/build`、`ws/devel` 等命名卷，绑定挂载 `ws/src/scripts/docker/data/opt/note/tooling`。`sim` 与 `sim-viz` 还共用宿主端口，一次只选一套，不得并行重编同一份卷。`.dockerignore` 排除宿主构建物和本机地图，镜像不包含现场数据。

## 二、构建

宿主机以本机仓库为例，异机替换仓根，不复制本机地图或 `.env`：

```bash
cd /home/jack/ysc-code/ros1-slam
# 首次配置才复制；已有 docker/.env 时先核对，勿直接覆盖
cp docker/.env.example docker/.env
bash docker/lint.sh
```

模板当前选 `fishros2/ros:noetic-desktop-full`；Dockerfile 未传参时选 `ros:noetic-ros-base-focal`。两者不是同一构建输入。基础镜像、架构和代理须显式选择；旧机参数不证明当前机器已具备镜像或 GPU。

完整仿真和 RViz 镜像：

```bash
cd /home/jack/ysc-code/ros1-slam/docker
WITH_RVIZ=1 SLIM=0 docker compose --profile sim build sim
```

基础仿真可用 `WITH_RVIZ=1 SLIM=1 docker compose --profile sim build sim`。`SLIM` 仅由 builder 给 **COPY 副本**施加 `CATKIN_IGNORE`，跳过列表以 Dockerfile 的 `SLIM_SKIP` 为准，包括部分定位、SCAN 等；它不是 `catkin_make --pkg` 白名单。需要被跳过的后端须完整构建，不在绑定的第三方源码中手工打忽略标记。

手工构建从仓库根提供上下文，`.env` 不会自动转成 build args：

```bash
cd /home/jack/ysc-code/ros1-slam
docker build -f docker/Dockerfile -t ros1-slam:noetic-sim \
  --build-arg PROFILE=sim --build-arg WITH_RVIZ=1 .
```

`PROFILE=core` 不主动安装 Gazebo，而仓内仿真包需要它；能否全树编译取决于基础镜像与排除面，不能承诺官方 core 镜像直接编译整仓。`-DROS_EDITION=ROS1`、先编译 Livox-SDK2 及消息头预生成顺序由构建件处理。

## 三、启动、环境和日志

Compose 操作均在仓库 `docker/`。显式写服务名，避免 profile 启动同时拉起默认开发服务：

```bash
docker compose --profile sim up -d --no-build sim
docker compose exec sim bash /opt/ros1-slam/docker/sim.sh probe
docker compose ps
docker compose logs --tail=200 sim
```

开发域另行选择 `docker compose up -d roscore slam`。`slam` 与 `sim-viz` 当前都含 NVIDIA device reservation，缺相应驱动/容器工具链时可能起不来；无 GPU 无头运行使用 `sim`。

`docker exec/compose exec` 不重新经过 entrypoint，新 shell 显式加载环境：

```bash
docker compose exec sim bash -lc '
  source /opt/ros1-slam/docker/env.sh
  rospack find faster_lio
  ls -lt /opt/ros1-slam/logs'
```

开发日志用 `docker compose exec slam ...` 或真实容器名 `ros1-slam-dev`，不能用 `docker exec slam ...`。`HOST_UID/HOST_GID` 用于构建用户；当前 Compose 的 `user:` 被注释，仅改 `.env` 不会让运行进程自动成为非 root。

改 C++ 后的增量构建：

```bash
docker compose exec sim bash -lc '
  export CPATH=/opt/ros/noetic/include
  bash /opt/ros1-slam/docker/build.sh'
```

`CPATH` 只在 builder 定义，runtime 不继承。`build.sh` 直接调用 catkin_make，不能替代干净构建的消息头预生成；需干净重建时优先重建镜像，按本机卷维护流程处理缓存。手工调用 `catkin_build.sh` 前还须确认顶层 `CMakeLists.txt` 和 Livox ROS1 `package.xml` 已按 Dockerfile 相同步骤生成。

静态检查和正式门禁分别按 [lint](lint.sh) 与[门禁清单](../scripts/verify/gate_chain.txt)。环境、面清单、历史基线或授权范围缺失不等于算法通过/失败，本轮未复跑门禁。

## 四、仿真操作

### 4.4 无头 smoke 与场地

`probe` 展示 GL、模型、插件和端口，**没有完整 fail-closed 判断**；必须检查缺失项，不能只看退出码。起跑前端口应干净，已运行后有监听并非新的失败。

```bash
# 前台阻塞，运行当前判据载体
docker compose exec sim bash /opt/ros1-slam/docker/sim.sh smoke stageA
```

或只起场地：

```bash
docker compose exec sim bash /opt/ros1-slam/docker/sim.sh gazebo
```

两者都会启动 Gazebo，只选一条。smoke 有效性、RTF 与 PASS/FAIL/INVALID 按原载体及当前授权判断；不为显示或提速改冻结判据、世界物理参数。本轮没有新 smoke 结果。

`sim.sh clean` 按名字匹配 gzserver、rosmaster、rviz、mapping 等进程，**不只清11313域**。仅在确认整个容器相关进程均归本次窗口时使用；实验共存时按本次 PID/启动句柄停止。

### 4.6 桌面观察

`sim-viz` 挂载 `/tmp/.X11-unix`。2026-10-29 起已停用 Xauthority cookie 绑定，以免容器在登录前建目录占住 GDM cookie 路径；不要恢复旧 `/run/user/1000/gdm/Xauthority` 挂载。

主机须具备可用宿主 X 和当前 Compose 所需 NVIDIA 环境：

```bash
# 宿主图形会话；DISPLAY以实际会话为准
xhost +si:localuser:root
cd /home/jack/ysc-code/ros1-slam/docker
# 确认 sim 的本次进程收尾后，再切换容器
docker compose --profile sim stop sim
X_DISPLAY="$DISPLAY" docker compose --profile viz up -d --no-build sim-viz
docker compose exec -e VIZ_HOST_X=1 -e X_DISPLAY="$DISPLAY" sim-viz \
  bash /opt/ros1-slam/docker/viz.sh check
```

在同一个 `sim-viz` 内另开终端运行场地/smoke，以及 `viz.sh gzclient` 或 `viz.sh rviz-raw`。`viz.sh` 宿主 X 模式缺省软件渲染；是否硬件加速看实际 renderer，不因选择了 `sim-viz` 就称“真 GPU”。撤销本次 X 放行用 `xhost -si:localuser:root`。

`rviz-raw` 固定 `rslidar`、看 `/rslidar_points`。`rviz-map` 缺 `/cloud_registered` 时会**自动启动 `m2_mapping.launch`**，不是无副作用旁观器，还会触及 faster-lio 默认 PCD 输出。`rviz-nav` 使用旧 `x2_g6` 命名空间，未必适合 MBF 或3D graph。

浏览器观察见 [Foxglove](foxglove/README.md)。当前 Compose 未发布 VNC 5900；`sim` 的 entrypoint 已可能起 Xvfb :99，不能照旧段再起同号并宣称宿主5900可连。远程 GUI 需另行提供端口、认证和生命周期方案，本文不假定 VNC 已可用。

### 4.6.1 交互式2D导航（MBF）

先退出同域旧场地/smoke：

```bash
docker compose exec sim-viz bash -lc '
  source /opt/ros/noetic/setup.bash
  source /opt/ros1-slam/ws/devel/setup.bash
  export ROS_MASTER_URI=http://127.0.0.1:11313 ROS_IP=127.0.0.1
  export GAZEBO_MASTER_URI=http://127.0.0.1:11345
  roslaunch lite3_gazebo_sim interactive_2d.launch rviz:=true'
```

RViz **2D Nav Goal** 经 `mbf_goal_bridge` 转成 `/move_base_flex/move_base` action。脚本客户端另开终端：

```bash
docker compose exec sim-viz bash -lc '
  source /opt/ros/noetic/setup.bash
  source /opt/ros1-slam/ws/devel/setup.bash
  export ROS_MASTER_URI=http://127.0.0.1:11313 ROS_IP=127.0.0.1
  python3 -B /opt/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim/scripts/sim_pose.py \
    --goal 3.0 0.0 --goal-backend mbf --wait-result-s 60'
```

默认 `sim_pose.py --goal` 面向另一 `move_base` 入口，MBF 必须显式选 `mbf`。本链使用平面地图/costmap，不走 support_map；操作演示不等于正式 A1—A7。

### 4.6.2 交互式3D导航与选面

```bash
docker compose exec sim-viz bash -lc '
  source /opt/ros/noetic/setup.bash
  source /opt/ros1-slam/ws/devel/setup.bash
  export ROS_MASTER_URI=http://127.0.0.1:11537 ROS_IP=127.0.0.1
  export GAZEBO_MASTER_URI=http://127.0.0.1:11538
  cd /opt/ros1-slam
  python3 -B ws/src/90_platform/v8_platform/framework/app_runner.py \
    --app ws/src/90_platform/v8_platform/framework/fixtures/apps/nist_sensor_3d.yaml \
    --execute --interactive --rviz --out /tmp/nav3d_interactive.json'
```

前置：本机 registry 有 app 所需的 `nist_sensor@2`，图/任务/世界与地图匹配。`--interactive` 等待 RViz **Publish Point** 选面；`--rviz` 带 TF、markers 和显示模型。app 无 `--world` 参数，世界在 carrier YAML 选择。

3D与楼梯使用同一 `support_map → support_planner → local_native → cmd3d_mux → gazebo_carrier3d` 技术面。`app_carrier` 是运动学载体，不能证明四足接触或真机爬楼。LIO定位变体还须独立 `/Odometry` 源，app 不替你完成 SLAM。

### 4.7 键盘采集 → 自建支撑图 → 导航

这是仿真 LiDAR + **GT位姿**采集，不使用真机 `scripts/mapping/m_*.sh`。前四步各用一个终端，在 `docker/` 操作。

**终端1：场地**

```bash
docker compose exec sim-viz bash -lc '
  source /opt/ros/noetic/setup.bash
  source /opt/ros1-slam/ws/devel/setup.bash
  export ROS_MASTER_URI=http://127.0.0.1:11313 ROS_IP=127.0.0.1
  export GAZEBO_MASTER_URI=http://127.0.0.1:11345
  roslaunch lite3_gazebo_sim m2_gazebo.launch gui:=true'
```

**终端2：记录器**

```bash
docker compose exec sim-viz bash -lc '
  source /opt/ros/noetic/setup.bash
  source /opt/ros1-slam/ws/devel/setup.bash
  export ROS_MASTER_URI=http://127.0.0.1:11313 ROS_IP=127.0.0.1
  rosrun lite3_gazebo_sim mapping_recorder.py \
    --out /home/ros1-slam/data/maps/mapped_m1.pcd --voxel 0.03'
```

**终端3：显示**

```bash
docker compose exec sim-viz bash -lc '
  source /opt/ros/noetic/setup.bash
  source /opt/ros1-slam/ws/devel/setup.bash
  export ROS_MASTER_URI=http://127.0.0.1:11313 ROS_IP=127.0.0.1
  rviz -d /opt/ros1-slam/docker/rviz_mapping3d.rviz'
```

**终端4：键盘（前台交互）**

```bash
docker compose exec sim-viz bash -lc '
  source /opt/ros/noetic/setup.bash
  source /opt/ros1-slam/ws/devel/setup.bash
  export ROS_MASTER_URI=http://127.0.0.1:11313 ROS_IP=127.0.0.1
  rosrun lite3_gazebo_sim keyboard_teleop.py'
```

`w/s` 前后、`a/d` 转向、`q/e` 横移、空格停。记录器终端 Ctrl+C 落 PCD 和同名 JSON；显示看 `/mapping/cloud`，固定帧 `odom`。输出名是例子，勿覆盖已有本机图。

**离线转换**：在具备工具依赖的环境，从仓库根执行：

```bash
python3 -B ws/src/90_platform/v8_platform/framework/map_from_pcd.py \
  --pcd data/maps/mapped_m1.pcd --map-id mapped_m1 --version 1 \
  --x-min -5 --x-max 8 --y-min -5 --y-max 5 --write-app --register
```

生成本机 `data/maps/generated/mapped_m1_v1.json`、`_task.json`、`_app.yaml`，登记 registry。过滤、世界和出生点参数按采集实际选择，例子不作为现场参数。

收尾前四终端的本次进程后，在 `docker/` 运行：

```bash
docker compose exec sim-viz bash -lc '
  source /opt/ros/noetic/setup.bash
  source /opt/ros1-slam/ws/devel/setup.bash
  export ROS_MASTER_URI=http://127.0.0.1:11537 ROS_IP=127.0.0.1
  export GAZEBO_MASTER_URI=http://127.0.0.1:11538
  cd /opt/ros1-slam
  python3 -B ws/src/90_platform/v8_platform/framework/app_runner.py \
    --app data/maps/generated/mapped_m1_v1_app.yaml \
    --execute --interactive --rviz --out /tmp/mapped_nav.json'
```

### 4.8 NIST数据世界与声明楼梯图

本机导入资产位于 `data/imports/venue_nist_dataset/`、`data/maps/worlds/`，不保证每台 clone 都有。先核世界/模型/registry，缺资产按来源包恢复，不自动重建现场地图。

在已加载环境的容器内启动参数示例：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11313 ROS_IP=127.0.0.1
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
roslaunch lite3_gazebo_sim m2_gazebo.launch gui:=true \
  world:=/opt/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim/worlds/nist_stairs_120_s020x170.world \
  model_path:=/opt/ros1-slam/data/maps/worlds \
  spawn_x:=0 spawn_y:=-2.6 spawn_z:=0.15 spawn_yaw:=1.5708
```

采集同§4.7；输出名要贯穿采集/转换，不能直接引用未生成的 `mapped_nist_gt1.pcd`。

[map_from_pcd.py](../ws/src/90_platform/v8_platform/framework/map_from_pcd.py) 支持 `--nist-profile/--world/--model-path`。[inflate_support_map.py](../ws/src/90_platform/v8_platform/framework/inflate_support_map.py) 只写外扩地图并可登记 registry，**不生成 `_app.yaml/_task.json`**。历史声明楼梯 app 需连同本机数据包恢复，核地图ID/版本/世界；不能只跑 inflate 就调用假定存在的 app。

[R1120](../note/进度/v8/M5/26-10-29/R1120.md)地面采集没有完整覆盖楼梯；[R1121](../note/进度/v8/M5/26-10-29/R1121.md)是声明支撑面外扩后的运动学导航。两者地图来源不同，均非物理四足/formal结论。

### 4.9—4.13 SLAM演示与AWS Hospital探针

| 记录 | 内容及复现入口 |
| --- | --- |
| [R1122](../note/进度/v8/M5/26-11-03/R1122.md) | m1_world_v2 + faster-lio对GT；[原命令](../note/进度/v8/M5/26-11-03/evidence/R1122/command.txt) |
| [R1123](../note/进度/v8/M5/26-11-03/R1123.md) | 受控两层survey，含yaw跳变负例；[原命令](../note/进度/v8/M5/26-11-03/evidence/R1123/command.txt) |
| [R1124](../note/进度/v8/M5/26-11-03/R1124.md) | AWS ros1世界和Fuel导入探针；[证据](../note/进度/v8/M5/26-11-03/evidence/R1124/) |
| [R1125](../note/进度/v8/M5/26-11-03/R1125.md) | 大厅GT teleport小环，后发现贴物；[原命令](../note/进度/v8/M5/26-11-03/evidence/R1125/command.txt) |
| [R1126](../note/进度/v8/M5/26-11-03/R1126.md) | 筛选走廊三种子，绝对z漂移仍在；[原命令](../note/进度/v8/M5/26-11-03/evidence/R1126/command.txt) |

原命令的容器/端口/输出只对应当批窗口，复现前核本机资源和许可。faster-lio 仍写默认 `data/maps/lite3.pcd`，先保护已有本机图，确认本次落盘与退净后再恢复，不在两个SLAM实验间共享这个输出。AWS资产留在本机忽略目录；电梯为静态模型，楼梯/多层几何不代表机器人通行。以上均为探针或scoped SLAM，非formal/真机结论。

## 五、排障与数据保护

- 拉取失败先区分镜像元数据、daemon拉取和构建apt下载，不沿用旧机网络读数。Compose构建代理是 `BUILD_HTTP_PROXY/BUILD_HTTPS_PROXY/BUILD_NO_PROXY`，不是原指南的 `HTTP_PROXY` 三项。
- 构建容器的 `127.0.0.1` 指自身；Dockerfile对loopback代理有拒绝检查，使用构建环境可达地址。daemon代理属系统配置，重启Docker会影响在用服务，不作为日常构建步骤。
- `.env`只用于Compose，直接 `docker build` 要显式传 `ROS_BASE_IMAGE/ROS_BASE_TAG`。更换基础镜像后核架构/Noetic/Gazebo与依赖，旧缓存不等于干净重建证据。
- 缺消息头按 `catkin_build.sh` 三步处理，不改第三方CMake或在源码打忽略标记隐藏失败。
- 无头用Mesa/Xvfb，桌面看实际renderer；不恢复已停用cookie挂载。
- ROS节点地址和动态端口须互访，只发布master不等于跨容器/跨机可用。独立rosbridge不能读取sim的loopback图。
- 容器真实路径 `/opt/ros1-slam/ws`、兼容软链 `/home/ros1-slam` 会影响隔离检查读数；不得由本文私改判据或预设“期望0”。
- 停止优先Ctrl+C/本次句柄；停本次容器用 `docker compose stop <服务名>`。`down`影响同项目其他服务，`down -v`还删除共享构建/日志等卷，不是日常清场命令。

历史数值与验证输出保留在note原件，本指南只维护当前命令和边界。本机磁盘、端口、镜像、地图文件和运行能力均须实际执行时现查。
