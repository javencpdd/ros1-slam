# ros1-slam-simulation · P2 仿真能力子项目

> 独立仓（2026-11-14 自 [ros1-slam](https://local/ros1-slam) `2d0c32a5` 的 `ws/src/95_simulation/` 拆出）。
> 自持运动学载体/真值/支撑格与 17 个 Gazebo 世界；**不持有**平台装配、BT、任务台。

## 布局

```
ws/src/                 # 独立 catkin 工作区源码（7 个包）
  v8_simulation/        # 自持 carrier/truth/support_cells + sim_backends.yaml
  lite3_gazebo_sim/     # 世界/场景/launch/verify
  lite3_description/ local_sensing/ map_generator/ mockamap/
  Mid360_simulation_plugin/livox_laser_simulation/
ws/{build,devel,install}  # 本仓独立编译产物（不入库）
config/ tools/          # 原 95_simulation/config、tools
```

## 依赖顺序

1. ROS1 Noetic（容器 `ros1-slam:noetic-sim` 已含）。
2. **P1 安装空间**（只为 `livox_ros_driver2` 等驱动/消息的编译期发现）：先构建 P1，或把 `P1_DEVEL` 指向其 `ws/devel/setup.bash`。
3. 本仓 `ws` 独立 `catkin_make`，产物留在本仓 `ws/install`。

`lite3_gazebo_sim` 对 `lite3_drivers`/`faster_lio`/`lite3_slam` 只有 `exec_depend`：**构建不需要 P1 源码**，运行期由 P1 安装空间提供。

## 构建 / 测试 / 发布

```bash
# 容器内（推荐）：源码挂到 /opt/ros1-slam/sim_ws
docker exec ros1-slam-sim bash -lc 'cd /opt/ros1-slam/sim_ws && source /opt/ros/noetic/setup.bash && source /opt/ros1-slam/ws/devel/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release'

make build      # 需已 source Noetic；P1_DEVEL=... 可选
make test       # v8_simulation 独立单测
make release    # install + dist/ros1-slam-simulation-<version>.tar.gz + manifest.json
```

## 与 P1 的契约

- P1 manifest 以 `launch.package: v8_simulation` / `lite3_gazebo_sim` 装配本仓包，包名不变。
- 容器把本仓 `ws` 挂到 `/opt/ros1-slam/sim_ws`，入口 source `sim_ws/devel/setup.bash`；`roslaunch <pkg>` 经 `ROS_PACKAGE_PATH` 解析。
- 场景脚本 `scenarios/two_floor_nav/scene.py` 的 `PLATFORM` 走 `V8_PLATFORM_ROOT`（集成期依赖 P1 平台源码），不硬编码单仓路径。
- 本仓**不 import** P1 包源码；`grep -rn "import v8_platform\|from v8_platform"` 为空。

## 来源

- 源仓 `ros1-slam` HEAD `2d0c32a5`，路径 `ws/src/95_simulation/`。
- 历史保留在源仓；本仓为新历史，首提交记录来源。
