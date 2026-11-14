# REPOS · 三仓物理拆分证据（2026-11-14）

> 任务：`T-20261114-02`（qoder）。目标：仿真/可视化成为独立子项目——物理分仓、各自独立工作区、独立发布物。
> 本目录只放文档与数据，不含可执行代码；发布物 manifest 原样复制。

## 1. 三仓

| 仓 | 路径 | HEAD | 工作区 | 发布物 |
| --- | --- | --- | --- | --- |
| P1 `ros1-slam` | `/home/jack/ysc-code/ros1-slam` | `29eb3693`（证据提交前） | `ws/`（catkin） | `target_workspace.py` 目标空间 |
| P2 `ros1-slam-simulation` | `/home/jack/ysc-code/ros1-slam-simulation` | `1314742` | `ws/`（独立 catkin） | `dist/ros1-slam-simulation-0.1.0.tar.gz` |
| P3 `ros1-slam-console` | `/home/jack/ysc-code/ros1-slam-console` | `52ac475` | `console/`（host-only） | `dist/ros1-slam-console-0.1.0.tar.gz` |

来源：P2/P3 首提交均记录自 `ros1-slam@2d0c32a5`；P1 历史与 `note/` 的 code_ref 未改写。三仓均**未 push**。

## 2. 发布物

| 仓 | 文件 | 字节 | sha256 | git head |
| --- | --- | --- | --- | --- |
| P2 | `ros1-slam-simulation-0.1.0.tar.gz` | 2,413,786 | `ae926037def19fdd0934d5bb2090ae7b010ef552151b57dcb2f4c5b99aa3fa56` | `1314742` |
| P3 | `ros1-slam-console-0.1.0.tar.gz` | 541,032 | `4c0b69c5e49d39dd81d2aba27aafab3b66831fadc8538e8c782ebcae3a6411a4` | `52ac475` |

> P2 manifest 的 `sha256` 字段为权威；本表只列读数，避免手抄漂移。manifest 见同目录 `*.manifest.json`。
> P2 包列表 7 个：`v8_simulation/lite3_gazebo_sim/lite3_description/local_sensing/map_generator/mockamap/livox_laser_simulation`；install 5.7 MB，含 `worlds/launch/scenarios/config/urdf/dynamics` 与 `faster_lio_sim.yaml`。
> P3 包含 `console/` 全量 UI、生成器、契约测试与 `paths.py`。

## 3. 验证读数

| 项 | 命令/位置 | 读数 |
| --- | --- | --- |
| P2 独立构建 | 容器内 `catkin_make`（`sim_ws`） | `BUILD_RC=0`，100% built |
| P2 独立安装 | 容器内 `catkin_make install` | `INSTALL_RC=0`，install 5.7 MB |
| P2 单测 | `python3 -m unittest discover -s ws/src/v8_simulation/test` | 15/15 OK |
| P3 全测 | `make test` | RC=0：voxel 5、agent 17、registry 4、scenes 2、BT 配置、mission events 全过 |
| P3 运行 | `console/start_agent.sh` | PID 从 P3 仓启动；`/api/health` `{"ok":true,"stacks":12}` |
| P1 解耦守卫 | `verify/test_decoupling.py` | 9/9 OK（含物理分仓断言） |
| P1 sim 后端 | `framework/tests/test_sim_backends.py` | 5/5 OK |
| P1 target_workspace | `scripts/build/test_target_workspace.py` | 27/27 OK；x86_sim 34 包（含 3 个 P2 包） |
| P1 framework | `unittest discover framework/tests` | **296 OK（10 skip）**，RC=0 |
| 容器发现 | `ros1-slam-sim-viz` 内 `rospack find` | `v8_simulation`→`/opt/ros1-slam/sim_ws/src/v8_simulation`；`lite3_gazebo_sim`→`.../sim_ws/src/lite3_gazebo_sim` |
| 容器 launch 解析 | `roslaunch --nodes lite3_gazebo_sim m1_gazebo.launch` | 5 节点（gazebo/robot_state_publisher/spawn/map_to_odom/tf_frame_assert） |
| 应用装配 | `upstairs_bt.yaml` 经 `V8_SIM_ROOT` 映射 | graph/task 均存在，加载成功 |

## 4. 契约

- 容器：P2 `ws`→`/opt/ros1-slam/sim_ws`；P2 仓根→`/opt/ros1-slam/sim_repo`（只读）；入口 source `sim_ws/devel/setup.bash`。
- 应用：P2 app/后端声明统一 `sim_ws/src/...`；`app_config._path`/`map_registry.load_map` 在宿主用 `V8_SIM_ROOT` 映射为 `<P2>/ws/src/...`。
- P3：`V8_REPO_ROOT`/`V8_SIM_ROOT`/`V8_CONTAINER_ROOT`；只读、无 ROS 写入。
- x86_sim：`target_workspace.py` 默认并入同级 `ros1-slam-simulation/ws/src`（`--extra-source` 可覆盖）；机器人目标不并入。

## 5. 限定 / 未做

- **未重跑完整 scoped BT Gazebo 窗口**：拆分后未启动 new Gazebo 端到端；装配契约由 `roslaunch --nodes`、应用装配加载、P2 单测与 P1 framework 覆盖。建议下个作业窗补 `two_floor_nav` 等价窗口。
- `sim` 与 `sim-viz` 都声明 11313/11345 并共用 devel 卷，互斥；当前运行 `sim-viz`（控制台栈所在），`sim` 保留 `Created`。既有端口重叠，非本次引入。
- 真机/M20 未触碰；`real_robot` 仍只 lite3，L2 未授权。
- P3 无 catkin 工作区（无 ROS 包），其“独立工作区”为自有 host-only 仓根；若要求形式上的 catkin 包需另立目标。
