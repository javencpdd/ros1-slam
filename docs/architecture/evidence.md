# ros1-slam 当前态架构 · 证据索引（docs 版）

- 建模日期：2026-10-30（只读核对；未运行新批次）
- 范围：仓库内可执行代码与配置；不含真机现场状态
- 图源：`diagrams/*.dot`；视图：`system-context.dot`（L1）、`system-runtime.dot`（L2/L3 分层）、`command-path.dot`（命令/反馈链）
- 置信度：high=代码/配置直证；medium=多源部分一致；low=命名/结构推断；unknown=待核

## 节点与证据

| id | 节点 | 状态 | 置信 | sourceRefs |
| --- | --- | --- | --- | --- |
| drv_lidar | 雷达驱动（livox/lslidar） | current/runtime | high | `ws/README.md:17-18` |
| imu | qnx2ros → `/imu/data` | current/runtime | high | `ws/README.md:17`；`note/00_README.md:27` |
| flio | faster-lio | current/runtime | high | `ws/README.md:115` |
| hdl | hdl_localization（备用） | current/config | high | `ws/README.md:117` |
| costmap | costmap_2d（MBF 层） | current/runtime | high | `ws/README.md:41`；`lite3_navigation/config/costmap_common.yaml` |
| mbf | MBF：GlobalPlanner + TEB/DWA | current/runtime | high | `lite3_navigation/config/mbf_navigation.yaml:5-19` |
| scan | SCAN plan_manage | current/runtime(sim) | high | `ws/README.md:118` |
| task | lite3_task | current/runtime | high | `ws/README.md:23` |
| mux | safe_cmd_vel_mux（七源/0.5 s/hold） | current/runtime | high | `lite3_safety/config/cmd_vel_mux.yaml:4-21`；`lite3_safety/launch/60_safety.launch:4` |
| supv | lite3_supervisor | current/runtime | high | `ws/README.md:26`；`cmd_vel_mux.yaml:6` |
| qnx | ros2qnx → QNX:43893 | current/L2 | high（端口/订阅）；medium（完整帧语义） | `message_transformer/src/ros2qnx.cpp:44,112` |
| supmap | 支撑图 JSON（离线） | current/data | high | `data/maps/nist_stairs_120_s020x170_support.json`；`R944` |
| a5 | A5 局部规划（ROS 节点） | experimental | high | `local_planner/ros_node.py:241-303`；`regression/a5_v8x1_runtime.py:164-174` |
| mjb | MuJoCo 桥（270 策略+物理） | experimental | high | `dynamics/a5_v8x1_bridge.py:177-191,264,281` |
| carrier | Gazebo 3D 运动学载体 | experimental | high | `scripts/gazebo_3d_carrier.py:2-7,43-46` |
| planar | Gazebo 平面驱动 | current/runtime(sim) | medium（配置在册，历史跑批） | `ws/README.md:35-36`；`mode_nav_sim.launch` |
| contracts | v8 contracts | design/unit | high | `v8_platform/README.md:10` |
| registry | v8 registry | design/unit | high | `registry/README.md:1-5` |
| composer | v8 composer | design/unit | high | `composer/README.md:1-5` |
| orch | v8 orchestrator | design/unit+SimulatedRunner | high | `orchestrator/README.md:1-8` |
| v8r | v8 representation | design/unit | high | `v8_platform/README.md:15` |
| v8p | v8 planner（有界 A*） | design/unit | high | `v8_platform/README.md:16` |
| val | v8 validator | design/unit | high | `v8_platform/README.md:17` |
| bnd | boundary S1（帧/速率桥/统计） | current/loopback-only | high | `boundary/protocol.py:11-15`；`boundary/rate_bridge.py:26-72`；`boundary/udp_link.py:14-88` |
| bndros | boundary ROS 节点 S2 | planned | high | `R954.md:64-68`（S2 待开） |
| robot | Lite3 真机（QNX） | L2/未授权 | high | `note/00_README.md:18,23-29`；`note/11_仿真到实机参数迁移表.md:106` |

## 关键边与证据

| 边 | 关系 | 置信 | sourceRefs |
| --- | --- | --- | --- |
| 雷达/IMU → faster-lio | 订阅 points/imu | high | `ws/README.md:115` |
| faster-lio → costmap | odom/TF | high | `ws/README.md:115,119` |
| costmap → MBF | 代价图输入 | high | `mbf_navigation.yaml:5-19` |
| MBF → mux | `/cmd_vel_raw` | high | `mode_nav_stub.launch:12-13`；`cmd_vel_mux.yaml:19` |
| SCAN → mux | `/scan_planner/cmd_vel` | high | `cmd_vel_mux.yaml:21` |
| task/supervisor → mux | task_cmd_vel / hold | high | `cmd_vel_mux.yaml:6,18` |
| mux → Gazebo 平面驱动 | `/cmd_vel`（仿真） | medium | `cmd_vel_mux.yaml:4`；仿真 bringup |
| mux → ros2qnx | `/cmd_vel`（真机 mode） | high | `cmd_vel_mux.yaml:2`；`ros2qnx.cpp:112` |
| ros2qnx → QNX | UDP 43893 | high（端口）/medium（帧） | `ros2qnx.cpp:44` |
| 支撑图 → A5 | map json | high | `a5_v8x1_runtime.py:172-174` |
| A5 → MuJoCo 桥 | cmd/trajectory/status | high | `a5_v8x1_bridge.py:177-179` |
| MuJoCo 桥 → A5 | pose/body/applied | high | `a5_v8x1_bridge.py:183-186`；`a5_v8x1_runtime.py:147-152` |
| A5 → Gazebo 3D 载体 | cmd；pose 回传 | high | `gazebo_3d_carrier.py:43-46` |
| mux → boundary | 规划接线 | unknown（未实现） | `R954.md:64-68`；全仓 grep 无引用 |
| boundary → 虚拟机器人 | UDP 定拍 | planned | `R954.md:64-68` |
| contracts→registry→composer→orchestrator | 设计链 | high | 各 README；`orchestrator/README.md:1-8` |
| v8 planner/validator → 运行链 | 未接入 | high（否定证据） | `v8_platform/README.md:40-43` |

## 假设与未知

- **A（medium）**：ROS 2D 导航链“已接通”依据配置与历史跑批记录；本会话未重新启动 Gazebo/MBF 做活体复跑。
- **B（medium）**：ros2qnx 的帧结构只读到 `cmd_code`/`data` 片段，完整 QNX 协议未逐字段核对；真机链路为 L2。
- **C（high）**：boundary 当前只在本机回环验证，未被 mux、A5、MuJoCo、Gazebo 任一运行件 import。
- **D（unknown）**：真机上 `/cmd_vel` 的实际发布者是否与仿真一致（mux 唯一写者）、以及 QNX 侧看门狗行为，需真机/台架验证。
- **E（low）**：`planar` 的“已接通”是按配置与历史证据归入；当前容器未复跑。
- 非业务应用：未按 Business Application Fitness 建模。
