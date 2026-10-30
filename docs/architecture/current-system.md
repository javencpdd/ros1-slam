# ros1-slam 当前态系统架构

> 日期：2026-10-30 · 只读建模 · 事实权威仍为代码与 `note/`
> 状态标签：**已接通运行 / 仿真实验 / 设计层（仅单测）/ 待接 / L2 未授权**

## 一句话结构

系统是**两个平面 + 一层治理 + 一个新边界**：

1. **ROS1 2D/2.5D 导航平面（已接通）**：驱动 → faster-lio → costmap → MBF(GlobalPlanner/TEB/DWA) 或 SCAN → `/cmd_vel_raw` / `/scan_planner/cmd_vel` → **safe_cmd_vel_mux** → `/cmd_vel` → Gazebo 平面驱动；真机段经 `ros2qnx` → QNX 43893（L2）。
2. **v8 三维/爬楼平面（仿真实验）**：支撑图 JSON → A5 局部规划 → MuJoCo 桥（270 策略＋接触物理）或 Gazebo 3D 运动学载体 → 位姿/状态回传 A5。
3. **v8 平台治理层（设计＋单测）**：contracts → registry → composer → orchestrator；另有 planner/validator/representation，**未接入运行链**。
4. **机器人边界（S1 回环完成，S2/S3 待接）**：33B UDP 帧＋速率桥＋健康统计，只在本机回环验证；未从 mux 取命令，也无上行遥测。

## 分层与职责

| 层 | 内容 | 当前状态 |
| --- | --- | --- |
| ① 传感/驱动 | 雷达驱动（livox/lslidar）、qnx2ros IMU、sim_adapter | 已接通运行 / 仿真 |
| ② 定位/建图 | faster-lio（主）、hdl_localization（备用） | 已接通运行 / 配置在位 |
| ③ 环境表征 | costmap_2d、离线支撑图 JSON、v8 representation（设计） | 运行 / 离线 / 设计 |
| ④ 规划 | MBF(GlobalPlanner+TEB/DWA)、SCAN、A5 局部规划、v8 A*（设计） | 运行 / 运行(仿真) / 实验 / 单测 |
| ⑤ 任务/仲裁 | lite3_task、supervisor、mux（七源/0.5 s/hold）、v8 validator（设计） | 运行 / 单测 |
| ⑥ 执行与边界 | Gazebo 平面驱动、ros2qnx（L2）、boundary S1/S2、Gazebo 3D 载体、MuJoCo 桥 | 运行 / L2 / 回环 / 实验 |

## 四条命令/反馈链

| 链 | 路径 | 状态 |
| --- | --- | --- |
| A · ROS 2D/2.5D 导航 | 雷达/IMU → faster-lio → costmap → MBF → `/cmd_vel_raw` → mux → `/cmd_vel` → Gazebo 平面驱动；真机段 mux → ros2qnx → QNX | 已接通；真机段 L2 |
| B · SCAN | faster-lio → `/cloud_registered` → SCAN plan_manage → `/scan_planner/cmd_vel` → mux | 已接通（仿真） |
| C · v8 爬楼 pilot | 支撑图 JSON → A5 → MuJoCo 桥或 Gazebo 3D 载体 → 位姿回传 | 仿真实验（R952/R953 合计 38/50） |
| D · 统一边界 | mux → boundary → UDP 50–500 Hz → 虚拟机器人 → 遥测 → 上层 | S1 回环完成；S2/S3 未接 |

## 关键接缝

1. **mux → boundary 未接**：现有 `/cmd_vel` 是事件驱动；50–500 Hz 定拍 UDP 只在 127.0.0.1 回环验证过。
2. **boundary 无上行**：没有遥测帧与状态回传，FAULT/断链安全状态机尚未落地；当前只做零速保持。
3. **v8 治理层与运行层断开**：contracts/registry/composer/orchestrator 仅经 SimulatedRunner 验证；planner/validator/representation 未接 A5/MBF/SCAN。
4. **两类仿真不等价**：MuJoCo 有接触动力学但只接 A5 pilot；Gazebo 3D 载体是运动学 `SetModelState`，不能证明爬楼物理可行。
5. **真机链路仍是厂商 `ros2qnx`**：逐条 `sendto` QNX 43893（事件驱动），换定拍链路属 L2。

## 当前态证据（摘要）

- mux 七源、0.5 s 超时、hold：`ws/src/60_safety/lite3_safety/config/cmd_vel_mux.yaml:4-21`。
- MBF 名单：`planners` GlobalPlanner/NavfnROS、`controllers` TEB/DWA：`ws/src/40_navigation/lite3_navigation/config/mbf_navigation.yaml:5-19`。
- bringup 同时 include mbf 与 safety：`ws/src/70_bringup/lite3_bringup/launch/mode_nav_stub.launch:54-58`、`mode_nav_sim.launch`。
- A5→MuJoCo 接线：`ws/src/90_platform/v8_platform/regression/a5_v8x1_runtime.py:164-174`、`ws/src/95_simulation/lite3_gazebo_sim/dynamics/a5_v8x1_bridge.py:177-191`。
- Gazebo 3D 载体：`ws/src/95_simulation/lite3_gazebo_sim/scripts/gazebo_3d_carrier.py:2-7,43-46`。
- boundary 未被运行件引用：全仓 grep 无 import；S1 证据见 `note/进度/v8/M5/26-10-29/R954.md`。
- v8 未接运行：`ws/src/90_platform/v8_platform/README.md:40-43`。

逐条 sourceRefs 与置信度见 [evidence.md](evidence.md)。

## 未知与边界

- MBF/SCAN 全链在本机是否可一键复跑（本次只读核对，未活体复跑）。
- `ros2qnx` 完整帧语义与 QNX 看门狗行为；真机 `/cmd_vel` 是否确为 mux 唯一写者。
- 本模型不改变验收：A5 `PARTIAL`，A1—A7 `NOT_VALIDATED`，V8-X2 属 L2。
