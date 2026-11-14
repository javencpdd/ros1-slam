# ws/ · 源码与配置入口

> 最新更新时间：2026-11-06 15:17:09 CST。现行路线：[计划书 v8.1](../note/计划书v8.1/README.md)；实际架构：[项目架构与目录](../note/架构/项目架构与目录.md)；进度与能力边界：[进度入口](../note/进度/README.md)。

本目录是 catkin 工作区。`src/` 放源码、配置与源码测试；`build/`、`devel/`、`install/` 与 `.profiles/` 是忽略的编译产物。地图和运行输出在仓根 `data/maps/`、`logs/`；进度、验收和证据在 `note/`。

## 源码分层

| 目录 | 作用 |
| --- | --- |
| `src/00_drivers/` | 驱动壳、雷达驱动和 QNX 传感/底盘桥 |
| `src/10_slam/` | faster_lio 建图、pcd2grid 与包装配置 |
| `src/20_localization/` | hdl/NDT/GICP 定位后端与包装 |
| `src/30_3rd/` | SCAN 上游规划包，保持其基线 |
| `src/30_perception/` | 自研高程/感知与厂商、kindr 依赖 |
| `src/40_navigation/` | MBF 导航装配、SCAN 接缝、自研 costmap 层 |
| `src/50_task/` | 传统任务状态机与原语 |
| `src/60_safety/` | M6 速度仲裁、超时、限速与 hold |
| `src/70_bringup/` | 传统 mode 顶层 launch |
| `src/90_supervisor/` | 心跳监管与 hold，不直接输出速度 |
| `src/90_platform/` | v8 契约/组合/装配/算法适配、NDT monitor、消息接口 |
| `src/95_simulation/` | **2026-11-14 迁出**：见 [ros1-slam-simulation](../../ros1-slam-simulation/README.md)（独立 `ws/` 工作区） |

> 物理分仓：仿真源码在 `../ros1-slam-simulation/`，控制台在 `../ros1-slam-console/`；本仓不再持有这两棵源码树。

## 按工作查入口

| 要做什么 | 入口 |
| --- | --- |
| 显式目标构建/只读依赖计划 | [构建说明](../tooling/build/README.md)；`lite3` 与 `x86_sim` 分开输出，`m20` 明确拒绝 |
| 传统建图/定位/导航组合 | [`lite3_bringup/launch/`](src/70_bringup/lite3_bringup/launch/)中的 `mode_mapping` / `mode_localize` / `mode_navigation` |
| v8 app、task、地图与 graph 绑定 | [`v8_platform/framework/nodes/app_runner.py`](src/90_platform/v8_platform/framework/nodes/app_runner.py)与[framework 说明](src/90_platform/v8_platform/framework/README.md) |
| v3执行域、机器人身份与运行前拒绝 | [`execution.py`](src/90_platform/v8_platform/framework/execution.py)和[框架说明](src/90_platform/v8_platform/framework/README.md)；声明/静态检查不作硬件许可 |
| Lite3 本机 graph/应用模板 | [`v8_platform/configs/lite3/`](src/90_platform/v8_platform/configs/lite3/)；各机覆盖使用 `*.local.app.yaml` |
| 位姿新鲜度与map→机体航向平面 | [`pose_safety.py`](src/90_platform/v8_platform/common/pose_safety.py)、[`local ros_node.py`](src/90_platform/v8_platform/local_planner/ros_node.py)、[`Lite3 carrier`](src/90_platform/v8_platform/framework/nodes/lite3_carrier_node.py)；物理停止链另验 |
| v8 装配/算法/协议阅读地图 | [项目架构与目录](../note/架构/项目架构与目录.md) |
| 仿真键盘建图和可视化命令 | [Docker 说明](../docker/README.md)；记录器 [`mapping_recorder.py`](../../ros1-slam-simulation/ws/src/lite3_gazebo_sim/scripts/mapping_recorder.py)与转换器 [`map_from_pcd.py`](src/90_platform/v8_platform/framework/tools/map_from_pcd.py) |
| Gazebo world/launch | [`lite3_gazebo_sim/`](../../ros1-slam-simulation/ws/src/lite3_gazebo_sim/) |
| MuJoCo 力矩/接触与阶面障碍 | [`dynamics/README.md`](../../ros1-slam-simulation/ws/src/lite3_gazebo_sim/dynamics/README.md) |
| 安全源表与最后速度仲裁 | [`cmd_vel_mux.yaml`](src/60_safety/lite3_safety/config/cmd_vel_mux.yaml)、[`safe_cmd_vel_mux.py`](src/60_safety/lite3_safety/scripts/safe_cmd_vel_mux.py) |
| 组件单测、场景与仿真回归 | [`v8_platform/`](src/90_platform/v8_platform/)各模块的 `test_*.py`、[`regression/`](src/90_platform/v8_platform/regression/) |
| 正式门禁 | [`scripts/verify/gate_chain.txt`](../scripts/verify/gate_chain.txt)及[复验作业单](../note/05_复验作业单.md) |

## 自研包与接口索引

| 包 | 入口 |
| --- | --- |
| `lite3_drivers` | [README](src/00_drivers/lite3_drivers/README.md) |
| `lite3_slam` | [README](src/10_slam/lite3_slam/README.md) |
| `lite3_localization` | [README](src/20_localization/lite3_localization/README.md) |
| `lite3_perception` | [README](src/30_perception/lite3_perception/README.md) |
| `lite3_navigation` | [README](src/40_navigation/lite3_navigation/README.md) |
| `lite3_scan` | [README](src/40_navigation/lite3_scan/README.md) |
| `lite3_costmap_layers` | [README](src/40_navigation/lite3_costmap_layers/README.md) |
| `lite3_task` | [README](src/50_task/lite3_task/README.md) |
| `lite3_safety` | [README](src/60_safety/lite3_safety/README.md) |
| `lite3_bringup` | [README](src/70_bringup/lite3_bringup/README.md) |
| `lite3_supervisor` | [README](src/90_supervisor/lite3_supervisor/README.md) |
| `lite3_gazebo_sim` | [README](../../ros1-slam-simulation/ws/src/lite3_gazebo_sim/README.md) |
| `v8_simulation` | [仿真薄入口与依赖](../../ros1-slam-simulation/ws/src/v8_simulation/README.md) |
| `lite3_description` | [README](../../ros1-slam-simulation/ws/src/lite3_description/README.md) |
| `v8_platform` | [README](src/90_platform/v8_platform/README.md) |
| `v8_ndt_odom` | [README](src/90_platform/v8_ndt_odom/README.md) |
| `v8_msgs` | [消息定义与构建入口](src/90_platform/v8_msgs/)（当前没有独立 README） |

## 工作边界

- 第三方副本保持原件；配置与接缝优先改自研包装。历史许可例外和隔离规则见 [硬规则](../note/00_README.md)。
- 真机运动桥 `ros2qnx` 仅属于获授权的真机模式；`mode_navigation.launch` 的 `enable_motion` 默认 false。文档导航不构成现场运行授权。
- 传统/真机 M6 mux 与 v8 仿真 `cmd3d_mux_node.py` 是不同适配器，同一运行图不得争写 `/cmd_vel`。SUP 只给 hold。
- 控制台已迁至 [ros1-slam-console](../../ros1-slam-console/README.md)（页面在 `console/ui`）；`data/console` 是 host-only 测试面，不进入 `lite3`/M20 target 或机器人镜像；`web/` 官网只提供文档面。源码/生成物边界见[项目架构 §7](../note/架构/项目架构与目录.md)。
- 仿真 Gazebo carrier、动态平面方盒和 MuJoCo 四足实验各有范围；运行结果不能跨载体、地图或策略移用。
- `framework/fixtures/` 包含 app 实际读取的 manifest/graph；不按名称把它当临时目录删除。旧证据、冻结判据及当前检查输入按引用与实际消费者核对后清理。
- ROS 检查须使用当前机器的 Noetic 与已构建工作区环境；宿主缺环境时使用适用容器，不能把前置缺失当算法结论。

## 统一工程约束

工程标准真源：`note/架构/工程标准.md`。[工程标准](../note/架构/工程标准.md)统一阶段验证落账后本地commit、用户决定push、Lite3/M20真机不构建仿真、x86仿真与训练开发，以及算法标准接口。原生与Docker入口共用显式目标构建器和清单闭包检查，目标环境编译/运行尚未验证；受支持框架已增加执行域/机器人身份校验；已增加速度坐标与源时间/单调时钟守卫，真实目标环境、接线与完整热替换仍待闭合；M20真实适配和可复现训练器也尚未提供，详见[审查记录](../note/进度/00_架构与工程标准审查_2026-11-05.md)。规范统一不等于这些能力已实现。
