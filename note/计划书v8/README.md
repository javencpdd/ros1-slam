# 计划书 v8：可组合的 2D/3D 导航平台

> 最新更新时间：2026-11-06 04:37:58 CST。工程标准真源：`note/架构/工程标准.md`。本轮核对规划入口、实际源码和当前排程，未产生新的算法或真机验收。

v8 在 ROS1/Noetic 基础上，以组件契约、app/graph 配置和运行装配组合建图、定位、地图表征、全局/局部规划及安全仲裁。当前结构和代码阅读顺序见[项目架构与目录](../架构/项目架构与目录.md)。

**当前安排**：按[30号补丁](30_目标隔离与运行域契约.md)继续目标隔离、运行模式及坐标/时间安全整改，代码与离线验证分别提交；版本成果报告、任务交接和历史批次审计按[29号补丁](29_版本成果总览与记录职责.md)维护。当前执行状态以[23号队列](23_v8任务队列.md)及 `.ai` 任务卡为准；旧算法、地图准备和真机验证仍保持取消，不从历史“下一步”自动恢复。

## 规划文件

| 文件 | 内容 |
| --- | --- |
| [21 中心计划书](21_ros1-slam中心计划书-v8.md) | 路线、保留范围与阶段目标 |
| [22 组件与组合契约](22_组件契约与组合架构.md) | 角色、标签、接口与拒绝规则 |
| [23 执行队列](23_v8任务队列.md) | 依赖、状态、交付物；当前排程见最新接续段 |
| [24 仿真验收与迁移](24_仿真验收与迁移方案.md) | 历史失败、有效输入、验收与迁移边界 |
| [25 调度作业单](25_v8自动调度作业单.md) | 选件、开窗、落账与停手规则 |
| [26 支撑图障碍与阶面验证](26_支撑图障碍与阶面验证增补.md) | 全局障碍输入、阶面初始化和相关验证范围 |
| [27 框架改造补丁](27_框架改造补丁-v0.1.md) | YAML 图、运行装配、MapDelta、机器人/验证配置 |
| [28 工程标准统一补丁](28_工程标准统一补丁.md) | 阶段commit、真机/仿真分工、稳定接口与调度；正文见[工程标准](../架构/工程标准.md) |
| [29 版本成果与记录职责](29_版本成果总览与记录职责.md) | 总览版本倒序、成果对比、任务/证据/审计索引各自职责 |
| [30 目标隔离与运行域契约](30_目标隔离与运行域契约.md) | 显式构建闭包、运行域/机器人声明、v3迁移与安全接续；目标环境另验 |

规划不是验收结论。当前能力仍为 **A2=PASS*，A1/A3/A4/A5/A6/A7=PARTIAL**，formal 和真机未签署；设计层收口、仿真运行证据、声明地图、GT 建图和物理执行各有范围。详细读数及更正见[进度 v8](../进度/v8/README.md)与[版本成果总览](../进度/00_总览.md)。

## 交付物在哪里

可执行代码已迁到 [`ws/src/90_platform/v8_platform/`](../../ws/src/90_platform/v8_platform/README.md)；本目录保存规划和增补，不再放代码副本。

| 源码入口 | 职责 |
| --- | --- |
| [contracts](../../ws/src/90_platform/v8_platform/contracts/README.md) | schema、校验与拒绝语义 |
| [registry](../../ws/src/90_platform/v8_platform/registry/README.md)、[composer](../../ws/src/90_platform/v8_platform/composer/README.md) | 组件注册、组合兼容与确定性锁 |
| [orchestrator](../../ws/src/90_platform/v8_platform/orchestrator/README.md) | 任务状态机、就绪和停止链 |
| [adapters](../../ws/src/90_platform/v8_platform/adapters/README.md)、[boundary](../../ws/src/90_platform/v8_platform/boundary/README.md) | 接口适配与机器人协议边界 |
| [representation](../../ws/src/90_platform/v8_platform/representation/README.md)、[planner](../../ws/src/90_platform/v8_platform/planner/README.md) | 多层支撑图、地图服务与全局规划 |
| [local_planner](../../ws/src/90_platform/v8_platform/local_planner/README.md)、[validator](../../ws/src/90_platform/v8_platform/validator/README.md) | 局部规划、轨迹/运动学检查与失效处置 |
| [framework](../../ws/src/90_platform/v8_platform/framework/README.md) | manifest、graph、app 装配和 ROS 运行入口 |
| [scenarios](../../ws/src/90_platform/v8_platform/scenarios/README.md)、`calibration/`、`regression/`、`delivery/` | 场景、校准/冻结、回归与复现 |
| [probe_mode3](../../ws/src/90_platform/v8_platform/probe_mode3/README.md) | SCAN mode3 的离线数学探针与边界 |

复现清单命令从仓库根执行，路径不再相对本规划目录：

```bash
python3 -B ws/src/90_platform/v8_platform/delivery/delivery.py manifest
```

测试入口和前置依赖按各模块 README 选择，当前整改的定向读数见[隔离记录](../进度/00_构建与运行隔离_2026-11-05.md)；本次未进行真实仿真或真机复验。旧版计划书及 R614/R659 早期状态只用于追溯，不能据“主线已收口”推导所有运行能力均已验收；真实数据、正式冻结与单项真机许可仍分别管理。
