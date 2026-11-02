# 计划书 v8：可组合的 2D/3D 导航算法平台

> 2026-10-24｜R614／批266｜按用户本轮「开发出计划书v8，并将其落册」的要求建立。
> 状态（2026-11-02，R1113）：规划文档已落册；2D v2 四格 4/4、3D Assembly 12/12×2、
> 传感器图 6/6、全局→MuJoCo 四足 2/2、A6 NDT 静态已跑；A1/A3/A4/A5/A6/A7 仍 PARTIAL，
> A2=PASS*；formal 与真机未签。**规划不是验收结论。**

目标是在 ROS1/Noetic 基础上，让用户按机器人、场景和能力标签选择定位、建图、地图表征、全局规划、局部规划与跟踪算法，由系统检查兼容性、组装运行，并用可复现的 2D/3D 仿真验证组合。

| 文件 | 回答什么问题 |
| --- | --- |
| [21 中心计划书](21_ros1-slam中心计划书-v8.md) | 为什么改、改成什么、保留什么、按什么顺序交付 |
| [22 组件与组合契约](22_组件契约与组合架构.md) | 标签如何使用、接口如何统一、哪些组合必须拒绝 |
| [23 执行队列](23_v8任务队列.md) | 具体任务、依赖、交付物、状态与完成条件 |
| [24 仿真验收与迁移](24_仿真验收与迁移方案.md) | 历史失败如何避免、如何测、什么才算完成、如何迁移 |
| [25 调度作业单](25_v8自动调度作业单.md) | 会话激活时怎么选任务、怎么切片、怎么落账、何时停手 |
| [27 框架改造补丁](27_框架改造补丁-v0.1.md) | YAML 组件图／运行装配／MapDelta／机器人／独立仿真；F0—F8 状态与完成登记 |

现行路线入口以 [note 总入口](../00_README.md) 为准；时间线以 [进度总览](../进度/00_总览.md) 为准；本次落册详情见 [R614](../进度/v8/公共/计划书线/26-10-24/R614.md)。v7 及更早记录保留为历史证据，不自动折算为 v8 验收结果。

下一实施起点以 [23 号执行队列](23_v8任务队列.md) 与 [00_总览](../进度/00_总览.md) 的现势为准；
截至 R1113，剩余非 L2 的真叠层世界尚未建，A1/A4/A5 formal、#151 formal 两键与 V8-X2 真机仍是 L2/新场景决策。
R614 当时的“未启动”只保留为历史背景，不描述当前状态。

## 交付件总览（用户指南，R614 落册／R659 增补）

V8-00—V8-14 主线十五件已按窄口径收口（各件证据见 [进度 v8](../进度/v8/README.md)）。可执行件位置：**R795 起迁至 [`ws/src/90_platform/v8_platform/`](../../ws/src/90_platform/v8_platform/)**（本目录只留计划书与说明）。下表链接指向新位置；测试仍可 `python3 -B test_*.py` 按源码路径复跑：

| 模块 | 件 | 回答什么 |
| --- | --- | --- |
| [contracts/](../../ws/src/90_platform/v8_platform/contracts/) | 四 schema＋validate.py（61 例） | 组件/机器人/地图/任务的契约与拒绝语义 |
| [probe_mode3/](../../ws/src/90_platform/v8_platform/probe_mode3/) | mode3 数学探针（14 例） | SCAN 接缝数学的可用边界（V8-03） |
| [registry/](../../ws/src/90_platform/v8_platform/registry/) | registry.py（19 项）＋13 manifest＋provenance | 组件注册、检索、声明/实测分离、溯源 |
| [composer/](../../ws/src/90_platform/v8_platform/composer/) | composer.py（17 项） | 组合结构/兼容检测、确定性锁文件 |
| [orchestrator/](../../ws/src/90_platform/v8_platform/orchestrator/) | orchestrator.py（13 项） | 任务状态机、条件就绪、停止链、预算 |
| [adapters/](../../ws/src/90_platform/v8_platform/adapters/) | io_match＋declarations（10 项） | 消息级接线、损失声明、组合矩阵 |
| [representation/](../../ws/src/90_platform/v8_platform/representation/) | support_map/map_service/elevation（19 项） | 多层表征、版本服务、建图管线 |
| [planner/](../../ws/src/90_platform/v8_platform/planner/) | support_planner.py（8 项） | 有界 A* 支撑图全局规划 |
| [validator/](../../ws/src/90_platform/v8_platform/validator/) | checker/enforcement/tracking（23 项）＋SCAN 裁决 | 轨迹监督、运动学铁则、失效撤销、跟踪 |
| [scenarios/](../../ws/src/90_platform/v8_platform/scenarios/) | scenarios.py（7 项） | SC01—SC14 场景框架＋独立评价器（A7） |
| [calibration/](../../ws/src/90_platform/v8_platform/calibration/) | calibration.py（8 项） | 冻结判据、定标/正式分离、封版 digest |
| [regression/](../../ws/src/90_platform/v8_platform/regression/) | regression.py（6 项） | 双轨×组合×场景 48 cell 矩阵＋失败定位 |
| [delivery/](../../ws/src/90_platform/v8_platform/delivery/delivery.py) | 规模曲线＋复现清单（5 项） | 性能定界与复现自证 |

**未接入说明（设计层当时口径，后续已推进）**：本表描述的是 R659 设计层；此后 R981/R1084/R1090/R1102/R1110—R1112 已接入真实 2D 栈、A1 矩阵、传感器图与全局→MuJoCo 四足，R1106 接入 A6 NDT 静态恢复。**formal A1/A4/A5、#151 两键与真机仍未签**；以 [00_总览](../进度/00_总览.md) 的现势为准。teb kinematics 定案与 slab A* 插桩仍须容器面/正式批。复现：`python3 -B delivery/delivery.py manifest` 输出交付树逐件 sha256 清单（再生成一致）。
