# ros1-slam · 四足机器人 SLAM 与导航

> 最新更新时间：2026-11-23 14:05 CST。**当前基线 v8.1**。本页只回答「现在的系统是什么、怎么跑、怎么验、还没做什么」；历史过程一律走[历史追溯入口](note/历史追溯入口.md)。

## 0. 三个入口（先看这里）

| 你要做什么 | 入口 | 说明 |
| --- | --- | --- |
| **理解当前系统** | 本页 → [项目架构与目录](note/架构/项目架构与目录.md) | 架构、能力、接口、运行边界 |
| **动手开发/调试/验收** | [开发维护入口](note/开发维护入口.md) | 构建、启动、调试、测试、门禁、操作规范 |
| **查历史（v7 及更早）** | [历史追溯入口](note/历史追溯入口.md) | v3—v7 已冻结，仅供追溯，日常开发不需要 |

**每类当前信息只有一个权威源**（其余文档只引用、不重复维护）：

| 主题 | 权威源 | 其余文档的角色 |
| --- | --- | --- |
| 仓库结构、两条装配链、单写者与运行边界 | [项目架构与目录](note/架构/项目架构与目录.md) | 本页 §1 只给 30 秒摘要 |
| 接口（话题/类型/参数/时序） | [运行链路与接口](note/架构/运行链路与接口_2026-11-23.md) + [接口清单 TSV](note/架构/接口清单_2026-11-23.tsv) | 本页只列入口，不抄表 |
| 能力状态与验收范围 | [24 号验证与交付方案](note/计划书v8.1/24_验证与交付方案.md) | [v8 能力表](note/进度/v8/README.md) 按运行面索引 |
| 构建/启动/测试/门禁命令 | [开发维护入口](note/开发维护入口.md) | 本页 §5 只给一行式速查 |
| 真机操作与取消口径 | [10 真机上机Runbook](note/10_真机上机Runbook.md) | — |
| 工程约束（提交、构建、算法接口） | [工程标准](note/架构/工程标准.md) | — |

## 1. 系统是什么（30 秒）

面向 Lite3 四足机器人，组合激光建图、定位、地图、全局规划、局部执行与安全仲裁，ROS1/Noetic。x86 承担 Docker + Gazebo + MuJoCo 开发与回归；Lite3 主机跑原生 ROS1，地图与现场配置各机独立。

两条装配链，共用同一套算法、传感器与安全件：

- **传统 ROS launch**：`lite3_bringup` 组合驱动、SLAM、定位、导航、安全仲裁与 SUP 监管。
- **v8 配置装配**：app YAML → graph → manifest 校验 → Assembly 起进程；运行数据经 ROS 话题传递。

2026-11-21 起算法与平台分层：算法在 [`ws/src/80_algorithm/`](ws/src/80_algorithm/)（`v8_algorithm` 规划/表示/局部执行/标定，`v8_grid` 栅格化），[`v8_platform`](ws/src/90_platform/v8_platform/) 只留框架、编排、注册、契约、组合、校验、回归与边界。

```mermaid
flowchart LR
    A["app / task / 本机地图"] --> G["graph + manifest"]
    G --> E["执行域 / 机器人身份校验"]
    E --> R["Assembly / ProcessRunner"]
    R --> N["ROS 节点与适配器"]
    N --> P["定位与地图 → 全局路径 → 局部命令"]
    P --> F["坐标 / 源时效 / 租约校验"]
    F --> S["安全仲裁 → 载体"]
    V["独立检查与评价"] -.-> N
```

### 三条链各自的职责（别混）

| 组件 | 职责 | 不做什么 |
| --- | --- | --- |
| **BT 编排**（`lite3_bt`） | 唯一的目标所有者与流程编排：何时要路径、何时走、何时停、失败怎么恢复 | **不写速度**；输出只有目标、取消、状态、租约、停止 |
| **局部执行**（`local_native`） | 把路径变成带时间戳的轨迹与速度命令，做源时效、几何与租约复核 | 不仲裁、不直接连底盘 |
| **安全链**（`safety_mux_lite3`、supervisor、超声/步态守卫） | 速度仲裁、限速、hold、单写者约束 | 不做规划 |

接口逐条（话题、类型、写者、读者、参数）见[运行链路与接口](note/架构/运行链路与接口_2026-11-23.md)与配套 `note/架构/接口清单_2026-11-23.tsv`。

## 2. 现在能做什么（能力与验收状态）

**A2 = PASS\*（窄域）；A1/A3/A4/A5/A6/A7 = PARTIAL；formal 与真机尚未签署（#151 两键 pending）。**

逐项含义、声明范围与「不能据此宣称什么」，权威在[24 号验证与交付方案](note/计划书v8.1/24_验证与交付方案.md)；按运行面索证见[v8 能力表](note/进度/v8/README.md)。

已具备的能力面（各自有范围，不能互相外推）：v8 契约与图装配、2D 导航栈、多层支撑图、3D 全局/局部规划、传感器建图、MuJoCo 执行、NDT 失锁恢复实验。

> 仿真 GT 建图、声明地图、单策略结果、运动学载体都只适用于记录中的范围，**不等于**四足物理验收或真机能力。

## 3. support / FAR 双后端：怎么工作、怎么切

| | support（默认） | far |
| --- | --- | --- |
| 全局规划数据源 | 本仓支撑图规划器 | 第三方 FAR overlay（带 intensity 的语义地形云） |
| 仓外依赖 | 无 | **有**，需要 overlay 挂载 |
| 切换点 | graph 的 `global_backend` | 同左 |
| 共同面 | 同一棵 BT、同一条安全链、同一个局部规划器 | 同左 |
| 额外要求 | — | `arrival_topic` **必须显式给**（`std_msgs/Bool`） |

```bash
roslaunch lite3_bringup mode_sim_bt.launch global_backend:=support
roslaunch lite3_bringup mode_sim_bt.launch global_backend:=far arrival_topic:=/v8/bt/arrived
```

设计与验收：[改造清单](note/架构/双后端可替换改造清单_2026-11-22.md)、[验证记录](note/架构/双后端可替换_验证记录_2026-11-22.md)、[前置条件确认单](note/架构/双后端可替换_前置条件确认单_2026-11-22.md)。

## 4. 从哪起手：launch / graph / manifest / 配置

| 类型 | 位置 | 典型文件 | 干什么 |
| --- | --- | --- | --- |
| launch（传统链） | `ws/src/70_bringup/lite3_bringup/launch/` | `mode_sim_bt.launch`、`mode_navigation.launch`、`interactive_2d.launch` | 一条命令起一串节点 |
| graph（装配） | `ws/src/90_platform/v8_platform/configs/lite3/` | `graph_3d_lite3_bt.yaml`、`graph_3d_lite3_far.yaml` | 实例 → 组件 → 命名空间 → 参数覆盖 |
| manifest（契约） | `ws/src/90_platform/v8_platform/framework/fixtures/manifests/` | `manifest_bt_console.json` 等 | 端口类型、所有权、停止方式 |
| 配置 | `ws/src/90_platform/v8_platform/configs/lite3/`、各包 `config/` | `local_native.yaml`、`graph_3d_lite3.yaml` | 阈值、话题名、后端选择 |
| app（一键跑） | `framework/fixtures/apps/` | `nist_sensor_3d.yaml` | `app_runner --app` 的入口 |

## 5. 构建、启动、测试、验证

| 目的 | 命令 | 期望 |
| --- | --- | --- |
| 看构建闭包（只读） | `bash docker/build.sh --plan` | 38 包，无 missing |
| 真编译 | `bash docker/build.sh` | 容器内 catkin 全量 |
| 起环境 | `cd docker && docker compose --env-file .env up -d` | 7 个服务常驻 |
| 起仿真 + BT 导航 | `roslaunch lite3_bringup mode_sim_bt.launch` | BT/全局/局部/安全链全起 |
| 只看仿真场地 | `bash docker/sim.sh gazebo`、`bash docker/sim.sh probe` | 场地/环境自检 |
| 单测 | `python3 scripts/dev/dev.py test` | v8_platform 344 项 OK |
| 正式门禁 | `python3 scripts/dev/dev.py gate` | 11 成员逐项 rc |
| 文档一致性 | `python3 scripts/dev/dev.py docs --check`、`… lint` | 均为 0 |

一键入口是 `scripts/dev/dev.py`（`status|build|test|gate|smoke|docs|lint`），详见[开发维护入口](note/开发维护入口.md)。

## 6. 还没做什么、已知限制

| 项 | 状态 | 权威位置 |
| --- | --- | --- |
| formal 签署 | 未签（两键 pending） | [24 号验证方案](note/计划书v8.1/24_验证与交付方案.md) |
| 真机验证 | 属 L2，需真人现场执行 | [上机 Runbook](note/10_真机上机Runbook.md) |
| M20 适配 | **未实现**（硬件/协议/ROS 适配缺失） | `tooling/build/profiles.json` 的 `m20.supported=false` |
| 支撑图数据新鲜度 | 建图后新地形在 A 链看不见（缺口已登记） | [归档索引 §四](note/进度/归档/README.md) |
| 结构性欠款 | 见技术债台账 | [08 技术债](note/08_技术债台账.md) |
| 当前在办任务 | 以 `.ai` 任务卡为准 | `python3 .ai/bridge.py ls`、[23 号队列](note/计划书v8.1/23_v8.1任务队列.md) |

## 7. 仓库结构与约定

```
ws/                 主 catkin 工作区（算法/平台/驱动/导航/安全）
subprojects/        仿真（独立 catkin 工作区）与控制台
note/               规则、台账、进度与证据（读法见 note/00_README.md）
data/  docker/  tooling/  scripts/  docs/  web/
```

- **第三方副本**保持原件；`framework/fixtures/` 是实际运行输入，不按名称当临时目录清理。
- **ROS 检查**用当前机器的 Noetic 与已构建工作区；前置缺失记环境问题，不能据此判算法通过/失败。
- **各机独立**：地图、实验输出、`ws/build|devel`、日志；仓内只留 [registry 模板](data/maps/registry.template.json)，本机用被忽略的 `*.local.app.yaml`。
- **控制台/官网**：控制台在 [subprojects/console](subprojects/console/README.md)，`data/console` 是 host-only 测试面；`web/` 只提供文档面。
- **真机运动**仍需单项授权，本文不授权启动硬件。
- **工程约束真源**：[工程标准](note/架构/工程标准.md)（提交纪律、目标构建、算法接口标准）。规范统一不等于能力已实现。

## 8. 新读者六问自检

| 问题 | 答案在哪 |
| --- | --- |
| 当前架构？ | §1 本页 + [项目架构与目录](note/架构/项目架构与目录.md) |
| 哪些文件现行、哪些退役？ | §0 三入口；退役件在[历史追溯入口](note/历史追溯入口.md)标「冻结」 |
| 各 ROS 组件的代码与配置？ | §4 四类入口表 + [运行链路与接口](note/架构/运行链路与接口_2026-11-23.md) |
| 怎么构建与测试？ | §5 + [开发维护入口](note/开发维护入口.md) |
| 哪些能力还没验证？ | §6 |
| 不看 v7 能开发吗？ | 能。日常只需本页 + 架构 + 开发维护三处 |
