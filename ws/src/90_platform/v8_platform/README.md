# v8_platform · 配置装配、规划与运行适配

> 最新更新时间：2026-11-05 11:49:54 CST。本次为文档与源码核对；验收状态以[项目进度](../../../../note/进度/README.md)为准。

本包保存 v8 平台自研代码：组件契约、注册与组合、进程编排、地图表征、全局/局部规划、ROS 适配和回归工具。计划与设计文档在[计划书 v8](../../../../note/计划书v8/README.md)，历史运行证据在[进度入口](../../../../note/进度/v8/README.md)。

## 从哪里开始

| 目录 | 当前职责 |
| --- | --- |
| [framework](framework/README.md) | app/task/map registry、v2 manifest 与 graph、Assembly 运行、选面/可视化、安全接线与热替换 |
| [contracts](contracts/README.md) | manifest/robot/map/task/map_delta 契约与语义校验；map v1/v2/v3 |
| [registry](registry/README.md) / [composer](composer/README.md) | 注册、证据与溯源分离、角色/地图/运动学兼容和确定性锁 |
| [orchestrator](orchestrator/README.md) | 任务生命周期、条件就绪、进程回收、停止链和预算 |
| [adapters](adapters/README.md) | 消息匹配、信息损失声明、定位跟踪/租约与 A6 恢复接口 |
| [representation](representation/README.md) | 多层支撑图、真实格集、版本/增量/持久化、语义地图与 ROS 查询 |
| [planner](planner/README.md) | 支撑图有界 A*、障碍/净空、ROS 服务和话题接口 |
| [local_planner](local_planner/README.md) / [validator](validator/README.md) | 原生三维局部规划、三维命令与安全仲裁接口、独立轨迹/跟踪检查 |
| [boundary](boundary/README.md) | UDP 命令/遥测适配、仿真端点、记录与回放 |
| [scenarios](scenarios/README.md) / [probe_mode3](probe_mode3/README.md) | SC01—SC14 定义与独立评价；SCAN 接缝数学探针 |
| [configs/lite3](configs/lite3/README.md) | Lite3 app/graph/局部配置模板与现场边界 |
| `calibration/` / `regression/` / `delivery/` | 冻结定标、运行矩阵、规模曲线与复现清单 |

`framework/fixtures/manifests/`、`graphs/`、`apps/` 中既有测试替身，也有实际运行装配输入；按 manifest 的 launch 和证据逐件判断，不能把整个 fixtures 当作可删样例。

## 使用入口

在仓根执行（Python 工具按源码路径导入；本包 CMake 只声明 catkin 包，不安装 Python 模块）：

```bash
python3 -B ws/src/90_platform/v8_platform/framework/cli.py check \
  ws/src/90_platform/v8_platform/framework/fixtures/manifests \
  ws/src/90_platform/v8_platform/framework/fixtures/graphs/graph_3d_cache.yaml
python3 -B ws/src/90_platform/v8_platform/framework/app_runner.py \
  --app ws/src/90_platform/v8_platform/framework/fixtures/apps/nist_sensor_3d.yaml
```

第二条缺省为 dry-run。`--execute` 会启动实际 ROS/仿真进程，运行前按[作业单](../../../../note/计划书v8/25_v8自动调度作业单.md)准备隔离环境。完整 app、建图与交互用法见 [framework](framework/README.md)、[Docker 运行指南](../../../../docker/README.md)。本机地图与 `*.local.app.yaml` 各机独立。

离线测试按模块运行，例如：

```bash
python3 -B ws/src/90_platform/v8_platform/contracts/test_contracts.py
python3 -B ws/src/90_platform/v8_platform/planner/test_support_planner.py
python3 -B ws/src/90_platform/v8_platform/local_planner/test_local_native.py
python3 -B ws/src/90_platform/v8_platform/framework/test_preflight.py
```

测试数量随源码变化；本文不把旧批次数或文档更新时间作为新的运行结果。

## 已接入与未接入的范围

v2 graph/Assembly 已有真实 2D 导航栈、三维全局/局部规划、地图增量和仿真载体入口。Gazebo 运动学 carrier、Gazebo 动态方盒和 MuJoCo 四足接触实验各有独立范围，见[架构说明](../../../../note/架构/项目架构与目录.md)。

旧注册表 `registry/data/manifests/` 中的 apt 与 SCAN 条目主要用于声明、组合与接线检查：这些 v1 条目**未接入** v2 命名端口图的实际启动，不能凭登记即计为运行支持。framework 中另有已接入的 `v8.nav2d.*` include 组件，不能由旧条目的限制推断整个 apt 导航栈未运行。

NDT 包 [v8_ndt_odom](../v8_ndt_odom/README.md) 仍仅为 monitor，不承担连续导航定位。formal、真机与各 A 项状态统一查[进度](../../../../note/进度/README.md)，本文件不签署验收。
