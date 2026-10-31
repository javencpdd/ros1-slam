# v8_platform · v8 平台层与定标夹具

v8 的**可执行代码**：平台层设计件、ROS 包装与 flat2d 定标夹具。
R795（2026-10-28）从 `note/计划书v8/` 整体迁入本包；文档仍留在 `note/计划书v8/`，证据在 `note/进度/v8/`。

## 模块

| 目录 | 作用 |
| --- | --- |
| `contracts/` | 组件/机器人/地图/任务四 schema ＋ validate（fail-closed） |
| `registry/` | manifest 加载、检索、声明/实测分离、溯源边车 |
| `composer/` | 组合结构/兼容检测、确定性锁文件 |
| `orchestrator/` | 任务状态机、条件就绪、停止链、预算 |
| `adapters/` | 消息级接线、信息损失声明；`follower_node.py` 为 ROS 跟踪节点 |
| `representation/` | 多层支撑图、版本服务、高程管线、2D 栅格适配、语义地图 `semantic_map/v1`（`v8.fw.semantic_provider` 可入 v2 图） |
| `planner/` | 支撑图有界 A*；`ros_node.py` 为 ROS 包装 |
| `validator/` | 独立轨迹检查器、运动学铁则、失效撤销、跟踪 |
| `scenarios/` | SC01—SC14 场景框架与独立评价器 |
| `calibration/` | 冻结判据与定标件；`flat2d/` 为 Gazebo 2D 定标夹具 |
| `regression/` | 双轨 × 组合 × 场景矩阵 |
| `delivery/` | 规模曲线与复现清单 |
| `probe_mode3/` | mode3 接缝数学探针 |

## 运行测试

```bash
cd ws/src/90_platform/v8_platform
for t in $(find . -name 'test_*.py' | sort); do
  (cd "$(dirname "$t")" && python3 -B "$(basename "$t")") || echo "FAIL $t"
done
```

单件示例：

```bash
python3 -B calibration/test_calibration.py   # 冻结判据
python3 -B calibration/flat2d/test_flat2d.py # 夹具几何/中继
```

## 未接入说明（如实）

apt 侧后端（map_server / global_planner / DWA / TEB）与 SCAN 运行面**未在本设计层实装运行**。
在册 manifest 声明、组合与接线只经设计层验证；实装运行、双轨实跑与 A1—A7 正式判据归仿真阶段。

## 运行面

- ROS 包装：`planner/ros_node.py`、`adapters/follower_node.py`。
- Gazebo 夹具：`calibration/flat2d/*.launch`，由脚本按绝对路径启动。
- `package.xml` / `CMakeLists.txt` 只用于工作区发现；本包不安装 Python 模块，测试与夹具直接按源码路径运行。

对应文档：[计划书 v8](../../../../note/计划书v8/README.md)、
[进度真身](../../../../note/进度/00_总览.md)。
