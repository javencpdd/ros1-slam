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
| `adapters/` | 消息级接线、信息损失声明；A6 recovery/lease、NDT candidate bridge、`follower_node.py` |
| `representation/` | 多层支撑图、版本服务、高程管线、2D 栅格适配、语义地图 `semantic_map/v1` |
| `planner/` | 支撑图有界 A*＋`ros_node.py` ROS 包装；3D 全局路径 |
| `local_planner/` | `local_native` 3D 滚动规划、独立评价、ROS 节点、停止链 |
| `validator/` | 独立轨迹检查器、运动学铁则、失效撤销、跟踪 |
| `framework/` | v2 图/YAML 装配、app/task/map registry、surface resolver、nav3d viz、Gazebo carrier、GT/LIO 对比、PCD→support_map（R1119） |
| `scenarios/` | SC01—SC14 场景框架与独立评价器 |
| `calibration/` | 冻结判据与定标件；`flat2d/` 为 Gazebo 2D 定标夹具 |
| `regression/` | 双轨 × 组合 × 场景矩阵；传感器建图（gazebo3d）、A1 3D/停止链矩阵 |
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

## 未接入（清单口径）

- `apt` 后端与 SCAN 运行面**未接入**实际运行：只在配置/设计面出现，不参与 A1—A8 计数，
  规模与复现清单见 `delivery/`。

## 当前运行面（截至 R1119）

- **2D**：`v8.nav2d.*` 真实栈由 v2 graph+Assembly 启动，SC09 四格 4/4（R981）。
- **3D**：Assembly 三轮 12/12（R1110）＋三种子起点扰动 12/12（R1111）；全局 34 点→local→MuJoCo 四足接触 2/2（R1112，单策略/声明图/预计算全局）；R1116—R1118 交互入口、`world→map`、`Marker Topic`/Lite3 显示；R1119 键盘建图（GT 位姿 72,762 点）→单面 support_map→3D 导航 31 点到达。
- **传感器图**：Gazebo 激光+GT odom survey 生成 6 层 support_map（R1102），同 XY 选面/歧义（R1108）。
- **A6**：NDT 静态失锁 `~relocalize`/`~initialpose`/lease 门（R1106）；R1114 修复 lease 晚订阅竞态并跑通 0.3 m 局部失锁（候选仍停在 prior，靠 NDT 跟踪回正）；NDT 仍 monitor-only；任意 kidnapping/运动认证/真机未过。
- **测试**：v8 逐文件 **80/80**、discovery **127**、adapters **44**；A1/A3/A4/A5/A6/A7 仍 PARTIAL，A2=PASS*，#151 formal pending。

## 运行面入口

- ROS 包装：`planner/ros_node.py`、`local_planner/ros_node.py`、`adapters/follower_node.py`、`adapters/a6_recovery_node.py`。
- Gazebo 夹具：`calibration/flat2d/*.launch`；3D/传感器回归在 `regression/gazebo3d_*`。
- 仿真建图→3D 导航：`framework/map_from_pcd.py`（PCD→support_map/app/task）＋
  `framework/app_runner.py --app data/maps/generated/<id>_v1_app.yaml --execute --interactive --rviz`；
  四终端建图命令见 `docker/README.md` §4.7。
- `package.xml` / `CMakeLists.txt` 只用于工作区发现；本包不安装 Python 模块，测试与夹具直接按源码路径运行。

对应文档：[计划书 v8](../../../../note/计划书v8/README.md)、
[进度真身](../../../../note/进度/00_总览.md)、[framework README](framework/README.md)。
