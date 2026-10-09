# lite3_task（模块 M5）

**统一索引头**（本头由文档批次生成并登记于 `note/04_配置改动diff.md`；下方正文为历史内容，未改写）
层 `50_task` ｜ 角色 `M5 任务编排` ｜ 契约先行的 FSM 层：原语注册表＋SMACH 简版；语义只消费上游事件，不替上游造失败。
**全局索引**：`ws/README.md`（工作区定位图·红线速记·调用姿势）｜ `note/进度/00_总览.md`（进度唯一真身）｜ `note/04_配置改动diff.md`（哪些改动被许可）｜ `note/07_判据载体代际对照表.md`（判据载体代际现值）｜ `note/05_复验作业单.md`（验收纪律 A–F 段）
**路径读法**：本表一律以 `ws/src/` 为根（如 `95_simulation/lite3_gazebo_sim/verify/m2_smoke.sh`）；以 `note/`、`data/`、`scripts/` 开头的以仓库根为起点；`/xxx` 形如 `/cmd_vel` 的是 ROS 话题名不是路径。

本包速查：

| 要做 | 位置 |
|---|---|
| 起编排 | `launch/50_task_smach_min.launch` |
| 原语与 goal 契约 | `config/task_primitives.yaml`（`target=local:scan_goal` 这类前缀空间在此定） |
| 编排主件 | `scripts/task_smach_min.py` |
| 验收件 | `verify/v_task_smach.sh`、`v_task_scan_primitive.sh`、`v_scan_task_live.sh` |

**红线/契约**：到达判据用 J1 观测件而非 FSM 内部态；任务层失败侧只有超时档（FSM 无失败事件）。

