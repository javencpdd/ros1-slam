# lite3_supervisor（模块 SUP）

**统一索引头**（本头由文档批次生成并登记于 `note/04_配置改动diff.md`；下方正文为历史内容，未改写）
层 `90_supervisor` ｜ 角色 `SUP 心跳监管` ｜ 监管话题/节点活性并在越界时处置；自身绝不产生速度指令。
**全局索引**：`ws/README.md`（工作区定位图·红线速记·调用姿势）｜ `note/进度/00_总览.md`（进度唯一真身）｜ `note/04_配置改动diff.md`（哪些改动被许可）｜ `note/07_判据载体代际对照表.md`（判据载体代际现值）｜ `note/05_复验作业单.md`（验收纪律 A–F 段）
**路径读法**：本表一律以 `ws/src/` 为根（如 `95_simulation/lite3_gazebo_sim/verify/m2_smoke.sh`）；以 `note/`、`data/`、`scripts/` 开头的以仓库根为起点；`/xxx` 形如 `/cmd_vel` 的是 ROS 话题名不是路径。

本包速查：

| 要做 | 位置 |
|---|---|
| 起监管 | `launch/90_supervisor.launch` |
| 看门狗参数 | `config/health_watch.yaml` |
| 主件 | `scripts/health_monitor.py` |

**红线/契约**：**SUP 红线**：`lite3_supervisor` 亦在 `/cmd_vel` 扫描范围内（ISO 项[6] 显式覆盖它）。

