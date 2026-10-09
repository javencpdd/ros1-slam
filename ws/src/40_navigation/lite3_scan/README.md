# lite3_scan

**统一索引头**（本头由文档批次生成并登记于 `note/04_配置改动diff.md`；下方正文为历史内容，未改写）
层 `40_navigation` ｜ 角色 `SCAN 集成壳` ｜ SCAN-Planner 独立进程的自有 launch/config 编排与验收面；上游五包在 `ws/src/30_3rd/` 按基线不改。
**全局索引**：`ws/README.md`（工作区定位图·红线速记·调用姿势）｜ `note/进度/00_总览.md`（进度唯一真身）｜ `note/04_配置改动diff.md`（哪些改动被许可）｜ `note/07_判据载体代际对照表.md`（判据载体代际现值）｜ `note/05_复验作业单.md`（验收纪律 A–F 段）
**路径读法**：本表一律以 `ws/src/` 为根（如 `95_simulation/lite3_gazebo_sim/verify/m2_smoke.sh`）；以 `note/`、`data/`、`scripts/` 开头的以仓库根为起点；`/xxx` 形如 `/cmd_vel` 的是 ROS 话题名不是路径。

本包速查：

| 要做 | 位置 |
|---|---|
| 起 SCAN | `launch/70_scan.launch`（include 上游 `30_3rd/plan_manage/launch/advanced_param.xml`，`grid_map/resolution=0.05` 在其 :51） |
| 仿真侧 LIO/TF 中继 | `launch/lio_relay_sim.launch` |
| A7 验收链 P1–P4 | `verify/v_scan_p1.sh`…`v_scan_p4.sh` 与被试 `p1_occ_sample.py`／`p2_goal_plan.py`／`p3_track.py`／`p4_replan.py`；三连独立件 `v_scan_p3_triple.sh` |
| 到达判据实体 | `scripts/scan_arrival_monitor.py`（J1 到达观测件；任务层不替上游造失败事件） |
| 其它契约件 | `verify/v_scan_launch_expand.sh`、`v_scan_relay_equiv.sh`、`v_tf_edge_conflict.sh`、`v_scan_o1.sh` |

**红线/契约**：**禁算法源码**（ISO 项[9]）＋**上游按基线不改**：需要改 SCAN 行为时在本壳包自持 relay/参数（批2 已定口径）。引用 P4 等活体结论必须连载体代际＋`.so` sha 一起引。

SCAN-Planner 独立进程壳包：只提供自有 launch/config 编排与参数外化落点，`ws/src/30_3rd/` 内 SCAN 五包源码零改动；本壳无编译目标（`architecture_independent`）。
