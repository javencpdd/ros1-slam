# lite3_slam（模块 M1）

**统一索引头**（本头由文档批次生成并登记于 `note/04_配置改动diff.md`；下方正文为历史内容，未改写）
层 `10_slam` ｜ 角色 `M1 建图（LIO）` ｜ 组织 faster-lio 的采图与存盘落点；faster_lio 本体是第三方副本。
**全局索引**：`ws/README.md`（工作区定位图·红线速记·调用姿势）｜ `note/进度/00_总览.md`（进度唯一真身）｜ `note/04_配置改动diff.md`（哪些改动被许可）｜ `note/07_判据载体代际对照表.md`（判据载体代际现值）｜ `note/05_复验作业单.md`（验收纪律 A–F 段）
**路径读法**：本表一律以 `ws/src/` 为根（如 `95_simulation/lite3_gazebo_sim/verify/m2_smoke.sh`）；以 `note/`、`data/`、`scripts/` 开头的以仓库根为起点；`/xxx` 形如 `/cmd_vel` 的是 ROS 话题名不是路径。

本包速查：

| 要做 | 位置 |
|---|---|
| 真机采图 | `launch/10_slam_mapping.launch` |
| 仿真建图 | `launch/10_slam_mapping_sim.launch` |
| LIO 参数 | `config/faster_lio_outdoor.yaml` |
| 存盘产物 | `data/maps/`（**`data/maps/lite3.pcd` 是样张红线件**，各跑前后取 sha） |

**红线/契约**：**唯一被许可的第三方源码改动**＝`faster_lio` 存盘路径的隔离改写，登记在 `note/04_配置改动diff.md` 并受 ISO 项[8] 长期监控；除此之外该包一字不改。

