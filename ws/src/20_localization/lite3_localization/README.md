# lite3_localization（模块 M2）

**统一索引头**（本头由文档批次生成并登记于 `note/04_配置改动diff.md`；下方正文为历史内容，未改写）
层 `20_localization` ｜ 角色 `M2 重定位/配准` ｜ 把 hdl 系与 ndt/fast_gicp 候选装配起来并对外给 `/LIO/odom_vehicle` 类话题；候选件本体都是第三方副本。
**全局索引**：`ws/README.md`（工作区定位图·红线速记·调用姿势）｜ `note/进度/00_总览.md`（进度唯一真身）｜ `note/04_配置改动diff.md`（哪些改动被许可）｜ `note/07_判据载体代际对照表.md`（判据载体代际现值）｜ `note/05_复验作业单.md`（验收纪律 A–F 段）
**路径读法**：本表一律以 `ws/src/` 为根（如 `95_simulation/lite3_gazebo_sim/verify/m2_smoke.sh`）；以 `note/`、`data/`、`scripts/` 开头的以仓库根为起点；`/xxx` 形如 `/cmd_vel` 的是 ROS 话题名不是路径。

本包速查：

| 要做 | 位置 |
|---|---|
| 重定位起节点 | `launch/hdl_localization.launch` |
| 批4 名字契约验收件 | `verify/v_fd_h_name_contract.sh`（**待真机窗口**，不在仿真侧收口） |
| 占位与下沉位 | `launch/sink/`、`launch/stub/` |

**红线/契约**：M2 的判据载体在仿真侧是 `95_simulation/lite3_gazebo_sim/verify/m2_smoke.sh`（改它＝换代际）；本包只承载话题名与 TF 契约，不放算法。

