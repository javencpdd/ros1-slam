# lite3_navigation（模块 M4）

**统一索引头**（本头由文档批次生成并登记于 `note/04_配置改动diff.md`；下方正文为历史内容，未改写）
层 `40_navigation` ｜ 角色 `M4 导航联邦（mbf）` ｜ MBF 参数与插件装配层：代价图、全局/局部规划器、恢复行为都在这层点单。
**全局索引**：`ws/README.md`（工作区定位图·红线速记·调用姿势）｜ `note/进度/00_总览.md`（进度唯一真身）｜ `note/04_配置改动diff.md`（哪些改动被许可）｜ `note/07_判据载体代际对照表.md`（判据载体代际现值）｜ `note/05_复验作业单.md`（验收纪律 A–F 段）
**路径读法**：本表一律以 `ws/src/` 为根（如 `95_simulation/lite3_gazebo_sim/verify/m2_smoke.sh`）；以 `note/`、`data/`、`scripts/` 开头的以仓库根为起点；`/xxx` 形如 `/cmd_vel` 的是 ROS 话题名不是路径。

本包速查：

| 要做 | 位置 |
|---|---|
| 导航主入口 | `launch/mbf_navigation.launch`（加载**底文件**，注意不加载 variants） |
| 裸话题中继 | `launch/cmd_vel_raw_relay.launch` |
| 代价图/规划器参数 | `config/costmap_common.yaml`（`resolution: 0.1`）、`global_costmap_params.yaml`、`local_costmap_params.yaml`、`global_planner_params.yaml`、`mbf_navigation*.yaml`、`planner_teb.yaml` |
| 三档覆盖文件 | `config/variants/{night,outdoor_open,crowd}.yaml`——**自记前置已结案但至今无人加载**＝待裁 B5（坑44 实例） |
| DWA 复核算件 | `verify/dwa_recheck.py`、`v_dwa_recheck.sh`、`v_dwr_exitclean_selftest.sh` |

**红线/契约**：**壳包禁算法源码**（`.cpp/.cc/.h` 零命中，ISO 项[9]）；`/cmd_vel` 只能来自 M6 mux，绕过即 ISO 项[6] 红。双 StaticLayer「可叠加」前提已被 R59 预检推翻（补丁 X2）⇒ 可通行层要走**我方层直接产出合并代价**，不要再塞第二个 layer。

