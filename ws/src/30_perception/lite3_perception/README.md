# lite3_perception（模块 M3）

**统一索引头**（本头由文档批次生成并登记于 `note/04_配置改动diff.md`；下方正文为历史内容，未改写）
层 `30_perception` ｜ 角色 `M3 感知（高程/可通行）` ｜ 自研壳**当前为空**：只有治理位目录，没有自有 launch/config。
**全局索引**：`ws/README.md`（工作区定位图·红线速记·调用姿势）｜ `note/进度/00_总览.md`（进度唯一真身）｜ `note/04_配置改动diff.md`（哪些改动被许可）｜ `note/07_判据载体代际对照表.md`（判据载体代际现值）｜ `note/05_复验作业单.md`（验收纪律 A–F 段）
**路径读法**：本表一律以 `ws/src/` 为根（如 `95_simulation/lite3_gazebo_sim/verify/m2_smoke.sh`）；以 `note/`、`data/`、`scripts/` 开头的以仓库根为起点；`/xxx` 形如 `/cmd_vel` 的是 ROS 话题名不是路径。

本包速查：

| 要做 | 位置 |
|---|---|
| 为什么是空的 | 厂商包 `30_perception/deeprobotics_local_height_map` 带 `CATKIN_IGNORE`（闭源、63 文件 0 代码件），其 `slope_threshold` 量纲不可核 ⇒ 见 v4.3 补丁 X5 |
| 楼梯可通行性的现状证据 | `95_simulation/lite3_gazebo_sim/stairstudy/`（离线预检四件）＋ `note/进度/仿真测试台/设计稿/H_第四通道离线分辨力预检结论.md` |
| 二期接口/判据契约件（零节点） | `95_simulation/lite3_gazebo_sim/verify/v_w3_terrain_design_contract.sh`——逐字钉住设计件 `note/进度/仿真测试台/证据/log/matrix_w3/spike3d_elevation_costmap_design.md`（落账 sha 01f30b8b、commit 5867efe）的接口表、三通道判据、`unknown=−1` 语义与开工前置 |

**红线/契约**：**不要在这里放算法源码**（40_navigation 同款禁令由 ISO 项[9] 监控）；实现需等 #46（合并代价层路线）与 B6（仿真竖向图案保真度）裁定。

