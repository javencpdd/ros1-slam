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

**红线/契约**：**这里不放算法**——但要说清约束性质：门禁项 `[9]` 只扫 `40_navigation` 下的 `.cpp/.cc/.h`，**并不监控 30_perception**（我原先写成"同款禁令由 [9] 监控"是过度声明，2026-10-09 更正）；此处的约束来自 N2/§2.6 硬规则 b 的意图，属纪律而非门禁红；实现需等 #46（合并代价层路线）与 B6（仿真竖向图案保真度）裁定。


## 更正与现状（R280，2026-10-14 04:5x；上文一字不回改，按本节读）

- **本包不再空壳**：S1 高程表征件骨架 `scripts/w3_elevation_node.py` 已落盘（落点＝部署架构师-小张 R278 裁定；主理人 2026-10-14 明文「放行 3D」）。
- **上文「这里不放算法」的适用范围照原样成立、不需撤回**：那句管的是**第三方算法源码**（门禁项 `[9]` 只扫 `40_navigation` 下的 `.cpp/.cc/.h`，实测该目录命中 0 件），本件是**自研 python 表征件**、不在其域 ⇒ 二者不冲突。★同批已核：`v_w3_terrain_design_contract.sh` 的 W5 联合判要求"落点件在位且全盘唯一＋规范位 yaml 在位"，落错包即真红。
- **本件的输出仍属待定形**：`#46 三态语义`（未标注⇒未知、绝不塌陷成可上）在册未销 ⇒ 代码里只实现"零覆盖⇒UNKNOWN、默认分支不产出 FREE"这一半，**不声称三态语义已定档**；`slope 25°`／`step 0.15m`／`roughness 0.05m` 三档一律读「未标定施工值（实测锚＝0 处，不得作真机能力上限）」。
- 达成判定权在验收侧；S3 及以后取决于 A45（X26 前件销账）。
