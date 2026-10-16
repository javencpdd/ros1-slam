# lite3_bringup（模块 TOP）

**统一索引头**（本头由文档批次生成并登记于 `note/04_配置改动diff.md`；下方正文为历史内容，未改写）
层 `70_bringup` ｜ 角色 `TOP 顶层编排` ｜ 按 mode 组合各平级模块的顶层 launch 层——`<include>` 平级模块是 TOP 的职责，不属 K3 违例。
**全局索引**：`ws/README.md`（工作区定位图·红线速记·调用姿势）｜ `note/进度/00_总览.md`（进度唯一真身）｜ `note/04_配置改动diff.md`（哪些改动被许可）｜ `note/07_判据载体代际对照表.md`（判据载体代际现值）｜ `note/05_复验作业单.md`（验收纪律 A–F 段）
**路径读法**：本表一律以 `ws/src/` 为根（如 `95_simulation/lite3_gazebo_sim/verify/m2_smoke.sh`）；以 `note/`、`data/`、`scripts/` 开头的以仓库根为起点；`/xxx` 形如 `/cmd_vel` 的是 ROS 话题名不是路径。

本包速查：

| 要做 | 位置 |
|---|---|
| mode 入口 | `launch/mode_mapping.launch`、`mode_localize.launch`、`mode_nav_sim.launch`、`mode_nav_stub.launch`、`mapping_viz.launch` 等 |
| 仿真桥与桩 | `scripts/sim_map_bridge.py`、`sim_pose_tf.py`、`stub_tf_broadcaster.py`、`map_snapshot_pub.py` |
| 简化机体描述 | `urdf/Lite3_prim.urdf` |
| 冒烟 | `verify/sim_nav_smoke.sh` |

**红线/契约**：K3 语义＝**平级 `lite3_*` 禁互 include**，TOP 下行不豁免；include 级 remap 的可见性由 ISO 项[5] 监控（曾有两条真 include 待返工）。

- **R348 README 同步（追加句）**＝本包 `config/` 目录此前未登记，现跑其内**一枚**在册件＝`config/elevation_costmap.yaml`＝**31 行／2293 字节／sha 前 8=`aad5adac`／模 664**／mtime 2026-10-14 04:48:55（＝R280 的 S2 规范位；★其 z 窗现读 `z_min=-2.0／z_max=2.0`，与 v6 §B.1.1 丙档设计值不同一事见 `note/计划书v6/23_v6任务队列.md` 的 `B-1` 行与请求单 `R 补八十二`，★不在本 README 判定）。
