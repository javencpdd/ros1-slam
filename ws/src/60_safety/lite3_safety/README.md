# lite3_safety（模块 M6）

**统一索引头**（本头由文档批次生成并登记于 `note/04_配置改动diff.md`；下方正文为历史内容，未改写）
层 `60_safety` ｜ 角色 `M6 安全仲裁` ｜ `/cmd_vel` 的唯一写者：多源优先级仲裁与安全闸门。
**全局索引**：`ws/README.md`（工作区定位图·红线速记·调用姿势）｜ `note/进度/00_总览.md`（进度唯一真身）｜ `note/04_配置改动diff.md`（哪些改动被许可）｜ `note/07_判据载体代际对照表.md`（判据载体代际现值）｜ `note/05_复验作业单.md`（验收纪律 A–F 段）
**路径读法**：本表一律以 `ws/src/` 为根（如 `95_simulation/lite3_gazebo_sim/verify/m2_smoke.sh`）；以 `note/`、`data/`、`scripts/` 开头的以仓库根为起点；`/xxx` 形如 `/cmd_vel` 的是 ROS 话题名不是路径。

本包速查：

| 要做 | 位置 |
|---|---|
| 起仲裁 | `launch/60_safety.launch` |
| 优先级与源表 | `config/cmd_vel_mux.yaml`——7 个源：`estop`(1，最高)→`teleop_joy`(2)→`teleop_app`(3)→`task`(4)→`navigation`(`/cmd_vel_raw`,5)→`demo`(6)→`scan_planner`(7，最低)；注释明写取 7 而非与 navigation 并列 5，**并列会让 teb 与 SCAN 同时活跃成时序竞态** |
| 仲裁主件 | `scripts/safe_cmd_vel_mux.py` |
| 验收件 | `verify/v_mux_sources.sh` |

**红线/契约**：**K2 红线**：任何包绕过 mux 直写 `/cmd_vel` ⇒ ISO 项[6] 直接红；采图全程不发 `/cmd_vel`。

