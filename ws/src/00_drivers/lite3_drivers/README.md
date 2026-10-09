# lite3_drivers（模块 M0）

**统一索引头**（本头由文档批次生成并登记于 `note/04_配置改动diff.md`；下方正文为历史内容，未改写）
层 `00_drivers` ｜ 角色 `M0 驱动与传感器接入` ｜ 把真机与仿真两侧传感/机体接进来，对上提供统一话题与 TF；本身不含算法。
**全局索引**：`ws/README.md`（工作区定位图·红线速记·调用姿势）｜ `note/进度/00_总览.md`（进度唯一真身）｜ `note/04_配置改动diff.md`（哪些改动被许可）｜ `note/07_判据载体代际对照表.md`（判据载体代际现值）｜ `note/05_复验作业单.md`（验收纪律 A–F 段）
**路径读法**：本表一律以 `ws/src/` 为根（如 `95_simulation/lite3_gazebo_sim/verify/m2_smoke.sh`）；以 `note/`、`data/`、`scripts/` 开头的以仓库根为起点；`/xxx` 形如 `/cmd_vel` 的是 ROS 话题名不是路径。

本包速查：

| 要做 | 位置 |
|---|---|
| 真机侧传感起链 | `launch/01_sensors.launch`（livox 驱动 remap→`/rslidar_points`、frame_id=rslidar） |
| 仿真侧起世界 | `launch/95_sim.launch`（world 已 arg 化，默认指向 95_simulation 的世界件） |
| 仿真机体/位姿桥 | `scripts/sim_adapter.py` |
| 参数落点 | `config/traj_maze.yaml` |
| 占位与下沉位 | `launch/sink/`、`launch/stub/`（两目录是治理位，非空即需登记） |

**红线/契约**：**真机安全规则一**：IMU 桥的 `ros2qnx` 反向通道只许在真机 mode 出现（源在 `00_drivers/message_transformer/src/`，属第三方副本层），stub/采图/回放模式出现即事故。

