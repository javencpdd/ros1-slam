# lite3_gazebo_sim（SIM 仿真域·测试台本体）

**统一索引头**（本头由文档批次生成并登记于 `note/04_配置改动diff.md`；下方正文为历史内容，未改写）
层 `95_simulation` ｜ 角色 `SIM 仿真域（测试台本体）` ｜ Gazebo 世界、机体仿真描述、推进插件、M2 判据载体与全部仿真侧验收件都在这里；另含楼梯可行性离线预检旁路。
**全局索引**：`ws/README.md`（工作区定位图·红线速记·调用姿势）｜ `note/进度/00_总览.md`（进度唯一真身）｜ `note/04_配置改动diff.md`（哪些改动被许可）｜ `note/07_判据载体代际对照表.md`（判据载体代际现值）｜ `note/05_复验作业单.md`（验收纪律 A–F 段）
**路径读法**：本表一律以 `ws/src/` 为根（如 `95_simulation/lite3_gazebo_sim/verify/m2_smoke.sh`）；以 `note/`、`data/`、`scripts/` 开头的以仓库根为起点；`/xxx` 形如 `/cmd_vel` 的是 ROS 话题名不是路径。

本包速查：

| 要做 | 位置 |
|---|---|
| 起世界 | `launch/m1_gazebo.launch`、`m2_gazebo.launch`（世界件本体 `worlds/m1_world.world`＝**红线件**，各跑取 `world_before.sha`／`world_after.sha`） |
| 仿真传感/建图/规划 | `launch/m1_mapping.launch`、`m2_mapping.launch`、`m2_hdl.launch`、`m2_teb.launch` |
| 机体描述（仿真实际用的） | `urdf/lite3_planar.urdf.xacro`——刚体盒子（只有 fixed 关节）＋雷达插件参数在 :85–96、传感器安装高在 :55（`origin xyz="0 0 0.38"`） |
| 推进插件源码 | `src/lite3_planar_drive.cpp`（`SetLinearVel`/`SetWorldPose`；打戳统一 SimTime 的坑25 根除改点在此） |
| **判据载体** | `verify/m2_smoke.sh` ＋ `verify/m2_exitclean_lib.sh` ⇒ **改这两个＝代际加行**（现值见 `note/07_判据载体代际对照表.md`） |
| 批跑与自测件 | `verify/m2_batch_runner.sh`、`retake_driver.sh`、`v_m2_exitclean_selftest.sh`、`v_g35_branch_calibration.sh`、`v_b3_timebase_contract.sh`、`v_pose_source.sh`、`v_br_expect_negative_selftest.sh`、`v_rd_archive_selftest.sh`、`v_w3_terrain_design_contract.sh` |
| 离线预检旁路（不被 launch 引用） | `stairstudy/{fixtures,precheck,sweep,bandstudy}.py` ＋ `stairstudy/worlds/stair_lab_*.world`（八件候选场景） |
| 探针小件 | `scripts/m2_tf_fresh.py`、`m2_free_map.py`、`m2_cmd_relay.py`、`m2_pose_source_audit.py`、`wait_sim_clock.sh` |

**红线/契约**：载体能力边界（补丁 X7 物证）：刚体盒子**能验 ①–④（表征/判据/代价/规划），不能验 ⑤（踏步/落脚/摩擦）**；带腿描述在 `95_simulation/lite3_description/urdf/Lite3.urdf`（12 revolute）但无腿控制器。保真度缺口 O-10（仿真竖向 50 线摊 ±90° ⇒ 楼梯立面带内仅 8 条线）待裁＝B6。

## 说明

本 README 是新建件（文档批次）：包内此前无 README，内容以上表为准；进度与证据不在本目录，见 `note/进度/`。
