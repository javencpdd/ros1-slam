# lite3_gazebo_sim（SIM 仿真域·测试台本体）

> 最新更新时间：2026-11-05 18:27:34 CST。现行规划见仓库 `note/计划书v8/`；本次核对源码入口与载体配置，历史指标仍只适用于各原记录。

**统一索引头**（模块导航；职责以本轮源码核对与原运行范围为准）
层 `95_simulation` ｜ 角色 `SIM 仿真域（测试台本体）` ｜ Gazebo 世界、机体仿真描述、推进插件、M2 判据载体与全部仿真侧验收件都在这里；另含楼梯可行性离线预检旁路。
**全局索引**：`ws/README.md`（工作区定位图·红线速记·调用姿势）｜ `note/进度/00_总览.md`（版本成果与版本差异）｜ `note/进度/01_批次.md`（历史批次审计定位）｜ `.ai`（当前任务与交接）｜ `note/04_配置改动diff.md`（哪些改动被许可）｜ `note/07_判据载体代际对照表.md`（判据载体代际现值）｜ `note/05_复验作业单.md`（验收纪律 A–F 段）
**路径读法**：本表一律以 `ws/src/` 为根（如 `95_simulation/lite3_gazebo_sim/verify/m2_smoke.sh`）；以 `note/`、`data/`、`scripts/` 开头的以仓库根为起点；`/xxx` 形如 `/cmd_vel` 的是 ROS 话题名不是路径。

本包速查：

> **容器口径**：`sim`（headless，容器内 Xvfb）与 `sim-viz`（宿主 X/GUI）共用 11313/11345，
> **一次只起一个**；首次 `--build`、之后 `--no-build`，启动/自检/排障见
> [`docker/README.md`](../../../../docker/README.md)。端口被占时先
> `docker compose --profile viz stop sim-viz`；容器 Up 但无端口则
> `docker compose --profile sim up -d --no-build --force-recreate sim`。

| 要做 | 位置 |
|---|---|
| 起世界 | `launch/m1_gazebo.launch`、`m2_gazebo.launch`（世界件本体 `worlds/m1_world.world`＝**红线件**，各跑取 `world_before.sha`／`world_after.sha`） |
| 仿真传感/建图/规划 | `launch/m1_mapping.launch`、`m2_mapping.launch`（两者加载**真机 outdoor** `faster_lio_outdoor.yaml`，仿真请改用 `lite3_slam/launch/10_slam_mapping_sim.launch` 的 `faster_lio_sim.yaml`）、`m2_hdl.launch`、`m2_teb.launch` |
| 交互式 2D 导航 | `launch/interactive_2d.launch`（MBF + `mbf_goal_bridge.py`；RViz 2D Nav Goal 直接驱动 MBF） |
| 交互式 3D / 自建图导航 | `framework/app_runner.py --execute --interactive --rviz` + `framework/fixtures/rviz/nav3d.rviz`；自建图入口见 `map_from_pcd.py` 生成物 |
| 键盘建图（仿真 GT 位姿） | `scripts/mapping_recorder.py` + `scripts/keyboard_teleop.py`；RViz 用 `docker/rviz_mapping3d.rviz` |
| 机体描述（仿真实际用的） | `urdf/lite3_planar.urdf.xacro`——刚体盒子（只有 fixed 关节）＋雷达位置由 `lidar_z=0.34`、`lidar_pitch_deg=15` 与 `base_to_rslidar` 的 xacro 展开共同确定；参数不是旧固定 `0.38` |
| 推进插件源码 | `src/lite3_planar_drive.cpp`（dynamic 分支 `SetLinearVel`/`SetAngularVel` 保留接触；kinematic 回退 `SetWorldPose`；打戳统一 SimTime 的坑25 根除改点在此） |
| **判据载体** | `verify/m2_smoke.sh` ＋ `verify/m2_exitclean_lib.sh` ⇒ **改这两个＝代际加行**（现值见 `note/07_判据载体代际对照表.md`） |
| 批跑与自测件 | `verify/m2_batch_runner.sh`、`retake_driver.sh`、`v_m2_exitclean_selftest.sh`、`v_g35_branch_calibration.sh`、`v_b3_timebase_contract.sh`、`v_pose_source.sh`、`v_br_expect_negative_selftest.sh`、`v_rd_archive_selftest.sh`、`v_w3_terrain_design_contract.sh` |
| 离线预检旁路（不被 launch 引用） | `stairstudy/{fixtures,precheck,sweep,bandstudy}.py` ＋ `stairstudy/worlds/stair_lab_*.world`（九件 Gazebo 候选场景，含 blank 对照） |
| 探针小件 | `scripts/m2_tf_fresh.py`、`m2_free_map.py`、`m2_cmd_relay.py`、`m2_pose_source_audit.py`、`wait_sim_clock.sh` |

**载体边界**：当前 M1/M2 xacro 默认 `driveMode=dynamic`，使用 `SetLinearVel`/`SetAngularVel` 的动态平面方盒；不是四足步态。v8 `gazebo_carrier3d` 是另一种运动学载体，MuJoCo 接触执行见 [dynamics](dynamics/README.md)。三者的执行和验收证据不能互代。传感器外参/扫描覆盖应取实际装配，不沿用早期 gen0 数字代表现态。

**当前驱动口径（R1109/R1113）**：URDF 实际 `driveMode=dynamic`；插件 beat 实测
cmd `.2→v_phy .199`、cmd `.5→w_phy .481`，z 稳定 `.11`。它是**动态平面方盒＋Gazebo 接触**，
不是 kinematic pose sink；仍无四足步态/接触状态估计。A5 同 lab 四例的 `reap` stdout 死锁
已在 R1113 修复（5/5、20/20 case VALID），测试台不再靠 500 s 外层 timeout。

## 说明

本 README 是新建件（文档批次）：包内此前无 README，内容以上表为准；进度与证据不在本目录，见 `note/进度/`。

## 交互式与自建图入口（R1116—R1121）

**技术面归属（先读）**：楼梯导航与 3D 导航是**同一技术面**，共用
`app_runner` + support_map + `support_planner` + `local_native` + `cmd3d_mux` +
`gazebo_carrier3d`，只是楼梯场景使用图上的 `step/slope` 边和踏面边界语义；
不是独立栈。2D MBF（`interactive_2d.launch`）是另一套平面栈；3D 建图是产图环节；
四足物理/真机爬楼是另一执行面。下面 R1119 平层与 R1121 楼梯是同一 3D 栈的两个场景。

**world vs scene**：`m1_gazebo`／`m2_gazebo` 的参数是完整 `world:=...`（`worlds/*.world`），没有 `scene` 参数；
`stairstudy` 场景库用 `stairstudy.launch world_file:=stair_lab_*.world`；`sim_scene.py --scene` 是运行中
spawn/delete 模型，不换 world，也不更新静态导航图。`stair_lab_*.world` 是 **Gazebo SDF 场景库**，
不是 MuJoCo 地图，也不是导航 support_map；MuJoCo 走 MJCF/`make_stairs_xml` 生成链，不加载这些 world。

**NIST 链定位**：`nist_stairs_120_s020x170.world` + `nist_stairs_120_nav@1` + `app_runner` 是
**formal 前的受控回归/集成夹具**，用于验证局部台阶语义、地图死带、goal 选面、TF 和 3D 栈集成；
它**不签 A1—A7 formal**，也不代表四足物理步态或真机。R1121 的 39 点到 step5 是这条夹具的
scoped 读数，不是 formal/物理成功率。

**Rich world 不是替代品**：多层室内 world 可用于感知/SLAM/演示，但视觉/碰撞网格不等于 support_map；
换世界后仍要声明地面/踏面、step 边、z 和足迹交叠，低台阶高度只是参数之一，还需 >0.18 m 的负控。
推荐 NIST/m1 做 formal 前回归，另开一个许可明确的多层 demo 世界；真机/物理四足仍走 V8-X1/V8-X2。
只验证 SLAM 时不需要 support_map/app/step 语义——这些是导航链的输入；09 演示线不消费它们。

**当前多层 world**：已有受控两层 `demo_two_floor.world`（12×8 m 底层、1.6 m 上层、
10×0.16 m 楼梯、墙/家具特征），用于演示线 v1 的纯 SLAM survey；它**不是 rich 公开
多层建筑**，也没有 support_map/app 导航链。AWS Hospital 已做导入探针（R1124）：单层 182 模型、
两层 361、三层 540 可在 Gazebo 11 加载；R1125 大厅小环 + R1126 安全走廊 3 seeds 已跑单层 scoped，
两层/三层未跑；未接专用 launch/support_map。另有 4 个顶层世界
（m1/m1_v2/NIST 原始/NIST 缩放）＋9 个单层 stairstudy 场景＋2 个 NIST 导入件；NIST 的 6 层
是楼梯支撑面，不是多层楼板；
`two_level_boxes` 只是地图契约夹具。m1/NIST/stairstudy 仍是 Gazebo 导航测试世界。

**演示线（09）**：丰富/多层公开 world 单独做 demo/scoped，用于建图、定位（SLAM）算法效果；
演示结果带截图/录包/指标，但不签 formal，也不代表 Lite3 物理通行；流程与指标见官网
「仿真与界面 → 09 演示线」。v0 已跑通 `m1_world_v2` + faster-lio vs GT：20 s / 0.25 m/s，
translated xy mean **9.5 mm**、max **22.7 mm**，rigid yaw mean **5.3 mrad**；单次 scoped，
只测 SLAM，不跑导航/运动学。证据 `note/进度/v8/M5/26-11-03/evidence/R1122/`；AWS Hospital
先做导入探针（R1124），R1125 已跑大厅小环 scoped；R1126 实时 LiDAR 发现该环 min clearance
0.168 m（不合格），改跑 y=-8 安全走廊。两层/三层未跑，rich 多层仍不是正式 demo 世界。v1 受控两层
`demo_two_floor.world` + `demo_slam_survey.py`：35 s GT teleport survey，
恒定 yaw 版 translated xy mean **248.7 mm**、rigid yaw **2.4 mrad**；首轮 yaw 突变版
translated 409.9 mm／rigid yaw 2.97 rad 作为负例；LIO PCD 已留证。v2 AWS Hospital 单层大厅
（R1125）：8 m 小环、0.2 m/s、40 s GT teleport，3 次重复；translated xy mean
**0.355/0.451/0.460 m**、max **0.756/0.979/0.826 m**，z mean ~**0.138 m**，rigid yaw mean
~**10.1 mrad**；静止对照 **0.057 m**。v2b 安全走廊（R1126）：实时 LiDAR 先筛路径，R1125
大厅小环 min 0.168 m（9/41<0.35）不合格；改走 y=-8 走廊 3 条 8 m 段（safe A/B/C，min 0.52 m）；
translated xy mean **0.2695 m**、max **0.4175 m**，RPE trans **0.0423 m**、yaw **1.16 mrad**，
z mean **0.9713 m**、max 2.03 m；0 个 >1 s 间隔/xy>0.5 m 段。仍只测 SLAM，无 `/cmd_vel`、
无导航/运动学、非 formal。只验证 SLAM 时不需要 support_map/app/step 语义；
地图/楼梯语义缺口只影响 03 导航全链。

- **2D 交互**：`roslaunch lite3_gazebo_sim interactive_2d.launch rviz:=true`（Gazebo m1 + MBF +
  `mbf_goal_bridge`）；RViz 显示 `/map`、MID360 点云和 Lite3 模型，2D Nav Goal 走 MBF。
- **3D 交互（预生成图）**：`app_runner.py --app .../nist_sensor_3d.yaml --execute --interactive --rviz`；
  RViz 显示 support surface、Lite3 和路径，Publish Point 选面。
  **这张卡本身只做 3D 导航**：固定预生成 `nist_sensor@2` 图、`localization.source=carrier`、
  默认 NIST 台阶世界，因此它自己不建图、不启动 faster-lio、也没有数据集/世界选择菜单；
  **这不等于仓库不能做**。选世界用 `m2_gazebo.launch world:=... model_path:=...` 或 app YAML 的 `carrier.world*`；
  建图见 §4.7/§4.8；LIO 定位需 `nist_sensor_3d_lio.yaml` + 另起 faster-lio 源。
- **键盘建图 → 自建图 3D 导航（R1119）**：
  `m2_gazebo.launch gui:=true` + `mapping_recorder.py` + `rviz_mapping3d.rviz` +
  `keyboard_teleop.py`（四个独立进程）；记录器用 `/rslidar_points` + `/sim/odom_gt`
  写 PCD，再由 `v8_platform/framework/map_from_pcd.py` 生成 support_map/app/task，
  最后 `app_runner.py` 用自建图起 3D 交互导航。完整命令见 `docker/README.md` §4.7。
- 端口：2D/建图 11313/11345；3D app 11537/11538；`sim` 与 `sim-viz` 一次只起一个。
- 边界：R1119 是**仿真 GT 位姿建图**，不是 LIO/SLAM；自建图目前是一层平地支撑面，
  导航载体是 `app_carrier` 运动学箱体，不是四足物理步态/真机。
- **导入 NIST 世界（R1120）**：`m2_gazebo.launch world:=... model_path:=... spawn_*:=...`
  可带 Lite3 进导入世界；键盘采点/RViz 3D 点云可跑，但地面遥操看不到上层踏步。
- **声明 NIST 楼梯导航（R1121）**：对声明图跑 `inflate_support_map.py --margin .20`
  生成 `nist_stairs_120_nav@1`，再用 `app_runner.py --app
  .../nist_stairs_120_nav_v1_app.yaml --execute --interactive --rviz`；
  点 step5 后全局 39 点、局部 `arrived`、载体 `(0.075,1.275,0.890358)`，app 六项 checks 全 true。
  局部规划把声明台阶的下一踏面当可跨越边界（真低顶仍拒绝），并把 tick 调到 1.2 s、
  max_step .30 m、wall watchdog 90 s。详见 `docker/README.md` §4.8。
- 边界：R1121 仍是 `app_carrier` 运动学箱体 + 声明支撑面外扩，不是键盘建图直接产出的
  完整楼梯图、不是四足物理步态/真机；平层 m1 的 §4.7 链不受影响。
