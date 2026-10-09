# ws/ · 工作区定位图（catkin workspace）

**这里只放代码与配置。** 进度、验收结论、证据、台账全在 [`../note/`](../note/00_README.md)；
日志在 `../logs/`（`ROS_LOG_DIR` 已指过去）；建图产物在 `../data/maps/`。
`build/` 与 `devel/` 是编译产物，已在 `.gitignore` 里排除，**不要手工改、也不要入库**。

> 蓝本与硬规则的正文不在本文档里，只给指针：隔离与真机安全三条硬规则＝[`../note/00_README.md`](../note/00_README.md)；
> 施工蓝本＝[`../note/计划书v4/`](../note/计划书v4/)；什么改动被许可过＝[`../note/04_配置改动diff.md`](../note/04_配置改动diff.md)。

## 一、模块层与归属（一层一件事）

| 层 | 自研壳（`lite3_*`） | 第三方副本 | 定位要点 |
|---|---|---|---|
| `00_drivers` | `lite3_drivers`（`launch/01_sensors.launch` 真机、`launch/95_sim.launch` 仿真、`scripts/sim_adapter.py`） | `livox_ros_driver2`、`lslidar_driver/_msgs`、`message_transformer` | **`message_transformer/src/ros2qnx.cpp` 只许在真机 mode 出现**（规则一：stub/采图/回放里出现即事故）；`qnx2ros.cpp` 是 IMU 桥 |
| `10_slam` | `lite3_slam` | `faster_lio`（**唯一被许可的第三方源码改动**：存盘路径隔离改写）、`pcd2grid` | 采图链：驱动 → `/rslidar_points` → faster_lio（`run_mapping_online`，lidar_type=4） |
| `20_localization` | `lite3_localization` | `hdl_localization`、`hdl_global_localization`、`fast_gicp`、`ndt_omp` | 重定位与全局配准的候选件都在这里，装配选择见 `70_bringup` |
| `30_3rd` | —（无壳） | SCAN 上游五包：`plan_env`、`plan_manage`、`bspline_opt`、`path_searching`、`traj_utils` | **按基线不改**；需要改行为时在 `40_navigation/lite3_scan` 壳包里自持（批2 已定口径） |
| `30_perception` | `lite3_perception` | `deeprobotics_local_height_map`（带 `CATKIN_IGNORE`）、`libraries/{kindr,kindr_ros}` | 厂商高程图包是**闭源**（63 文件、0 代码件），其 `slope_threshold` 量纲不可核 ⇒ 见补丁 X5 |
| `40_navigation` | `lite3_navigation`（参数与插件装配）、`lite3_scan`（SCAN 集成） | — | **壳包内禁算法源码**（`.cpp/.cc/.h` 零命中，ISO 项[9]）；SCAN 参数由 `lite3_scan/launch/70_scan.launch` include 上游 `advanced_param.xml` 注入 |
| `50_task` | `lite3_task`（FSM＋`task_primitives`） | `pipeline_tracking` | 任务层不替上游造失败事件（到达判据实体在 `lite3_scan` 侧） |
| `60_safety` | `lite3_safety` | — | **`/cmd_vel` 唯一写者＝safe_cmd_vel_mux**（K2 红线：任何包绕过它直写即门禁红） |
| `70_bringup` | `lite3_bringup` | — | TOP 编排层，`<include>` 各平级模块是它的职责（K3 语义：平级 `lite3_*` 禁互 include，TOP 下行不豁免） |
| `90_supervisor` | `lite3_supervisor` | — | 心跳监管；**SUP 自身绝不写 `/cmd_vel`**（SUP 红线） |
| `95_simulation` | `lite3_gazebo_sim` | `Mid360_simulation_plugin`、`local_sensing`、`mockamap`、`map_generator`、`lite3_description`、`config` | 仿真域全部入口、世界件、验收件、离线预检件都在 `lite3_gazebo_sim/` 下（见下节） |

## 二、"要找什么，去哪儿"（按问题查，不按目录查）

| 想做的事 | 直接去这里 |
|---|---|
| 起仿真 / 换世界件 | `95_simulation/lite3_gazebo_sim/launch/{m1_gazebo,m2_gazebo}.launch`；`00_drivers/lite3_drivers/launch/95_sim.launch`（world 已 arg 化） |
| 看/改世界（**红线，勿改**） | `95_simulation/lite3_gazebo_sim/worlds/m1_world.world`（各跑批对它取 `world_before/after.sha`） |
| 机体模型 | `95_simulation/lite3_description/urdf/Lite3.urdf`（12 个 revolute，**无腿控制器**）；仿真实际用的是 `lite3_gazebo_sim/urdf/lite3_planar.urdf.xacro`（刚体盒子＋雷达插件参数在 :85–96，安装高 0.38m） |
| 仿真驱动／打戳／插件源码 | `95_simulation/lite3_gazebo_sim/src/lite3_planar_drive.cpp` |
| 冒烟、五连、退净判据 | `95_simulation/lite3_gazebo_sim/verify/`：`m2_smoke.sh`（**判据载体**，改动即换代际）、`m2_exitclean_lib.sh`、`m2_batch_runner.sh`、零节点标定件 `v_m2_exitclean_selftest.sh`、`v_g35_branch_calibration.sh`、`v_b3_timebase_contract.sh` |
| 楼梯可行性**离线预检**（不被任何 launch 引用） | `95_simulation/lite3_gazebo_sim/stairstudy/`：`fixtures.py`（八实体＋参数化 world）、`precheck.py`（分辨力矩阵＋T0–T9）、`sweep.py`（距离窗）、`bandstudy.py`（竖向带宽反事实）；产物 world 在同级 `worlds/stair_lab_*.world` |
| SCAN 的验收链 P1–P4 | `40_navigation/lite3_scan/verify/`：`v_scan_p1..p4.sh` 与被试 `p1_occ_sample.py`/`p2_goal_plan.py`/`p3_track.py`/`p4_replan.py`；三连独立件 `v_scan_p3_triple.sh` |
| SCAN 参数在哪 | `30_3rd/plan_manage/launch/advanced_param.xml`（如 `grid_map/resolution=0.05` 在 :51）；我方注入点 `40_navigation/lite3_scan/launch/70_scan.launch` |
| 导航代价图/规划器参数 | `40_navigation/lite3_navigation/config/`：`costmap_common.yaml`（`resolution: 0.1`）、`planner_teb.yaml`、`variants/`（三档覆盖文件，**当前无人加载**＝待裁 B5） |
| 安全仲裁与优先级 | `60_safety/lite3_safety/{launch,config,verify}` |
| 任务原语与注册表 | `50_task/lite3_task/{config,verify}` |
| 采图前体检 | `../scripts/verify/premap_check.sh`（`sudo` 侧，阻塞/警告两档 rc） |
| 门禁与自检件 | `../scripts/verify/`：`v3_isolation.sh`（[1]–[9]）、`v_sh_py_syntax.sh`、`v_harness_generation_table.sh --check`（代际对账）、`v_deferred_clause_scan.sh`（缓做条款巡检） |
| 属主/ACL 修复（需你在本机执行） | `../scripts/ops/b3_pre_chown.sh --apply/--verify` |

## 三、红线速记（违反任何一条 = 当日批跑读数作废）

- **`ros2qnx` 仅真机 mode**；采图全程不发 `/cmd_vel`。
- `/cmd_vel` 唯一写者是 M6 mux；`lite3_supervisor` 亦在扫描面内。
- 自研壳里禁出现开发侧家目录字面串（ISO 项[8] 扫全 `ws/src`，含第三方副本）。
- 40_navigation 壳包禁算法源码（ISO 项[9]）。
- 红线件本体：`worlds/m1_world.world`、样张 `../data/maps/lite3.pcd`（各跑前后取 sha）。
- 在册端口 `11311/11312/11313/11345`：跑前跑后都要退净，残留即判不洁（判据库 `m2_exitclean_lib.sh`）。
- 第三方包"原样复制、一字不改"，唯一许可＝`faster_lio` 存盘路径改写。

## 四、跑批前的调用姿势（先自证，再谈红绿）

    source /opt/ros/noetic/setup.bash
    source /home/ros1-slam/ws/devel/setup.bash
    # 或直接：source /home/ros1-slam/scripts/env.sh（含 foxy 清理＋devel 守卫）

**为什么必须写出来**：ISO 项[4] 用 `rospack find`，没 source 环境时会把"跑不动"报成"隔离违规"；
`catkin_make` 同理会 rc=127。任何门禁件失败，**先复核这两行**再判被测件——这是在册教训（已挂待裁 A2）。

## 五、`m2_smoke.sh` 一句话警告

它是**判据载体**：改它（或 `m2_exitclean_lib.sh`）＝代际加一行，五连必须重新起算；
只改仿真插件源码/`.so` **不**产生新代际行，那种变更要在 `../note/07_判据载体代际对照表.md` 落"被测件面注记"，
引用时必须连载体代际＋`.so` sha 一起引（两代证据不可混引）。
