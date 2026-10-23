# ws/ · 工作区定位图（catkin workspace）

**这里只放代码与配置。** 进度、验收结论、证据、台账全在 [`../note/`](../note/00_README.md)；
日志在 `../logs/`（`ROS_LOG_DIR` 已指过去）；建图产物在 `../data/maps/`。
`build/` 与 `devel/` 是编译产物，已在 `.gitignore` 里排除，**不要手工改、也不要入库**。

> 蓝本与硬规则的正文不在本文档里，只给指针：隔离与真机安全三条硬规则＝[`../note/00_README.md`](../note/00_README.md)；
> 现行施工规划指针＝[`../note/00_README.md`](../note/00_README.md)，本次落册见 [v8 计划入口](../note/计划书v8/README.md)；什么改动被许可过＝[`../note/04_配置改动diff.md`](../note/04_配置改动diff.md)。
> 下表描述既有工作区。v8 的组件平台、多层规划和新仿真合同尚待实现，本次只更新规划与入口。

## 一、模块层与归属（一层一件事）

> **路径读法**：本文所有相对路径以**所在行的层/包为根**（如 `00_drivers` 行里的 `launch/01_sensors.launch` 即 `ws/src/00_drivers/lite3_drivers/launch/…`）；写作 `ws/…`、`note/…`、`data/…`、`scripts/…` 的以仓库根为起点；`/cmd_vel` 这类以 `/` 开头的是 ROS 话题名。跨层引用一律写全路径。

| 层 | 自研壳（`lite3_*`） | 第三方副本 | 定位要点 |
|---|---|---|---|
| `00_drivers` | `lite3_drivers`（`launch/01_sensors.launch` 真机、`launch/95_sim.launch` 仿真、`scripts/sim_adapter.py`） | `livox_ros_driver2`、`lslidar_driver` 与 `lslidar_msgs`、`message_transformer` | **`message_transformer/src/ros2qnx.cpp` 只许在真机 mode 出现**（规则一：stub/采图/回放里出现即事故）；`qnx2ros.cpp` 是 IMU 桥 |
| `10_slam` | `lite3_slam` | `faster_lio`（**唯一被许可的第三方源码改动**：存盘路径隔离改写）、`pcd2grid` | 采图链：驱动 → `/rslidar_points` → faster_lio（`run_mapping_online`，lidar_type=4） |
| `20_localization` | `lite3_localization` | `hdl_localization`、`hdl_global_localization`、`fast_gicp`、`ndt_omp` | 重定位与全局配准的候选件都在这里，装配选择见 `70_bringup` |
| `30_3rd` | —（无壳） | SCAN 上游五包：`plan_env`、`plan_manage`、`bspline_opt`、`path_searching`、`traj_utils` | **按基线不改**；需要改行为时在 `40_navigation/lite3_scan` 壳包里自持（批2 已定口径） |
| `30_perception` | `lite3_perception` | `deeprobotics_local_height_map`（带 `CATKIN_IGNORE`）、`libraries/{kindr,kindr_ros}` | 厂商高程图包是**闭源**（63 文件、0 代码件），其 `slope_threshold` 量纲不可核 ⇒ 见补丁 X5 |
| `40_navigation` | `lite3_navigation`（参数与插件装配）、`lite3_scan`（SCAN 集成） | — | **壳包内禁算法源码**（`.cpp/.cc/.h` 零命中，ISO 项[9]）；SCAN 参数由 `lite3_scan/launch/70_scan.launch` include 上游 `advanced_param.xml` 注入 |
| `50_task` | `lite3_task`（FSM＋`task_primitives`） | `pipeline_tracking` | 任务层不替上游造失败事件（到达判据实体在 `lite3_scan` 侧） |
| `60_safety` | `lite3_safety` | — | **`/cmd_vel` 唯一写者＝safe_cmd_vel_mux**（K2 红线：任何包绕过它直写即门禁红） |
| `70_bringup` | `lite3_bringup` | — | TOP 编排层，`<include>` 各平级模块是它的职责（K3 语义：平级 `lite3_*` 禁互 include，TOP 下行不豁免） |
| `90_supervisor` | `lite3_supervisor` | — | 心跳监管；**SUP 自身绝不写 `/cmd_vel`**（SUP 红线） |
| `95_simulation` | `lite3_gazebo_sim`、`lite3_description`、`config/`（自研侧仿真参数 `faster_lio_sim.yaml` ＋ 语义标注 `config/passability/`）、`tools/passability/`（标注工具，非 catkin 包） | `Mid360_simulation_plugin`、`local_sensing`、`mockamap`、`map_generator` | 仿真域全部入口、世界件、验收件、离线预检件都在 `lite3_gazebo_sim/` 下（见下节） |

## 二、"要找什么，去哪儿"（按问题查，不按目录查）

| 想做的事 | 直接去这里 |
|---|---|
| 起仿真 / 换世界件 | `95_simulation/lite3_gazebo_sim/launch/{m1_gazebo,m2_gazebo}.launch`；`00_drivers/lite3_drivers/launch/95_sim.launch`（world 已 arg 化） |
| 看/改世界（**红线，勿改**） | `95_simulation/lite3_gazebo_sim/worlds/m1_world.world`（各跑批对它取 `world_before.sha`／`world_after.sha`） |
| 机体模型 | `95_simulation/lite3_description/urdf/Lite3.urdf`（12 个 revolute，**无腿控制器**）；仿真实际用的是 `95_simulation/lite3_gazebo_sim/urdf/lite3_planar.urdf.xacro`（刚体盒子＋雷达插件参数在 :85–96，传感器安装高在 :55（`origin xyz="0 0 0.38"`）） |
| 仿真驱动／打戳／插件源码 | `95_simulation/lite3_gazebo_sim/src/lite3_planar_drive.cpp` |
| 冒烟、五连、退净判据 | `95_simulation/lite3_gazebo_sim/verify/`：`m2_smoke.sh`（**判据载体**，改动即换代际）、`m2_exitclean_lib.sh`、`m2_batch_runner.sh`、零节点标定件 `v_m2_exitclean_selftest.sh`、`v_g35_branch_calibration.sh`、`v_b3_timebase_contract.sh` |
| 楼梯可行性**离线预检**（不被任何 launch 引用） | `95_simulation/lite3_gazebo_sim/stairstudy/`：`fixtures.py`（八实体＋参数化 world）、`precheck.py`（分辨力矩阵＋T0–T9）、`sweep.py`（距离窗）、`bandstudy.py`（竖向带宽反事实）；产物 world 在同级 `worlds/stair_lab_*.world` |
| SCAN 的验收链 P1–P4 | `40_navigation/lite3_scan/verify/`：`v_scan_p1.sh`…`v_scan_p4.sh` 与被试 `p1_occ_sample.py`/`p2_goal_plan.py`/`p3_track.py`/`p4_replan.py`；三连独立件 `v_scan_p3_triple.sh` |
| SCAN 参数在哪 | `30_3rd/plan_manage/launch/advanced_param.xml`（如 `grid_map/resolution=0.05` 在 :51）；我方注入点 `40_navigation/lite3_scan/launch/70_scan.launch` |
| 导航代价图/规划器参数 | `40_navigation/lite3_navigation/config/`：`costmap_common.yaml`（`resolution: 0.1`）、`planner_teb.yaml`、`variants/`（三档覆盖文件，**当前无人加载**＝待裁 B5） |
| 标「这级楼梯能不能上」（语义标注） | `ws/src/95_simulation/tools/passability/passability.py`（`check`/`build`/`preview`/`diff`/`verify-built`/`selftest`）；源标注写在 `config/passability/passability.yaml`，**生成物只读**；说明见 `note/进度/仿真测试台/设计稿/I_方向四先行子集_语义标注通道设计落稿.md` |
| 安全仲裁与优先级 | `60_safety/lite3_safety/{launch,config,verify}` |
| 任务原语与注册表 | `50_task/lite3_task/{config,verify}` |
| 采图前体检 | `scripts/verify/premap_check.sh`（在仓库根下）（`sudo` 侧，阻塞/警告两档 rc） |
| 门禁与自检件 | `scripts/verify/`：`v3_isolation.sh`（[1]–[9]）、`v_sh_py_syntax.sh`、`v_harness_generation_table.sh --check`（代际对账）、`v_deferred_clause_scan.sh`（缓做条款巡检） |
| 属主/ACL 修复（需你在本机执行） | `scripts/ops/b3_pre_chown.sh --apply/--verify` |

## 二·b、包级 README 索引（统一头，2026-10-09 起）

**12 个自研壳/描述包各有一份 `README.md`，开头是同一块「统一索引头」**（层·角色·一句话 → 全局索引 → 本包速查表 → 红线契约）；要看某个包从哪儿起、验收件在哪、哪条红线管着它，先读它自己的 README，不必回到本文档翻。

| 包 | 角色 | README |
|---|---|---|
| `00_drivers/lite3_drivers` | M0 驱动与传感接入 | [README](src/00_drivers/lite3_drivers/README.md) |
| `10_slam/lite3_slam` | M1 建图（LIO） | [README](src/10_slam/lite3_slam/README.md) |
| `20_localization/lite3_localization` | M2 重定位/配准 | [README](src/20_localization/lite3_localization/README.md) |
| `30_perception/lite3_perception` | M3 感知（R280 起非空壳：含自研 python 表征件 `scripts/w3_elevation_node.py`；第三方算法源码仍禁，门禁 `[9]` 域不受触——见该包 README 的 R280 更正节） | [README](src/30_perception/lite3_perception/README.md) |
| `40_navigation/lite3_navigation` | M4 导航联邦（mbf） | [README](src/40_navigation/lite3_navigation/README.md) |
| `40_navigation/lite3_scan` | SCAN 集成壳 | [README](src/40_navigation/lite3_scan/README.md) |
| `50_task/lite3_task` | M5 任务编排 | [README](src/50_task/lite3_task/README.md) |
| `60_safety/lite3_safety` | M6 安全仲裁（`/cmd_vel` 唯一写者） | [README](src/60_safety/lite3_safety/README.md) |
| `70_bringup/lite3_bringup` | TOP 顶层编排 | [README](src/70_bringup/lite3_bringup/README.md) |
| `90_supervisor/lite3_supervisor` | SUP 心跳监管 | [README](src/90_supervisor/lite3_supervisor/README.md) |
| `95_simulation/lite3_gazebo_sim` | SIM 仿真域本体（含判据载体与离线预检旁路） | [README](src/95_simulation/lite3_gazebo_sim/README.md) |
| `95_simulation/lite3_description` | DESC 机体描述（URDF/mesh） | [README](src/95_simulation/lite3_description/README.md) |

**故意不动的第三方副本 README**（保持上游原文，属"一字不改"承诺面）：`00_drivers/livox_ros_driver2`、`10_slam/pcd2grid`、`20_localization/{fast_gicp,hdl_global_localization,hdl_localization,ndt_omp}`、`95_simulation/Mid360_simulation_plugin`。
**无 README 的一片**：`30_3rd/`（SCAN 上游五包，按基线不改，参数落点见上表 §二）与 `30_perception/libraries/`（kindr 系）。

## 三、红线速记（违反任何一条 = 当日批跑读数作废）

- **`ros2qnx` 仅真机 mode**；采图全程不发 `/cmd_vel`。
- `/cmd_vel` 唯一写者是 M6 mux；`lite3_supervisor` 亦在扫描面内。
- 自研壳里禁出现开发侧家目录字面串（ISO 项[8] 扫全 `ws/src`，含第三方副本）。
- 40_navigation 壳包禁算法源码（ISO 项[9]）。
- 红线件本体：`95_simulation/lite3_gazebo_sim/worlds/m1_world.world`、样张 `../data/maps/lite3.pcd`（各跑前后取 sha）。
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

## 六、各算法的存储地址与独立性（2026-10-10 实测核对；**为"以后各自能单独升级"而写**）

> 核对方式：`package.xml` 依赖面逐包扫、跨包 `#include` 扫、launch 的 remap 接缝扫。
> **结论不是设计意图的复述，是查出来的**——查不动的地方（编译进 `.so` 的厂商件）一律标"未证"。

### 6.1 独立性结论（能单独换算法吗：能，但有三处接缝要一起改）

| 检查项 | 实测结果 |
|---|---|
| 算法核之间有没有**代码级耦合**（跨包 `#include` 对方的头） | **无**。`faster_lio`、`hdl_localization`、`30_3rd/*`（SCAN 规划器五包）、`deeprobotics_local_height_map` 各自只 include 自己包内的头（例：`plan_env/src/grid_map.cpp:1` 只引 `plan_env/grid_map.h`） |
| 算法核之间有没有**编译期耦合**（`package.xml` 互相 depend） | **无**。三个算法核互不依赖 |
| 那它们靠什么连起来？ | **只有 ROS 话题与 TF**（下表 §6.2 的"接缝"列）⇒ 换算法＝换话题名/消息格式，不动对方代码 |
| 唯一的例外（**壳→壳的构建顺序耦合，不是代码耦合**） | `lite3_scan` 的 `package.xml` 依赖 `lite3_slam`＋`lite3_gazebo_sim`；`lite3_gazebo_sim` 依赖 `lite3_slam`＋`lite3_drivers`＋`livox_laser_simulation`；`lite3_localization` 依赖 `hdl_localization` ⇒ **换算法核时这些壳包要一起重编**，但不会传染到别的算法 |
| 未证 | `livox_laser_simulation.so`、厂商 `deeprobotics_*` 件的内部耦合（无源码，看不见） |

### 6.2 存储地址总表（**每条都写"换它时要动哪几处"**）

| 干什么 | 算法包（存储地址） | 编译状态 | 换掉它要一起改的接缝 |
|---|---|---|---|
| 激光-惯性里程计／建图（**在用**） | `ws/src/10_slam/faster_lio/` | ✅ 编译 | 入：`/rslidar_points`、`/livox/imu`；出：`/cloud_registered`、`/cloud_registered_body`、`/Odometry`、TF `camera_init→body`；参数在 `10_slam/lite3_slam/config/faster_lio_outdoor.yaml`（**注意：仿真链加载的也是这一份**，见 `m2_mapping.launch:7`；`95_simulation/config/faster_lio_sim.yaml` 只有 `10_slam_mapping_sim.launch` 会用） |
| 点云转栅格（离线工具，也碰高程） | `ws/src/10_slam/pcd2grid/` | ✅ 编译 | 只吃 `.pcd`＋`config.yaml`，不接在线话题 |
| 激光重定位（**备用定位路线**） | `ws/src/20_localization/hdl_localization/`、`hdl_global_localization/`、`ndt_omp/`、`fast_gicp/` | ✅ 编译 | 入 `/velodyne_points`＋初始位姿；出 `/hdl_odom`（壳包 `lite3_localization` 转发） |
| 局部／全局轨迹规划（**在用**） | `ws/src/30_3rd/{plan_env,plan_manage,bspline_opt,path_searching,traj_utils}/`（包名 `scan_planner`） | ✅ 编译 | 入：`/cloud_registered`（经 relay）＋`/map_generator/global_cloud`；出：`/grid_map/occupancy`、`/planning/bspline`、`/scan_planner/cmd_vel`；参数 `plan_manage/param/advanced_param.xml` |
| 导航编排（MBF 联邦） | `ws/src/40_navigation/lite3_navigation/` | ✅ | 话题 `/move_base_flex/*`；配置 `config/*.yaml` |
| 安全与指令仲裁 | `ws/src/60_safety/lite3_safety/` | ✅ | **`/cmd_vel` 唯一写者**（红线） |
| 任务编排 | `ws/src/50_task/lite3_task/` | ✅ | 原语／action 接口 |
| 仿真载体与传感器 | `ws/src/95_simulation/{lite3_gazebo_sim,Mid360_simulation_plugin,local_sensing,map_generator,lite3_description}/` | ✅ | 世界件 `lite3_gazebo_sim/worlds/m1_world.world`（**sha 钉住**） |
| 监管 | `ws/src/90_supervisor/lite3_supervisor/` | ✅ | 心跳／`/cmd_vel` 归属面 |

### 6.3 **已搁置：2.5D／高程方向**（搁置 ≠ 删除；**这些地址里的东西不许清理、不许改**）

主理人 2026-10-10 裁定：**2.5D 高程方向暂时搁置，留后期备用**。它的东西分散在四处，恢复时按这张表找：

| 是什么 | 存储地址 | 现状 |
|---|---|---|
| **载体包**（厂方高程图／局部高度图节点） | `ws/src/30_perception/deeprobotics_local_height_map/` | **`CATKIN_IGNORE` ＋ `.cpp/.h` 数为 0**（上游源码缺失）⇒ 编译不出、无对象可挂；包内 `launch/config/rviz` 骨架原位保留，**不要删** |
| **设计稿**（我方 2.5D 方案与判据） | `note/进度/仿真测试台/设计稿/C_全局任务优先级总排序.md`（#6 行）、`…/F_多层导航起工设计_落稿不实现.md`（**§7 的"2.5D 单高度假设"就在这条链上**）、`…/I_方向四先行子集_语义标注通道设计落稿.md`（第四通道＝高程先验注入）；#6 设计稿本体在开发侧对照库 | 落稿状态，未施工 |
| **预检件与结论**（实测过，且正是它推翻 A 档前提） | `note/进度/仿真测试台/证据/raw/r59_w3_precheck_20261009_0522/`（`预检结论.md` ＋ `probe/` 15 件：双 StaticLayer「覆盖不叠加」的四格对照在此）；`note/进度/仿真测试台/证据/log/matrix_w3/spike3d_elevation_costmap_design.md`（3D spike 设计）；可复跑对账件 `scripts/verify/v_w3_terrain_design_contract.sh`；设计档 `note/进度/SCAN集成/设计/m_3d多层_载体缺口与出路.md`（**含 §0 更正节**） | 证据在册、sha 可验 |
| **配置键面**（挂高程层要动的地方） | `ws/src/40_navigation/lite3_navigation/config/mbf_navigation.yaml`（双 StaticLayer 挂载位）、`costmap_common.yaml`（`resolution: 0.1`）、`ws/src/30_3rd/plan_manage/param/advanced_param.xml`（`grid_map/resolution=0.05`）；`scan_3d` 第 4 档键面**尚未入库**（variants 现仅三档） | 键在、消费者缺 |

**恢复入口（写死，免得以后重新考古）**：
1. 先读 `note/进度/SCAN集成/设计/m_3d多层_载体缺口与出路.md` 的 §0——它分清了一件容易混的事：**"多楼层" ≠ "悬挑/桥洞"**。前者可用"一层一图＋切图"实现且**不需要撤 §7**；后者才必须撤 §7。
2. 再确认两个前置：`#46`（双 StaticLayer 覆盖不叠加，A 档前提已被上面那份预检推翻）路线是否已选；`deeprobotics_local_height_map` 的上游源码是否补齐。
3. 判据侧连带项：请求单 **C12**（撤不撤 §7）、**A8/A12/B6/C6**（第四通道阈值与可感知距离窗）。
4. **载体边界提醒**：即便高程层做出来，在册仿真载体是刚体盒子 ⇒ 只能验到"规划层"，**"狗能不能上这段楼梯"最终仍必须真机**（补丁 X7／G 稿）。

### 6.3-补（R205 更正，**另起句；上面那张表与"恢复入口"四条原文不回改**）

三条更正都来自本轮只读复核（留底 `note/进度/SCAN集成/证据/log/raw/r205_role_reviews/`），逐条回过原件：

1. **路径写错**：上表"配置键面"那格把 `advanced_param.xml` 记在 `plan_manage/param/` 下；`find ws/src -name advanced_param.xml` 实测**只有 `ws/src/30_3rd/plan_manage/launch/advanced_param.xml` 一处**（livox 侧另有同名件，与本条无关）。⇒ 恢复时**按本条地址找**，别照上表那条走空。
2. **我高读了 M 稿**：上面第 1 条恢复入口写「M 稿 §0 分清了"多楼层 ≠ 悬挑/桥洞"」——回原件读 §0 表格原文，那一行是把「多楼层（跨层、悬挑、桥洞）」**三件事并作一行**写的，**并未分清**。⇒ 该区分**是我方的判断、不是源文件的结论**；引用它时须按"我方推断"标注，不得写成"§0 已分清"。
3. **"被预检推翻"要降档**：上表把这份预检写成「正是它推翻 A 档前提」，第 2 条恢复入口写「A 档前提已被上面那份预检推翻」。格级复验后应读作「**在册配置下未证实可叠加**」：四格对照里**只有"换序"那一格留有原始读数件**（`probe/cost3.txt`），承载"覆盖"结论的那格盘上无原件（`cost.txt`/`cost2.txt` 是同 sha 的 76 字节 `WARNING: topic … does not appear`）；且探针面为 `track_unknown_space: true`、`combination_method` 是 a=0/b=1 的**混配**，而本部署该键全为 `false` ⇒ **结论不迁移到本部署**，待补测三格见计划书 v4.5 候选稿 Part 4 的 4.2。

### 6.4 一条纪律（为"以后各自升级"服务）

新增或替换算法时：**一算法一独立包、只经话题/TF 相接、不得跨包 `#include` 对方头文件、不得在 `package.xml` 里 depend 另一个算法核**（壳包 depend 算法核是允许的方向，**反向禁止**）。
现状：这条**基本成立**（§6.1 是实测），但**还没有机器可查的闸**——`v3_isolation.sh` 的 [5] 只查 launch 互 include、[9] 只查 `40_navigation` 下的算法源码文件，**跨包 `#include` 与 `package.xml` 互 depend 目前无人守**。补这条检查属门禁变更，需小美裁定后落地（可与 #54／A15 并案）。

**§6.3-补 追加（R210，主理人 2026-10-11 明文；上面三条更正原文照旧不回改）**：上面第 2 条提到的"多楼层 ≠ 悬挑/桥洞"这一族，本轮又冒出一个**第四解法**的说法（"每层各挂一张单高度／2.5D 图＋切图"，理由是 §7 断的是"**单张**表达不了多层"、并未禁"**每层一图**"）。主理人裁定＝**如果它只和 2.5D 地图相关，就不做**；回原件核过，该解法的字面载体**正是 2.5D 高程图**，而本方向已按 C12「暂时搁置、放着存档即可」。⇒ **此处只登记为"存档事实"，不是解法清单里的第四条、也不进路线图**；恢复该方向时仍按上面第 1 条的地址找，但**别把这句读成"已有免撤 §7 的路"**——那条路要不要走，仍要另一次明文。

---

## ★同步戳（追加节·ws；上方旧文一字未回改）★

- **对齐到**＝进度真身末号 **R518** ／ 本 boot **批174**（零窗·派单批）。
  （★本文件档头戳停在 **R280／2026-10-11**；上方正文照旧、**不回改**，
    由本节把「当前对齐到哪一批」说清楚。）
- 本目录＝catkin 工作区＝**源码面**：只放代码与配置；
  进度／验收／证据／台账全在 [`../note/`](../note/00_README.md)；
  日志在 `../logs/`（`ROS_LOG_DIR` 已指过去、**运行时 `mkdir -p` 生成、已 gitignore**）；
  建图产物在 `../data/maps/`。
- ★本目录不入库的＝`build/` `devel/`（编译产物，红线：不手工改、不入库）。
- 同步时刻＝2026-10-21 03:32 CST。
