# 官方 8.2.1 导航启动链 → 本项目真机适配（EVT 前置准备）

状态：**真人已于 2026-11-09 16:17 授权「允许进入 EVT 段」**；本批据此把阶段标记推进到 EVT，
并完成 G0／G1 的离线开工件。入场是阶段放行，**不等于 G2 及以后各门自动放行**
（`Runbook:4` 仍要求作业窗＋单项作业单）。本件不构成单项作业许可，不代替权利方签字，
不产生 PASS/FAIL。所有官方数值均为只读勘察读数，一律标为「禁止回填」。

采集时间 2026-11-09；采集方式＝对 `192.168.1.103`（hostname `lite`，用户 `ysc`）的**只读** SSH
勘察，未写远端、未下载官方源文件、未启动任何官方脚本、未执行任何策略或二进制。

★路径记法★：下文 `<VENDOR_STACK>` 是官方主机生产栈根的抽象占位。绝对字面路径属于
只读勘察读数，按 v5.3 隔离规矩**不写入 `ws/src`**（仅 `note/` 来源记录与 `opt/ref` 可写），
逐字原文完整保存在 `~/.cache/vendor-nav-evt-20261109/official_readonly_snapshot.txt`（9 节 260 行）。

---

## 一、是否进入 EVT 段：结论与依据

**结论：已获授权，进入 EVT 段。** 真人 2026-11-09 16:17 明确「允许进入 EVT 段」，
按 `:211` 默认动作＝推进与 `:214` 代勾边界，本批把阶段标记从「前置准备」推进到 **EVT**，
并落地 G0／G1 的离线开工件（门控模型＋空白参数对照件）。

★但入场 ≠ 逐门放行★：EVT 段自身的门控结构由 `Runbook:4` 与 `note/09_跨工具协作契约.md:213`
规定。真人授权的是**阶段入场**，而 G2「作业窗与单项作业单」、G3「外参实测」、
G4「真机执行」、G5「判据面签字」各自仍是在册硬约束下的逐项放行点；中标 L2 四类中的
「真机及其参数」「红线（真机／旧判据门值／历史证据）」「官方表述笔权」。

⇒ 本批是**授权面内的最大推进**：把不需要真机输入的 G0、G1 全部做完（含
`SIM_TO_REAL_PARAM_SHEET.yaml.template` 这一**机器可审计的签值骨架**，与既有的
`note/11_仿真到实机参数迁移表.md` 人类可读迁移规则草案互补；口径见第七节补记），
并把 G2—G5 各自的证据要求机器可读化（`framework/evt_gates.py`），
真人只需按门给值／勾放行即可继续。
若真人认为「允许进入」应解释为同时放行 G2—G5，一句话即可纠正，本模型可撤销、不留既成事实。

### EVT 门控表（机器可读版＝`evt_gates.py` 的 `GATES`）

| 门 | 名称 | 放行权 | 进入条件 | 本批状态 |
|---|---|---|---|---|
| G0 | 离线设计冻结 | self（非 L2） | 官方链契约＋影子就绪门＋定向测试 | **已开**（三件齐、41 单测全绿） |
| G1 | 仿真→真机参数对照件就绪 | L2 | 模板在位＋签值 | 模板已落（全 null），**待签值** |
| G2 | 作业窗与单项作业单批准 | L2 | 作业窗／作业单／急停／看护 | 未给 |
| G3 | 现场预检与外参实测 | L2 | preflight rc0／外参实测／对照件签值 | 未给 |
| G4 | 真机执行 | L2 | 作业窗／作业单／预检／enable_motion 放行 | 默认关 |
| G5 | 判据面签字与正式验收 | L2 | 判据签字／正式验收措辞 | 未签 |

★`Runbook:230` 原写「外参／TF／速度上限／地图版本无仿真→真机对照件」，应读作
「无**已填真机值、经现场复核的生效**对照件」：`note/11_仿真到实机参数迁移表.md`
（v0.1 草案，2026-10-28，182 行）早已给出逐项迁移规则，只是真机值全标「待实测」。
本批 `SIM_TO_REAL_PARAM_SHEET.yaml.template` 补的是**机器可审计的签值骨架**，
不是填值件、也不宣称首创；填任何 `real_value` 即真机回填，须 L2＋单项作业单，
`audit_param_sheet()` 会当场报红。两份件都仍是草案/空白态，缺口**未闭合**。★

### 现场段尚缺的前置（逐条可核）

| # | 前置 | 在册证据 | 状态 |
|---|---|---|---|
| 1 | 真人批准的作业窗与单项作业单 | `Runbook:4`；`mode_navigation.launch` 头注「开工前置 1」 | 未给（G2） |
| 2 | 外参实测（`base_link→rslidar`） | `hdl_localization.launch:31—35` 仍为 0 占位；G8-2 判据「恒等阵＝不通过」 | 未量（G3） |
| 3 | 仿真→真机参数迁移对照件 | `Runbook:230`「无对照件」＝无已填真机值的生效件；`note/11` 已有规则草案 | 签值骨架已补（与 note/11 互补），**待签值**（G1） |
| 4 | formal 与真机签署 | `note/进度/00_总览.md:36,40` | 未签（G5） |
| 5 | 急停可达、看护在场、机器人站立 | `mode_navigation.launch` 头注「开工前置 3」 | 现场条件（G2） |
| 6 | `enable_motion` 与 `ros2qnx` 真机链路 | `mode_navigation.launch:29` 默认 false；`boundary/protocol.py:3-4` | 默认关（G4） |
| 7 | 判据门值不得现场改 | `Runbook:4`／`:15`／`:236` | 硬约束 |

---

## 二、官方 8.2.1 三段启动链（只读勘察，逐字可查）

官方手册截图＝「8.2.1 单目标点定位导航使用方法」。三步：

### 第 1 步：启动对应雷达驱动

```
cd ~/lite_cog_ros2/system/scripts/lidar
./start_lslidar.sh     # Leishen
./start_livox.sh       # Livox
./start_rslidar.sh     # Robosense
```

`103` 实机 `system/scripts/lidar/` 只有 **2 个**脚本：

- `start_livox.sh`（119 B）：`source .../driver/mid360_ws/install/setup.bash` →
  `ros2 launch livox_ros_driver2 msg_MID360s_launch.py`
- `start_lslidar.sh`（117 B）：`source .../driver/leishen_ws/install/setup.bash` →
  `ros2 launch lslidar_driver lslidar_c16_launch.py`

★手册列了 `start_rslidar.sh`，**实机不存在**。这是官方文档与实机装配的漂移，不能据此推断
Robosense 链路可用，也不能把它当作项目待办。★

### 第 2 步：启动定位导航节点

```
cd <VENDOR_STACK>/system/scripts/nav
./start_nav.sh
```

`start_nav.sh`（178 B，2024-08-12）全文三行：

```
source <VENDOR_STACK>/navigation2-foxy/install/setup.bash
source <VENDOR_STACK>/nav/install/setup.bash
ros2 launch hdl_localization lite_localization.launch.py
```

`nav/src/hdl_localization/launch/lite_localization.launch.py` 的关键读数：

- `debug = False` ⇒ `map_server`／`rviz` 分支**不启动**；实际起的是
  `hdl_localization_composition` ＋ `dr_nav2.launch.py`。
- `hdl_localization_composition`：`globalmap_pcd=<VENDOR_STACK>/system/map/lite3.pcd`、
  `reg_method=NDT_OMP`、`ndt_neighbor_search_method=DIRECT1`、
  `ndt_neighbor_search_radius=3.0`、`ndt_resolution=1.5`、`downsample_resolution=0.5`、
  `use_imu=True`、`cool_time_duration=2.0`、`enable_robot_odometry_prediction=False`、
  `odom_child_frame_id=base_link`、`convert_utm_to_local=True`、
  `specify_init_pose=True`（初位姿写死 0,0,0,w=1）、`use_global_localization=False`、`t_diff=0.25`。
- remap：`/velodyne_points → /rslidar_points`；`/gpsimu_driver/imu_data → /imu/data`。
- ★`launch_base2lidar_tf_broadcaster()` 定义了外参
  `['0.16','0.0','0.47','1.57','0.0','0.0','base_link','rslidar']`，但
  `generate_launch_description()` **从未调用它**；`dr_nav2.launch.py` 里同款
  `launch_static_tf_broadcaster()` 的 `ld.add_action(...)` 被整行注释。
  ⇒ **两处外参 broadcaster 都是死代码，官方真机栈并不发布这组外参**。
  这正解释 `note/02_问题与坑.md:62`「官方 URDF 全树未找到、外参数值只能现场量」。★

`nav/src/dr_nav2/launch/dr_nav2.launch.py`：`nav2_bringup/bringup_launch.py`，
`map=<VENDOR_STACK>/system/map/lite3.yaml`、
`params_file=dr_nav2/config/lite_nav2.yaml`、`use_sim_time=False`、`autostart=True`、
`map_subscribe_transient_local=True`，另起 `rviz2`。

`lite_nav2.yaml`（594 行）关键项：`global_frame: map`、`robot_base_frame: base_link`、
`odom_topic: /leg_odom2`、`cmd_vel_topic: /cmd_vel_raw`、costmap `resolution: 0.10`、
plugins `static_layer + stvl_obstacle_layer + inflation_layer`、`inflation_radius: 0.40`、
stvl `topic: /rslidar_points`、DWB `max_vel_x 1.0 / max_vel_y 0.2 / max_vel_theta 0.8`。
BT 插件表含 `nav2_initial_pose_received_condition_bt_node` 与
`nav2_reinitialize_global_localization_service_bt_node`。

★官方 yaml 内部 `use_sim_time` 自相矛盾：`:3/:33/:145/:257` 为 `False`，`:419` 为 `True`，
而 launch 显式传 `False`。以 launch 为准，不照搬 yaml 内部值。★

地图：`system/map/lite3.yaml`＝`lite3.pgm`，`resolution: 0.05`，
`origin: [-3.55, -7.05, 0]`，`occupied_thresh 0.65`，`free_thresh 0.196`，`mode: trinary`；
`lite3.pcd` 166,744,189 B（2026-11-07 16:15 更新），另有 `lite3_bad_backup.pcd` 13 MB。

### 第 3 步：开启导航后对机器狗定位进行初始化

手册第 3 步正文在截图中被截断。仓内可核的官方机制有四：

1. `lite_localization.launch.py` 的 `specify_init_pose=True`＋硬编码原点；
2. `lite_nav2.yaml:18` 的 `nav2_initial_pose_received_condition_bt_node`；
3. `dr_nav2/rviz/dr_nav2.rviz:367` 的 `Value: /initialpose`（2D Pose Estimate 工具）；
4. `hdl_localization/README.md:45`「The estimated pose can be reset using 2D Pose Estimate on rviz」。

⇒ **第 3 步是本项目 ROS1 链真正缺的一环**：`hdl_localization.launch` 有
`specify_init_pose=true`，但 `mode_navigation.launch` 没有任何「显式初位姿／定位收敛／地图就绪／
里程计链就绪」的开工门，`/initialpose` 只在历史 `m2_smoke.sh` 验证脚本里被注入。

---

## 三、接口对照与禁止回填清单

官方栈＝**ROS2 Foxy + Nav2 + hdl_localization_composition**；本项目＝**ROS1 Noetic +
mbf_costmap_nav + safe_cmd_vel_mux**。两者不是同一框架，**接口名可比、数值不可搬**。

| 接口 | 官方值 | 项目现值 | 处置 |
|---|---|---|---|
| `points_topic` | `/rslidar_points` | `/rslidar_points` | 已对齐 |
| `global_frame` / `robot_base_frame` | `map` / `base_link` | `map` / `base_link` | 已对齐 |
| `costmap_resolution` | `0.10` | `0.1` | 数值相等 |
| `specify_init_pose` | `True` | `true` | 语义相同，但项目缺开工门 |
| `imu_topic` | `/imu/data` | `/imu/data`（faster_lio 用 `/livox/imu`） | **禁回填**：Q12 端口冲突 |
| `odom_topic` | `/leg_odom2` | `/hdl_odom`，不做腿式里程预测 | **禁回填** |
| `cmd_vel_topic` | `/cmd_vel_raw`（Nav2 直出） | `mbf → /cmd_vel_raw → mux → /cmd_vel → ros2qnx` | **禁回填**：`Runbook:236` 禁止绕过 mux |
| `autostart` | `True` | `enable_motion` 默认 false | **禁回填**：项目默认更安全 |
| `use_sim_time` | launch `False`，yaml 内部相左 | 项目用 `/clock` 门控 | **禁回填** |
| `inflation_radius` | `0.40` | `0.2` | **禁回填**：costmap 门值 |
| `ndt_resolution` | `1.5` | `2.0` | **禁回填**：定位判据门值 |
| `downsample_resolution` | `0.5` | `1.0`（globalmap_server 0.1） | **禁回填**：判据门值 |
| `max_vel_x` | `1.0` | `cmd_vel_mux.yaml max_linear_mps 0.5` | **禁回填**：速度上限 |
| `base_link→rslidar` | `(0.16,0,0.47,1.57,0,0)` | `tf_*` 全 0 占位 | **禁回填**：官方为死代码；G8-2 恒等阵不通过 |
| `globalmap_pcd` | `.../system/map/lite3.pcd` | `/home/ros1-slam/data/maps/lite3.pcd` | **禁回填**：地图版本无对照件 |
| `map_yaml` | `lite3.pgm` res 0.05 origin [-3.55,-7.05,0] | 2D `/map` 由 M4 供；样张 z=nan 曾被拒载 | **禁回填** |

机器可读版＝`framework/vendor_nav_chain.py` 的 `VENDOR_INTERFACES` 与 `NO_BACKFILL_KEYS`
（12 个键）。`no_backfill_report()` 会把任何一项「官方值出现在项目生效设置里」判为
`backfill_detected`，空输入判 `insufficient_data` 而非印绿。

★为什么不能照官方数值改导航算法★：这些值全部落在 `Runbook:4/:129/:230/:236` 与契约
`:213` 的禁改面（外参／TF／速度上限／判据门值／地图版本）；且官方栈与项目栈的
规划器（DWB vs TEB）、安全层（无 mux vs 唯一写者 mux）、里程计源（腿式 vs qnx2ros IMU）
都不同，同数值不同语义。**本批改的是「启动就绪层」与「接口对照层」，不是算法参数。**

---

## 四、本批落地件（全部默认关闭、零回归）

| 件 | 作用 | 默认 |
|---|---|---|
| `framework/vendor_nav_chain.py` | 官方三段链只读契约、17 项接口对照、12 键禁回填判据、fail-closed 初位姿就绪门；`--report` / `--json` / `--selftest` | 纯函数，不启节点 |
| `framework/nav_init_readiness_node.py` | 影子就绪门 ROS 桥；只订 4 个输入、只发 1 个诊断话题 `/v8/nav/init_readiness` | `readiness_mode=off` |
| `framework/evt_gates.py` | EVT G0—G5 门控模型、空白参数对照件审计器、三连自测 | 只读，`entry_is_not_gate_release=true` |
| `configs/lite3/SIM_TO_REAL_PARAM_SHEET.yaml.template` | **机器可审计的签值骨架**（4 类／20 个 `real_value: null`），与既有 `note/11` 迁移规则草案互补，不宣称补齐缺口 | `TEMPLATE_ONLY` / `effective: false` |
| `configs/lite3/SENSOR_ODOMETRY_INVENTORY.md` | 传感器与里程计现状盘点：唯一 LIO、VIO 零实现、四类易混淆实现、口径纠偏（2026-11-09 补） | 只读事实，不决策 |
| `configs/lite3/VENDOR_NAV_EVT_READINESS.md` | 本件：官方只读勘察、对照表、门控、待勾项 | 文档 |
| `framework/test_vendor_nav_chain.py` | 16 个定向单测 | — |
| `framework/test_nav_init_readiness.py` | 12 个定向单测（ROS stub，无需 roscore） | — |
| `framework/test_evt_gates.py` | 13 个定向单测（入场不放门、空白模板审计） | — |
| `scripts/preflight_navigation.sh` 追加第 7 项 | 显式 `VENDOR_NAV_REPORT=1` 才打印官方链对照 | 默认关闭，**不参与 rc** |

就绪门三档：`off`（完全惰性，`start()` 在需要 ROS 之前就返回）｜`shadow`（只报不拦）｜
`assist`（只在诊断话题里给 `veto` **建议**，不写 `/cmd_vel`、不碰 mux／hold／ros2qnx、
不改任何判据门值，也不构成物理阻断）。

四个必需信号＝`initial_pose_received` / `localization_converged` / `map_ready` /
`odom_chain_ready`。★收敛信号在仓内**无注册生产者**，故保持 `None` ⇒ 门恒为
`insufficient_data`／`not_ready`，这是刻意的 fail-closed，不是缺陷。接生产者属另行裁定。★

既有 tracked 运行配置（`mode_navigation.launch`、`hdl_localization.launch`、
`mbf_navigation.launch`、`costmap_common.yaml`、`cmd_vel_mux.yaml`、`graph_3d_lite3.yaml`、
`01_sensors.launch`、真机链路 `ros2qnx.cpp`）**逐字未动**——它们有大量在册行号引用
（如 `hdl_localization.launch:89`、`costmap_common.yaml:14/:46`），插行即造成引用漂移。

---

## 五、验收读数（本批现跑）

- `vendor_nav_chain.py --selftest` 与 `evt_gates.py --selftest`：主测／注入有效性／反控三连均 `OK`。
- 新增单测 **41 个全绿**：`test_vendor_nav_chain` 16 ＋ `test_nav_init_readiness` 12 ＋
  `test_evt_gates` 13。
- 一次性综合断言 **118/118 PASS**：含官方只读快照 9 节来源回查、9 个 tracked 运行配置
  与 HEAD 逐字对表（零漂移）、12 个禁回填键的有效层扫描、`preflight` 前 53 行逐字不变＋
  反推复原、模块三连、模板空白审计、默认路径零回归、文档数字与实物对表。
- `preflight_navigation.sh` 默认路径：改前改后 stdout 与 rc **逐字一致**
  （`VENDOR_NAV_REPORT` 未设时新增分支完全不打印）；`VENDOR_NAV_REPORT=1` 时打印对照，
  rc 与默认路径相同；`bash -n` 通过。
- 既有相关单测与门禁 `SYN`／`PYST` 见收口报告。
- 证据自带字段：`hardware=false`／`live_topic_writes=0`／`motion_topic_writes=0`／
  `gate_thresholds_modified=false`／`produces_pass_fail=false`／
  `veto_is_recommendation_only=true`／`converged_producer_registered=false`／
  `entry_is_not_gate_release=true`。

---

## 六、仍待真人勾（G2—G5 逐门放行，本批不代勾）

1. **G2**：作业窗、单项作业单编号、现场看护与急停确认。★阶段入场已授权，此项是
   EVT 段内部的逐门放行，不是重新申请入场★。
2. **G3**：外参实测值、`base_link→rslidar` 实装方式（驱动提供还是静态 TF）、
   `preflight_navigation.sh` 现场 rc=0 回执。
3. **G1 签值**：`SIM_TO_REAL_PARAM_SHEET.yaml.template` 的 20 个 `real_value`
   （外参／TF／速度上限／判据门值／地图版本）逐项给值与依据；本批一个都没填。
4. **G4**：`enable_motion`／`ros2qnx` 真机链路、`assist` 档接安全链的授权。
5. **G5**：定位收敛判据与生产者、判据面签字、正式验收措辞。
6. 是否把影子门挂进 `mode_navigation.launch`（当前**不挂任何 launch**）。
7. 官方表述笔权：本件对官方代码的描述仅为只读勘察记录，不代替厂商文档或许可结论。
8. 若真人认为「允许进入 EVT 段」应同时放行 G2—G5，请明示；`evt_gates.py` 的
   `release:Gn` 令牌可直接逐门放行，不必改代码。

### 推荐选项（供真人勾）

- **甲（推荐）**：按门推进——先给 G2 作业窗与单项作业单，同时让现场量外参填 G1/G3；
  G4 真机执行前再跑一次 `preflight_navigation.sh`。理由是 G4 一旦放行即产生实物动作，
  且 `Runbook:4/:236` 的硬约束需要留痕。
- **乙**：先用现有仿真台做一次 EVT 预演（不接真机、不填外参），把影子门 `shadow` 档
  与门控状态机在仿真里跑通，再进 G2。适合先验证门本身是否好用。
- **丙**：若「允许进入」本意即放行全部内门，请明示；本批模型可撤销，不留既成事实。

---

## 七、边界

- 全程对 `103` 只读；未改／未下载／未删除 `<VENDOR_STACK>/` 任何原件
  （`note/计划书v5/21_…v5.3.md:118` 生产侧禁搬／改／删）。
- 未连真机、未起节点、未写任何话题、未改门值、未填真机数值、未产生 PASS/FAIL。
- 本件与 `GAIT_PARAM_CONTRACT.md`／`GAIT_RUNTIME_WIRING.md` 同属「机制先落、数值待勾」线，
  互不引用对方数值，也不互相顶替判据。

---

## 八、2026-11-09 补记：口径纠偏与传感器拓扑

### 8.1 参数迁移对照件的口径纠偏

本批第 1、2、4 节原把 G1 写成「补齐 `Runbook:230` 所缺对照件」，措辞不准，已就地更正：
`note/11_仿真到实机参数迁移表.md`（v0.1 草案，2026-10-28，182 行）**早已存在**，
逐项给出仿真值／真机值／迁移规则／复核命令／✅⚠❌ 标记，只是真机值全部标
「待实测（作业单）」。`Runbook:230` 的「无对照件」应读作
「**无已填真机值、经现场复核的生效对照件**」，两者不矛盾。

| | `note/11_…迁移表.md` | `SIM_TO_REAL_PARAM_SHEET.yaml.template` |
| --- | --- | --- |
| 形态 | 人类可读规则表 | 机器可审计签值骨架 |
| 回答 | 这个值能不能搬、怎么复核 | 谁在哪个作业单下填、填了要被判红 |
| 真机值 | 逐项「待实测」 | 20 个 `real_value: null` |
| 作用 | 迁移决策依据 | G1 门控的证据容器 |

⇒ 本批补的是**互补件**，不宣称首创、不宣称缺口已闭合；G1 仍为「待签值」。

### 8.2 传感器与里程计拓扑（直接影响真机适配）

**仓内唯一的传感器-惯性里程计是 `faster_lio`（紧耦合 LiDAR-Inertial，IESKF＋iVox）；
没有任何 VIO，也没有第二种传感器-惯性里程计。** 其余带「里程计／定位」字样的实现
分属 NDT 定位、腿式里程转发、纯运动学积分、仿真真值直给四类，均非同类算法。

- 视觉/VIO：**无源码、无驱动、无图像话题生产者**。`ws/src/00_drivers/` 5 个包全是
  雷达＋IMU／本体遥测桥；`70_scan.launch:58` 明写「Lite3 无深度相机，depth 分支不消费」；
  `note/计划书v5/21_…v5.md:327` 把 `realsense2_camera` 明确排到二期。
- 在册选型记录中**既没有「决定不引入 VIO」也没有「预留 VIO」的明文**，
  应按「未规划、未评估、未实现」读，不能读成「评估后否掉了 VIO」。
- 官方 Lite3 有广角相机与「视觉跟随」功能（`note/03_外部文件来源清单.md:72`），
  但那是跟随/停障功能，不是视觉里程计；官方定位链同样是
  `/leg_odom2` ＋ `/rslidar_points` ＋ hdl_localization NDT。

完整盘点、逐条路径行号、易混淆四类实现、以及「若引入 VIO 的接入点与红线」
见同目录 `SENSOR_ODOMETRY_INVENTORY.md`。

★引入新传感器/新估计器属**授权面外的新决策族（L2）**，本节只登记现状，
不提议、不决策、不改任何配置。真人若要评估 VIO，需另勾作业面并解决
`/Odometry` 单写者归属、相机外参标定、图像时间戳与 `camera_info` 基线
（仓内目前一项都没有）。★
