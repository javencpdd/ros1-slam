# 传感器与里程计现状盘点（只读勘察，2026-11-09）

状态：**在册事实盘点**，不是设计提案，不新增传感器、不授权真机动作、不产生 PASS/FAIL。
目的：回答「本项目是不是只有 LiDAR-惯性里程计，有没有视觉-惯性里程计（VIO）」，
并把结论固化下来，避免 EVT 真机适配阶段反复勘察。

## 一、一句话结论

**是。仓内唯一的「传感器-惯性里程计」是 `faster_lio`（紧耦合 LiDAR-Inertial
Odometry，IESKF + iVox），没有任何 VIO，也没有第二种传感器-惯性里程计。**

仓里其余带「里程计／定位／位姿」字样的实现分属四类，均不是同类算法：
NDT 点云定位、腿式里程转发、纯运动学积分、仿真真值直给。见第三节。

## 二、唯一的 LIO

| 项 | 在册事实 |
| --- | --- |
| 算法 | `ws/src/10_slam/faster_lio/package.xml:7-9` 自述「Lightweight Tightly Coupled **Lidar-inertial Odometry**」，源自 FastLIO2 / Faster-LIO (RA-L 2022) |
| 滤波 | `src/laser_mapping.cc:18-23` 迭代 ESEKF（`kf_.init_dyn_share`）；状态向量 `include/use-ikfom.hpp:14-15` 含 pos/rot/外参 R,T/vel/bg/ba/grav |
| 惯性融合 | `src/laser_mapping.cc:279-280`「IMU process, kf prediction, undistortion」；`include/imu_processing.hpp:23,37-38` IMU 递推＋点云去畸变。**是 IESKF 前向传播，不是 GTSAM 式预积分因子** |
| 输入 | `src/laser_mapping.cc:244-255` 只订两个：`common/lid_topic`（点云）与 `common/imu_topic`（IMU），**无任何图像订阅** |
| 输出 | `src/laser_mapping.cc:265` `/Odometry`（`camera_init → body`）＋ `/cloud_registered` 等 |
| 启动 | `ws/src/10_slam/lite3_slam/launch/10_slam_mapping.launch:6`（真机）／`10_slam_mapping_sim.launch`（仿真）；`lite3_slam` 是启动壳，无 `.cpp/.h` |
| 在册登记 | `v8_platform/registry/data/manifests/manifest_odometry_faster_lio.json:3,13-20` 角色 `odometry`；graph 中 `graph_faster_lio.yaml:4-6` 等 12 处引用 |

IMU 源（Q12 分支）：真机 `lite3_slam/config/faster_lio_outdoor.yaml:3-6` 用 Mid-360 内置
`/livox/imu`（实测 199.9 Hz），因 UDP 43897 被 `jetson2motion` 占用而弃用 `qnx2ros`
的 `/imu/data`；仿真 `lite3_gazebo_sim/config/faster_lio_sim.yaml:12` 由 `sim_adapter` 合成同话题。
`note/11_仿真到实机参数迁移表.md:48` 定性：**话题名同源、数据源不同**，换源即改
lidar↔IMU 几何，须重估外参。

## 三、四类「不是第二种传感器-惯性里程计」的实现

| 类别 | 代表 | 为何不算 |
| --- | --- | --- |
| **NDT 点云定位** | `20_localization/hdl_localization/apps/hdl_localization_nodelet.cpp:88-95`（NDT_OMP）、`:68` 发 `/odom`→remap `/hdl_odom`；内建 UKF `include/kkl/alg/unscented_kalman_filter.hpp:16-21`；`hdl_global_localization` 全局重定位；`v8_ndt_odom/src/ndt_odom_node.cpp:42`「only monitor role is authorized; continuous backend is not enabled」 | 输出是 **map 系全局定位**（`README.md:48-49`「Estimated sensor pose in the map frame」），不是里程计递推；UKF 只是定位滤波 |
| **腿式里程转发** | `00_drivers/message_transformer/src/qnx2ros.cpp:50-54,123-129` 发 `/leg_odom`、`/leg_odom2`、`/imu/data` | 从 QNX `robot_state` 直接填消息，**无滤波无估计**；hdl 只取 `twist`（`hdl_localization_nodelet.cpp:460`），`enable_robot_odometry_prediction=false` |
| **纯运动学积分** | `v8_platform/local_planner/kinematic3d_node.py:15-19` `x+vx*dt`；`v8_simulation/scripts/gazebo_carrier3d_node.py:2-6` Gazebo SetModelState | 测试载体，自述「No physics/contact」「kinematic pose sink」，不消费传感器 |
| **仿真真值直给** | `95_simulation/.../src/lite3_planar_drive.cpp:120` 取 `WorldPose()` 发 `/sim/odom_gt`；`70_bringup/.../sim_pose_tf.py:10-11` 发 `/Odometry` 真值；`gazebo_truth.py:4-6`「verification-side oracle, not part of any algorithm graph」；`mapping_recorder.py:6`「This is not LIO/SLAM」 | 评测用的 oracle / GT 建图参考，不是估计器 |

另有 `10_slam/pcd2grid`（PCD→2D 栅格）与 `ndt_omp`／`fast_gicp`（配准库），是地图/配准工具，
不是里程计。

## 四、视觉 / VIO：确切状态

**无源码、无驱动、无图像话题生产者。** 视觉相关命中只有四类残留：

1. **存活探针（非估计）**：`00_drivers/message_transformer/src/sensor_checker.cpp:118-126`
   `class Realsense : public Sensor` 只订 `/camera/depth/color/points` 点云、发存活 Bool；
   `nx2app.cpp:98-99,137` 用于「realsense is down → systemctl restart」。
2. **第三方原件 depth 分支**：`30_3rd/plan_env/src/grid_map.cpp:758-766` 的
   `depthPoseCallback`；本项目固定 lidar 分支 `40_navigation/lite3_scan/launch/70_scan.launch:53`
   `sensor_type="lidar"`，`:58` 明写「**depth 分支不消费（Lite3 无深度相机，A6 不适用行）**」。
   `30_3rd/plan_manage/launch/run.launch:28,35` 的 `/camera/...` 只是原件占位参数。
3. **第三方拷贝件的悬空话题**：`30_perception/deeprobotics_local_height_map/config/input_pose_hei.yaml:2`
   `/vins_estimator/body_pose_stamped`、`rviz/height_map_camera.rviz:639`
   `/feature_tracker/feature_img`、`launch/log.launch:4` `multi_camera_leg_feature_tracker` —— 仓内
   均无对应包/节点，属未清理残留。
4. **仿真渲染相机**（非真机传感器）：`95_simulation/local_sensing/src/opengl_render_node.cpp`
   OpenCV 渲染输出 `sensor_msgs/Image`；`note/进度/v8/M3/26-10-27/R711.md:1`「Gazebo 深度相机
   输出恒为 3.0 m——判死」、`R725.md:9` MuJoCo 96×72 osmesa 约 2.4—3.0 Hz。

驱动清单：`ws/src/00_drivers/` 仅 5 个包 —— `lite3_drivers`（起链壳）、
`livox_ros_driver2`（Mid-360 雷达＋内置 IMU）、`lslidar_driver`/`lslidar_msgs`（C16，
`01_sensors.launch:11` 注明「保留不入链」）、`message_transformer`（QNX↔ROS 遥测桥）。
**无 `usb_cam`、`realsense2_camera`、无 `/image_raw` 生产者**。
`note/计划书v5/21_…v5.md:327` 明确「`realsense2_camera` … **二期才复制，一期清单中删除**」，
`note/03_外部文件来源清单.md:20` 同条禁令。

框架名级检索 `vins|VINS|open_vins|orb_slam|rtabmap|visual_odometry|VINS-Fusion|OpenVINS`
在 `ws/src/`、`note/**/*.md`、`docs/`、`opt/` 中：**零个实现**。
（唯一形近命中是 `opt/upstream/lvr2/ext/kintinuous/kfusion/FusionStage.hpp`，
属第三方 KinectFusion 体积重建，未接入。）

关于官方 Lite3：`note/03_外部文件来源清单.md:72` 抄录产品手册「智能感知＝**广角相机×1** ＋
超声波雷达×2（出厂不含激光雷达）」、`:68-69`「视觉跟随与停障辅助互斥」。
官方有相机与「视觉跟随」**功能**，但仓内无其源码；官方定位链（本目录
`VENDOR_NAV_EVT_READINESS.md`）是 `/leg_odom2` ＋ `/rslidar_points` ＋ hdl_localization NDT，
**同样不是 VIO**。

## 五、在册选型记录里有没有「考虑过 VIO」

**没有。** 精确检索 `不引入视觉|预留 VIO|视觉惯性|视觉里程|多传感器融合` 在 `note/**/*.md`
零命中 —— 既没有「决定不引入 VIO」的明文，也没有「预留 VIO」的明文。有的只是**等价的
排除性事实**：

- 传感器选型 A3 自始至终只有「雷达 + IMU」：`v5/21_…v5.md:70`（Leishen C16）→
  `v4/21_…v4.1-雷达翻案补丁.md:14`（翻案为 Livox Mid-360）。
- `realsense2_camera` 二期（见上）。
- depth 分支不消费、Lite3 无深度相机（见上）。
- 坑14 口径 `note/02_问题与坑.md:107-110`「规划仿真不可验估计层；Gazebo 物理仿真已实证
  可验 faster_lio…」——全仓「估计层」一词只指 faster_lio。
- 验收能力面 A1—A7（`note/计划书v8/24_仿真验收与迁移方案.md:61-68`）中 A6 是
  「定位与地图链、两类定位后端可切换」，**无 VIO 项**；`note/进度/00_总览.md:36` 记
  `A2=PASS*（窄域）；A1/A3/A4/A5/A6/A7=PARTIAL`。

⇒ 结论要按「**未规划、未评估、未实现**」读，不能读成「评估后否掉了 VIO」。

## 六、若将来要引入 VIO：接入点与红线（仅供真人裁定，本次不决策）

属**授权面外的新决策族（L2）**，本盘点不提议、不引入、不改任何配置。仅列既有约束：

- **帧**：faster_lio 世界帧 `camera_init`／机体 `body`（`faster_lio_outdoor.yaml:47-48`）；
  hdl 输出 `map → base_link`；`base_link→rslidar` 由驱动提供（`Runbook:128`「不得重复发布」）；
  `map→odom` 存在双父病根（`note/02_问题与坑.md:219` 坑26）。
- **话题**：`/Odometry` 是**单写者红线**（`95_simulation/.../verify/v_pose_source.sh:50-52`
  「/Odometry 发布者恰 1」）；新增位姿源必须先解决发布者归属，不能并行双发。
- **外参**：Runbook `:4/:15/:230/:236`「外参／TF／速度上限／判据门值不得现场改」；
  Q12「换 IMU 源即改 lidar↔IMU 几何，须重估外参」。仓内**无任何相机外参、
  图像时间戳或 `camera_info` 基线**可复用。
- **参数对照**：`note/11_仿真到实机参数迁移表.md`（人类可读迁移规则草案）
  与本目录 `SIM_TO_REAL_PARAM_SHEET.yaml.template`（机器可审计的签值骨架）互补，
  见下节口径说明。

## 七、口径纠偏（2026-11-09，本批）

上一批 `VENDOR_NAV_EVT_READINESS.md` 曾把 G1 模板写成「补齐 `Runbook:230` 所缺对照件」，
措辞不准：`note/11_仿真到实机参数迁移表.md` **早在 2026-10-28 就存在**（v0.1 草案，
182 行，逐项给出仿真值／真机值／迁移规则／复核命令／✅⚠❌ 标记）。

两者不矛盾，是**分工不同**：

| | `note/11_…迁移表.md` | `SIM_TO_REAL_PARAM_SHEET.yaml.template` |
| --- | --- | --- |
| 形态 | 人类可读规则表（md） | 机器可审计签值骨架（yaml.template） |
| 内容 | 「这个值能不能搬、怎么复核」 | 「谁在哪个作业单下填、填了要被判红」 |
| 真机值 | 逐项标「待实测（作业单）」 | 20 个 `real_value: null` |
| 作用 | 迁移决策依据 | G1 门控的证据容器 |

`Runbook:230` 的「无仿真→真机对照件」应读作「**无已填真机值、经现场复核的生效对照件**」；
两份件都还是草案/空白态，均未解除该缺口。本批已把相关措辞统一为
「补机器可审计的签值骨架，与 note/11 互补」，不宣称首创、不宣称缺口已闭合。
