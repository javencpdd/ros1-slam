# 21 号计划书 v4.1 补丁：雷达身份翻案（C16 → Livox Mid-360）+ S9 判无用

> **性质**：v4 补丁稿（v4 正文一字不动；本文件按条目给出修订内容与落点，由主理人裁定收编后落位 `note/计划书v4/`，与 v4 正文并列）。
> **产出**：部署架构师 小张，2026-10-04。
> **证据纪律**：全部证据为只读实测（ysc 生产配置/源码只读 grep/cat/diff、/opt 系统包头文件、被动抓包结果、实机目视），未修改 `/home/ysc` 下任何文件；拿不到实证的一律标 Q，不编造。
> **贯穿约束（用户裁定，最高优先级）**：`/home/ysc/` 一切文件只读；任何要用的文件**先复制到 `/home/ros1-slam/` 对应位置后从副本使用**——含 Livox_SDK2 的编译安装（S10）。

---

## v4.1 变更日志（X1–X10，相对 v4）

| 级别 | # | 变更（内容 + v4 落点） | 原因（实测证据） |
|---|---|---|---|
| 🔴 事实 | **X1** | **A3 修订（传感器身份翻案）**：传感器 = **Livox Mid-360**（192.168.1.201；点云 UDP `201:56300 → 103:56301`，IMU UDP `201:56400 → 103:56401`）。原 A3"Leishen C16（2368/2369）"作废；C16 相关条目标注"**软件遗留，不适用本机实机**"。牵连：A8 端口口径、§1.2 基线行、§7.5、§10 P5.3 | 被动抓包 4 条 + 生产栈 3 处 + 实机目视（§1 证据链） |
| 🟠 架构 | **X2** | **M0 采图链驱动翻案**：lslidar_c16.launch → **livox_ros_driver2**（include 生产同款 `msg_MID360s.launch`）；**话题契约不变**——驱动 remap 后仍发布 `/rslidar_points`（PointCloud2，10 Hz，frame_id=rslidar）。`lslidar_driver`/`lslidar_msgs` 复制件与 launch **保留文件不入链**（对照/回退）。P0.5 传感器侧正则增补 `livox` | 生产链拓扑实测（§2.2）：Mid-360 输出经 remap 伪装成 C16 话题名喂 faster_lio，v4 的话题对账（R15）恰好因此无需改 |
| 🟢 裁定 | **X3** | **S9（nav_core_adapter）判定无用**（裁定二）：MBF 原生**双加载器三对**已覆盖 nav_core 插件回退加载；nav_core_adapter 的 plugin xml 只声明 `nav_core::*` base（方向是 move_base 用 mbf 插件，反向），与 mbf 加载路径无关。**处置：不安装、已装也不卸载（无害保留）**；A11/N1"前置 S9"、回退触发条件①、C0 注释、§9 P4 增补行、工期 S9 行相应修订；**Q9 关闭** | `costmap_navigation_server.h:236-241` + `nav_core_plugins.xml` + `apt-cache rdepends` 零消费（§2 证据） |
| 🟠 事实 | **X4** | **v3 V4 注记纠错**：`lidar_type: 4` = **C16Handler 分支**（`enum class LidarType { AVIA=1, VELO32, OUST64, C16 }`），**不是** v3/v4 所注"Velodyne 分支"；值 4 本身正确不变，v2"用 2"也仍然错误 | 沙箱副本 `include/pointcloud_preprocess.h:87` 枚举实测 + 生产 `c16.yaml` 行内注释（"4 for C16 Leishen Lidar"） |
| 🟠 清单 | **X5** | **§7.1 复制清单增补第 18/19 条**：①`mid360_ws/src/livox_ros_driver2` → `ws/src/00_drivers/`（ROS1 包）；②`Livox_SDK2` → **沙箱副本** `opt/3rdparty/`（纯 cmake 库，**不进 ws、不入编译批次**）。均按"原样复制 + ysc 串改写扫描 + 排除 build/.git"惯例 | §3 |
| 🟠 sudo | **X6** | **S10 新增**：Livox_SDK2 核查/安装步骤——**本机 /usr/local 已有 ysc 历史安装（0.0.2），核查通过则零操作**；缺才装，且一律从沙箱副本 cmake（编译以 ros1-slam 身份，仅 `sudo make install` 用 sudo） | §4 |
| 🟡 配置 | **X7** | **C1 修订（注释级，参数值全部不变）**：imu_topic 裁决 `/imu/data`（生产主证据）+ 备选 `/livox/imu`（Q12）；lidar_type 注释按 X4 修正；外参两来源矛盾标 Q13；scan_line/time_scale 注明 C16Handler 分支不消费；launch node type 勘误 `mapping_online` → **`run_mapping_online`** | §5 全文替换稿 |
| 🟡 编译 | **X8** | **P5 增补**：批 1 白名单加 `livox_ros_driver2`（S10 先于批 1）；Livox_SDK2 零批次；faster_lio 零源码改动 → 批 2 原样无重编译；P5.3"都不需要 Livox"行作废 | §6 |
| 🟡 Q/R | **X9** | **Q3 作废重立**（Mid-360 安装几何现场实测）；**Q7 关闭**（C16 时代 ROS2 栈遗留，有解）；**Q12/Q13 新增**；**Q9 关闭**；R15 对账行更新、R17 口径同步裁定二、**R20 新增**（外设类条目必须实机核验） | §7 |
| 🟢 台账 | **X10** | 04 台账两条 + §15 工期修订（S9 行删除、P3b +0.5h）+ §17 状态行 | §8 |

---

## §1 裁定一：传感器身份翻案（A3 → A3.1）

### 1.1 证据链（四条，相互独立）

| # | 证据 | 内容 | 来源（只读） |
|---|---|---|---|
| E1 | **被动抓包** | `192.168.1.201:56300 → 103:56301` + `201:56400 → 103:56401` 持续有流量；**C16 的 2368/2369 零流量**；.201 可达 | 用户现场 tcpdump |
| E2 | **端口对照** | `MID360s_config.json`：lidar 侧 `point_data_port: 56300`/`imu_data_port: 56400`，host（`192.168.1.103`）侧 `point_data_port: 56301`/`imu_data_port: 56401`——与 E1 逐口吻合（**勘误**：抓包注记把 56300 标为"IMU 端口"，按配置 56300=**点云**、56400=IMU） | `/home/ysc/lite_cog/drivers/mid360_ws/src/livox_ros_driver2/config/MID360s_config.json` |
| E3 | **生产栈存在** | ①`drivers/mid360_ws`（catkin ROS1 工作区，devel 已编译出 `livox_ros_driver2_node`）+ `drivers/Livox_SDK2`（build/ + install_manifest.txt 证明历史 cmake install）；②`/home/ysc/.ros/log/` 有 `livox_ros_driver2_node_*.log`（2024-09-12、2025-09-16 等多批次）与同期 `run_mapping_online_*.log`（faster_lio 与 livox 驱动同日联跑）；③`lite_cog_ros2/driver/` 另有双份 | `/home/ysc/lite_cog/drivers/`、`/home/ysc/.ros/log/` |
| E4 | **实机目视** | 穹顶 + 保护罩形态 = Mid-360 | 用户现场目视 |

### 1.2 生产链拓扑实测（本裁定关键——"伪装链"）

生产 ROS1 采图链（与 Mid-360 实机联跑过的形态）：

```
[livox_ros_driver2]  msg_MID360s.launch
   ├─ xfer_format: 0        → 本 fork 枚举 kPointCloud2Msg=0（lddc.h:42-47；⚠️ 上游 driver2 的 0/1 语义以本 fork 为准）
   ├─ frame_id: rslidar
   ├─ <remap /livox/lidar → /rslidar_points>
   ├─ user_config_path = MID360s_config.json（host 192.168.1.103 / lidar 192.168.1.201）
   └─ 恒等 TF base_link→rslidar（注释里另有 7.5° 俯仰 TF，未启用 → Q13）
        │  sensor_msgs/PointCloud2（10 Hz）
        ▼
[faster_lio]  mapping_c16.launch + config/c16.yaml
   ├─ lid_topic:  /rslidar_points      ← 驱动 remap 后的话题，天然对上
   ├─ imu_topic:  /imu/data            ← 狗体 IMU（qnx2ros 发布 200 Hz，UDP 43897）
   ├─ lidar_type: 4                    ← C16Handler 分支（PointCloud2 → x/y/z/intensity）
   └─ extrinsic_est_en: true           ← 外参在线估计（初始外参不敏感的兜底）
```

**要点**：生产把 Mid-360 的输出**伪装成 C16 的话题名与坐标系**喂给 faster_lio 的 C16Handler 分支——C16Handler 只读 PointCloud2 的 x/y/z/intensity（`leishen_ros::Point`，`fromROSMsg` 按字段名映射，Mid-360 多出的 tag/line/time 字段被忽略），兼容成立且经生产实机验证。因此：

- **M0 驱动选型翻案，但 v4 的话题契约（M0→M1 = `/rslidar_points`）一字不改**（R15 对账自动满足）；
- faster_lio **零源码改动、零重编译**（C1 全为参数级）；
- 沙箱 faster_lio 副本与生产源码 diff 一致（`config/c16.yaml`、`launch/mapping_c16.launch` 逐字节比对通过），`thirdparty/livox_ros_driver`（仅 msg 包，供 AVIA 分支的 CustomMsg 头）已在副本内——**faster_lio 无外部 Livox 依赖**；Livox 依赖只在 M0 驱动（livox_ros_driver2 → SDK2 静态库）。

### 1.3 A3.1 修订文本（替换 §1.1 A3 行）

| # | 假设（v4.1） | 依据 |
|---|---|---|
| **A3.1** | 传感器 = **Livox Mid-360**（192.168.1.201，点云→103:56301，IMU→103:56401）+ 狗体 IMU（`/imu/data`，qnx2ros 发布） | §1.1 四条证据 + 生产 c16.yaml；C16 为软件遗留（lslidar 复制件/launch 保留不入链，不适用本机实机） |
| **A8 注记** | 隔离窗口的传感器端口口径同步改为 Livox 端口对（56100/56101 命令、56200/56201 推送、**56300/56301 点云**、56400/56401 IMU）——SDK 绑定 host 五端口，**同一时刻只能有一个 livox 驱动实例**；P0.5 传感器侧正则增补 `livox` | MID360s_config.json 端口表 |

§7.5 同步标注：C16 参数（msop 2368/difop 2369/frame_id rslidar 等）**仅存档作软件遗留对照**，不适用本机实机。

---

## §2 裁定二：S9（nav_core_adapter）判定无用

### 2.1 证据（本架构师对 /opt 只读复核，与用户考证及主理人装机源码复核一致）

| # | 证据 | 内容 |
|---|---|---|
| E1 | `/opt/ros/noetic/include/mbf_costmap_nav/costmap_navigation_server.h:236-241` | MBF 原生**双加载器三对**：`ClassLoader<CostmapRecovery>`‖`ClassLoader<nav_core::RecoveryBehavior>`、`ClassLoader<CostmapController>`‖`ClassLoader<nav_core::BaseLocalPlanner>`、`ClassLoader<CostmapPlanner>`‖`ClassLoader<nav_core::BaseGlobalPlanner>`；类注释原文 "makes Move Base Flex backwards compatible to the old move_base…nav_core/… as plugin interfaces"——**mbf 对 nav_core 插件有原生回退加载器，无需任何 adapter** |
| E2 | `/opt/ros/noetic/share/nav_core_adapter/nav_core_plugins.xml` | 仅声明两个 class：`base_class_type="nav_core::BaseLocalPlanner"`（LocalPlannerAdapter）与 `"nav_core::BaseGlobalPlanner"`（GlobalPlannerAdapter），**无任何 mbf_* base**——即 adapter 是把 **mbf 插件包装成 nav_core 插件给 move_base 用**（方向相反），mbf 加载路径完全不经过它 |
| E3 | `apt-cache rdepends --installed ros-noetic-nav-core-adapter` | 反向依赖**零消费** |

### 2.2 结论与修订面

**结论：nav_core_adapter 与 mbf 的插件加载路径无关，S9 无用。** C0 三个 type 串（`global_planner/GlobalPlanner`、`teb_local_planner/TebLocalPlannerROS`、`clear_costmap_recovery/ClearCostmapRecovery`）经 **mbf 原生 nav_core 回退加载器**直接加载。

| v4 条目 | v4.1 修订 |
|---|---|
| §1.1 A11 / 变更日志 N1"前置 S9" | 删"前置"：M4 形态不再依赖 S9；全局规划器加载路径 = mbf 原生 nav_core 回退加载器（E1） |
| §1.1 A11 表内"全局规划器加载需 S9（Q9）" | 更正为"经原生回退加载器加载，无需安装" |
| §5 P0.4 S9 行 | 标注"**判定无用（v4.1 X3），不执行**；如已装则无害保留、不卸载" |
| §4 DAG [S9] 节点 | 标注作废（不阻塞 P3a） |
| §8 C0 `mbf_navigation.yaml` 注释"加载须经 S9 nav_core_adapter（Q9）" | 更正为"经 mbf 原生 nav_core 回退加载器加载（costmap_navigation_server.h:236-241，v4.1 X3）" |
| §8 回退决策表触发条件①（Q9 不批 → 回退） | **删除**；回退仅剩触发条件②（P6/P7 联调阻塞） |
| §9 P4 增补行（rospack find nav_core_adapter） | 修订为"无需核对（S9 已判无用）；若机器上已装，仅登记存在、不依赖" |
| §15 工期 S9 行（5 min） | 删除 |
| Q9 | **✅ 关闭（S9 判无用，无需安装）** |
| Q11 ③（recovery type 串可加载性） | 风险下调：加载器存在性已实证（E1），残余仅"class 串与库的实际可加载性"P6 实证，保留在 Q11 包内 |
| R17 残余风险表述 | 同步更正（见 §7 R17 行） |

### 2.3 处置

1. 04 台账一条（§8）；2. Q9 关闭；3. **不安装**（未装）；4. 若后续发现已装（例如别的项目装过），**不卸载**——它对 mbf 链路零影响，卸载反而动系统状态，违背最小扰动。

---

## §3 复制清单增补（§7.1 增第 18/19 条；§3.2 双目录制条款适用）

> 双目录制：第三方包**原样复制、一个字不改**（唯一例外仍是 V2 存盘路径，本批新增两包均无 `/home/ysc` 硬编码，§7.4 扫描预期 0 命中）；改动只准进 `lite3_*` 壳。

| # | 来源（只读源） | 目标（沙箱副本） | 复制口径 |
|---|---|---|---|
| **18** | `/home/ysc/lite_cog/drivers/mid360_ws/src/livox_ros_driver2` | `/home/ros1-slam/ws/src/00_drivers/livox_ros_driver2` | 排除 `.git`（包内无 build 残留）；**package.xml 已是 ROS1 版**（catkin buildtool，生产经 build.sh 以 `-DROS_EDITION=ROS1` 构建）；随包 `launch_ROS1/`、`config/`（MID360s_config.json 即生产实机配置）、`msg/`、`src/`、`3rdparty/rapidjson`；**串改写扫描预期 0 命中**（无 /home/ysc 串）；版本 1.0.0（src/include/livox_ros_driver2.h） |
| **19** | `/home/ysc/lite_cog/drivers/Livox_SDK2` | `/home/ros1-slam/opt/3rdparty/Livox_SDK2` | 排除 `build/`；**纯 cmake 库（project livox_sdk2，SDK 0.0.2）——不进 ws、不入任何编译批次**；仅作 S10 的安装源。⭐ 用户硬约束：**绝不在 `/home/ysc` 侧 build，一律从本副本编译安装** |
| 备注 | `/home/ysc/lite_cog_ros2/driver/mid360_ws`、`…/driver/Livox_SDK2` | — | 另有双份，仅登记来源，不使用 |

复制命令沿用 §7.2 `copy_pkg`（tar 管道排除 .git/build/devel）；`opt/3rdparty/` 目录权限随 `opt/`（555 参照 `opt/ref/` 惯例，属主 ros1-slam）。

---

## §4 S10 新增（sudo 清单，§5 P0.4 追加一行）

> **先核查、缺才装**：本机 `/usr/local` 已有 ysc 历史安装（`liblivox_lidar_sdk_static.a` + `liblivox_lidar_sdk_shared.so` + `livox_lidar_{api,def,cfg}.h`，SDK 0.0.2）——**核查通过则 S10 零操作，仅登记**。理由：livox_ros_driver2 ROS1 分支 `find_library(LIVOX_LIDAR_SDK_LIBRARY liblivox_lidar_sdk_static.a /usr/local/lib)`（CMakeLists.txt:94）**硬编码 /usr/local**，故安装目标必须是 /usr/local（不可用 HOME 前缀替代，且不改第三方 CMakeLists）。

```bash
# ---- S10.0 核查（ros1-slam 或 ysc 身份均可，只读）----
ls -l /usr/local/lib/liblivox_lidar_sdk_static.a /usr/local/include/livox_lidar_api.h \
      /usr/local/include/livox_lidar_def.h /usr/local/include/livox_lidar_cfg.h
# 四件全在 → S10 结束（登记"已装，核查通过，版本 0.0.2"），跳过 S10.1–S10.3

# ---- S10.1 缺则装：一律从沙箱副本（§3 第 19 条），编译以 ros1-slam 身份在家目录进行 ----
cd /home/ros1-slam/opt/3rdparty/Livox_SDK2
mkdir -p build && cd build
cmake ..          # 无特殊开关；samples 随主工程
make -j3          # OOM 则 -j2（同 P5.0 swap 策略）
# ---- S10.2 仅安装一步需要 sudo（装到硬编码的 /usr/local）----
sudo make install
# ---- S10.3 复核 + 台账 ----
ls /usr/local/lib | grep livox
echo "- S10 Livox_SDK2 0.0.2 从沙箱副本 opt/3rdparty 安装至 /usr/local（livox_ros_driver2 编译前置）$(date +%F)" \
  | sudo tee -a /home/ros1-slam/note/04_配置改动diff.md
```

**执行时序**：S10 必须先于 **P5 批 1**（livox_ros_driver2 编译期 find_library）。禁止从 `/home/ysc/lite_cog/drivers/Livox_SDK2` 发起任何 build。

---

## §5 C1 修订：faster_lio_outdoor.yaml v4.1 版（全文替换稿）

> **参数值与 v3 转换稿逐项相同（X7 只修注释与勘误）**；落位不变：`/home/ros1-slam/ws/src/10_slam/lite3_slam/config/faster_lio_outdoor.yaml`（P3a 执行）。**五个裁决**：①`imu_topic: /imu/data`（见 5.1）；②`lid_topic: /rslidar_points`（生产 remap 同款，不变）；③`lidar_type: 4`（C16Handler 分支——X4 注释纠错，值不变，绝不可"修正"为 2）；④`extrinsic_est_en: true` + 初始外参维持 v4 原值（两来源矛盾标 Q13，在线估计兜底）；⑤`pcd_save.interval: 10`（Q6 维持）。

```yaml
# faster_lio_outdoor.yaml —— ROS1 版（v3 C1 转换自 /home/ysc/faster_lio_online.yaml；v4.1 X7 注释修订，参数值与 v3 转换稿一致，勿再"修正"）
common:
  imu_topic: /imu/data            # v4.1 裁决（5.1）：主证据 = 生产 c16.yaml 实测值（与 Mid-360 实机联跑的 ROS1 生产配置）。
                                  # 由 qnx2ros 发布（UDP 43897 → 200 Hz，qnx2ros.cpp:53/261）。
                                  # 备选：Mid-360 内置 IMU /livox/imu（sensor_msgs/Imu ≈200 Hz，lddc.cpp:634）——生产未验证，
                                  # 且换 IMU 即变 lidar↔IMU 几何，须重估外参 → 仅 Q12 现场验证后走变更流程切换，禁止静默改。
  lid_topic: /rslidar_points      # v4.1 维持：生产 msg_MID360s.launch 已 remap /livox/lidar→/rslidar_points，契约不变（A3.1/X2）
  time_sync_en: true
cube_side_length: 1000.0
esti_plane_threshold: 0.1
feature_extract_enable: false
filter_size_map: 0.3
filter_size_surf: 0.3
ivox_grid_resolution: 0.3
ivox_nearby_type: 18
map_file_path: ''
mapping:
  acc_cov: 0.1
  b_acc_cov: 0.0001
  b_gyr_cov: 0.0001
  det_range: 150.0
  extrinsic_R: [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
  extrinsic_T: [0.075, 0.0, 0.023]
  extrinsic_est_en: true          # 在线估计兜底：初始外参两来源矛盾（Q13），现场实测后如需固化走变更流程
  gyr_cov: 0.1
max_iteration: 3
path_save_en: true
pcd_save:
  interval: 10        # Q6 维持：interval=0 时 laser_mapping.cc:747 条件恒假可能永不落盘，10 保证周期性写盘，现场验证后可回调
  pcd_save_en: true
preprocess:
  blind: 1.0
  lidar_type: 4       # v4.1 注释修正（X4）：4 = C16Handler 分支（include/pointcloud_preprocess.h:87
                      #   enum class LidarType { AVIA=1, VELO32, OUST64, C16 }）——v3/v4 所注"Velodyne 分支"有误，值 4 不变。
                      #   C16Handler 只读 PointCloud2 的 x/y/z/intensity（leishen_ros::Point，fromROSMsg 按名映射），
                      #   与 livox_ros_driver2 的 Mid-360 PointCloud2 输出兼容（生产同款链路已实机验证）；
                      #   不消费逐点时间 → scan 内无去畸变（与生产行为一致，出厂即如此）。
                      #   绝不可"修正"为 2：VELO32 解析 Velodyne 字段布局，Mid-360 不匹配。
  scan_line: 16       # C16Handler 分支不消费该参数（仅 AviaHandler 用 num_scans_），保留原值占位
  time_scale: 0.001   # 同上，C16Handler 分支不消费，保留原值
publish:
  dense_publish_en: false
  path_publish_en: false
  scan_bodyframe_pub_en: true
  scan_effect_pub_en: true
  scan_publish_en: true
  tf_imu_frame: body
  tf_world_frame: camera_init
runtime_pos_log_enable: false
use_sim_time: false
```

**与生产的两处已知差异（不改动，仅登记，如需对齐走变更流程）**：生产 `c16.yaml` 另含 `point_filter_num: 3`（v3 转换稿未含 → 代码默认 2，`laser_mapping.cc:86`）与 `fov_degree: 180`（转换稿未含，仅记录用途）。二者只影响抽稀密度/记录，不影响隔离与结构。

### 5.1 imu_topic 裁决理由（Q12 的现场验证设计）

| 方案 | 证据 | 优势 | 代价/风险 |
|---|---|---|---|
| **A. `/imu/data`（主选，维持 v4 值）** | 生产 c16.yaml 实测值（与 Mid-360 联跑过）；v4 A3 的 IMU 半边本来就成立 | 生产验证；qnx2ros 200 Hz；change 零（v4 C1 一字不改） | 依赖 message_transformer + QNX 遥测 UDP 43897（若端口被占则 bind 失败 → P0.5/Q8 覆盖） |
| B. `/livox/imu`（备选） | 驱动实测发布（lddc.cpp:634，sensor_msgs/Imu） | 少一个依赖节点；驱动内置 200 Hz | 生产未验证；IMU 位置变化 → faster_lio lidar↔IMU 外参需重估；extrinsic_est_en 兜底但收敛性未证 |

现场验证（Q12，P3b 作业单第 1/4 节执行）：`rostopic hz /imu/data` 与 `rostopic hz /livox/imu` 双测——主选 A 成立的判据 = `/imu/data` ≈200 Hz 且 qnx2ros bind 成功；A 不可用时**停，报告主理人**按 B 切换并登记 diff，不得现场静默改。

### 5.2 launch 加载形态勘误（C1(b) 块，随 X7）

```xml
<launch>
  <rosparam command="load" file="$(find lite3_slam)/config/faster_lio_outdoor.yaml"/>
  <!-- rosparam 在 node 外：全局 NodeHandle（run_mapping_online.cc:25）读全局命名空间 -->
  <node pkg="faster_lio" type="run_mapping_online" name="faster_lio_online" output="screen"/>
  <!-- ★ v4.1 勘误：可执行名 = run_mapping_online（app/CMakeLists.txt:1 add_executable(run_mapping_online …)），
       v3/v4 写的 type="mapping_online" 不存在，roslaunch 会直接失败 -->
</launch>
```

---

## §6 M0/launch 修订（§2 形态 1 替换）与 P5 增补

### 6.1 `lite3_drivers/launch/01_sensors.launch`（v4.1 形态，X2）

```xml
<launch>
  <!-- 采图链驱动 = Livox Mid-360（v4.1 A3.1/X2）。include 生产同款 launch（自带全部关键语义：
       remap /livox/lidar→/rslidar_points；frame_id=rslidar；user_config_path=MID360s_config.json
       （host 192.168.1.103 / lidar 192.168.1.201）；xfer_format=0（本 fork 枚举 0=PointCloud2，lddc.h:43）；
       publish_freq 10 Hz；恒等 TF base_link→rslidar；node required=true）。
       注：broadcast code 不需要——SDK2 按 config IP 识别设备，launch 的 bd_list 参数对 Mid-360 无效（无害占位）。 -->
  <include file="$(find livox_ros_driver2)/launch_ROS1/msg_MID360s.launch"/>
  <!-- 狗体 IMU（C1 主选 /imu/data，200 Hz）：qnx2ros 监听 QNX 遥测 UDP 43897（server_port 默认值，qnx2ros.cpp:261） -->
  <node pkg="message_transformer" type="qnx2ros" name="qnx2ros" output="screen"/>
  <!-- ⚠️ ros2qnx（订阅 /cmd_vel 直发 QNX 底盘）禁令原样保留（v4 §2/V1）；
       lslidar_c16.launch 文件保留不入链（软件遗留对照，A3.1） -->
</launch>
```

配套修订：P0.5 传感器侧正则扩为 `lslidar|livox|faster_lio|hdl_|roscore|rosmaster`（驱动 smoke/P6 重查用；端口级唯一性见 A8 注记——**同一时刻只能有一个 livox 驱动实例**，起驱动前必须确认 ysc 侧无 livox 进程）。

### 6.2 P5 增补（X8）

```bash
catkin_make -j3 -DCATKIN_WHITELIST_PACKAGES="lslidar_msgs;lslidar_driver;livox_ros_driver2"  # 批 1：M0 驱动（v4.1：+livox_ros_driver2；lslidar 保留编译作对照/回退）
```

- **S10 先于批 1**（编译期 find_library SDK 静态库）。
- Livox_SDK2 = 纯 cmake 库，**零批次**（不进 ws）。
- **faster_lio 批 2 原样**：C1 v4.1 全为参数级，零源码改动 → 无重编译需求（V2 存盘路径改写已在 P2 完成）。
- **P5.3 常见失败表作废一行**：原"lidar_type=4 走 Velodyne 分支……都不需要 Livox"——本机实装后 **M0 需要 Livox**（SDK2 静态库，S10）；faster_lio 仍不需要外部 Livox（`thirdparty/livox_ros_driver` 仅 msg 包，已在副本内，X4）。

---

## §7 Q/R 表修订（X9）

### 7.1 Q 表（§15）

| # | 状态变化 | 内容 |
|---|---|---|
| **Q3** | **作废重立** | ~~C16 安装倾角~~ → **Mid-360 安装几何现场实测**（离地高度/俯仰/朝向），方法 = 卷尺 + 照片记录法（P3b 作业单第 4 节）；产出回填两处：C3 `min_z`（`tf_echo base_link rslidar` 恒等 TF 不可依赖，以卷尺实测为准）与 Q13 外参参照。**开** |
| **Q7** | **✅ 关闭（有解）** | `/rslidar_points_filtered` = C16 时代 ROS2 栈遗留（源即 `faster_lio_online.yaml` 的 `lid_topic`，ROS2 格式）；ROS1 生产链 c16.yaml 从未用它（用 `/rslidar_points`）。沙箱词汇表维持 `/rslidar_points`，与 v4 C1 裁决一致，无需回改 |
| **Q9** | **✅ 关闭（裁定二）** | S9 判无用，无需安装；回退触发条件①删除 |
| **Q12** 🆕 | **✅ 关闭（2026-10-04，A 不可用按 §5.1 走 B）** | **imu_topic 现场验证**：`/imu/data`（qnx2ros，判据 ≈200 Hz + UDP 43897 bind 成功）与 `/livox/imu`（驱动内置，≈200 Hz）双测；主选 A 不可用时停走变更流程（§5.1）。**闭环（P3b 首采）**：A 不可用——UDP 43897 被 root 侧 jetson2motion 占用，qnx2ros bind error 死亡（主理人主机侧实测复现）；按 B 切 `/livox/imu` 实测 **199.9 Hz** + Mid-360 工厂外参（Q13 配套），diff 台账两条已登记（commit a821e01），Q12 修复后首张有效图落盘 |
| **Q13** 🆕 | 开 | **外参来源矛盾现场裁决**：`MID360s_config.json`（pitch 18°、x=187、z=400）vs `msg_MID360s.launch` 注释 TF（0.18786/0.12902、pitch 0.1309 rad≈7.5°）两来源互相矛盾；当前以 `extrinsic_est_en: true` 在线估计兜底，现场卷尺实测后决定是否固化初始外参 |

### 7.2 R 表（§13）

| # | 修订 |
|---|---|
| **R15** | 对账行更新：上游真实发布名 = 驱动 remap 后 `/rslidar_points`（msg_MID360s.launch 实测）✅、`/imu/data`（qnx2ros.cpp:53）✅、`/Odometry`（laser_mapping.cc:264）✅ |
| **R17** | 口径同步裁定二：残余风险表述中"加载路径"部分收窄——原生回退加载器存在性已实证（X3/E1），残余仍为 costmap 挂载命名空间与 class 串实际可加载性（Q11，P6 复核） |
| **R20** 🆕 | **外设类条目必须实机核验**。教训（A3 翻案）：v3/v4 核验对象是软件产物（launch 文件、驱动源码、日志），未对实机做流量级/目视核验，导致 C16 假设带病进入 v4。规约：凡涉传感器/外设型号、接线、端口、安装几何的条目，登记前必须有**被动抓包或实机目视**级证据；软件产物证据只能旁证 |

---

## §8 台账、工期与状态（X10）

### 8.1 追加 `note/04_配置改动diff.md`（两条）

```
- v4.1/X1 雷达身份翻案：A3 Leishen C16 → Livox Mid-360（抓包 56300/56301+56400/56401，C16 2368/2369 零流量；生产栈 mid360_ws+Livox_SDK2+MID360s_config.json host 103/lidar 201；实机目视穹顶+保护罩）；M0 驱动 lslidar→livox_ros_driver2（include msg_MID360s.launch，/rslidar_points 契约不变）；C1 注释修正（lidar_type 4=C16Handler 分支）；Q3 重立/Q7 关闭/Q12/Q13 新增。2026-10-04
- v4.1/X3 裁定二：S9（nav_core_adapter）判定无用——MBF 原生 nav_core 回退加载器（costmap_navigation_server.h:236-241 三对 ClassLoader）+ nav_core_plugins.xml 仅 nav_core base（方向=move_base 用 mbf 插件）+ apt rdepends 零消费；Q9 关闭、回退触发①删除；不安装、不卸载。2026-10-04
```

### 8.2 §15 工期修订

| Phase | 修订 |
|---|---|
| S9 行 | **删除**（X3） |
| P3b | 1–2 h → **1.5–2.5 h**（+Q3 体检与 Q12 双 IMU 验证 ≈0.5 h；作业单见 `/tmp/p3b_runbook.md` 收编） |
| P5 | +S10 核查 5 min（已装则零安装） |

### 8.3 §17 状态追加

- ✅ **v4.1 两项事实裁定收编**：①A3.1 雷达身份翻案（Mid-360，四条独立证据 + 生产"伪装链"拓扑实测，M0 驱动翻案而话题契约不变）；②S9 判无用（MBF 原生双加载器实证，Q9 关闭）。
- ⏸ 新增待确认：Q12（imu_topic 双测）/ Q13（外参矛盾裁决）；Q3 重立（Mid-360 安装几何）；R20 生效（外设条目实机核验规约）。

---

## 附：给施工（小明）与验收（小美）的注记

**施工**：
1. 执行序：P2 增补复制（§3 第 18/19 条，含 `mkdir -p opt/3rdparty`）→ §7.4 扫描（18 预期 0 命中）→ S10.0 核查（缺才 S10.1–S10.3）→ P5 批 1（§6.2 白名单）→ 其余按 v4。
2. 权限：`opt/3rdparty/` 参照 `opt/ref/`（属主 ros1-slam、目录 555）；S10.2 的 sudo make install 在 P0.7 会话内执行。
3. 全程不写 `/home/ysc`；Livox_SDK2 编译只发生在 `/home/ros1-slam/opt/3rdparty/Livox_SDK2/build`。

**验收**：
- G2 复验项：`find /home/ros1-slam/ws/src/00_drivers/livox_ros_driver2 -name "*.xml"` 可解析、`package.xml` buildtool=catkin；`grep -rn "/home/ysc" ws/src/00_drivers/livox_ros_driver2 ws/src/10_slam/faster_lio` 期望 0。
- G3a 项不变；新增：`rosparam load faster_lio_outdoor.yaml`（v4.1 版）无报错且 `preprocess/lidar_type` == 4。
- G4 增一行：`ls /usr/local/lib/liblivox_lidar_sdk_static.a` 存在（S10 核查证据）。
- P6 判据衔接：`rostopic hz /rslidar_points` ≥9 Hz（判据 2 原文不变，生产者名变为 livox）；`rosnode list | grep ros2qnx` 在 mapping/stub 模式为空（V1 原文不变）。
