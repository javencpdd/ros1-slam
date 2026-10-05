# 21 号计划书 v4.2 补丁：仿真测试台两层入册（坑14 修订 + Q14 落定 + M2 计划）

> **性质**：v4 补丁稿（v4 正文一字不动，v4.1 补丁稿同样不动；本文件按条目给出修订内容与落点，由主理人裁定收编后落位 `note/计划书v4/`，与 v4/v4.1 并列）。
> **产出**：部署架构师 小张，2026-10-05。
> **证据纪律**：全部证据为已发生事实（G8sim 两轮 FAIL 关闭、仿真测试台 G-G8b PASS、GazeboM1 PASS、验收报告 O1–O5、02 坑册、04 台账），只读引用不编造；拿不到实证的一律标 Q。
> **用户裁定（最高优先级，2026-10-05）**：真机手动操作项全部**暂存**（狗充电门控），主线走自动化仿真；G8/G9 真机判据不删不改，只标状态。

---

## v4.2 变更日志（X1–X7，相对 v4.1）

| 级别 | # | 变更（内容 + v4 落点） | 原因（已发生实证） |
|---|---|---|---|
| 🟡 边界 | **X1** | **坑14 修订**：仿真保真度边界精确化——「规划仿真≠物理仿真」。EGO-Planner 规划仿真（on-rails/无扫描几何/无物理 IMU）不可验估计层【维持】；**Gazebo 物理仿真（fratopa Mid360 插件 + planar_move + 物理 IMU 200Hz）已实证可验 faster_lio**（LIO 轨迹 vs 仿真真值全程偏差 1.5cm = 0.06%，四元数恒等）【新增】；hdl 仿真验证属 M2 待证【边界外延】 | GazeboM1 小美验收「核心命题结论」：02 坑14 判死的 render 仿真局限在 Gazebo 物理仿真下不复存在 |
| 🟠 入册 | **X2** | **仿真测试台两层正式入册为回归工具**：①规划层 G-G8b（mode_nav_sim + sim_nav_smoke 13 CHECK）②物理层 GazeboM1（mode_nav_gazebo + m1_smoke 四判据，M2 起升格 m2_smoke）——各自入口命令/判据/边界成文；三层地图体系随记 | 主理人「仿真可用化」裁决 + Q14 待落定项收口 |
| 🟠 计划 | **X3** | **M2 施工计划**（5 项数值判据）：M2-1 planar_move yaw 语义诊断+修复（先诊断后修）→ M2-2 teb 物理闭环 → M2-3 hdl 重定位 Gazebo 版 → M2-4 判据③回归 → M2-5 勘误 O1/O4/O5 落账。施工包=/tmp/m2_payload + /tmp/m2_manifest.md | GazeboM1 验收报告 M2 移交清单①–⑤ |
| 🟢 裁定 | **X4** | **Q14 落定**：仿真测试台=**正式回归工具**（非 Gate 替代）——每次触导航/SLAM 配置与代码，跑两层 smoke 作回归前置；G8/G9 仍真机正路，仿真 PASS 不豁免真机 Gate | 主理人 G8-sim 关闭裁决 + GazeboM1「非计划书 G0–G9 阶段」定位收编 |
| 🟡 台账 | **X5** | **G8-sim 预演史收编 + 勘误 O1/O4/O5**：两轮返工 FAIL 记录（坑14 佐证素材）、并行会话端口争用仲裁（c260789 唯一落账先例）、O1（176→1587 口径）/O4（env.sh 重复 3 行）/O5（opt/upstream、.sdformat 归置）登记文案 | G8sim/仿真测试台/GazeboM1 三份进度文档 |
| 🟡 Q/R | **X6** | **Q13 增补倾向值（7.5°，G8 残差反推）**；R20 之外新增教训条目 R21–R23（rosrun 同名顶替、fratopa 自身 collision 包射线、SDF `_topic` 类参数以 strings 实证为准） | 04:99、D-M1-5、m1 manifest F 节 |
| 🟡 工期 | **X7** | **§15 工期修订**：G8/G9 真机项（含 P3b 加密重采）标「⏸ 暂存待狗」；新增 M2 工期 4–6 h（仿真主线，无狗依赖） | 用户裁定 + M2 施工包预估 |

---

## §1 X1：坑14 修订（02 坑册登记文案，替换原坑14 末段）

**原坑14 措辞**：「G8-sim 仿真预演终止……该仿真器为 EGO-Planner 规划仿真，非物理 LIO 仿真……faster_lio 在其上的发散是传感器仿真保真度不足」。

**v4.2 修订（边界精确化，原结论维持）**：

| 仿真形态 | 传感器/位姿/IMU 来源 | 可验证层 | 判死/实证依据 |
|---|---|---|---|
| **规划仿真**（EGO-Planner render，95_sim.launch 链） | render 点云**无扫描几何**（ray-fill 幻影点）、位姿 **on-rails** 回放、IMU 为位姿差分**合成** | 仅**规划/导航层**（M4/M5/M6 活性+正确性） | G8-sim 两轮返工 5+3 连发散 FAIL（坑13/14，2026-10-05 关闭）——估计层判死【维持】 |
| **物理仿真**（Gazebo，fratopa Mid360 插件 + planar_move + Gazebo IMU 200Hz） | fratopa ray 传感器**真扫描几何**（飞点 0.0000%）、物理引擎驱动位姿、**物理 IMU**（噪声模型在位） | **faster_lio 已实证**（M1）；**hdl 待证**（M2-3） | GazeboM1 PASS（2026-10-06）：570 帧/52 万点/飞点 0%，faster_lio 自身轨迹 vs 真值全程偏差 **1.5cm（0.06%）**、四元数恒等——traj.txt CSV 独立抽验 |

**登记口径**：坑14 的正确表述自 v4.2 起为「**规划仿真不可验估计层**；Gazebo 物理仿真**已实证可验 faster_lio**（判据=轨迹偏差 1.5cm/0.06%），hdl 属 M2 待证；两层均不可替代 G8/G9 真机 Gate」。M1 残余（闭环掉头错位 26.4m）已归因 harness/插件层（planar_move odom yaw 语义），**不构成估计层反证**。

## §2 X2：仿真测试台两层入册（新增计划书 §11.6「仿真回归测试台」）

### 2.1 规划层 · G-G8b（已验收 PASS，2026-10-05）

- **入口**：`roslaunch lite3_bringup mode_nav_sim.launch`（私有 master 11312；use_gpu:=false 实测降级）＋无人值守冒烟 `sim_nav_smoke.sh`（13 CHECK 全 PASS，SMOKE_EXIT=0）。
- **判据**（13 CHECK 摘要）：mockamap 发布在位、/rslidar_points ≥0.5Hz（实测 3.62）、/map 占据格（5041）、map→base_link TF、全局 costmap 占据（11562）、GetPath×3 outcome=0 且末点偏差 0.00m、ExePath 后 /cmd_vel_raw 有输出、/cmd_vel 写者唯一=safe_cmd_vel_mux、ros2qnx 全程不存在、退净零残留。
- **边界**：位姿=on-rails 真值回放（sim_pose_tf @20Hz），cmd_vel 不驱动仿真狗——**非闭环、非估计层**（坑14 边界）；无任何估计层节点。
- **判据常数备案**：GetPath 长度判据 `max(4×直线距, 34.8)+2`（树直径口径 33.6→34.8 已勘误，smoke 代码不动）。

### 2.2 物理层 · GazeboM1→M2（已验收 PASS，2026-10-06；闭环项 M2 收口）

- **入口**：`roslaunch lite3_gazebo_sim m1_gazebo.launch`（headless gzserver，GAZEBO_MASTER_URI=11355 + ROS master 11313 隔离域）＋`m1_mapping.launch`＋无人值守 `m1_smoke.sh`（**四判据**）。M2 起入口升格 `mode_nav_gazebo.launch`（M2 施工包已含）。
- **四判据**（M1 实测值）：①/rslidar_points ≥5Hz（实测 13.2，全窗折算≈10=插件名义，O3 口径注记）；②/livox/imu ≥100Hz（实测 264.4，折算≈200）；③闭环直行 5m 回原点 /Odometry 漂移 ≤0.5m（**实测 26.411 FAIL，归因 harness/插件 yaw 语义，M2-1/M2-4 收口**）；④/cloud_registered 出图且 |z|>10m 飞点 <1%（实测 0.0000）。
- **边界**：估计层**可验**（faster_lio 已实证、hdl M2 待证）；/cmd_vel 仅 sim 域发布、11312 生产 master 全程零监听、foxy 栈（jetson2motion）零接触——隔离红线与真机同款。
- **M1 关键实证（入册依据）**：物理运动 26.4m 全程出图 570 帧/522,883 点/飞点 0%，LIO 轨迹 vs 真值 1.5cm（0.06%）——物理仿真可支撑 LIO 类算法仿真验证。

### 2.3 三层地图体系（随记，两层测试台共用语境）

| 层 | 产物 | 来源 | 消费方 |
|---|---|---|---|
| raw master | `data/maps/lite3.pcd`（faster_lio 存盘，V2 唯一写穿点，样张 sha256 c034c906…） | 真机采图（P3b，⏸ 暂存待狗）/仿真另存 | hdl globalmap（lite3） |
| 工作图 4 字段 | sim_venue 类 `<name>.pcd` + `hdl_localization` 参数化 map_name | M2-3 仿真建图（备份/恢复样张红线内） | hdl 仿真重定位 |
| 2D | /map OccupancyGrid（规划层=sim_map_bridge 真值图；物理层=m2_free_map 免费格底图+障碍层实时注入） | mockamap / 自研发布器 | costmap static_layer |

## §3 X3：M2 计划（施工包=/tmp/m2_payload + /tmp/m2_manifest.md，小张终稿）

| 项 | 内容 | 数值判据 | 顺序 |
|---|---|---|---|
| M2-1-D | **诊断**：planar_move yaw 语义探针（m2_probe_yaw）——纯转 5s 记录 odom yaw vs gt yaw vs IMU 姿态，三分支定因（H1 odom 开环×物理可转 / H2 物理不可转×odom 混合） | 出对账表，根因落 02 | **先行，先诊断后修** |
| M2-1 | **修复**：修法 A（harness 状态机改 IMU/真值反馈闭环）保 M1 判据回归；修法 B（自研 lite3_planar_drive 替身插件入壳，odom/TF 全闭环，dynamic/kinematic 双模式）为 M2-2 硬前置 | turn/back 双 done；drift ≤0.5m | D 后 |
| M2-2 | **teb 物理闭环**：mbf/teb 输出 cmd_vel_raw → relay → /cmd_vel 驱动伪狗（M1 已证 cmd_vel 可驱动）；免真值、免 static 真值图（m2_free_map 底图+障碍层实时） | 3m 目标闭环到达偏差 **≤0.35m** 且 **≤120s**（odom 判，gt 参考） | 依赖 M2-1B |
| M2-4 | **判据③回归**：M1 漂移判据在闭环修复后重测 | 直行 5m 回原点 /Odometry 漂移 **≤0.5m**，四判据全 PASS | 与 M2-1 同跑 |
| M2-3 | **hdl 重定位 Gazebo 版**：仿真建图存 sim_venue.pcd（样张 pcd 备份/恢复 sha256 红线）→ hdl map_name:=sim_venue 载入 → /initialpose 注入真值邻域 | 静置收敛 **≤0.3m / 10°（≤10s）**，30s 内无 **>0.5m** 跳变 | 依赖 M2 建图 |
| M2-5 | **勘误落账**：O1/O4/O5 登记文案（§5） | 文案入 02/04 | 随批 |

## §4 X4：Q14 落定

- **仿真测试台 = 正式回归工具**（入册 §11.6）：凡触 M4/M5/M6 配置与代码 → 规划层 smoke 回归；凡触 faster_lio/hdl/驱动/adapter/sim 包 → 物理层 smoke 回归。**非 Gate 替代**：G8/G9 仍真机正路，仿真 PASS 不豁免真机 Gate、不改动 G0–G9 判据原文。
- Q14 状态：**✅ 关闭（v4.2 X4 收编）**。

## §5 X5：G8-sim 预演史 + 勘误

### 5.1 两轮返工 FAIL（坑14 佐证素材，保持结论不翻案）

| 轮 | 对策 | 结果 |
|---|---|---|
| 首轮 5 连 | adapter 三真 bug 修复（陀螺奇异回退/stamp 域/点云戳） | 里程计发散 1e3~1e5 m |
| 返工 R1 | 幻影 zcut 过滤/min_raylength 0.3/外参锁定/time 全 0/戳配对 | 3 连回归 FAIL（漂移 5.9–11 万 m） |
| 返工 R2 | 轨迹运动形态重构（物理三缺陷归零，trajcheck |a|≤1.0056m/s²） | 3 连回归 FAIL（同形态）——「非物理运动形态」假设被实验证伪 |

**收编结论**：R2 排除法（静置零漂 0.019m、原地转向 0.069m 干净、平移起步 onset 锁定）+ GazeboM1 正向实证，共同支撑「规划仿真保真度不足 ≠ 我方 LIO/adapter 缺陷」——坑14 v4.2 修订的两条证据腿。

### 5.2 并行会话仲裁（流程先例入册）

M1 期间并行会话端口争用（11312/Gazebo master）：主理人仲裁「双方停手、互证采纳、c260789 唯一落账」。**规约**：仿真主线单会话承载；确需并行时先在进度文档登记窗口与 master 端口分配（11312 生产 / 11313 sim-ROS / 11345 默认 / 11355 sim-Gazebo）。

### 5.3 勘误清单（移交小明登记 02/04，文案随 M2-5）

| # | 勘误 | 更正 |
|---|---|---|
| O1 | 「No point 176→3」口径不可溯源 | 实为 **1587→3**（三件留证一致：final4 mapping.log / payload logs-20261006_014656 / glog；NoPoint=TooFew=1587） |
| O4 | 04 台账尾部 env.sh 重复登记 3 条（108=109 逐字同、110 改写重复） | 删 109/110 两条，留 108，注明勘误 |
| O5 | opt/upstream/（U1–U6 持久仓）与 .sdformat/ git 未跟踪 | 归置裁定：建议 opt/upstream 入库（来源可追溯）、.sdformat 入 .gitignore——**Q15 待主理人裁定** |

## §6 X6：Q/R 表增量

| # | 变更 |
|---|---|
| **Q13** | 增补倾向值：俯仰 **7.5°**（G8-sim/Gazebo 残差反推口径，与 msg_MID360s.launch 注释 TF 一致）；仍以现场卷尺/量角器实测终裁，⏸ 暂存待狗 |
| **R21** 🆕 | **rosrun 同名节点顶替**：后者踢前者（04:99 实证）——常驻发布器一律节点名分离（pcd_map_pub/pcd_raw_pub 先例），禁止同名双起 |
| **R22** 🆕 | **传感器自身 collision 包射线**：fratopa 射线起点恒=传感器原点，落于任何自身 collision 内即最近命中 <range.min 被置 0（D-M1-5 全零点根因）——取舍声明④：**sensor 起点不得落于任何自身 collision 内**；上游 model.sdf 以 sensor pose 抬 0.05 避让同款坑 |
| **R23** 🆕 | **SDF/插件参数以本机 strings/头文件实证为准**：文档参数可能不存在（planar_move 此版无 commandTimeout 键，manifest F.1）、可能无效（SDK2 bd_list 对 Mid-360 无效——按 config IP 识别设备）、写法敏感（gazebo_ros_imu topicName 绝对路径）——入链前逐键实证，不复述文档 |

## §7 X7：工期修订（§15）

| 项 | v4.2 状态 |
|---|---|
| P3b 采图/加密重采、G8 真机、G9 真机 | **⏸ 暂存待狗**（用户裁定：充电门控 + 现场量 2 数 + 牵行许可；判据原文不动） |
| Q3 安装几何 / Q13 外参终裁 | ⏸ 随真机窗口（现场量） |
| **M2（新增）** | **4–6 h**：M2-1-D 诊断 0.5h → M2-1 修复+M2-4 回归 1–1.5h → M2-2 teb 闭环 1–1.5h → M2-3 hdl 1–1.5h → M2-5 落账 0.5h（仿真主线，无狗依赖，单会话） |
| 主线推进序 | M2 仿真收口 → （待狗）P3b 补采 → G8 → G9 |

## §8 台账与状态

- 04 台账追加：v4.2/X1 坑14 修订、X2 两层入册、X4 Q14 关闭（三条文案随 M2 manifest 附）。
- §17 状态追加：✅ G-G8b PASS（2026-10-05）、✅ GazeboM1 PASS（2026-10-06，闭环项划转 M2）、⏸ G8/G9 真机暂存待狗、🆕 M2 施工包下发。
- 本补丁不动 v4/v4.1 任何正文；与 M2 施工包（/tmp/m2_manifest.md）配套生效。

---

**附：给施工（小明）注记**：M2 按 manifest 执行序「先诊断后修」；全部改动限自研壳（lite3_gazebo_sim/lite3_bringup/lite3_localization），系统包与拷贝包零改动；样张 pcd c034c906… 备份/恢复 sha256 红线贯穿 M2-3。
**附：给验收（小美）提示**：M2 验收=四判据回归全 PASS + teb 闭环 ≤0.35m/120s + hdl 三判据 + O1/O4/O5 勘误对账 + 隔离复核（11312 零监听/foxy 快照/拷贝包 diff）。
