# 21 · 103 主机 `ros1-slam` 用户：模块化架构隔离部署计划书（v4）

> **状态：计划书（Plan）。v4 = v3 的设计模式修订版**——设计渊源：ROS 2 navigation2 联邦式架构对标分析的**八个已验证设计决策**，ROS 1 落地映射见 **§2.6**。**P0–P2 已按 v3 执行完毕（2026-10-04，G0/G1/G1.5/G2 全 PASS，见 `note/进度/00_总览.md`），正文相应章节冻结注记（N9）；v4 变更仅作用于 P3a 及以后。本文不执行**：不创建目录、不复制文件、不修改 103 上任何内容；**S9 仅登记为 sudo 清单项**（§5 P0.4 / §15 Q9），不把"安装"写成已完成。
> 基线来自 2026-10-04 对 `lite3-f20-1-103`（192.168.1.103，即本机）的二次实测 + **P3a.0 导航栈组件只读实测**（`/tmp/P3a0_mbf组件实测.log`，690 行——**mbf 消息字段名、插件 type 字符串、参数名的唯一权威来源，引用以它为准**）。
> v3 骨架（阶段 DAG、Gate、双目录制、回滚矩阵、风险表）全部保留；v4 落位 `note/计划书v4/`（新目录；v3 原文件一字不动，N10）。

---

## v3 变更日志（相对 v2，按严重程度排序）—— v4 原样保留，只增不改

| 级别 | # | 变更 | 原因（实测证据） |
|---|---|---|---|
| 🔴 安全 | **V1** | **`message_transformer` 拆分启动**：`ros2qnx`（订阅 `/cmd_vel` 直发 QNX 底盘，`ros2qnx.cpp:112`）只在真机 mode 启动；stub/mapping/replay mode **严禁**。M0 一期只起 `qnx2ros`（产 `/imu/data`，`qnx2ros.cpp:53`） | v2 的 `mode_nav_stub` 起完整 M0 会把 stub 假速度真发到底盘 |
| 🔴 事实 | **V2** | **faster-lio 存盘路径硬编码写穿隔离**：`laser_mapping.cc:751` 硬编码 `/home/ysc/lite_cog/system/map/lite3.pcd`，复制后建图会直接写回 ysc。处置：复制后立即改写为 `/home/ros1-slam/data/maps/`，并作为**唯一许可的第三方包源码改动**登记 diff | 实测源码；v2 隔离检查只扫 `lite3_*` 自研壳，扫不到第三方包内部——盲区已用自检 [8] 补上 |
| 🔴 事实 | **V3** | **`faster_lio_online.yaml` 是 ROS2 格式**（`/faster_lio_online: ros__parameters:`）。v2 清单第 16 条"ROS1 格式可直接用"为误判。v3 给出完整 ROS1 转换版（§C1） | 实测文件头部；这正是 v2 自己 R7 警告的坑 |
| 🔴 事实 | **V4** | **`lidar_type: 4`（Velodyne 分支）**，非 v2 所说"用 2"。全 yaml 见 §C1，勿"修正" | `faster_lio_online.yaml` 实测 + `pointcloud_preprocess.cc:20` switch 对照 |
| 🔴 事实 | **V5** | **M0→M1 话题断链**：faster-lio 订阅 `lid_topic: /rslidar_points_filtered`，但该话题全库**无生产者**（驱动只发 `rslidar_points`）。处置：转换 yaml 时改为 `/rslidar_points` | 全库 grep 零命中 + `lslidar_driver.cpp:159` |
| 🟠 流程 | **V6** | **P3 时序修正**：建图需要已编译的工作区，v2 却把"采第一张图"放在编译（P5）之前。拆为 P3a（纯写配置）与 P3b（P5 后现场采图），G3 移到 P3b | 逻辑硬伤 |
| 🟠 流程 | **V7** | **新增 P1.5 自研壳脚手架**：批 5 要编译 8 个 `lite3_*` 包，但 v2 从无一步生成它们的 package.xml/CMakeLists | 逻辑硬伤 |
| 🟠 流程 | **V8** | **新增 C0 配置种子**：C2/C3 要修改的 `planner_teb.yaml`/`costmap_common.yaml`，v2 从未提供底稿来源。P3a 第一步从 `opt/ref/` 拷入自研壳再改 | 逻辑硬伤 |
| 🟠 流程 | **V9** | **权限块时序修正**：`chown -R` 移到 P2 复制完成之后（v2 标注"Phase 1 执行"，复制还没发生，白做） | 逻辑硬伤 |
| 🟠 脚本 | **V10** | **`v2_isolation.sh` 重写为修复版（8 项）**：修 [2] `grep|head && FAIL=1` 恒真 bug、[5] 管道子 shell 丢 FAIL bug；新增 [8] 全 ws 扫描（补 V2 盲区）；`env.sh` 加 devel 存在性守卫 | v2 脚本 [2] 永远 FAIL、[5] 违规检测不到 |
| 🟡 冲突 | **V11** | **ACL 与 chmod 冲突处置**：v2 §3.3 的 `chmod 750/755/644` 会把本机已有的 `user:ysc:rwx` ACL mask 压成 r-x，静默废掉 ysc 写权限。P0 先 `setfacl -b -R` 清干净，按 v2 纯属主模型走 | 2026-10-04 上一轮操作给 `/home/ros1-slam` 设了默认 ACL |
| 🟢 优化 | **V12** | **不复制、不编译 `teb_local_planner`/`move_base`/`map_server`**：三者系统已装（`/opt/ros/noetic/share/`，teb 版本 0.9.1 与 lite_cog 源码一致）。删 v2 复制清单 6/11/12 + 编译批 4。二期真要改 teb 源码再走"复制+编译+遮蔽"流程 | 实测系统包齐全 |
| 🟢 优化 | **V13** | **R11 降级、Q2 关闭**：`message_transformer` 的 include 实测全部自包含（自家 `protocol.h`/`sensor_logger.h`）+ 标准 ROS 头，catkin 标准包依赖，无隐藏 SDK | 实测源码 |
| 🟢 优化 | **V14** | **编译前临时 swap**：本机 zram 仅 2×571 MB，`-j3` 编 `fast_gicp`+`faster_lio` 仍有 OOM 余量风险。P5 前加 4G swapfile，编完即撤 | 实测 `swapon --show` |
| 🟢 优化 | **V15** | 小项：路径改写白名单补 `*.rviz`/`*.cfg`；sudo 每阶段保持会话；`location_record.py` 实际在 `pipeline_tracking/tools/`（v2 误写 scripts/） | 实测 |

---

## v4 变更日志（相对 v3，N1–N10）

| 级别 | # | 变更（内容 + 落点） | 原因（实测证据） |
|---|---|---|---|
| 🔴 架构 | **N1** | **M4 服务层从单体 move_base 升级为 mbf_costmap_nav 0.4.0 联邦式**（Nav2 联邦式架构的 ROS 1 现成实现）。实测依据：teb 插件描述符**双注册**（同时声明 `nav_core::BaseLocalPlanner` 与 `mbf_costmap_core::CostmapController`，可被 mbf 原生加载）；四 action Result 均带 `uint32 outcome + string message` 错误码；Goal 均带按名点单字段（GetPath: `planner`，ExePath: `controller`，Recovery: `behavior`，MoveBase 另有 `recovery_behaviors: string[]`）。**前置 S9**：`apt install ros-noetic-nav-core-adapter`（0.3.0，官方源）——仅全局规划器（global_planner/navfn 只注册了 `nav_core::BaseGlobalPlanner`）需要适配；**用户不批 S9 则整体回退 move_base 单体形态**（回退决策表见 §8：move_base 形态下 C11/C12/C13 保留，错误码退化为 move_base actionlib 三态 outcome，C11 降级为简化映射）。落点：§1.1（A11/A12）、§1.2（P3a.0 实测行）、§2.6、§8（C0 重写 + 回退表）、§15（Q9/Q10/Q11）、§5 P0.4（S9） | P3a.0 只读实测（寇豆码，2026-10-04，690 行 log，裁决可行） |
| 🟠 规约 | **N2** | 新增 **§2.6 导航栈设计规约**：八决策映射表 + 硬规则——lite3_navigation 壳包只许 launch/config/scripts/verify（**policy 层**），算法一律系统插件（**mechanism 层**），壳包内禁出现第三方算法源码；隔离自检新增 **[9]**：`find /home/ros1-slam/ws/src/40_navigation -name "*.cpp" -o -name "*.cc" -o -name "*.h"` 期望 0 命中。落点：§2.6、§11.2（[9]）、§8 G3a | 用户拍板 + P3a.0（teb/global_planner 均为系统插件） |
| 🟠 诊断式恢复 | **N3** | 新增 **C11** `lite3_navigation/config/recovery_policy.yaml`——恢复原子动作注册 + 错误码→处置分级映射（局部重试 / 重规划 / 清图重规划 / 中止上报），错误码分类**以实测 log 中 GetPathResult/ExePathResult/RecoveryResult 的 outcome 常量表为准**（GetPath 15 档 SUCCESS=0…MAP_ERROR=62）；**C5 升级**：TaskTransfer.py 副本从盲目 `MAX_RETRY=3` 改为读 recovery_policy.yaml 分级重试，登记 04_配置改动diff.md。落点：§8 C11/C5、§3 | P3a.0（四 Result 错误码常量表实测） |
| 🟠 词汇库契约 | **N4** | 新增 **C12** `lite3_task/config/task_primitives.yaml` 任务原语注册表 v1——navigate_to_pose / get_path / exe_path / recover / wait / clear_costmaps 六原语，每原语 = {action/service 目标， 类型， 关键 outcome 语义}；一期 M5 用系统 **SMACH 2.5.2**（实测可 import/实例化）写最小顺序状态机验证契约；py_trees 列二期（外部依赖 R19）。clear_costmaps 原语的服务挂载点实测未覆盖 → 标注"联调确认"（并入 Q11）。落点：§8 C12、§15 Q10/Q11、§16 | 用户拍板（契约先行）+ P3a.0（smach 2.5.2 ✅） |
| 🟠 lifecycle 模拟 | **N5** | 新增 **C13**——lite3_supervisor（SUP，90_supervisor 壳包，v3 从未赋职责）正式定职责：按 mode 的有序启动清单、模块 1Hz 心跳（liveness topic）监控、失联处置（通知 M6 立即零速 → 降级关停本 mode 模块）；一期 scope = health_watch.yaml（watch 清单：模块名/心跳话题/超时）+ 最小监控脚本骨架。**红线写死：SUP 绝不直接写 /cmd_vel，/cmd_vel 唯一写者永远是 M6 safe_cmd_vel_mux**（入 §2.6 规约与自检 [6] 语义）。落点：§8 C13、§2.6、§11.2 [6]、§12 mode 表、§13 R18 | Nav2 决策 7（lifecycle+bond）的 ROS 1 模拟 |
| 🟠 纵深管道 | **N6** | C7/C8 内容保留，补管道规约：`controller(M4) → /cmd_vel_raw → safe_cmd_vel_mux(M6) → /cmd_vel → ros2qnx（仅真机 mode_navigation）`；collision_guard（多边形安全区急停）登记二期 §16。落点：§2、§2.6、§8 C7/C8、§16 | Nav2 决策 8（cmd_vel 纵深防御） |
| 🟡 吸收坑1/Q8 | **N7** | P0.5 双查正则扩充——传感器侧 `lslidar\|faster_lio\|hdl_\|roscore\|rosmaster` + 运动链路侧 `ros2qnx\|message_transformer\|lite3_ros_bridge\|jetson2motion\|jetson2app\|transfer_launch`（合并正则见 §5 P0.5）；**Q8 硬停止线正文化**：root 侧 foxy 栈（jetson2motion 订阅 /cmd_vel 直发底盘）与 ysc monitor 桥在跑期间，P6/P7 沙箱起节点（stub）前必须重跑 P0.5 双查**并与用户确认处置**。落点：§5 P0.5、§11/§12 前置、§15 Q8 | 坑1/坑3（`note/02_问题与坑.md`；实机 ps 4 行长期在跑） |
| 🟡 Gate 增强 | **N8** | **G3a 增两条**：`rosparam load mbf_navigation.yaml` 无报错且 `planners/controllers/recovery_behaviors` 三列表非空；插件 type 字符串与 P3a.0 实测 log 中的插件描述符逐一比对一致。**G5/G6 增两条**：mode_nav_stub 下 `rostopic list` 可见 mbf 三 action 话题族；SUP 心跳话题按 health_watch.yaml 清单全部存活。**G4 增**：`rospack find mbf_costmap_nav` 指向 /opt/ros/noetic（证明系统版，同 teb 的 A9 校验）。落点：§8 G3a、§10 G4、§11.3–11.5 | mbf 形态的可验收性 |
| 🟡 冻结注记 | **N9** | §5/§6/§7（P0/P1/P2）头部标注"✅ 已按 v3 执行完毕（2026-10-04）·内容冻结，仅 ①P0.5 正则扩充（N7）与 ②S9 新增 ③坑2/坑4 执行注记对后续阶段生效"；P0–P2 其余正文原样保留不重写事实（Gate 结果、日期以 `note/进度/00_总览.md` 为准）。落点：§5/§6/§7 头部 | 00_总览.md：G0/G1/G1.5/G2 全 PASS（2026-10-04） |
| 🟢 台账与工期 | **N10** | v4 落位 `note/计划书v4/`（新目录；v3 原文件一字不动，旧版保留）；§13 风险表增 **R17**（mbf 参数/底稿格式差异——已实测给出转换表，残余风险=costmap 挂载命名空间未实测）/ **R18**（SUP 误碰 cmd_vel——规约+自检 [6] 防线）/ **R19**（py_trees 外部依赖，二期评估）；§15 工期调整（P3a 1–2h→2–2.5h 含 C11–C13；P6 +0.5h SUP 联调；S9 用户侧 5 min）；§16 二期入口增补四条。落点：§13/§15/§16、文档头部 | 台账一致性 |

---

## §0 三份文档的分工

| 文档 | 回答的问题 | 本文怎么用 |
|---|---|---|
| **19 号** | 场景选择、SCAN/PCT 归位、施工次序 | 提供次序（W1–W7）与验收判据（A1–A5 / B1–B5） |
| **20 号** | 模块切分、接口契约、launch 组织 | 提供骨架（M0–M6、`/cmd_vel_raw`、三层 launch、variants、Stub/Sink） |
| **21 号（本文 v4）** | 具体怎么在 `ros1-slam` 家目录建起来 | 目录树 + 属主权限 + 分阶段命令 + 校验点 + 回滚 + 风险 + 导航栈设计规约（§2.6） |

> **`ros1-slam` 是手术台，不是新家**（v2 §0.1 结论保留）：`ysc/lite_cog` 原样作为对照组与回退基准，单向复制（A6）。

---

## §1 假设与基线（v3 全部经二次实测；v4 增补经 P3a.0 只读实测）

### 1.1 假设

| # | 假设 | 依据 / v3·v4 变更 |
|---|---|---|
| **A1** | ROS = **Noetic**（`/opt/ros/noetic`），不碰 foxy | 实测双装（foxy + noetic），本文只用 noetic |
| **A2** | 20 号 M0–M6 七模块；一期实装 M0/M1/M2/M6 + M4 stub，M3/M5 占位 | 同 v2；v4 由 A11/A12 细化 |
| **A3** | 传感器 = Leishen C16（192.168.1.201）+ 狗体 IMU（`/imu/data`） | 实测 `lslidar_c16.launch`；`/imu/data` 由 **`qnx2ros`** 发布（`qnx2ros.cpp:53`） |
| **A4** | 网络 192.168.1.0/24，有出网 | 实测 |
| **A5** | 执行者 = `ysc`（有 sudo）代劳；`ros1-slam` 无 sudo、当前无法登录 | 实测；Q1 |
| **A6** | 隔离单向：`ysc/lite_cog` → `ros1-slam`，**绝不反向** | v3 新增强调：V2 写穿路径是唯一已发现的反向通道，已处置 |
| **A7** | 一期不引入 SCAN/PCT 源码，只预留 `opt/research/` | 同 v2 |
| **A8** | 与 ysc 侧不同时抢 LiDAR UDP（2368）**及底盘运动链路** | v3：P0.5 扩为"查传感器 + 运动链路"（V1）；v4：正则再扩充（N7） |
| **A9** | v3 新增：teb/move_base/map_server/mbf 用系统已装版本，不复制不编译 | V12；v4 P3a.0 复核：mbf 栈 0.4.0 全系系统包 ✅ |
| **A10** | v3 新增：`message_transformer` 四节点中，一期只允许 `qnx2ros`（+可选 `sensor_checker`）；`ros2qnx`/`nx2app` 仅真机 mode | 实测 `CMakeLists.txt` add_executable 清单 + `ros2qnx.cpp:112` |
| **A11** | **v4 新增**：M4 服务层 = **mbf_costmap_nav 0.4.0 联邦式**；全局规划器加载需 S9（`ros-noetic-nav-core-adapter` 0.3.0，Q9）——**不批则整体回退 move_base 单体**（§8 回退决策表） | P3a.0：teb 双注册可被 mbf 原生加载；nav_core_adapter 未装、apt 官方源可装 |
| **A12** | **v4 新增**：M5 一期 = **SMACH 2.5.2**（系统版）最小顺序状态机验证 C12 契约（细化 A2 的"M5 占位"）；py_trees 列二期（R19） | P3a.0：smach/smach_ros 2.5.2 import + StateMachine 实例化 OK（Q10 关闭） |

### 1.2 实测基线（2026-10-04 二次复核 + P3a.0 只读实测）

| 项 | 实测值 | 对计划的影响 |
|---|---|---|
| `ros1-slam` 用户 | `uid=1001 gid=1003`，**不在 sudo/video/i2c/gpio/plugdev** | P0.2 补组 |
| **`/home/ros1-slam` ACL** | `user:ysc:rwx` + **default ACL 已设**（2026-10-04 上一轮操作） | ⚠️ 与 §3.3 chmod 冲突 → P0 先 `setfacl -b`（V11） |
| SSH 登录 ros1-slam | ❌ Permission denied | 全程 `sudo -u` 代劳 → `as_ros1slam.sh` |
| 磁盘 / 系统 / 资源 | **43 GB 可用**（70G/117G）；Ubuntu 20.04.6 + gcc 9.4.0；6 核 / 6.7 GiB / **zram 2×571 MB** | 预算见 §1.6；⚠️ 编译加临时 swap（V14） |
| ROS 与库版本 | noetic；`tf2 0.7.7`、`pcl_ros 1.7.4`；PCL/Eigen/Boost/OpenCV/Ceres = 1.10.0 / 3.3.7 / 1.71 / 4.2.0 / 1.14.0 | 同 v2 |
| **系统已装导航栈** | `move_base` `move_base_flex` `mbf_costmap_nav` **`teb_local_planner`(0.9.1)** `map_server` `octomap_ros` `move_base_sequence` 全部 ✅ | **A9/V12：不复制不编译** |
| **导航栈组件实测（P3a.0，2026-10-04）** | mbf_msgs / mbf_abstract_nav / mbf_costmap_nav **0.4.0** ✅（服务器本体 `/opt/ros/noetic/lib/mbf_costmap_nav/mbf_costmap_nav`）；`nav_core_adapter` ❌ 未装（apt 官方源候选 **0.3.0**）→ S9；smach/smach_ros **2.5.2** ✅；teb 0.9.1 **双注册**；move_base 1.17.3 对照 ✅；**mbf_costmap_nav 不带样例 yaml/launch**；插件注册参数实测 `planners`/`controllers`/`recovery_behaviors`（`[{name, type}]` 元组列表） | **N1/N2/N4 落地依据；字段权威 = `/tmp/P3a0_mbf组件实测.log`** |
| 32 个关键 ROS 包 | ✅ 全装（P4 仅核对） | P4 仅核对 |
| `py_trees` / `twist_mux` | ❌ 均未装 | 编排用 SMACH（v4：A12 定案）；M6 自研 mux |
| **logrotate** | ✅ **已装并启用**（2026-10-04 修复） | 磁盘风险项消除 |
| 相机与 LiDAR | `/dev/video*` 666 + plugdev ACL（ros1-slam 可直读）；C16 以太网 UDP 2368/2369，驱动发布相对名 `rslidar_points`（`lslidar_driver.cpp:159`，launch 参数 `pointcloud_topic`）；当前 `ps aux` 无 lslidar/faster_lio/hdl/roscore（施工窗口干净） | — |
| **root 侧常驻进程（坑1/坑3）与施工窗口** | foxy 栈 `transfer_launch`/`jetson2app`/`jetson2motion`（jetson2motion 订阅 `/cmd_vel` 直发底盘）+ ysc monitor 桥 `lite3_ros_bridge`，ps 合计 4 行长期在跑；沙箱相关进程（lslidar/faster_lio/hdl/roscore）当前无 | **Q8 硬停止线（N7）**：P6/P7 起节点前重跑 P0.5 双查并与用户确认处置 |
| **faster-lio 关键事实** | `lidar_type: 4`；`lid_topic: /rslidar_points_filtered`（**无生产者**）；存盘路径硬编码 `/home/ysc/lite_cog/system/map/lite3.pcd`（`laser_mapping.cc:751`；**坑2：实测两处，751 与 862**）；参数经**全局 NodeHandle** 读取（`run_mapping_online.cc:25`） | V2/V4/V5 + §C1 |
| **message_transformer** | catkin ROS1 包，4 节点：`qnx2ros`（→`/imu/data` 200Hz）、`ros2qnx`（`cmd_vel`→底盘）、`nx2app`（joy）、`sensor_checker`；include 全自包含 | A10、V1、V13 |

### 1.4 继承自 19 号的五个硬事实（不变）

F1 TEB 未避障（`include_costmap_obstacles: false`）；F2 STVL 三坑（`clearing:false`/`min_z:0.2`/`obstacle_range:3.0`）；F3 `dyn_reconfigure.py` 失效且压速 0.1；F4 `system/map/` 为空；F5 顺序不能倒（补图→修避障→mux→稳 D2→SCAN→PCT；**G3→G8→G9 顺序不能倒**）。

### 1.6 磁盘预算（v3 微调；v4 无变化）

源码 ≈ 0.19 GB（少了 teb/move_base/map_server）；`build/` 3–6 GB（少了批 4，略降）；`devel/` 0.5 GB；
地图 + bag < 0.3 GB + 0.5 GB/趟（规则不变：bags 只留 3 趟）；**合计 ≈ 5–6.5 GB，余量 ≈ 36 GB ✅**。
> v4 注记：S9（nav_core_adapter）为 apt 系统包，装在 `/opt/ros`，**不占 ws 磁盘预算**。

---

## §2 目标形态（v3 修订点 + v4 修订点）

架构图、模块编号、`/cmd_vel_raw` 契约、`variants/`、Stub/Sink 全部继承 v2 §2。**v3 两处修订 + v4 一处修订**：

1. **M0 的 `message_transformer` 启动拆分**（V1）：

```xml
<!-- lite3_drivers/launch/01_sensors.launch —— 一期唯一允许的形态 -->
<node pkg="message_transformer" type="qnx2ros" name="qnx2ros" output="screen"/>
<!-- ⚠️ ros2qnx（订阅 /cmd_vel 直发 QNX 底盘）在此 launch 中绝对禁止出现；
     它只允许出现在 mode_navigation（真机导航）中，且启动前必须过 P0.5 运动链路检查。
     nx2app（joy 桥）、sensor_checker 同理，一期不启动。 -->
```

2. **M1 话题契约**（V5）：C16 驱动发布 `rslidar_points`（相对名，launch 起在根命名空间即 `/rslidar_points`）；faster-lio 配置的 `lid_topic` 一律写 **`/rslidar_points`**。`/rslidar_points_filtered` 话题名从沙箱词汇表中删除（ysc 侧该话题的真实生产者待查，见 Q7，不影响本文）。

3. **（v4，N6）cmd_vel 纵深管道显式化**：
   `controller(M4) ──► /cmd_vel_raw ──► safe_cmd_vel_mux(M6) ──► /cmd_vel ──► ros2qnx（仅真机 mode_navigation）`。
   设计规约与两条红线见 **§2.6**；collision_guard（多边形安全区急停，管道第五环）登记二期 §16。

### 2.6 ★ 导航栈设计规约（v4 新增，N2）

> 设计渊源：ROS 2 navigation2 联邦式架构对标分析的**八个已验证设计决策**，逐一映射到 ROS 1 沙箱落地物；本节是 M4/M5/SUP 的**红线级规约**，自检脚本（§11.2）与 Gate（§8/§10）承接执行。

| # | Nav2 设计决策（已验证） | ROS 1 落地物（本沙箱） | 计划书落点 |
|---|---|---|---|
| 1 | policy / mechanism 分离 | lite3_navigation 壳包只含 launch/config/scripts（policy 层）；算法 = 系统插件 teb / global_planner / mbf（mechanism 层） | §2.6 / §8 C0 / 自检 [9] |
| 2 | 一切皆 action（可抢占、可取消） | mbf GetPath / ExePath / Recovery 独立 actionlib 服务器 + M5 原语封装（C12） | §8 C11/C12 |
| 3 | 窄插件契约 | nav_core / mbf_costmap_core 接口收敛；壳包零算法源码 | §2.6 / 自检 [9] |
| 4 | 插件单步化、服务器持环 | mbf 服务器定频驱动 teb（`controller_frequency`，机制内置） | 无需施工（说明行） |
| 5 | 运行期按名点单 | goal 内 `planner` / `controller` / `behavior` 字符串字段（P3a.0 实测） | §8 C11/C12 |
| 6 | 恢复一等公民 + 诊断式恢复 | recovery_policy.yaml 错误码→处置分级映射（policy 资产，C11） | §8 C11/C5 |
| 7 | lifecycle + bond | SUP 有序启动 + 1Hz 心跳 + 失联降级（ROS 1 模拟，C13） | §8 C13 / §12 |
| 8 | cmd_vel 纵深防御管道 | cmd_vel_raw → mux → cmd_vel → ros2qnx（仅真机）；collision_guard 二期 | §8 C7/C8 / §16 |

**硬规则（红线级，违反 = Gate 不过）**：**a) `/cmd_vel` 唯一写者 = M6 `safe_cmd_vel_mux`**——SUP（及任何其他模块）只能"通知 M6 零速"（`/lite3/supervisor/hold`，C8/C13），**绝不直接写 /cmd_vel**，自检 [6] 承接（lite3_supervisor 在扫描范围内）；
**b) 壳包禁算法源码**——`lite3_*` 壳包（尤其 lite3_navigation）内禁出现第三方算法 `.cpp/.cc/.h`，自检 [9] 承接（40_navigation 下期望 0 命中）；A9 例外流程不变：二期改 teb 源码走"复制 teb 源码 → 编译 → devel 遮蔽系统版 → diff 登记"（§16）。

---

## §3 目录结构

完整目录树继承 v2 §3.1，v3 修订四处 + v4 修订一处：

1. **笔记目录定名为 `note/`**（用户指定；v2 写的是 `notes/`，全文已统一）。
2. **`40_navigation/` 下不再出现 `teb_local_planner`、`move_base`、`navigation` 三个目录**（A9）——只有自研壳 `lite3_navigation/`。
3. `10_slam/map_server` 目录**删除**（系统已装；`rosrun map_server map_saver` 直接可用）。
4. 新增 **`opt/ref/` → 自研壳的"配置种子"语义**（见 C0）。
5. **（v4，N3/N4/N5）壳包职责注记**——新增配置文件清单落位：`lite3_navigation/config/` 增 `mbf_navigation.yaml`（C0）与 `recovery_policy.yaml`（C11）；`lite3_task/config/` 增 `task_primitives.yaml`（C12）+ `scripts/task_smach_min.py`；`lite3_supervisor/config/` 增 `health_watch.yaml`（C13）+ `scripts/health_monitor.py`。M3（lite3_perception）一期仍占位不变。

### 3.2 双目录制 v3 修订条款

> **第三方包"原样复制、一个字不改"原则增加唯一例外（V2）**：
> `faster_lio/src/laser_mapping.cc:751` 的存盘路径硬编码写穿隔离边界，**必须**在复制后立即改写为
> `/home/ros1-slam/data/maps/`。这是全文唯一许可的第三方包源码改动，必须登记进
> `note/04_配置改动diff.md`（旧值/新值/理由/日期），且自检 [8] 会把它纳入永久监控。
> 判断标准写死：**凡影响隔离边界（读写 `/home/ysc`）的第三方源码行为，隔离规则优先于"一字不改"。**
> v4 注记（§2.6 硬规则 b）：本条款管第三方包；自研壳方向的反向约束（壳包内禁算法源码）由自检 [9] 承接。

### 3.3 属主与权限（v3 时序修正）

权限模型表继承 v2（`/home/ros1-slam` 750、目录 755、文件 644、脚本 755、`opt/ref/` 555）。**v3 修正执行时序与前置（V9/V11）**：
⓪ **P0.6（复制之前）**`sudo setfacl -bR /home/ros1-slam`（清掉 2026-10-04 设的 ysc ACL，避免与后续 chmod 的 mask 冲突；此后一切访问走 `sudo -u` / `sudo cp`）→ ① **P2 复制完成后**`sudo chown -R ros1-slam:ros1-slam /home/ros1-slam` → ② 紧随 ① 按权限模型 chmod（v2 原 ② ③ ④ 不变）→ ⑤（可选，P2 末）重建 ysc 便利通道：`sudo setfacl -Rm u:ysc:rX /home/ros1-slam && sudo setfacl -Rdm u:ysc:rwx /home/ros1-slam`（**必须在 ② 之后**；rX 不破坏脚本 755，与 v2 的 mask 顾虑相容）。

---

## §4 阶段总览与依赖（v4 修订 DAG：P0–P2 已执行冻结；P3a.0 ✅ 已完成；S9 为新前置）

```
Phase 0  环境准备（登录/组/ACL清理/磁盘/施工窗口）[需 root] ── Gate G0：可用 + ACL 已清              ✅ PASS 2026-10-04
   ▼
Phase 1  目录初始化 + catkin 初始化 ── Gate G1：骨架齐全                                            ✅ PASS
   ├─ P1.5 ★ 自研壳脚手架生成（lite3_* 包骨架）[v3 新增 V7] ── Gate G1.5：package.xml 就位          ✅ PASS
   ▼
Phase 2  外部文件复制 + faster-lio 存盘路径改写 + 路径扫描替换 ── Gate G2：v3_isolation.sh 退出码 0  ✅ PASS
   ├─ P2 末：统一 chown + chmod（v3 时序 V9）
   ▼
P3a.0 ★ 导航栈组件只读实测（v4 新增节点，2026-10-04 完成；产出 /tmp/P3a0_mbf组件实测.log）           ✅
   ▼
[S9] ★ nav_core_adapter 安装（用户 sudo，Q9；不批 → §8 回退决策表切 move_base 形态）                 [v4 新增，未执行]
   ▼
Phase 3a ★ 配置重构（C0–C10 + v4 新增 C11–C13；含 mbf_navigation.yaml 转换稿）[纯写文件，不依赖编译]
   │  Gate G3a：rosparam 可加载 + 三列表非空 + type 串比对 + diff 文档齐全 [v4 增强 N8]
   ▼
Phase 4  依赖核对（实测已满足，10 分钟；v4：含 S9 后 nav_core_adapter 命中核对）
   ▼
Phase 5  编译构建（v3 五批 → 实为四批 V12；先加临时 swap V14）── Gate G4：devel/setup.bash 存在 + ROS_PACKAGE_PATH 无 ysc + teb/mbf 系统版核验 [v4 增强 N8]
   ▼
Phase 3b ★ 现场采图（原 C1 执行部分，依赖 P5 编译产物）[v3 时序修正 V6] ── Gate G3：data/maps/ 三件齐
   ▼
Phase 6  分层验证（Stub/Sink 逐模块；G5–G9）
   │  ★ 前置（v4 N7/Q8 硬停止线）：重跑 P0.5 双查 + 与用户确认处置；G5/G6 判据增强（N8）
   ▼
Phase 7  场景组合启动（modes/）── ★ 前置（N7/Q8 硬停止线）：同上
   ▼
Phase 8  回滚（任意阶段可执行）
```

---

## §5 Phase 0：环境准备

> ✅ **冻结注记（N9）**：本章已按 v3 执行完毕（2026-10-04，Gate **G0 PASS**，见 `note/进度/00_总览.md`）。正文冻结不改写；对后续阶段生效的仅有 **①P0.5 正则扩充（N7，P6/P7 重查适用）②S9 新增（P0.4）③P0.7 坑4 施工规约注记**。

### P0.1 登录可用（同 v2：A 设密码 + B 分发公钥，推荐都做）

### P0.2 补充用户组（同 v2：`plugdev,video` 必、`i2c,gpio` 选、**sudo 不加**）
实测 `gpio/i2c/plugdev/video` 四个组都存在，一条 `usermod` 即可。

### P0.3 系统依赖清单（同 v2，全部 ✅；CUDA 11.4 < 11.7 硬阻塞 PCT 不变）

### P0.4 需要 sudo 的操作清单（继承 v2 S1–S6，另加；v4 增 S9）

| # | 操作 | Phase |
|---|---|---|
| S7 | `setfacl -bR`（V11）+ P2 末 chown/chmod | P0/P2 |
| S8 | 临时 swap 增删（V14） | P5 前/后 |
| **S9** | **`sudo apt install ros-noetic-nav-core-adapter`（0.3.0，官方源 packages.ros.org；仅装系统包，不进 ws、不改第三方源码）** | **P3a 前（Q9：用户批准并自行执行；不批 → §8 回退决策表 move_base 形态）** |

### P0.5 施工窗口确认（v3 扩展 V1；v4 正则再扩充 N7 —— 对 P6/P7 重查生效）

```bash
ps aux | grep -E "lslidar|faster_lio|hdl_|roscore|rosmaster" | grep -v grep   # 传感器侧，期望空
ps aux | grep -E "ros2qnx|message_transformer|lite3_ros_bridge|jetson2motion|jetson2app|transfer_launch" | grep -v grep  # ★ 运动链路侧（v4 扩充 N7），期望空
```

> 第二条是 v3 新增：若 ysc 侧 monitor/桥接栈在跑，`/cmd_vel` 链路活跃，stub 模式的任何手滑 pub
> 都可能真驱动机器狗。**真机 mode（含 ros2qnx）启动前必须重查本条。**
> **v4 注记（N7，吸收坑1）**：v3 原运动链路正则漏掉 root 侧 foxy 栈三进程（`transfer_launch`/`jetson2app`/`jetson2motion`——jetson2motion 订阅 /cmd_vel 直发底盘）与 ysc monitor 桥 `lite3_ros_bridge`，实机 ps 实测 4 行长期在跑（P0 执行时已按扩充正则重查登记，见 `note/01_部署记录.md` P0.5 节与 `02_问题与坑.md` 坑1）。**两侧合并等价正则**：`lslidar|faster_lio|hdl_|roscore|rosmaster|ros2qnx|message_transformer|lite3_ros_bridge|jetson2motion|jetson2app|transfer_launch`。P0 执行事实：判定 = 登记 Q8 后继续，P0–P2 零节点启动。
> **Q8 硬停止线（N7，正文化）**：root 侧 foxy 栈与 ysc monitor 桥在跑期间，**P6/P7 沙箱起节点（stub）前必须重跑本节双查并与用户确认处置**（§11/§12 前置、§15 Q8）。

### P0.6 ★ 清理 ACL（v3 新增，V11）

```bash
sudo setfacl -bR /home/ros1-slam
getfacl /home/ros1-slam | grep -c "user:ysc"   # 期望 0
```

### P0.7 sudo 会话技巧

每阶段开一个持权 shell（`sudo -v` 刷新 15 分钟票据；或 `sudo -i` 常驻），避免几十条 `sudo -u` 命令反复输密码。

> **坑4 执行注记（v4 收编，对 P3a 及以后生效）**：①sudo 时间戳按会话记账，**跨 Bash 会话必须重新刷新凭据**（每施工块开头 `printf '%s\n' "$PW" | sudo -S -p '' -v`）；②**证据采集用"重定向到文件再 cat"**，不在管道子 shell 内裸 sudo（P1 实测报「a terminal is required」；P3a.0 证据暂存 /tmp 亦因此，见 §8.0）。

---

## §6 Phase 1：目录初始化

> ✅ **冻结注记（N9）**：本章已按 v3 执行完毕（2026-10-04，Gate **G1 / G1.5 双 PASS**，见 `note/进度/00_总览.md`）。
> 正文冻结；`gen_shell_pkgs.sh` 全文见 v3 §P1.5（已执行，产物即当前沙箱 9 个 lite3_* 壳，此处不再重复代码块）。

P1.1 建骨架 / P1.2 `catkin_init_workspace` 同 v2（目录树按 §3 修订：无 teb/move_base/map_server 子目录，笔记目录为 `note/`）。

### P1.5 ★ 自研壳脚手架生成（v3 新增，V7）

生成 9 个 `lite3_*` 壳包骨架（package.xml + CMakeLists + launch/stub/sink/config/scripts/verify + README），
包位清单（已执行，与沙箱现状一致）：

```bash
ROOT=/home/ros1-slam/ws/src
# 00_drivers/lite3_drivers=M0  10_slam/lite3_slam=M1  20_localization/lite3_localization=M2  30_perception/lite3_perception=M3
# 40_navigation/lite3_navigation=M4  50_task/lite3_task=M5  60_safety/lite3_safety=M6  70_bringup/lite3_bringup=TOP  90_supervisor/lite3_supervisor=SUP
```

**Gate G1.5**：`find /home/ros1-slam/ws/src -name package.xml | grep -c lite3_` → **9**（至少 8）→ **PASS（2026-10-04）**。
M5（lite3_task）同样生成，虽为一期占位，但空壳可编译，省去白名单特判。

> v4 注记：lite3_task / lite3_supervisor 两壳在 v4 中正式赋职责（C12/C13，§8）；骨架不变。

### P1.6 `.bashrc` 追加（v3 落实，V10 关联）

```bash
sudo -u ros1-slam tee -a /home/ros1-slam/.bashrc <<'EOF'

# ==== ros1-slam sandbox env ====
source /home/ros1-slam/scripts/env.sh 2>/dev/null || true
EOF
```

（`env.sh` 在 P6.1 才写全，此处先容错；自检 [3] 负责把关。）

### P1.3 `as_ros1slam.sh` / P1.4 `note/00_README.md` 三条硬规则：同 v2。

---

## §7 Phase 2：外部文件复制与路径改写

> ✅ **冻结注记（N9）**：本章已按 v3 执行完毕（2026-10-04，Gate **G2 PASS**，见 `note/进度/00_总览.md`）。正文冻结不改写；**坑2 执行注记**：faster-lio 存盘路径硬编码实测**两处**（`laser_mapping.cc:751 与 862`），§7.2a 原文只写 751，实际执行以 sed 全局替换一次覆盖两处（文件内 /home/ysc 计数=0，`04_配置改动diff.md` 第 1 条两处行号并载，G2 证据独立抽验两行均为改写后路径）；后续若重做复制，按两处执行。

### 7.1 复制清单（v3 修订版）

| # | 来源 | 目标 | v3 变更 |
|---|---|---|---|
| 1–2 | `…/drivers/leishen_ws/src/lslidar_driver` + `lslidar_msgs` | `ws/src/00_drivers/`（同名子目录） | 不变 |
| 3 | `…/transfer/src/message_transformer` | `ws/src/00_drivers/message_transformer` | 不变（★ M0 必需） |
| 4 | `…/slam/src/faster-lio` | `ws/src/10_slam/faster_lio` | 不变（排除 .git）；**复制后立即执行 §7.2a 存盘路径改写** |
| 5 | `…/slam/src/pcd2grid` | `ws/src/10_slam/pcd2grid` | 不变 |
| ~~6~~ | ~~`…/slam/src/map_server`~~ | ~~删除~~ | **🆕 不复制**：系统已装（A9） |
| 7–10 | `…/nav/src/hdl_localization` + `hdl_global_localization` + `ndt_omp` + `fast_gicp` | `ws/src/20_localization/…` | 不变 |
| ~~11~~ | ~~`…/nav/src/teb_local_planner`~~ | ~~删除~~ | **🆕 不复制**：系统已装 0.9.1，与源码版本一致（V12） |
| ~~12~~ | ~~`…/nav/src/move_base`~~ | ~~删除~~ | **🆕 不复制**：系统已装 |
| 13 | `…/nav/src/navigation/config/` | `opt/ref/navigation_params_orig/` | 不变（只读参照；navigation 包**任何部分都不进 ws**） |
| 14 | `…/voa/src/deeprobotics_local_height_map`+`libraries` | `ws/src/30_perception/` | 不变 |
| 15 | `…/pipeline/src/pipeline_tracking` | `ws/src/50_task/pipeline_tracking` | 不变；C4 排除 `scripts/dyn_reconfigure.py`；**`location_record.py` 在 `tools/` 子目录**（v2 误写 scripts/） |
| 16 | `/home/ysc/faster_lio_online.yaml` | `ws/src/10_slam/lite3_slam/config/faster_lio_outdoor.yaml` | **🆕 ROS2→ROS1 转换后才落位**（转换稿见 §C1，原文件留档 `opt/ref/faster_lio_ros2_orig.yaml`） |
| 17 | `/home/ysc/c16__lslidar_driver_node.yaml` | `opt/ref/lidar_c16_orig.yaml` | 不变（ROS2 格式只作参照） |
| ~~18~~ | `realsense2_camera` | — | 🆕 明确**二期才复制**，一期清单中删除 |

### 7.2 复制命令：同 v2（`copy_pkg` tar 管道，排除 `.git/build/devel`；**永不删 `/home/ysc/lite_cog/`**）。

### 7.2a ★ faster-lio 存盘路径改写（v3 新增，V2，复制后立即执行；坑2：实测两处 751/862）

```bash
F=/home/ros1-slam/ws/src/10_slam/faster_lio/src/laser_mapping.cc
sudo grep -n '/home/ysc/lite_cog/system/map' "$F"     # 实测命中两处：751 与 862（坑2）
sudo sed -i 's#/home/ysc/lite_cog/system/map/lite3#/home/ros1-slam/data/maps/lite3#g' "$F"
sudo grep -n '/home/ros1-slam/data/maps' "$F"         # 复核改写成功（两处）
echo "- faster_lio/src/laser_mapping.cc:751/862 存盘路径 /home/ysc/... → /home/ros1-slam/data/maps/（隔离改写，唯一许可的第三方源码改动）$(date +%F)" \
  | sudo tee -a /home/ros1-slam/note/04_配置改动diff.md
```

> v4 注记：命令保持 v3 的 sed 全局替换形态（天然覆盖两处）；执行事实见章首坑2 注记。

### 7.3 绝不复制 build/devel/.git：同 v2（R1/R2 机制与翻车案例保留）。

### 7.4 路径改写（v3 白名单扩充，V15）

```bash
cd /home/ros1-slam
# 扫描留档（白名单 *.launch/*.yaml/*.xml/*.sh/*.py/*.rviz/*.cfg/CMakeLists.txt/*.cpp/*.h/*.cc）
grep -rn "/home/ysc" --include="*.launch" --include="*.yaml" --include="*.xml" \
     --include="*.sh" --include="*.py" --include="*.rviz" --include="*.cfg" \
     --include="CMakeLists.txt" --include="*.cpp" --include="*.h" --include="*.cc" \
     ws/src 2>/dev/null | tee note/02_问题与坑.md.scan
# 替换（仅 ws/src；notes 与 opt/ref 保持原样以存来源）
grep -rl "/home/ysc" --include="*.launch" --include="*.yaml" --include="*.sh" \
     --include="*.py" --include="*.rviz" --include="*.cc" --include="*.cpp" \
     ws/src 2>/dev/null | xargs -r sudo sed -i \
  's#/home/ysc/lite_cog/system/map#/home/ros1-slam/data/maps#g; s#/home/ysc#/home/ros1-slam#g'
grep -rn "/home/ysc" ws/src 2>/dev/null | wc -l   # 复扫：期望 0（laser_mapping.cc 已在 7.2a 处理）
```

> v3 注：`--include="*.cc"` 是必要的——faster-lio 源文件是 `.cc` 后缀，v2 白名单漏了它，而 751 行这类硬编码恰在 `.cc` 里。
> v4 注记：P2 实际执行"按 map→data/maps 先行 + 全量替换"两步（快照 `02_问题与坑.md.scan`：替换前命中 4 行/3 文件，复扫=0），与上式语义一致。

### 7.5 C16 参数（v3 无变更，保留）：以 `lslidar_c16.launch` 默认值为准（msop 2368 / difop **2369** / frame_id `rslidar` / max_range 150 / min_range 0.30），`c16__lslidar_driver_node.yaml`（ROS2 格式、difop 56401）仅留档不用。

---

## §8 Phase 3a：配置重构（纯写文件，编译前完成）—— v4 重写章节（mbf 主路线）

> 本章为 v4 改动最大章节：8.0 收编 P3a.0 实测；C0 重写为 mbf 形态；C1–C4/C6/C9/C10 不变；C5 升级（N3）；C7/C8 补管道规约（N6）；新增 C11/C12/C13（N3/N4/N5）与回退决策表（N1）；G3a 增强（N8）。前置：S9（Q9 批准后安装；不批 → 本章末回退决策表）。

### 8.0 ★ P3a.0 导航栈组件只读实测（✅ 2026-10-04 完成，收编 N1）

执行者：寇豆码，只读实测（未起节点、未 roscore、未 apt install、未写 ws/opt、未触碰 `/home/ysc/lite_cog/`）。原始证据：`note/进度/计划书v4升级/证据/P3a0_mbf组件实测.log`（690 行；P3a.0 时因 sudo 受限先暂存 /tmp，**已按坑4 规约归档至证据目录**，/tmp 留同内容副本）。**本节及全文一切 mbf 字段名/参数名/插件 type 串以该 log 为唯一权威。**

**组件清单裁决表**：

| 组件 | 实测结果 | 处置 |
|---|---|---|
| mbf_msgs / mbf_abstract_nav / mbf_costmap_nav | **0.4.0** ✅（rospack + dpkg 双证；服务器本体 `/opt/ros/noetic/lib/mbf_costmap_nav/mbf_costmap_nav`） | **N1 主路线成立** |
| move_base_flex / executive_smach（metapackage） | dpkg 均已装（0.4.0 / 2.5.2）；`rospack find` 拒识（package.xml 在但无 build_type，异常原样记录） | 不影响：服务器在 mbf_costmap_nav，metapackage 仅依赖聚合 |
| nav_core_adapter | ❌ 未装；apt 候选 **0.3.0-2focal.20250521.021323**（官方源） | **S9（Q9 用户 sudo）** |
| smach / smach_ros | **2.5.2** ✅ import + StateMachine 实例化 OK（P3a.0 python3 实测） | C12 一期成立（**Q10 关闭**） |
| teb_local_planner | 0.9.1，**双注册**（plugin xml 同时声明 `nav_core::BaseLocalPlanner` 与 `mbf_costmap_core::CostmapController`） | mbf 原生可载，**无需适配** |
| move_base | 1.17.3 ✅ | 回退形态对照（回退决策表） |

**关键实测事实（C0/C11/C12 的依据）**：① mbf_costmap_nav **不带样例 yaml/launch**（find 空）→ 配置全自研；② 插件列表参数名 = **`planners`/`controllers`/`recovery_behaviors`**，格式 `[{name, type}]` 元组列表，默认取各列表第一个（`strings libmbf_abstract_server.so` 原文）；③ 改名参数（`move_base_legacy_relay.py:93/102/104`）：`max_planning_retries→planner_max_retries`、`max_controller_retries→controller_max_retries`、`recovery_behavior_enabled→recovery_enabled`；④ 同名保留：`controller_frequency`/`planner_frequency`/`oscillation_timeout`/`oscillation_distance`（patience 两项原样透传）；⑤ Goal 按名点单字段：GetPathGoal.`planner`、ExePathGoal.`controller`、RecoveryGoal.`behavior`、MoveBaseGoal 另有 `recovery_behaviors: string[]`；⑥ Result 均带 `uint32 outcome + string message`，常量表见 C11（15/19/10/11 档）；⑦ costmap 三底稿结构原样沿用，**挂载命名空间未实测（Q11）**；⑧ mbf_msgs 另有 CheckPath/CheckPose/CheckPoint 三个 srv（验证用）；⑨ ★**防踩坑**：action 消息是 **GetPathAction/ExePathAction 族**——`rosmsg show mbf_msgs/GetPath` 会报错，正确查询 `rosmsg show mbf_msgs/GetPathGoal`（Result/Feedback 同理），action 定义在 `action/GetPath.action`。

### C0 — ★ 配置种子（v4 重写为 mbf 形态，N1/N8）

v3 基线保留：P3a 第一步从 `opt/ref/` 拷入底稿再改。v4 修订：**六底稿 → 五件原样拷 + 一件（move_base_params）转换**。

```bash
REF=/home/ros1-slam/opt/ref/navigation_params_orig
NAV=/home/ros1-slam/ws/src/40_navigation/lite3_navigation/config
sudo mkdir -p "$NAV/variants"
sudo cp "$REF/common_costmap_params.yaml"     "$NAV/costmap_common.yaml"          # C3 目标（P3a.0 实测文件名）
sudo cp "$REF/global_costmap_params.yaml"     "$NAV/global_costmap_params.yaml"
sudo cp "$REF/local_costmap_params.yaml"      "$NAV/local_costmap_params.yaml"
sudo cp "$REF/global_planner_params.yaml"     "$NAV/global_planner_params.yaml"
sudo cp "$REF/teb_local_planner_params.yaml"  "$NAV/planner_teb.yaml"             # C2 目标
sudo cp "$REF/move_base_params.yaml"          "$NAV/move_base_params.yaml"        # 回退底稿：仅存档不加载（回退决策表）
```

**六底稿归宿与转换规则**（依据 P3a.0 log）：

| v3 C0 六底稿 | v4 归宿 | 转换规则 |
|---|---|---|
| `move_base_params.yaml` | **`mbf_navigation.yaml`（新写，全文见下）** | `base_global_planner`/`base_local_planner` 单字符串 → **planners/controllers 注册列表**（`[{name, type}]`，默认取第一个）；recovery_behaviors 注册表来自 C11；改名三项；同名项原样；`make_plan_clear_costmap`/`make_plan_add_unreachable_goal` 为 move_base 特有**不迁移**（Q11 确认 GlobalPlanner 参数等价物） |
| `teb_local_planner_params.yaml` | `planner_teb.yaml`（原样拷，C2 目标） | 顶层键 `TebLocalPlannerROS:` 不变；teb 参数在 mbf 下**同名生效**（同一插件二进制） |
| `common_costmap_params.yaml` | `costmap_common.yaml`（原样拷，C3 目标） | 结构沿用；挂载 ns 见 launch 注记（Q11） |
| `global_costmap_params.yaml` / `local_costmap_params.yaml` | 原样拷 | 结构沿用；挂载 ns（Q11） |
| `global_planner_params.yaml` | 原样拷 | 顶层键 `GlobalPlanner:` 不变 |

**`lite3_navigation/config/mbf_navigation.yaml` 完整草案**：

```yaml
# mbf_navigation.yaml —— M4 服务器主配置（mbf_costmap_nav 0.4.0 联邦式，v4 C0）
# 转换自：opt/ref/navigation_params_orig/move_base_params.yaml（mbf 不带样例 yaml/launch，实测 find 空 → 全自研）
# 字段权威：/tmp/P3a0_mbf组件实测.log（P3a.0，2026-10-04）——参数名禁止凭记忆增删
# ---- 插件注册表：[{name, type}] 元组列表（strings libmbf_abstract_server.so 实测；默认取各列表第一个，log 原文）----
planners:
  - name: "GlobalPlanner"
    type: "global_planner/GlobalPlanner"   # 串源=原版 move_base_params.yaml base_global_planner 实测值；
                                           # 该插件仅注册 nav_core::BaseGlobalPlanner（rospack plugins 实测）→ 加载须经 S9 nav_core_adapter（Q9）
controllers:
  - name: "TebLocalPlannerROS"
    type: "teb_local_planner/TebLocalPlannerROS"   # 串源=teb_local_planner_plugin.xml 双注册实测（mbf_costmap_core::CostmapController）→ mbf 原生可载
recovery_behaviors:    # 注册表来自 C11 recovery_policy.yaml；type 串与可加载性见 Q11（P3a.0 log 无 recovery 插件描述符）
  - name: "clear_costmaps"
    type: "clear_costmap_recovery/ClearCostmapRecovery"
    # ★ Q11 联调首查：log 仅含包名 clear_costmap_recovery 与 ccr_plugin.xml 路径，未含 class 串；
    #   且该插件只注册 nav_core 命名空间，mbf 侧加载路径（是否同样需 adapter）未实测。
    #   兜底：加载失败则本列表退空 + recovery_enabled 保持 false，恢复全走 M5 层 C11 分级（G3a 该子项随 Q11 决议修订）。
# ---- 改名参数（move_base_legacy_relay.py:93/102/104 实测映射）----
planner_max_retries: -1       # ← 底稿 max_planning_retries: -1（六底稿中唯一在稿的改名项）
recovery_enabled: false       # ← 底稿未含（mbf 默认开）。★ v4 设计裁决：mbf 内置自动恢复链关闭，恢复编排收口 M5（C11 分级映射），
                              #   避免双重恢复；注册表保留供 M5 显式按名调用 Recovery 原语（C12 recover）。
# controller_max_retries（← max_controller_retries）：底稿未含该参数，mbf 默认值生效，不虚设。
# ---- 同名保留参数（底稿原值原样迁移，P3a.0 实测同名）----
planner_frequency: 0.0
controller_frequency: 10.0
planner_patience: 3.0
controller_patience: 3.0
oscillation_timeout: 0.0
oscillation_distance: 0.1
# ---- move_base 特有参数 make_plan_clear_costmap / make_plan_add_unreachable_goal 不迁移（GlobalPlanner 等价物 Q11）----
# ---- costmap 三底稿结构原样沿用，launch 按命名空间分别加载；挂载命名空间未实测（Q11）：P6 下 rosparam get
#      复核 ~global_costmap / ~local_costmap 后回填固化；底稿子键以 opt/ref 实际为准 ----
```

**launch 主线草案**（`lite3_navigation/launch/40_navigation.launch`；rviz 默认关 = C10）：

```xml
<launch>
  <arg name="rviz" default="false"/>
  <!-- M4 主线：mbf_costmap_nav 联邦式服务器（v4 N1）。rosparam 在 node 标签内 → 参数落私有命名空间；
       各文件挂载 ns 为 v4 设计稿，P3a.0 未实测 → Q11（P6 下 rosparam get 复核后固化/修正）。 -->
  <node pkg="mbf_costmap_nav" type="mbf_costmap_nav" name="move_base_flex" output="screen">
    <rosparam command="load" file="$(find lite3_navigation)/config/mbf_navigation.yaml"/>
    <rosparam command="load" file="$(find lite3_navigation)/config/planner_teb.yaml"/>
    <rosparam command="load" file="$(find lite3_navigation)/config/global_planner_params.yaml"/>
    <rosparam command="load" file="$(find lite3_navigation)/config/costmap_common.yaml"        ns="global_costmap"/>
    <rosparam command="load" file="$(find lite3_navigation)/config/global_costmap_params.yaml" ns="global_costmap"/>
    <rosparam command="load" file="$(find lite3_navigation)/config/costmap_common.yaml"        ns="local_costmap"/>
    <rosparam command="load" file="$(find lite3_navigation)/config/local_costmap_params.yaml"  ns="local_costmap"/>
  </node>
  <!-- C7：/cmd_vel_raw relay（源话题名 Q11 联调确认后固化） -->
</launch>
```

### C1 — ★★ 补地图前置：faster-lio 配置转换（V2/V3/V4/V5 落地，19 号 W1）—— v4 不变

转换稿**全文以 v3 §C1 为准**（ROS2→ROS1 完整转换版，v3 原文一字不动保留于 `note/计划书v3/`，N10；落位 `/home/ros1-slam/ws/src/10_slam/lite3_slam/config/faster_lio_outdoor.yaml`，勿再"修正"）。四个关键裁决（不可改）：**V3** 源文件是 ROS2 格式，v3 已给出 ROS1 转换版（顶层直接是参数组）；**V5** `lid_topic: /rslidar_points`（原 `/rslidar_points_filtered` 全库无生产者；imu_topic `/imu/data` 由 qnx2ros 发布）；**V4** `lidar_type: 4`（VelodyneHandler 分支，`pointcloud_preprocess.cc:20`），绝不可"修正"为 2；**Q6** `pcd_save.interval: 10`（原值 0 时 `laser_mapping.cc:747` 条件恒假可能永不落盘，先按 10 处理）。

**launch 加载形态（不变）**：`lite3_slam/launch/10_slam_mapping.launch` 中 `<rosparam command="load" file="$(find lite3_slam)/config/faster_lio_outdoor.yaml"/>` 必须放在 `<node pkg="faster_lio" type="mapping_online" name="faster_lio_online"/>` **标签之外**——`run_mapping_online.cc:25` 用全局 NodeHandle 读全局命名空间（副作用：参数占全局命名空间，同一 roscore 只能跑一套 faster-lio，可接受）。**建图执行流程**：tmux 版脚本 `tmux_mapping.sh`（W4）挪到 Phase 3b（V6）执行，产物校验见 §10 P5.4。

### C2 — TEB 避障（W2）—— v4 不变，补 mbf 同名注记

对 `planner_teb.yaml`（C0 拷入的底稿）设
`include_costmap_obstacles: true`、`enable_homotopy_class_planning: false`、`max_vel_x: 0.4`、`planning_horizon: 4.0`、`dt_ref: 0.4`。
**v4 注记**：teb 参数在 mbf 下**同名生效**（同一插件二进制 0.9.1，C0 launch 经私有命名空间加载 `TebLocalPlannerROS:` 段，挂载位 Q11）。校验 G3b（`teb_markers` 含 `PolygonObstacles` ≥1）同 v2。

### C3 — STVL 三坑（W3）—— v4 不变，补 mbf 同名注记

`costmap_common.yaml`（底稿同上）改
`clearing: true`、`obstacle_range: 8.0`、`voxel_decay: 10.0`、`voxel_size: 0.10`；`min_z` 先 `tf_echo base_link rslidar` 体检再定（C16 几何依据同 v2：ring7 打 10 cm 台阶上限 8.6 m；Q3）。
**v4 注记**：costmap 参数名在 mbf 下同名生效（costmap 底稿结构原样沿用，挂载 ns Q11）。

### C4 — 不复制 `dyn_reconfigure.py`（W5）：复制后 `sudo rm -f …/pipeline_tracking/scripts/dyn_reconfigure.py` 并记录；运行期校验 `max_vel_x ≥ 0.4` 同 v2。

### C5 — `TaskTransfer.py` 限次重试（W6）—— v4 升级（N3）

v3 基线：改副本 `MAX_RETRY = 3`（盲目三次）。
**v4 升级**：副本改为读取 `lite3_navigation/config/recovery_policy.yaml`（C11），按错误码**处置分级**重试——`retry_local` 级按 `max_retries` 限次；`replan` / `clear_and_replan` 级升级处置；`abort_report` 级立即停止并上报。登记 `note/04_配置改动diff.md`（旧值/新值/理由/日期）。
（TaskTransfer.py 任务级失败语义与 mbf action outcome 的对接在 P6 mode_nav_stub 联调中验证。）

### C6 — 航点目录（W7）：同 v2；**修正**：录航点工具路径为 `…/pipeline_tracking/tools/location_record.py`（PyQt5，需 `ssh -X` 或现场 HDMI，退化方案手写 JSON 不变）。

### C7 — `/cmd_vel_raw` 改名落地（v3 基础 + v4 管道规约 N6）

v3 基线：topic_tools relay；**M4 输出绝不叫 /cmd_vel**。
**v4 管道规约（N6）**：`controller(M4) → /cmd_vel_raw → safe_cmd_vel_mux(M6) → /cmd_vel → ros2qnx（仅真机 mode_navigation）`。
★ mbf 形态下 relay **源话题 = mbf 服务器的速度输出话题**（P3a.0 未覆盖该话题名 → **Q11 联调确认后固化** launch 行）；确认前 C7 不落 launch 行，G3a 不校验本项。（回退形态下源 = move_base `~cmd_vel`，v3 原形态。）

### C8 — M6 自研 `safe_cmd_vel_mux.py`（v3 基础 + v4 N5/N6）

v3 基线：六源优先级、超时淘汰、**无源发零速**、恒 20 Hz；Lite3 底盘 250 ms 超时不能断。
**v3 运行链路事实保留**：M6 输出 `/cmd_vel` 的下游消费者是 `ros2qnx`（`ros2qnx.cpp:112`）——所以 **stub 模式下 `/cmd_vel` 只到 mux 为止**，绝不启动 ros2qnx（V1）。
**v4 增补**：① 管道规约（N6）：`/cmd_vel_raw → mux → /cmd_vel → ros2qnx（仅真机）`；② **监督保持输入（C13 配套）**：订阅 `/lite3/supervisor/hold`（std_msgs/Bool），true 时无视各源直接输出零速并保持直到 false——SUP 是该话题唯一发布者（红线：SUP 绝不直接写 /cmd_vel，§2.6 硬规则 a）；③ 登记口径：mux 速度源数量仍为六（hold 是控制输入，不是速度源）；增补项登记 diff。

### C9 — `variants/` 三档：同 v2（outdoor_open / crowd / night）。

### C10 — rviz 默认关闭：同 v2（实测 `local_rslidar_imu.launch:74` rviz `required="true"`，SSH 无 X 会拖死整个 launch——判断正确，保留）。

### C11 — ★★ 诊断式恢复策略（v4 新增，N3）

**文件**：`/home/ros1-slam/ws/src/40_navigation/lite3_navigation/config/recovery_policy.yaml`（policy 资产）——恢复原子动作注册 + 错误码→处置分级映射（**retry_local 局部重试 / replan 重规划 / clear_and_replan 清图重规划 / abort_report 中止上报**）。
**消费者**：lite3_task SMACH 状态机（C12）、TaskTransfer.py 副本（C5 升级版）；错误码常量**全部摘自 P3a.0 实测 log 的 outcome 常量表，禁止凭记忆增删**。

```yaml
# =============================================================================
# recovery_policy.yaml —— M4/M5 诊断式恢复策略（v4 新增 C11，N3）
# 错误码权威：/tmp/P3a0_mbf组件实测.log（P3a.0，2026-10-04）；outcome_tables 禁止手改
# =============================================================================
# ---- 恢复原子动作注册表（由 C12 原语组合实现；键名 = M5 处置分级命名空间，边界见下行澄清）----
# ★ 命名空间边界：本表键（retry_local 等）= M5 处置分级键（C11 策略命名空间），≠ mbf RecoveryGoal.behavior 点单值（后者仅限 C0 recovery_behaviors 注册表 name，当前仅 clear_costmaps；键→点单值是否/如何映射由 SMACH 原语实现层决定）
recovery_actions:
  retry_local:      {primitive: exe_path,       max_retries: 2, backoff_s: 1.0}   # 局部重试：同参数重发 ExePath
  replan:           {primitive: get_path,       max_retries: 2, backoff_s: 1.0}   # 重规划：GetPath → ExePath
  clear_and_replan: {primitive: clear_costmaps, then: [get_path, exe_path], max_retries: 1, backoff_s: 2.0}   # clear_costmaps 服务挂载点 Q11 联调确认（C12）
  abort_report:     {primitive: wait, notify: [M6, SUP]}   # 中止上报：停止发运动目标（零速由 M6 无源超时兜底，C8）+ 上报
  wait:             {primitive: wait}                      # 等待（退避载体）
# ---- 权威参考：outcome 常量表（P3a.0 log 原样摘录，禁止手改）----
outcome_tables:
  get_path:   {SUCCESS: 0, FAILURE: 50, CANCELED: 51, INVALID_START: 52, INVALID_GOAL: 53, NO_PATH_FOUND: 54, PAT_EXCEEDED: 55, EMPTY_PATH: 56, TF_ERROR: 57, NOT_INITIALIZED: 58, INVALID_PLUGIN: 59, INTERNAL_ERROR: 60, OUT_OF_MAP: 61, MAP_ERROR: 62, STOPPED: 63}
  exe_path:   {SUCCESS: 0, FAILURE: 100, CANCELED: 101, NO_VALID_CMD: 102, PAT_EXCEEDED: 103, COLLISION: 104, OSCILLATION: 105, ROBOT_STUCK: 106, MISSED_GOAL: 107, MISSED_PATH: 108, BLOCKED_PATH: 109, INVALID_PATH: 110, TF_ERROR: 111, NOT_INITIALIZED: 112, INVALID_PLUGIN: 113, INTERNAL_ERROR: 114, OUT_OF_MAP: 115, MAP_ERROR: 116, STOPPED: 117}
  recovery:   {SUCCESS: 0, FAILURE: 150, CANCELED: 151, PAT_EXCEEDED: 152, TF_ERROR: 153, NOT_INITIALIZED: 154, INVALID_PLUGIN: 155, INTERNAL_ERROR: 156, STOPPED: 157, IMPASSABLE: 158}
  move_base:  {SUCCESS: 0, FAILURE: 10, CANCELED: 11, COLLISION: 12, OSCILLATION: 13, START_BLOCKED: 14, GOAL_BLOCKED: 15, TF_ERROR: 16, INTERNAL_ERROR: 17, PLAN_FAILURE: 50, CTRL_FAILURE: 100}
# ---- 错误码 → 处置分级映射（键 = outcome_tables 常量名；SUCCESS=成功路径，不列即不走恢复）----
disposition:   # default 覆盖 FAILURE/NOT_INITIALIZED/INVALID_PLUGIN/INTERNAL_ERROR；move_base 表服务整链语义（navigate_to_pose / 回退形态共用）
  get_path:  {CANCELED: abort_report, INVALID_GOAL: abort_report, PAT_EXCEEDED: abort_report, OUT_OF_MAP: abort_report, STOPPED: abort_report,
              NO_PATH_FOUND: clear_and_replan, MAP_ERROR: clear_and_replan, EMPTY_PATH: replan, TF_ERROR: retry_local, default: replan}
  exe_path:  {CANCELED: abort_report, PAT_EXCEEDED: abort_report, INVALID_PATH: abort_report, OUT_OF_MAP: abort_report, STOPPED: abort_report,
              OSCILLATION: clear_and_replan, ROBOT_STUCK: clear_and_replan, MAP_ERROR: clear_and_replan,
              MISSED_GOAL: replan, MISSED_PATH: replan, BLOCKED_PATH: replan, NO_VALID_CMD: retry_local, COLLISION: retry_local, TF_ERROR: retry_local, default: replan}
  recovery:  {IMPASSABLE: abort_report, CANCELED: abort_report, STOPPED: abort_report, PAT_EXCEEDED: abort_report, TF_ERROR: retry_local, default: retry_local}
  move_base: {CANCELED: abort_report, COLLISION: clear_and_replan, OSCILLATION: clear_and_replan, START_BLOCKED: clear_and_replan,
              GOAL_BLOCKED: clear_and_replan, PLAN_FAILURE: clear_and_replan, TF_ERROR: retry_local, CTRL_FAILURE: retry_local, default: abort_report}
```

> 登记口径：C11 为新文件，登记 `04_配置改动diff.md`（新增行，理由 = N3）；C0 `recovery_behaviors` 列表来自本注册表（当前仅 clear_costmaps 占位条目，Q11）。

### C12 — ★★ 任务原语注册表（v4 新增，N4：契约先行 + SMACH 简版）

**文件**：`/home/ros1-slam/ws/src/50_task/lite3_task/config/task_primitives.yaml`（policy 资产，注册表 v1）。
**六原语**：navigate_to_pose / get_path / exe_path / recover / wait / clear_costmaps，每原语 = {action/service 目标， 类型， 关键 outcome 语义}；M5 编排层（一期 SMACH 2.5.2，二期 py_trees）**只依赖本注册表**，不直接耦合 mbf_msgs 细节，错误码语义引用 C11。

```yaml
# task_primitives.yaml —— M5 任务原语注册表 v1（v4 新增 C12，N4）
# 目标权威：action 类型与 Goal 字段名 = P3a.0 实测 log；action/service 话题名未实测 → Q11 回填
version: 1
primitives:
  navigate_to_pose:
    target: "action:mbf_msgs/MoveBaseAction"     # action 话题名随 mbf 服务器节点名，Q11 联调确认
    goal_fields: [target_pose, planner, controller, recovery_behaviors]   # MoveBaseGoal 实测字段；点单值取 C0 注册表 name
    outcome_semantics: "recovery_policy.yaml#outcome_tables.move_base"
  get_path:
    target: "action:mbf_msgs/GetPathAction"
    goal_fields: [use_start_pose, start_pose, target_pose, tolerance, planner, concurrency_slot]
    by_name_field: planner                       # 按名点单（§2.6 决策 5）
    outcome_semantics: "recovery_policy.yaml#outcome_tables.get_path"
  exe_path:
    target: "action:mbf_msgs/ExePathAction"
    goal_fields: [path, controller, concurrency_slot, tolerance_from_action, dist_tolerance, angle_tolerance]
    by_name_field: controller
    outcome_semantics: "recovery_policy.yaml#outcome_tables.exe_path"
  recover:
    target: "action:mbf_msgs/RecoveryAction"
    goal_fields: [behavior, concurrency_slot]
    by_name_field: behavior                      # behavior 点单值仅限 C0 recovery_behaviors 注册表 name（当前仅 clear_costmaps）
    outcome_semantics: "recovery_policy.yaml#outcome_tables.recovery"
    # ★ 命名空间边界：C11 recovery_actions 键（retry_local 等）为 M5 处置分级键，非点单值；处置键→behavior 点单值的映射由 SMACH 原语实现层决定（P6 接线按此实现，按 C11 键字面点单会被拒）
  wait:
    target: "local:timer"                        # 无 ROS 目标：本地定时器（params 默认 duration_s: 1.0）
    outcome_semantics: "succeeded/aborted 二态"
  clear_costmaps:
    target: "service:挂载点联调确认"              # ★ 实测未覆盖：mbf 侧 clear_costmaps 服务名/类型 → Q11（并入）；succeeded/aborted 二态
```

**一期 SMACH 最小验证状态机**（`/home/ros1-slam/ws/src/50_task/lite3_task/scripts/task_smach_min.py`）：
用系统 SMACH 2.5.2（P3a.0 实测可 import/实例化）写最小**顺序**状态机验证 C12 契约；transitions 键 = C11 处置分级；
**不接真实 action client**（话题名 Q11 回填后于 P6 接线）。

```python
#!/usr/bin/env python3
# lite3_task 一期最小顺序状态机（C12，N4）：验证 task_primitives.yaml 契约
import rospy, yaml
import smach

class PrimitiveState(smach.State):
    """读 task_primitives.yaml 原语契约；一期返回骨架 outcome，P6 换真实 action client（经 C11 disposition 映射）。"""
    def __init__(self, name, registry, outcomes):
        self.name, self.spec = name, registry['primitives'][name]
        smach.State.__init__(self, outcomes=outcomes)
    def execute(self, ud):
        rospy.loginfo('[M5] 原语 %s（target=%s）', self.name, self.spec['target'])
        return 'succeeded'

def main():
    rospy.init_node('lite3_task_smach_min')
    with open(rospy.get_param('~task_primitives')) as f:
        registry = yaml.safe_load(f)
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
    with sm:
        # transitions 键 = C11 处置分级：retry_local→retry / replan→replan / clear_and_replan→clear / abort_report→aborted
        smach.StateMachine.add('WAIT', PrimitiveState('wait', registry, ['succeeded', 'aborted']),
                               transitions={'succeeded': 'GET_PATH', 'aborted': 'aborted'})
        smach.StateMachine.add('GET_PATH', PrimitiveState('get_path', registry, ['succeeded', 'retry', 'aborted']),
                               transitions={'succeeded': 'EXE_PATH', 'retry': 'GET_PATH', 'aborted': 'aborted'})
        smach.StateMachine.add('EXE_PATH', PrimitiveState('exe_path', registry,
                                     ['succeeded', 'retry', 'replan', 'clear', 'aborted']),
                               transitions={'succeeded': 'succeeded', 'retry': 'EXE_PATH',
                                            'replan': 'GET_PATH', 'clear': 'GET_PATH', 'aborted': 'aborted'})
    rospy.loginfo('task outcome: %s', sm.execute())

if __name__ == '__main__':
    main()
```

> **py_trees 列二期**（外部依赖 R19：未装）；二期将同一注册表对接 py_trees/BT（§16）。

### C13 — ★ lite3_supervisor（SUP）职责定义（v4 新增，N5）

`90_supervisor/lite3_supervisor` 壳包 v3 从未赋职责。v4 定职责（Nav2 决策 7 lifecycle+bond 的 ROS 1 模拟）：

1. **按 mode 的有序启动清单**：SUP 在各 mode 顶层 launch 中**第一个启动**（§12 mode 表 SUP 列），按 health_watch.yaml 的 mode 组等待各模块 liveness 首跳后记 ready。2. **模块 1Hz 心跳监控**：各模块 launch 内置一行 rostopic 心跳（policy 层零代码），如 M4：`<node pkg="rostopic" type="rostopic" name="liveness_m4" args="pub -r 1 /lite3/liveness/m4_navigation std_msgs/Empty"/>`
3. **失联处置**：连续 `timeout_s` 无心跳 → ①发布 `/lite3/supervisor/hold=true`（M6 立即零速，C8）→ ②降级关停本 mode 模块（一期 scope：log fatal + 列出待停模块清单；实际关停接线在 P7 联调）。

**一期 scope（最小可用）**：`config/health_watch.yaml` + `scripts/health_monitor.py` 骨架，全文如下。

**红线（写死，入 §2.6 硬规则 a 与自检 [6] 语义）**：**SUP 绝不直接写 /cmd_vel**——`/cmd_vel` 唯一写者永远是 M6 `safe_cmd_vel_mux`；SUP 的全部输出仅有 `/lite3/supervisor/hold`（Bool）与日志。

```yaml
# health_watch.yaml —— SUP 一期 watch 清单（v4 新增 C13，N5）
# 位置：/home/ros1-slam/ws/src/90_supervisor/lite3_supervisor/config/health_watch.yaml
# 红线：SUP 绝不写 /cmd_vel（唯一写者 = M6 safe_cmd_vel_mux，§2.6-a / 自检 [6]）
defaults: {timeout_s: 3.0}    # 1 Hz 心跳，连续 3 s 失联即触发处置
modes:
  mode_mapping:               # 本 mode 无 M6 → 失联不涉及零速通知（on_loss: degraded_shutdown）
    watch: [{module: M0_drivers, topic: /lite3/liveness/m0_drivers}, {module: M1_slam, topic: /lite3/liveness/m1_slam}]
  mode_localize:
    watch: [{module: M0_drivers, topic: /lite3/liveness/m0_drivers}, {module: M2_localization, topic: /lite3/liveness/m2_localization}, {module: M6_safety, topic: /lite3/liveness/m6_safety}]
    on_loss: notify_m6_zero_speed_then_shutdown
  mode_nav_stub:
    watch: [{module: M0_drivers, topic: /lite3/liveness/m0_drivers}, {module: M4_navigation, topic: /lite3/liveness/m4_navigation}, {module: M6_safety, topic: /lite3/liveness/m6_safety}]
    on_loss: notify_m6_zero_speed_then_shutdown
  mode_navigation:            # 二期
    watch: [{module: M0_drivers, topic: /lite3/liveness/m0_drivers}, {module: M2_localization, topic: /lite3/liveness/m2_localization}, {module: M4_navigation, topic: /lite3/liveness/m4_navigation}, {module: M6_safety, topic: /lite3/liveness/m6_safety}]
    on_loss: notify_m6_zero_speed_then_shutdown
```

```python
#!/usr/bin/env python3
# /home/ros1-slam/ws/src/90_supervisor/lite3_supervisor/scripts/health_monitor.py
# SUP 一期最小监控骨架（C13，N5）。红线：本脚本绝不发布任何 cmd_vel 类话题（唯一输出 = hold）。
import rospy, yaml, threading
from std_msgs.msg import Bool, Empty

class HealthMonitor:
    def __init__(self):
        mode = rospy.get_param('~mode', 'mode_nav_stub')
        with open(rospy.get_param('~health_watch')) as f:
            cfg = yaml.safe_load(f)
        self.mode_cfg = cfg['modes'][mode]
        self.timeout = rospy.Duration(self.mode_cfg.get('timeout_s', cfg['defaults']['timeout_s']))
        self.last_seen, self.lock = {}, threading.Lock()
        self.hold_pub = rospy.Publisher('/lite3/supervisor/hold', Bool, queue_size=1, latch=True)  # 唯一输出，绝无 /cmd_vel
        for w in self.mode_cfg['watch']:
            self.last_seen[w['module']] = rospy.Time.now()
            rospy.Subscriber(w['topic'], Empty, self._beat, callback_args=w['module'])
        rospy.Timer(rospy.Duration(1.0), self._check)

    def _beat(self, _msg, module):
        with self.lock:
            self.last_seen[module] = rospy.Time.now()

    def _check(self, _event):
        now = rospy.Time.now()
        lost = [m for m, t in self.last_seen.items() if now - t > self.timeout]
        if lost:
            rospy.logfatal('[SUP] 心跳失联: %s —— hold=true（通知 M6 立即零速）；待降级关停 = 本 mode watch 全表', ','.join(lost))
            self.hold_pub.publish(Bool(data=True))

if __name__ == '__main__':
    rospy.init_node('lite3_supervisor')
    HealthMonitor()
    rospy.spin()
```

### 回退决策表（mbf 联邦式 → move_base 单体形态，v4 新增，N1）

**触发条件**：① Q9 用户不批 S9；② P6/P7 联调发现 mbf 阻塞（costmap 挂载 / 插件加载类问题短期不可解）——经主理人裁决切回。

| 项 | mbf 联邦式（主路线） | move_base 单体（回退形态） |
|---|---|---|
| C0 配置 | `mbf_navigation.yaml`（注册列表 + 改名参数） | C0 六底稿原样（`move_base_params.yaml` 直接用，已在 C0 存档）+ recovery_behaviors 参数清单（move_base 原生参数） |
| 全局规划器 | 经 S9 adapter 加载（global_planner/navfn 仅注册 nav_core） | move_base 原生加载（原生 nav_core，无 adapter） |
| 错误码 | 四 action 细粒度 outcome（15/19/10/11 档） | move_base actionlib **三态 outcome**（成功/抢占/中止） |
| C11 | 全量分级映射（outcome_tables 四表） | **降级为简化映射**：仅 retry_local（限次）+ abort_report 两级；replan/clear_and_replan 并入 retry_local 超限后人工介入 |
| C12 原语 | 六原语全量 | navigate_to_pose 照常（action 类型换回 move_base 标准 `move_base_msgs/MoveBaseAction`）；get_path/exe_path/recover 原语一期冻结（注册表保留，标注禁用） |
| C13 SUP | 不受影响 | **不受影响**（SUP 只依赖心跳与 hold，不感知导航形态） |
| C7 relay 源 | mbf 速度输出话题（Q11） | move_base `~cmd_vel`（v3 原形态） |

> 回退是**纯配置切换**（两套 yaml 均在 P3a 产出/存档，切启动 launch 即可），无施工回滚（§14）。

**Gate G3a（v3 基线 + v4 增强 N8）**：① `rosparam load` 上述 yaml 无报错（含 **`mbf_navigation.yaml`**）；
② `planners` / `controllers` / `recovery_behaviors` **三列表非空**（rosparam get 核对）；
③ **插件 type 字符串与 P3a.0 实测 log 中的插件描述符逐一比对一致**（planners/controllers 硬比对：teb 双注册描述符 + 原版 move_base_params 的 global_planner 串；recovery 条目 log 无描述符 → Q11 联调回填后补比对，不作为 G3a 硬判据）；
④ `note/04_配置改动diff.md` 覆盖 C2/C3/C4/C5/§7.2a + **C0（转换稿）/C5 升级/C11–C13 新增** 全部改动。

---

## §9 Phase 4：依赖核对

流程同 v2（先 source 再 import 的教训保留）。v3 结论：**R11 解除、Q2 关闭**（V13）——`message_transformer` 的 include 实测仅 `../include/protocol.h`、`../include/sensor_logger.h`、`message_transformer/ComplexCMD.h`、`message_transformer/SimpleCMD.h` 与标准 ROS 头；package.xml 依赖全为标准 catkin 包，批 5 编译它基本无悬念。

**v4 增补（N1/N8）**：若 Q9 已批且 S9 已装，P4 增一行核对——`rospack find nav_core_adapter` 应命中 **`/opt/ros/noetic/share/nav_core_adapter`**（dpkg ros-noetic-nav-core-adapter 0.3.0）；并补取证 `rospack plugins --attrib=plugin nav_core_adapter`（P3a.0 时未装，装后补录，供 C0 planners 加载路径佐证）。

## §10 Phase 5：编译构建

### P5.1 分批白名单（v3 修订，V12）

```bash
catkin_make -j3 -DCATKIN_WHITELIST_PACKAGES="lslidar_msgs;lslidar_driver"                                  # 批 1：M0 驱动
catkin_make -j3 -DCATKIN_WHITELIST_PACKAGES="faster_lio;pcd2grid"                                          # 批 2：M1 建图
catkin_make -j3 -DCATKIN_WHITELIST_PACKAGES="ndt_omp;fast_gicp;hdl_localization;hdl_global_localization"   # 批 3：M2 定位（最耗时：fast_gicp）
catkin_make -j3 -DCATKIN_WHITELIST_PACKAGES="message_transformer;lite3_drivers;lite3_slam;lite3_localization;lite3_perception;lite3_navigation;lite3_task;lite3_safety;lite3_bringup;lite3_supervisor"  # 批 4：全部自研壳 + message_transformer（v3：原批 4 teb/move_base 删除，壳包提前一次编完）
catkin_make -j3 -DCATKIN_WHITELIST_PACKAGES=""                                                             # 批 5：清空白名单，全量收尾
```

> v4 注记：mbf_costmap_nav / nav_core_adapter（S9 后）均为系统 apt 包，**零编译零复制**（A9/A11）。

### P5.0 临时 swap（v3 新增，V14，编译前执行、P5 完成后撤除）

```bash
sudo fallocate -l 4G /tmp/build_swap && sudo chmod 600 /tmp/build_swap   # P5 前
sudo mkswap /tmp/build_swap && sudo swapon /tmp/build_swap
# …… 编译 ……（OOM 处置：-j3 → -j2）
sudo swapoff /tmp/build_swap && sudo rm -f /tmp/build_swap               # P5 完成后撤除
```

OOM 处置同 v2（`-j3`→`-j2`）；§P5.3 常见失败表继承 v2，
其中"livox 头文件"一行按 V4 修订：**`lidar_type=4` 走 Velodyne 分支，与 v2 的"C16 用 2"说法相反**，但结论一致——都不需要 Livox。

### P5.2 Gate G4：同 v2（devel/setup.bash 存在、`rospack find` 三例、`ROS_PACKAGE_PATH` 无 ysc）。v3 追加：`rospack find teb_local_planner` 应指向 **`/opt/ros/noetic/share/teb_local_planner`**（证明在用系统版，A9 生效）。**v4 追加（N8）**：`rospack find mbf_costmap_nav` 应指向 **`/opt/ros/noetic/share/mbf_costmap_nav`**（证明 mbf 在用系统版而非 ws 复制件，同 teb 的 A9 校验）。

### P5.4 ★ PCD 产物回收（v3 新增，衔接 V2）

路径已在 §7.2a 改写为 `/home/ros1-slam/data/maps/lite3.pcd`，**现场采图（Phase 3b）结束后**：

```bash
ls -lh /home/ros1-slam/data/maps/lite3.pcd            # 期望 >0 且随建图增长
roslaunch lite3_slam 11_postprocess.launch             # pcd2grid: PCD → /map
rosrun map_server map_saver -f /home/ros1-slam/data/maps/lite3
ls -l /home/ros1-slam/data/maps/                       # Gate G3：.pcd/.yaml/.pgm 三件齐
```

---

## §11 Phase 6：分层验证

> ★ **v4 前置（N7/Q8 硬停止线）**：本章任何起节点（stub）步骤之前，必须**重跑 §5 P0.5 双查（扩充正则）并与用户确认处置**——root 侧 foxy 栈（transfer_launch/jetson2app/jetson2motion，jetson2motion 订阅 /cmd_vel 直发底盘）与 ysc monitor 桥 lite3_ros_bridge 长期在跑（Q8，§15）。

### 11.1 `scripts/env.sh`（v3 加守卫，V10）—— v4 不变

```bash
#!/usr/bin/env bash
export ROS_MASTER_URI=http://192.168.1.103:11312
export ROS_IP=192.168.1.103
source /opt/ros/noetic/setup.bash
# ★ v3 守卫：devel 不存在时明确报错而不是喷一屏 source 失败
[ -f /home/ros1-slam/ws/devel/setup.bash ] || { echo "[env.sh] ws/devel 不存在——先完成编译（P5）"; return 1 2>/dev/null || exit 1; }
source /home/ros1-slam/ws/devel/setup.bash
export ROS_HOME=/home/ros1-slam/.ros
export ROS_LOG_DIR=/home/ros1-slam/logs
export ROS_PACKAGE_PATH=$(echo "$ROS_PACKAGE_PATH" | tr ':' '\n' | grep -v "/home/ysc" | paste -sd:)
```

### 11.2 ★ 隔离自检 `scripts/verify/v3_isolation.sh`（v3 修复版 V10 + V2 盲区；v4 增补 [9]，N2）——文件名保持 `v3_isolation.sh` 不变（G2 证据链已引用该名）；[1]–[8] 语义与 v3 完全一致，P6 执行时整脚本按下面全文重新落盘（含 [9]）。

```bash
#!/usr/bin/env bash
FAIL=0
TMP=$(mktemp)
say() { echo "$1"; }
say "[1] ROS_PACKAGE_PATH 中是否含 /home/ysc"
echo "$ROS_PACKAGE_PATH" | tr ':' '\n' | grep -q "/home/ysc" && { echo "  违规"; FAIL=1; }
say "[2] 自研壳中是否含 /home/ysc"
grep -rln "/home/ysc" /home/ros1-slam/ws/src/*/lite3_* 2>/dev/null && FAIL=1
say "[3] .bashrc 是否 source 了其他 devel"
grep -qn "lite_cog\|/home/ysc" /home/ros1-slam/.bashrc 2>/dev/null && { echo "  违规"; FAIL=1; }
say "[4] rospack 解析路径是否都在 /home/ros1-slam（teb 例外：系统包）"
for p in faster_lio hdl_localization lite3_bringup lite3_safety; do
  rospack find "$p" 2>/dev/null | grep -q "/home/ros1-slam" || { echo "  BAD: $p"; FAIL=1; }
done
say "[5] ★ 模块间是否互相 include launch（20 号 K3）—— v3 修复子 shell 丢 FAIL bug"
: > "$TMP"
for L in /home/ros1-slam/ws/src/*/lite3_*/launch/*.launch; do
  [ -e "$L" ] || continue
  SELF=$(basename "$(dirname "$(dirname "$L")")")
  grep -o 'find lite3_[a-z_]*' "$L" 2>/dev/null | sed 's/find //;s/)//' | sort -u | \
  while read -r n; do [ "$n" != "$SELF" ] && echo "$L -> $n" >> "$TMP"; done
done
[ -s "$TMP" ] && { cat "$TMP"; FAIL=1; }
say "[6] ★ 是否有人绕过 M6 直写 /cmd_vel（20 号 K2）—— v3 修复管道计数 bug；v4 语义注记：lite3_supervisor 亦在扫描范围（SUP 红线：绝不写 /cmd_vel，N5/C13）"
grep -rn "cmd_vel" /home/ros1-slam/ws/src/*/lite3_*/launch/ 2>/dev/null \
  | grep -v "lite3_safety" | grep -v "cmd_vel_raw" > "$TMP" || true
[ -s "$TMP" ] && { cat "$TMP"; FAIL=1; }
say "[7] 磁盘余量 ≥ 15 GB（19 号反判据④）"
AVAIL=$(df -BG /home/ros1-slam | awk 'NR==2{gsub("G","",$4); print $4}')
[ "$AVAIL" -lt 15 ] && { echo "  仅剩 ${AVAIL}G"; FAIL=1; }
say "[8] ★★ 全 ws/src 扫描 /home/ysc 硬编码（v3 新增：覆盖第三方包，补 V2 盲区）"
grep -rln "/home/ysc" /home/ros1-slam/ws/src 2>/dev/null && FAIL=1
say "[9] ★★ 40_navigation 壳包禁算法源码（v4 新增，N2/§2.6 硬规则 b：policy 层零 .cpp/.cc/.h）"
HITS=$(find /home/ros1-slam/ws/src/40_navigation \( -name "*.cpp" -o -name "*.cc" -o -name "*.h" \) 2>/dev/null)
[ -n "$HITS" ] && { echo "  命中算法源码文件："; echo "$HITS"; FAIL=1; }
rm -f "$TMP"
[ $FAIL -eq 0 ] && echo "=== ISOLATION OK ===" || echo "=== ISOLATION FAILED ==="
exit $FAIL
```

### 11.3–11.5 验证矩阵、命令序列、验收判据

继承 v2，v3 修订四处（下表 1–4）；v4 增补（N7/N8，下表 5–7）：

1. **M0 拆分校验（V1）**：G5 判据追加一条——`rosnode list | grep ros2qnx` 期望**为空**（stub/mapping/localize 模式下出现即为事故）。
2. **M1 判据（V5/V2）**：`rostopic hz /Odometry` ≥10 Hz 的前置是 faster-lio 真的吃到点云——直接验 `rostopic hz /rslidar_points` ≥9 Hz（M0 已过 G5）再起 M1；PCD 落盘位置以 §P5.4 为准。
3. **M4 判据**：单位阵 TF + `teb_markers PolygonObstacles` 非空不变；**`rostopic info /cmd_vel` publisher 只能是 `safe_cmd_vel_mux`**。
4. A1–A5 / 20 号十条判据表整体保留。
5. **G5 判据追加（mbf action 族，N8）**：mode_nav_stub 下 `rostopic list` 可见 mbf 三 action 话题族（GetPath / ExePath / Recovery；精确话题名随 mbf 服务器节点名，Q11 联调回填后固化到判据表）。
6. **G6 判据追加（SUP 心跳，N8）**：SUP 心跳话题按 `health_watch.yaml` 当前 mode 清单**全部存活**（逐条 `rostopic list` 可见且 `rostopic hz` ≈ 1 Hz）。
7. **外部进程检查提示（N7）**：mode_nav_stub 下 ros2qnx 禁令保留（判据 1）之外，P0.5 扩充正则同时覆盖 `jetson2motion` 类 root 侧外部进程——起节点前双查命中即停下走 Q8 确认，不得"先起再说"。

## §12 Phase 7：场景组合（modes/）

> ★ **v4 前置（N7/Q8 硬停止线）**：与 §11 相同——P7 起任何 mode 前，重跑 P0.5 双查（扩充正则）并与用户确认处置；真机 mode（含 ros2qnx）启动前强制重查运动链路侧。

三层 launch 与铁律同 v2（20 号 K2/K3 承接：/cmd_vel 唯一写者 M6、模块间不互相 include）；**v4 铁律补一句（N5/C13）**：SUP 在每个 mode 顶层 launch 中第一个启动，且 SUP 仅有 `/lite3/supervisor/hold` 输出，绝不写 /cmd_vel。

mode 表 v3 修订 + v4 SUP 列：

| mode | 包含 | SUP 启动位与 watch 清单（v4） | v3/v4 变更 |
|---|---|---|---|
| `mode_mapping` | M0(驱动+**qnx2ros**) + M1 | SUP 最先起；watch = `health_watch.yaml#modes.mode_mapping`（M0/M1） | v3：明确 M0 仅 qnx2ros；v4：+SUP |
| `mode_localize` | M0(同上) + M2 + M6 | SUP 最先起；watch = `…mode_localize`（M0/M2/M6） | 同上 |
| `mode_nav_stub` ★ | M0(**仅驱动+qnx2ros**，不含 ros2qnx) + M4(**mbf 联邦式服务器**，stub TF) + M6 | SUP 最先起；watch = `…mode_nav_stub`（M0/M4/M6） | **V1：真机运动桥严禁入列（原样）**；v4：M4 升级 mbf（N1），G5/G6 增 mbf action 族 + SUP 心跳判据（N8） |
| `mode_navigation`（二期） | 真定位 + 真导航 + **此时才允许 ros2qnx** | SUP 最先起；watch 全链 | 启动前强制重跑 P0.5 运动链路检查（v4：扩充正则 + Q8 用户确认） |

## §13 跨目录风险表（v3 修订版 + v4 增补 N10）

R1–R10、R12 继承 v2；v3 修订与新增、v4 新增如下：

| # | 风险 | v3 变更 / v4 处置 |
|---|---|---|
| R11 | message_transformer 隐藏依赖 | **v3 降级为低**：实测 include 自包含（V13），保留"批 4 单独编"的处置 |
| **R13** 🆕v3 | **`ros2qnx` 写穿隔离 = 真机失控** | stub/mapping/replay mode 出现 ros2qnx 即事故。防线：§2 launch 禁入 + G5 校验 `rosnode list` + P0.5 双查 |
| **R14** 🆕v3 | **ACL 与 chmod 互相破坏** | P0.6 `setfacl -bR` 先清（V11）；P2 末如需 ysc 便利通道，ACL 在 chmod 之后重建 |
| **R15** 🆕v3 | **话题断链**（`/rslidar_points_filtered` 无生产者类问题） | 教训：复制来的 yaml 里的**每个 topic 参数**都要与上游 launch 的实际发布名对账（本文已对账：imu_topic ✅ / lid_topic 已修 ✅ / pointcloud_topic ✅） |
| **R16** 🆕v3 | **第三方包内硬编码路径**（`laser_mapping.cc:751` 类） | 复制后立即 §7.2a 改写 + 自检 [8] 永久监控；登记 diff |
| **R17** 🆕v4 | **mbf 参数/底稿格式差异**（move_base→mbf 迁移错漏） | 已实测给出转换表（§8 C0：注册列表格式、改名三项、make_plan_* 不迁移）；**残余风险 = costmap 挂载命名空间未实测（Q11）**——P6 mode_nav_stub 首日 `rosparam get` 复核后固化 |
| **R18** 🆕v4 | **SUP 误碰 /cmd_vel（零速通知变直控）** | 规约级防线：§2.6 硬规则 a（SUP 只能通知 M6 零速）+ 自检 [6]（lite3_supervisor 在扫描范围）；SUP 输出仅 `/lite3/supervisor/hold` |
| **R19** 🆕v4 | **py_trees 外部依赖（二期）** | 一期不用（SMACH 2.5.2 系统版实测可用，Q10 关闭）；py_trees 未装，二期评估装法与版本后再入计划（§16） |

## §14 回滚：同 v2。补充一行：faster-lio 源码改写若出问题 → 按 §7.2a 的 diff 记录逆向恢复（或整包重复制 + 重做改写，注意坑2：751 与 862 两处）。

**v4 补一行（N1）**：v4 配置回退 = 按 §8 回退决策表切 move_base 形态（**纯配置，无施工回滚**；C13 SUP 不受影响）。

## §15 工期与待确认项

| Phase | 内容 | v4 修订后预估（N10） |
|---|---|---|
| **P0–P2** | 用户/组/ACL/窗口 + 目录/脚手架/bashrc + 复制/改写/权限 | 15+20+40 min（✅ 已执行 2026-10-04） |
| **S9** | **nav_core_adapter 安装（用户 sudo）** | **5 min（Q9 批准后；不批 → §8 回退决策表）** |
| P3a | 配置重构（C0–C10 + **C11–C13**，含 P3a.0 收编与 mbf_navigation.yaml） | **2–2.5 h**（v3 估 1–2 h） |
| P4 | 依赖核对 | 10 min |
| P5 | 编译（四批，无 teb/move_base/map_server）+ swap | 1–2 h |
| P3b | 现场采图 + PCD 回收 | 1–2 h（要现场走一趟） |
| P6 | 分层验证（含 **SUP 联调 +0.5 h**、Q11 mbf 复核） | **3.5–5.5 h**（v3 估 3–5 h） |
| P7 | 组合启动 | 1 h |
| **合计** | | **约 2 个工作日**（不含采图往返；P0–P2 与 P3a.0 已完成不计入剩余工期） |

| # | 待确认 | 状态 |
|---|---|---|
| Q1 | ros1-slam 能否登录 | 开（P0.1 解决） |
| ~~Q2~~ | ~~message_transformer 隐藏依赖~~ | **✅ 关闭（V13：自包含）** |
| Q3 | C16 安装倾角（定 `min_z`） | 开（P3a 先验 10 min） |
| Q4 | `/Odometry` 话题名与频率（二期 SCAN 输入） | 开 |
| Q5 | `location_record.py`（`tools/` 下）图形环境 | 开（ssh -X / HDMI / 手写 JSON） |
| **Q6** 🆕 | `pcd_save.interval=0` 的落盘语义（走查显示 747 行条件恒假可能永不落盘） | 开；v3 已先按 `interval: 10` 处理，现场验证 |
| **Q7** 🆕 | ysc 侧 `/rslidar_points_filtered` 的真实生产者（可能是未复制的过滤节点，或该 yaml 从未在 ysc 侧生效） | 开；沙箱已按 `/rslidar_points` 裁决，不影响本文 |
| **Q8** 🆕 | **root 侧 foxy 栈（transfer_launch/jetson2app/jetson2motion，jetson2motion 订阅 /cmd_vel 直发底盘）+ ysc monitor 桥（lite3_ros_bridge）长期在跑**（坑3） | **开；硬停止线（N7）：P6/P7 起节点（stub）前重跑 P0.5 双查并与用户确认处置（§11/§12 前置）** |
| **Q9** 🆕 | **S9：`sudo apt install ros-noetic-nav-core-adapter`（0.3.0，官方源）由用户执行** | **开；不批 → §8 回退决策表 move_base 单体形态** |
| ~~**Q10**~~ | ~~smach 可用性~~ | **✅ 关闭（P3a.0 实测：smach/smach_ros 2.5.2 import + StateMachine 实例化 OK）** |
| **Q11** 🆕 | **mbf 联调确认包**：① costmap 三底稿挂载命名空间（~global_costmap/~local_costmap 复核）；② clear_costmaps 服务挂载点/类型；③ recovery_behaviors 注册表 type 串与 mbf 侧可加载性（是否同样需 adapter）；④ mbf action 话题族精确名（随节点名）与 C7 relay 源话题名（mbf 速度输出）；⑤ make_plan_* 的 GlobalPlanner 参数等价物 | **开；P6（mode_nav_stub 联调）关闭后回填固化** |

## §16 二期入口：同 v2（SCAN → L4 局部规划器、PCT → L3 全局、CUDA 11.4 硬阻塞、同步/异步 wrapper 坑保留）。补充：SCAN 若需改 teb 源码级共存，届时按 A9 例外流程走"复制 teb 源码 → 编译 → devel 遮蔽系统版"，并登记 diff。

v4 增补四条（N10）：

1. **py_trees/BT 词汇库**：二期将 C12 任务原语注册表对接 py_trees 行为树（外部依赖 R19：未装，评估 apt/pip 装法与版本后立项）。
2. **collision_guard**：多边形安全区急停（cmd_vel 纵深管道第五环，§2.6 决策 8；与 M6 mux 协同设计后入表）。
3. **SCAN 走 mbf 插件位按名点单**：SCAN→L4 局部规划器落地为 C0 `controllers` 注册表第二个条目，goal 内 `controller` 字符串按名切换（§2.6 决策 5 红利，零架构改动）。
4. **S9 已装后可再评估 mbf 原生全局规划器**（mbf 命名空间原生注册者），替代/并存 adapter 路线。

## §17 计划书状态

- ✅ 环境二次实测（新增：ACL 现状、swap、系统导航栈、message_transformer 节点与 include、faster-lio 参数命名空间与存盘路径、驱动话题名、lidar_type）
- ✅ 真机安全缺口已闭合（V1：qnx2ros/ros2qnx 拆分 + G5 校验 + P0.5 双查）
- ✅ 隔离体系补盲（V2 + 自检 [8]：第三方包内硬编码路径纳入监控）
- ✅ 四处事实错误修正（V3/V4/V5 + location_record 路径）
- ✅ 流程硬伤修复（V6 时序、V7 脚手架、V8 配置种子、V9 权限时序、V10 脚本 bug）
- ✅ ACL 冲突处置（V11）；✅ 编译优化落地（V12/V14：少三个包 + 临时 swap）
- ✅ **P0–P2 已按 v3 执行完毕（2026-10-04：G0/G1/G1.5/G2 全 PASS，见 `note/进度/00_总览.md`；正文冻结注记 N9）**
- ✅ **P3a.0 导航栈组件只读实测完成（2026-10-04，690 行 log；N1 事实基础，Q10 随之关闭）**
- ✅ **N1–N10 设计决策收编（M4 升级 mbf_costmap_nav 联邦式 + 契约先行/SMACH 简版 + SUP 职责 + 八决策规约 §2.6）**
- ⏸ 未执行：P3a 及以后全部阶段（本文不执行；S9 待 Q9 用户批准，未装）；⏸ 待确认：Q1/Q3/Q4/Q5/Q6/Q7/**Q8**/**Q9**/**Q11**（§15）
