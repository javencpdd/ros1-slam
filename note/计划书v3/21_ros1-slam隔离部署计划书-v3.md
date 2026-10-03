# 21 · 103 主机 `ros1-slam` 用户：模块化架构隔离部署计划书（v3）

> **状态：计划书（Plan），未执行。本文不创建目录、不复制文件、不修改 103 上任何内容。**
> 基线来自 2026-10-04 对 `lite3-f20-1-103`（192.168.1.103，即本机）的**二次实测**（在 v2 实测基础上复核并新增 8 项取证）。
>
> **v3 = v2 的全面修订版。** v2 骨架（阶段 DAG、Gate、双目录制、回滚矩阵）经审查全部保留；
> v3 修正了 v2 的 **4 处事实错误、1 处真机安全缺口、5 处流程硬伤、2 个自检脚本 bug**，
> 并吸收实测带来的 3 项优化（可省约 40 分钟编译）。

---

## v3 变更日志（相对 v2，按严重程度排序）

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

## §0 三份文档的分工

| 文档 | 回答的问题 | 本文怎么用 |
|---|---|---|
| **19 号** | 场景选择、SCAN/PCT 归位、施工次序 | 提供次序（W1–W7）与验收判据（A1–A5 / B1–B5） |
| **20 号** | 模块切分、接口契约、launch 组织 | 提供骨架（M0–M6、`/cmd_vel_raw`、三层 launch、variants、Stub/Sink） |
| **21 号（本文 v3）** | 具体怎么在 `ros1-slam` 家目录建起来 | 目录树 + 属主权限 + 分阶段命令 + 校验点 + 回滚 + 风险 |

> **`ros1-slam` 是手术台，不是新家**（v2 §0.1 结论保留）：`ysc/lite_cog` 原样作为对照组与回退基准，单向复制（A6）。

---

## §1 假设与基线（v3 全部经二次实测）

### 1.1 假设

| # | 假设 | 依据 / v3 变更 |
|---|---|---|
| **A1** | ROS = **Noetic**（`/opt/ros/noetic`），不碰 foxy | 实测双装（foxy + noetic），本文只用 noetic |
| **A2** | 20 号 M0–M6 七模块；一期实装 M0/M1/M2/M6 + M4 stub，M3/M5 占位 | 同 v2 |
| **A3** | 传感器 = Leishen C16（192.168.1.201）+ 狗体 IMU（`/imu/data`） | 实测 `lslidar_c16.launch`；`/imu/data` 由 **`qnx2ros`** 节点发布（实测 `qnx2ros.cpp:53`） |
| **A4** | 网络 192.168.1.0/24，有出网 | 实测 |
| **A5** | 执行者 = `ysc`（有 sudo）代劳；`ros1-slam` 无 sudo、当前无法登录 | 实测；Q1 |
| **A6** | 隔离单向：`ysc/lite_cog` → `ros1-slam`，**绝不反向** | v3 新增强调：V2 的写穿路径是唯一已发现的反向通道，已处置 |
| **A7** | 一期不引入 SCAN/PCT 源码，只预留 `opt/research/` | 同 v2 |
| **A8** | 与 ysc 侧不同时抢 LiDAR UDP（2368）**及底盘运动链路** | v3 扩展：P0.5 检查项从"只查 LiDAR"扩为"查传感器 + 运动链路"（V1） |
| **A9** | **v3 新增**：teb/move_base/map_server/mbf 用系统已装版本，不复制不编译 | V12 |
| **A10** | **v3 新增**：`message_transformer` 四节点中，一期沙箱只允许 `qnx2ros`（+可选 `sensor_checker`）；`ros2qnx`/`nx2app` 仅真机 mode | 实测 `CMakeLists.txt` add_executable 清单 + `ros2qnx.cpp:112` |

### 1.2 实测基线（2026-10-04 二次复核）

| 项 | 实测值 | 对计划的影响 |
|---|---|---|
| `ros1-slam` 用户 | `uid=1001 gid=1003`，**不在 sudo/video/i2c/gpio/plugdev** | P0.2 补组 |
| **`/home/ros1-slam` ACL** | `user:ysc:rwx` + **default ACL 已设**（2026-10-04 上一轮操作） | ⚠️ 与 §3.3 chmod 冲突 → P0 先 `setfacl -b`（V11） |
| SSH 登录 ros1-slam | ❌ Permission denied | 全程 `sudo -u` 代劳 → `as_ros1slam.sh` |
| 磁盘 | **43 GB 可用**（70G 已用 / 117G） | 预算见 §1.6 |
| 系统 | Ubuntu 20.04.6，gcc 9.4.0 | — |
| **CPU/内存/swap** | 6 核 / 6.7 GiB / **zram 2×571 MB** | ⚠️ 编译加临时 swap（V14） |
| ROS | noetic；`tf2 0.7.7`、`pcl_ros 1.7.4` | — |
| PCL/Eigen/Boost/OpenCV/Ceres | 1.10.0 / 3.3.7 / 1.71 / 4.2.0 / 1.14.0 | 同 v2 |
| **系统已装导航栈** | `move_base` `move_base_flex` `mbf_costmap_nav` **`teb_local_planner`(0.9.1)** `map_server` `octomap_ros` `move_base_sequence` 全部 ✅ | **A9/V12：不复制不编译** |
| 32 个关键 ROS 包 | ✅ 全装 | P4 仅核对 |
| `py_trees` / `twist_mux` | ❌ 均未装 | 编排用 SMACH；M6 自研 mux |
| **logrotate** | ✅ **已装并启用**（2026-10-04 修复） | 磁盘风险项消除 |
| 相机 `/dev/video*` | 666 + plugdev ACL | ros1-slam 可直读 |
| LiDAR | 以太网 UDP 2368/2369 | 驱动发布相对名 `rslidar_points`（`lslidar_driver.cpp:159`，launch 参数 `pointcloud_topic`） |
| 当前进程 | `ps aux` 无 lslidar/faster_lio/hdl/roscore | 施工窗口干净 |
| **faster-lio 关键事实** | `lidar_type: 4`；`lid_topic: /rslidar_points_filtered`（**无生产者**）；存盘路径硬编码 `/home/ysc/lite_cog/system/map/lite3.pcd`（`laser_mapping.cc:751`）；参数经**全局 NodeHandle** 读取（`run_mapping_online.cc:25`） | V2/V4/V5 + §C1 |
| **message_transformer** | catkin ROS1 包，4 节点：`qnx2ros`（→`/imu/data` 200Hz）、`ros2qnx`（`cmd_vel`→底盘）、`nx2app`（joy）、`sensor_checker`；include 全自包含 | A10、V1、V13 |

### 1.4 继承自 19 号的五个硬事实（不变）

F1 TEB 未避障（`include_costmap_obstacles: false`）；F2 STVL 三坑（`clearing:false`/`min_z:0.2`/`obstacle_range:3.0`）；F3 `dyn_reconfigure.py` 失效且压速 0.1；F4 `system/map/` 为空；F5 顺序不能倒（补图→修避障→mux→稳 D2→SCAN→PCT）。

### 1.6 磁盘预算（v3 微调）

| 项 | 占用 | v3 变更 |
|---|---|---|
| 源码 | ≈ 0.19 GB | 少了 teb/move_base/map_server |
| `build/` | 3–6 GB | 少了批 4，略降 |
| `devel/` | 0.5 GB | |
| 地图 + bag | < 0.3 GB + 0.5 GB/趟 | 规则不变：bags 只留 3 趟 |
| **合计** | **≈ 5–6.5 GB**，余量 ≈ 36 GB ✅ | |

---

## §2 目标形态（v3 修订点）

架构图、模块编号、`/cmd_vel_raw` 契约、`variants/`、Stub/Sink 全部继承 v2 §2。**两处修订**：

1. **M0 的 `message_transformer` 启动拆分**（V1）：

```xml
<!-- lite3_drivers/launch/01_sensors.launch —— 一期唯一允许的形态 -->
<node pkg="message_transformer" type="qnx2ros" name="qnx2ros" output="screen"/>
<!-- ⚠️ ros2qnx（订阅 /cmd_vel 直发 QNX 底盘）在此 launch 中绝对禁止出现；
     它只允许出现在 mode_navigation（真机导航）中，且启动前必须过 P0.5 运动链路检查。
     nx2app（joy 桥）、sensor_checker 同理，一期不启动。 -->
```

2. **M1 话题契约**（V5）：C16 驱动发布 `rslidar_points`（相对名，launch 起在根命名空间即 `/rslidar_points`）；faster-lio 配置的 `lid_topic` 一律写 **`/rslidar_points`**。`/rslidar_points_filtered` 话题名从沙箱词汇表中删除（ysc 侧该话题的真实生产者待查，见 Q7，不影响本文）。

---

## §3 目录结构

完整目录树继承 v2 §3.1，v3 修订四处：

1. **笔记目录定名为 `note/`**（用户指定；v2 写的是 `notes/`，全文已统一）。
2. **`40_navigation/` 下不再出现 `teb_local_planner`、`move_base`、`navigation` 三个目录**（A9）——只有自研壳 `lite3_navigation/`。
3. `10_slam/map_server` 目录**删除**（系统已装；`rosrun map_server map_saver` 直接可用）。
4. 新增 **`opt/ref/` → 自研壳的"配置种子"语义**（见 C0）。

### 3.2 双目录制 v3 修订条款

> **第三方包"原样复制、一个字不改"原则增加唯一例外（V2）**：
> `faster_lio/src/laser_mapping.cc:751` 的存盘路径硬编码写穿隔离边界，**必须**在复制后立即改写为
> `/home/ros1-slam/data/maps/`。这是全文唯一许可的第三方包源码改动，必须登记进
> `note/04_配置改动diff.md`（旧值/新值/理由/日期），且自检 [8] 会把它纳入永久监控。
> 判断标准写死：**凡影响隔离边界（读写 `/home/ysc`）的第三方源码行为，隔离规则优先于"一字不改"。**

### 3.3 属主与权限（v3 时序修正）

权限模型表继承 v2（`/home/ros1-slam` 750、目录 755、文件 644、脚本 755、`opt/ref/` 555）。
**v3 修正执行时序与前置（V9/V11）**：

| 步骤 | 时机 | 命令 |
|---|---|---|
| ⓪ 清 ACL | **P0.6（复制之前）** | `sudo setfacl -bR /home/ros1-slam`（清掉 2026-10-04 设的 ysc ACL，避免与后续 chmod 的 mask 冲突；此后一切访问走 `sudo -u` / `sudo cp`） |
| ① chown | **P2 复制完成后** | `sudo chown -R ros1-slam:ros1-slam /home/ros1-slam` |
| ② chmod 目录/文件 | 紧随 ① | v2 原 ② ③ ④ 不变 |
| ⑤ 重建 ysc 便利通道（可选） | P2 末 | 若希望 ysc 免 sudo 浏览沙箱：`sudo setfacl -Rm u:ysc:rX /home/ros1-slam && sudo setfacl -Rdm u:ysc:rwx /home/ros1-slam`，**此步必须在 ② 之后**，且注意 rX 不会破坏脚本 755（与 v2 的 mask 顾虑相容） |

---

## §4 阶段总览与依赖（v3 修订 DAG）

```
Phase 0  环境准备（登录/组/ACL清理/磁盘/施工窗口）      [需 root]
   │  Gate G0：ros1-slam 可用（登录 或 确认 sudo 代劳）；ACL 已清
   ▼
Phase 1  目录初始化 + catkin 初始化
   │  Gate G1：骨架齐全
   ├─ P1.5 ★ 自研壳脚手架生成（8 个 lite3_* 包骨架）     [v3 新增 V7]
   │  Gate G1.5：8 个包 catkin 可识别（package.xml 就位）
   ▼
Phase 2  外部文件复制 + faster-lio 存盘路径改写 + 路径扫描替换
   │  Gate G2：v3_isolation.sh 退出码 0
   ├─ P2 末：统一 chown + chmod（v3 时序 V9）
   ▼
Phase 3a ★ 配置重构（W2–W7 落地 + C0 配置种子 + C1 yaml 转换）   [纯写文件，不依赖编译]
   │  Gate G3a：配置就位（rosparam 可加载、diff 文档齐全）
   ▼
Phase 4  依赖核对（实测已满足，10 分钟）
   ▼
Phase 5  编译构建（v3 五批 → 实为四批，V12；先加临时 swap V14）
   │  Gate G4：devel/setup.bash 存在 + ROS_PACKAGE_PATH 无 ysc
   ▼
Phase 3b ★ 现场采图（原 C1 的执行部分，依赖 P5 编译产物）        [v3 时序修正 V6]
   │  Gate G3：data/maps/ 有 lite3.pcd/.yaml/.pgm
   ▼
Phase 6  分层验证（Stub/Sink 逐模块；G5–G9）
   ▼
Phase 7  场景组合启动（modes/）
   ▼
Phase 8  回滚（任意阶段可执行）
```

---

## §5 Phase 0：环境准备

### P0.1 登录可用（同 v2：A 设密码 + B 分发公钥，推荐都做）

### P0.2 补充用户组（同 v2：`plugdev,video` 必、`i2c,gpio` 选、**sudo 不加**）
实测 `gpio/i2c/plugdev/video` 四个组都存在，一条 `usermod` 即可。

### P0.3 系统依赖清单（同 v2，全部 ✅；CUDA 11.4 < 11.7 硬阻塞 PCT 不变）

### P0.4 需要 sudo 的操作清单（继承 v2 S1–S6，另加）

| # | 操作 | Phase |
|---|---|---|
| S7 | `setfacl -bR`（V11）+ P2 末 chown/chmod | P0/P2 |
| S8 | 临时 swap 增删（V14） | P5 前/后 |

### P0.5 施工窗口确认（v3 扩展，V1）

```bash
ps aux | grep -E "lslidar|faster_lio|hdl_|roscore|rosmaster" | grep -v grep   # 传感器侧，期望空
ps aux | grep -E "ros2qnx|message_transformer|lite3_ros_bridge" | grep -v grep  # ★ 运动链路侧，期望空
```

> 第二条是 v3 新增：若 ysc 侧 monitor/桥接栈在跑，`/cmd_vel` 链路活跃，
> stub 模式的任何手滑 pub 都可能真驱动机器狗。**真机 mode（含 ros2qnx）启动前必须重查本条。**

### P0.6 ★ 清理 ACL（v3 新增，V11）

```bash
sudo setfacl -bR /home/ros1-slam
getfacl /home/ros1-slam | grep -c "user:ysc"   # 期望 0
```

### P0.7 sudo 会话技巧

每阶段开一个持权 shell（`sudo -v` 刷新 15 分钟票据；或 `sudo -i` 常驻），避免几十条 `sudo -u` 命令反复输密码。

---

## §6 Phase 1：目录初始化

P1.1 建骨架 / P1.2 `catkin_init_workspace` 同 v2（目录树按 §3 修订：无 teb/move_base/map_server 子目录，笔记目录为 `note/`）。

### P1.5 ★ 自研壳脚手架生成（v3 新增，V7）

```bash
# /home/ros1-slam/scripts/gen_shell_pkgs.sh —— 生成 8 个 lite3_* 壳包骨架
#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/ros1-slam/ws/src
declare -A PKGS=(
  [00_drivers/lite3_drivers]="M0"
  [10_slam/lite3_slam]="M1"
  [20_localization/lite3_localization]="M2"
  [30_perception/lite3_perception]="M3"
  [40_navigation/lite3_navigation]="M4"
  [50_task/lite3_task]="M5"
  [60_safety/lite3_safety]="M6"
  [70_bringup/lite3_bringup]="TOP"
  [90_supervisor/lite3_supervisor]="SUP"
)
for rel in "${!PKGS[@]}"; do
  name=$(basename "$rel"); d="$ROOT/$rel"
  mkdir -p "$d"/{launch/stub,launch/sink,config,scripts,verify}
  cat > "$d/package.xml" <<EOF
<?xml version="1.0"?>
<package format="2">
  <name>$name</name>
  <version>0.1.0</version>
  <description>lite3 shell package (${PKGS[$rel]}) - self-owned launch/config/scripts only</description>
  <maintainer email="ysc@local">ysc</maintainer>
  <license>Proprietary</license>
  <buildtool_depend>catkin</buildtool_depend>
</package>
EOF
  printf 'cmake_minimum_required(VERSION 3.0.2)\nproject(%s)\n\ncatkin_package()\n' "$name" > "$d/CMakeLists.txt"
  echo "# $name（模块 ${PKGS[$rel]}）" > "$d/README.md"
done
```

**Gate G1.5**：`find /home/ros1-slam/ws/src -name package.xml | grep -c lite3_` → **9**（含 50_task 的 8 壳 + 未来补充；至少 8）。
M5（lite3_task）同样生成，虽为一期占位，但空壳可编译，省去白名单特判。

### P1.6 `.bashrc` 追加（v3 落实，V10 关联）

v2 目录树里标了"[追加]"却没有 Phase 落实，v3 定为 P1 收尾步骤：

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

### 7.1 复制清单（v3 修订版）

| # | 来源 | 目标 | v3 变更 |
|---|---|---|---|
| 1 | `…/drivers/leishen_ws/src/lslidar_driver` | `ws/src/00_drivers/lslidar_driver` | 不变 |
| 2 | `…/drivers/leishen_ws/src/lslidar_msgs` | `ws/src/00_drivers/lslidar_msgs` | 不变 |
| 3 | `…/transfer/src/message_transformer` | `ws/src/00_drivers/message_transformer` | 不变（★ M0 必需） |
| 4 | `…/slam/src/faster-lio` | `ws/src/10_slam/faster_lio` | 不变（排除 .git）；**复制后立即执行 §7.2a 存盘路径改写** |
| 5 | `…/slam/src/pcd2grid` | `ws/src/10_slam/pcd2grid` | 不变 |
| ~~6~~ | ~~`…/slam/src/map_server`~~ | ~~删除~~ | **🆕 不复制**：系统已装（A9） |
| 7 | `…/nav/src/hdl_localization` | `ws/src/20_localization/…` | 不变 |
| 8 | `…/nav/src/hdl_global_localization` | 同上 | 不变 |
| 9 | `…/nav/src/ndt_omp` | 同上 | 不变 |
| 10 | `…/nav/src/fast_gicp` | 同上 | 不变 |
| ~~11~~ | ~~`…/nav/src/teb_local_planner`~~ | ~~删除~~ | **🆕 不复制**：系统已装 0.9.1，与源码版本一致（V12） |
| ~~12~~ | ~~`…/nav/src/move_base`~~ | ~~删除~~ | **🆕 不复制**：系统已装 |
| 13 | `…/nav/src/navigation/config/` | `opt/ref/navigation_params_orig/` | 不变（只读参照；v3 明确：navigation 包**任何部分都不进 ws**） |
| 14 | `…/voa/src/deeprobotics_local_height_map`+`libraries` | `ws/src/30_perception/` | 不变 |
| 15 | `…/pipeline/src/pipeline_tracking` | `ws/src/50_task/pipeline_tracking` | 不变；C4 排除 `scripts/dyn_reconfigure.py`；**注意 `location_record.py` 在 `tools/` 子目录**（v2 误写 scripts/） |
| 16 | `/home/ysc/faster_lio_online.yaml` | `ws/src/10_slam/lite3_slam/config/faster_lio_outdoor.yaml` | **🆕 ROS2→ROS1 转换后才落位**（转换稿见 §C1，原文件同时留档 `opt/ref/faster_lio_ros2_orig.yaml`） |
| 17 | `/home/ysc/c16__lslidar_driver_node.yaml` | `opt/ref/lidar_c16_orig.yaml` | 不变（ROS2 格式只作参照） |
| ~~18~~ | `realsense2_camera` | — | 🆕 明确**二期才复制**，一期清单中删除 |

### 7.2 复制命令：同 v2（`copy_pkg` tar 管道，排除 `.git/build/devel`）。

### 7.2a ★ faster-lio 存盘路径改写（v3 新增，V2，复制后立即执行）

```bash
F=/home/ros1-slam/ws/src/10_slam/faster_lio/src/laser_mapping.cc
sudo grep -n '/home/ysc/lite_cog/system/map' "$F"     # 应命中 751 行
sudo sed -i 's#/home/ysc/lite_cog/system/map/lite3#/home/ros1-slam/data/maps/lite3#g' "$F"
sudo grep -n '/home/ros1-slam/data/maps' "$F"         # 复核改写成功
echo "- faster_lio/src/laser_mapping.cc:751 存盘路径 /home/ysc/... → /home/ros1-slam/data/maps/（隔离改写，唯一许可的第三方源码改动）$(date +%F)" \
  | sudo tee -a /home/ros1-slam/note/04_配置改动diff.md
```

### 7.3 绝不复制 build/devel/.git：同 v2（R1/R2 机制与翻车案例保留）。

### 7.4 路径改写（v3 白名单扩充，V15）

```bash
cd /home/ros1-slam
# 扫描留档
grep -rn "/home/ysc" --include="*.launch" --include="*.yaml" --include="*.xml" \
     --include="*.sh" --include="*.py" --include="*.rviz" --include="*.cfg" \
     --include="CMakeLists.txt" --include="*.cpp" --include="*.h" --include="*.cc" \
     ws/src 2>/dev/null | tee note/02_问题与坑.md.scan
# 替换（仅 ws/src；notes 与 opt/ref 保持原样以存来源）
grep -rl "/home/ysc" --include="*.launch" --include="*.yaml" --include="*.sh" \
     --include="*.py" --include="*.rviz" --include="*.cc" --include="*.cpp" \
     ws/src 2>/dev/null | xargs -r sudo sed -i \
  's#/home/ysc/lite_cog/system/map#/home/ros1-slam/data/maps#g; s#/home/ysc#/home/ros1-slam#g'
# 复扫：期望 0（laser_mapping.cc 已在 7.2a 处理）
grep -rn "/home/ysc" ws/src 2>/dev/null | wc -l   # 期望 0
```

> v3 注：`--include="*.cc"` 是必要的——faster-lio 源文件是 `.cc` 后缀，v2 的白名单漏了它，
> 而 751 行这类硬编码恰恰在 `.cc` 里。若 7.2a 已手工改写，此处会命中 0，属预期。

### 7.5 C16 参数（v3 无变更，保留）：以 `lslidar_c16.launch` 默认值为准（msop 2368 / difop **2369** / frame_id `rslidar` / max_range 150 / min_range 0.30），`c16__lslidar_driver_node.yaml`（ROS2 格式、difop 56401）仅留档不用。

---

## §8 Phase 3a：配置重构（纯写文件，编译前完成）

### C0 — ★ 配置种子（v3 新增，V8）

C2/C3 修改的对象在复制清单里**没有底稿来源**（navigation 只进了 `opt/ref/`）。P3a 第一步：

```bash
REF=/home/ros1-slam/opt/ref/navigation_params_orig
NAV=/home/ros1-slam/ws/src/40_navigation/lite3_navigation/config
sudo mkdir -p "$NAV/variants"
sudo cp "$REF/teb_local_planner_params.yaml" "$NAV/planner_teb.yaml"      # 底稿，文件名以实际为准
sudo cp "$REF/costmap_common_params_*.yaml"  "$NAV/costmap_common.yaml"
# 其余底稿按 opt/ref 实际文件补齐（local costmap、global costmap、mbf 配置）
ls "$REF"   # 先看有什么再拷，文件名以现场为准
```

### C1 — ★★ 补地图前置：faster-lio 配置转换（V2/V3/V4/V5 落地，19 号 W1）

**（a）ROS2 → ROS1 转换稿**（`ws/src/10_slam/lite3_slam/config/faster_lio_outdoor.yaml`）。
依据：`run_mapping_online.cc:25` 用**全局 NodeHandle**，参数读自全局命名空间 →
launch 中 `<rosparam>` 必须放在 **node 标签之外**，yaml 顶层直接是参数组：

```yaml
# faster_lio_outdoor.yaml —— ROS1 版（v3 C1 转换自 /home/ysc/faster_lio_online.yaml，勿再"修正"）
common:
  imu_topic: /imu/data            # qnx2ros 发布（qnx2ros.cpp:53，200 Hz）
  lid_topic: /rslidar_points      # ★ V5：原 /rslidar_points_filtered 全库无生产者，改驱动实际话题
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
  extrinsic_est_en: true
  gyr_cov: 0.1
max_iteration: 3
path_save_en: true
pcd_save:
  interval: 10        # ★ v3：原值 0（ROS2 yaml 实测）。interval=0 时 laser_mapping.cc:747 的
                      #   `pcd_save_interval_ > 0` 恒假，走查显示全程只累积不落盘——
                      #   落盘语义存疑（Q6），改 10 保证周期性写盘；现场验证后可回调
  pcd_save_en: true
preprocess:
  blind: 1.0
  lidar_type: 4       # ★ V4：实测=4 → VelodyneHandler 分支（pointcloud_preprocess.cc:20）。
                      #   C16 驱动输出与该分支兼容；绝不可按 v2 说法"修正"为 2
  scan_line: 16
  time_scale: 0.001
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

**（b）launch 加载形态**（`lite3_slam/launch/10_slam_mapping.launch`，注意 rosparam 在 node 外）：

```xml
<launch>
  <rosparam command="load"
            file="$(find lite3_slam)/config/faster_lio_outdoor.yaml"/>
  <!-- ★ rosparam 在 node 之外：全局 NodeHandle 读全局命名空间（/common/imu_topic 等）。
       副作用：参数占据全局命名空间，同一 roscore 只能跑一套 faster-lio（沙箱单实例，可接受） -->
  <node pkg="faster_lio" type="mapping_online" name="faster_lio_online" output="screen"/>
</launch>
```

**（c）建图执行流程（原 v2 C1 的 tmux 版脚本保留：`tmux_mapping.sh`，W4）**，挪到 Phase 3b（V6）执行，产物校验见 §10 P5.4。

### C2 — TEB 避障（W2）：对 `planner_teb.yaml`（C0 拷入的底稿）设
`include_costmap_obstacles: true`、`enable_homotopy_class_planning: false`、`max_vel_x: 0.4`、`planning_horizon: 4.0`、`dt_ref: 0.4`。校验 G3b（`teb_markers` 含 `PolygonObstacles` ≥1）同 v2。

### C3 — STVL 三坑（W3）：`costmap_common.yaml`（底稿同上）改
`clearing: true`、`obstacle_range: 8.0`、`voxel_decay: 10.0`、`voxel_size: 0.10`；`min_z` 先 `tf_echo base_link rslidar` 体检再定（C16 几何依据同 v2：ring7 打 10 cm 台阶上限 8.6 m）。

### C4 — 不复制 `dyn_reconfigure.py`（W5）：复制后 `sudo rm -f …/pipeline_tracking/scripts/dyn_reconfigure.py` 并记录；运行期校验 `max_vel_x ≥ 0.4` 同 v2。

### C5 — `TaskTransfer.py` 限次重试（W6）：改副本 `MAX_RETRY = 3` 同 v2。

### C6 — 航点目录（W7）：同 v2；**修正**：录航点工具路径为 `…/pipeline_tracking/tools/location_record.py`（PyQt5，需 `ssh -X` 或现场 HDMI，退化方案手写 JSON 不变）。

### C7 — `/cmd_vel_raw` 改名落地：同 v2（topic_tools relay；M4 输出绝不叫 /cmd_vel）。

### C8 — M6 自研 `safe_cmd_vel_mux.py`：同 v2（六源优先级、超时淘汰、**无源发零速**、恒 20 Hz；Lite3 底盘 250 ms 超时不能断）。**v3 补一句运行链路事实**：M6 输出 `/cmd_vel` 的下游消费者是 `ros2qnx`（`ros2qnx.cpp:112`）——所以 **stub 模式下 `/cmd_vel` 只到 mux 为止**，绝不启动 ros2qnx（V1）。

### C9 — `variants/` 三档：同 v2（outdoor_open / crowd / night）。

### C10 — rviz 默认关闭：同 v2（实测 `local_rslidar_imu.launch:74` rviz `required="true"`，SSH 无 X 会拖死整个 launch——判断正确，保留）。

**Gate G3a（v3 新增）**：`rosparam load` 上述 yaml 无报错；`note/04_配置改动diff.md` 覆盖 C2/C3/C4/C5/§7.2a 全部改动。

---

## §9 Phase 4：依赖核对

流程同 v2（先 source 再 import 的教训保留）。v3 结论：**R11 解除、Q2 关闭**（V13）——
`message_transformer` 的 include 实测仅 `../include/protocol.h`、`../include/sensor_logger.h`、
`message_transformer/ComplexCMD.h`、`message_transformer/SimpleCMD.h` 与标准 ROS 头；
package.xml 依赖全为标准 catkin 包。批 5 编译它基本无悬念。

## §10 Phase 5：编译构建

### P5.1 分批白名单（v3 修订，V12）

```bash
# 批 1：M0 驱动
catkin_make -j3 -DCATKIN_WHITELIST_PACKAGES="lslidar_msgs;lslidar_driver"
# 批 2：M1 建图
catkin_make -j3 -DCATKIN_WHITELIST_PACKAGES="faster_lio;pcd2grid"
# 批 3：M2 定位（最耗时：fast_gicp）
catkin_make -j3 -DCATKIN_WHITELIST_PACKAGES="ndt_omp;fast_gicp;hdl_localization;hdl_global_localization"
# 批 4：全部自研壳 + message_transformer（v3：原批 4 teb/move_base 删除，壳包提前到此处一次编完）
catkin_make -j3 -DCATKIN_WHITELIST_PACKAGES="message_transformer;lite3_drivers;lite3_slam;lite3_localization;lite3_perception;lite3_navigation;lite3_task;lite3_safety;lite3_bringup;lite3_supervisor"
# 批 5：清空白名单，全量收尾
catkin_make -j3 -DCATKIN_WHITELIST_PACKAGES=""
```

### P5.0 临时 swap（v3 新增，V14，编译前执行、P5 完成后撤除）

```bash
sudo fallocate -l 4G /tmp/build_swap && sudo chmod 600 /tmp/build_swap
sudo mkswap /tmp/build_swap && sudo swapon /tmp/build_swap
# …… 编译 ……
sudo swapoff /tmp/build_swap && sudo rm -f /tmp/build_swap
```

OOM 处置同 v2（`-j3`→`-j2`）；§P5.3 常见失败表继承 v2，
其中"livox 头文件"一行按 V4 修订：**`lidar_type=4` 走 Velodyne 分支，与 v2 的"C16 用 2"说法相反**，但结论一致——都不需要 Livox。

### P5.2 Gate G4：同 v2（devel/setup.bash 存在、`rospack find` 三例、`ROS_PACKAGE_PATH` 无 ysc）。
v3 追加：`rospack find teb_local_planner` 应指向 **`/opt/ros/noetic/share/teb_local_planner`**（证明在用系统版，A9 生效）。

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

### 11.1 `scripts/env.sh`（v3 加守卫，V10）

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

### 11.2 ★ 隔离自检 `scripts/verify/v3_isolation.sh`（v3 修复版，V10 + V2 盲区）

```bash
#!/usr/bin/env bash
FAIL=0
TMP=$(mktemp)

say() { echo "$1"; }

say "[1] ROS_PACKAGE_PATH 中是否含 /home/ysc"
if echo "$ROS_PACKAGE_PATH" | tr ':' '\n' | grep -q "/home/ysc"; then echo "  违规"; FAIL=1; fi

say "[2] 自研壳中是否含 /home/ysc"
if grep -rn "/home/ysc" /home/ros1-slam/ws/src/*/lite3_* >/dev/null 2>&1; then
  grep -rln "/home/ysc" /home/ros1-slam/ws/src/*/lite3_* 2>/dev/null; FAIL=1
fi

say "[3] .bashrc 是否 source 了其他 devel"
if grep -qn "lite_cog\|/home/ysc" /home/ros1-slam/.bashrc 2>/dev/null; then echo "  违规"; FAIL=1; fi

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
if [ -s "$TMP" ]; then cat "$TMP"; FAIL=1; fi

say "[6] ★ 是否有人绕过 M6 直写 /cmd_vel（20 号 K2）—— v3 修复管道计数 bug"
: > "$TMP"
grep -rn "cmd_vel" /home/ros1-slam/ws/src/*/lite3_*/launch/ 2>/dev/null \
  | grep -v "lite3_safety" | grep -v "cmd_vel_raw" >> "$TMP" || true
if [ -s "$TMP" ]; then cat "$TMP"; FAIL=1; fi

say "[7] 磁盘余量 ≥ 15 GB（19 号反判据④）"
AVAIL=$(df -BG /home/ros1-slam | awk 'NR==2{gsub("G","",$4); print $4}')
[ "$AVAIL" -lt 15 ] && { echo "  仅剩 ${AVAIL}G"; FAIL=1; }

say "[8] ★★ 全 ws/src 扫描 /home/ysc 硬编码（v3 新增：覆盖第三方包，补 V2 盲区）"
if grep -rn "/home/ysc" /home/ros1-slam/ws/src >/dev/null 2>&1; then
  grep -rln "/home/ysc" /home/ros1-slam/ws/src 2>/dev/null; FAIL=1
fi

rm -f "$TMP"
[ $FAIL -eq 0 ] && echo "=== ISOLATION OK ===" || echo "=== ISOLATION FAILED ==="
exit $FAIL
```

### 11.3–11.5 验证矩阵、命令序列、验收判据

继承 v2，v3 修订四处：

1. **M0 拆分校验（V1）**：G5 判据追加一条——`rosnode list | grep ros2qnx` 期望**为空**（stub/mapping/localize 模式下出现即为事故）。
2. **M1 判据（V5/V2）**：`rostopic hz /Odometry` ≥10 Hz 的前置是 faster-lio 真的吃到点云——先 `rostopic echo /common/imu_topic` 不需要；直接验 `rostopic hz /rslidar_points` ≥9 Hz（M0 已过 G5）再起 M1；PCD 落盘位置以 §P5.4 为准。
3. **M4 判据**：单位阵 TF + `teb_markers PolygonObstacles` 非空不变；**`rostopic info /cmd_vel` publisher 只能是 `safe_cmd_vel_mux`**。
4. A1–A5 / 20 号十条判据表整体保留。

## §12 Phase 7：场景组合（modes/）

三层 launch 与铁律同 v2。mode 表 v3 修订：

| mode | 包含 | v3 变更 |
|---|---|---|
| `mode_mapping` | M0(驱动+**qnx2ros**) + M1 | 明确 M0 仅 qnx2ros |
| `mode_localize` | M0(同上) + M2 + M6 | 同上 |
| `mode_nav_stub` ★ | M0(**仅驱动+qnx2ros**，不含 ros2qnx) + M4(stub TF) + M6 | **V1：真机运动桥严禁入列** |
| `mode_navigation`（二期） | 真定位 + 真导航 + **此时才允许 ros2qnx** | 启动前强制重跑 P0.5 运动链路检查 |

## §13 跨目录风险表（v3 修订版）

R1–R10、R12 继承 v2。修订与新增：

| # | 风险 | v3 变更 |
|---|---|---|
| R11 | message_transformer 隐藏依赖 | **降级为低**：实测 include 自包含（V13），保留"批 4 单独编"的处置 |
| **R13** 🆕 | **`ros2qnx` 写穿隔离 = 真机失控** | stub/mapping/replay mode 出现 ros2qnx 即事故。防线：§2 launch 禁入 + G5 校验 `rosnode list` + P0.5 双查 |
| **R14** 🆕 | **ACL 与 chmod 互相破坏** | P0.6 `setfacl -bR` 先清（V11）；P2 末如需 ysc 便利通道，ACL 在 chmod 之后重建 |
| **R15** 🆕 | **话题断链**（`/rslidar_points_filtered` 无生产者类问题） | 教训：复制来的 yaml 里的**每个 topic 参数**都要与上游 launch 的实际发布名对账（本文已对账：imu_topic ✅ / lid_topic 已修 ✅ / pointcloud_topic ✅） |
| **R16** 🆕 | **第三方包内硬编码路径**（`laser_mapping.cc:751` 类） | 复制后立即 §7.2a 改写 + 自检 [8] 永久监控；登记 diff |

## §14 回滚：同 v2。补充一行：faster-lio 源码改写若出问题 → 按 §7.2a 的 diff 记录逆向恢复（或整包重复制 + 重做改写）。

## §15 工期与待确认项

| Phase | 内容 | v3 修订后预估 |
|---|---|---|
| P0 | 用户/组/ACL/窗口 | 15 min |
| P1 | 目录 + 脚手架 + bashrc | 20 min |
| P2 | 复制 + 存盘路径改写 + 扫描替换 + 权限 | 40 min |
| P3a | 配置重构（C0–C10） | 1–2 h |
| P4 | 依赖核对 | 10 min |
| P5 | 编译（四批，无 teb/move_base/map_server）+ swap | **1–2 h**（v2 估 1.5–2.5h） |
| P3b | 现场采图 + PCD 回收 | 1–2 h（要现场走一趟） |
| P6 | 分层验证 | 3–5 h |
| P7 | 组合启动 | 1 h |
| **合计** | | **约 1.5 个工作日**（不含采图往返） |

| # | 待确认 | 状态 |
|---|---|---|
| Q1 | ros1-slam 能否登录 | 开（P0.1 解决） |
| ~~Q2~~ | ~~message_transformer 隐藏依赖~~ | **✅ 关闭（V13：自包含）** |
| Q3 | C16 安装倾角（定 `min_z`） | 开（P3a 先验 10 min） |
| Q4 | `/Odometry` 话题名与频率（二期 SCAN 输入） | 开 |
| Q5 | `location_record.py`（`tools/` 下）图形环境 | 开（ssh -X / HDMI / 手写 JSON） |
| **Q6** 🆕 | `pcd_save.interval=0` 的落盘语义（走查显示 747 行条件恒假可能永不落盘） | 开；v3 已先按 `interval: 10` 处理，现场验证 |
| **Q7** 🆕 | ysc 侧 `/rslidar_points_filtered` 的真实生产者（可能是未复制的过滤节点，或该 yaml 从未在 ysc 侧生效） | 开；沙箱已按 `/rslidar_points` 裁决，不影响本文 |

## §16 二期入口：同 v2（SCAN → L4 局部规划器、PCT → L3 全局、CUDA 11.4 硬阻塞、同步/异步 wrapper 坑保留）。补充：SCAN 若需改 teb 源码级共存，届时按 A9 例外流程走"复制 teb 源码 → 编译 → devel 遮蔽系统版"，并登记 diff。

## §17 计划书状态

- ✅ 环境二次实测（新增：ACL 现状、swap、系统导航栈、message_transformer 节点与 include、faster-lio 参数命名空间与存盘路径、驱动话题名、lidar_type）
- ✅ 真机安全缺口已闭合（V1：qnx2ros/ros2qnx 拆分 + G5 校验 + P0.5 双查）
- ✅ 隔离体系补盲（V2 + 自检 [8]：第三方包内硬编码路径纳入监控）
- ✅ 四处事实错误修正（V3/V4/V5 + location_record 路径）
- ✅ 流程硬伤修复（V6 时序、V7 脚手架、V8 配置种子、V9 权限时序、V10 脚本 bug）
- ✅ ACL 冲突处置（V11）
- ✅ 编译优化落地（V12/V14：少三个包 + 临时 swap）
- ⏸ 未执行：本文不创建任何目录、不复制任何文件
- ⏸ 待确认：Q1/Q3/Q4/Q5/Q6/Q7（§15）
