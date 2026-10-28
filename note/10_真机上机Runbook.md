# Lite3 真机上机 Runbook（草案 v0.1）

> 状态：**草案，未上机验证**。本文件只描述现有仓库中真实存在的入口与约束，**不构成上机许可**。
> 真机属 L2：须真人批准作业窗、按单项作业单执行；外参／TF／速度上限／判据门值不得现场改。
> 依据：`note/00_README.md`（三条硬规则）、`note/09_跨工具协作契约.md`（A6、真机红线）、
> `note/计划书v8/24_仿真验收与迁移方案.md`（A0—A8、迁移规则）、`note/计划书v8/23_v8任务队列.md`（V8-X2 未开工）。
> 编写日期 2026-10-28。仓库基线：`main = 4e81247`。

---

## 0. 先确认三件事（缺一不发车）

1. **批准**：本次上机有真人批准的作业窗与单项作业单；V8-X2「真机逐项迁移」当前仍是**未开工**，不能默认沿用仿真结论。
2. **三件套**：该项的仿真判据读数通过 ＋ 验收记录 ＋ 真机作业单（S4 出口口径）。
3. **范围**：只跑作业单列出的 mode；不在真机上改外参、TF 树、速度上限、判据门值；不启 `ros2qnx`（真机导航 mode 尚未建立，见 §8）。

---

## 1. 目标机与网络

| 项 | 值 | 来源 |
| --- | --- | --- |
| 机型 | Lite3，Jetson tegra194 / aarch64，Ubuntu 20.04 | `docker/README.md` §一 |
| 系统 | ROS1 **Noetic**，`catkin_make`（不是 catkin_tools/colcon） | 同上 |
| 主机 IP / ROS master | `192.168.1.103:11312` | `scripts/env.sh` |
| 雷达 | Livox Mid-360，`192.168.1.201`，UDP `56301/56401` | `01_sensors.launch` 头注、`docker/README.md` |
| IMU 桥 | `qnx2ros` 监听 QNX 遥测 UDP **43897** | `01_sensors.launch` |
| 仿真端口（真机必须空） | `11311/11313/11345` | `ws/README.md` §三 |

端口冲突排查：

```bash
ss -lnt | grep -E '11311|11312|11313|11345'
ss -ulnp | grep -E '43897|56301|56401'
```

---

## 2. 取代码到 Lite3

远端：`git@github.com:javencpdd/ros1-slam.git`。**2026-10-28 实测** `refs/heads/main = 4e81247`（与开发机 HEAD 一致）。

### 方案 A：直接在 Jetson 上 clone（需能访问 GitHub）

```bash
git clone git@github.com:javencpdd/ros1-slam.git /home/ros1-slam
cd /home/ros1-slam && git log --oneline -1   # 核对与作业单记录的基线一致
```

无 SSH key 时用 HTTPS（需要账号令牌）：

```bash
git clone https://github.com/javencpdd/ros1-slam.git /home/ros1-slam
```

### 方案 B：离线 bundle（内网／无外网）

```bash
# 开发机
git bundle create ros1-slam-4e81247.bundle --all
scp ros1-slam-4e81247.bundle ros1-slam@192.168.1.103:/home/
# Jetson
git clone /home/ros1-slam-4e81247.bundle /home/ros1-slam
```

### 两条必须在克隆后补的东西

1. **`data/maps/*` 不入库**（`.gitignore` 只放行 `.gitkeep`）。样张三件套要单独拷贝：
   `lite3.pcd` / `lite3.pgm` / `lite3.yaml` → `/home/ros1-slam/data/maps/`。
   拷贝后记录 sha256，作为当次作业基线。
2. **`ws/build`、`ws/devel` 不入库**，必须在 Jetson 上重新编译。

> 目录必须落在 `/home/ros1-slam`：`scripts/env.sh` 与 `scripts/mapping/m_*.sh` 都写死了该路径；
> 换路径就要同步改这些脚本，属另一次变更。

---

## 3. Jetson 原生构建

依赖基线以 `docker/Dockerfile` 的 deps 阶段为准（同一份清单，容器／原生通用）。主要类别：

- 构建：`build-essential cmake pkg-config ninja-build git`
- 数学／点云：`libpcl-dev libeigen3-dev libtbb-dev libboost-all-dev libyaml-cpp-dev`
- 求解／并行：`libnlopt-dev libnlopt-cxx-dev libomp-dev libgoogle-glog-dev`
- Python：`python3-numpy python3-yaml python3-scipy python3-opencv`
- ROS：`ros-noetic-{catkin,roscpp,rospy,tf,tf2,pcl-ros,costmap-2d,nav-core,global-planner,dwa-local-planner,teb-local-planner,move-base,move-base-flex,map-server,spatio-temporal-voxel-layer,sob-layer,smach,grid-map,...}`

**Livox-SDK2 必须先装**（`livox_ros_driver2` 编译依赖 `/usr/local/lib/liblivox_lidar_sdk_static.a`）。

```bash
# 0) 账户与权限：真机既有运行账户为 ros1-slam（P7 演练口径）
sudo -iu ros1-slam

# 1) 环境
source /opt/ros/noetic/setup.bash

# 2) 编译（-DROS_EDITION=ROS1 不能漏，否则 livox_ros_driver2 走 ROS2 分支）
cd /home/ros1-slam/ws
catkin_make -DROS_EDITION=ROS1

# 3) 入口自检
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/scripts/env.sh          # 清 foxy、锁 noetic、devel 守卫
rospack find lite3_bringup
rospack find lite3_navigation
```

注意：

- 若手工用过 `catkin_make --pkg`，白名单会写进 `CMakeCache.txt`；后续先清缓存（`docker/README.md` 坑清单）。
- `scripts/env.sh` 会清掉 foxy 环境；**不要**在 foxy 生产栈仍在跑时起本栈（43897 会冲突，见 §4）。

---

## 4. 上机前检查表（全绿才起栈）

| # | 检查 | 命令 / 判据 | 阻塞 |
| --- | --- | --- | --- |
| 1 | foxy 生产栈已停（Q8／方案 A） | 按现场作业单执行；停后确认 | 是 |
| 2 | 43897 空闲 | `ss -ulnp \| grep 43897` 无输出 | 是 |
| 3 | ROS master 端口空闲 | `ss -lnt \| grep 11312` 无输出 | 是 |
| 4 | 无残留节点／进程 | `rosnode list` 空；`ps` 无 roscore/roslaunch | 是 |
| 5 | 雷达网络 | `ping -c 3 192.168.1.201` | 是 |
| 6 | 样张三件套在位 | `ls data/maps/lite3.{pcd,pgm,yaml}`；记录 sha256 | 是 |
| 7 | 磁盘余量 | 日志与 PCD 落盘空间；`df -h /home` | 是 |
| 8 | 时间同步 | `date` 与地面站一致 | 是 |
| 9 | 版本基线 | `git -C /home/ros1-slam rev-parse --short HEAD` 与作业单一致 | 是 |
| 10 | 外参／TF 未改 | `base_link→rslidar` 由驱动提供；不得重复发布 | 是 |
| 11 | 速度上限 | 以作业单批准值为准，现场不改 | 是 |
| 12 | 急停／遥控器 | 急停可达、遥控器接管可用 | 是 |
| 13 | 场地 | 清空、人员撤出运动范围 | 是 |
| 14 | mux 配置 | `lite3_safety/config/cmd_vel_mux.yaml` 与作业单一致 | 是 |
| 15 | 采图体检（仅采图） | `bash scripts/mapping/m_check.sh`（11 项，rc=0） | 是 |
| 16 | `ros2qnx` 零命中 | 起栈后 `rosnode list \| grep ros2qnx` 为空 | 是 |
| 17 | 日志目录可写 | `/home/ros1-slam/logs` | 否 |
| 18 | 备份 | 样张、关键配置已备份（见 §7） | 是 |

---

## 5. 三种现有 mode（真机可直接起的全部）

共同起法：先起 master，再单命令起 mode；`rviz` 默认 `false`（SSH 无 X）。

```bash
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/scripts/env.sh
roscore -p 11312
# 另开一个终端（同一账户、同一环境）
roslaunch lite3_bringup mode_mapping.launch
```

### 5.1 `mode_mapping`（建图，不动机器人）

- 组合：SUP + M0（驱动 + `qnx2ros`）+ M1（FAST-LIO）。
- 预期：`/rslidar_points` ≈10 Hz、`/livox/imu` ≈200 Hz、`/Odometry` ≈10 Hz；`/lite3/supervisor/hold` 静默；`ros2qnx` 零。
- **GAP-5 红线**：`faster_lio pcd_save_en=true`，退出时可能覆盖 `data/maps/lite3.pcd`。
  起栈前备份三件套并记 sha256；退净后复核，漂移即恢复并登记。
- 参考：v7 P7 演练 8 节点（**历史读数，不作本次判据**）。

### 5.2 `mode_localize`（定位，不动机器人）

- 组合：SUP + M0 + M2（hdl，自研承载）+ M6（安全 mux）。
- 前置：43897 空闲（`qnx2ros` 要 bind）；foxy 已停。
- 预期：`/imu/data` ≈200 Hz；`/globalmap` latched、frame=`map`；`map→base_link` 活估在动；**无 `map_server`**（A6 裁决）；`/cmd_vel` 无发布者。
- 参考：P7 演练 12 节点（历史读数）。

### 5.3 `mode_nav_stub`（导航桩，不驱动机器人）

- 组合：SUP + M0 + stub TF（`map→base_link` 恒等 @10 Hz）+ M6 + M4（MBF）。
- 预期：`/cmd_vel` 发布者**唯一 = `safe_cmd_vel_mux`**；`/cmd_vel_raw` 由 MBF 输出；`ros2qnx` 零。
- 用途：验证规划/仲裁链活性；**不动底盘**。真机运动桥 `ros2qnx` 只在二期真机导航 mode，且当前没有该 launch。
- 参考：P7 演练 11 节点（历史读数）。

---

## 6. 采图专项（`mode_mapping` 的固化流程）

见 `scripts/mapping/README.md`，五步：

```bash
tmux new-session -A -s p3b
cd /home/ros1-slam/scripts/mapping
bash m_check.sh          # ① 体检 11 项
bash m_clean.sh          # ② 清场
bash m_start.sh          # ③ 三窗启动：core / drv / slam
bash m_verify.sh         # ④ 频率联锁验证（ODOM=1 加验 /Odometry）
bash m_pose.sh           # ⑤ 回起点记坐标
NAME=YYYYMMDD_采集 bash m_stop.sh   # 停机 + 产物改名
```

铁律：

- 停机顺序：**SLAM 先停**，等日志打出 `current scan saved` 与 `finish done`，再停驱动与 master；
  没打完**不许关 tmux 窗**。
- 采图全程**不发 `/cmd_vel`**；`ros2qnx` 禁启。
- `m_stop.sh` 默认按“月日_时分”重命名产物；需要保留 `lite3.pcd` 用 `KEEP_LATEST=1`。

---

## 7. 停止、退净与回滚

### 7.1 正常停止

1. `mode_mapping`：按 §6 的 `m_stop.sh`；其他 mode：在 `roslaunch` 窗 `Ctrl-C`。
2. 退净判据：

```bash
rosnode list                      # 空
ss -lnt | grep 11312              # 无输出
ss -ulnp | grep 43897             # 无输出
ps -ef | grep -E 'roscore|roslaunch' | grep -v grep   # 无输出
```

3. 复核样张三件套 sha256；与 §4 基线一致。

### 7.2 回滚

- 地图被覆盖：从备份恢复 `lite3.{pcd,pgm,yaml}`，复核 sha256，登记偏差。
- 栈起不来：先查 §4 第 1—4 项（foxy、43897、端口、残留），不要靠反复重启掩盖。
- 恢复 foxy 生产栈：按 P7 的 A11/A12 口径恢复并核对 enabled 状态（现场作业单为准）。

---

## 8. 已知缺口（本草案明确不覆盖）

| 缺口 | 现状 | 影响 |
| --- | --- | --- |
| **真机导航 mode** | `mode_navigation.launch` **不存在**；P7 明确二期不建文件 | 现在不能在真机上闭环行驶；`mode_nav_stub` 只验证链，不动底盘 |
| 底盘运动桥 | `ros2qnx` 仅允许出现在真机导航 mode；启前须 P0.5 运动链路双查 | 无该 mode 就没有经批准的运动通路 |
| 参数迁移表 | 外参／TF／速度上限／地图版本无“仿真→真机”对照件 | 现场不得自行填值，需单项作业单给值 |
| 仿真定标值 | 定位 0.22 m 等是**仿真带域**值（走廊、LIO 单后端、9 run） | 不能当实机精度证书 |
| **仿真步态（V8-X1）** | 准静态爬行只在 MuJoCo 验证（相位机＋数值 IK＋关节 PD，平地）；**只承诺相位结构与判据阈值**（步序、落足点规则、支撑裕度 ≥5 cm、滑移/高度门限） | 真机增益（kp/kd、±30 N·m 限幅）须重标；接触/摩擦、时延带宽、位姿估计（IMU／腿式里程计）均未迁移——**不承诺真机鲁棒性**，见 R826 §6 |
| 原生安装脚本 | 仅有 `docker/Dockerfile` 依赖清单，无 Jetson 一键安装 | §3 的手工安装尚未实机验证 |
| 回滚脚本 | 无；按 §7 手工执行 | 依赖现场作业单 |

**禁止事项**：改第三方原件；直发 `/cmd_vel` 绕过 mux；在非真机导航 mode 启 `ros2qnx`；现场改外参／TF／速度上限；把仿真 PASS 当实机验收；**把 V8-X1 仿真步态的 PASS 当真机可用性证据**（只可引用相位结构与判据阈值，见 R826 §6）。

---

## 9. 留证

每次上机至少记录：`git rev-parse --short HEAD`、样张三件套 sha256、起止时间、mode 名、
`rostopic hz` 关键读数、`rosnode list` 截图或文本、`/cmd_vel` 发布者清单、异常与处置。
按三态通道记 PASS / FAIL / INVALID；INVALID 不入串。作业单与记录进 `note/进度/` 对应批次。

---

## 附录 A · 文件索引

| 主题 | 文件 |
| --- | --- |
| 硬规则（ros2qnx、隔离、对照组） | `note/00_README.md` |
| 真机红线与 A6 | `note/09_跨工具协作契约.md` |
| 迁移与验收面 | `note/计划书v8/24_仿真验收与迁移方案.md` |
| 真机任务状态 | `note/计划书v8/23_v8任务队列.md`（V8-X2） |
| 环境入口 | `scripts/env.sh` |
| 采图流程 | `scripts/mapping/README.md` |
| 驱动链 | `ws/src/00_drivers/lite3_drivers/launch/01_sensors.launch` |
| 三种 mode | `ws/src/70_bringup/lite3_bringup/launch/mode_{mapping,localize,nav_stub}.launch` |
| 安全 mux | `ws/src/60_safety/lite3_safety/{launch/60_safety.launch,config/cmd_vel_mux.yaml}` |
| 导航（MBF） | `ws/src/40_navigation/lite3_navigation/launch/mbf_navigation.launch` |
| 历史实机演练 | `note/进度/v7/S4/P7/进度文档.md` |
| 容器构建与依赖 | `docker/README.md`、`docker/Dockerfile` |

## 附录 B · 修订

- v0.1（2026-10-28）：首版草案，依据当前仓库与 v7 P7 历史演练记录；**未在真机上执行验证**。
- v0.2（2026-10-28）：按真人指示补「仿真步态」交接口径——§8 新增缺口行「仿真步态（V8-X1）」、
  禁止事项补一句；口径来源 `R826` §6／`R828` §6（另起句，不改旧读数）。
