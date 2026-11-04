# Lite3 真机 3D 移植准备（离线批，2026-11-03）

> 更新时间：2026-11-04（v8 现行口径校对）｜ 权威口径：计划书 v8（note/计划书v8/）；本文件如与 v8 冲突，以 v8 为准

真机已关机。本目录是「真机回来即可接」的离线产物与读数，不含任何在真机上执行过的动作。
**本批已在容器内把整图跑到 local_arrived，命令链止于供 mux 取用的 `/cmd_vel_raw`。**

## 0. 真机资产（`data/maps/real_lite3_20261103/`，gitignore）

| 文件 | 说明 |
|---|---|
| `lite3.pcd` | 真机 2026-10-07 建图点云，386,549 点；sha256 `aaa09362…46f34`（与机器人逐字一致） |
| `lite3.yaml` | 2D 地图元数据：resolution 0.05、origin [-8.8, -4.35, 0] |
| `lite3_real_v2.json` | 本批生成：8 个支撑面 + **350 个障碍盒**，schema_version=2 |
| `lite3_real_v2_task.json` | 起终点任务（goal 已按 0.55 m 障碍余量回拉） |
| `registry.json` / `env_inventory.txt` / `m2_stage_b.log` | 注册表、真机环境清单（506 行）、运行日志 |

### 0.1 仓根统一入口（2026-11-03）

每台主机仓库位置可以不同，统一由一处声明，优先级：

1. `ROS1_SLAM_ROOT` 环境变量；
2. `scripts/ros1_slam.local.env`（**不入库**，每主机可写 `ROS1_SLAM_ROOT=/home/ros1-slam`）；
3. `scripts/ros1_slam.env`（入库模板，留空＝自动推导）；
4. 由 `scripts/env.sh` / Python `framework/paths.py` 所在仓库自动推导（容器 `/opt/ros1-slam`、Jetson `/home/ros1-slam` 都成立）。

已接入：`scripts/env.sh`（并 export 给所有子进程）、`framework/app_config.py`、`framework/map_from_pcd.py`、
以及 6 个 verify 件（保留各自的 `VCC_ROOT`/`VLB_ROOT`/… 优先，其后回落到 `ROS1_SLAM_ROOT`）。

现读：容器内 `source scripts/env.sh` ⇒ `ROS1_SLAM_ROOT=/opt/ros1-slam`；宿主 `ROS1_SLAM_ROOT=$PWD` 下账构
`rc=0`，不设时旧缺省 `/home/ros1-slam` 不在位 ⇒ `rc=9`（行为可辨）。

## 1. 真实点云 → support_map（修了 4 处实机缺陷）

| # | 缺陷 | 修法 |
|---|---|---|
| 1 | `elevation.extract_surfaces` 按 z 均值**精确相等**分带；实机每格均值都不同 ⇒ 连通分量恒 0 | 改为**种子锚定的 z 带聚类**（`z_band` 缺省＝`z_tol`） |
| 2 | `classify_surfaces` 的 z 带写死 `-0.06..1.05`（NIST 调参） | `build_map_from_points` 增加 `surface_z_min/surface_z_max`，缺省不变 |
| 3 | 家具顶面（0.42–0.99 m）被当支撑面 ⇒ 局部规划判「低天花板」 | 新增 `--obstacle-min-z`：带外被拒的面转成 `obstacles` 盒；支撑带收到 `≤0.15` |
| 4 | 起终点由最大面随手选，可能贴着家具 | `choose_start_goal` 增加障碍余量（0.55 m），沿起终点线回拉 |

复跑式：

    python3 -B ws/src/90_platform/v8_platform/framework/map_from_pcd.py \
      --pcd data/maps/real_lite3_20261103/lite3.pcd --map-id lite3_real --version 2 \
      --resolution 0.15 --voxel 0.03 --z-min -0.30 --z-max 2.50 \
      --surface-z-min -0.20 --surface-z-max 0.15 --obstacle-min-z 0.15 \
      --min-cells 4 --min-side 0.25 --min-area 0.50 --xy-gap 0.20 \
      --out data/maps/real_lite3_20261103/lite3_real_v2.json

现读：386,549 点 → 177,008（voxel .03）→ 41,210（水平面）→ **8 面（地面 m0 110.9 m²）＋350 障碍盒**。

### 1.1 ★ v4 追加（2026-11-03 真机静态窗后，旧 v2 文字不回改）★

v2 的三个实测问题：①障碍是整簇 AABB 且 z 从 -0.2 起，按半径 0.35 膨胀后占 76.4% 地图，机器人停机位被判碰撞；
②hdl 位姿 z=0.0 与地面 -0.045、低面 +0.035 之间只余 4.8 mm 到 `support_tol` 上限；
③面足迹只有 bbox，空角也冒充接触面。v4 的修法：

| 改动 | 实现 |
|---|---|
| 地面对齐（a） | `--z-align`：最低支撑面归零（本图 +0.04518），面/障碍同步平移；任务 z 随之为 0 |
| 近地碎片并入（d） | `--absorb-near-floor 0.08`：地面以上 ≤8 cm 的碎片并入地面（本图 m2/m11），≥8 cm 仍为台阶面 |
| 真实格集（c） | 面携带 `cells`（RLE，绝对格号＝floor(x/res)）→ map **schema v3**；`resolve`/`safe_ceiling`/轨迹检查器按格集判归属，空角不再命中；无格集的旧图回退 bbox |
| 支撑取下方（b） | 叠面时取机身下方最高的面（`select_support`），不再「恰好一个」拒绝 |
| 障碍细化（B） | `--obstacle-mode surfaces` 时被拒面按实格拆矩形（本图 350→2004 盒） |
| 全点云障碍层（D） | `--obstacle-mode full`：墙＋家具，0.20 m 格、0.4 m z 层、稀疏 z 桶剔除；带真实 z 区间（墙 0.1–2.3 m，桌面只占自己那层）。本图 **461 盒**；机器人高度处的占用从 76.4% 降到 69.3% |
| 起终点（a/c） | `choose_start_goal` 改为在实格上做 BFS，选有障碍余量、连通、≤`--goal-max` 的点 |
| 局部余量（E） | `local_native.yaml` `clearance_m 0.2 → 0.10`（0.35 外接圆＋10 cm 安全环；0.2 等于要求 1.1 m 宽通道） |

v4 复跑式：

    python3 -B ws/src/90_platform/v8_platform/framework/map_from_pcd.py \
      --pcd data/maps/real_lite3_20261103/lite3.pcd --map-id lite3_real --version 4 \
      --resolution 0.15 --voxel 0.03 --z-min -0.30 --z-max 2.50 \
      --surface-z-min -0.20 --surface-z-max 0.15 --obstacle-min-z 0.15 \
      --min-cells 4 --min-side 0.25 --min-area 0.50 --xy-gap 0.20 \
      --obstacle-mode full --obstacle-cell 0.20 --obstacle-min-points 5 \
      --obstacle-band "0.02,2.50" --z-align --absorb-near-floor 0.08 --goal-max 1.5 \
      --out data/maps/real_lite3_20261103/lite3_real_v4.json

现读：**6 面**（m2/m11 已并入地面；地面 z=0.0）＋ **461 障碍盒**；v4 任务 start (-0.075,-0.375,0)、
goal (-1.575,-0.375,0)（1.5 m，BFS 选点）。真机换停机位后只生成任务、不重建地图：

    python3 -B .../map_from_pcd.py --task-only --map data/maps/real_lite3_20261103/lite3_real_v4.json \
      --map-id lite3_real --version 4 --start-x <实测x> --start-y <实测y> --goal-max 1.5 --out <task.json>

## 2. 真机数据的规划读数（纯 Python）

    GLOBAL: ok  length 1.09 m  waypoints 9   （v1 无障碍时 5.93 m/36 点）
    LOCAL : ok  reason=arrived  expansions 16  checker ok

## 3. 命令链（红线已过）

    local3d(TwistStamped /v8/local/cmd_vel_3d，诊断用)
      └─ cmd_twist(Twist) → /cmd_vel_raw
           → safe_cmd_vel_mux（唯一 /cmd_vel 写者）
             → /cmd_vel → ros2qnx（仅 mode_navigation）

- 生产 mux 源表本来就有 `navigation: /cmd_vel_raw`，**未改 mux 配置**；
- `framework/safety.py` 静态闸现读 **SAFETY GATE ok**；
- `command_writers = {'/cmd_vel': 'mux', '/cmd_vel_raw': 'local'}`；app 不写 `/cmd_vel`。

### 3.1 速度上限（0.5 m/s）

- `local_native.yaml`：`max_speed_mps: 0.5`；
- `cmd_vel_mux.yaml`：`max_linear_mps: 0.5`、`max_angular_rps: 1.0`（mux 内最后一道软件钳位；
  既有 2D 规划上限 `max_vel_x=0.4`，故对 2D 无行为变化；3D 原先 0.6 会被收到 0.5）；
- 冒烟实测：`/cmd_vel_raw` 与 `/cmd_vel` 均 `|v| = 0.500`；
- 说明：`lite3-monitor.service` 是机器人侧 **root 属主的 systemd 单元**（`note/03` 只有元数据登记，
  仓内无正文，关机前未取）。本批按 0.5 m/s 实现；若该服务是分轴/不同单位，回来读到原件后再对齐。

### 3.1.1 机器人包络（《绝影 Lite3 探索版产品手册 V2.0.0》§1.4，第 7 页）

| 项 | 手册值 | 采用值（`local_native.yaml`） |
|---|---|---|
| 站立尺寸（长×宽×高） | 610×370×436 mm | 半径 0.35 m（与 2D 栈 `costmap_common.yaml:102 robot_radius: 0.35` 一致）、高 0.436 m |
| 趴地尺寸 | 680×370×145 mm | —（趴地不导航） |
| 整机重量 | 12.2 kg | — |
| **连续楼梯高度** | **15 cm** | `max_step_m: 0.15`、步态门限 `max_step_m: 0.15` |
| 斜坡坡度 | 40° | `max_slope_deg: 40.0` |

复跑读数（真机地图、半径 0.35/高 0.436）：全局 0.64 m/6 点、局部 `arrived`、检查器 0 违规。
终点余量随半径提高：`map_from_pcd --goal-margin 0.85`（半径 + 净空 + 0.3 缓冲）。

### 3.2 底盘守护与步态切换（默认关）

- `ros2qnx.cpp` 增加 `~chassis_guard_enable`（默认 **false**，关闭时行为与旧版一致）：
  心跳 `0x21040001`（≥2 Hz，独立线程）、退出 AI `0x2101052B`、移动模式 `0x21010D06`、
  步态切换（`0x21010407` 高踏步 / `0x21010307` 平地中速 / 其余五种）；
- 步态切换带速度门限 `~guard_gait_min_speed`（默认 0.10 m/s），超速拒绝并上报；
- 新增 `~guard_dry_run`：为 true 时只计数不发送，用于离线联调；
- 新增 `/v8/chassis/gait`（String）与 `/v8/chassis/status`（JSON）；
- 注意：参数必须用私有名（`_chassis_guard_enable:=true`），目标默认 `192.168.1.120:43893`。

决策节点 `framework/stair_gait_guard_node.py`（纯逻辑在 `gait_decision.py`）：

    /v8_support_map + /v8/3d/global/path + /v8/local/pose
      → stair_ahead（前视 1.5 m）→ 状态机（进入 1.0 m / 退出 1.5 m / 超高拒绝）
      → /v8/chassis/gait

联调读数：距台阶 1.0 m → `high_step`；路径清空 → `flat_mid`；7 枚状态机单测全过。

### 3.4 前方超声波保底（速度 ≤ 0.1 m/s）

- `qnx2ros` 新增 `/us_publisher/ultrasound`（`Float64MultiArray`，index 0＝前、1＝后）；
  旧话题 `/us_publisher/ultrasound_distance` 保留不动（它发的是 index 1＝后）。
- `framework/ultrasonic_guard_node.py`：前方 ≤0.3 m → **cap 0**（停车档）；≤0.8 m → cap 0.1；
  ≥1.2 m 恢复 0.5（迟滞）；`~fail_cap: true`（已置）＝超声数据过期即 0.1。
- 生产 mux 新增动态限速：读 `speed_cap_topic`，取 `min(静态上限, cap)`；
  `speed_cap_required: true`（已置）时 cap 过期即退到 `blind_cap_mps: 0.1`。
- 感知失明：`dynamic_obstacles_node ~blind_timeout_s: 2.0`（已置）＝点云断流 2 s 后**停发**
  障碍心跳 ⇒ 局部规划器超时停车（fail-closed）。

联调读数（前方值 → `/v8/safety/speed_cap` → `/cmd_vel`，`/cmd_vel_raw` 恒 0.5）：

```
front 0.5 m → cap 0.1 → /cmd_vel 0.1
front 3.5 m → cap 0.5 → /cmd_vel 0.5
front 0.2 m → cap 0.0 → /cmd_vel 0.0
无超声（真机档）→ cap 0.1 → /cmd_vel 0.1（容器冒烟现读：|v| = 0.100）
```

7 枚决策单测（迟滞、过期 fail-open/closed、非法量程）全过。

## 4. 容器内整图冒烟（viz 容器，未动真机）

    app_runner --app configs/lite3/lite3_real_3d.app.yaml --execute --no-gazebo --port 11551

    静态 TF map→base_link 在任务起点（v4 时 = (-0.075, -0.375, 0)）代替 hdl
    checks = {assembly_ready: True, map_configured: True, global_path: True,
              local_arrived: True, carrier_at_local_goal: False}
    v4 现读：path = 11 poses / 1.50 m；local_trajectory.reason = arrived（-1.575,-0.375,0）
    （v2 旧读 9 poses；两者都因静态 TF 不动而 carrier_at_local_goal=False）

`carrier_at_local_goal=False` 是预期的：冒烟用静态 TF，底盘不会动。整图 7 件全部起齐：
`carrier / map / perception / global / path_goal / local / mux`。

## 5. 本批新增/修改件

| 件 | 说明 |
|---|---|
| `framework/lite3_carrier_node.py` | 位姿适配器：TF 或 /odom → `/v8/local/pose` + `localization_ok`；不订阅任何命令话题 |
| `framework/obstacle_heartbeat_node.py` | 动态障碍心跳（默认空表）；局部规划器 fail-closed 需要它 |
| `manifest_carrier_lite3.json`（夹具＋生产） | `v8.fw.carrier_lite3` |
| `manifest_local_native_raw.json` | `v8.fw.local_native_raw`：多一条 Twist 输出 `/cmd_vel_raw` |
| `manifest_safety_mux_lite3.json` | `v8.fw.safety_mux_lite3`：生产 `safe_cmd_vel_mux`，唯一 `/cmd_vel` 写者 |
| `manifest_obstacle_heartbeat.json` | `v8.fw.obstacle_heartbeat` |
| `app_config.py` / `app_runner.py` | `backend: lite3`（不拉 Gazebo，传位姿参数） |
| `local_planner/ros_node.py` | 新增 `~twist_topic`（Twist 镜像，缺省关闭） |
| `local_planner/local_native.py` | 支持 map v2（合并地图障碍）；`support_tol_m`（缺省 1e-6，真机 0.05） |
| `validator/trajectory_checker.py` | 支持 v2 与障碍碰撞检查（`E_TRAJ_OBSTACLE`） |
| `configs/lite3/` | 生产 app、组合图、局部配置、本 README |
| `contracts/schema/map.v3.json` | 面可带 `cells`（RLE）；v1/v2 旧图仍走各自 schema |
| `representation/support_map.py` | `encode_cells/decode_cells/surface_cells/contains_xy/select_support`；有格集时归属判格，无则回退 bbox |
| `representation/elevation.py` | 提取面时保留真实格集（绝对格号＝floor(x/res)） |
| `regression/gazebo3d_mapping.py` | 合并时并格集；`z_align`/`absorb_near_floor_m`/`refined_surface_obstacles`/`full_cloud_obstacles`（含稀疏 z 桶剔除） |
| `local_planner/local_native.py` | 支撑取下方（b）、格集归属（c）、顶棚/机体扫掠按格集（c）、静态障碍栅格索引 |
| `validator/trajectory_checker.py` | 支撑/净空按格集判归属 |
| `planner/support_planner.py` | 起终点与支撑格按真实格集 |
| `framework/map_from_pcd.py` | v4 CLI（z-align/absorb/obstacle-mode/obstacle-band/goal-max）＋`--task-only` 现场生成任务 |

## 6. 尚未完成（真机回来前做不了或属 L2）

1. **机器人包络**：已按手册 §1.4 填入（半径 0.35 / 高 0.436 / 台阶 0.15 / 坡 40°），
   仍建议在真机上用实测复核一次（手册值 vs 实际装载）。
2. **动态障碍**：心跳目前发空表；真实感知（人或物）未接线。
3. **`mode_navigation` 已建**（2026-11-03 `df39f4d1`，`enable_motion` 缺省 false）；ros2qnx 只在
   `enable_motion:=true` 时起，运动仍属 L2。
4. **A3/A4/A5 仍 PARTIAL**：本批只证真机数据能跑通规划链，不改变签署状态。

## 7. 真机回来后的执行序（须 L2 批作业窗）

1. `mode_localize` 起 hdl，确认 `map→base_link` 与 `/odom` 频率；
2. 起本 app（`backend: lite3`），命令只到 `/cmd_vel_raw`，mux `hold` 保持 true —— 不动底盘；
3. 验收片 1：`/cmd_vel` 唯一写者＝`safe_cmd_vel_mux`、`ros2qnx` 零、位姿与 hdl 一致；
4. 放开 hold（L2），低速空场，验收 20 Hz 不中断、急停链路有效；
5. 补实测包络与动态感知后，才谈闭环导航。
