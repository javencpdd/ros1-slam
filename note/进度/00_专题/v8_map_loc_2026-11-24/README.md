# V8 建图＋定位仿真专项（2026-11-24，进行中）

> 目标：在隔离 Gazebo（master 11313）中用 **faster_lio 建图 + hdl_localization 定位** 替代 carrier 真值，
> 驱动既有 graph+support 导航闭环。本文件只记**已取得的运行证据**，未取得的一律标 NOT RUN / INVALID。

## 1. P1 传感器：PASS（实测）

| 项 | 读数 | 来源 |
| --- | --- | --- |
| `/livox/lidar`（livox CustomMsg，仿真激光） | **9.88 Hz** | `m1_gazebo.launch`（`lite3_planar.urdf.xacro` 的 `livox_mid360_plugin`） |
| `/rslidar_points`（PointCloud2，LIO 的 lid_topic） | **11.45 Hz** | `sim_adapter`（CustomMsg→PointCloud2） |
| `/livox/imu` | **200.4 Hz** | 同一 xacro 的 IMU 传感器（faster_lio imu_topic 契约） |
| TF | `map→odom` 父帧唯一、`tf_frame_assert PASS` | 启动日志 |

⇒ faster_lio 的输入契约（`lid_topic=/rslidar_points`、`imu_topic=/livox/imu`、`use_sim_time=true`）在仿真侧**成立**。

## 2. P2 建图：**INVALID（未取得可控产物）**

已做的：
- `faster_lio` 可执行（`run_mapping_online`）能起、能产出 `/Odometry`（x≈0.012, y≈0.023）；
- 用脚本驱动 `/cmd_vel` 原地转 + 前进约 40 s 采图。

失败点（明确）：
1. **PCD 保存路径参数未生效**：日志仍写 `/home/ros1-slam/data/maps/lite3.pcd`；
   检查运行二进制 `strings | grep "pcd_save/path"` = **0** ⇒ 我改的源码**没有被编进二进制**（`catkin_make -DCATKIN_WHITELIST_PACKAGES=faster_lio` 报 rc=0 但未真正重编该包）。
2. 因此本轮**没有**得到独立命名的 PCD，也未做 `map_from_pcd` 转换与覆盖率/漂移量化。

补充事实（不是本轮结论）：
- 容器内 `/home/ros1-slam/data/maps/` 已有历史建图产物（`mapped_m1.pcd` 1.2M、`mapped_nist.pcd` 5.0M、`sim_venue.pcd` 11M 等），
  说明**过去跑过仿真建图**；但这些产物不在仓库内，且没有同轮的可复现脚本与指标，不能当作本轮 PASS。

## 3. P3 定位：**NOT RUN**

- 需要先把 P2 的 PCD 落到可控路径并生成 support_map；
- 另有一条历史约束需先核实：`m2_hdl.launch` 头部注释写明「hdl 的仿真验证已被 M2-R2C 裁决终止转真机 G8」——
  若该裁决仍有效，则 hdl 在仿真中的验收口径需重新定义，不能直接沿用。

## 4. P4 导航闭环（定位输出替代真值）：**NOT RUN**

依赖 P2/P3。既有 graph+support 闭环（真值位姿）仍是 PASS 基线，未被破坏。

## 5. 下一步（按序）

1. 修构建：用与容器实际构建方式一致的方式重编 faster_lio（`catkin build` 或核对 whitelist 是否命中），
   以 `strings` 命中 `pcd_save/path` 为验收；
2. 重跑建图 → `/tmp/v8map/*.pcd` → `map_from_pcd` 生成 support_map；
3. 量化：点数/bbox/覆盖率（对 world 可达区域）、与真值轨迹的漂移；
4. 再接 hdl_localization（先核实历史裁决口径）→ 替换 `carrier.pose/localization` 驱动导航。

## 6. 边界

未伪造传感器、未固定 `localization_ok=true`、未绕过安全门控、未接触真机；
FAR 两组合仍 INVALID；VCC RED=39 与 formal 状态未动。

---

## 7. 第二轮：建图跑通（P2 部分 PASS）

### 7.1 构建问题（未解决，但已绕过）

| 事实 | 读数 |
| --- | --- |
| 主工作区 `catkin_make -DCATKIN_WHITELIST_PACKAGES=faster_lio` | **rc=1**：`tf/transform_broadcaster.h`、`ros/types.h` 找不到（cmake 未带 `/opt/ros/noetic` 前缀） |
| profile 工作区同名命令 | rc=0 但**未真正重编**该包（`strings` 命中 `pcd_save/path` = 0） |
| `rosrun` 实际解析 | `/opt/ros1-slam/ws/devel/lib/faster_lio/run_mapping_online`（Nov 21 旧二进制） |

⇒ 参数化改动**尚未生效**。为不阻塞验证，改用「**备份样张 → 用默认硬编码路径跑 → 取回 → 还原**」，
还原后 sha256 与备份一致（`047c7257…`），**未污染样张**。

### 7.2 一个真实的坑：`pcd_save/interval` 会清空缓冲

`interval=5` 时每次落盘都 `pcl_wait_save_->clear()` 且写**同一文件名** ⇒ 最终只剩最后一批点：
两次短跑分别只有 **2,574 / 1,752 点**，`map_from_pcd` 直接报 `no horizontal points after normal filter`。

改为 `interval=-1`（不中途落盘，只在退出时整图保存）后：

| 项 | 读数 |
| --- | --- |
| PCD | **4.3 MB / 139,698 点** |
| support_map | **19 个 surface**，res 0.1 m，origin (1.40, −4.50)，extent **41×65 格 = 4.1 × 6.5 m** |
| 产物 | `data/maps/generated/v8_flat_full_v1.json`（32 KB）＋ `_app.yaml` / `_task.json` |

### 7.3 精度：LIO 里程计 vs Gazebo 真值（短程）

| 项 | 值 |
| --- | --- |
| 真值终点 | (0.6227, −4.1390) |
| LIO `/Odometry` 终点 | (0.6279, −4.1199) |
| 终点偏差 | **≈ 0.020 m** |

⚠ 这是**单次终点**比对，不等于全程漂移；全程漂移需按轨迹逐点对齐后统计（本轮未做）。

### 7.4 覆盖率：**未达标**

覆盖只有 4.1 × 6.5 m（60 s 遥操作、单向直行 + 两次原地转）；而 `m1_world.world` 的可达区域远大于此
（对照既有 `nist_sensor_v2.json` 为 15 × 18 m）。⇒ 建图**能出图**，但**覆盖不足**，达不到"可导航地图"标准。

## 8. P3 定位：**INVALID（受既有裁决约束）**

`note/02_问题与坑.md`（M2-3 终态补记，M2-R2C 最终轮 2026-10-06）明确：

> hdl 仿真验证**终止**——/clock 门控 + --no-bond 落地后，manager 于 /clock 放行后 ~2 sim-s 内仍 **SIGSEGV exit -11**

即 hdl 在仿真中的验证已被**既有正式裁决终止**（转真机 G8）。按本轮授权「不擅自推翻既有正式裁决」：
P3 记 **INVALID**，不擅自用 faster_lio 里程计冒充地图重定位。

## 9. P4 导航闭环（估计位姿替代真值）：**NOT RUN**

依赖 P3。

## 10. 各阶段结论

| 阶段 | 结论 | 依据 |
| --- | --- | --- |
| P1 传感器 | **PASS** | 点云 11.45 Hz、IMU 200 Hz、TF 断言通过 |
| P2 建图 | **FAIL（部分达成）** | 能出 4.3 MB / 139,698 点 PCD 与 19 面 support_map；但覆盖仅 4.1×6.5 m，未达可导航标准；构建未生效靠备份绕过 |
| P3 定位 | **INVALID** | hdl 仿真验证已被 M2-R2C 裁决终止 |
| P4 导航闭环 | **NOT RUN** | 依赖 P3 |

---

## 11. 第三轮：构建与参数真正打通（P2 复跑）

### 11.1 三个真因，逐个击破

| # | 现象 | 根因 | 证据 |
| --- | --- | --- | --- |
| 1 | 改了源码但二进制不含参数 | `run_mapping_online` 是**薄可执行**，字面量在 `libfaster_lio.so`；且主工作区 `catkin_make` 缺 `/opt/ros/noetic/include` | `grep -ac pcd_save/path libfaster_lio.so` 由 0→1；补 `-DCMAKE_CXX_FLAGS=-I/opt/ros/noetic/include -I.../devel/include` 后 rc=0 |
| 2 | 设了 `pcd_save/path` 但仍写死路径 | 该参数是**私有**参数（`~pcd_save/...`），需 `rosparam set /faster_lio_online/pcd_save/path`，且**必须在节点 init 之前**设 | 先设后起：`pre_set=/tmp/v8map/out.pcd` |
| 3 | 退出时仍写死路径 | 只改了「中途落盘」分支；`LaserMapping::Finish()` 里还有一处硬编码 | 本次补上，重建后生效 |

### 11.2 结果：**路径生效、样张零污染**

| 项 | 读数 |
| --- | --- |
| PCD 输出 | **`/tmp/v8map/out.pcd`，7.0 MB / 226,592 点** |
| 样张 `lite3.pcd` | sha256 `047c7257…` **前后一致（未改动）** |
| support_map | 19 surfaces（raw 749 / consolidated 403），res 0.1，origin (1.5, −4.4) |
| 覆盖 | **3.90 m × 7.10 m** |

### 11.3 P2 结论：仍 **FAIL**（阻塞项已消除，覆盖不足）

- 构建、参数、样张保护**三项都已修好**（可复现：`map_run9.sh`）；
- 但覆盖只有 3.9×7.1 m，远小于可导航需求（对照 `nist_sensor_v2.json` 15×18 m）⇒ 地图**不可用于导航验收**；
- 轨迹误差本轮未做全程统计（上一轮终点比对 0.020 m 仅单点）。

## 12. P3 定位：复现尝试 **INVALID（未取得定位结果；历史裁决保留）**

| 步 | 结果 |
| --- | --- |
| 起 hdl 冒烟（`hdl_m1_tracking_smoke.launch`，底图用本轮 PCD） | 节点在跑（3 节点 / 2 进程），**40 s 内未出现 SIGSEGV** |
| 输出 | `/hdl_odom`、`/aligned_points`、`/status` 话题**存在但无消息** ⇒ 未收敛/未初始化 |
| 历史裁决 | `note/02_问题与坑.md`（M2-R2C）仍有效：hdl 仿真验证终止（~2 sim-s 内 SIGSEGV exit -11） |

⇒ 本轮**没有**推翻裁决，也**没有**用 LIO 里程计顶替地图重定位。P3 记 **INVALID**（本次未复现崩溃，但也未取得定位输出）。

## 13. P4 导航闭环（算法位姿替代真值）：**NOT RUN**

依赖 P3。

## 14. 四阶段结论（本轮）

| 阶段 | 结论 | 关键读数 |
| --- | --- | --- |
| P1 传感器 | **PASS** | 点云 11.45 Hz / IMU 200 Hz / TF 断言通过 |
| P2 建图 | **FAIL** | 226,592 点、19 面、覆盖 3.9×7.1 m；构建与参数已修好、样张未污染 |
| P3 定位 | **INVALID** | hdl 49 s 内未崩溃但无定位输出；历史裁决保留 |
| P4 导航闭环 | **NOT RUN** | 依赖 P3 |

