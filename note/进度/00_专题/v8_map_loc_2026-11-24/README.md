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

---

## 15. 第四轮：3D 地图确认、路径自检、覆盖率与全程误差

### 15.1 地图是 **3D**（不是 2D 占据栅格）

以 `v8_flat_r4_v1.json` 为例：

| 字段 | 值 | 说明 |
| --- | --- | --- |
| `kind` | `support_map` | 支撑面地图（3D 规划器消费） |
| `surfaces[].representation` | **`elevation_grid`** | 高度栅格，不是 occupancy |
| `surfaces[].z_reference` | 0.01 / 0.02 / 0.12 / 0.19 / 0.27 / 0.34 … | **每面独立高度层** |
| `surfaces[].thickness_m` | 0.05 | 面厚度 |
| `surfaces` 数 | **30**（raw 813 → consolidated） | 多层/多段支撑面 |
| 源 PCD 字段 | `x y z intensity normal_x/y/z curvature` | 完整 3D 点云 |

⇒ 产物是**分层高程 3D 地图**；`map_from_pcd` 输出的 2D 网格只是每个面的 `cells` 索引，不代表整体是 2D。

### 15.2 输出路径自检（已强制，且失败即退出）

`map_run10.sh` 开头做四道检查，任一不满足直接 `exit`：

1. 必须显式传输出路径（否则 `exit 2`）；
2. 路径等于样张 `lite3.pcd` 或其同名（`exit 3`）；
3. 输出目录与样张同目录（`exit 4`）；
4. 目录不可写（`exit 5`）。

本轮实测：`路径自检通过: /tmp/v8map/run11.pcd`，运行结束 `saved to /tmp/v8map/run11.pcd`，
样张 `lite3.pcd` **sha 未变 ✓** —— 不再依赖备份恢复。

### 15.3 建图读数（更长路线：直行 + 两次 90° 转 + 再直行）

| 项 | 读数 |
| --- | --- |
| PCD | **8.6 MB / 279,069 点**（`/tmp/v8map/run11.pcd`） |
| support_map | **30 面**（raw 813），res 0.1 m |
| 覆盖 | **4.10 m × 8.90 m** |
| 行程 | GT 终点 (3.829, 0.000)，约 **4.3 m** 实际行进 |

### 15.4 全程轨迹误差（LIO vs Gazebo 真值）

| 指标 | 值 |
| --- | --- |
| 采样点 | **5**（受探针逐点 `rostopic echo` 速率限制，非高频） |
| 终点 | GT (3.829, 0.000) vs LIO (3.979, 0.023) |
| **RMSE** | **0.105 m** |
| 最大分量误差 | \|ex\| 0.187 m，\|ey\| 0.024 m |
| 相对行程漂移 | ≈ **3.9 %**（0.15 m / 3.83 m） |

⚠ 采样仅 5 点，只能算"量级证据"，不是严格的全程统计；需高频采集（直接订阅并按时间戳存盘）才能给出可信 RMSE。

### 15.5 P2 结论：**PASS（按本轮导航路线口径）**

覆盖 4.10×8.90 m 足以包含本轮 4.3 m 的行驶路线，且地图为 3D 分层、点数与面数正常；
按用户"以实际导航路线评估覆盖、不强制对齐旧图尺寸"的口径 ⇒ **PASS**。
遗留：全程 RMSE 采样过稀（5 点）。

## 16. P3 定位：再次尝试仍 **INVALID**

| 项 | 结果 |
| --- | --- |
| 节点 | `velodyne_nodelet_manager` + `globalmap_server_nodelet` + `hdl_localization_nodelet` 均在跑 |
| 崩溃 | **0 次**（`grep -ic SIGSEGV/exit -11/died` = 0） |
| `/hdl_odom` | **no new messages**（下发 `/initialpose` 后 12 s 仍无输出） |
| 日志 | roslaunch 日志中无 map/load/error 关键字可读（nodelet 输出未落盘） |

⇒ 本轮**未复现 SIGSEGV**，但也**未取得定位输出**；`M2-R2C` 历史裁决保留不动。
下一步需要：核对 hdl 的 `map_pcd` 加载是否成功（PCD 含 intensity/normal 字段可能被拒）、
`/imu/data` 与 TF（`map→base_link`）是否齐备、以及是否需要 `/gps/fix` 或全局重定位服务触发。

## 17. P4 导航闭环：**NOT RUN**（依赖 P3）

## 18. 四阶段结论（第四轮）

| 阶段 | 结论 |
| --- | --- |
| P1 传感器 | **PASS**（未退化：点云 11.45 Hz、IMU 200 Hz、TF 断言通过） |
| P2 建图 | **PASS（本轮路线口径）**：279,069 点 / 30 面 / 4.10×8.90 m / RMSE 0.105 m（5 点采样） |
| P3 定位 | **INVALID**：无崩溃、无定位输出；历史裁决保留 |
| P4 导航闭环 | **NOT RUN** |

---

## 19. P3 深挖：hdl "加载了但没工作"（第五轮）

### 19.1 四条硬证据

| 观测 | 读数 | 含义 |
| --- | --- | --- |
| `/hdl_localization_nodelet` 订阅 | **只有 `/clock`** | 没有订点云、没有订 IMU、没有 TF 监听动作 |
| `/globalmap_server_nodelet` 发布 | **只有 `/rosout`** | **地图没加载**，`/map` 无数据 |
| `/rslidar_points` 订阅者 | **None** | 与第 1 条互证：hdl 从未订阅激光 |
| 两个 nodelet 的 stdout 日志 | **0 字节** | 既无"加载成功"横幅，也无错误输出 |

旁证：插件描述齐全（`nodelet_plugins.xml` + `package.xml` 的 `<export>`），
`libhdl_localization_nodelet.so` 里有 **98 个 nodelet 相关符号**，文件权限正常（`-rw-r--r--`，属主 ros1-slam）。

### 19.2 因果链（当前最合理解释）

`globalmap_server` 的 `onInit` 在加载 `globalmap_pcd` 阶段没有走完 → 不发布 `/map`
→ `hdl_localization` 等 `/map` 才订阅激光 → 于是 `/rslidar_points` 无订阅者、`/hdl_odom` 无输出。
**不是崩溃**：本轮 `grep -ic "SIGSEGV|exit -11|died"` = **0**。

### 19.3 本轮结论：P3 = **INVALID**

- 未取得任何定位输出（`/hdl_odom` no new messages）；
- 未复现历史裁决里的 SIGSEGV；
- `M2-R2C` 裁决**原样保留**，未改写；
- 未用 faster_lio 里程计顶替"地图重定位"。

### 19.4 下一步三个精确探针（按序）

1. **前台手工加载**：`rosrun nodelet nodelet load hdl_localization/GlobalmapServerNodelet <mgr>`
   在终端直看返回（roslaunch 把它的 stdout 吞了，日志 0 字节）；
2. **验 PCD 兼容性**：hdl 侧用 `pcl::io::loadPCDFile<PointT>`（`PointT=pcl::PointXYZ`）；
   我们的 PCD 含 `intensity/normal_x/y/z/curvature`，先用 `pcl_convert_pcd_ascii_binary` 或
   `map_from_pcd` 生成纯 XYZ 版 PCD 再试（**不改算法，只改输入格式**）；
3. **验 `/map` 是否出现**：若 map server 正常，应看到 `/map` 有 latched 数据，再谈 hdl 订阅。

## 20. 四阶段最终（第五轮）

| 阶段 | 结论 | 依据 |
| --- | --- | --- |
| P1 传感器 | **PASS** | 点云 11.45 Hz / IMU 200 Hz / TF 断言通过 |
| P2 建图 | **PASS（路线口径）** | 279,069 点、30 面、4.10×8.90 m、RMSE 0.105 m（5 点采样） |
| P3 定位 | **INVALID** | hdl 未订阅点云、map 未加载、无 `/hdl_odom`；无崩溃；裁决保留 |
| P4 导航闭环 | **NOT RUN** | 依赖 P3 |


---

## 21. P3 深挖第二轮：抓到决定性错误，仍未出 `/map`（第六轮）

### 21.1 前台手工加载，拿到真实报错

`rosrun nodelet nodelet load hdl_localization/GlobalmapServerNodelet mgr` 后，**manager 的 stdout** 出现：

```
[pcl::PCDReader::read] Could not find file ''.
[pcl::NormalDistributionsTransform::setInputTarget] Invalid or empty point cloud dataset given!
[pcl::VoxelGridCovariance::applyFilter] No input dataset given!
```

⇒ **`globalmap_pcd` 被读成空串**，PCD 根本没打开 ⇒ 不发布 `/map` ⇒ `hdl_localization` 等 `/map`
⇒ 不订阅 `/rslidar_points`（实测订阅者 None）⇒ 没有 `/hdl_odom`。

这条把 P3 的根因从"崩不崩"缩小到**参数解析**，与点云格式、TF、时间戳无关。

### 21.2 为什么 roslaunch 里参数没用

launch 是把 `<param name="globalmap_pcd">` 写在 **`nodelet load` 那个节点**下；
而 nodelet 内部节点的私有命名空间由 manager 决定，**不是 load 节点的名字**。
实测把参数同时写到 `/globalmap_pcd`、`/gms/globalmap_pcd`、`/mgr/gms/globalmap_pcd`、
`/globalmap_server_nodelet/globalmap_pcd` 四处后再加载，**30 s 内仍无 `/map`**，mgr 日志也没有新增行
⇒ 说明当前流程在 `onInit` 更早处就停了（或加载请求未被 manager 处理），需更细的调试手段。

### 21.3 本轮 P3 结论：**INVALID（根因已缩小到参数/命名空间）**

- 不是 SIGSEGV（0 次）；
- 不是点云/TF/时间戳（hdl 根本没走到订阅）；
- 决定性错误是 **PCD 路径为空**；补参后仍未发布 `/map`，需继续用 `rosconsole` 或 gdb 看 `onInit` 停在哪。

### 21.4 继续所需（下轮，不需要新授权）

1. `export ROSCONSOLE_CONFIG_FILE` 提高 nodelet 日志级别，或直接 `gdb -p <manager>` 看卡点；
2. 确认 `nodelet load` 的返回码（当前 stdout 0 字节说明 CLI 没打印成功横幅，可能仍在等 manager 的 service）；
3. 若参数路径确认走不通，可改用 **非 nodelet 的可执行**（hdl_localization 源码里是否有 standalone app）绕开 nodelet 命名空间问题。

---

## 22. P3 收官：定位链查清，并**复现了 M2-R2C 的 SIGSEGV**（第七轮）

### 22.1 三个被误判的点，逐个纠正

| 我之前的判断 | 事实 | 依据（源码） |
| --- | --- | --- |
| 地图发到 `/map` | map server 发的是 **`/globalmap`**（PointCloud2, latched） | `globalmap_server_nodelet.cpp:36` `nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 5, true)` |
| hdl 订 `/rslidar_points` | hdl **硬编码** `/velodyne_points` | `hdl_localization_nodelet.cpp:63` `mt_nh.subscribe("/velodyne_points", 5, ...)` |
| 参数写在 launch 的 load 节点下即可 | nodelet 的私有 ns 是**自己的 `__name`**；必须 `rosparam set /<name>/globalmap_pcd` 且**在 load 之前** | 实测：这样设后 `/globalmap` 立刻出现 |

### 22.2 补齐后链路确实通了

- `rosrun topic_tools relay /rslidar_points /velodyne_points` → `/velodyne_points` 出现订阅者（hdl 侧）；
- `rosparam set /gms/globalmap_pcd <pcd>` + 在 `/gms` 命名空间下发参 → **`/globalmap` 出现**（`rostopic list` 命中 1）；
- 至此 map→hdl 的地图链、点云链**都成立**。

### 22.3 最后一跳：**manager SIGSEGV（复现历史裁决）**

加载 `HdlLocalizationNodelet` 后，脚本报：

```
/tmp/hdl_ok.sh: line 36: 213553 Segmentation fault   setsid rosrun nodelet nodelet manager __name:=mgr
```

即 **nodelet manager 进程崩溃**（退出信号 11），与 `note/02_问题与坑.md` 的
「M2-R2C 最终轮 2026-10-06：hdl 仿真验证终止——/clock 门控 + --no-bond 落地后，manager 于 /clock 放行后
~2 sim-s 内仍 SIGSEGV exit -11」**完全一致**。

⇒ 崩溃发生在 **hdl 的 NDT_OMP 匹配阶段**（与点云/地图/参数都无关：那时地图已出、点云已订），
这是**算法/构建层面的第三方问题**，不是配置问题。

### 22.4 P3 最终结论：**INVALID（崩溃已复现，裁决得到印证）**

- 定位**未取得输出**（`/hdl_odom` no new messages）；
- 崩溃**已复现**（manager SIGSEGV），与历史裁决同因同象；
- **不改写裁决**、**不换 LIO 顶替**、**不降门控**；
- 若要在仿真里推进地图重定位，需要第三方 hdl 的 NDT 在 Gazebo 仿真时钟/点云下的稳定性修复
  （属算法层，超出本轮"最小修复"边界，且会动第三方实现）。

## 23. 四阶段最终结论（本轮收口）

| 阶段 | 结论 | 关键证据 |
| --- | --- | --- |
| P1 传感器 | **PASS** | 点云 11.45 Hz、IMU 200 Hz、TF 断言通过 |
| P2 建图 | **PASS（路线口径）** | 279,069 点 / 30 面 / 4.10×8.90 m / RMSE 0.105 m；路径自检生效、样张零污染 |
| P3 定位 | **INVALID** | 链路查清：`/velodyne_points` 桥接 + `/globalmap` 命名空间；随后 **manager SIGSEGV 复现**（裁决同因） |
| P4 导航闭环 | **NOT RUN** | 依赖 P3 的定位输出 |

---

## 24. P3 最后一试：去零点适配器（未救回，崩溃稳定复现）

### 24.1 新证据：点云 72.8% 是 (0,0,0) 填充点

`nan_chk.py` 对 `/rslidar_points` 抽样 20,000 点：

```
points=20000 checked=20000 NaN=0 zero=14555 (72.8%)
```

livox 仿真输出**定长 20000 点**，未命中的射线用 `(0,0,0)` 补齐。
这类退化输入（上万条原点）对 NDT 的目标/源点云是典型的崩因。

### 24.2 写了适配器仍崩

新增 `scripts/verify/cloud_denull.py`：按点拷贝、丢弃全零/非有限点，保持字段布局不变（**不改算法**）。
用它替换裸 relay 后再起 hdl：

```
/tmp/hdl_final.sh: line 37: 217532 Segmentation fault  setsid rosrun nodelet nodelet manager __name:=mgr
```

**manager 再次 SIGSEGV**（第二次稳定复现），`/hdl_odom` 仍 no new messages。

### 24.3 结论

- 崩溃**不随去零点而消失** ⇒ 真正诱因在 hdl 的 NDT_OMP 与 Gazebo 仿真时钟/点云组合，
  属**第三方算法层**；
- 按 `M2-R2C` 裁决不改写、不换 LIO 顶替、不动第三方实现；
- **P3 = INVALID（崩溃稳定复现）**，**P4 = NOT RUN**。

### 24.4 如果要继续，只剩这三条（都需要你点头）

| 选项 | 内容 | 代价 |
| --- | --- | --- |
| A | 给 hdl 打补丁：NDT 输入前强制 trim/去 NaN + 限制点数上限，并在崩溃点加日志 | 动第三方算法实现 |
| B | 换定位方案（如 `hdl_global_localization` 的 BBS 前端 + 自研匹配），重开裁决 | 重新设计，超出本轮 |
| C | 只保留 P1/P2 成果，P3/P4 作为"受既有裁决阻断"长期挂起 | 无代码风险，但导航闭环无法完成 |

---

## 25. P3 按 A 方案执行：加日志 + 两道护栏，崩溃仍未消除（第八轮）

### 25.1 为什么没用 gdb

容器内 **无 gdb**（只有 `catchsegv`），且 `ulimit -c = 0`、`core_pattern = core` ⇒ **拿不到 core**。
因此改用"源码内打点 + 护栏"的方式定位崩溃点（补丁可回退、行为可复现）。

### 25.2 补丁（三处，均在第三方 `hdl_localization_nodelet.cpp`）

1. `globalmap_callback`：target < 100 点时拒绝 `setInputTarget`，并打印实际点数；
2. `points_callback`：打印 `raw/filtered` 点数；`filtered < 30` 时跳过配准；
3. `align` 前后各打一行（`align begin: target=… source=…` / `align done: … points`）。

### 25.3 打点结果：**第一次配准是成功的**

rosout 实测（`/opt/ros1-slam/logs/latest/*.log`）：

```
globalmap received!
scan: raw=5439 filtered=…            (231 行)
align begin: target=… source=…       (287 行)
align done: 474 points               (289 行)   ← 第一次 align 成功
```

之后**再次 SIGSEGV**（manager 进程，exit -11）。⇒ 崩溃**不在首帧配准**，也不是"target 为空"。

### 25.4 第二道补丁：回调互斥（怀疑多线程 spinner 数据竞争）

nodelet 用 `mt_nh`（多线程），`voxelgrid`/`pose_estimator` 都是成员状态，加了
`std::lock_guard<std::mutex> callback_mutex_` 串行化整个 `points_callback`。

**结果：仍然 SIGSEGV**（第三次复现）⇒ **不是简单的回调并发竞争**。

### 25.5 本轮 P3 结论：**INVALID（补丁未消除崩溃，缺调试器取栈）**

| 项 | 结果 |
| --- | --- |
| 地图链 | ✅ `/globalmap` 正常（19 万+ 点） |
| 点云链 | ✅ `/velodyne_points` 有订阅者，`raw=5439` 到帧 |
| 首帧配准 | ✅ `align done: 474 points` |
| 稳定性 | ❌ 首帧后 manager SIGSEGV（三次复现：裸跑 / 去零点 / 加互斥） |
| 定位输出 | ❌ `/hdl_odom` 无消息 |

**下一步必须换手段**（不是再猜）：

1. **装 gdb**（`apt-get install gdb`，需网络授权）→ `gdb -p <manager>` 或 core 分析，直接取崩栈；
2. 或把 hdl 编译成 **Debug + ASAN**（`-fsanitize=address -g`）→ 让崩溃自带调用栈与越界报告；
3. 二者都需要"允许安装调试器/改编译选项"，属环境级改动，**等你点头**。

## 26. 四阶段最终（第八轮）

| 阶段 | 结论 |
| --- | --- |
| P1 传感器 | **PASS** |
| P2 建图 | **PASS（路线口径）**（279,069 点 / 30 面 / 4.10×8.90 m / RMSE 0.105 m） |
| P3 定位 | **INVALID**（三道补丁后仍崩；无 gdb 无法取栈） |
| P4 导航 | **NOT RUN** |

---

## 27. ASAN 路线执行：未取到报告，改动已回退（第九轮）

### 27.1 构建（独立目录，不污染 devel）

CMake 配置本身缺 `/opt/ros/noetic/include`，`catkin_make` 直接编不过 ⇒ 改为**手工编译**：
从 `build/.../CMakeFiles/hdl_localization_nodelet.dir/{flags.make,link.txt}` 提取参数，
在 `/tmp/asan` 用 `-fsanitize=address -g -O1 -fno-omit-frame-pointer` 重编两个 .o 并链接：

| 产物 | 大小 |
| --- | --- |
| `pose_estimator.cpp.o`（ASAN） | 49 MB |
| `hdl_localization_nodelet.cpp.o`（ASAN） | 48 MB |
| `libasan_hdl.so`（ASAN，soname 仍为 libhdl_localization_nodelet.so） | **50 MB** |

踩到两个编译坑（已解决）：`ROS_PACKAGE_NAME` 必须传**带引号的字符串宏**（shlex 会吃掉引号）；
链接清单要从 `link.txt` 里剔除两个 `.o`。

### 27.2 运行：用 ASAN 库 + LD_PRELOAD

`devel/lib/libhdl_localization_nodelet.so` 先备份再替换为 ASAN 版，
`LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libasan.so.4`（本机只有 .so.4）启动 manager。

| 观测 | 结果 |
| --- | --- |
| 节点 | mgr / gms / hdl 三个都在（未崩） |
| `/globalmap` | 有（1） |
| `/hdl_odom` | **仍无消息** |
| ASAN 报告 | **无**（`ASAN_OPTIONS=log_path=/tmp/v8map/asan` 未产出文件） |

⇒ 在 ASAN 版本下**崩溃没有复现**，同时**也没有定位输出**；两种可能：
①ASAN 的分配器/时序掩盖了原崩溃；②20 s 窗口太短、或触发条件（连续多帧 + 地图更新并发）未满足。

### 27.3 回退（已做，重要）

- `devel/lib/libhdl_localization_nodelet.so` 曾被我替换为 ASAN 版（50 MB），
  已删除并 `catkin_make` 重建：现为 **704 KB、`grep -c fsanitize` = 0**，正式二进制未被污染；
- ASAN 构建产物只在 `/tmp/asan`，源码补丁独立可 `git revert`。

### 27.4 P3 本轮结论：**仍 INVALID**

- 崩溃在**普通构建**下稳定复现（三次）；
- ASAN 构建下不复现、也未出报告 ⇒ **拿不到根因栈**；
- 结论：要定位必须 **gdb 附加/分析 core**（容器无 gdb、`ulimit -c=0`），或写更精确的复现脚本（长时间、含地图更新）。

## 28. 四阶段最终（第九轮）

| 阶段 | 结论 | 依据 |
| --- | --- | --- |
| P1 传感器 | **PASS** | 点云 11.45 Hz / IMU 200 Hz / TF 断言 |
| P2 建图 | **PASS（路线口径）** | 279,069 点 / 30 面 / 4.10×8.90 m / RMSE 0.105 m |
| P3 定位 | **INVALID** | 普通构建三次崩；ASAN 不复现且无报告；`/hdl_odom` 无输出 |
| P4 导航 | **NOT RUN** | 依赖 P3 |

---

## 29. ASAN 120 s 复测：库确实加载，但回调从未进入（判定 INVALID，非 PASS）

### 29.1 两次 120 s 试验的读数

| 项 | 第 1 次（未设 use_sim_time） | 第 2 次（补 use_sim_time） |
| --- | --- | --- |
| 载入的库 | 50 MB（ASAN） | 50 MB（ASAN） |
| **libasan 是否真加载** | `maps` 命中 **5** | `maps` 命中 **5** |
| manager 存活 | ✅（120 s 未崩） | ✅（120 s 未崩） |
| `scan: raw` 日志条数 | **0** | **0** |
| `align done` 日志条数 | **0** | **0** |
| `/hdl_odom` 发布器 | 存在（pubs=1） | 存在（pubs=1） |
| `/hdl_odom` 消息 | 0 | 0 |
| ASAN 报告文件 | 0 | 0 |

### 29.2 结论：**这次"没崩"不构成证据**

`scan` 与 `align` 计数都是 **0** ⇒ **hdl 的 `points_callback` 从未被调用**，配准根本没发生。
没有进入被测代码路径的"不崩"不能当 PASS，只能判 **INVALID（复现条件未达成）**。

旁证：`/velodyne_points` 当时有 **1 个发布者、2 个订阅者、hz ≥1**，数据在流；
`timeout lidar / Disgarding` 日志计数为 **0** ⇒ 不是被时间戳超时丢弃。

### 29.3 回退（这次踩到一个坑，已用重建修好）

`asan_run120.sh` 每轮执行 `cp -f $LIB $LIB.normal` 再覆盖 `$LIB` ⇒ **第二轮把 ASAN 版存成了"正式备份"**，
于是"还原"后仍是 50 MB 的 ASAN 库（`sanitize=1`）。

**正确回退**：`rm -f` 目标 .so **与** 对应 .o，再 `catkin_make` 强制重编 ⇒ 现为
**704 KB、`grep -c fsanitize` = 0**，正式二进制已确认干净。

### 29.4 P3 结论：**INVALID（ASAN 未取到根因栈，条件未达成）**

ASAN 路线能编、能加载、能长跑，但**没跑到崩点**，因此没有报告。
要继续需要：**让 hdl 真正消费点云**（先解决"订阅在、回调不进"），或者换 **gdb/core** 直接看崩栈。

## 30. 四阶段最终（第十轮）

| 阶段 | 结论 | 依据 |
| --- | --- | --- |
| P1 传感器 | **PASS** | 点云 11.45 Hz / IMU 200 Hz / TF 断言 |
| P2 建图 | **PASS（路线口径）** | 279,069 点 / 30 面 / 4.10×8.90 m / RMSE 0.105 m |
| P3 定位 | **INVALID** | 普通构建三次崩；ASAN 120 s 不崩但回调 0 次（未跑到崩点）；`/hdl_odom` 无消息 |
| P4 导航 | **NOT RUN** | 依赖 P3 |

## 31. P3 收官（第十一轮）：先修四处判据缺陷，再抓到真根因

### 31.1 前几轮的判定建立在错误的读数上

「ASAN 120 s 不崩、回调 0 次」这个结论，实际由四个判据缺陷叠加而成，逐个纠正：

| 缺陷 | 事实 | 判定依据 |
| --- | --- | --- |
| 查 `/mgr` 的 Subscriptions | `/mgr` 是 manager 进程，自身永远 `Subscriptions: None`；nodelet 实例另外注册 | 同一时刻 maps 里 `.so` 命中 0，才说明连库都没打开 |
| 查 `/opt/ros1-slam/logs/latest/` | 那是 roslaunch 的 run 目录；`rosrun` 起的 manager 不写那里，`NODELET_INFO` 打在 manager 进程 stdout | 同一轮 manager.log 有 154 条 `scan: raw` |
| `timeout 8 rostopic hz \| grep -q` 判有无数据 | `rostopic hz` 不会自己退出，timeout 必然以 124 杀掉它，脚本开着 `pipefail` ⇒ 整管道返回 124 ⇒ `if` 永远判假 | Gazebo 明明 10 Hz 在发点云，脚本却卡在 [1] 等 450 秒 |
| `LD_LIBRARY_PATH` 前置注入调试库 | class_loader 用 rospack 拼**绝对路径** `devel/lib/...`，前置对它无效 | libSegFault 的 maps 里始终是 `devel/lib//libhdl_localization_nodelet.so`，`.dbg` 命中 0 |

第四条尤其致命：用它算出的「调试版回溯行号」是假结论（拿 dbg 偏移去解正式库的地址）。

### 31.2 ASAN 注入被判死：不是掩盖 bug，是根本没加载

`MODE=asan` 时 manager 进程出现 `State: R`、**`Threads: 1`**、`wchan: 0`；
`rosnode ping /mgr` connection refused，`rosservice call /mgr/list` 无法通信；
`nodelet load` 静默挂起 40 s 零输出，`hdl.log` 0 字节。
⇒ 把 libasan 预加载进**未用 ASAN 编译**的 nodelet manager，会让它单线程自旋、XMLRPC 完全不响应，
load 请求从未被处理，`.so` 两份都没打开。所以「ASAN 120 s 不崩」不是 ASAN 掩盖了 bug，
而是**什么都没跑起来**。

### 31.3 换 catchsegv + 预加载：一次拿到真现场

- 注入改为 `LD_PRELOAD=<libstdbuf>:devel/.dbg/libhdl_localization_nodelet.so`：
  两份 soname 都是 `libhdl_localization_nodelet.so`，dlopen 按 soname 去重 ⇒ class_loader 再
  dlopen 绝对路径时复用已加载的调试版。**不覆盖 devel/lib，也不改插件 XML**。
- 外层套 `catchsegv`（libSegFault），stdout/stderr 行缓冲，崩溃前的 `NODELET_INFO` 全部落盘。
- 闸门前置：manager 起来后立刻 `grep /proc/<pid>/maps`，`preload_dbg_in_maps=5` 证明注入命中，
  **在崩溃之前**就排除「没注入」这个假象。

于是拿到完整现场：`align done: N points` 之后第一次 `publish_odometry` 就崩：

```
nodelet: boost/smart_ptr/shared_ptr.hpp:734:
shared_ptr<T>::operator->() [with T = const nav_msgs::Odometry_<...>]:
Assertion `px != 0' failed.  →  Aborted
```

### 31.4 根因：`leg_odom2_msg` 空 shared_ptr 解引用

`hdl_localization_nodelet.cpp` 原第 477 行：

```cpp
odom.twist = leg_odom2_msg->twist;
```

`leg_odom2_msg` 是 `nav_msgs::OdometryConstPtr`，只在 `/leg_odom2` 回调里赋值；仿真（以及任何
没有腿部里程计的载具）该话题没有 publisher ⇒ 它永远为空。

- 正式库带 `-DNDEBUG`，boost 的 `BOOST_ASSERT(px != 0)` 被编译掉 ⇒ 裸解引用 NULL ⇒ **SIGSEGV**；
- 调试库断言生效 ⇒ **Aborted**。

两种现象同一根因 —— 这就是历史裁决 M2-R2C 里反复复现的那个「偶发段错误」。

### 31.5 最小修复

保持语义不变：有腿部里程计就照旧填 twist，没有就不填（保持默认零值）。

```cpp
if (leg_odom2_msg) {
  odom.twist = leg_odom2_msg->twist;
}
```

不动任何算法参数、不动安全门控。

### 31.6 验证读数（MODE=normal，修复后）

| 项 | 读数 |
| --- | --- |
| 加载 | `load_result=instance_at=1s`；`maps_live=6`（正式库 sha `00f975bdd53711f7`） |
| 回调 | `scan: raw` 154 次 / `align begin` 154 次 / `align done` 154 次 |
| 崩溃 | `crash_at=none`（30 s 观察期）；ASAN 报告 0 份 |
| 丢帧 | `lidar_timeout_drops=0`（`lidar_timeout_ms` 200→500，仅放宽不丢帧，不改配准参数） |
| 输出 | `/hdl_odom` **8.407 Hz** |

### 31.7 定位误差（对 Gazebo 真值 `/sim/odom_gt`）

时间戳配对（|Δt| ≤ 50 ms）共 108 对：

| 指标 | mean | median | max |
| --- | --- | --- | --- |
| 绝对 XY（m） | 0.0265 | 0.0266 | 0.0452 |
| 相对 XY（m，去常数偏置后） | 0.0117 | 0.0117 | 0.0292 |
| 偏航（rad） | 0.0045 | 0.0041 | 0.0130 |

常数偏置 `(0.0162, -0.0011) m` 是 map 系与 odom 系原点之差。

### 31.8 两个顺带纠正的配置事实

- `robot_odom_frame_id` 必须＝`odom`：`publish_odometry` 按它选边。默认值 `robot_odom` 在 Gazebo 的
  TF 树里没有这条边，会走 else 分支发 `map→base_link`，与 Gazebo 的 `odom→base_link` 撞成双父。
- 真定位接入时 `m1_gazebo.launch` 必须 `map_to_odom:=false`（launch 注释本就这么要求，之前没照做）。

### 31.9 复现用件（均已入仓）

- `scripts/verify/asan_hdl_build.sh`：`MODE=dbg|asan` 两档构建，产物落 `devel/.dbg` / `devel/.asan`，不碰 `devel/lib`。
- `scripts/verify/asan_hdl_repro.sh`：`MODE=normal|dbg|asan` 分级复现，十道闸门，stdout 全部落 evidence。
- `scripts/verify/hdl_pose_error.py`：`/hdl_odom` 对 `/sim/odom_gt` 的误差探针（只读）。

## 32. 四阶段最终（第十一轮）

| 阶段 | 结论 | 依据 |
| --- | --- | --- |
| P1 传感器 | **PASS** | 点云 10.1 Hz / `/clock` 1000 Hz |
| P2 建图 | **PASS（路线口径）** | 279,069 点 / 30 面 / 4.10×8.90 m / RMSE 0.105 m |
| P3 定位 | **PASS** | 根因＝空 shared_ptr，已最小修复；`/hdl_odom` 8.4 Hz；XY 误差 mean 0.0265 m / max 0.0452 m |
| P4 导航 | 见 §33 | 用算法位姿驱动 graph+support |

## 33. P4 第一轮：装配跑通到 BT，未达成到达（第十二轮）

### 33.1 为什么 P4 不能沿用现成装配

| 现成件 | 问题 |
| --- | --- |
| `graph_3d_gazebo_bt.yaml` | 位姿来自 `gazebo_carrier3d` 自己算的模型位姿，是**真值**，不能用来证明定位参与了导航 |
| `graph_3d_gazebo_loc.yaml` | 结构对（loc 独占 pose、carrier 位姿挪到 pose_raw），但地图是 NIST、且会 spawn 一台 app_carrier |
| `v8.fw.carrier_lite3` | 执行域声明是 `real_robot` / `robot_id=lite3`，放进 simulation 图直接撞 `E_EXEC_MODE`；**不做绕过**，那等于削弱执行域门控 |
| `v8.fw.carrier3d`（kinematic） | 自己按 cmd 积分位姿，接不了外部输入 |

⇒ 新建 `graph_3d_m1_hdl_bt.yaml`：`loc` 取自 `v8.fw.localization_source`（`lio_topic=/hdl_odom`）并独占
`/v8/local/pose`；`carrier` 保留 `gazebo_carrier3d` 但 `spawn=false`、位姿输出挪到 `pose_raw`，
只为满足 app_runner「图里必须有名为 carrier 的实例且 component 与 backend 对应」这条硬校验。
感知点云改 `/rslidar_points`（不跑 faster_lio，没有 `/cloud_registered`）。

### 33.2 地图必须换，否则起点在图外

`v8_flat_r4_v1.json` 的源是 `run11.pcd`（208,174 点），该文件随 `/tmp` 清空已**永久丢失**；
且它的 ROI 是 x∈[1.5,5.6]、y∈[−4.5,4.4]，**不含 lite3 的实际起点 (0,0)**。

改用还在的 `mapped_m1.pcd`（与 hdl 的 globalmap 同一份，坐标系天然自洽）重新生成：

- `v8_m1_hdl_v1.json`：`frame=map`、`origin=[-0.5,-5.2]`、`extent=130×112` ⇒ x∈[−0.5,12.5]、y∈[−5.2,5.9]
- 自动 task：start (1.15, 0.45) → goal (4.65, 0.45)，直线 3.5 m，到达容差 0.15 m
- 起点被工具挪到 (1.15,0.45) 是正常的：`--drop-self` 会挖掉机器人脚下的点，(0,0) 本身是洞
- 顺带发现：`data/maps/*` 被 gitignore，`registry.json` 从不入仓（只有 `registry.template.json`），
  所以这份地图与注册表都是本机产物，需要按 §33.5 的命令重建

### 33.3 本轮修掉的三个真缺陷

| 缺陷 | 症状 | 修法 |
| --- | --- | --- |
| `localization_node.py` 的 `yaw_of` 只认四元数消息 | `/sim/odom_gt` 一到就 `AttributeError: 'tuple' object has no attribute 'w'`，loc 节点被打死，`localization_ok` 永远没有 | `yaw_of` 同时接受属性形式与 `(qx,qy,qz,qw)` 元组 |
| `manifest_localization_source.json` 的 `launch.type` 带 `/` | `framework/nodes/localization_node.py` 让 roslaunch 找不到类型，进程根本不启动，**连 `v8-3d-loc-*.log` 都不建**；8 个实例只起 7 个 | 新增顶层 shim `scripts/run_localization_source.py`，`type` 改裸名（与 `run_map_file.py` 同模式，即 F4 的同类遗漏） |
| 判据按数量判「都起来了」 | 7/8 也判过，缺的正是 loc，于是后面的 `E_PLAN_NOT_READY`、`loc_ok` 空全被当成下游问题 | 脚本增加「`/v8/3d/loc(/|$)` 必须在列表里」这道独立闸门 |

### 33.4 当前读数（如实）

| 项 | 读数 | 判定 |
| --- | --- | --- |
| 装配实例 | 8/8 全起：carrier, global, loc, local, map, mux3d, perception, bt | ✅ |
| 地图 | `/v8_map_status`：`ok=true, map_id=v8_m1_hdl, surfaces=1` | ✅ |
| 定位 | `/v8/local/localization_ok: True`；`/v8/local/pose` 6.6 Hz | ✅ |
| 全局规划 | `/v8/3d/global/status`：`ok=true, points=36, expansions=35` | ✅ |
| BT | `root=RUNNING`、`path_state=ready`、`path_points=36`、`localization_ok=true`、`leg_active=true` | ⚠️ 停在 RUNNING |
| local 目标 | `/v8/local/goal` **无消息** | ❌ BT 未进 FollowPath |
| 速度 | `/cmd_vel` 恒 0 | ❌ |
| 到达 | 未发生 | ❌ |

**P4 结论：PARTIAL（未达成）。** 链路已通到 BT 且全局路径有解，卡在 BT → local 的目标下发。

### 33.5 复现命令

```bash
# 0) 重建支持地图与注册表（本机产物，不入仓）
cp data/maps/registry.template.json data/maps/registry.json
cd ws/src/90_platform/v8_platform/framework && python3 tools/map_from_pcd.py \
  --pcd /opt/ros1-slam/data/maps/mapped_m1.pcd --map-id v8_m1_hdl --version 1 \
  --x-min -2.5 --x-max 15.2 --y-min -8.7 --y-max 8.9 \
  --start-x 0.0 --start-y 0.0 --goal-max 3.5 \
  --out /opt/ros1-slam/data/maps/generated/v8_m1_hdl_v1.json --register
#   注意：该工具会顺手重写同目录的 *_app.yaml（把 localization.source 改回 carrier），
#   所以跑完必须再按 §33.1 的取值把它改回来

# 1) 起 Gazebo（真定位接入，占位桥必须退位）+ hdl
SKIP_GAZEBO=0 MODE=normal DUR=20 bash scripts/verify/asan_hdl_repro.sh

# 2) 起装配、发目标、采样
bash scripts/verify/p4_nav_hdl.sh
```

### 33.6 下一轮要查的（已缩小范围）

BT 的 `request_path` 已 SUCCESS、`path_state=ready`，但 `/v8/local/goal` 没有任何消息。下一步看
`lite3_bt/trees/navigate_with_planner.xml` 里 FollowPath 的前置条件与 `out_goal` 的发布路径：
是 `follow_path` 叶节点没被 tick 到，还是 tick 到了但 `out_goal` 的 latch/频率让它没落到话题上。
判据不能只看 `root=RUNNING`，要盯 `nodes[]` 里 `follow_path` 那一项的状态。

## 34. P4 第二轮：拿到非零运动，未达成到达（第十三轮）

### 34.1 上一轮判断错了一处

§33 说「卡在 BT → local 的目标下发」。实测 BT 的 `nodes[]` 里
`wait_path SUCCESS / follow_path SUCCESS / wait_arrival RUNNING` —— **通过 BT 是正常的**，
它已经走完 follow_path 并进入等到达。真正的断点在下游：local 只走了 0.308 m 就停。

### 34.2 五层根因，逐层剥开

| # | 现象 | 根因 | 修法 |
| --- | --- | --- | --- |
| 1 | `/v8/local/localization_ok` 永远没有 | `localization_node.yaw_of` 只认四元数消息，而 `error_between` 传的是 `(qx,qy,qz,qw)` 元组 ⇒ `/sim/odom_gt` 一到就 `AttributeError` 把 loc 打死 | `yaw_of` 兼容属性与元组两种形态 |
| 2 | 图里 8 个实例只起 7 个，缺 loc，且不报错 | `manifest_localization_source.launch.type` 带 `/`（`framework/nodes/...`），roslaunch 找不到类型，进程根本不启动、连日志都不建 | 加顶层 shim `v8_platform/scripts/run_localization_source.py`，type 改裸名 |
| 3 | local 拒绝：`E_LOCAL_SUPPORT: start requires a matching z support` | 地图 `z_reference=5.7e-05`（地面点中位 z），hdl 的 `publish_odometry` 强制 `z=0.0`，而 `support_tol_m` 默认 **1e-6** ⇒ 差 57 微米就判定无支撑 | 建图加 `--z-align`，`z_reference` 归 0 |
| 4 | 同上，但起点处连地面都没有 | 雷达正下方是盲区，`mapped_m1.pcd` 在 `|x|,|y|≤0.35` 内点数为 **0**，地图在起点是个洞 | 建图工具新增 `--fill-start X,Y`（默认关闭），按半径补一块起点平台 |
| 5 | 车只走 0.30 m 就停，BT 等 139.5 s 后 arrival timeout | `local_native` 是**步进式**：收到一个 goal 走 `max_step_m`（0.30 m）就 `segment_complete`，要等下一个目标；而 `path_goal3d` 的实现只转发 `poses[-1]`（路径终点），等价于发一次就没了 | `path_goal_node.py` 改为按路径顺序逐点下发，当前点到达（`arrive_m`）或超时后推进 |

第 5 条是 P4 的关键：`graph_3d_gazebo_loc.yaml` 里那个 `path_goal` 实例从未真正逐点工作过，
和 `localization_source` 的 type 问题一样，属于既有实现缺陷。

### 34.3 顺带确认的两件事

- `app_runner` **强制自起 master**（`ROS_MASTER_URI=http://127.0.0.1:11537/`），与外部 roscore 完全隔离，
  `--no-gazebo` 也不行 ⇒ 外部已跑的 Gazebo+hdl 它看不见。复用现有 master 只能走
  `cli.py plan --out-launch` 渲染后 `roslaunch`。
- `local` 的 `E_LOCAL_NO_ROUTE: goal lies inside an inflated obstacle volume` 来自动态障碍的 **AABB**：
  m1 场景里有一个 `(1.65–4.62, −1.40–3.01)` 的 3×4.4 m 大盒（L 形墙/家具的轴对齐包围盒），
  加 0.30 m 膨胀后覆盖机器人正前方。**机器人起点距该盒仅 11 cm**，因此任何方向都判碰撞。
  这不是参数问题，是起点位置与障碍 AABB 的几何死锁。

### 34.4 实测读数（本轮最好的一次）

| 项 | 读数 |
| --- | --- |
| 装配实例 | 9/9 全起（含 `path_goal`） |
| 定位 | `/v8/local/localization_ok: True`；hdl 在 (1.048,−4.236) 对真值 (1.0,−4.245)，差 4.8 cm |
| 全局路径 | `ok=true, points=16` |
| 非零运动 | **`max_speed_mps=0.6`，`speed_frames=462`** |
| 位移一致性 | `est_travel 0.2737 m` vs `gt_travel 0.2735 m`（算法位姿与真值差 0.2 mm） |
| 到达 | 未达成：`arrive_m` 之外的 waypoint 被动态障碍膨胀覆盖（`E_LOCAL_NO_ROUTE`） |
| 定位失效回零 / 取消 ACK | 未测（依赖到达先成立） |

### 34.5 P4 结论：**PARTIAL —— 非零运动与位姿一致性达成，到达未达成**

链条上每一环都已单点验证通过（装配 9/9、定位 ok、路径有解、目标逐点下发、速度 0.6 m/s），
剩下的是**场景几何**：m1 世界里机器人起步位置紧贴一堵 L 形墙的 AABB，膨胀 0.30 m 后没有可行方向。

### 34.6 下一轮

不必再查代码，先把**起点与走廊**选在障碍 AABB 之外（用 AABB 与 support cells 做联合筛选，
本轮已用 BFS + AABB 采样试过一轮，但障碍是**动态**的，逐点目标仍可能被新出现的盒子覆盖）。
两条可选路径：

1. 让 `dynamic_obstacles` 只对**影响当前走廊**的盒子做膨胀（现在是对全部盒子求并），
   但这动了安全判据，需要单独裁决，不在本次授权内；
2. 换一个更空旷的仿真世界（或把机器人放到远离 L 形墙的位置），先验证「算法位姿驱动到达」这件事本身。

## 35. 碰撞模型与动态膨胀优化（第十四轮）

### 35.1 先量化：AABB 里有多少是空气

`scripts/verify/obstacle_aabb_audit.py` 用真实帧跑与线上完全相同的管线，判据只用点云自身：
占用面积 = 框内非空 xy 栅格数 × voxel²，空洞率 = 1 - 占用/投影。

| 指标 | 优化前 | 优化后（min_fill=0.45） |
| --- | --- | --- |
| 框数 | 1 | 7 |
| 总投影面积 | 6.620 m² | **1.461 m²（-78%）** |
| 实际占用 | 1.420 m² | 1.460 m² |
| 总空洞率 | **0.785** | **0.001** |
| **膨胀后面积** | 10.223 m² | **7.906 m²（-23%）** |
| 未覆盖点 | 8 | 8（`coverage_ok=True`） |

`uncovered=8` 两档相同：那是 `min_points=8` 历来丢弃的零星簇，不是分裂造成的。
分裂**只收紧不放宽**：每个点仍落在某个返回框内，膨胀距离一字未改。

### 35.2 实现：递归二分，超预算回退

`dynamic_obstacles.cluster_boxes` 新增 `min_fill / min_extent_m / max_depth / voxel_m`，
**默认 `min_fill=1.0` 完全保持历史行为**（一个连通分量一个 AABB）。填空：

- 判据用**体素格数**而非米制换算：`fill = 非空格数 / (ki跨度 × kj跨度)`，避免浮点换算误差；
- 低于阈值且长边 > `min_extent_m` 时沿长边按中位数切开，递归至 `max_depth`；
- 分裂后总数超过 `max_objects` 时**整体回退成粗框** —— 宁可多占空间，也不能因为数量上限把某片点丢掉。

### 35.3 受控场景：一条真实可走、却被 AABB 判死的通道

L 形墙（横臂 x∈[1,3] y∈[0,0.3]，竖臂 x∈[1,1.3] y∈[0.3,2]），探针段 (2.0,1.0)→(2.0,2.5)：

| | 框数 | 面积 | 该段判定 |
| --- | --- | --- | --- |
| 优化前 | 1 | 3.572 m² | **碰撞（误判）** |
| 优化后 | 2 | **0.837 m²（-77%）** | **可通行** |

`false_block_removed = true`。这是「通行率提升」的直接证据：**误判消除，真障碍一处未放过**。

### 35.4 机体膨胀核实：没有重复叠加

- URDF 实测机体 `0.5481 × 0.2170 × 0.1264 m`，半宽 0.1085 m，腿部关节外露约 0.11–0.13 m；
- `radius_m=0.15` 覆盖横向并留 36 mm 裕量；纵向半长 0.274 m 由**扫掠线段**本身覆盖，不需要圆半径；
- `uncertainty_m` 线上**恒为 0**（仅测试传入）⇒ 膨胀只有一处：`radius_m + clearance_m`；
- `gate_m=0.8` 是**数据关联**门限（速度估计），不参与碰撞，未动。

### 35.5 动态膨胀：只增不减 + fail-closed

`local_native.dynamic_margin_extra(config, speed)`：

```
extra = v·reaction_s + v²/(2·brake_decel_mps2)，上限 dynamic_margin_cap_m
```

| 量 | 取值 | 依据 |
| --- | --- | --- |
| `reaction_s` | 0.10 | obstacles 实测 10 Hz，覆盖一个感知周期 |
| `brake_decel_mps2` | 1.0 | Lite3 实际制动能力高于此值，取小值属安全侧 |
| `dynamic_margin_cap_m` | 0.30 | 避免高速时把通道整条算死 |
| 定位误差 | **不单独计入** | `clearance_m=0.10` 已大于 hdl 实测 XY 误差（mean 0.0265 / max 0.0452 m） |

速度不可知（None / NaN / 负）时按 `max_speed_mps` 取值 —— **fail-closed**。
7 项单测覆盖：速度单调性、判定翻转、注入有效性（把两项清零后余量必须与速度无关）、fail-closed、上限、非负。
`local_planner` 全套 **161 + 7 = 168 项全过**。

### 35.6 四场景回归

| 场景 | 结论 | 读数 |
| --- | --- | --- |
| A 复杂障碍（真实帧） | 通行率不变 | 0.921 → 0.921；该帧最近障碍 1.796 m，空洞没落在候选路径上 |
| A2 L 形（受控） | **误判消除** | 3.572→0.837 m²，该段 碰→不碰 |
| B 狭窄通道 | 判据与几何自洽 | 理论需净宽 **0.613 m**，实测最小可通过 **0.65 m**，余量 +0.037 |
| C 动态障碍 | 安全侧增长 + fail-closed | extra 随速度 0.006→0.24；速度未知时判碰；侧向 1.0 m 绕行全速度可过 |
| D 正常通行 | 未被误伤 | 空旷与远处障碍均可通过 |

### 35.7 真实导航：优化前后对比（同一走廊、同一目标）

| 配置 | speed_frames | 位移 est / gt | 到达 | 碰撞 |
| --- | --- | --- | --- | --- |
| 无优化（基线，§34） | 462 | 0.2737 / 0.2735 m | 否 | **0** |
| 只开 AABB 细化 | 9 | 0.0423 / 0.0455 m | 否 | **0** |
| 只开动态膨胀 | 22 | 0.1144 / 0.1167 m | 否 | **0** |
| 两者全开（默认交付） | — | 见 §35.8 | 否 | **0** |

**主因是动态膨胀**：段速度约 0.25 m/s ⇒ extra ≈ 0.056 m，安全距离 0.25 → 0.306 m（+22%）。
该走廊在此余量下判 `E_LOCAL_COLLISION`（`no collision-free, supported progress within horizon`），
机器人只前进 6 cm 即停。**这是"只增不减"的正确后果，不是缺陷**：原本能过的走廊在更严余量下不够宽。

### 35.8 结论与边界

- **碰撞次数 0**（三轮实测均未发生真实碰撞）；
- **AABB 过度包围已量化并消除**（空洞率 78.5%→0.1%，受控 L 形误判消除）；
- **动态膨胀按设计与测试工作**，代价是该走廊不再满足安全余量；
- **BT reached / 定位失效停止 / 取消** 仍未取得：到达在 m1 的这条走廊上不可行，
  需要**更宽的场景**才能验证（结论与 §34.6 一致，只是现在多了一层"余量已收紧"的约束）。

下一步（按优先级）：
1. 在 m1 里找一条净宽 > 0.7 m 的走廊重跑（B 场景已给出可通过阈值，可按此筛选）；
2. 若 m1 无此走廊，换世界；不要为了通过而回调 `reaction_s` / `clearance_m`。

## 36. P4 四项目标：一项 PASS、一项 PARTIAL、一项 INVALID、一项未跑（第十五轮）

### 36.1 先解决"去哪跑"：净宽是硬约束

`scripts/verify/free_space_width.py`：障碍点栅格化 → 方形膨胀 0.306 m → 多源距离变换 →
可达区内最大内切圆直径。**必须用 support map 做遮罩**，否则会把地图外的空白算成 25 m 通道。

结论：m1 世界**有**开阔区，最大净宽 **3.0 m**，位于 (3.75, 2.45)，可达 4539/4885 格。
原 (1.0,−4.25) 走廊在 0.306 余量下起点即判碰，**该处不可再用**。

### 36.2 四项结果

| 目标 | 判定 | 读数 | 阻塞点 |
| --- | --- | --- | --- |
| 1 正常导航 | **PARTIAL** | 非零运动：`speed_frames` 最高 **764**、`max_speed 0.6`、位移 **0.6169** vs 真值 **0.6174**（差 0.5 mm）；BT `outcome=reached` | reached 是在**估计坐标系**成立：该点 hdl 估计 (3.724,2.159) 对真值 (3.825,2.399)，**误差 0.26 m > 容差 0.15 m** |
| 2 障碍安全 | **PASS** | 多轮实测 **碰撞 0 次**；密集区 `E_LOCAL_COLLISION` 拒动（`obs=27`） | — |
| 3 定位失效安全 | **INVALID** | 两次均**未在运动中触发**：固定时刻触发时车已停（`speed_first_frame_after=0`） | 探针 `ROSInterruptException` 未捕获导致 JSON 不落盘（已修） |
| 4 取消安全 | **NOT RUN** | — | 依赖 3 的同一运动窗口 |

### 36.3 本轮修掉的两个探针缺陷

- `safety_probe.py`：kill nodelet manager 后 roslaunch 关停 master，rospy 抛
  `ROSInterruptException`，未捕获 ⇒ 进程直接退出、**JSON 压根不落盘**（表现为"测了没数据"）。
  改为 `try/except` + 结果照常写出。
- `p4_nav_probe.py`：`travel_gt_1m` 写死 1 m，短程目标下永远不成立；改成
  「至少走完初始距离的一半」，语义等价、不涉安全参数。
- `safety_probe.py` 增加 `--on-speed`：固定时刻触发容易在车已停下时误判为"运动中"。

### 36.4 必须记下的问题：开阔区定位精度会掉

(3.75,2.45) 一带估计误差 **0.26 m**（P3 实测是 0.0265 m mean / 0.0452 max）。
开阔区点云特征少，hdl 配准质量下降 ⇒ **BT 的 reached 判据（用估计位姿）在误差大于容差时不可信**。
这不是 BT 的缺陷，是"到达判定依赖定位精度"这一事实的暴露；
要拿到可信的 reached，需要**同时满足**：定位误差 << 容差，且路径真的走完。

### 36.5 验收边界（保持不变）

FAR INVALID、VCC RED=39、真机禁止**均未改动**。本轮未放宽任何安全参数：
`gate_m 0.8`、`radius_m 0.15`、`clearance_m 0.10`、`reaction_s 0.10`、`brake_decel_mps2 1.0`
全部保持原值；`min_fill` 仍为 0.45（细化启用）。

### 36.6 下一轮

1. 3/4 项需要**能持续运动的窗口**：用 0.5–0.6 m 的短目标（已在 36.2 第 1 行验证可走 0.62 m），
   在 `--on-speed 0.10` 触发，探针已修好；
2. 若要可信的 reached，先在开阔区把 hdl 误差压到 0.05 m 以内（或换特征更丰富的位置），
   否则 reached 永远只能算"估计意义下成立"。
