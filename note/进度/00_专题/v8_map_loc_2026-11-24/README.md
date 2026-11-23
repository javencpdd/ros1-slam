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
