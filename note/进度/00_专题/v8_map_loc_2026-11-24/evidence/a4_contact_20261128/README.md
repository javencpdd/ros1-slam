# A4 高度/接触点契约证据（2026-11-28）

## 结论

**测量完成**：按已修正的 T1/T2/T3 接线，M1 平地、`v8_m1_hdl` 地图、GT 隔离 M2 基线下采集 118 个有效配对帧。

| 指标 | 读数 |
|---|---|
| 接触残差 `plan_z − z_reference` | min **−0.06103 m**，p50 **−0.05417 m**，mean **−0.05416 m**，max **−0.04756 m** |
| 标准差的逐帧波动 | **0.00315 m** |
| 0.05 m 内 | **否**（abs_max 0.06103） |
| 0.08 m 内 | 是（abs_max 0.06103），但 **0.08 m 不是已批准容差，仅作读数** |
| 配对最大时差 | 1e-9 s（plan/pose/odom 同源戳） |

原始逐帧文件：`contact_residual.csv`（119 行=表头+118 帧），含 `plan_stamp/pose_stamp/odom_stamp/gt_stamp`、`plan_frame/pose_frame/odom_frame/odom_child`、base/plan xyz、`z_reference`、逐帧残差、四元数、GT。

地图：`data/maps/generated/v8_m1_hdl_v1.json`，sha256 `e92ccbaec96db4e54da32185c1a816af81ca4858ad9bdb378e553809b34f4251`；surface `m0`，`z_reference=0.0`。

## 契约修正

- `431228eb` 的 `contact_pose` 原来只有 xyz 元组；现在返回显式对象：
  `contact_pose = {xyz, z_reference, surface_id, source}`。`z_reference` 不再隐含在 xyz 第三维里。
  `estimated_pose` 仍保持原估计位姿，不被接触点覆盖；`trajectory` 与碰撞/净空/制动判据未改。
- `test_plan_reports_contact_and_consistency_flag` 已断言 `contact_pose.z_reference` 与关联面一致。
- 预测位姿失支撑、反馈仍在支撑面时，`track_segment` 返回零速 `hold`/`E_LOCAL_SUPPORT`；
  新增测试 `test_predicted_support_loss_holds_zero_while_feedback_is_on_support`。**
  这条不是把“预测脱离支撑”一刀切判死：既有转层/超调测试证明某些合法转层需要继续，
  所以未改成无条件 `E_LOCAL_SUPPORT`；只锁“零速保护”。**

## 现有负控覆盖（已跑绿）

- `test_support_assoc.py` 10 项：默认/拓宽关联容差、歧义多层拒绝、越界格拒绝、输入不变、迁移提示。
- `test_support_assoc_outlet.py` 4 项：合法单层/多层、位姿离开支撑零速、地图版本变更阻塞。
- `test_local_native.py` 28 项：多层同 XY、短台阶、错误高度不认证、薄顶棚净空、停止尾段、非有限输入、任务过期、支撑边界。
- 新增接触字段/预测零速控制 2 项在本轮。

## 未闭合

- `test_execution.py` 在 HEAD `7b1f6847` **基线本身**即有 9 项红（6 FAIL/3 ERROR，起点 z 漂移 0.0073 m
  对 `support_assoc_tol_m=1e-6` 判 E_LOCAL_SUPPORT）；在隔离 worktree 复跑同一提交得到同一组红，
  不是本轮 A4 引入。该组覆盖“制动尾段/运动中失关联/终端 settle”，因此 A4 的这两项**不能声称完成**，
  且不能用 0.08 改测试绕过。需另立“合法多层连通图上非零残差 plan/track 与制动尾段”的
  具名负控并决定支撑关联容差口径。
- 0.08 m 未批准正式使用；`support_tol_m` 仍 1e-6，未改。
- 坡面/台阶实机残差未测；本证据只覆盖平地 M2 隔离基座。

## 复跑

```bash
# 隔离基座 + 闸门 + 定位探针（见 A2/A3 README）
docker exec ros1-slam-sim bash -lc \
  'source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash; \
   export ROS_MASTER_URI=http://127.0.0.1:11313; \
   python3 /opt/ros1-slam/scripts/verify/contact_residual_probe.py \
   --map /opt/ros1-slam/data/maps/generated/v8_m1_hdl_v1.json \
   --seconds 12 \
   --out-json /opt/ros1-slam/note/进度/00_专题/v8_map_loc_2026-11-24/evidence/a4_contact_20261128/contact_residual.json \
   --out-csv /opt/ros1-slam/note/进度/00_专题/v8_map_loc_2026-11-24/evidence/a4_contact_20261128/contact_residual.csv'
```

## 文件指纹

`fingerprints.sha256` 记录探针、局部规划器、测试、地图与现场配置；`SHA256SUMS` 覆盖本目录全部文件。
