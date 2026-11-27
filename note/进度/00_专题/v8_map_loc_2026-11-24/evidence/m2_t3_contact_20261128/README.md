# T3 接触位姿接线与高度残差根因（2026-11-28）

## 结论

1. **T3 已生产但从未接线**：`/v8/local/pose_plan` 由 loc 发布，manifest 没有该输出口，三张 M2 LIO 图的 `local.pose` 仍接 `loc.pose`（base_link）。本切片补端口与边，测试 5/5 通过。
2. 接线后运行时确认 local 真正消费接触位姿：`start z` 从 base 的 `0.0625` 变为 `-0.0489`，`/v8/local/pose_plan.z=-0.04677`，与 base−0.11 一致。
3. **A5 仍阻断**：local 报 `E_LOCAL_SUPPORT | z=-0.0489 z_tol=1e-06 候选=0`，`/cmd_vel=0`；同刻 GT `z=0.1100`、base 估计 `z=0.0625`。不是 T3 未消费，而是定位 z 本身低约 4.8 cm。
4. 20 s 静止采样：`/Odometry.z=-0.0534 → -0.0542`（范围 -0.0574..-0.0378），GT z 恒 0.110000。**这是固定偏置，不是持续漂移。**
5. 偏置与 `faster_lio_outdoor.yaml` 的 `mapping/extrinsic_T=[-0.011,-0.02329,0.04412]`（真机 Mid-360 工厂外参）同量级：LIO `body` 是 IMU 系，当前 T1 锚点 `z=0.45` 与 T2 `sensor_to_base` 却按“body=雷达”推导。**T2/T1 的 body 定义尚未统一，不能把它当纯平移误差修掉。**
6. 仿真侧把外参置零**不是**可用修法：实测 `/Odometry.z` 升到 `0.10..0.19`、x/y 在 5 s 内漂 `0.24/-0.06`，比原配置更差；`sim_extrinsic_zero` 保留为诊断开关，默认 `false`，未改生产 yaml。

## T3 接线改动

- `manifest_localization_source.json`：新增输出口 `pose_plan`，topic `/v8/local/pose_plan`。
- `graph_3d_m2_lio_bt.yaml`、`graph_3d_m2_lio_bt_far.yaml`、`graph_3d_m2_lio_accept_bt.yaml`：
  `loc.pose -> local.pose` 改为 `loc.pose_plan -> local.pose`；BT/感知仍用 base 位姿。
- `test_graph_3d_assembly.py`：新增 manifest 端口与三张图的接线断言，5/5 OK。

## 运行时读数（本目录）

- `/v8/local/pose_plan`：`(5.18306, 2.47366, -0.04677)`
- `/v8/local/pose`（base）：`(5.20805, 2.47505, 0.06253)`
- `/sim/odom_gt`：`(5.20000, 2.45000, 0.11000)`
- `/v8/local/status`：`stop=true, reason=E_LOCAL_SUPPORT, z=-0.0489, z_tol=1e-06, 候选=0`
- `/cmd_vel`：零
- 9/9 节点，目标受理（`goal_once.json accepted=true`）

## 复跑

```bash
# 底座（隔离 GT；本目录读数用的历史 spawn 0.15）
docker exec -e M2_ISOLATE_GT=1 -e M2_MAP_SPAWN_X=5.2 -e M2_MAP_SPAWN_Y=2.45 \
  -e M2_MAP_SPAWN_Z=0.45 -e M2_SPAWN_Z=0.15 ros1-slam-sim bash -lc \
  'source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash; \
   export ROS_MASTER_URI=http://127.0.0.1:11313; bash /opt/ros1-slam/docker/sim.sh down; \
   sleep 2; bash /opt/ros1-slam/docker/sim.sh base m2'

# 图 + 单个目标（p4_nav_hdl 的 NO_GOAL 模式）
# 读数：/v8/local/status、/v8/local/pose、/v8/local/pose_plan、/sim/odom_gt、/cmd_vel
```

## 未闭合

- body=IMU（含工厂外参）与 T1/T2 的统一定义：需按 IMU 初始位姿重算锚点与 T2，或证明 LIO 输出应改回雷达系；这是 A2 的核心，不能仅凭 `support_tol_m` 放宽容差解决。
- `support_tol_m/support_assoc_tol_m` 仍为原值，`0.08` 未批准、未使用。
- A5 四例、FAR 消费链、GT TF/话题独立关断仍未完成。

## 补记（2026-11-28 晚）

本目录的 T2 仍是雷达版（`0.08799,0,-0.328415`），已由
`../m2_imu_t2_20261128/` 的 IMU 版 T2（`0.076998,-0.02329,-0.284295`）取代。
修正后 base z=0.11235（GT 0.11000），plan z 20 s 范围 -3.18..+4.33 mm；
本目录的 `z=-0.0489` 是修正前读数，保留为历史现场。
