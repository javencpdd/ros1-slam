# Faster-LIO body=IMU 的 T2 修正与残差闭合（2026-11-28）

## 结论

Faster-LIO 的 `body` 不是雷达原点，而是 **IMU**：生产配置 `faster_lio_outdoor.yaml` 的
`mapping/extrinsic_T=[-0.011,-0.02329,0.04412]` 是“雷达在 IMU 系中的位置”。
旧 T2 只做了 `base->rslidar` 的逆，把 body 当雷达，导致 base z 恒低约 4.8 cm。

按完整链路重算：

```text
base_from_imu = base_from_lidar(0,0,0.34, pitch +15deg)
                * lidar_from_imu(extrinsic_T 取反, R=I)
T2 = inverse(base_from_imu)
   = xyz [0.076998, -0.02329, -0.284295], rpy [0, -15deg, 0]
```

三张 M2 LIO 图已改为该 T2。活体读数：

| 量 | 值 |
|---|---|
| `/v8/local/pose` base z | `0.11235`（GT `0.11000`，+2.35 mm） |
| `/v8/local/pose_plan` z（单帧） | `-0.00348` |
| 20 s plan z | mean `+0.001618`，min `-0.003181`，max `+0.004329` |
| 20 s base z | mean `0.111618`，min `0.106819`，max `0.114329` |
| `/v8/local/status` | `E_LOCAL_SUPPORT | z=0.0036 z_tol=1e-06` |

即：**定位高度误差从约 4.8 cm 降到 3–4 mm**；剩下的 3.6 mm 是 LIO 静止噪声，不是 T2 错。

## 为什么仍阻断

`support_assoc_tol_m` 仍是 `1e-06`，而实测接触面残差最大 `4.33 mm`。要过起点关联，至少需要
覆盖这一噪声量级的容差。`0.08` 仍是它的 ~18 倍，不适用；按当前 20 s 证据，`0.01` 有 2.3 倍余量。
**本切片没有改任何安全门值**，等待 L2 裁决。

## 反控：仿真外参置零不成立

在 `m2_mapping.launch` 里把 `mapping/extrinsic_T` 置零（`sim_extrinsic_zero:=true`）后：

- `/Odometry.z` 升到 `0.10..0.19`；
- x/y 在 5 s 内漂 `0.24/-0.06`；
- 比原配置更差。

因此该开关保留为诊断项，默认 `false`，生产 yaml 与仿真底座均未改。

## 测试

- `test_graph_3d_assembly`、`test_graph_wiring`、`test_pose_contract`、`test_localization_contract`：**52/52 OK**。
- 图测试新增：三张 M2 LIO 图必须使用 IMU 版 T2（`0.076998,-0.02329,-0.284295` / `-15°`），并接线 `loc.pose_plan -> local.pose`。

## 文件

- `local_status.yaml`、`pose_plan.yaml`、`local_pose.yaml`、`gt_odom.yaml`、`cmd_vel.yaml`、`goal_once.json`
- `imu_t2_trend.csv`：20 s 的 base/plan/GT 逐帧 z
- `nav_graph/`、`nav_graph_trend/`：两次装配的 plan/readiness/log

## 未闭合

- A5 四例仍因 `support_assoc_tol_m=1e-6` 无法起步；`0.01` 需 L2。
- GT TF/话题独立关断、FAR/感知运行时 GT 消费仍未完成。
- `support_tol_m`、`0.08` 均未使用。
