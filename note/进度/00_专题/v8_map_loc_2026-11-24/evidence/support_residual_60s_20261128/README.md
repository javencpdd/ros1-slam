# 60 s 静止接触残差（2026-11-28）

## 结论

在 T2 修正后的 M2 图上静止采样 60 s（599 个 plan/base 样本，3000 个 GT 样本）：

| 量 | 均值 | 最小 | 最大 | 备注 |
|---|---|---|---|---|
| `pose_plan.z` | `-0.000524 m` | `-0.004861 m` | `+0.002123 m` | abs max **4.861 mm**，std 1.700 mm |
| `pose.z`（base） | `0.109476 m` | `0.105139 m` | `0.112123 m` | GT `0.110000` 恒定 |

即 20 s 的 4.329 mm 不是偶然；60 s 的绝对最大是 **4.861 mm**。
`support_assoc_tol_m=1e-6` 仍会挡住；**0.005 只剩 0.14 mm 余量，不安全**；`0.01` 约有 2 倍余量，
`0.08` 仍是 16 倍以上，不适用。本切片未改任何门值。

## 文件

- `residual_60s.csv`：base/plan/GT 逐帧 z
- `nav_graph/`：9/9 装配、plan、readiness、log
- `sampler.log`：采样脚本输出（n/均值/极值）

## 复跑

```bash
# 底座：默认 M2 隔离 GT；图 NO_GOAL 起 9 节点
# 订阅 /v8/local/pose、/v8/local/pose_plan、/sim/odom_gt 60 s
# 注意：rospy.sleep 用仿真时间；RTF≈0.4 时 60 仿秒≈150 墙钟秒
```
