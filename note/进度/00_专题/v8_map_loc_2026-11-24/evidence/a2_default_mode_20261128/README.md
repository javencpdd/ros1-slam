# A2 历史默认模式回归（2026-11-28）

## 结论

不带 `M2_ISOLATE_GT` 起 M2 底座，历史调用方语义保持不变：

- `/sim/odom_gt`：`frame_id=odom`、`child_frame_id=base_link`；
- `odom→base_link` TF：可查（`gt_tf=3` Translation 行）；
- `body→base_link` TF：不可查（`body_tf=0`，隔离模式才发）；
- `/Odometry`：10.000 Hz，仍为 LIO 位姿；
- `/sim/odom_gt` 仍在该名，未改 raw 名。

即 GT 隔离仍为显式 opt-in；`m2_gazebo` 的新增 `gt_odom_topic`、xacro 帧名参数默认值与历史一致。
本切片未改代码，只做回归取证；随后已恢复隔离底座。

## 文件

- `gt_odom.yaml`：frame/child/pose
- `gt_tf.log`、`body_tf.log`
- `SHA256SUMS`
