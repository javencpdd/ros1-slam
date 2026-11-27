# A3 GT 评测域独立关断（2026-11-28）

## 结论

新增一个**只在隔离控制里使用**的 raw GT 域 + 评测中继，GT 的 TF 与话题可分别关断，
生产链（`map→camera_init→body→base_link→rslidar`）不受影响。

| 状态 | `/sim/odom_gt` | `gt_odom→gt_base` | 生产链 |
|---|---|---|---|
| raw 底座、未起中继 | 0 帧 | 不存在 | 正常 |
| 中继开 | 50 Hz | 存在 | 正常 |
| **只关 GT TF** | 50 Hz | 不存在 | 正常 |
| **只关 GT 话题** | 0 帧（`no new messages`） | 存在 | 正常 |
| **两者全关** | 0 帧 | 不存在 | **9/9 节点 PASS，`map→rslidar` 仍可查** |

全关时的生产读数：

- `/v8/local/pose`：`(5.19197, 2.44662, 0.11128)`，与 GT z=0.11 同量级；
- `tf_echo map rslidar`：`Translation [5.196, 2.468, 0.402]`，RPY `[0.076, 14.923, 0.016]` 度；
- `/v8/local/status`：`E_LOCAL_NOT_READY`（未发目标，属预期）；
- `/sim/odom_gt`：0 条平均速率；`gt_odom→gt_base`：无 Translation；
- raw 域 `gt_raw_odom→gt_raw_base`：仍存在（原始真值保留，供评测）；
- 感知订阅：`/rslidar_points` + `/v8/local/pose`，**无 GT**；
- BT 订阅：`/v8/local/pose`，**无 GT**。

FAR 运行时（补记）：在 raw 底座、eval GT 全关、无中继时启动 `graph_3d_m2_lio_bt_far.yaml`，
10/10 节点就绪、`/terrain_cloud` 2.3 Hz、`/sim/odom_gt` 0 帧、`gt_odom→gt_base` 无 Translation；
`/far_planner` 只订 `/terrain_cloud`、`/tf`、`/tf_static`，`/v8_global_far` 只订
`/tf`、`/v8/local/pose`；生产 `map→rslidar` 仍可查。FAR 侧不再只是静态证明。

## 实现

- `lite3_planar.urdf.xacro`：新增 `gt_odom_topic`，默认 `/sim/odom_gt`（历史行为不变）。
- `m2_gazebo.launch`：透传 `gt_odom_topic`，默认不变。
- `docker/sim.sh`：`M2_GT_RAW=1`（仅在 `M2_ISOLATE_GT=1` 时）切到
  `gt_raw_odom/gt_raw_base` + `/sim/odom_gt_raw`；默认仍 `gt_odom/gt_base` + `/sim/odom_gt`。
- `scripts/verify/gt_eval_relay.py`：raw→eval 中继，`~set_topic` / `~set_tf`
  两个 `SetBool` 服务；`~status` 每秒 JSON（received/topic_sent/tf_sent）。

## 复跑

```bash
# 1) raw 底座
docker exec -e M2_ISOLATE_GT=1 -e M2_GT_RAW=1 -e M2_MAP_SPAWN_X=5.2 -e M2_MAP_SPAWN_Y=2.45 \
  -e M2_MAP_SPAWN_Z=0.45 -e M2_SPAWN_Z=0.15 ros1-slam-sim bash -lc \
  'source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash; \
   export ROS_MASTER_URI=http://127.0.0.1:11313; bash /opt/ros1-slam/docker/sim.sh down; \
   sleep 2; bash /opt/ros1-slam/docker/sim.sh base m2'

# 2) 中继（默认 topic/tf 都开）
docker exec -d ros1-slam-sim bash -lc \
  'source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash; \
   export ROS_MASTER_URI=http://127.0.0.1:11313; \
   exec python3 /opt/ros1-slam/scripts/verify/gt_eval_relay.py __name:=gt_eval_relay \
   _in_topic:=/sim/odom_gt_raw _out_topic:=/sim/odom_gt _tf_parent:=gt_odom _tf_child:=gt_base'

# 3) 独立关断
rosservice call /gt_eval_relay/set_tf "{data: false}"
rosservice call /gt_eval_relay/set_topic "{data: false}"
```

## 文件

- `relay.log`、`set_tf_off.log`、`set_tf_on.log`、`set_topic_off.log`
- `tf_off_status.yaml`、`tf_off_topic_hz.log`、`tf_off_tf_echo.log`、`tf_on_tf_echo.log`
- `topic_off_status.yaml`、`topic_off_hz.log`、`topic_off_tf_echo.log`
- `both_off_status.yaml`、`both_off_tf_echo.log`、`gt_off_hz.log`、`gt_off_eval_tf2.log`、`gt_raw_tf2.log`
- `production_pose.yaml`、`production_pose_plan.yaml`、`production_status.yaml`
- `production_tf_map_rslidar.log`、`perception_subs.txt`、`bt_subs.txt`、`v8_nodes.txt`
- `nav_graph_gt_off/`：全关时 9/9 装配

## 未闭合

- GT 恢复后是否自动回到原任务仍以 A3 gate-recovery 的 stop-latch 证据为准（本切片未重跑任务）。
- `support_assoc_tol_m=1e-6` 的 3–4 mm 阻断仍需 L2（见 `m2_imu_t2_20261128/`）。

## FAR 运行时读数（far_graph_gt_off/）

- 节点：`far_planner`、`v8_global_far` + 8 个 v8 实例，10/10；
- `/terrain_cloud`：2.301 Hz（window 4）；
- `/sim/odom_gt`：0 条平均速率；`gt_odom→gt_base`：0 Translation；
- `/v8/local/pose`：`(5.18373, 2.44903, 0.11370)`；
- `map→rslidar`：可查；
- `/far_planner` 订阅：`/terrain_cloud`、`/tf`、`/tf_static`；
- `/v8_global_far` 订阅：`/tf`、`/tf_static`、`/v8/local/pose`；
- 证据文件：`far_graph_gt_off/far_subs.txt`、`global_far_subs.txt`、`terrain_hz.log`、`gt_off_hz.log`、`gt_off_tf.log`、`pose.yaml`、`production_tf.log`。
