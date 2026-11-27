# A2 生产 TF / 导航位姿契约证据（2026-11-28）

## 结论

A2 本切片 PASS（限定为 M2 隔离基座 + 定位契约探针）：

- T1/T2/T3 已从三处分散常量收敛到 `framework/pose_contract.py` 的同一套刚体运算。
- `localization_node` 完整消费 T1（map←camera_init，含 15° 初始姿态）、T2（body→base_link，逆 URDF 外参）和 T3（接触点 world-z −0.11）。
- 活体非零 yaw 下：`/v8/local/pose` 与 TF `map→base_link` 差 **0.000 m**；`pose_plan` 比 `pose` 低 **0.110000 m**；GT yaw **0.5508 rad**，定位 yaw **0.5480 rad**（差 0.0028 rad）。
- 生产 TF 审计链 `rslidar ← base_link ← body ← camera_init ← map`，**VERDICT PASS**；GT 在 `gt_odom→gt_base` 独立分支。
- FAR 图已把 raw `/Odometry` 收到图内 `lio_gate` 后，loc 消费 `/Odometry_gated`；loc 参数中的 `lio_origin_z` 已从 0.11 改为 0.45。

## 关键纠正（源码 + 活体）

### 1. Faster-LIO 的 body / camera_init 语义

- `laser_mapping.cc:686` 的 `child_frame_id = "body"` 只说明输出帧名，不证明 body 是水平机体。
- `imu_processing.hpp:146-152` 初始化只设置 `grav`、`bg`、`offset_T_L_I`、`offset_R_L_I`，**没有用 IMU orientation 把 `rot` 旋到重力水平**；`camera_init` 就是首帧 body 系。
- 本仿真 IMU 挂在 `rslidar` 上，`frameName=rslidar`；URDF `base_to_rslidar` 为 `xyz(0,0,0.34) rpy(0,15°,0)`。
- 活体静止 `/livox/imu/linear_acceleration` 为 `(-2.532, 0.011, 9.476)`，证明重力在 body 中有非零 x 分量，body 并非水平系。

### 2. 416a9aef 的 T2 推导错误

URDF `p=(0,0,0.34)` 是父系（base_link）中的平移，不是 `R·(0,0,0.34)` 的结果。逆变换为：

```
t = -Rᵀ p, R = R_y(15°)
  = (0.34 sin15°, 0, -0.34 cos15°)
  ≈ (0.0880, 0, -0.3284)
rpy = (0, -15°, 0)
```

**纯平移 T2 只在本轮静止水平姿态下碰巧得到相同 base 位姿；非零姿态下不成立。** 新测试 `test_pure_translation_t2_is_not_vacuous` 在非零 roll/pitch/yaw 下给出 >0.10 m 误差。

## 活体读数

T1 静态边：

```
map→camera_init: translation (0,0,0.450), rpy (0,15°,0)
```

T2 静态边（隔离模式）：

```
body→base_link: translation (0.088,0,-0.328), rpy (0,-15°,0)
```

GT 分支：

```
gt_odom→gt_base: translation (0,0,0.110), rpy (0,0,0)
```

非零 yaw 运行时探针：`loc_tf_runtime_nonzero_yaw.json`

- `verdict=PASS`，`failed=[]`
- `contact_offset_median_m=0.11`
- `loc_minus_tf_m=0.0`
- `gt_yaw=0.5507681745`，定位/TF yaw `=0.5479906440`
- `loc_minus_gt_m=0.060779`（这是估计误差，不是契约误差；z 残差约 −0.056 m，与 T1/T2/T3 重算的负残差方向一致）

审计：`tf_audit_live.log` → `VERDICT PASS`，链上无 GT。

## GT 显式隔离

- `m2_gazebo.launch` 的 `gt_odom_frame` / `gt_base_frame` 默认恢复为历史 `odom` / `base_link`；只有显式传 `gt_odom` / `gt_base` 才隔离。
- `m2_mapping.launch` 的 `isolate_gt` 默认 false：
  - false：保留历史 `map→odom` 恒等边，不添加 `body→base_link`，历史 Stage A/B/C 调用方行为面不变；
  - true：添加真实逆外参 `body→base_link`，GT 不进入生产链。
- `docker/sim.sh base m2` 通过 `M2_ISOLATE_GT=1` 显式开隔离；默认值不变。
- 本证据窗使用 `M2_ISOLATE_GT=1`。
- **未在本切片重跑 Stage A/B/C 全量**：它们仍是历史默认模式，需在 A5 用隔离/历史两套模式分别复跑；本切片只核对了 launch 展开与默认参数面。

## 改动面

- 新增 `framework/pose_contract.py`（纯刚体函数）
- 修改 `framework/nodes/localization_node.py`（T1 全姿态、T2、T3、列表参数兼容）
- M2 三张图：FAR 图加 `lio_gate` 并改用 `/Odometry_gated`；三图均给 T1/T2/T3 参数
- `m2_gazebo.launch`、`m2_mapping.launch`、`docker/sim.sh`
- 新增测试：`tests/test_pose_contract.py` 和定位契约新增 `parse_vector` 用例
- 新增探针：`scripts/verify/loc_tf_runtime_probe.py`、`scripts/verify/loc_tf_contract_probe.launch`

## 复跑

```bash
# 隔离基座
docker exec ros1-slam-sim bash -lc \
  'M2_ISOLATE_GT=1 bash /opt/ros1-slam/docker/sim.sh base m2'

# 定位契约探针
docker exec -d ros1-slam-sim bash -lc \
  'source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash; \
   export ROS_MASTER_URI=http://127.0.0.1:11313; \
   exec roslaunch /opt/ros1-slam/scripts/verify/loc_tf_contract_probe.launch'

# 非零姿态
docker exec ros1-slam-sim bash -lc \
  'source /opt/ros/noetic/setup.bash; export ROS_MASTER_URI=http://127.0.0.1:11313; \
   timeout 3 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
   "{linear: {x: 0.0}, angular: {z: 0.35}}"'

# 运行时契约
docker exec ros1-slam-sim bash -lc \
  'source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash; \
   source /opt/ros1-slam/sim_ws/devel/setup.bash; \
   export ROS_MASTER_URI=http://127.0.0.1:11313; \
   python3 /opt/ros1-slam/scripts/verify/loc_tf_runtime_probe.py --seconds 4'

# TF 审计
docker exec ros1-slam-sim bash -lc \
  'source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash; \
   export ROS_MASTER_URI=http://127.0.0.1:11313; \
   python3 /opt/ros1-slam/scripts/verify/tf_chain_audit.py \
   --root map --target rslidar --gt-frames odom,gt_odom'
```

## 未闭合（A3/A4）

- TF 审计仍用 `time.time()` 与 ROS 源戳混比，尚未改成 ROS/monotonic 分离。
- 采集未填边级 publishers；当前 publishers 只在话题级。
- 50 ms 桶/xyz 压缩仍只是旁证，不代替发布者身份。
- 五组 GT 隔离负控、定位闸门“同时约束估计 TF 与导航位姿”尚未执行。
- 0.08 m 容差仍未批准；本切片未改 `support_tol_m`。
