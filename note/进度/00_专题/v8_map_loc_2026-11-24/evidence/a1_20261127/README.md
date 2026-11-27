# A1 雷达数据流恢复证据（2026-11-27）

## 结论

**PASS（数据链恢复）**，限定为 `ros1-slam-sim` 容器内 `sim.sh base m2` 的只读探针结果：

| 环节 | 读数 | 判定 |
|---|---|---|
| `/clock` | 6052 帧，源戳全部前进 | PASS |
| `/livox/lidar` | 82 帧 / 9.54 s = 10.026 Hz；20000 点/帧，其中 5460 点非零；82/82 戳唯一且前进 | PASS |
| `/rslidar_points` | 79 帧 / 9.54 s = 9.534 Hz；frame=rslidar，width=5472 | PASS |
| `/livox/imu` | 1640 帧 / 8.2 s = 200.0 Hz；frame=rslidar | PASS |
| `/Odometry` | 80 帧 / 9.54 s = 9.54 Hz；frame=camera_init，child=body | PASS |
| 实载雷达插件 | `/opt/ros1-slam/sim_ws/devel/lib/liblivox_laser_simulation.so`，SHA256 `abba3ba350f634a932b969e586875c50298130a10b460d9c28edc4f5186b5c11` | PASS |

发布/订阅链由现场 `rostopic info` 证明：

- `/livox/lidar`：publisher `/gazebo`，subscriber `/sim_adapter`
- `/rslidar_points`：publisher `/sim_adapter`，subscriber `/faster_lio_online`
- `/Odometry`：publisher `/faster_lio_online`

## 第一处真实断点

进场时 `ros1-slam-sim` 只有 Xvfb（PID 46），没有 `gzserver`、`roslaunch`、ROS master；不是“插件在跑但传感器不发布”。直接后果是 `/livox/lidar`、`/rslidar_points`、`/Odometry` 全部不存在。

用既有 `docker/sim.sh base m2` 启动后，`BASE READY（m2 + LIO）`，上述五路数据全部恢复。**未重启整套 compose、未改插件、未重打时间戳。**

插件本身有“有订阅者才发布”的行为；`sim_adapter` 对 `/livox/lidar` 的常驻订阅即激活源。单独起 gzserver 后用一次性 `rostopic echo -n1` 探测，容易在订阅退出后看不到持续流，不能据此判插件失效。

## 探针与自纠

新增只读探针 `scripts/verify/a1_sensor_chain_probe.py`。它只订阅、统计和读取 `/proc/<gzserver>/maps`，不发布、不改戳。

首跑曾把自己判 FAIL：RTF≈0.37 时用墙钟分母，把 10 Hz 源误算成 3.8 Hz。第二版改为按 ROS 源戳跨度算 `source_rate_hz`，并补非零点、IMU 200 Hz、LIO frame 对名判据；复跑 PASS。首跑是探针口径错误，不作为数据链结论；本目录只保存第二版可复跑证据。

## 复跑

```bash
docker exec ros1-slam-sim bash /opt/ros1-slam/docker/sim.sh base m2
docker exec ros1-slam-sim bash -lc '
  source /opt/ros/noetic/setup.bash
  source /opt/ros1-slam/ws/devel/setup.bash
  source /opt/ros1-slam/sim_ws/devel/setup.bash
  export ROS_MASTER_URI=http://127.0.0.1:11313
  python3 /opt/ros1-slam/scripts/verify/a1_sensor_chain_probe.py \
    --seconds 8 --wait-first 15 \
    --out /opt/ros1-slam/note/进度/00_专题/v8_map_loc_2026-11-24/evidence/a1_20261127/a1_probe_rerun.json'
```

## 文件指纹

- 探针源码：见 `fingerprints.sha256`
- 启动/配置/URDF：见 `fingerprints.sha256`
- 原始 JSON：`a1_probe.json`
- 现场上下文：`runtime_context.txt`
- 全文件校验：`SHA256SUMS`