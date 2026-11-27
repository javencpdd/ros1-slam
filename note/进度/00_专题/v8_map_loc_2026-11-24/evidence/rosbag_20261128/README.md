# rosbag 录制/回放与速率复验（2026-11-28）

## 结论

在 M2 底座上录 8 s、4 话题，随后在**独立 master（11314/11315）**回放，
录制与回放速率一致，未向 11313 验收域重复发布：

| 话题 | 录制 | 回放实测 |
|---|---|---|
| `/Odometry` | 80 msgs / 8 s = 10 Hz | **10.043 Hz** |
| `/tf` | 480 msgs / 8 s = 60 Hz | **60.120 Hz** |
| `/clock` | 8002 msgs / 8 s ≈ 1000 Hz | bag 内计数（未单独 hz） |
| `/tf_static` | 3 msgs | latency 保留 |

bag：`m2_rosbag_20261128.bag`，684 KB，version 2.0，duration 8.0 s，
messages 8565，无压缩。

## 方法

```bash
# 录制（sim master 11313）
rosbag record --duration=8 -O /tmp/m2_rosbag_20261128.bag \
  /Odometry /clock /tf /tf_static

# 回放：独立 master，避免污染验收域
roscore -p 11314
ROS_MASTER_URI=http://127.0.0.1:11314 rosbag play --clock --rate 1 /tmp/m2_rosbag_20261128.bag
ROS_MASTER_URI=http://127.0.0.1:11314 rostopic hz /Odometry
ROS_MASTER_URI=http://127.0.0.1:11314 rostopic hz /tf
```

## 文件

- `m2_rosbag_20261128.bag`：原始 bag
- `bag_info.log`：`rosbag info`
- `replay_hz_odom.log`：回放 `/Odometry` 10.043 Hz
- `replay_hz_tf.log`：回放 `/tf` 60.120 Hz
- `replay_hz_clock.log`：本轮窗口未覆盖（playback 已结束），时钟计数以 `bag_info.log` 为准

## 边界

- 这是**速率/可回放性**证据，不是定位精度或导航验收。
- 回放只测消息面；不使用 GT 参与控制，不发 goal/cmd_vel。
