# boundary · 机器人边界适配器（S1—S4）

> 高频命令/遥测走 UDP；ROS 侧仍只由 mux 写 `/cmd_vel`。真机 S5（QNX 43893）未授权，保持 L2。

## 帧

- 下行 `CommandFrame` 33 B：magic `V8BD`、ver、mode（ACTIVE/HOLD）、seq、单调 t_ns、
  vx/vy/wz（机体系）、flags、CRC16。
- 上行 `TelemetryFrame` 182 B：magic `V8TM`、state/fault、seq、t_ns、位置/机体系速度/
  四元数/角速度/比力、12 关节位置与速度、CRC16。
- `protocol.py` 负责编解码与坏帧拒绝；`udp_link.py` 负责发送、接收、序号缺口/重复统计。
- 适配器不会把规划周期当发送周期：`rate_bridge.py` 锁存最新命令并限幅/斜率/超时归零。

## S2：ROS 包装与 MuJoCo 端点

```bash
# 适配器：订阅 /cmd_vel，向 MuJoCo 端点 5006 发命令，5015 收回遥测
cd ws/src/90_platform/v8_platform
python3 -m boundary.ros_bridge _target:=127.0.0.1:5006 _bind:=127.0.0.1:5015 \
  _rate_hz:=200 _command_topic:=/cmd_vel
# MuJoCo 端点（容器内；需要 mujoco/onnxruntime）
cd ../95_simulation/lite3_gazebo_sim/dynamics
python3 boundary_endpoint.py --xml /tmp/mjcf/Lite3.xml --policy /tmp/new_policy.onnx \
  --bind 127.0.0.1:5006 --target 127.0.0.1:5015 --seconds 5
```

适配器发布 `/boundary/odom`、`/boundary/imu`、`/boundary/joint_states`（`joint_0..11`）
与 `/boundary/status`。遥测缺失/过期或机器人 fault 时只发 HOLD 零速；不发布 `/cmd_vel`。

## S3：Gazebo 端点

```bash
python3 -m boundary.gazebo_endpoint --model s3_box \
  --bind 127.0.0.1:5026 --telemetry-target 127.0.0.1:5025 --seconds 5
```

`SetModelState` 运动学载体（非接触动力学）；同一 33 B 命令/182 B 遥测契约。
测试用 `boundary/s3_gazebo_check.py` 发命令并核对模型位置。

## S4：记录/回放与报告

```bash
python3 -m boundary.s2_loopback --target 127.0.0.1:5006 --bind 127.0.0.1:5005 \
  --seconds 7 --command 0.2,0,0 --record /tmp/frames.jsonl
python3 -m boundary.recording summarize --file /tmp/frames.jsonl --out /tmp/report.json
python3 -m boundary.recording replay --file /tmp/frames.jsonl --kind command \
  --target 127.0.0.1:5017 --rate-hz 500
```

报告含发送/接收率、间隔 p50/p99、序号丢包、应用层到达延迟与临时目标。
延迟口径是“端点 t_ns → 应用到达”，含接收线程调度；S1 的纯传输读数另记。

## 边界

- 以上均为本机回环/仿真；未接 QNX、未碰厂商栈、未接真机（S5=L2）。
- MuJoCo 端点是 270 策略的探测载体，命令上限默认 0.25 m/s；不是 V8-X1 正式验收。
- Gazebo 端点是运动学位姿汇，不证明接触/步态能力。
- 报告中的阈值是 R954 §4 的临时目标，未冻结；不为验收签字。
