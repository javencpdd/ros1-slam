# boundary · UDP 命令与遥测适配

> 最新更新时间：2026-11-06 04:34:56 CST。本次为文档与源码核对；验收状态以[项目进度](../../../../../note/进度/README.md)为准。

本目录将 ROS 速度意图变成固定频率 UDP 命令，遥测回传 ROS。它订阅 mux 输出，不发布 `/cmd_vel`，不连接厂商 QNX 实机链；以下端点仅用于本机回环/仿真。

## 协议与桥接

- `CommandFrame` 为 33 B：V8BD、版本、ACTIVE/HOLD、序号、单调时刻、机体系 vx/vy/wz、flags、CRC16。
- `TelemetryFrame` 为 182 B：V8TM、状态/故障、序号/时刻、位置/机体系速度、四元数、角速度/比力、12 关节位置/速度、CRC16。
- [protocol.py](protocol.py) 编解码并拒坏帧；[udp_link.py](udp_link.py) 处理收发与序号统计。
- [rate_bridge.py](rate_bridge.py) 锁存最近规划命令，限幅、限斜率并在超时归零。规划周期和发送周期分别控制。
- [ros_bridge.py](ros_bridge.py) 支持 Twist/TwistStamped 输入，发布 `/boundary/odom`、`imu`、`joint_states` 与 `status`；无遥测、遥测过期或 fault 时只发 HOLD 零速。

框架的 simulation 域仅允许严格解析的 IPv4 回环 target/bind；localhost 规范为 127.0.0.1，拒绝远端地址、URL、缺 target、非法端口和当前 AF_INET 后端不支持的 IPv6。直接 ROS 桥也在创建 UDP socket 前复核现场私参；它需要先读取 ROS 参数，不能据此宣称任意入口都在 ROS 导入前完成检查。

ROS 桥 `~rate_hz` 缺省 500，可设 50—500。遥测发布与命令环分离。该频率是配置和运行机制，实际吞吐须看对应记录。

## 仿真端点

以下从仓根、已加载本机 Noetic/工作区环境的终端运行。先启动需要的仿真端点，再启动桥；端口只作本机示例。

```bash
PYTHONPATH=ws/src/90_platform/v8_platform python3 -B -m boundary.ros_bridge \
  _target:=127.0.0.1:5006 _bind:=127.0.0.1:5015 \
  _rate_hz:=200 _command_topic:=/cmd_vel
```

MuJoCo 端点需要已准备的 MJCF、策略和 mujoco/onnxruntime：

```bash
python3 -B ws/src/95_simulation/lite3_gazebo_sim/dynamics/boundary_endpoint.py \
  --xml /tmp/mjcf/Lite3.xml --policy /tmp/new_policy.onnx \
  --bind 127.0.0.1:5006 --target 127.0.0.1:5015 --seconds 5
```

Gazebo 端点需要已存在的模型和隔离 Gazebo 服务：

```bash
PYTHONPATH=ws/src/90_platform/v8_platform python3 -B -m boundary.gazebo_endpoint \
  --model s3_box --bind 127.0.0.1:5026 \
  --telemetry-target 127.0.0.1:5025 --seconds 5
```

Gazebo 端点使用 SetModelState 运动学位姿汇，不证明接触/步态。MuJoCo 端点是策略探测载体，默认速度上限 0.25 m/s；其接口运行不能替代 V8-X1 或真机验收。更多接触执行说明见[dynamics](../../../95_simulation/lite3_gazebo_sim/dynamics/README.md)。

## 记录、汇总与回放

```bash
PYTHONPATH=ws/src/90_platform/v8_platform python3 -B -m boundary.s2_loopback \
  --target 127.0.0.1:5006 --bind 127.0.0.1:5005 \
  --seconds 7 --command 0.2,0,0 --record /tmp/frames.jsonl
PYTHONPATH=ws/src/90_platform/v8_platform python3 -B -m boundary.recording summarize \
  --file /tmp/frames.jsonl --out /tmp/report.json
PYTHONPATH=ws/src/90_platform/v8_platform python3 -B -m boundary.recording replay \
  --file /tmp/frames.jsonl --kind command --target 127.0.0.1:5017 --rate-hz 500
```

回环接收端口须与端点 telemetry target 配对，不能同时与桥占用同一端口。报告含频率、间隔分位数、序号丢包和应用到达延迟；时延从端点单调时刻到应用接收，含线程调度，不是纯网络时延。报告中的临时目标没有因此冻结为正式判据。

离线测试：`python3 -B ws/src/90_platform/v8_platform/boundary/test_boundary.py`。运行和批准范围见[进度](../../../../../note/进度/v8/README.md)；真实 QNX/机器人参数与运动仍须单项授权。
