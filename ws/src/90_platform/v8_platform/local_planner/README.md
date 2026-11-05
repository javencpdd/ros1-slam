# local_planner · 原生三维局部规划与命令适配

> 最新更新时间：2026-11-05 11:49:54 CST。本次为文档与源码核对；验收状态以[项目进度](../../../../../note/进度/README.md)为准。

[local_native.py](local_native.py) 在“支撑面 × XY 栅格 × 未来时刻”上搜索有限时域运动，含等待、面内移动和显式有向转层。它从当前状态到局部目标重新规划，不消费全局点列后补时间戳。[path_goal_node.py](path_goal_node.py) 当前只将全局 Path 末点转成局部目标，不能描述为逐点跟踪完整全局路线。

## 核心接口与有效域

`plan(map_value, start, goal, obstacles=(), now=0.0, config=None, robot_profile=None)` 输出带绝对时间、支撑面、地图 ID/版本、有效期与末端保持段的结果。`command` 拒过期/旧地图/定位失效的结果，`track_segment` 按当前时间与实际位置核剩余已验段。

输入接受 map v1/v2/v3；静态地图 obstacles 与运行输入的带速度/时间窗三维盒一起参与包络碰撞检查，cells 用于真实支撑归属。支撑 z 表示足底/接触高度，不能直接当 base_link 高度。平面常高的面与显式 step/slope 边表达离散接触状态，跨层连续过程不制造新支撑面。

默认 Config 是实验运动学约束，不是 Lite3 定标。robot profile 中 declared+prohibited 的能力不能驱动可执行判断；场景/机器人几何须由调用方明确给出。滚动搜索在凹形或需先远离目标的障碍前可能停住，不保证全局完备。`command_latency_bound_s` 是实验通信建模界，不替代实测通信/停车认证。

[evaluate.py](evaluate.py) 不导入局部规划器，以真值障碍、密采样和现有独立检查器评价输出，不修复轨迹；有限采样间距不等于连续碰撞证明。

## ROS 输入输出

从仓根、已加载工作区环境且专用 master 就绪时运行：

```bash
python3 -B ws/src/90_platform/v8_platform/local_planner/ros_node.py \
  _map_json:=/absolute/support-map.json
```

也可用 `~map_topic` 消费版本化 JSON 地图；两者至少提供一个。缺省接口：

| 方向 | 话题 | 契约 |
| --- | --- | --- |
| 输入 | `/v8/local/pose`、`/v8/local/goal` | PoseStamped，frame=map，z 为支撑高度 |
| 输入 | `/v8/local/obstacles` | String JSON，例 `{"obstacles":[]}`；可含 min/max/velocity/stamp |
| 输入 | `/v8/local/localization_ok` | Bool |
| 输出 | `/v8/local/cmd_vel_3d` | TwistStamped，map 系 linear.xyz |
| 输出 | `/v8/local/trajectory`、`/v8/local/status` | String JSON，含结果/状态/地图身份 |
| 可选输出 | `~twist_topic` | 缺省关闭；plain Twist 镜像，供指定实验/安全链使用 |

pose/obstacles/localization 需持续心跳，缺失/过期输出零速。输入世界变更撤销旧 generation；独立 fork worker 执行规划，控制/ROS 回调不等待长搜索。计划有效期按仿真时刻判，墙钟 `~wall_watchdog_s` 处理 /clock 停转；终端停车锁存与可重规划瞬态零速分别处理。`~pause_planning` 仅用于故障注入。

## 两种命令链

[cmd3d_mux_node.py](cmd3d_mux_node.py) 为实验三维 carrier 保留 map 系 xyz，检查 frame/时效/hold 后写唯一 `/cmd_vel`；它不是 Lite3 平面 mux 的同义替代。[pose_mux.py](pose_mux.py) 提供定位源交接门，相关 smoke 验证一致切换与跳变拒绝。

Lite3 app 采用 plain Twist→`/cmd_vel_raw`→生产 safe_cmd_vel_mux→`/cmd_vel`。目前 ros_node 的 plain Twist 直接镜像 map 系速度，没有独立按机器人 yaw 旋转到机体系；接入平面底盘前须核清坐标语义，不能凭静态/轴对齐冒烟声称任意朝向真机导航完成。完整配置与限速/感知边界见[Lite3 配置](../configs/lite3/README.md)。

## 测试和运行探针

```bash
python3 -B ws/src/90_platform/v8_platform/local_planner/test_local_native.py
python3 -B ws/src/90_platform/v8_platform/local_planner/test_execution.py
python3 -B ws/src/90_platform/v8_platform/local_planner/test_ros_node_policy.py
python3 -B ws/src/90_platform/v8_platform/local_planner/test_planner_worker.py
python3 -B ws/src/90_platform/v8_platform/local_planner/test_cmd3d_arbiter.py
python3 -B ws/src/90_platform/v8_platform/local_planner/demo.py --out /tmp/a5-local.json
```

`runtime_probe.py --out /tmp/a5-ros.json` 在已有专用 master 上创建/退净节点，含移动盒、短转层、封路、定位失效、规划刷新暂停探针；`--case dynamic` 仅跑子集并标 complete_suite=false。它用实收命令零阶保持积分和完整 execution_trace 评价，明确 gazebo=false/dynamics=false，不能当接触动力学证据。

Gazebo 动态方盒、真实激光停车和 MuJoCo 单策略接触执行已有各自历史运行记录，见[进度](../../../../../note/进度/v8/README.md)；不能继续使用早期“尚无任何 ROS/Gazebo 结果”的说法，也不能由局部实验扩大为生产、任意四足步态或真机验收。本文未复跑这些实验。
