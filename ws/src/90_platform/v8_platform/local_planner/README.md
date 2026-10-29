# R655 原生三维局部规划备选

`local_native.plan(map, start_xyz, goal_xyz, obstacles, now, config)` 从当前支撑位置
在「支撑面 × XY 栅格 × 未来时刻」空间搜索新的有限时域运动，含等待、面内移动、
按显式有向边转层。障碍是带速度/时间窗的三维盒；整段相对运动 slab 相交核验机器人
包络与净空，不靠端点避障。输出带绝对时间、支撑面 ID、地图 ID/版本、有效期和末端
保持段；终检调用独立 `trajectory_checker`。它不消费全局点列后补时间戳。

支持图坐标 z 表示足底/支撑高度，绝非 `base_link` 高度。缺省 Config 明确是实验运动学
载体约束，不能作为 Lite3 已测能力。传入 robot profile 时，`declared + prohibited`
能力立即拒绝，并执行独立运动学核验；原有真机与第三方边界不变。

`evaluate.py` 不导入局部规划器，以独立密采样、真值障碍和现有检查器评价输出；
不会改写或修复轨迹。该密采样评价有显式最大空间/时间间距，不冒充连续碰撞证明。

```bash
python3 -B ws/src/90_platform/v8_platform/local_planner/test_local_native.py
python3 -B ws/src/90_platform/v8_platform/local_planner/test_execution.py
python3 -B ws/src/90_platform/v8_platform/local_planner/demo.py --out /tmp/a5-local.json
```

示例逐段执行速度到下一个已支撑状态、再从该状态重规划，覆盖新障碍绕行、带速度的
三维障碍、短转层三维到达、无路停车及旧轨迹拒绝。
这是 Python 运行证据，尚未因此取得 Gazebo 或接触动力学证据。滚动局部搜索可能在
需先远离目标的凹形障碍前停住，调用方应提供全局段的局部目标；本件不声称全局完备。

## ROS1 接线

```bash
python3 -B ws/src/90_platform/v8_platform/local_planner/ros_node.py \
  _map_json:=/absolute/support-map.json
```

输入：`/v8/local/pose`、`/v8/local/goal` 为 `geometry_msgs/PoseStamped`，
`header.frame_id=map`，z 必须是支撑高度；`/v8/local/obstacles` 为 `std_msgs/String`，
JSON 如 `{"obstacles":[{"min":[0.6,-0.2,0],"max":[1,0.2,0.8],"velocity":[0,0,0],"stamp":0}]}`；
`/v8/local/localization_ok` 为 `std_msgs/Bool`。
pose、obstacles、localization 都需持续心跳，缺失/过期立即输出零速。

输出：`/v8/local/cmd_vel_3d` 为 `geometry_msgs/TwistStamped`（map 系 linear.xyz），
`/v8/local/trajectory`、`/v8/local/status` 为 JSON String。
输出仍须通过载体适配与安全仲裁，不能把 3D 命令接入只消费 XY/yaw 的底盘后宣称跨层。
跨层显式边表达离散接触状态转移；现行 map.v1 没有面内连续坡面。控制器在执行已验
局部段时持续接收实际 xyz，不从中途未落面的位姿重新搜索；实到该段目标后再规划。
不能把跨层过程中的连续高度假装成新支撑面。每次发令都按当前时间和实际位置重新
核验整个剩余段，延迟指令不能继承旧预测时刻的避障结论。跨层全链仍需载体运行取证。
节点有独立墙钟控制循环，规划阻塞、ROS 仿真钟停转时也会执行零速看门狗；输入世界
变更会撤销旧输出，规划结果的 generation 复核防止并发更新后接受旧障碍预测。
`~experimental_config` 接受 Config 字段；`~robot_profile_json` 可启用已验证能力限制。
本节点未替代真机控制器，启动本节点不构成真机运行授权。

## 待目标环境执行的最小观测

在专用 roscore 已就绪且完成开窗预检后，用契约夹具
`contracts/fixtures/map_single_floor.json` 作为 `_map_json` 起节点；保持三个独立心跳：
PoseStamped `map/(0,0,0)`、Bool `true`、String `{"obstacles":[]}`，各 10 Hz；
向 goal 发一次 `map/(1,0,0)`。观测 cmd_vel_3d 的 `linear.x` 为非零，
随后停掉 obstacles 心跳，期望状态 `E_LOCAL_OBSTACLES_STALE` 且三向速度全零。
恢复心跳后加入障碍 `min=[0.6,-0.2,0], max=[1,0.2,0.8]`，应观测到新轨迹绕行；
把障碍改成封路盒并将 pose 放至 `(-0.4,0,0)`，期望 `E_LOCAL_NO_ROUTE` 和零速。
命令输出必须以独立订阅采样取数，不能仅以状态自报判过。以上是待执行说明，
尚无本片 ROS/Gazebo 运行结果；完整示例与 watchdog 的 Python 对抗证据见 demo/test。

`runtime_probe.py --out /tmp/a5-ros.json` 可在已有专用 master 上执行这些实际消息探针；
它自行启动/退净节点，含移动盒闭环、短台阶接触状态闭环、封路、定位失效、暂停规划
后有效期归零五例。`--case dynamic` 可只复跑动态反例，结果会标 `complete_suite=false`。
载体按每条实收三维指令的接收时刻做零阶保持积分，连续反馈实际 xyz；不吸附回面。
`execution_trace` 保留全部等待与命令变化，`trace` 记录实际接触事件。独立评价器用前者
核静态/动态碰撞、后者核地图支撑，逐个校验未落面的执行点是否确在已声明跨层边的
实际接触事件之间及其几何线上；缺失/不一致均判失败，不能靠丢采样洗绿。
节点继承 probe 的进程组，lab 超时清组不会遗留独立 session；正常退出只终止/回收
自身 child PID。输出明确带 `gazebo=false/dynamics=false`。
终端控制按反馈时戳和已发命令估计当前位置，扣除在途旧命令可能走过的距离，保持
实际接触状态而非吸附到参考面。`command_latency_bound_s=0.01` 是此实验载体的通信
建模界，不能替代真实部署认证或冻结停车预算；probe 逐条记录实收指令时延，实际
超界必须停止本次验证并记 FAIL，不得继续引用该界声称保证落面或运行安全。
`~pause_planning` 仅为仿真故障注入：暂停规划刷新但保留独立控制看门狗，用来量取
有效期过后实收三向零速。该参数不赋予任何真实硬件连接许可。

## 正式状态（R881 收口）

自 R881 起，本件收为 A5 的算法/申报域正式路线：只对声明支撑图 `map.v1`（平坦/离散转层）
负责，输出必须经载体适配与安全仲裁；不接真机、不声称四足接触或跨层物理、不声称全局完备。
当前 `ros_node.py` 只是实验接线，不随本路线收为生产/导航节点；`command_latency_bound_s=0.01`
仅为实验载体通信建模界，本批未实测，不替代部署认证或冻结停车预算。第三方 3D 路线未做
同基准对照，不声称优于第三方。R881 实测出的运行时缺陷必须先修：`validity_s` 的仿真/墙钟
时基未统一（RTF<1 时墙钟看门狗会先于计划的 `valid_until` 撤销输出）、零速不锁存
（出现零→非零→零）、`reason=ok` 也发零速、零后复动；修好并复测前，不得用本节点测全链
停车预算。
