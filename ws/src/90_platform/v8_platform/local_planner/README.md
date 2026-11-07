# local_planner · 原生三维局部规划与命令适配

> 最新更新时间：2026-11-07 16:32:52 CST。本次补已提交局部段支撑、边界行程、跨层中间态与仿真事件等待；验收状态以[项目进度](../../../../../note/进度/README.md)为准。

[local_native.py](local_native.py) 在“支撑面 × XY 栅格 × 未来时刻”上搜索有限时域运动，含等待、面内移动和显式有向转层。它从当前状态到局部目标重新规划，不消费全局点列后补时间戳。[path_goal_node.py](path_goal_node.py) 当前只将全局 Path 末点转成局部目标，不能描述为逐点跟踪完整全局路线。

## 核心接口与有效域

`plan(map_value, start, goal, obstacles=(), now=0.0, config=None, robot_profile=None)` 输出带绝对时间、支撑面、地图 ID/版本、有效期与末端保持段的结果。`command` 拒过期/旧地图/定位失效的结果，`track_segment` 按当前时间与实际位置核剩余已验段。

输入接受 map v1/v2/v3；静态地图 obstacles 与运行输入的带速度/时间窗三维盒一起参与包络碰撞检查，cells 用于真实支撑归属。支撑 z 表示足底/接触高度，不能直接当 base_link 高度。平面常高的面与显式 step/slope 边表达离散接触状态，跨层连续过程不制造新支撑面。

默认 Config 是实验运动学约束，不是 Lite3 定标。robot profile 中 declared+prohibited 的能力不能驱动可执行判断；场景/机器人几何须由调用方明确给出。滚动搜索在凹形或需先远离目标的障碍前可能停住，不保证全局完备。`command_latency_bound_s` 是实验通信建模界，不替代实测通信/停车认证。

[evaluate.py](evaluate.py) 不导入局部规划器，以真值障碍、密采样和现有独立检查器评价输出，不修复轨迹；有限采样间距不等于连续碰撞证明。

## 已提交段的完成与停车支撑

`track_segment` 的近点、横向越点和到期保持段三条完成出口，都要求原始实际反馈和当前命令历史预测落在声明的目标支撑面上。归属与 `plan` 共用同一选择逻辑：v3 使用实际 cells，旧无 cells 地图使用原半径缩盒，z 使用原 `support_tol_m`。还要核原始反馈→当前预测→前一命令在原 `command_latency_bound_s` 内的停车尾段全程仍有目标支撑；v3 在每个穿格事件和区间内检查，端点合法不能掩盖中间空格。原近点阈值、碰撞/不确定性检查与各 Config 门值保持。

当前点已支持、但预测或停车尾段会出格时，发零速并保留已提交段，不提前报告完成。实际反馈未归属目标支撑时，仅在实际点和预测点都处于已有 `5e-6 m` 微修正带内，才用实际反馈决定原微速修正方向，并分别检查预测与实际修正的碰撞段；同面段远距离越点失去支撑则安全拒绝；显式跨层段在中间高度水平越过目标时不提前完成或据此拒绝，而是继续跟踪到原目标支撑与完成条件成立。零长保持段不授新增纠偏能力，到期时不支持直接拒绝；未到期仍按原等待时间和碰撞检查保持，不能提前完成。微修正仍受原 `5e-5 m/s` 上限约束；当已支持的命令行程会在下一控制周期加原传输界内离开支撑时，才复用原有剩余距离/尾量储备限幅。非微速接近段还把已有 `uncertainty_m` 叠加到显式旧命令位移和新命令行程作同向支撑检查，必要时从原储备式扣除该不确定度；未获得目标支撑的一侧不使用此限幅。它不修改反馈、世界位置或地图，不扩大恢复范围。

`feedback_uncertainty` 将时间戳等于反馈源时间的已发命令切换也计入在途不确定度，覆盖零速已签发但尚未传输、旧非零命令仍在执行的区间；位姿预测和时钟合同不变。`track_segment` 在碰撞半径扩张前拒绝非有限或负不确定度；有限不确定度若已超过剩余距离、旧尾量与原机身包络之和，则直接零速保持，避免无意义的超范围格枚举。该计算安全保护不拦截 unsupported 侧的既有有限修正。

仿真 opt-in 的位姿/时钟等待使用独立 `/clock` 代数条件等待：每次资格检查前记录当前代数，回调只增加代数并唤醒等待者，不取位姿状态锁；检查前后到达的事件都不会被清除，无新事件时真正阻塞而非持续置位忙等。原 10 ms 预算、截止前资格、迟到唤醒拒绝、源时间戳和接收租约均不变。默认零等待、非仿真、真机 profile 和锁步路径不启用该事件等待。

R1127 upstairs-r3 的完成前原始点仍在地面格内，随后旧负向命令的停车尾段把它带入已从地面移除的楼梯投影格。upstairs-r4 另暴露 step07 边界接近时的预测/传输越界。对应回归使用真实场景地图和保留数值，覆盖这两条时序、源年龄预测、反向尾量、格角空洞、同刻零速切换、延迟反馈及停车后下一次规划可解析支撑。离线回归不替代修复后 Gazebo 目标运行验收。

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
| 可选输出 | `~twist_topic` | 缺省关闭；同一 base_link 原点的 yaw 平面 XY，map→body 显式旋转，拒非零竖直请求 |

pose/obstacles/localization 需持续心跳，缺失/过期输出零速。PoseStamped保留非零源时间戳，旧/未来/倒退戳及重复源戳停止推进时均失效；源年龄与单调墙钟进展年龄任一达到既有 input_max_age_s 即拒绝。重复回调不续租。输入世界变更撤销旧 generation；独立 fork worker 执行规划，控制/ROS 回调不等待长搜索。计划有效期按ROS测量时钟判，单调墙钟 `~wall_watchdog_s` 提供独立租约；终端停车锁存与可重规划瞬态零速分别处理。`~pause_planning` 仅用于故障注入。

## 仿真位姿与时钟的交付顺序

`~sim_pose_clock_wait_s` 默认0，保持原有严格未来戳拒绝。仅实验仿真可以显式启用：`/use_sim_time` 必须是真布尔 true，无 robot profile、无 plain-Twist mirror、非 lockstep；等待必须为有限非负数，不超过既有 `experimental_config.command_latency_bound_s`，且严格小于 `input_max_age_s`。非法组合在初始化时拒绝。两层场景选择0.01 s，不据此提高或替代任何既有时钟/支撑/碰撞门值。

该适配只解决不同连接上位姿可能早于本进程内置 `/clock` 到达的顺序问题。回调先持已有共享锁、保留原接收 monotonic，再最多按声明预算作 wall sleep/yield，让独立的 rospy 时钟回调追上原始 source stamp；只有预算内真实追上才进入原 guard，stamp 不改写，pose_at/progress_at 不延后。等待消耗仍计入原租约。控制线程发布前的最新位姿、generation、ROS和墙钟租约复核保持。

停钟、超时、晚醒、shutdown 和非法时钟均拒绝；到期后即使时钟已追上也不能接受。被拒 stamp 仍登记 observed_stamp，重复发送不能解冻。等待中真实回拨会通过原 guard 的 epoch 机制隔离已接受和待收旧 stamp，不直接改 common 内部状态，也不把 now 改成 source。实际时钟故障撤销的轨迹不自动复活；若停在两踏面之间，仍须明确恢复/重新初始化，不能放宽支撑高度或强行 snap 使测试通过。

显式启用时，local status 附 `pose_clock_ordering`，记录最近一次超前交付的 source/initial_ros、原接收墙钟、等待耗时、结果和累计等待/接受/超时数。它只作诊断，不授额外运动许可，也不把有限 Python 等待当作硬实时保证。默认0不增加该诊断或执行等待。该适配的目标运行结论见对应场景证据；本段不签真机或四足接触。

## 两种命令链

[cmd3d_mux_node.py](cmd3d_mux_node.py) 为实验三维 carrier 保留 map 系 xyz，检查 frame/时效/hold 后写唯一 `/cmd_vel`；它不是 Lite3 平面 mux 的同义替代。[pose_mux.py](pose_mux.py) 提供定位源交接门，相关 smoke 验证一致切换与跳变拒绝。

Lite3 app 采用 plain Twist→`/cmd_vel_raw`→生产 safe_cmd_vel_mux→`/cmd_vel`。启用plain输出时验证有限非零四元数并规范化，按 Rz(yaw) 的逆旋转 map XY 到同一base原点的机体航向平面：yaw=π/2时 map(1,0)→body(0,-1)。非零vz请求（绝对值>1e-6，仅数值零容差）、非法姿态/速度（含数值转换溢出）或源时间均在两条命令发布之前决定归零，plain六轴零；关闭plain时保留map三维运动。此适配不含完整6D参考点平移、roll/pitch动态控制或支撑高度补偿，目标机/爬阶能力尚未验证。完整配置与限速/感知边界见[Lite3 配置](../configs/lite3/README.md)。

轨迹计算后，在与输入回调共用的锁内重新核generation、最新位姿/姿态、所有心跳及原有ROS/墙钟租约，再双发布和记录最终命令。header与反馈积分history记录实际发布时刻，锁步tick只选择计算时刻；/clock事件等待超时仍发零速。位姿撤销及时间epoch改变均清旧history/轨迹，即使失效后在两次控制tick之间恢复也不能复用旧反馈预测；worker结果的最终接受/轨迹发布同样与回调撤销互斥，旧generation不能恢复旧结果。被拒样本不续租，同一未来样本不得仅靠ROS追平恢复；新epoch经新的有效源推进恢复后不永久封禁旧stamp数值。新鲜源推进后可对现有goal重新规划。已经锁存的终端停止继续保持，只有新的map frame goal被接受才解除；这不宣称已完成goal全部几何校验。公共纯策略见[pose_safety.py](../common/pose_safety.py)。

## 测试和运行探针

```bash
python3 -B ws/src/90_platform/v8_platform/local_planner/test_local_native.py
python3 -B ws/src/90_platform/v8_platform/local_planner/test_execution.py
python3 -B ws/src/90_platform/v8_platform/local_planner/test_ros_node_policy.py
python3 -B ws/src/90_platform/v8_platform/local_planner/test_pose_command_adapter.py
python3 -B -m unittest discover -s ws/src/90_platform/v8_platform/common
python3 -B ws/src/90_platform/v8_platform/local_planner/test_planner_worker.py
python3 -B ws/src/90_platform/v8_platform/local_planner/test_cmd3d_arbiter.py
python3 -B ws/src/90_platform/v8_platform/local_planner/demo.py --out /tmp/a5-local.json
```

`runtime_probe.py --out /tmp/a5-ros.json` 在已有专用 master 上创建/退净节点，含移动盒、短转层、封路、定位失效、规划刷新暂停探针；`--case dynamic` 仅跑子集并标 complete_suite=false。它用实收命令零阶保持积分和完整 execution_trace 评价，明确 gazebo=false/dynamics=false，不能当接触动力学证据。

Gazebo 动态方盒、真实激光停车和 MuJoCo 单策略接触执行已有各自历史运行记录，见[进度](../../../../../note/进度/v8/README.md)；不能继续使用早期“尚无任何 ROS/Gazebo 结果”的说法，也不能由局部实验扩大为生产、任意四足步态或真机验收。本文未复跑这些实验。
