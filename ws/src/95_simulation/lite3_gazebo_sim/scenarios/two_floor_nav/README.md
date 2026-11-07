# 两层室内 Gazebo 规划测试世界

> 最新更新时间：2026-11-07 19:38:40 CST。工程标准真源：[工程标准](../../../../../../note/架构/工程标准.md)。本场景使用 x86/ROS1/Noetic/Gazebo Classic 11；不安装到真机、不调用真实运动驱动。

一套自持的两层室内世界，包含楼板、楼梯、家具、可绕障碍及楼梯完全阻断反例。world、地图、任务和启动配置均随 Git 分发，不需要下载外部模型，也不依赖本机 `data/maps/registry.json`。入口是 [two_floor_nav.py](../../../../../../scripts/sim/two_floor_nav.py)。

## 内容与执行范围

- 房间 16×12 m，底层支撑面 z=0；西侧上层为 x∈[-8,-4]、y∈[-6,6]，顶面 z=1.6 m。
- 10 级楼梯，每级高 0.16 m、踏深 0.8 m、宽 2 m；上下层同 XY 支撑面同时存在。
- 墙、家具、上层栏杆和绕障箱具有匹配的 visual/collision 几何；blocked 世界在第 5 踏面放置贯穿全宽的障碍。
- [scene.py](scene.py) 是同源生成器。[geometry](geometry/open.json)、[world](worlds/open.world)、[map](maps/open.json) 和场景私有 [registry](registry.json) 由相同几何产生。地图使用 v3 精确 cells，地面移除被楼梯覆盖的投影；踏面不外扩。
- 规划实验包络 radius=0.30 m、height=0.50 m、clearance=0.05 m；全局显式 radius=0.35 m（含余量）。这是 `generic_sim` 声明包络，包住当前载体 visual，不能当成经过测量或验证的真实 Lite3 包络。

载体复用现有 `gazebo_carrier3d`：按速度积分后用 SetModelState 更新一个 visual-only 方盒。世界有碰撞几何，载体没有接触物理/足端，因此本测试验证已知地图中的规划、命令和运动学执行；台阶接触转移仍是运动学近似，不证明四足防坠、步态或真实上楼能力。定位输入为载体真值，不是 SLAM/LIO。RViz 中可显示 Lite3 模型，Gazebo 中的执行载体仍是该方盒。

现有 `path_goal` 适配器只把完整全局路径的末点交给局部规划器。该场景分别检查全局和局部跨层/避障，并保存完整全局路径；不据两者都到达宣称局部遵守完整全局走廊。

场景显式设置 `sim_pose_clock_wait_s=0.01`：位姿消息先于本节点 `/clock` 到达时，在既有命令延迟包络内等待时钟追上，再按原源时间戳和原接收墙钟检查；超时、停钟或回拨仍拒绝。默认适配器关闭该等待，真机、plain Twist、机器人 profile 和锁步模式禁止启用。app_runner 在自有 Gazebo master 上先选择模拟时间，再初始化 ROS 节点。

场景还显式启用 `sim_contact_egress=True`，默认组件配置为关闭。仅模拟时间、无机器人profile且无plain-Twist时，已实际执行并认证的下降step可保留原受检轨迹中紧接的一条同落脚面段，以近似表达旧阶面接触的退出；原结果及有效期不变。只给有单调分离证明的旧源面保留有限例外，其他障碍仍检查；原段合法完成且实际/预测/在途和不确定性包络全清后才重新规划。凭据换代失效、退出后不能复活。这是运动学近似，不是真足触传感或四足动力学。

## 默认离线检查

在项目根运行，不启动 ROS、Gazebo、Docker，也不修改现场地图或 registry：

```bash
python3 -B scripts/sim/two_floor_nav.py
python3 -B scripts/sim/two_floor_nav.py --case downstairs
python3 -B scripts/sim/two_floor_nav.py --case upper_detour
python3 -B scripts/sim/two_floor_nav.py --case blocked
python3 -B ws/src/95_simulation/lite3_gazebo_sim/scenarios/two_floor_nav/test_scene.py
```

默认检查生成物未漂移、私有 registry、执行域、graph/plan 与全局规划。blocked 必须得到精确 `E_PLAN_NO_ROUTE`，不能用其他异常替代。离线测试还覆盖上下楼重复局部重规划、同 XY 不同楼层、真实几何/cells 对齐和独立观察负例；合成 trace 正控只验证检查器，不是引擎证据。

## 在已准备的 x86 环境启动

先使用本机已经核定的 Noetic 与 `x86_sim` 目标构建环境；入口不自动 source 旧 devel、不自动构建或启动 Docker。按现行作业单登记独立仿真窗口及回执目录，回执 `_window_receipt.tsv` 首列是唯一窗口 ID，第三列必须为 `sim`，第二列是存在且不逃出仓根的相对目录，末列包含 `two_floor_nav case=<当前case>`。

入口核回执、执行域和空闲端口，再调用原件 `scripts/verify/v8_window_preflight.sh`；rc 非 0 立即拒绝，不使用 FORCE、不改负载门或 ISO。仅通过预检才创建 ROS/Gazebo 进程。每次调用在新进程运行一例，使用新输出路径保留上次证据。

```bash
mkdir -p /tmp/two-floor-nav-example
python3 -B scripts/sim/two_floor_nav.py --execute --case upstairs \
  --window-id YOUR_REGISTERED_WINDOW_ID --port 11657 \
  --out /tmp/two-floor-nav-example/upstairs.json
```

将 `--case` 换成 `downstairs`、`upper_detour` 或 `blocked` 可跑其他用例。前三者默认观察上限 180 s；blocked 默认 20 s，必须有规划器明确拒绝、密集六轴零命令和全程位姿漂移不超过 0.10 m，超时本身不算通过。输出文件和 trace/runtime 目录必须是新的。

GUI/RViz 交互选择目标：

```bash
python3 -B scripts/sim/two_floor_nav.py --execute --case upstairs \
  --window-id YOUR_REGISTERED_WINDOW_ID --port 11657 --gui --rviz \
  --out /tmp/two-floor-nav-example/interactive.json
```

在 RViz 使用 Publish Point 选择支撑面；Ctrl-C 请求关闭。交互模式只签会话就绪/记录完整性，不签预设任务的上楼到达。纯 headless 启动不需要 DISPLAY；GUI/RViz 需要可用图形环境。

## 输出与过程检查

`--out run.json` 对应 `run.app.json`、`run.trace.json`、`run.preflight.log` 和新的 `run.trace.json.runtime/`。runtime 内独立保存 ROS_HOME、ROS 日志和进程日志，不追加旧 `/tmp/r1103_*`。总报告区分原 app 结论与场景过程验收；blocked 的 app rc6 只在精确拒绝和零运动等检查均通过后才成为场景正控。

显式 trace 记录完整全局 Path、global/status、局部轨迹/状态、最终六轴 cmd、ROS pose、Gazebo model_states 以及限频 `/clock`。model_states 按接收墙钟最多 50 Hz 采样当前消息，队列为 1，不积压旧观测；采样周期记入 metadata。源 pose stamp 与回调接收 monotonic 分别保留，不能把 ros_time、模拟时间和墙钟混算。新增场景检查包括：

- 观测首尾覆盖、每路最大间隙 0.25 s、Gazebo/ROS 全程至少 95% 配对及身份/坐标/源时间合法；缺样、NaN、回调异常或有界缓存溢出均失败。
- 按载体原有 latest-command/0.5 s 失效语义在接收 monotonic 时间积分；累计 XY 差不超过 0.15 m，Z 允许原有十次每次最多 0.03 m 的 snap 加 0.05 m 观测余量。此近似明确限定运动学接触转移，不当成四足物理精度。
- 全局、局部、实际位姿的完整连续段对真实几何求交；同时核载体 visual 和声明规划包络。安全端点不能掩盖中间穿箱。
- 与原 task 目标及其 0.20 m 声明容差比较，上下楼必须经过全部台阶高度；绕障要求可观测绕行；终止命令核完整六轴。

这些是本新场景的检查，不替换历史正式门禁、门值或原实验结论。目标环境是否已执行及独立结论以[本次交付记录](../../../../../../note/进度/00_完整Gazebo规划场景_2026-11-06.md)为准。

## 维护

```bash
python3 -B ws/src/95_simulation/lite3_gazebo_sim/scenarios/two_floor_nav/scene.py
python3 -B ws/src/95_simulation/lite3_gazebo_sim/scenarios/two_floor_nav/scene.py --write
```

无参数只检查；`--write` 明确重建本场景生成物。改几何需同时重建并重验 map/world/task/registry 及过程负例，不修改旧 demo/NIST/第三方资产。原生 box 世界不代表建筑测绘数据。
