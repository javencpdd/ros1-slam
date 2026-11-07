# framework · YAML 组件图与导航应用

> 最新更新时间：2026-11-08 03:05:24 CST。本次补 Gazebo carrier 的支撑分段积分；验收状态以[项目进度](../../../../../note/进度/README.md)为准。

framework 用 manifest、graph 和 app 配置装配 ROS1 进程。它负责配置检查、启动/健康/停止与应用入口；算法数据走 ROS 话题，规划控制与边界发送不经过 Python 编排循环。装配成功不等于算法或机器人能力通过验收。

活动 Gazebo carrier 与 Gazebo/MuJoCo verification manifest 从 [v8_simulation](../../../95_simulation/v8_simulation/README.md) 启动，仿真依赖由 companion 声明；核心源码仍在本包供薄入口调用。原同名测试路径保留，source/devel 之外的 install-only 用法尚未验证。

## 输入与阅读顺序

| 输入/实现 | 当前职责 |
| --- | --- |
| [fixtures/manifests](fixtures/manifests) | manifest v3 运行组件：显式 execution.modes/robot_id、命名 ports、dimensions、launch；同时包含真实节点、include 与明确的测试替身 |
| [fixtures/graphs](fixtures/graphs) / [config.py](config.py) / [graph.py](graph.py) | pipeline_graph/v1：实例、namespace、参数/config、显式端口连线、外部输入、反馈策略；生成启动 DAG 和 digest |
| [render.py](render.py) / [runtime.py](runtime.py) | assembly_plan/v2、受域校验的 launch、默认运行命令与 Assembly；消费现有 ProcessRunner/Orchestrator |
| [app_config.py](app_config.py) / [app_runner.py](app_runner.py) | navigation_app/v1：graph、地图 registry、任务、定位/载体与可视化；dry-run/execute |
| [map_registry.py](map_registry.py) / [surface_resolver.py](surface_resolver.py) | map_id/version/path/sha 绑定，起终点 z 与 surface_hint 消歧 |
| [map_from_pcd.py](map_from_pcd.py) / [inflate_support_map.py](inflate_support_map.py) | PCD→支撑图/任务/app/本机登记；声明足迹外扩生成单独变体 |
| [safety.py](safety.py) / [hotswap.py](hotswap.py) | 安全接线、否决停止、受限实例替换 |
| [robot.py](robot.py) / [verification.py](verification.py) | robot profile 能力检查；与算法图分开的验证后端/真值通道 |

组件只有 manifest 一份定义，不另维护 components.yaml。v2/v3 的兼容 io 由 ports 加载期派生；v1 manifest 仍供旧 registry/composer 使用，但不能进入命名端口图（`E_CFG_MANIFEST_V1`）。同组件可创建多个实例，实例 ID 与 ROS 节点名不能冲突。

## 最小使用

以下从仓根执行，check/plan 不启动 ROS：

```bash
python3 -B ws/src/90_platform/v8_platform/framework/cli.py check \
  ws/src/90_platform/v8_platform/framework/fixtures/manifests \
  ws/src/90_platform/v8_platform/framework/fixtures/graphs/graph_3d_cache.yaml \
  --package-index ws/src/90_platform/v8_platform/framework/fixtures/package_index.json
python3 -B ws/src/90_platform/v8_platform/framework/cli.py plan \
  ws/src/90_platform/v8_platform/framework/fixtures/manifests \
  ws/src/90_platform/v8_platform/framework/fixtures/graphs/graph_3d_cache_sim.yaml \
  --out-plan /tmp/plan.json --out-launch /tmp/graph.launch
python3 -B ws/src/90_platform/v8_platform/framework/app_runner.py \
  --app ws/src/90_platform/v8_platform/framework/fixtures/apps/nist_sensor_3d.yaml
```

最后一条缺省 dry-run；地图文件与 registry 必须在本机。`--execute` 会真实起进程；缺省另起隔离 roscore，可通过 `--port` 指定端口，Gazebo master 使用其下一端口。`--no-gazebo` 只跳过 Gazebo，仍会另起 roscore；`--real` 才沿用现场 master/传感器，不能把两者混淆。`--interactive --rviz` 启用选点和显示，Ctrl+C 收尾。运行前按[作业单](../../../../../note/计划书v8.1/25_v8.1执行与迁移规则.md)准备环境，真实机器人连接与运动须已有对应授权。

## graph 检查与实际启动

检查命名端口、方向、消息类型/schema、必需输入与重复接线、非反馈边环、命令双写者、Path 两端维度、配置缺失/重复键/类型与 launch 规格。反馈边必须标 `feedback` 和策略，不参加启动 DAG。`--package-index` 是静态声明快照，不能证明真实包可执行或插件实现可用。

launch 支持：

- node：rosrun package/type，实例参数作为私有参数。
- include：roslaunch package/launch_file，实例参数合并为 launch args；不接受 namespace 或实例 config。
- plugin：nodelet standalone；实例名/namespace/参数传给宿主。
- nodelet_group：外部 manager 与多个 nodelet，由临时 launch 装载。

实例 `config` 相对 graph 文件解析，须为 YAML/JSON 映射，内容哈希进入 digest。node/plugin 的 config/param_files 已通过 [param_exec.py](param_exec.py) 预载 ROS 私有参数后 exec rosrun，graph params 最后覆盖；旧“只渲染、实际不加载”的限制已不适用。实际参数优先级为 config → manifest param_files → 实例 params。后端 selector 会检查有效值并将默认值写入最终参数；selector 与无法离线解析的 param_files 组合会拒绝，param_schema 的其他字段仍只登记。

manifest 的 node/plugin remap 会追加到命令；图边 remap 仍不能视为通用运行期自动改线。include 无通用 remap；nodelet 在既有探针中未按传入 remap 改题名，端口 topic/namespace 必须与真实发布订阅一致。临时 launch/参数规格文件不会由当前 default_command 自动回收。

Assembly 真正启动进程，条件就绪/失败清理/运行健康/停止链由 [orchestrator](../orchestrator/README.md) 承担；调用方须驱动健康检查。mux 最后停不替代消息级实时看门狗。`app_runner` 对缺失或空全局路径生成空摘要，不取末点；仿真退出先通过既有 hold 通道请求 mux 零速并留出观测时间，再停止 Assembly，不新增 `/cmd_vel` 旁路。

## 运行模式与机器人身份

app/graph 成对声明 `execution_mode: simulation|real_robot` 与 `robot_id`；真实模板固定 `real_robot/lite3`，M20 尚无运行后端。组件 v3 的 `execution.robot_id: null` 显式表示不绑定机器人；缺声明代表未知，不能推导硬件支持。evidence、tags、CPU 架构与执行许可均不替代模式声明。

`prepare_execution()` 在环境修改、mkdir、日志文件、包发现、ROS 导入或 Popen 前完成选图、载体映射、参数填充、图/安全/域检查。`--real` 必须与显式 real_robot/lite3 app 一致，复用现场 master；Gazebo/GT/未知后端、错误机器人或仿真组件会拒绝。CLI 的 `--real` 不请求 Gazebo；`--no-gazebo` 只用于 simulation，不能让 Lite3 模板伪装成容器仿真。

Assembly（含自定义命令工厂）、独立 default_command、三条 hotswap apply 和可执行 render_launch 均复核同一守卫。graph.check 的独立快照保存原始图输入、所选组件摘要与报告摘要，组件摘要进入 graph digest；render_plan/launch 必须复核该原始绑定，旧无绑定报告须重新 check，不能用变化后的 catalog 为旧报告重新盖章。计划保存独立组件摘要及 `execution_sha256`，绑定上下文、catalog 和选中组件身份；检查后修改 catalog/模式/机器人会拒绝。该摘要用于兼容一致性，不是操作许可或完整计划认证。热替换不能改变部署身份，记录与新旧实例不符时在停旧进程/回调/工厂之前拒绝。

旧 v2 manifest 和未分类图可做 check/plan 离线分析（CLI plan 仅给 --out-plan；请求 --out-launch 会额外验证可执行域）；执行要求 v3 的显式域，旧 assembly_plan/v1 不直接启动。旧合法仿真仅在所有选中组件明确支持 simulation 且不绑定机器人时窄兼容为 generic_sim。`graph_2d*` 的五个旧混合模板含 ros2qnx，保留离线分析并拒绝启动/可执行 XML；运行测试改用 `_sim` 副本。`graph_3d_real*` 中 real 指 ROS 实现，其载体仍为运动学仿真。

统一守卫也前置于可传图的 graph_3d_assembly_smoke、runtime_lab 与 boundary_chain_runtime；实际 smoke 不属于离线测试。外部 bringup/任意手工 rosrun 不受 Python 装配守卫统管，现场已有运动桥仍须按现场授权处理。本轮没有启动 ROS、仿真或硬件。速度坐标/位姿源时间已有下述代码与离线正反控；图边自动接线等其他缺口继续保留，目标环境能力不由静态检查晋级。

## 当前运行接线

- 2D：`v8.nav2d.*` include 启动实际导航栈与生产 mux；GlobalPlanner/Navfn、DWA/TEB 选择由图参数表达。
- 3D：map provider→全局 topic planner→path_goal→local_native→三维 mux/载体。`path_goal_node.py` 仅把全局 Path 末点转为局部目标；原生局部规划不逐点消费整条全局路线。
- `gazebo_carrier3d_node.py` 是 static box/SetModelState 的运动学载体，含支撑高度钳制和同高度支撑边界分段积分；零 z 速度命令不会积分出当前 v3 cell run/footprint，显式 z 转层仍可按命令跨 XY 边界。Gazebo 动态方盒和 MuJoCo 四足接触实验为不同执行链，不能互代。Lite3 carrier 只提供 TF/odom 位姿与健康：原非零源stamp、精确parent/child、有限几何必须有效；重复查询不续租，ROS源年龄和monotonic源推进年龄共同判失效。错误odom坐标直接拒绝，不改名为map；非阻塞TF查询和墙钟节拍保证停钟时仍能发布健康false，无效时不重播旧pose。最终check/pose/health/status与回调撤销共用锁；拒绝样本的stamp也被观察，同一个未来样本不能靠ROS追平恢复；ROS回退后先隔离旧缓存，只有新的有效源推进才解除隔离，正常新epoch可连续跨过旧stamp数值。数值与现场参数未改。纯策略见[公共helper](../common/pose_safety.py)，测试见[test_lite3_carrier.py](test_lite3_carrier.py)，目标TF/支撑参考点仍见[现场配置](../configs/lite3/README.md)。
- localization_node 支持 sim_gt/faster_lio/ndt 来源；NDT `navigation_allowed=false`。GT-LIO raw/translated/rigid 比较在 localization_compare，不把事后对齐当独立精度认证。
- `map_cache_node`/`map_merge_node` 和 typed v8_msgs 已有真实图入口；旧 graph_3d_cache 使用测试替身。MapDelta/持久化/语义桥细节见[representation](../representation/README.md)。
- PCD 建图可生成 app/task 并登记本机地图；`--register` 修改本机 registry。地图和 `*.local.app.yaml` 不作为跨机固定资产。完整建图/可视化步骤见[Docker 指南](../../../../../docker/README.md)。

## 安全、独立验证与替换

`safety.check_plan` 允许只读图没有 `/cmd_vel` 写者；若有则最多一个且必须为 safety_arbiter。其他带 command_topic 的输出必须连到该仲裁器，否则拒绝。trajectory_validator 最多一个，须有 path 输入/输出并在图上；`enforce(assembly, verdict)` 仅显式 approved=true 放行，其余否决走装配停止链。

`verification.py` 将模拟器/真值后端放在算法图之外，拒绝真值话题与算法端口重叠及 real_robot 计划配仿真 backend。Gazebo 静态物体与 MuJoCo 落体后端证明验证通道可运行，不等于已完成场景化算法轨迹评价。`perf.py` 只测启动前 check+plan 开销，不测控制周期。

`selector.py ... --out-html /tmp/selector.html` 生成候选下拉与预计算拒绝报告；导出的组合仍须 cli check。示例见[framework-selector.html](../../../../../docs/architecture/framework-selector.html)。

`hotswap.prepare` 核同角色/端口与命令所有权；apply 停整图，apply_instance 先停目标再启替换，失败回滚；状态经 V8_HOT_SWAP_* 临时文件/环境传递，组件须自己读取。apply_overlap 仅允许无命令、无 TF 的只读 provider，先确认新实例就绪再停旧实例。命令/位姿源另有专用 mux 交接探针，不能据此称任意命令/TF 链热换已实现零中断。

## 定向检查与证据

```bash
python3 -B ws/src/90_platform/v8_platform/framework/test_execution.py
python3 -B ws/src/90_platform/v8_platform/framework/test_framework.py
python3 -B ws/src/90_platform/v8_platform/framework/test_runtime.py
python3 -B ws/src/90_platform/v8_platform/framework/test_preflight.py
python3 -B ws/src/90_platform/v8_platform/framework/test_safety.py
python3 -B ws/src/90_platform/v8_platform/framework/test_navigation_app.py
python3 -B ws/src/90_platform/v8_platform/framework/test_hotswap.py
```

真实运行 smoke 如 plugin/nodelet_group、map_cache_graph、graph_3d_real、Gazebo/MuJoCo verification 会启动外部进程，按作业窗执行；typed 地图 smoke 须先构建 [v8_msgs](../../v8_msgs)。当前设计见[v8.1架构与接口](../../../../../note/计划书v8.1/22_架构与接口契约.md)，历史收口记录见[27 号框架补丁](../../../../../note/计划书v8/27_框架改造补丁-v0.1.md)、[v8 进度](../../../../../note/进度/v8/README.md)。本文不复用旧固定测试条数，也不新增 formal/真机结论。
