# framework · YAML 驱动的 ROS1 组件图（v0.1）

> 2026-10-31 起。做**配置、装配计划与运行装配**：组件目录、实例、命名端口、显式连线、
> 启动依赖、命令唯一写者、地图 schema 兼容、锁摘要、ROS1 launch 骨架、运行器接线、
> MapDelta 事务、机器人 profile、独立验证配置、安全接线与启动开销基准。
> **不加载具体算法、不改现有判据/运行链。** 数据面仍走 ROS 话题，Python 编排环只在启动/停止。

## 1. 组件清单 + 图 YAML

- **组件源＝registry manifest v2**（`manifest.v2.json`）：只有 `ports`（命名输入输出、
  消息类型、schema、topic、command_topic）、`dimensions` 与 `launch`（package/type）；
  `io` 不再落盘，由 registry/composer 加载时从 ports 派生（v1 清单继续带 io 且合法）。
  v1 无命名端口，不能进入图（`E_CFG_MANIFEST_V1`）。
- `graph.yaml`：选择实例、参数、namespace、显式 `from: instance.port` → `to: instance.port`、
  外部输入，以及反馈边与策略。

`launch` 支持 `kind: node`（package/type/param_files/remap）、`kind: include`
（package/launch_file/args）与 `kind: plugin`（nodelet standalone）；
`--package-index` 用包索引校验 package/launch file/executable/param file/插件包，
不启动 ROS。

`runtime.py` 把 assembly plan 转成 `ComponentSpec` 并交给现有 `ProcessRunner/Orchestrator`；
默认命令为 `rosrun`/`roslaunch`，测试注入假命令。include 的 manifest/graph 参数以
`key:=value` 传给 `roslaunch`，node 的 graph `params` 以 `_key:=value` 私有参数传给 `rosrun`；
嵌套路径 launch 先用 `rospack find` 解析为绝对路径（`roslaunch pkg nested/file.launch` 只认
basename）。plugin 以 `rosrun nodelet nodelet standalone pkg/Class` 启动，实例 id 作为
`__name`，graph namespace 作为 `__ns`，params 仍是 `_key:=value`；宿主固定为 `nodelet`。
manifest 的 `launch.remap` 对 node/plugin 都以 `from:=to` 追加到运行命令；实测本环境
`nodelet standalone` 不消费这些 remap（topic 仍随节点名），插件端口需用命名空间或
manifest 端口 topic 与真实名一致。
启动、READY 就绪、停止序（mux 最后）、失败清理与健康检测均有测试。

真实 2D 栈已接入：`v8.nav2d.dwa_stack`/`v8.nav2d.teb_stack` 是 include 组件，
`v8.nav2d.mux` 拉起真实 `60_safety.launch`，四张 `graph_2d_stack_*.yaml` 只改
`global_planner`/`local_planner` 参数。R981 在 Gazebo m1/SC09/seed0 上由 Assembly
启动 nav+mux，四格（GlobalPlanner/Navfn × DWA/TEB）全 PASS；`ProcessRunner` 在停止前
快照 roslaunch 后代并对其独立会话发信号，否则只杀 roslaunch 会遗留各节点。

`robot.py` 读取 robot.v1 profile：declared/prohibited 值不得驱动规划（`E_ROBOT_UNVERIFIED`），
仅 tested/allowed 值可用；robot_adapter 必须提供 `cmd`（TwistStamped）与 `odom`/`imu` 遥测端口。
`v8.fw.robot_lite3` 与 `v8.fw.robot_test_b` 同端口，仅改图 YAML 的实例组件即可切换。

`verification.py` 选择独立验证后端（模拟器组件）与场景，**不进入算法图**；真值通道不得与算法话题
重叠（`E_VERIFY_TRUTH_LEAK`）；后端通过注入 runner 运行，失败只记 status=error，不改算法 digest。
两个假后端验证切换；R1006 增加真实 Gazebo 后端 `v8.verify.gazebo_truth`：headless
empty world + `/gazebo/model_states` → `/v8/verify/truth`，由 `verification.run` 注入
runner 启动并等待 truth；算法图不含该后端，truth 话题也不在算法端口集合中。

`map_service.propose_delta` 接收 `map_delta/v1`：delta_id 幂等重放、旧 base/同 id 异内容/无变化
拒绝；结果先过 map 契约再原子升版并通知监听者，失败不改变当前版本。
`snapshot/save/load` 持久化当前地图、证据层、history 与 applied_deltas：临时文件+fsync+
原子替换，恢复时重算 digest 并核对 history 末行，坏文件/异 map_id/回退 fail-closed。
`delta_map_node` 的 `~state_path` 在每次成功 delta 后落盘，ROS 重启恢复版本与幂等表（R983）。
`MapStore(root)` 以 map_id 为文件名键管理多图快照：显式 `load/ids/load_all`，路径穿越与
坏文件 fail-closed（R989）；ROS provider 仍单 map_id。

`v8.boundary.adapter` 把 mux 的 `/cmd_vel` 接到边界 UDP：`ros_bridge.py` 支持
`command_type=twist|twist_stamped`，`snapshot` 图 `graph_boundary_mux.yaml` 用真实
`v8.nav2d.mux` + 边界节点；R984 已用 Gazebo 运动学端点跑通 mux→UDP→端点→遥测回链；R988 把遥测发布移出命令环，
200 Hz 请求达 202.8 Hz、500 Hz 请求达 485.8 Hz（6 s 窗）；R991 500 Hz 60 s 窗达 500.4 Hz，零丢帧。

`safety.py` 是接线门：`/cmd_vel` 必须恰好一个写者且角色为 `safety_arbiter`；其他
`command_topic` 输出必须显式连到该写者，否则 `E_SAFETY_ADVICE`；`trajectory_validator`
最多一个，且必须同时有 path 入/出边。`Assembly.start` 启动前强制此检查
（`E_SAFETY_ARBITER`/`E_SAFETY_ADVICE`/`E_SAFETY_VALIDATOR`）；validator 否决经
`safety.enforce(assembly, verdict)` 直接走停止链（mux 最后），不是只记日志。

`perf.py` 只测**启动前** check+plan 的开销（p50/p95/max，`startup_only=True`）。50 Hz 规划/控制
走 ROS 话题，500 Hz 命令发送在边界适配器专用进程（33B UDP），都不经过 Python 编排环；
本基准不冒充控制周期时延。

不再维护独立的 `components.yaml`：组件定义只有 manifest 一份；v2 的 io 是**加载期派生视图**，不落盘。

对应关系：**组件类型可复用，实例 ID 唯一；同名组件可起多个实例；连线必须显式到端口。**

## 2. 命令

```bash
cd ws/src/90_platform/v8_platform
python3 -B framework/cli.py check framework/fixtures/manifests \
  framework/fixtures/graphs/graph_3d_cache.yaml \
  --package-index framework/fixtures/package_index.json
# 每实例配置文件：graph 的 config: 指向 YAML/JSON；运行前解析并计入 digest
python3 -B framework/cli.py check framework/fixtures/manifests \
  framework/fixtures/graphs/graph_2d_config.yaml
python3 -B framework/cli.py plan framework/fixtures/manifests \
  framework/fixtures/graphs/graph_3d_cache.yaml \
  --out-plan /tmp/plan.json --out-launch /tmp/graph.launch
python3 -B framework/test_framework.py      # 26 项
python3 -B framework/test_runtime.py        # 9 项（F3 假命令/真实 include 参数/plugin/remap）
python3 -B framework/test_robot.py          # 4 项（F5 profile/端口）
python3 -B framework/test_verification.py   # 5 项（F6 独立验证配置）
python3 -B framework/test_preflight.py   # 5 项（实例 config/维度预检）
python3 -B framework/test_safety.py      # 8 项（F7 接线/否决即停/启动基准）
python3 -B contracts/test_manifest_v2.py    # 6 项（v2 ports→io）
python3 -B registry/test_v2_manifest.py     # 4 项（registry/composer v2 兼容）
python3 -B framework/plugin_smoke.py        # 真实 ROS：nodelet plugin 装配/参数/IO/停止
python3 -B framework/gazebo_verification_smoke.py  # 真实 Gazebo 验证后端/truth/清理
```

`check` 输出 digest、启动序、反馈边、命令话题写者与安全接线报告（唯一写者/建议边/validator）；
`plan` 额外写 assembly plan 与 launch 骨架。错误 rc=5（带理由码），用法错误 rc=9。

## 3. 已覆盖的检查

| 检查 | 理由码 |
| --- | --- |
| 未知组件 / 未知端口 / 端口方向 | `E_GRAPH_COMPONENT` / `E_GRAPH_PORT` |
| 消息类型、地图 schema 不一致 | `E_GRAPH_TYPE` / `E_GRAPH_SCHEMA` |
| 必需输入未接、同一输入重复接 | `E_GRAPH_INPUT` / `E_GRAPH_INPUT_DUP` |
| 非反馈边形成启动环 | `E_GRAPH_STARTUP_CYCLE` |
| 反馈边缺策略 | `E_GRAPH_FEEDBACK_POLICY` |
| 同一命令话题多个写者 | `E_GRAPH_COMMAND` |
| 实例 ID/ROS 节点名冲突 | `E_CFG_INSTANCE_ID` / `E_GRAPH_NAME_COLLISION` |
| YAML 重复键、缺字段、类型错 | `E_CFG_DUP_KEY` / `E_CFG_FIELD` / `E_CFG_TYPE` |
| v1 清单缺命名端口 | `E_CFG_MANIFEST_V1` |
| launch 未知包/文件/可执行/参数文件 | `E_LAUNCH_PACKAGE` / `E_LAUNCH_FILE` / `E_LAUNCH_TYPE` |
| launch 规格非法（node/include/plugin/路径） | `E_CFG_LAUNCH` / `E_CFG_LAUNCH_KIND` / `E_CFG_LAUNCH_PATH` |
| plugin 宿主非 `nodelet`、type 非 `pkg/Class`、带 launch_file/param_files/args | `E_CFG_LAUNCH` |
| plugin 插件包不在包索引 | `E_LAUNCH_PACKAGE` |
| plugin 类不在索引 `plugin_classes`（索引给出该字段时） | `E_LAUNCH_TYPE` |
| 实例 config 缺失/非法 YAML/重复键/非映射 | `E_CFG_MISSING` / `E_CFG_YAML` / `E_CFG_DUP_KEY` / `E_CFG_TYPE` |
| config 用在 include 组件 | `E_CFG_CONFIG_KIND` |
| Path 跨 2D/3D 组件直连 | `E_GRAPH_DIMENSION` |
| 组件 dimensions 与其地图 schema 不一致 | `E_GRAPH_MAP_DIMENSION` |
| `/cmd_vel` 写者不唯一或非 `safety_arbiter` | `E_SAFETY_ARBITER`（`Assembly.start` 同拒） |
| 命令建议未接到仲裁器 | `E_SAFETY_ADVICE` |
| validator 多于一个或不在 path 上 | `E_SAFETY_VALIDATOR` |
| validator 否决缺少 `approved` 字段 | `E_SAFETY_VERDICT` |

反馈边不参与启动 DAG：`merge.delta → map.delta` 可带 `policy.period_s`，启动不会互等。

## 4. 示例（3D 双缓存）

`fixtures/graphs/graph_3d_cache.yaml` 起了**两个同类型 `map_cache` 实例**，各自观察同一
点云/里程计，delta 经 `map_merge` 回到 `map3d.delta`（反馈边，5 s 策略）。这验证了：

- 同组件多实例可配置；
- 命名端口不会因消息类型相同而混淆；
- 局部缓存 → 增量合并 → 全局地图的连线可被静态校验；
- 启动序只依赖非反馈边，缓存更新环不会阻塞启动。

## 5. 边界（如实）

- 真实入口：包索引已对上 20 个 v2 组件（11 个真实 + 9 个 `*_test_double`）；R981 的真实 2D
  nav+mux 已由 Assembly 在 Gazebo 启动。R982 增加 `v8.fw.hdl_localization` 与两张定位图，
  但 hdl 真实 launch 未执行（stock nodelet 需 `/leg_odom2`；M2-R2C 终止仿真域；GT 辅助 probe
  不计独立定位）；faster_lio 仍为 include，3D/其他算法仍未 v2 化。
- 实例 `config` 在 plan/launch 渲染层生效，但真实 `rosrun` 命令不加载 node 的
  `rosparam` 文件；当前真实接入用 include 组件（参数走 launch args）。
- plugin 只支持 `nodelet standalone` 单进程宿主：不支持外部 nodelet manager /
  `nodelet load`、不支持 bond；pluginlib 类只按包索引 `plugin_classes` 静态校验，
  不解析 XML/不验证类实现。实测 nodelet standalone 忽略 remap 参数。
- include 组件没有通用 `remap` 入口：图中连入 include 的边必须让端口 topic 与上游一致，
  否则真实启动不会改线；node/plugin 的 manifest remap 已进运行命令，图边 remap 仍只在
  渲染骨架里。
- 包索引是声明快照，不是 rospack 活体扫描；验证已接真实 Gazebo empty-world 后端（R1006），
  未接 MuJoCo 后端与场景化真值；`gazebo_truth.py` 以 ground_plane 静态位姿证明通道，
  不冒充算法评价；
  热插拔、状态迁移、运行期改线未实现。
- validator 否决走的是**编排层停止链**，不是 msg 级实时看门狗；50/500 Hz 的承载路径未做实测对比，
  只记录启动 check+plan 的 p95。
- 不改 `contracts/registry/composer/orchestrator` 既有语义；A1—A7 状态不变。

## 6. 实例配置文件与预检

- graph 的实例可加 `config: path/to/params.yaml`（相对 graph 文件或绝对路径）；该文件顶层必须是映射，
  重复键、语法错误、缺失文件均在启动前拒绝，内容 sha256 进入 assembly digest 与 plan。
- `config` 取代该实例 manifest 的 `launch.param_files`；graph 的 `params` 仍在其后覆盖。
  `include` 组件不接受实例 `config`，用 `args`/`params`。
- 预检同时检查：`nav_msgs/Path` 边两端组件 `dimensions` 必须一致（`E_GRAPH_DIMENSION`）；
  组件自身的 `dimensions` 必须与 `occupancy_grid/v1`/`support_map/v1|v2` 输入输出匹配
  （`E_GRAPH_MAP_DIMENSION`）。这能提前拦住“2D 局部接 3D 全局/地图”的直连。
- 参数 schema（manifest 的 `param_schema`）目前只登记、未做字段级校验；字段级 schema 校验是后续切片。

## 7. 收口状态

F0—F8 已完成：证据索引、未接入项与回滚说明见
[27 号补丁 §11/§12](../../../../../note/计划书v8/27_框架改造补丁-v0.1.md)；
每批读数在 `note/进度/v8/M5/26-10-29/evidence/R955—R965`。
不再新增并行组件模型；v1 manifest 与既有运行链语义不变。

## 8. V8-X3 v0.1 图形化选配与热替换

```bash
cd ws/src/90_platform/v8_platform
python3 -B framework/selector.py framework/fixtures/manifests \
  framework/fixtures/graphs/graph_2d.yaml --out-html /tmp/selector.html
```

页面把每个实例的同角色候选组件列成下拉框；单实例替换的 `graph.check`/`safety.check_plan`
结果（含拒绝码）预生成在页面里，组合结果导出为 JSON（YAML 兼容）后仍由 `cli.py check` 复核。
仓库内示例：[framework-selector.html](../../../../../docs/architecture/framework-selector.html)。
它不新增第二套组件模型，也不在浏览器里复制校验器。

热替换 `hotswap.py` 使用 `hot_swap/v1` 合同：`prepare` 要求新旧组件角色与命名端口签名一致、
`/cmd_vel` 所有权不变，输出带 epoch 与不透明 `state` 的记录。`apply` 停整装配后换组合；
`apply_instance` 只停目标实例，核验旧进程退净后启动替换，其他实例 PID 与任务 `RUNNING`
保持不变，新实例启动失败则重启旧组件并记 `rolled_back`。记录中的 `state` 写成临时 JSON，
经 `V8_HOT_SWAP_STATE`/`V8_HOT_SWAP_INSTANCE`/`V8_HOT_SWAP_EPOCH` 交给新进程；组件须显式
读取。仍是**先停后启**，不是零中断；运行期 TF 实际探测未做。
