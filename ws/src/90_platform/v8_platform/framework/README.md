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

`launch` 支持 `kind: node`（package/type/param_files/remap）与 `kind: include`
（package/launch_file/args）；`plugin` 托管暂未实现（F2 明确拒绝）。`--package-index` 用包索引
校验 package/launch file/executable/param file，不启动 ROS。

`runtime.py` 把 assembly plan 转成 `ComponentSpec` 并交给现有 `ProcessRunner/Orchestrator`；
默认命令为 `rosrun`/`roslaunch`，测试注入假命令。启动、READY 就绪、停止序（mux 最后）、
失败清理与健康检测均有测试；真实 ROS 启动不在本片。

`robot.py` 读取 robot.v1 profile：declared/prohibited 值不得驱动规划（`E_ROBOT_UNVERIFIED`），
仅 tested/allowed 值可用；robot_adapter 必须提供 `cmd`（TwistStamped）与 `odom`/`imu` 遥测端口。
`v8.fw.robot_lite3` 与 `v8.fw.robot_test_b` 同端口，仅改图 YAML 的实例组件即可切换。

`verification.py` 选择独立验证后端（模拟器组件）与场景，**不进入算法图**；真值通道不得与算法话题
重叠（`E_VERIFY_TRUTH_LEAK`）；后端通过注入 runner 运行，失败只记 status=error，不改算法 digest。
本片用两个假后端验证切换，不启动 MuJoCo/Gazebo。

`map_service.propose_delta` 接收 `map_delta/v1`：delta_id 幂等重放、旧 base/同 id 异内容/无变化
拒绝；结果先过 map 契约再原子升版并通知监听者，失败不改变当前版本。

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
python3 -B framework/cli.py plan framework/fixtures/manifests \
  framework/fixtures/graphs/graph_3d_cache.yaml \
  --out-plan /tmp/plan.json --out-launch /tmp/graph.launch
python3 -B framework/test_framework.py      # 22 项
python3 -B framework/test_runtime.py        # 5 项（F3 假命令运行器）
python3 -B framework/test_robot.py          # 4 项（F5 profile/端口）
python3 -B framework/test_verification.py   # 5 项（F6 独立验证配置）
python3 -B framework/test_safety.py         # 7 项（F7 接线/否决即停/启动基准）
python3 -B contracts/test_manifest_v2.py    # 6 项（v2 ports→io）
python3 -B registry/test_v2_manifest.py     # 4 项（registry/composer v2 兼容）
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

- "真实入口"仅指包/launch/可执行/参数文件在**声明索引**中对上（5 个），未在宿主执行；
  `v8_platform/*_test_double` 有 9 个（含第二个机器人适配器）。`kind: plugin` 托管未实现。
- runtime 测试用假命令；真实 `rosrun/roslaunch` 未执行。包索引是声明快照，不是 rospack 活体扫描；
  验证测试用假后端，未启动 MuJoCo/Gazebo；ready_marker 需按组件配置，plugin 托管、热插拔、
  状态迁移、运行期改线未实现。
- validator 否决走的是**编排层停止链**，不是 msg 级实时看门狗；50/500 Hz 的承载路径未做实测对比，
  只记录启动 check+plan 的 p95。
- 不改 `contracts/registry/composer/orchestrator` 既有语义；A1—A7 状态不变。

## 6. 下一片

F8 收口：证据索引、旧入口/旧文档指针清理、未接入项与回滚说明；不再新增并行组件模型。
