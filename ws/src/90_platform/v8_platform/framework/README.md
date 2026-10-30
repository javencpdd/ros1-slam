# framework · YAML 驱动的 ROS1 组件图（v0.1）

> 2026-10-31 起。首版只做**配置与装配计划**：组件目录、实例、命名端口、显式连线、
> 启动依赖、命令唯一写者、地图 schema 兼容、锁摘要、ROS1 launch 骨架。
> **不启动 ROS、不加载具体算法、不改现有判据/运行链。** launch 输出是骨架，真装配归下一片。

## 1. 组件清单 + 图 YAML

- **组件源＝registry manifest v2**（`manifest.v2.json`）：只有 `ports`（命名输入输出、
  消息类型、schema、topic、command_topic）、`dimensions` 与 `launch`（package/type）；
  `io` 不再落盘，由 registry/composer 加载时从 ports 派生（v1 清单继续带 io 且合法）。
  v1 无命名端口，不能进入图（`E_CFG_MANIFEST_V1`）。
- `graph.yaml`：选择实例、参数、namespace、显式 `from: instance.port` → `to: instance.port`、
  外部输入，以及反馈边与策略。

不再维护独立的 `components.yaml`：组件定义只有 manifest 一份；v2 的 io 是**加载期派生视图**，不落盘。

对应关系：**组件类型可复用，实例 ID 唯一；同名组件可起多个实例；连线必须显式到端口。**

## 2. 命令

```bash
cd ws/src/90_platform/v8_platform
python3 -B framework/cli.py check framework/fixtures/manifests \
  framework/fixtures/graphs/graph_3d_cache.yaml
python3 -B framework/cli.py plan framework/fixtures/manifests \
  framework/fixtures/graphs/graph_3d_cache.yaml \
  --out-plan /tmp/plan.json --out-launch /tmp/graph.launch
python3 -B framework/test_framework.py      # 18 项
python3 -B contracts/test_manifest_v2.py    # 6 项（v2 ports→io）
python3 -B registry/test_v2_manifest.py     # 4 项（registry/composer v2 兼容）
```

`check` 输出 digest、启动序、反馈边、命令话题写者；`plan` 额外写 assembly plan 与 launch 骨架。
错误 rc=5（带理由码），用法错误 rc=9。

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

反馈边不参与启动 DAG：`merge.delta → map.delta` 可带 `policy.period_s`，启动不会互等。

## 4. 示例（3D 双缓存）

`fixtures/graphs/graph_3d_cache.yaml` 起了**两个同类型 `map_cache` 实例**，各自观察同一
点云/里程计，delta 经 `map_merge` 回到 `map3d.delta`（反馈边，5 s 策略）。这验证了：

- 同组件多实例可配置；
- 命名端口不会因消息类型相同而混淆；
- 局部缓存 → 增量合并 → 全局地图的连线可被静态校验；
- 启动序只依赖非反馈边，缓存更新环不会阻塞启动。

## 5. 边界（如实）

- 清单里 `v8_platform/*_test_double` 是测试替身；`faster_lio`、`map_server`、mux 等
  是真实 ROS1 包名，但 **launch 骨架尚未与真实参数/启动文件逐项对表**。
- 未接 `Orchestrator/ProcessRunner` 运行；未生成 roslaunch 可用的参数全量映射；
  未实现热插拔、状态迁移、运行期改线。
- 地图 provider 的增量提交/版本事务、机器人 profile、MuJoCo 验证配置都不在本片。
- 不改 `contracts/registry/composer/orchestrator` 既有语义；新图模型独立，下一片再决定
  是否复用 manifest 或迁移。

## 6. 下一片

1. **launch 适配表**：组件 launch 包/类型 → 真实 launch/参数入口；测试替身逐步替换。
2. **运行装配**：把 assembly plan 交给现有 `ProcessRunner`/`Orchestrator`，就绪/停止/清理复用。
3. **地图增量契约**：`MapDelta`、基版本、原子合并、幂等与更新通知。
4. **机器人适配与独立验证配置**：command/telemetry 端口、仿真场景独立选择，不并入核心图。
