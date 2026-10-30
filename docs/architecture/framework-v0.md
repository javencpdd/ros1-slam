# 统一 YAML 组件图 v0.1（ROS1）

> 2026-10-30 起；R956 已把组件源并入 registry manifest v2。目标框架的第一片：**配置与装配计划**，不是完整运行时。
> 代码：`ws/src/90_platform/v8_platform/framework/`；framework 22 项＋runtime 5 项＋契约 v2 6 项＋registry/composer v2 4 项；V8 全树 50 套件 0 失败。

## 1. 解决什么

把“选组件、起几个实例、端口怎么接、参数在哪、启动依赖是什么、谁写命令”从散落的 launch
收拢到两份 YAML：

- **组件源＝registry manifest v2**：`ports`（命名端口/类型/schema/topic/command_topic）＋`dimensions`＋`launch`；`io` 由 registry/composer 从 ports 派生，不落盘；v1 清单仍合法但不能进图。
- `graph.yaml`：实例、参数、namespace、显式 `instance.port → instance.port`、外部输入、反馈边策略。
- 不再维护独立的 `components.yaml`。

核心变化是**类型与实例分离**：同一个 `map_cache` 可以起 `cache_a`、`cache_b`；连接必须落到
命名端口，不再按消息类型猜生产者。

## 2. 已实现

| 能力 | 结果 |
| --- | --- |
| YAML 解析 | 重复键、缺字段、类型错 fail-closed |
| 实例/端口 | 未知组件/端口、方向错、同名输入重复绑定拒绝 |
| 连线 | 消息类型与地图 schema 必须一致；转换需显式组件 |
| 启动 | 非反馈边构成 DAG；反馈边必须带策略，不参与启动排序 |
| 命令 | 每个命令话题只允许一个写者 |
| 锁 | catalog sha256 + 归一化图 → assembly digest |
| 输出 | `plan.json` 与 ROS1 launch 骨架（node/param/remap） |

3D 双缓存 fixture 验证了：两个同类型实例、命名端口不混淆、`merge.delta → map.delta`
反馈边不阻塞启动；manifest v1 兼容性由 `E_CFG_MANIFEST_V1` 明确拒绝进入图；v2 不再落盘 io，派生视图通过 registry/composer 兼容测试。

## 3. 边界（如实）

- catalog 中的 `*_test_double` 是测试替身；真实包的 launch 条目尚未逐项对表。
- **没有启动 ROS**：launch 是骨架，参数/remap 还没有交给 `ProcessRunner/Orchestrator`。
- 没有地图增量事务、机器人 profile、仿真验证配置、热插拔或运行期改线。
- 不改既有 contracts/registry/composer/orchestrator 语义；新图模型独立演进。

## 4. 下一片

1. 组件 launch 适配表与真实参数映射；
2. assembly plan → `ProcessRunner`/`Orchestrator` 的启动、就绪、停止；
3. `MapDelta` 契约：基版本、原子合并、幂等、更新通知；
4. 机器人 command/telemetry 端口与独立仿真验证配置。

完整说明见 [framework/README.md](../../ws/src/90_platform/v8_platform/framework/README.md)。
