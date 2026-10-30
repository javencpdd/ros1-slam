# 统一 YAML 组件图 v0.1（ROS1）

> 2026-10-30 起；R956 把组件源并入 registry manifest v2，R963/R964 接上独立验证与安全接线。
> 目标框架的第一片：**配置、装配计划与运行装配**；数据面仍走 ROS 话题。
> 代码：`ws/src/90_platform/v8_platform/framework/`；framework 22 项＋runtime 5 项＋robot 4 项＋
> verification 5 项＋safety 7 项＋selector 3 项＋hotswap 5 项＋契约 v2 6 项＋registry/composer v2 4 项；
> V8 全树 57 套件 0 失败。

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
| 命令 | 每个命令话题只允许一个写者；`/cmd_vel` 写者必须是 `safety_arbiter` |
| 锁 | catalog sha256 + 归一化图 → assembly digest |
| 输出 | `plan.json` 与 ROS1 launch（node/include/param/remap/arg） |
| 运行装配 | plan → `ComponentSpec` → `ProcessRunner/Orchestrator`；就绪/停止/清理，mux 最后 |
| 地图增量 | `map_delta/v1`：delta_id 幂等、旧 base 拒绝、原子升版、监听通知 |
| 机器人 profile | robot.v1：declared/prohibited 不驱动规划；Lite3 与 test_b 仅改 YAML 切换 |
| 独立验证 | backend 不进算法图；真值不泄漏算法话题；失败只记 status=error |
| 安全接线 | 双 `/cmd_vel` 写者/越权建议拒绝；validator 否决走停止链 |
| 性能预算 | 启动 check+plan p50/p95/max；500 Hz 发送不经 Python 编排环 |
| 图形化选配 | 同角色候选预检、拒绝码、导出 JSON/YAML 兼容图 |
| 热替换合同 | `hot_swap/v1`：角色/端口/所有权门＋停止后换组合 |

3D 双缓存 fixture 验证了：两个同类型实例、命名端口不混淆、`merge.delta → map.delta`
反馈边不阻塞启动；v1 清单以 `E_CFG_MANIFEST_V1` 明确拒绝进图。

运行时只做启动/停止：50 Hz 规划与控制走 ROS 话题；500 Hz 命令发送在边界适配器专用进程
（33B UDP），不进 Python 编排环。`perf.py` 只记录启动前 check+plan 的开销，`startup_only=True`。

## 3. 边界（如实）

- catalog 中的 `*_test_double` 是测试替身；真实包的 launch 条目尚未逐项对表，也未在宿主执行
  `rosrun/roslaunch`。`kind: plugin` 托管未实现。
- 运行装配测试用假命令/假进程；未启动真实 ROS 组件。validator 否决是编排层停止，不是 msg 级
  实时看门狗；运行中热插拔/状态迁移未实现（V8-X3 仅停止后换组合）。
- 独立验证用假后端，未启动 MuJoCo/Gazebo。
- 不改既有 contracts/registry/composer/orchestrator 语义；A1—A7 仍 `NOT_VALIDATED`。

## 4. 收口状态

F0—F8 已完成；F1—F7 证据索引与“未接入项/回滚说明”见
[27 号补丁 §11/§12](../../note/计划书v8/27_框架改造补丁-v0.1.md)。每批读数位于
`note/进度/v8/M5/26-10-29/evidence/R955—R965`。A1—A7 状态不变，仍 `NOT_VALIDATED`。

## 5. V8-X3 v0.1

图形化选配页示例：[framework-selector.html](framework-selector.html)（生成命令见
[framework/README.md](../../ws/src/90_platform/v8_platform/framework/README.md) §7）。
热替换：`hotswap.prepare` 检查角色/端口/命令所有权；`apply` 停整装配后换组合，
`apply_instance` 只停目标实例、保留其他实例与任务 `RUNNING`，启动失败回滚旧组件。
仍是先停后启；组件内部状态未序列化，运行期 TF 实际探测未做。

完整说明见 [framework/README.md](../../ws/src/90_platform/v8_platform/framework/README.md)。
