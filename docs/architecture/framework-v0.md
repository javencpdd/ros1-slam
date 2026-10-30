# 统一 YAML 组件图 v0.1（ROS1）

> 2026-10-30 起；R956 把组件源并入 registry manifest v2，R963/R964 接上独立验证与安全接线，
> R981 完成真实 2D 栈的 graph→plan→Assembly 启动四格实测。
> 目标框架的第一片：**配置、装配计划与运行装配**；数据面仍走 ROS 话题。
> 代码：`ws/src/90_platform/v8_platform/framework/`；framework 24 项＋runtime 6 项＋robot 4 项＋
> verification 5 项＋safety 7 项＋selector 3 项＋hotswap 5 项＋契约 v2 6 项＋registry/composer v2 4 项；
> V8 全树 59 套件 0 失败。

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
| 地图增量 | `map_delta/v1`：delta_id 幂等、旧 base 拒绝、原子升版、监听通知；ROS provider 消费（R976） |
| 地图持久化 | `MapService.save/load` 原子快照、恢复重算 digest/核对 history、跨 ROS 重启保留幂等表（R983） |
| 机器人 profile | robot.v1：declared/prohibited 不驱动规划；Lite3 与 test_b 仅改 YAML 切换 |
| 独立验证 | backend 不进算法图；真值不泄漏算法话题；失败只记 status=error |
| 安全接线 | 双 `/cmd_vel` 写者/越权建议拒绝；validator 否决走停止链 |
| 性能预算 | 启动 check+plan p50/p95/max；500 Hz 发送不经 Python 编排环 |
| 图形化选配 | 同角色候选预检、拒绝码、导出 JSON/YAML 兼容图 |
| 实例配置预检 | 每实例 `config` 文件解析/重复键/映射/摘要；取代 manifest param_files |
| 维度配对 | Path 跨 2D/3D 直连拒绝；地图 schema 与组件 dimensions 不一致拒绝 |
| 热替换合同 | `hot_swap/v1`：角色/端口/所有权门＋停止后换组合 |
| 真实 2D 装配 | `v8.nav2d.dwa_stack`/`teb_stack` + `v8.nav2d.mux` 四张 graph；Assembly 启动 nav+mux，GlobalPlanner/Navfn × DWA/TEB 4/4 PASS（R981） |
| 真实停止 | `ProcessRunner` 快照 roslaunch 后代并对独立会话发信号，避免只杀 roslaunch 遗留节点 |

3D 双缓存 fixture 验证了：两个同类型实例、命名端口不混淆、`merge.delta → map.delta`
反馈边不阻塞启动；v1 清单以 `E_CFG_MANIFEST_V1` 明确拒绝进图。

R981 的四张 2D graph 只把 `nav` 实例指向 DWA/TEB 栈并覆盖 `global_planner`/`local_planner`；
`mux` 实例仍拉起真实 `60_safety.launch`。运行后从参数服务器读到实际
`base_global_planner`/`base_local_planner`；`/cmd_vel_raw` 发布者为 move_base、订阅者为
safe_cmd_vel_mux，`/cmd_vel` 唯一发布者为 mux；四格 driver 全 PASS，停止后无残留进程。

运行时只做启动/停止：50 Hz 规划与控制走 ROS 话题；500 Hz 命令发送在边界适配器专用进程
（33B UDP），不进 Python 编排环。`perf.py` 只记录启动前 check+plan 的开销，`startup_only=True`。

## 3. 边界（如实）

- catalog 中 9 个真实组件已对上包索引，9 个 `*_test_double` 是测试替身；R981 的真实 2D
  nav+mux 已由 Assembly 在 Gazebo 启动。R982 增加 hdl v2 清单和两张定位图（faster_lio / hdl），
  但 hdl 真实 launch 未执行：`/leg_odom2`、M2-R2C 与 GT 辅助 probe；3D/其他算法仍未 v2 化。
- node 实例的 `config` 只在 plan/launch 渲染层生效，真实 `rosrun` 不加载其 rosparam；include
  没有通用 remap 入口，连入 include 的边必须 topic 一致。`kind: plugin` 托管未实现。
- 真实验证测试仍用假后端，未启动 MuJoCo/Gazebo；validator 否决是编排层停止，不是 msg 级
  实时看门狗；运行中热插拔/状态迁移未实现（V8-X3 仅停止后换组合）。
- 独立验证用假后端，未启动 MuJoCo/Gazebo。
- 不改既有 contracts/registry/composer/orchestrator 语义；A1—A7 仍 `NOT_VALIDATED`。

## 4. 收口状态

F0—F8 已完成；F1—F7 证据索引与“未接入项/回滚说明”见
[27 号补丁 §11/§12](../../note/计划书v8/27_框架改造补丁-v0.1.md)。每批读数位于
`note/进度/v8/M5/26-10-29/evidence/R955—R981`。A1—A7 状态不变，仍 `NOT_VALIDATED`。

## 5. V8-X3 v0.1

图形化选配页示例：[framework-selector.html](framework-selector.html)（生成命令见
[framework/README.md](../../ws/src/90_platform/v8_platform/framework/README.md) §7）。
热替换：`hotswap.prepare` 检查角色/端口/命令所有权；`apply` 停整装配后换组合，
`apply_instance` 只停目标实例、保留其他实例与任务 `RUNNING`，启动失败回滚旧组件；
`state` 经 `V8_HOT_SWAP_STATE` 临时 JSON 交给新进程。仍是先停后启，运行期 TF 实际探测未做。

完整说明见 [framework/README.md](../../ws/src/90_platform/v8_platform/framework/README.md)。
