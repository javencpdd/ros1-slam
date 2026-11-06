# composer · 结构兼容检查与确定性锁

> 最新更新时间：2026-11-06 04:34:56 CST。本次为文档与源码核对；验收状态以[项目进度](../../../../../note/进度/README.md)为准。

[composer.py](composer.py) 消费[注册表](../registry/README.md)，提供 `compose(store, ids)`、`compatibility(store, entries)` 与 `lock(store, ids)`。它检查声明之间能否组合，不启动 ROS，也不凭组合通过证明运行能力。

## 检查范围

- 选中 id 必须存在，重复选择去重；同一职责不能被重复分配。
- 角色依赖按 `ROLE_DEPS` 的“任一满足”集合检查：global_localization/mapping/representation 需 odometry；global_planner 需 map_server/representation/mapping 任一；controller 和 trajectory_validator 需 local_planner；safety_arbiter 需 controller。
- TF child 不能双 parent，TF 和角色依赖不能成环；每个 command_topic 只能有一个写者。
- 地图消费者须找到同类型地图生产者；运动角色 controller/local_planner 不得使用 not_applicable，规划器与控制器运动学按允许配对表检查。global_planner 产路径，可声明不适用运动学。
- 报告保留 declared_only/tested 分档、角色、命令所有权、TF child 与兼容条目。tested 的精确证据仍须回读 manifest。

`lock()` 在组合通过后生成规范序列化 sha256 摘要和逐组件 version/evidence/declared_only/sha256。选择顺序与重复选择不改变锁身份。它锁定输入声明，不是启动后的实际配置快照或许可结论。

## 与其他层的关系

消息级输入生产者匹配已在 [adapters/io_match.py](../adapters/io_match.py) 实现；信息损失声明在 [adapters](../adapters/README.md)。composer 返回的 `adapters` 槽仍是空列表，不能据此声称自动选择或执行了适配器。

运行预算、条件就绪、健康和停止链由 [orchestrator](../orchestrator/README.md) 实现。v2/v3 manifest 的实例/命名端口、显式连线和 Assembly 由 [framework](../framework/README.md) 管理；本组合器的角色唯一约束不替代 framework 的多实例图规则。

运行的 execution_mode/robot_id 与组件域统一由 framework/execution.py 复核，本组合器的静态兼容不替代该检查或现场许可。

常见拒绝码：`E_COMP_EMPTY/UNKNOWN_ID/DUP_ROLE/MISSING_ROLE/TF_CYCLE/TF_DUP/CMD_DUP/MAP_SUPPLY/KINEMATICS`。

## CLI 与测试

从仓根执行；下面故意缺依赖角色，应返回 rc 5 和拒绝报告：

```bash
python3 -B ws/src/90_platform/v8_platform/composer/composer.py \
  ws/src/90_platform/v8_platform/registry/data/manifests \
  --select v8.odometry.faster_lio,v8.controller.dwa_adapter
python3 -B ws/src/90_platform/v8_platform/composer/test_composer.py
```

CLI 可加 `--provenance FILE` 和 `--lock OUT`；锁只在通过后写出。rc 0 通过、rc 5 组合拒绝、rc 9 用法/前置错误。定向测试覆盖真实登记组合、冲突负例、兼容与锁稳定性，当前数量以测试输出为准。
