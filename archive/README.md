# archive/

非活跃源码归档。此处内容不参与 catkin 构建、不进入任何 target 的包闭包。

## lite3_task（2026-11-21 归档）

- 原位置：`ws/src/50_task/lite3_task`（SMACH 任务编排，289 行，5 个状态类，零 launch 引用）。
- 处置：真人 2026-11-21 勾选「归档」；代码与 git 历史保留，不接线、不构建。
- 恢复：`git mv archive/lite3_task ws/src/50_task/lite3_task`，并在 `tooling/build/profiles.json` 的
  `local_packages` 与对应 target `roots` 中恢复 `lite3_task`；随后按构建纪律重建。
