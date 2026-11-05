# orchestrator · 生命周期、条件就绪与停止链

> 最新更新时间：2026-11-05 11:49:54 CST。本次为文档与源码核对；验收状态以[项目进度](../../../../../note/进度/README.md)为准。

[orchestrator.py](orchestrator.py) 提供 `TaskState/Task`、`ComponentSpec/ComponentRunner` 和 `Orchestrator`。调用方提交组件、提供 runner，再执行 start、health_check、cancel 或 complete。既有可注入的 SimulatedRunner，也有[真实 ProcessRunner](process_runner.py)，本目录不再仅限模拟运行。

## 运行语义

- submit 按 requires.resources 逐件计数校验预算；未列的预算键不设限，超预算拒绝入队。
- start 依次启动并轮询 runner.ready()；只有可观测条件成立才进入 RUNNING。超时或启动失败走停止链，不能以固定等待冒充就绪。
- health_check 检测进程死亡和可选 run_timeout。调用方须持续调用；仅设置时限不会启动后台监管线程。
- 停止按消费方到生产方顺序，safety_arbiter 最后；先使用声明的停止方法，grace 后升级 force，再核验可观测退净。非法状态迁移和清理失败分别报 `E_ORCH_TRANSITION`、`E_ORCH_STOP_FAILED`。
- 每次状态迁移保留时间、前态、后态与原因；任务 runner 集合相互隔离。

ProcessRunner.ready() 非阻塞；stdout 后台持续排空，避免半行或大日志堵住就绪。重复 start 不覆盖已有 PID；leader 退出后仍清理原进程组后代，必要时 TERM→KILL。roslaunch 可能产生独立会话，运行器会快照后代并清理，不能只凭父 PID 消失判断完成。

本层只决定生命周期。地图/命名端口接线与唯一写者检查在 [framework](../framework/README.md)，高频命令与急停看门狗在各 ROS 节点和 mux。

## 测试与运行证据入口

从仓根运行离线/OS 进程测试：

```bash
python3 -B ws/src/90_platform/v8_platform/orchestrator/test_orchestrator.py
python3 -B ws/src/90_platform/v8_platform/orchestrator/test_process_runner.py
python3 -B ws/src/90_platform/v8_platform/regression/a2_process_lifecycle.py \
  --out /tmp/a2-process.json
```

第三条会创建真实子进程、文件锁和 TCP 监听端口，注入父进程崩溃、就绪/运行超时、后续 exec 失败、取消与任务隔离；Linux subreaper 回收孤儿。评价端重新取得锁和端口，避免把 PID 退出当作资源回收。

`regression/a2_gazebo_stop.py` 另消费已就绪的隔离 master、Gazebo 和 mux，测候选源 SIGKILL、hold 与 `/gazebo/model_states` 物理停车；它需要按[作业单](../../../../../note/计划书v8/25_v8自动调度作业单.md)开窗。该工具不覆盖感知触发，不冻结停车预算，也不把命令回显当物理速度。

A2 与 formal 状态查[进度](../../../../../note/进度/v8/README.md)；本文不把设计层测试升级为正式验收。
