# v8 编排（V8-06，R646：任务 API＋就绪门＋停止链＋资源预算）

> 2026-10-25 起。设计层交付：任务状态机（启停/取消/状态＋事件留痕）、**条件式就绪门**
> （可观测条件轮询、超时拒绝——A2「无固定等待冒充就绪」的设计层落点）、停止链
> （逆角色序＋**仲裁器最后停**＋挂起升级 force＋终态核验退净）、资源预算校验
> （V8-05 留白在此闭合）。消费 composer 锁文件（流程6 产物）。
> 真实进程/ROS/容器运行面与活体故障注入按 25 号活体窗纪律归仿真阶段（V8-07+/V8-13），
> 本层以 SimulatedRunner 注入等价故障验证停止链逻辑。

## 1. 交付内容

| 件 | 作用 | 边界 |
| --- | --- |---|
| [orchestrator.py](orchestrator.py) | `TaskState` 状态机（非法迁移 E_ORCH_TRANSITION）＋`Task` 事件留痕；`wait_ready`（条件轮询，永就绪＝E_ORCH_READY_TIMEOUT 进停止链）；`stop_chain`（ROLE_STOP_ORDER 逆角色序、safety_arbiter 永远最后、declared stop_method＋kill 兜底、grace 挂起升级 force、终态核验 E_ORCH_STOP_FAILED/E_ORCH_NO_RUNNER）；`check_budget`（requires.resources 逐件计数对预算，E_ORCH_RESOURCE）；`Orchestrator`（submit 预算 fail-closed→start 就绪门→health_check→cancel/complete）；`ComponentRunner` 抽象＋`SimulatedRunner` 故障注入 | 运行面适配器归 V8-07+ |
| [test_orchestrator.py](test_orchestrator.py) | 13 项：预算三态、happy path＋停止序（dwa 先/mux 后）＋事件留痕、**条件就绪不睡满**（3 轮就绪 1 s 内返）、永就绪超时退净、运行中死亡捕获、取消、停止挂起升级 force、非法迁移守卫、声明式停止方式（dwa=process_group_signal、none→kill）、**双任务隔离**（A 失败 B 完成）、composer 锁集成（真实六件套全流程） | A2 三判据设计层证据 |

## 2. A2 判据映射（设计层证据；正式 A2 验收归仿真矩阵 V8-13）

| A2 要求 | 本层证据 |
| --- | --- |
| 无固定等待冒充就绪 | wait_ready 只认条件成立；死亡件/永就绪件一律超时拒绝进停止链（test_readiness_by_condition / test_ready_timeout / 复核探针） |
| 失效可停车 | 运行中死亡被 health_check 捕获→停止链退净→FAILED（test_die_in_running_detected_and_stopped） |
| 其他任务不受影响 | 双任务隔离：A 的停止链不碰 B 的 runner，B 正常 COMPLETED（test_task_isolation） |

## 3. 语义要点

- **停止序**：消费方先停（controller→planner→representation→…→odometry），safety_arbiter 永远最后——拆除期间 `/cmd_vel` 仍被仲裁器闸住（22 号命令唯一所有权的拆除面）。
- **资源预算**：预算键＝requires.resources 枚举名（cpu/gpu_optional/gpu_required）逐件计数；未列键＝不设限（显式声明语义）；预算不过 submit 即拒、不入队。
- **事件留痕**：每次迁移记 (ts, from, to, cause)，P8 回写面素材。

## 4. 运行方式

```bash
python3 -B test_orchestrator.py   # 13 项
```
