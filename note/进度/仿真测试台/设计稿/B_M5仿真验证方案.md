# B. M5 仿真验证方案（C12 既定，v1.0 终稿）

## B1. 现状评估（定点实证 2026-10-07）

| 件 | 状态 |
|---|---|
| task_smach_min.py + task_primitives.yaml | P3a 已落盘（S3 切片），**从未运行** |
| 50_task_smach_min.launch | 内置 `<param task_primitives>` 注入（实证 line5）→ roslaunch 即可，无需手动传参 |
| mode_nav_sim | G-G8b PASS（13 CHECK 全过，2026-10-05 run3），**sim_nav_smoke.sh 判据已验收冻结，不许碰** |
| Q11（action 话题名对账） | task_primitives.yaml 头注自证「action 话题名未实测」→ **首跑最大风险点** |

## B2. 修法清单（M5.1 首跑，0.5 人日）

1. **Q11 对账**：起 mode_nav_sim → `rostopic info /move_base_flex/get_path` 等，逐条核 yaml 内 action target 与实测命名空间一致；不一致只改 yaml（壳包 config，diff 登记）。
2. **判据件**：新增独立 `verify/v_task_smach.sh`（壳包规范位，不碰 13 CHECK smoke）；CHECK-14 方案作废（矩阵 v2 已裁决）。
3. **首跑**：`roslaunch lite3_task 50_task_smach_min.launch`，观察 SMACH 状态机 outcome。
4. **失败预案**：outcome=aborted → 分层定位（action 连接失败=Q11 残留 / GetPath 失败=起点终点不可达 / ExePath 失败=teb 参数）；preempted=超时参数。

## B3. 验证判据（怎么知道做对了）

- 判据一：SMACH outcome=**succeeded**，3 连跑全过（沿用三连红线）。
- 判据二：单任务链（GetPath→ExePath→到达）时长 **dt≤60s**（maze2d 12×12 单目标，对齐 teb 闭环 dt=8.6s 基线量级，留 SMACH 开销余量）。
- 判据三：v_task_smach.sh 输出 True/False 可判定，证据 log 入 `note/进度/…/证据/`。

## B4. M5.2 SCAN 原语扩展（依赖 A 项完成后启动，1 人日）

task_primitives.yaml 增 `scan_navigate` 原语（pub /scan_planner/goal + 监听 FSM reached）→ 同判据口径复跑 3 连 → M5 收口。M5.1 与 M5.2 解耦：M5.1 不等 SCAN。
