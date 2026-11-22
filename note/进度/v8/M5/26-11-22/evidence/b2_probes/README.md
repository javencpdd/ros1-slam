# B2 探针留底（补证据）

## 为什么有这一轮

B2 当时（2026-11-22 下午）为了不污染仓库，把四个探针写在 `.git/` 下。两个后果：

1. **`.git/` 不是发布位置** —— 脚本既没入库、也没进证据目录，换机或清目录即丢；
2. **探针只 `print` 不落盘** —— 读数随终端消失，只剩验证记录里的汇总表。

于是卡面第 5／6／8／9 条验收的「证据」实际是一张我自己写的表，按仓库自己的规矩
（`bridge.py done` 不收自报）等于没有证据。本轮把脚本收进仓，并把 stdout 落到文件。

## 复跑入口

```
bash b2_probes.sh <输出目录>                    # 全部四轮，在 ros1-slam-sim 容器内执行
ROUNDS=regoal bash b2_probes.sh <输出目录>      # 只跑其中一轮
```

链路：`mode_sim_bt.launch global_backend:=far arrival_topic:=/v8/bt/arrived`（BT 在场，
目标走 `/v8/bt/goal`），与 B2 当时一致。

**每轮各起一套独立栈**（`round_accept`／`round_drift`／`round_stale`／`round_regoal`）。
第一版把三个探针串在同一套栈上跑，结果被状态污染：`far_accept` 的取消把 `state.halted`
置真，之后 `far_stale` 的航点超时门按规定在 halted 时不报（`far_node.py` 的 `stale()`
首句就是 `if ... or self.halted`），于是 `STALE_REPORTED=False`——看起来像「超时门不生效」，
其实是上一轮留下的 halted。改为独立栈后同一件读数变成 `STALE_REPORTED=True`。
`far_stale.py` 文件头自己就写了「与取消门分开跑」，第一版违反了它。

## 实际跑过哪几轮

| 目录 | 内容 | 结论 |
|---|---|---|
| `run1/` | 三个探针串在一套栈上 | **作废**（状态污染，保留作缺陷证据） |
| `run2/` | 独立栈四轮中的前三轮 | 有效 |
| `run3/` | `round_regoal` 第一版判据 | **判据写错**（拿 FAR 航点去对目标点），保留作证据 |
| `run4/` | `round_regoal` 修正判据后 | 有效，PASS |

## 逐件职责

| 件 | 服务于卡面第几条 | 说明 |
|---|---|---|
| `_ready_gate.py` | 前置 | 等地形云出现 `intensity ≥ 0.2` 的格才放行。没有这道门，「没有路径」会被读成判据失效，实际是上游没起步（B5 前两轮的真因） |
| `far_accept.py` | 5、6、9 | 四项判据 ＋ 取消三项 ＋ 取消后重发。原样保留 B2 的版本，不改进，为了读数可比 |
| `_cancel_drift.py` | 8 | 取消后不许重新起步 ＋ `/v8/local/goal` 不再被接纳 ＋ 位置漂移。**这一条此前从未被任何探针量过** |
| `far_stale.py` | 9 | 航点超时 ＋ 杀 `far_planner`。★会把上游杀掉，只能单独一轮★ |
| `_regoal_via_bt.py` | 9 | 取消后重发目标，走 **`/v8/bt/cancel`** 这个 BT 真实入口（原因见下） |
| `far_probe.py` | 5 | 单发一次目标看路径与 `/v8/local/goal`，是最早的粗探针 |
| `far_harness.py` | — | 合成台架，证明上游件本身能建图（定因第一步） |
| `far_diag.py` `far_diag2.py` `far_diag3.py` | — | 在 overlay 里打点拿 `DIAG1/2/3`，锁到 `ObsNeighborCloudWithTerrain` |
| `far_recplay.py` | — | 录一帧真实点云复放，输入 `real_cloud.npz` **不在本目录** ⇒ 只作方法留底，不可原样复跑 |

## 一条必须记住的接线事实

目标链是 `/v8/bt/goal` ＝[bt_console RequestPath]＝> `/v8/global/goal` ＝[全局后端]＝> `/global_path`。
**BT 才是目标所有者**。`far_accept.py` 的取消直接发 `/v8/local/cancel`（R6② 授权的第二写者），
BT 对此不知情、树还停在 mission 分支 ⇒ 此时再往 `/v8/bt/goal` 发目标没人接，
`ACCEPT_REGOAL_NEW_STAMPS` 恒 0（run1／run2 两次独立重跑都是 0）。B2 当时记的 `1`
极可能是取消窗口边界上的残留帧。要走真实入口：发 `/v8/bt/cancel`（`<seq>:<scope>:<reason>`），
等 BT 转成 `/v8/local/cancel` 并拿到同 seq 的 `cancelled` ack，再发新目标。


## 三处量法上的讲究（改探针前必读）

1. **位置漂移从「刹停之后」起算**，不是从「取消那一刻」。取消瞬间车还在跑，
   `P1 − P0` 里混着刹车距离，那是正常行为。判据窗口是 `P_brake`（取消后 4 s）→ `P1`（再 8 s）；
   `P0 → P_brake` 只作旁证打印，叫 `DRIFT_BRAKE_M`。
2. **位姿通道要活着**。车不动时若位姿也断了，`P1` 等于旧值、漂移恒 0，是个假绿。
   所以数后窗位姿帧数，少于 5 帧直接判 `FAIL`。
3. **等待一律墙钟**（`time.monotonic`）。仿真 `/use_sim_time` 为真，实测墙钟只推进约 0.44 倍，
   按 ROS 时间算出的窗口会短一倍多。
