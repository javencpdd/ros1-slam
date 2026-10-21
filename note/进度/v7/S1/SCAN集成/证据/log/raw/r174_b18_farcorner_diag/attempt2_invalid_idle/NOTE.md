# attempt2 ＝ 停手窗（探针自证不过 ⇒ 未判被试，读数不可引用）

**这一窗是唯一一次真把链跑起来、并触发停手条件的窗**，价值在两个被实测到的形状，不在被试结论：

1. **尝试①（默认 patience＝3.0，客户端上界 30s→本窗写的是 60s）没拿到 result**：
   `outcome=None、state=1（ACTIVE）、墙钟=60.0s、result_returned=False`
   ⇒ 这一 boot 上服务器**连自己的 patience 中止都没回话**＝t 稿 §4 分列的第二型（服务/挂起型），
   与上一 boot 的 3.2s 回话（型＝patience 掐断）**不是同一形态**。
2. **放宽没落地**：`set→查询 {'planner_patience': 3.0} ⇒ 未落地` ⇒ 脚本按设计走 INVALID 退出、**没有把"放宽后仍失败"写成结论**。
   成因现成可指：尝试①那个目标还挂在 ACTIVE（`state=1`），**mbf 的 reconfigure 锁在 goal 进行中新写的配置吞掉**
   ——这条正是在册 `w3_t1_probe2.sh` 第 4 行"dynparam 已实证会卡死在 mbf reconfigure 锁上"那句的**现场复现**，
   区别只在在册那次是"卡死"、这次是"写了不生效"。

**因此本窗的两条可引用产出**：①目标未回话时必须先 `cancel_goal` 落到终态再动 DR（已改进件）；
②客户端等待上界该与在册 harness 同值 30s（已改），否则"没回话"与"回话慢"分不开。
红线面：样张 sha 起前＝退后一致、foxy 4 个 PID 逐位一致、teardown 零残留、11313 零监听（见本目录 `redline_summary.txt`）。
