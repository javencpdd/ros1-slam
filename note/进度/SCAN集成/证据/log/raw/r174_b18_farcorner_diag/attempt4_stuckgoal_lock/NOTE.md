# attempt4 ＝ 停手窗（探针这次没坏：它抓到的是被试侧的真形状）

链正常起、服务器 5.5s 就绪、`planner_patience` 现值读到 3.0（与生效件同值）。

- **尝试①（默认 patience 3.0，客户端上界 30s）**：`outcome=None、state=1(ACTIVE)、墙钟=30.0s、result_returned=False`
- **取消也不生效**：`cancel_goal()` 后**轮询 20.0s 仍未到终态**（`terminal_reached=False`、状态还是 ACTIVE）
  ⇒ 这一型不是"规划慢"，是**服务器对该对不回话、也不受取消控制**。
- **因此 DR 放宽必然落不了地**：`set→查询 {'planner_patience': 3.0} ⇒ 未落地` ⇒ 件按设计 **rc=9 INVALID 退出、没有把"放宽后仍失败"写成结论**。

**对照实证（这才是本窗的可引用产出）**：attempt1 里同样一段 `update_configuration` **当场落地**，
差别只有一个——attempt1 那次前一个 goal **已经回话结束**（服务器空闲），attempt2/4 那次前一个 goal **还挂在 ACTIVE**。
⇒ 结论＝**"放宽 patience"这个动作必须发生在任何 goal 之前**；放在 goal 之后，测到的永远是锁、不是被试。
本窗之后件已按此重排（地图门 → 空闲写 DR → 回查 → 唯一一次 goal）。

红线面：样张 sha 起前＝退后一致、foxy 4 PID 逐位一致、teardown 零残留、11313 零监听（见 `redline_summary.txt`）。
