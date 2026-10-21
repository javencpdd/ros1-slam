# attempt1 ＝ 作废窗（施工侧探针缺陷，被试读数不可引用）

链起来了、服务器 5.0s 就绪、DR 现值读到 `planner_patience=3.0`（与 t 稿 §1 生效件同值，**载体面这次读对了**）。
两次尝试的**墙钟是真实的**（①3.2s ②60.2s），但 **outcome 全部丢失**：

    'GetPathResult' object has no attribute 'state'

缺陷形状值得留：**不是取不到字段，而是取字段的赋值方式把已经取到的值销毁了**。
原句 `oc, st, msg = r.outcome, r.state, ...` 先整体算右值元组 ⇒ `r.state` 抛错 ⇒ 三个名字一个都没绑上，
except 里再把 `oc` 写成 -1，于是**真实 outcome 被覆盖**。`state` 本就属于 actionlib 客户端（`client.get_state()`），不在结果消息里。
修法：逐字段独立 try（见现件），并加"探针自证不过 ⇒ INVALID、不判被试"的前置门。
