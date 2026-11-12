# BT-ORCH / port-truth —— 端口对表真源（live 快照）

- 采集：2026-11-12，私有 master 端口 11663，栈 `graphs/navigation_bt.yaml`（two_floor_nav `upstairs`），
  命令 `rostopic list -v`。
- `topics.txt`：原始输出（53 行，33 个唯一 topic，含类型与发布/订阅计数）。
- `verify/port_truth.json`：由 `topics.txt` 解析的可机读真源（`schema: v8_port_truth/v1`），
  供 `verify/port_conformance.py` 对表。
- `run2-summary.json`：同一跑的 app 结果（`ok=true`，距目标 `0.1855 m`），证明快照取自**活栈**而非声明。

## 用途与发现

- `port_conformance.check(report, catalog, truth)` 用外部观测核对每个声明端口；两分同错声明**不能互证兼容**。
- 实测发现：`/v8/local/cmd_vel_3d` 的活类型是 `geometry_msgs/TwistStamped`；
  mux 的输入/输出都叫 `cmd`，输出的 `command_topic` 必须优先，否则会把输入的 `cmd_topic` 套到输出上（假阳性，已修）。
- 快照是**单次观测**：换栈、换主机或上真机都要重采，不把一次快照当永久契约。
- 本快照只覆盖 `two_floor_nav` 的 BT 栈；TEB/DWA 等 apt 组件未在跑，未纳入（不得用同名声明互证）。
