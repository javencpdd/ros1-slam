# BT-ORCH / upstairs-formal-r3 —— 三项目解耦后 formal 回归

- 日期：2026-11-12
- 任务卡：`T-20261112-07`
- 窗口：`BT-ORCH-upstairs-r3`（回执见 `scripts/verify/_window_receipt.tsv`）
- source：`3b4fc0a7`（三项目解耦提交）
- 性质：**FORMAL**（原版 preflight `rc=0`；trace 校验 `ok`）
- 目的：验证「P2 自持 carrier/truth、P1 app 声明仿真后端、P3 只读」没有改变既有功能。

## 读数

- `ok=true`；六项 checks 全 `true`；`validation_error=null`。
- `carrier_final=(-5.76142, 1.11986, 1.6)`；全局路径 57 poses；本地 `reason=arrived`。
- BT 事件（`bt_events.log`）：goal queued → dequeued → global path requested → path ready(57)
  → follow path → dispatch (-5.9,1.1,1.6) → arrival budget。

## 对比

与解耦前的 `upstairs-formal`（r2）同结果：`ok=true`、载体到上层目标、57 点路径。
**解耦未改变行为**。

## 不声称

四足步态/物理接触、真机、实时建图、完整全局走廊；能力域仍是 declared 几何地图 + Gazebo 运动学载体。

## 文件

- `summary.json` / `summary.app.json` / `summary.preflight.log` / `summary.trace.json.gz` / `bt_events.log` / `hashes.sha256`。
- `.runtime/` 为运行日志，不入库。
