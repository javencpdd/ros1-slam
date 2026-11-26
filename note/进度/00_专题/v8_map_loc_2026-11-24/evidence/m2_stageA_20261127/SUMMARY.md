# M2 stageA 修复前后（2026-11-27）

| | 修复前 | 修复后 |
|---|---|---|
| 判定 | FAIL (9/2) | **PASS (11/0)** |
| 闭环 | fwd_back TIMEOUT | out/turn/back 全 done |
| drift | 27.170 | 0.486 |
| gt_drift | 27.170 | 0.486 |
| 点云/IMU/飞点 | 9.6 / 200 / 0 | 8.0 / 200 / 0 |
| RTF | 0.422 | 0.437 |

根因：回程是定速开环直行，终止判据只有"离起点 ≤0.5 m"这一条；某一帧跨过 0.5 m 圆后条件永不成立 ⇒ 一路开到 27.17 m 才被 150 s 墙钟掐断。
反证 RTF 不是原因：27.17 m ÷ 0.5 m/s = 54 s 仿真时间 < 窗内可用 63 s（150 s × 0.422）。
修法：保留 0.5 m 终点窗，另加"沿出程轴已回到起点以内(x-sx≤0.35)"与"累计行程 >25 m 判 RUNAWAY"两条终止条件。

## Stage B（M2-2）修复后：PASS
bt_outcome=reached dt=15.1 s（sim 时钟）dev_odom=0.097 dev_gt=0.095 pose_class_errors=0

根因：`ws/.profiles/x86_sim/src` 有一份内容完全相同的 lite3_bringup，包名解析撞出
RLException: multiple files named [mode_sim_bt.launch] ⇒ /clock 永不到 ⇒ 整段 FAIL。
修法：优先用 ws/src 下的规范 launch（显式路径），找到才回退包名。

## Stage C 干净重跑（2026-11-27 04:5x）：暴露坐标系偏移
SMOKE initialpose=(0.150,-0.100,-5deg) gt=(0.000,0.000)
SMOKE reloc conv=False dev=1046.597 worst30s=3508.262
20Hz 逐样本 trace 见 stageC_trace_1046m.tsv：d 从 1046 m 增至 3508 m。
⇒ 映射段生成的 sim_venue.pcd 在 Faster-LIO 的 camera_init 系里，与 Gazebo 世界坐标相差 ~1 km，
   HDL 载入后位姿落在 map 系原点上，与 GT 无从比较；此前的 0.66 是在旧栈（已对齐）上测到的。
