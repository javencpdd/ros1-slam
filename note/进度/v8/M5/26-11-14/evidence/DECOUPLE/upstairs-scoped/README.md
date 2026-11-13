# DECOUPLE / upstairs-scoped —— 目录迁移后的 scoped BT 回归

- 日期：2026-11-14
- 窗口：`DECOUPLE-upstairs-r1`（回执见 `scripts/verify/_window_receipt.tsv`）
- source：`529e65b4`（三项目守卫与 P2 install-space 测试）；运行时代码同 HEAD
- 性质：**scoped 回归**，不新增 formal R 编号；declared-map Gazebo 运动学链，不签四足物理/真机
- 目的：验证 `framework/` 分目录、P1/P2/P3 解耦后，`app_runner → navigation_bt → Gazebo` 现有功能不变

## 命令

```bash
# 容器 ros1-slam-sim-viz，仓库 /opt/ros1-slam
V8_FORMAL_ALIAS_ROOT=/home/ros1-slam \
python3 -B scripts/sim/two_floor_nav.py --case upstairs --execute --orchestrator bt \
  --window-id DECOUPLE-upstairs-r1 \
  --out note/进度/v8/M5/26-11-14/evidence/DECOUPLE/upstairs-scoped/summary.json \
  --port 11657 --timeout-s 120
```

## 读数

- `summary.json`：`ok=true`、`preflight_rc=0`、`app_rc=0`、`validation_error=null`。
- 六项 checks 全 true：`assembly_ready / map_configured / global_path / local_arrived /
  carrier_at_local_goal / markers_published`。
- 启动序：`map → global → bt → local → mux3d → carrier`；BT 组件在图内。
- 全局路径 57 poses，末点 `(-5.9,1.1,1.6)`；局部 `reason=arrived`；载体
  `carrier_final=(-5.748855,1.121592,1.6)`；markers 40。
- trace：pose 2653、gazebo_model 2630、command 2623、local_status 1758、
  local_trajectory 26、global_path 1、global_status 4；最大 xy 误差 0.02923 m，
  最大 z 误差 0.21399 m，最大命令/模型/位姿间隔均 ≤0.065 s。
- 与 r3 `carrier=(-5.76142,1.11986,1.6)`、r2 `(-5.7597,1.1200,1.6)` 同路径、同到达；
  末点差异约 1.2 cm，属运行间波动。

## 判读边界

- 不声称四足步态/物理接触、真机、实时建图或完整全局走廊；地图是 declared 几何，载体是
  Gazebo 运动学 `SetModelState`。
- 本跑单独不记 BT 事件日志；BT 参与由启动序与全局路径/到达链证明。
- `.runtime/` 为运行日志，不入库；`summary.trace.json.gz` 为压缩 trace。

## 文件

- `summary.json` / `summary.app.json` / `summary.preflight.log` / `summary.trace.json.gz` / `hashes.sha256`。
