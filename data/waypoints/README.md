# data/waypoints/ —— 航点目录（C6，19 号 W7）

本目录存放任务航点 JSON，由 M5 任务层（TaskTransfer.py，C5 改写版）消费。

## 录航点工具（v4 C6 修正口径）
- 工具路径：`ws/src/50_task/pipeline_tracking/tools/location_record.py`（注意在 **tools/** 子目录，非 scripts/）
- 依赖 PyQt5 图形环境：需 `ssh -X` 或现场 HDMI（Q5 开）
- 退化方案：手写 JSON（键位与既有航点文件一致即可），落在本目录

## 约定
- P3a 仅落目录占位（本 README）；航点 JSON 在 P3b/P6 现场录制后生成
- 与同级 `data/maps/`（建图三件套 .pcd/.yaml/.pgm，Gate G3）无耦合依赖
