# data/waypoints/ —— 航点目录（C6，19 号 W7）

本目录存放任务航点 JSON，由 M5 任务层（TaskTransfer.py，C5 改写版）消费。

## 录航点工具（v4 C6 修正口径）
- 工具路径：`ws/src/50_task/pipeline_tracking/tools/location_record.py`（注意在 **tools/** 子目录，非 scripts/）
- 依赖 PyQt5 图形环境：需 `ssh -X` 或现场 HDMI（Q5 开）
- 退化方案：手写 JSON（键位与既有航点文件一致即可），落在本目录

## 约定
- P3a 仅落目录占位（本 README）；航点 JSON 在 P3b/P6 现场录制后生成
- 与同级 `data/maps/`（建图三件套 .pcd/.yaml/.pgm，Gate G3）无耦合依赖

---

## ★同步戳（追加节·data/waypoints；上方旧文一字未回改）★

- **本目录当前状态（现跑 `ls -A`）**＝**仍只有 `README.md` 一枚**，
  目录内**无任何航点 JSON** ⇒ 上方「P3a 仅落目录占位；
  航点 JSON 在 P3b／P6 现场录制后生成」那句**仍然成立、未过期**。
- **录制工具在位（现跑 `test -f`）**＝
  `ws/src/50_task/pipeline_tracking/tools/location_record.py` ✔
  （★注意＝在 `tools/` 子目录，不是 `scripts/`★）。
- **消费方在位**＝`ws/src/50_task/pipeline_tracking/scripts/TaskTransfer.py` ✔。
- **与同级 `data/maps/` 的关系**＝**无耦合依赖**
  （`maps/` 现跑 6 枚＝`lite3_1007.pcd`／`lite3_1007_polished.pcd`／`lite3.pcd`／
  `lite3.pgm`／`lite3.yaml`／`sim_venue.pcd`＝Gate G3 建图三件套）。
- 对齐到＝进度真身末号 **R518** ／ 本 boot **批174**；同步时刻＝2026-10-21 03:32 CST。
- ★上方正文一字未回改★。
