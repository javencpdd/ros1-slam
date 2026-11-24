# VCC RED=39 的逐条勘误对照（2026-11-25）

> 口径：**原文一字不回改**（在册规矩：原文＋另起勘误句），只在引用后补勘误括注；不删除任何判据实现、不动 VCC 本件、不改任何门值。基线来源：`note/进度/00_专题/00_VCC历史RED基线_2026-11-23.tsv`（39 条）。
> 本表 16 个唯一 (目标, 行号) 组合覆盖全部 39 条；同一组合被多处引用时逐处补注。

## 1. 16 个组合与真身

| # | 原引用 | 条数 | 真身（核实） | 证据命令/出处 |
| --- | --- | --- | --- | --- |
| 1 | `note/进度/v7/S3/进度文档.md:972` | 8 | note/进度/v7/S1/SCAN集成/进度文档.md:972（1001 行） | `wc -l note/进度/v7/S1/SCAN集成/进度文档.md`＝1001；批207（7fe53237）提交正文记「真身是 SCAN集成 那份 1001 行的第 972 行」 |
| 2 | `ws/src/20_localization/hdl_localization/launch/hdl_localization.launch:89` | 12 | ws/src/20_localization/lite3_localization/launch/hdl_localization.launch:89（壳包，第 89 行为 /odom remap 说明） | `wc -l` 第三方原件＝70 行（<89）；`sed -n 89p` 壳包为 /odom 契约注释，与引文语义一致；批207 正文亦称「ws/ 侧 hdl_localization.launch:89 存量」 |
| 3 | `ws/src/95_simulation/lite3_gazebo_sim/urdf/lite3_planar.urdf.xacro:140` | 4 | subprojects/simulation/ws/src/lite3_gazebo_sim/urdf/lite3_planar.urdf.xacro | `ls` 该路径在位；`wc -l` 覆盖第 140 行（分仓迁移，内容同源） |
| 4 | `ws/src/80_algorithm/v8_algorithm/local_planner/fixtures.py:113` | 2 | **已失效**（该件 28eccbce 由 v8_platform/local_planner 迁入后仅 45 行） | `wc -l`＝45（<113）；`git log --follow --name-status` 显示 R093 迁入 |
| 5 | `ws/src/95_simulation/config/faster_lio_sim.yaml:29` | 2 | subprojects/simulation/ws/src/lite3_gazebo_sim/config/faster_lio_sim.yaml | `ls` 该路径在位；`wc -l` 覆盖第 29 行（分仓迁移，内容同源） |
| 6 | `ws/src/95_simulation/config/faster_lio_sim.yaml:37` | 1 | subprojects/simulation/ws/src/lite3_gazebo_sim/config/faster_lio_sim.yaml | `ls` 该路径在位；`wc -l` 覆盖第 37 行（分仓迁移，内容同源） |
| 7 | `ws/src/CMakeLists.txt:79` | 1 | **已失效**（符号链，catkin toplevel 仅 69 行） | `ls -l` 显示指向 /opt/ros/noetic/share/catkin/cmake/toplevel.cmake；容器内 `wc -l`＝69（<79）；批2（1d659b6a）已摘跟踪 |
| 8 | `ws/src/95_simulation/lite3_gazebo_sim/launch/m2_hdl.launch:6` | 1 | subprojects/simulation/ws/src/lite3_gazebo_sim/launch/m2_hdl.launch | `ls` 该路径在位；`wc -l` 覆盖第 6 行（分仓迁移，内容同源） |
| 9 | `ws/src/95_simulation/lite3_gazebo_sim/launch/m2_mapping.launch:11` | 1 | subprojects/simulation/ws/src/lite3_gazebo_sim/launch/m2_mapping.launch | `ls` 该路径在位；`wc -l` 覆盖第 11 行（分仓迁移，内容同源） |
| 10 | `ws/src/95_simulation/lite3_gazebo_sim/src/lite3_planar_drive.cpp:28` | 1 | subprojects/simulation/ws/src/lite3_gazebo_sim/src/lite3_planar_drive.cpp | `ls` 该路径在位；`wc -l` 覆盖第 28 行（分仓迁移，内容同源） |
| 11 | `ws/src/95_simulation/lite3_gazebo_sim/src/lite3_planar_drive.cpp:111` | 1 | subprojects/simulation/ws/src/lite3_gazebo_sim/src/lite3_planar_drive.cpp | `ls` 该路径在位；`wc -l` 覆盖第 111 行（分仓迁移，内容同源） |
| 12 | `ws/src/95_simulation/lite3_gazebo_sim/verify/m2_smoke.sh:78` | 1 | subprojects/simulation/ws/src/lite3_gazebo_sim/verify/m2_smoke.sh | `ls` 该路径在位；`wc -l` 覆盖第 78 行（分仓迁移，内容同源） |
| 13 | `95_simulation/config/faster_lio_sim.yaml:18` | 1 | subprojects/simulation/ws/src/lite3_gazebo_sim/config/faster_lio_sim.yaml | `ls` 该路径在位；`wc -l` 覆盖第 18 行（分仓迁移，内容同源） |
| 14 | `ws/src/95_simulation/lite3_gazebo_sim/launch/m2_mapping.launch:7` | 1 | subprojects/simulation/ws/src/lite3_gazebo_sim/launch/m2_mapping.launch | `ls` 该路径在位；`wc -l` 覆盖第 7 行（分仓迁移，内容同源） |
| 15 | `95_simulation/config/faster_lio_sim.yaml:38` | 1 | subprojects/simulation/ws/src/lite3_gazebo_sim/config/faster_lio_sim.yaml | `ls` 该路径在位；`wc -l` 覆盖第 38 行（分仓迁移，内容同源） |
| 16 | `ws/src/95_simulation/mockamap/src/maps.cpp:599` | 1 | subprojects/simulation/ws/src/mockamap/src/maps.cpp | `ls` 该路径在位；`wc -l` 覆盖第 599 行（分仓迁移，内容同源） |

## 2. 逐条归因（与基线表同序）

| 序 | 引用方 | 原引用 | 类别 | 处置 |
| --- | --- | --- | --- | --- |
| 1 | 01_批次.md | `note/进度/v7/S3/进度文档.md:972` | 原位 | 补勘误句指向真身 |
| 2 | 01_批次.md | `ws/src/20_localization/hdl_localization/launch/hdl_localization.launch:89` | 原位 | 补勘误句指向真身 |
| 3 | 02_问题与坑.md | `ws/src/20_localization/hdl_localization/launch/hdl_localization.launch:89` | 原位 | 补勘误句指向真身 |
| 4 | 02_问题与坑.md | `ws/src/95_simulation/lite3_gazebo_sim/urdf/lite3_planar.urdf.xacro:140` | 直映射（分仓路径） | 补勘误句指向真身 |
| 5 | 04_配置改动diff.md | `note/进度/v7/S3/进度文档.md:972` | 原位 | 补勘误句指向真身 |
| 6 | 04_配置改动diff.md | `note/进度/v7/S3/进度文档.md:972` | 原位 | 补勘误句指向真身 |
| 7 | 04_配置改动diff.md | `note/进度/v7/S3/进度文档.md:972` | 原位 | 补勘误句指向真身 |
| 8 | 04_配置改动diff.md | `note/进度/v7/S3/进度文档.md:972` | 原位 | 补勘误句指向真身 |
| 9 | 04_配置改动diff.md | `note/进度/v7/S3/进度文档.md:972` | 原位 | 补勘误句指向真身 |
| 10 | 04_配置改动diff.md | `note/进度/v7/S3/进度文档.md:972` | 原位 | 补勘误句指向真身 |
| 11 | 04_配置改动diff.md | `note/进度/v7/S3/进度文档.md:972` | 原位 | 补勘误句指向真身 |
| 12 | 04_配置改动diff.md | `ws/src/20_localization/hdl_localization/launch/hdl_localization.launch:89` | 原位 | 补勘误句指向真身 |
| 13 | 04_配置改动diff.md | `ws/src/20_localization/hdl_localization/launch/hdl_localization.launch:89` | 原位 | 补勘误句指向真身 |
| 14 | 04_配置改动diff.md | `ws/src/20_localization/hdl_localization/launch/hdl_localization.launch:89` | 原位 | 补勘误句指向真身 |
| 15 | 04_配置改动diff.md | `ws/src/20_localization/hdl_localization/launch/hdl_localization.launch:89` | 原位 | 补勘误句指向真身 |
| 16 | 04_配置改动diff.md | `ws/src/20_localization/hdl_localization/launch/hdl_localization.launch:89` | 原位 | 补勘误句指向真身 |
| 17 | 04_配置改动diff.md | `ws/src/20_localization/hdl_localization/launch/hdl_localization.launch:89` | 原位 | 补勘误句指向真身 |
| 18 | 04_配置改动diff.md | `ws/src/20_localization/hdl_localization/launch/hdl_localization.launch:89` | 原位 | 补勘误句指向真身 |
| 19 | 04_配置改动diff.md | `ws/src/20_localization/hdl_localization/launch/hdl_localization.launch:89` | 原位 | 补勘误句指向真身 |
| 20 | 04_配置改动diff.md | `ws/src/80_algorithm/v8_algorithm/local_planner/fixtures.py:113` | 原位 | 已失效 → 已在引用处补注失效原因（不指新位置） |
| 21 | 04_配置改动diff.md | `ws/src/95_simulation/config/faster_lio_sim.yaml:29` | 补 lite3_gazebo_sim 段（原引用缺段） | 补勘误句指向真身 |
| 22 | 04_配置改动diff.md | `ws/src/95_simulation/config/faster_lio_sim.yaml:37` | 补 lite3_gazebo_sim 段（原引用缺段） | 补勘误句指向真身 |
| 23 | 04_配置改动diff.md | `ws/src/CMakeLists.txt:79` | 未找到 | 已失效 → 已在引用处补注失效原因（不指新位置） |
| 24 | 05_复验作业单.md | `ws/src/20_localization/hdl_localization/launch/hdl_localization.launch:89` | 原位 | 补勘误句指向真身 |
| 25 | 05_复验作业单.md | `ws/src/80_algorithm/v8_algorithm/local_planner/fixtures.py:113` | 原位 | 已失效 → 已在引用处补注失效原因（不指新位置） |
| 26 | fd_双写者与前馈源裁定.md | `ws/src/20_localization/hdl_localization/launch/hdl_localization.launch:89` | 原位 | 补勘误句指向真身 |
| 27 | fd_双写者与前馈源裁定.md | `ws/src/95_simulation/lite3_gazebo_sim/launch/m2_hdl.launch:6` | 直映射（分仓路径） | 补勘误句指向真身 |
| 28 | fd_双写者与前馈源裁定.md | `ws/src/95_simulation/lite3_gazebo_sim/launch/m2_mapping.launch:11` | 直映射（分仓路径） | 补勘误句指向真身 |
| 29 | fd_双写者与前馈源裁定.md | `ws/src/95_simulation/lite3_gazebo_sim/src/lite3_planar_drive.cpp:28` | 直映射（分仓路径） | 补勘误句指向真身 |
| 30 | fd_双写者与前馈源裁定.md | `ws/src/95_simulation/lite3_gazebo_sim/src/lite3_planar_drive.cpp:111` | 直映射（分仓路径） | 补勘误句指向真身 |
| 31 | fd_双写者与前馈源裁定.md | `ws/src/95_simulation/lite3_gazebo_sim/urdf/lite3_planar.urdf.xacro:140` | 直映射（分仓路径） | 补勘误句指向真身 |
| 32 | fd_双写者与前馈源裁定.md | `ws/src/95_simulation/lite3_gazebo_sim/urdf/lite3_planar.urdf.xacro:140` | 直映射（分仓路径） | 补勘误句指向真身 |
| 33 | fd_双写者与前馈源裁定.md | `ws/src/95_simulation/lite3_gazebo_sim/verify/m2_smoke.sh:78` | 直映射（分仓路径） | 补勘误句指向真身 |
| 34 | l_fasterlio_轴向丢点候选环节.md | `95_simulation/config/faster_lio_sim.yaml:18` | 补 lite3_gazebo_sim 段（原引用缺段） | 补勘误句指向真身 |
| 35 | l_fasterlio_轴向丢点候选环节.md | `ws/src/95_simulation/lite3_gazebo_sim/launch/m2_mapping.launch:7` | 直映射（分仓路径） | 补勘误句指向真身 |
| 36 | n_自遮挡过滤_球形blind换机体OBB.md | `95_simulation/config/faster_lio_sim.yaml:38` | 补 lite3_gazebo_sim 段（原引用缺段） | 补勘误句指向真身 |
| 37 | v_台账引用勘误对照表.md | `ws/src/95_simulation/config/faster_lio_sim.yaml:29` | 补 lite3_gazebo_sim 段（原引用缺段） | 补勘误句指向真身 |
| 38 | v_台账引用勘误对照表.md | `ws/src/95_simulation/mockamap/src/maps.cpp:599` | 直映射（分仓路径） | 补勘误句指向真身 |
| 39 | 进度文档.md | `ws/src/95_simulation/lite3_gazebo_sim/urdf/lite3_planar.urdf.xacro:140` | 直映射（分仓路径） | 补勘误句指向真身 |

## 3. 落地方式

- 只在**真引用**后插括注；同句已含「写成／实为／勘误／已删」等勘误关键词的段落**不重复插**（那些本就判 FIXUP 不判红）。
- 插入**不新增行**：所有改动都是在原行内追加括注，因此 `文件:行号` 引用面不变（行数逐一复核，04 段数仍 529）。
- 失效项不指向新位置，只写明失效原因，避免制造一个「看起来能解析」的假引用。

## 4. 复跑读数（2026-11-25）

命令（与基线同面）：

```bash
VCC_ROOT=<仓根> VCC_MAX=500 bash scripts/verify/v_citation_check.sh \
  $(grep -vE "^ *(#|$)" scripts/verify/claim_face.txt)
```

| 计数项 | 基线（11-23） | 本次（11-25） | 说明 |
| --- | --- | --- | --- |
| RED | **39** | **0** | 39 条全部转为勘误句或指向真身 |
| rc | 6 | 5 | 5＝仍有 AMBER，非干净；**不称"干净"** |
| AMBER | 35 | 35 | 未动（简写件名族，坑52 口径，不在本批范围） |
| 勘误句 | 75 | 159 | 新增 43 处括注（含指向真身时带入的新单点引用） |
| 单点引用 | 744 | 785 | 增加 41＝括注里"实为 X:Y"带来的新引用 |
| OK | 588 | 585 | 减少 3：3 条原本 OK 的引用因同句 ±40 字内出现勘误关键词，按 VCC 口径转判 FIXUP |
| 豁免红 | 6 | 6 | 未动 |

★三件必须说清的事★：

1. **判据实现一字未改**：`v_citation_check.sh` 未编辑（其自报 sha＝`223a7decf861`，与基线一致），
   没有删除任何判据分支、没有调松窗口、没有加豁免。
2. **RED=0 不等于零缺陷**：OK 由 588→585，是 3 条正确引用被"同句勘误"吸收所致——这是
   VCC 在册口径（同句有勘误关键词即判 FIXUP 不判红）的已知副作用，本表如实登记，不用它冒充"更干净"。
3. **"确实失效"已登记且不指假位置**：`ws/src/CMakeLists.txt:79` 与
   `v8_algorithm/local_planner/fixtures.py:113` 两条不指向任何新位置，只写明失效原因
   （前者是 catkin toplevel 符号链、批2 起摘跟踪且仅 69 行；后者迁入后仅 45 行）。
