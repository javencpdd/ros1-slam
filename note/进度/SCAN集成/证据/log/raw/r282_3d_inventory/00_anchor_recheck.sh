#!/bin/bash
# R282 锚点回验：盘点单 al 稿里每条「逐字原句」必须能在它所声称的出处命中（漏一条即本件报红）
# ★第一跑（05:50）本件自身有两个缺陷：①一条"占位"检查白计入 fail；②rc=0 那行写在判定之前 ⇒ 无条件先打印"全命中"。
#   都是"打印面错、判定面也可能错"那一族，改法＝删占位、判定只出口一次。正文引用无一处靠这两条缺陷蒙过。
set -u
cd /home/ros1-slam
D=note/进度/SCAN集成/设计
S=ws/src/95_simulation/lite3_gazebo_sim
R=note/进度/SCAN集成/证据/log/raw
R64=note/进度/仿真测试台/证据/raw/r64_stair_fourth_channel_precheck
fail=0; tot=0
chk(){ tot=$((tot+1)); if grep -qF -- "$2" "$1" 2>/dev/null; then printf 'OK    %-22s ← %s\n' "$(basename "$1")" "$2"; else printf '★不符 %-22s ← %s\n' "$(basename "$1")" "$2"; fail=$((fail+1)); fi; }
echo "== m 稿（第五节原句）=="
chk "$D/m_3d多层_载体缺口与出路.md" '对您问题的直接回答＝没有完成'
chk "$D/m_3d多层_载体缺口与出路.md" '没有实现、没有一次活体读数'
chk "$D/m_3d多层_载体缺口与出路.md" '定位仿真❌（hdl segv 转真机 G8）'
chk "$D/m_3d多层_载体缺口与出路.md" '局部✅（teb 定型，DWA 裁撤）／组合✅（双 PASS）／多地图✅（perlin3d 除外）'
chk "$D/m_3d多层_载体缺口与出路.md" '漂移 1.5cm／0.06%'
echo "== n 稿／请求单／R82 留底（丙组 C1）=="
chk "$D/n_自遮挡过滤_球形blind换机体OBB.md" '这一面仿真测不了'
chk "$D/n_自遮挡过滤_球形blind换机体OBB.md" '带腿＋地面碰撞的机体件'
chk "note/进度/00_裁定请求单_2026-10-09.md" '整个传感器所属模型'
chk "note/进度/00_裁定请求单_2026-10-09.md" '近场地面点一个都没有'
chk "$R/r82_selfmask_20261010_042738/probe_stdout.txt" '无自回波'
echo "== 计划书 v5.3（丙组 C3／§0.14／C12）=="
chk "note/计划书v5/21_ros1-slam隔离部署计划书-v5.3.md" '不能表达非零机体朝向'
chk "note/计划书v5/21_ros1-slam隔离部署计划书-v5.3.md" '有出处≠已标定，实测锚＝0 处，不得作真机能力上限'
chk "note/计划书v5/21_ros1-slam隔离部署计划书-v5.3.md" '暂时搁置、放着存档即可'
chk "note/计划书v5/21_ros1-slam隔离部署计划书-v5.3.md" '仍须人类另行明文'
echo "== yaml／world／README（乙组 B2／B5、丙组 C4）=="
chk "ws/src/70_bringup/lite3_bringup/config/elevation_costmap.yaml" '未标定施工值（实测锚＝0 处）'
chk "ws/src/95_simulation/lite3_gazebo_sim/worlds/m1_world.world" '<max_step_size>0.001</max_step_size>'
chk "ws/src/95_simulation/lite3_gazebo_sim/worlds/m1_world.world" 'model://ground_plane'
chk "$S/stairstudy/worlds/stair_lab_stairs_easy.world" '<size>0.02 1.200 0.060</size>'
chk "$S/README.md" '不能验 ⑤（踏步/落脚/摩擦）'
chk "$R64/index.txt" '不被任何在册 launch 引用'
echo "== r64 读数与预检件（甲组 A4）=="
chk "$R64/index.txt" 'peri_amp 四档全 0'
chk "$R64/index.txt" '八格全不可分'
chk "$R64/index.txt" '加 2cm 噪声后 0.15 档翻不可分'
chk "$S/stairstudy/precheck.py" '判据可行性预检'
echo "== 设计件与工单（乙组 B3、§0 数字）=="
chk "$D/ag_3Dspike_施工工单_待授权.md" '增第二实例'
chk "$D/ak_v4.3_X1X2X5复核_小张评审_落册输入.md" '多层／悬挑类场景**不得**以 `/elevation_grid`'
echo "== R280／R281 留底（甲组 A1–A3）=="
chk "$R/r280_s1s2_landing/00_selftest_raw.txt" 'W3_ELEV_SELFTEST PASS=10 FAIL=0'
chk "$R/r281_s1_offline_smoke/01_smoke_raw_v2.txt" 'S1_OFFLINE_SMOKE 帧数=14'
chk "$R/r281_s1_offline_smoke/01_smoke_raw_v2.txt" 'VERDICT: rc=0 无违反项'
echo "== 面差实录（本批不入 claim_face，请示挂 #165）=="
echo "盘上 设计/*.md=$(ls "$D"/*.md | wc -l) 件｜claim_face 内 设计/ 条目=$(grep -c '设计/' scripts/verify/claim_face.txt) 件｜claim_face 总条目=$(grep -vcE '^ *(#|$)' scripts/verify/claim_face.txt) 件"
echo "未入面者＝$(comm -13 <(grep '设计/' scripts/verify/claim_face.txt | sed 's#.*/##' | sort) <(ls "$D"/*.md | xargs -n1 basename | sort) | tr '\n' ' ')"
echo "=================================================="
echo "回验条数＝$tot 条｜未命中＝$fail 条"
if [ "$fail" = 0 ]; then echo "AL_ANCHOR_VERDICT: rc=0 全部锚点在其声称的出处逐字命中"; exit 0; fi
echo "AL_ANCHOR_VERDICT: rc=5 有 $fail 条锚点未命中（正文须再改或降为转述）"; exit 5
