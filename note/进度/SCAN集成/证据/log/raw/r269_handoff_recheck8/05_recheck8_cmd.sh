#!/bin/bash
# R269 第八次移交项清偿复跑（只读；零窗、零改件、他人志文不代改）
cd /home/ros1-slam || exit 1
V4="note/计划书v4/21_ros1-slam隔离部署计划书-v4.md"
FD="note/进度/SCAN集成/设计/fd_双写者与前馈源裁定.md"
cnt(){ local f="$1" pat="$2"; [ -f "$f" ] || { echo "件不存在"; return; }; printf '处=%-3s 行=%-3s' "$(grep -oE "$pat" "$f" 2>/dev/null | wc -l)" "$(grep -cE "$pat" "$f" 2>/dev/null)"; }
last(){ git log -1 --format='%h %ad' --date=short -- "$1" 2>/dev/null; }
echo "# 00_第八次移交项清偿复跑（R269，只读）｜$(date '+%F %H:%M:%S')｜身份 $(whoami)"
echo "## 一、逐张单（处＝匹配次数／行＝含匹配的行数，两单位并列＝A32⑧；式子逐条写出＝动作㉓′）"
echo "① 06 稿引 launch 冒号行号"
printf '   cmd: grep -oE "launch[:：][0-9]+" note/06_三态通道普查.md | wc -l  ⇒ '; cnt note/06_三态通道普查.md 'launch[:：][0-9]+'; echo "   最后改动=$(last note/06_三态通道普查.md)"
echo "② 仿真测试台进度文档 引 CMakeLists:192（冒号形）"
printf '   cmd: grep -oE "CMakeLists\\.txt[:：]192" …/仿真测试台/进度文档.md | wc -l  ⇒ '; cnt note/进度/v7/S3/仿真测试台/进度文档.md 'CMakeLists\.txt[:：]192'; echo "   最后改动=$(last note/进度/v7/S3/仿真测试台/进度文档.md)"
echo "② 同一处的中文形（对照：式子差不是清偿）"
printf '   cmd: grep -oE "第 ?192 ?行"（限该件）⇒ '; cnt note/进度/v7/S3/仿真测试台/进度文档.md '第 ?192 ?行'; echo
echo "②′ 04 同形 / README 同形"
printf '   04     ⇒ '; cnt note/04_配置改动diff.md 'CMakeLists\.txt[:：]192'; echo "   最后改动=$(last note/04_配置改动diff.md)"
printf '   README ⇒ '; cnt note/00_README.md 'CMakeLists\.txt[:：]192'; echo "   最后改动=$(last note/00_README.md)"
echo "③ P5 引 CMakeLists:40"
printf '   cmd: grep -oE "CMakeLists\\.txt[:：]40" note/进度/v7/S4/P5/进度文档.md | wc -l  ⇒ '; cnt note/进度/v7/S4/P5/进度文档.md 'CMakeLists\.txt[:：]40'; echo "   最后改动=$(last note/进度/v7/S4/P5/进度文档.md)"
echo "④ M2 引 buffer_core.cpp 冒号行号"
printf '   cmd: grep -oE "buffer_core\\.cpp[:：][0-9]+" note/进度/v7/S2/M2/进度文档.md | wc -l  ⇒ '; cnt note/进度/v7/S2/M2/进度文档.md 'buffer_core\.cpp[:：][0-9]+'; echo "   最后改动=$(last note/进度/v7/S2/M2/进度文档.md)"
echo "⑤ fd 稿引 v_scan_p{1,2}.sh 冒号行号"
printf '   cmd: grep -oE "v_scan_p[12]\\.sh[:：][0-9]+" %s | wc -l  ⇒ ' "$FD"; cnt "$FD" 'v_scan_p[12]\.sh[:：][0-9]+'; echo "   最后改动=$(last "$FD")"
echo "⑦ 计划书 v4.1 引 CMakeLists:94"
V41="note/计划书v4/21_ros1-slam隔离部署计划书-v4.1-雷达翻案补丁.md"
printf '   cmd: grep -oE "CMakeLists\\.txt[:：]94" v4.1 件 ⇒ '; cnt "$V41" 'CMakeLists\.txt[:：]94'; echo "   最后改动=$(last "$V41")"
echo "单8 fd 稿按行号引 v4 正文 829／889"
printf '   cmd: grep -oE "v4[:：](829|889)" 于 fd 稿 ⇒ '; cnt "$FD" 'v4[:：](829|889)'; echo "   最后改动=$(last "$FD")"
echo "## 二、负面对照（盘上到底有没有那件）"
echo "   cmd: find ws/src opt -name buffer_core.cpp ⇒ 命中 $(find ws/src opt -name 'buffer_core.cpp' 2>/dev/null | wc -l) 个"
echo "   cmd: wc -l < ws/src/95_simulation/lite3_gazebo_sim/launch/m2_mapping.launch ⇒ $(wc -l < ws/src/95_simulation/lite3_gazebo_sim/launch/m2_mapping.launch 2>/dev/null) 行（单① 的越界判据）"
echo "## 三、单8 漂移量（本次仍量化到行）"
echo "   v4 现 $(( $(wc -l < "$V4") )) 行（R257 时亦为此值须自比）"
echo "   实测 v4 第 829 行现行内容＝'$(sed -n '829p' "$V4" | cut -c1-46)'"
echo "   实测 v4 第 889 行现行内容＝'$(sed -n '889p' "$V4" | cut -c1-46)'"
m1=$(grep -nE 'M1' "$V4" | grep -E '频率|Hz' | head -1 | cut -d: -f1)
q4=$(grep -nE '^\| Q4 ' "$V4" | head -1 | cut -d: -f1)
echo "   真句现在所在行＝M1 频率门 第 ${m1:-未命中} 行（差 $((${m1:-0}-829))）；Q4 行 第 ${q4:-未命中} 行（差 $((${q4:-0}-889))）"
echo "## 四、面对账（跑前先断言"这次只读数没顺手改志文"）"
A=""; for f in $(grep -v "^ *#" scripts/verify/claim_face.txt); do A="$A $PWD/$f"; done
bash scripts/verify/v_citation_check.sh $A 2>&1 | grep -E '^\[计数\]'
bash scripts/verify/v_ledger_citation_baseline.sh --check note/进度/SCAN集成/证据/log/raw/r233_vcc_window40/baseline_note63w40_r233.tsv $A 2>&1 | grep -E 'LCB VERDICT'
echo "## 五、八轮合起来的读法（不据此收口）"
echo "   本轮＝第八次（前七次 R116／R128／R166／R175／R183／R197／R257）"
