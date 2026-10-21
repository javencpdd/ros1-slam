#!/bin/bash
# R265 普查：对每个「志文里被反复引用的数」当场跑一遍它的候选命令，把式子与读数一起留底。
# 零改动：只读文件系统与 git，不跑节点、不起窗、不改任何件。
cd /home/ros1-slam || exit 1
echo "== date: $(date '+%F %H:%M')  == whoami: $(whoami) =="
echo
echo "### A 组：一条命令跑出唯一值（可当场复跑）"
echo "[A1] claim_face 条目（口径＝非注释非空行）"
printf '   cmd: grep -cvE "^ *(#|$)" scripts/verify/claim_face.txt => '
grep -cvE '^ *(#|$)' scripts/verify/claim_face.txt
echo "[A2] note/ 下 .md 全量"
printf '   cmd: find note -name "*.md" -type f | wc -l => '
find note -name '*.md' -type f | wc -l
echo "[A3] parity_face 条目"
printf '   cmd: grep -cvE "^ *(#|$)" scripts/verify/parity_face.txt => '
grep -cvE '^ *(#|$)' scripts/verify/parity_face.txt
echo "[A4] PARITY 存量表条目"
printf '   cmd: grep -cvE "^ *(#|$)" scripts/verify/parity_known_findings.tsv => '
grep -cvE '^ *(#|$)' scripts/verify/parity_known_findings.tsv
echo "[A5] 本会话批次数（git 可数）"
printf '   cmd: git log --oneline 3ae4404~1..HEAD -- note | wc -l => '
git log --oneline 3ae4404~1..HEAD -- note | wc -l
echo "[A6] 05 追加块数（口径＝### 追加X 标题行）"
printf '   cmd: grep -cE "^### 追加[一二三四五六七八九十]+" note/05_复验作业单.md => '
grep -cE '^### 追加[一二三四五六七八九十]+' note/05_复验作业单.md
echo "[A7] tooling/skill 镜像件数（口径＝排除清单自身与 RESTORE.sh）"
printf '   cmd: find tooling/skill -type f ! -name _sha256_manifest.txt ! -name RESTORE.sh | wc -l => '
find tooling/skill -type f ! -name '_sha256_manifest.txt' ! -name 'RESTORE.sh' | wc -l
echo "[A8] tooling/skill 清单登记数"
printf '   cmd: grep -cvE "^ *(#|$)" tooling/skill/_sha256_manifest.txt => '
grep -cvE '^ *(#|$)' tooling/skill/_sha256_manifest.txt
echo "[A9] 清单里列出的路径实际不存在者（在 tooling/skill 下逐项 test -e）"
miss=0
while read -r h p; do [ -e "./tooling/skill/${p#./}" ] || miss=$((miss+1)); done < tooling/skill/_sha256_manifest.txt
echo "   missing=$miss"
echo
echo "### B 组：同一量两条式子都给数（口径未绑定 ⇒ 须写明用哪条）"
echo "[B1] 「志文面」三式"
printf '   清单口径 grep -cvE "^ *(#|$)" claim_face.txt => '
grep -cvE '^ *(#|$)' scripts/verify/claim_face.txt
printf '   原始行数 wc -l claim_face.txt => '
wc -l < scripts/verify/claim_face.txt
printf '   全量 find note -name "*.md" => '
find note -name '*.md' -type f | wc -l
echo "[B2] 「豁免表条数」两式"
printf '   非注释行 grep -cvE "^ *(#|$)" citation_known_pseudo_reds.tsv => '
grep -cvE '^ *(#|$)' scripts/verify/citation_known_pseudo_reds.tsv
printf '   表头行（首行是否表头） => '
head -1 scripts/verify/citation_known_pseudo_reds.tsv | cut -c1-24
echo "[B3] 「坑册条目数」三式"
printf '   标题行 grep -cE "^## 坑" => '
grep -cE '^## 坑' note/02_问题与坑.md
printf '   三级标题 grep -cE "^### 坑" => '
grep -cE '^### 坑' note/02_问题与坑.md
printf '   去重坑号 => '
grep -oE '^#+ 坑[0-9]+' note/02_问题与坑.md | grep -oE '[0-9]+' | sort -n -u | wc -l
printf '   最大坑号 => '
grep -oE '^#+ 坑[0-9]+' note/02_问题与坑.md | grep -oE '[0-9]+' | sort -n | tail -1
echo "[B4] 「tooling 目录文件数」两式（含/不含清单与恢复脚本自身）"
printf '   find tooling -type f => '
find tooling -type f | wc -l
printf '   find tooling/skill -type f => '
find tooling/skill -type f | wc -l
echo
echo "### C 组：没有命令能跑出该数（台账累计值／须人工判定）"
echo "[C1] 「声明面＝65」——在册句是每批手写的覆盖面声明；无 face 文件、无件打印此数"
printf '   能否 grep 出唯一命令？件内是否打印过 "声明面"：'
grep -rl "声明面" scripts/ 2>/dev/null | wc -l
echo "[C2] 「正式门禁串＝八件」——串成员只在 05 的散文句里，无清单文件、无运行器脚本"
printf '   scripts/ 下含 "门禁串" 的件数：'
grep -rl "门禁串" scripts/ 2>/dev/null | wc -l
printf '   scripts/verify 下 .sh 件数（可跑，但不是"串成员数"）：'
ls scripts/verify/*.sh | wc -l
echo "[C3] 「远角 4 对」——留底 attempt 目录可数，"对"须人工配对"
printf '   留底 attempt 目录数 find r174_b18_farcorner_diag -maxdepth 1 -type d -name "attempt*" => '
find note/进度/SCAN集成/证据/log/raw/r174_b18_farcorner_diag -maxdepth 1 -type d -name 'attempt*' | wc -l
printf '   在册句里的对数（grep 计数，不作读数用）：'
grep -rhoE '远角 4 对' note/ --include=*.md | wc -l
echo "[C4] 「w 稿移交单第七次复跑＝七轮零恶化」——轮次靠留底目录与在册句对起来，无件自报轮次"
printf '   含 w 稿复跑留底的目录数：'
find note -type d -name '*handoff*' -o -type d -name '*transfer*' | wc -l
echo "[C5] 「活体窗数（A27 口径）」——目录命名不统一，两条模式给出不同数"
printf '   模式A find note -type d | grep -cE "seed|witness" => '
find note -type d | grep -cE 'seed|witness'
printf '   模式B find note -type d -name "r2[0-9][0-9]_*" => '
find note -type d -name 'r2[0-9][0-9]_*' | wc -l
