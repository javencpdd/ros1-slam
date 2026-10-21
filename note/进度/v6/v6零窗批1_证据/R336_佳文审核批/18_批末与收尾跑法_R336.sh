set -u
cd /home/ros1-slam
D=note/进度/v6/v6零窗批1_证据/R336_佳文审核批
echo "# R336 批末与收尾｜起 $(date '+%F %T')｜身份 $(id -un)｜★零窗（链路第三环＋五补落地批）"
bash scripts/verify/run_gate_chain.sh 2>&1 | grep -vE '^   ★收尾'
echo
( set +u; export PYTHONNOUSERSITE=1; source /opt/ros/noetic/setup.bash >/dev/null 2>&1; source ws/devel/setup.bash >/dev/null 2>&1
  bash scripts/verify/v_citation_check.sh $(grep -vE '^ *(#|$)' scripts/verify/claim_face.txt) > "$D/11_VCC.txt" 2>&1; echo "VCC rc=$? ｜ $(grep -E '单点引用=' "$D/11_VCC.txt" | tail -1)"
  python3 scripts/verify/v_claim_field_audit.py > "$D/12_规矩九.txt" 2>&1; echo "规矩⑨ rc=$? ｜ $(grep -E '含结论强度词行=' "$D/12_规矩九.txt" | tail -1) ｜ 须人读=$(grep -oE '须人读=[0-9]+' "$D/12_规矩九.txt" | tail -1)"
  bash scripts/verify/v_ledger_citation_baseline.sh --check note/进度/SCAN集成/证据/log/raw/r308_claim_face_swap/baseline_note70w40_r308.tsv $(grep -vE '^ *(#|$)' scripts/verify/claim_face.txt) > "$D/13_LCB.txt" 2>&1; echo "LCB rc=$? ｜ 同值项数=$(grep -c '同值' "$D/13_LCB.txt")"
  bash scripts/verify/v_ledger_batch_consistency.sh > "$D/14_账构.txt" 2>&1; echo "账构 rc=$? ｜ $(grep -oE 'rc=[0-9]+' "$D/14_账构.txt" | tail -1)"
  ws/src/95_simulation/lite3_gazebo_sim/verify/v_w3_terrain_design_contract.sh > "$D/15_契约件.txt" 2>&1; echo "契约件 rc=$? ｜ $(grep -oE 'PASS=[0-9]+ FAIL=[0-9]+ ABSENT=[0-9]+' "$D/15_契约件.txt" | tail -1)"
  bash scripts/verify/v_evidence_reachability.sh > "$D/16_VAR.txt" 2>&1; echo "VAR rc=$? ｜ $(grep -oE 'PASS=[0-9]+ FAIL=[0-9]+' "$D/16_VAR.txt" | tail -1)" )
echo
echo "## 改后反扫（证零新号形）＋清盘三条核对式＋仓态"
for f in note/04_配置改动diff.md "note/进度/00_总览.md" note/进度/00_裁定请求单_2026-10-09.md "note/计划书v6/23_v6任务队列.md" "note/进度/SCAN集成/设计/aq_终稿施工侧自审单_第一版.md"; do
  out=$(bash scripts/verify/v_citation_refscan.sh "$f" 2>&1); r=$?; printf '%s ｜ rc=%s ｜ %s\n' "$(basename "$f")" "$r" "$(printf '%s\n' "$out" | grep -oE '有 [0-9]+ 处按行号的引用|零行号引用' | head -1)"
done
K1=$(git ls-files | grep -cE '\.pyc$|__pycache__'); K2=$(cat scripts/verify/syn_face.txt scripts/verify/pyst_face.txt scripts/verify/parity_face.txt scripts/verify/claim_face.txt | grep -vE '^ *(#|$)' | grep -cE '\.pyc|__pycache__'); K3=$(find ws/src/30_3rd -name '*.pyc' | grep -c .); PC=$(find . -path ./.git -prune -o -type d -name __pycache__ -print | grep -c .)
echo "K1=$K1 K2=$K2 K3=$K3 产物目录在场=$PC"
if [ "$K1" = 0 ] && [ "$K2" = 0 ] && [ "$K3" = 0 ]; then find . -path ./.git -prune -o -type d -name __pycache__ -print0 | xargs -0 -r rm -rf; echo "三条全过 ⇒ 已清，清后=$(find . -path ./.git -prune -o -type d -name __pycache__ -print | grep -c .)"; else echo "★未全过 ⇒ 不清盘"; fi
echo "两面并打＝syn $(grep -vE '^ *(#|$)' scripts/verify/syn_face.txt | grep -c .)/$(git -c core.quotepath=false ls-files 'scripts/*.sh' 'note/进度/**/scripts/*.sh' 'ws/src/**/verify/*.sh' 'tooling/*.sh' | grep -c .)｜pyst $(grep -vE '^ *(#|$)' scripts/verify/pyst_face.txt | grep -c .)/$({ find ws/src -path '*/lite3_*' -name '*.py'; find scripts -name '*.py'; find note/进度 -path '*/scripts/*.py' -name '*.py'; } | grep -v __pycache__ | sort -u | grep -c .)"
echo "串成员=$(grep -cvE '^ *(#|$)' scripts/verify/gate_chain.txt) 窗回执=$(grep -vE '^ *(#|$)' scripts/verify/_window_receipt.tsv | grep -c .) claim=$(grep -vE '^ *(#|$)' scripts/verify/claim_face.txt | grep -c .) gate_chain diff=$(git diff --numstat scripts/verify/gate_chain.txt | grep -c .)"
V53=note/计划书v5/21_ros1-slam隔离部署计划书-v5.3.md
echo "v5.3=$(wc -l < $V53) 行 sha8=$(sha256sum $V53 | cut -c1-8) diff=$(git diff --numstat $V53 | grep -c .) 拦人句命中=$(grep -c '本件不是终稿' $V53)"
echo "tooling 顶层条目=$(grep -vc '^#' tooling/_sha256_manifest.txt) 自验 rc=$(cd tooling && sha256sum -c _sha256_manifest.txt >/dev/null 2>&1; echo $?) 豁免面复验=$(sed -n '8p' tooling/_forbid_str_exempt.tsv | sed 's/^#   //' > /tmp/r336chk.sh; bash /tmp/r336chk.sh; echo rc=$?)"
echo "计数现跑＝04 $(wc -l < note/04_配置改动diff.md)｜总览 $(wc -l < note/进度/00_总览.md)｜请求单 $(wc -l < note/进度/00_裁定请求单_2026-10-09.md)｜aq $(wc -l < note/进度/SCAN集成/设计/aq_终稿施工侧自审单_第一版.md)｜23号 $(wc -l < note/计划书v6/23_v6任务队列.md)｜05 $(wc -l < note/05_复验作业单.md)"
git diff --numstat | grep -v '^0	0$' | cat
echo "load1=$(awk '{print $1}' /proc/loadavg) 止 $(date '+%F %T')"
