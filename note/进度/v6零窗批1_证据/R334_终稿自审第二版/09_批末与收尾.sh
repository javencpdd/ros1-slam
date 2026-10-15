set -u
cd /home/ros1-slam
D=note/进度/v6零窗批1_证据/R334_终稿自审第二版
echo "# R334 批末与收尾｜起 $(date '+%F %T')｜身份 $(id -un)｜★零窗批"
echo "## 一、本批全部落盘后的计数现跑（★自指数只在写完后取、正文不写）"
for f in note/04_配置改动diff.md note/进度/00_总览.md note/00_README.md note/03_外部文件来源清单.md "note/进度/SCAN集成/设计/aq_终稿施工侧自审单_第一版.md" "note/计划书v6/23_v6任务队列.md" note/05_复验作业单.md note/进度/00_裁定请求单_2026-10-09.md "note/计划书v5/21_ros1-slam隔离部署计划书-v5.3.md"; do
  printf '%-72s 行数=%s\n' "$f" "$(wc -l < "$f")"
done
echo "-- numstat（已跟踪件）--"; git diff --numstat | cat
echo "-- 档头两处形状现跑（★本批新兜法首用）--"
echo "L2 加粗标签枚数=$(sed -n '2p' note/进度/00_总览.md | grep -o '\*\*更新时间：' | grep -c .)（期望 1）"
echo "L2 四连星号处数=$(sed -n '2p' note/进度/00_总览.md | grep -o '\*\*\*\*' | grep -c .)（在册历史值＝R332 版 2 处／R331 版 1 处＝两式混用、非缺陷）"
echo "v5.3 sha 前 8=$(sha256sum "note/计划书v5/21_ros1-slam隔离部署计划书-v5.3.md" | cut -c1-8) 行数=$(wc -l < "note/计划书v5/21_ros1-slam隔离部署计划书-v5.3.md") git diff 该件＝$(git diff --numstat "note/计划书v5/21_ros1-slam隔离部署计划书-v5.3.md" | grep -c .)（空＝一字未动）"
echo
echo "## 二、批末十件串（在册运行器、只报不判）"
bash scripts/verify/run_gate_chain.sh 2>&1 | grep -vE '^   ★收尾'
echo
echo "## 三、逐件明细（在册调用式：VCC 显式喂 claim_face、规矩⑨＝python3、LCB＝--check＋r308 基线）"
( set +u; export PYTHONNOUSERSITE=1; source /opt/ros/noetic/setup.bash >/dev/null 2>&1; source ws/devel/setup.bash >/dev/null 2>&1
  bash scripts/verify/v_citation_check.sh $(grep -vE '^ *(#|$)' scripts/verify/claim_face.txt) > "$D/10_VCC明细.txt" 2>&1; echo "VCC rc=$? ｜ $(grep -E '单点引用=' "$D/10_VCC明细.txt" | tail -1)"
  echo "VCC [RED] 行实数=$(grep -c '\[RED\]' "$D/10_VCC明细.txt")"
  python3 scripts/verify/v_claim_field_audit.py > "$D/11_规矩九明细.txt" 2>&1; echo "规矩⑨ rc=$? ｜ $(grep -E '含结论强度词行=' "$D/11_规矩九明细.txt" | tail -1)"
  bash scripts/verify/v_ledger_citation_baseline.sh --check note/进度/SCAN集成/证据/log/raw/r308_claim_face_swap/baseline_note70w40_r308.tsv $(grep -vE '^ *(#|$)' scripts/verify/claim_face.txt) > "$D/12_LCB.txt" 2>&1; echo "LCB rc=$? ｜ 同值项数=$(grep -c '同值' "$D/12_LCB.txt")"
  bash scripts/verify/v_evidence_reachability.sh > "$D/13_VAR.txt" 2>&1; echo "VAR rc=$? ｜ $(grep -oE 'PASS=[0-9]+ FAIL=[0-9]+' "$D/13_VAR.txt" | tail -1)"
  ws/src/95_simulation/lite3_gazebo_sim/verify/v_w3_terrain_design_contract.sh > "$D/14_契约件.txt" 2>&1; echo "契约件 rc=$? ｜ $(grep -oE 'PASS=[0-9]+ FAIL=[0-9]+ ABSENT=[0-9]+' "$D/14_契约件.txt" | tail -1)"
)
echo
echo "## 四、清盘三条核对式（★正确式先过、后删）与仓态"
echo "K1=$(git ls-files | grep -cE '\.pyc$|__pycache__') K2=$(cat scripts/verify/syn_face.txt scripts/verify/pyst_face.txt scripts/verify/parity_face.txt scripts/verify/claim_face.txt | grep -vE '^ *(#|$)' | grep -cE '\.pyc|__pycache__') K3=$(find ws/src/30_3rd -name '*.pyc' | grep -c .) 产物目录在场=$(find . -path ./.git -prune -o -type d -name __pycache__ -print | grep -c .)"
if [ "$(git ls-files | grep -cE '\.pyc$|__pycache__')" = 0 ] && [ "$(cat scripts/verify/syn_face.txt scripts/verify/pyst_face.txt scripts/verify/parity_face.txt scripts/verify/claim_face.txt | grep -vE '^ *(#|$)' | grep -cE '\.pyc|__pycache__')" = 0 ] && [ "$(find ws/src/30_3rd -name '*.pyc' | grep -c .)" = 0 ]; then
  find . -path ./.git -prune -o -type d -name __pycache__ -print0 | xargs -0 -r rm -rf
  echo "三条全过 ⇒ 已清；清后产物目录=$(find . -path ./.git -prune -o -type d -name __pycache__ -print | grep -c .)"
else echo "★未全过 ⇒ 不清盘、公开留名"; fi
echo "两面并打＝syn 清单 $(grep -vE '^ *(#|$)' scripts/verify/syn_face.txt | grep -c .) ／ 派生式 $(git -c core.quotepath=false ls-files 'scripts/*.sh' 'note/进度/**/scripts/*.sh' 'ws/src/**/verify/*.sh' 'tooling/*.sh' | grep -c .)；pyst 清单 $(grep -vE '^ *(#|$)' scripts/verify/pyst_face.txt | grep -c .) ／ 派生式 $({ find ws/src -path '*/lite3_*' -name '*.py'; find scripts -name '*.py'; find note/进度 -path '*/scripts/*.py' -name '*.py'; } | grep -v __pycache__ | sort -u | grep -c .)"
echo "串成员=$(grep -cvE '^ *(#|$)' scripts/verify/gate_chain.txt) 窗回执=$(grep -vE '^ *(#|$)' scripts/verify/_window_receipt.tsv | grep -c .) ISO ws_src_files=$(grep -oE 'ws_src_files=[0-9]+' "$D/../.."/../../..//dev/null 2>/dev/null | head -1)"
echo "30_3rd 全树文件=$(find ws/src/30_3rd -type f | grep -c .) tooling 文件=$(find tooling -type f | grep -c .) tooling 顶层条目=$(grep -vc '^#' tooling/_sha256_manifest.txt) 顶层自验 rc=$(cd tooling && sha256sum -c _sha256_manifest.txt >/dev/null 2>&1; echo $?)"
echo "豁免面复验＝$(sed -n '8p' tooling/_forbid_str_exempt.tsv | sed 's/^#   //' > /tmp/r334chk.sh; bash /tmp/r334chk.sh; echo " rc=$?")"
echo "raw 目录=$(find note/进度/SCAN集成/证据/log/raw -maxdepth 1 -type d | tail -n +2 | grep -c .) 其中无清单=$(for x in note/进度/SCAN集成/证据/log/raw/*/; do [ -f "${x}_sha256_manifest.txt" ] || echo x; done | grep -c .)"
echo "非本批改动面（不提交不回退）＝$(git status --porcelain | grep -cE '^ M|^\?\?')"
echo "load1=$(awk '{print $1}' /proc/loadavg) 止 $(date '+%F %T')"
