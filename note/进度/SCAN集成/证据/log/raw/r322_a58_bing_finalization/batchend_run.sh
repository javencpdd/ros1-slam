#!/bin/bash
# R320 批末：固定跑法 ①–⑥ ＋ 八件串 ＋ 收尾跑序（只报不判，不改任何件）
set +u
cd /home/ros1-slam || exit 9
export PYTHONNOUSERSITE=1
D=note/进度/SCAN集成/证据/log/raw/r322_a58_bing_finalization
OUT="$D/20_batch_end_runs.txt"
IDS=$(whoami)
TS=$(date '+%Y-%m-%d %H:%M:%S')
A=$(grep -vE '^ *($|#)' scripts/verify/claim_face.txt | sed 's#^#/home/ros1-slam/#')
FACES="note/进度/00_裁定请求单_2026-10-09.md note/04_配置改动diff.md note/00_README.md note/进度/00_总览.md note/计划书v5/21_ros1-slam隔离部署计划书-v5.3.md note/进度/SCAN集成/设计/as_终稿自主深度审核单_施工侧第二版.md"
{
  echo "# r322 留底：批末固定跑（A58＝丙 范围化终稿落册批）｜时刻=$TS｜身份=$IDS"
  echo "## 八件串（只报不判运行器）"
  bash scripts/verify/run_gate_chain.sh > "$D/21_chain_raw.txt" 2>&1
  echo "串运行器 rc=$?"
  grep -E 'rc=' "$D/21_chain_raw.txt" | head -12
  echo "## ① 账构"
  bash scripts/verify/v_ledger_batch_consistency.sh > "$D/_t1.txt" 2>&1
  echo "账构 rc=$?"
  grep -E '^\[判定\]|VERDICT' "$D/_t1.txt"
  echo "## ② VCC（70 面）"
  bash scripts/verify/v_citation_check.sh $A > "$D/_t2.txt" 2>&1
  echo "VCC rc=$?"
  grep -E '^\[计数\]|^\[四元组\]|VERDICT' "$D/_t2.txt"
  echo "  RED 行数实数=$(grep -c '^\[RED\]' "$D/_t2.txt")  AMBER 打印行数=$(grep -c '^\[AMBER\]' "$D/_t2.txt")"
  echo "## ③ LCB 对 r308 基线"
  bash scripts/verify/v_ledger_citation_baseline.sh --check \
    note/进度/SCAN集成/证据/log/raw/r308_claim_face_swap/baseline_note70w40_r308.tsv $A > "$D/_t3.txt" 2>&1
  echo "LCB rc=$?"
  grep -E 'VERDICT|同值项数|不等' "$D/_t3.txt" | head -5
  echo "  同值项数=$(grep -c '同值' "$D/_t3.txt")"
  echo "## ④ 规矩⑨（70 面默认）"
  python3 scripts/verify/v_claim_field_audit.py > "$D/_t4.txt" 2>&1
  echo "audit rc=$?"
  grep -E '面＝|^\[计数\]' "$D/_t4.txt"
  echo "  [报] 行实数=$(grep -cE '^ *\[报\]' "$D/_t4.txt")"
  echo "## ④′ 规矩⑨（只跑本批被改五件）"
  python3 scripts/verify/v_claim_field_audit.py $FACES > "$D/_t5.txt" 2>&1
  echo "audit5 rc=$?"
  grep -E '^\[计数\]' "$D/_t5.txt"
  echo "  本批五件 [报] 行逐条："; grep -E '^ *\[报\]' "$D/_t5.txt" | sed 's#/home/ros1-slam/##g'
  echo "## ⑤ refscan 逐件"
  for f in $FACES; do
    bash scripts/verify/v_citation_refscan.sh "$f" > "$D/_t6.txt" 2>&1
    rc=$?
    n=$(grep -oE '合计行号引用=[0-9]+' "$D/_t6.txt" | tail -1)
    echo "  $(basename "$f") rc=$rc $n"
  done
  echo "## ⑥ 预检（本批零窗，跑只为留「能否开」的数，不开窗）"
  bash scripts/verify/v_window_preflight.sh > "$D/_t7.txt" 2>&1
  echo "preflight rc=$?"
  grep -E 'VERDICT' "$D/_t7.txt" | tail -2
  grep -E 'load1' "$D/_t7.txt" | tail -1
  echo "## 收尾跑序 K1/K2/K3 ＋ 清 __pycache__"
  K1=$(git ls-files | grep -c '__pycache__')
  K2=$(grep -rl '__pycache__' --include=_sha256_manifest.txt note/ | wc -l)
  K3=$(find ws/src/30_3rd -name '*.pyc' -o -name '__pycache__' | wc -l)
  echo "K1=$K1 K2=$K2 K3=$K3（三条须全 0 才许清盘）"
  echo "清前=$(find note ws/src scripts -name '__pycache__' -type d | wc -l)"
  if [ "$K1" -eq 0 ] && [ "$K2" -eq 0 ] && [ "$K3" -eq 0 ]; then
    find note ws/src scripts -name '__pycache__' -type d -exec rm -rf {} +
  else
    echo "★三条核对式未全过 ⇒ 不清盘，公开留名"
  fi
  echo "清后=$(find note ws/src scripts -name '__pycache__' -type d | wc -l)"
  echo "## 契约件（清盘后跑）"
  bash ws/src/95_simulation/lite3_gazebo_sim/verify/v_w3_terrain_design_contract.sh > "$D/_t8.txt" 2>&1
  echo "契约件 rc=$?"
  grep -E 'CONTRACT VERDICT' "$D/_t8.txt"
  echo "30_3rd=$(find ws/src/30_3rd -type f | wc -l)"
  echo "## ★R321 本批专项校验"
  V53="note/计划书v5/21_ros1-slam隔离部署计划书-v5.3.md"
  AR="note/进度/SCAN集成/设计/as_终稿自主深度审核单_施工侧第二版.md"
  V53="note/计划书v5/21_ros1-slam隔离部署计划书-v5.3.md"
  AR="note/进度/SCAN集成/设计/as_终稿自主深度审核单_施工侧第二版.md"
  echo "v5.3 行数=$(wc -l < "$V53") ^##=$(grep -c '^## ' "$V53") ^###=$(grep -c '^### ' "$V53") sha前8=$(sha256sum "$V53" | cut -c1-8) numstat=[$(git diff HEAD --numstat -- "$V53")]"
  echo "v5.3 件尾新节存在=$(grep -c 'T-R17 乙案落册记录（R321）' "$V53") 指针句存在=$(grep -c 'R17／#120 的处置自 R321 起' "$V53")"
  echo "ar 稿行数=$(wc -l < "$AR") sha前8=$(sha256sum "$AR" | cut -c1-8) mode=$(stat -c '%a' "$AR")"
  echo "请求单行数=$(wc -l < note/进度/00_裁定请求单_2026-10-09.md) R补节数=$(grep -c '^## R 补' note/进度/00_裁定请求单_2026-10-09.md)"
  echo "他人志文 sha 未动核对：fd=$(sha256sum "note/进度/SCAN集成/设计/fd_双写者与前馈源裁定.md" | cut -c1-8) fd_i=$(sha256sum "note/进度/SCAN集成/设计/fd_i_反向静态边裁定与处置.md" | cut -c1-8)"
  echo "总览条目行数=$(grep -c '^- 2026-' note/进度/00_总览.md) 总览件尾新节存在=$(grep -c '施工侧自判的阶段表与下一步指引（R321 追加' note/进度/00_总览.md)"
  echo "脏表应仍为 0＝未跟踪 scripts/verify 件数=$(git ls-files --others --exclude-standard scripts/verify | wc -l)"
  echo "## 面与成员（现跑）"
  echo "gate_chain=$(grep -cvE '^ *(#|$)' scripts/verify/gate_chain.txt) syn=$(grep -cvE '^ *(#|$)' scripts/verify/syn_face.txt) pyst=$(grep -cvE '^ *(#|$)' scripts/verify/pyst_face.txt) parity=$(grep -cvE '^ *(#|$)' scripts/verify/parity_face.txt) claim=$(grep -cvE '^ *(#|$)' scripts/verify/claim_face.txt)"
  echo "窗回执非注释=$(grep -cvE '^ *(#|$)' scripts/verify/_window_receipt.tsv)"
  echo "未跟踪 scripts/verify=$(git ls-files --others --exclude-standard scripts/verify | wc -l)"
  echo "load1=$(cut -d' ' -f1 /proc/loadavg)"
} > "$OUT" 2>&1
rm -f "$D"/_t[1-8].txt
echo "WROTE $OUT"; wc -l "$OUT"
