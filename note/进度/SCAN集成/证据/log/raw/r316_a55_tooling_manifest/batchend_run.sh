#!/bin/bash
# R313 批末：固定跑法 ①–⑥ ＋ 八件串 ＋ 收尾跑序（只报不判，不改任何件）
set +u
cd /home/ros1-slam || exit 9
export PYTHONNOUSERSITE=1
D=note/进度/SCAN集成/证据/log/raw/r316_a55_tooling_manifest
OUT="$D/20_batch_end_runs.txt"
IDS=$(whoami)
TS=$(date '+%Y-%m-%d %H:%M:%S')
A=$(grep -vE '^ *($|#)' scripts/verify/claim_face.txt | sed 's#^#/home/ros1-slam/#')
FACES="note/进度/00_裁定请求单_2026-10-09.md note/04_配置改动diff.md note/00_README.md note/进度/00_总览.md"
{
  echo "# r316 留底：批末固定跑（A55＝乙 tooling 顶层清单）｜时刻=$TS｜身份=$IDS"
  echo "# ★ISO 原样自证串不入本件（含禁列路径字面量）；写入不经 cut -c"
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
  echo "  RED 行数实数=$(grep -c '^\[RED\]' "$D/_t2.txt")"
  echo "## ③ LCB 对 r308 基线"
  bash scripts/verify/v_ledger_citation_baseline.sh --check \
    note/进度/SCAN集成/证据/log/raw/r308_claim_face_swap/baseline_note70w40_r308.tsv $A > "$D/_t3.txt" 2>&1
  echo "LCB rc=$?"
  grep -E 'VERDICT|同值项数|不等' "$D/_t3.txt" | head -5
  echo "  同值项数=$(grep -c '同值' "$D/_t3.txt")"
  echo "## ④ 规矩⑨（70 面默认）"
  python3 scripts/verify/v_claim_field_audit.py > "$D/_t4.txt" 2>&1
  echo "audit rc=$?"
  grep -E '面＝|\[计数\]' "$D/_t4.txt"
  echo "## ④′ 规矩⑨（只跑本批被改五件）"
  python3 scripts/verify/v_claim_field_audit.py $FACES > "$D/_t5.txt" 2>&1
  echo "audit5 rc=$?"
  grep -E '\[计数\]' "$D/_t5.txt"
  echo "## ⑤ refscan 改后逐件"
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
  echo "## ★A55 本批专项校验（顶层清单三式）"
  echo "顶层清单行数=$(wc -l < tooling/_sha256_manifest.txt) 条目=$(grep -cE '^[0-9a-f]{64}  ' tooling/_sha256_manifest.txt)"
  ( cd tooling && sha256sum -c _sha256_manifest.txt > /tmp/tm.txt 2>&1 ); echo "顶层清单校验 rc=$? 非OK行=$(grep -vc ': OK' /tmp/tm.txt)"; rm -f /tmp/tm.txt
  echo "顶层清单存在=$( [ -f tooling/_sha256_manifest.txt ] && echo 是 || echo 否 ) mode=$(ls -l tooling/_sha256_manifest.txt | awk '{print $1}')"
  echo "unit 可读性复跑（只读）=$(for u in /etc/systemd/system/lite3-*.service /etc/systemd/system/transfer_ros2.service; do [ -r "$u" ] && echo -n 1 || echo -n 0; done | tr -cd 1 | wc -c)/7"
  echo "## 面与成员（现跑）"
  echo "gate_chain=$(grep -cvE '^ *(#|$)' scripts/verify/gate_chain.txt) syn=$(grep -cvE '^ *(#|$)' scripts/verify/syn_face.txt) pyst=$(grep -cvE '^ *(#|$)' scripts/verify/pyst_face.txt) parity=$(grep -cvE '^ *(#|$)' scripts/verify/parity_face.txt) claim=$(grep -cvE '^ *(#|$)' scripts/verify/claim_face.txt)"
  echo "窗回执非注释=$(grep -cvE '^ *(#|$)' scripts/verify/_window_receipt.tsv)"
  echo "load1=$(cut -d' ' -f1 /proc/loadavg)"
} > "$OUT" 2>&1
rm -f "$D"/_t[1-8].txt
echo "WROTE $OUT"; wc -l "$OUT"
