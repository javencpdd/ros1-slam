#!/bin/bash
# R313 起手：改件前钩子（⑤）＋当期基线（①②③④）只读留底
# 用法：bash <本件>   （在工作区根目录跑）
set +u
cd /home/ros1-slam || exit 9
D=note/进度/SCAN集成/证据/log/raw/r313_a54_a55_rulings_arrival
OUT="$D/00_prechange_and_baseline.txt"
FILES="note/进度/00_裁定请求单_2026-10-09.md
note/进度/SCAN集成/设计/z_决策单_一页可签.md
note/04_配置改动diff.md
note/00_README.md
note/进度/00_总览.md"
{
  echo "=== R313 起手时刻 ==="
  date '+%Y-%m-%d %H:%M:%S'
  echo "load1=$(cut -d' ' -f1 /proc/loadavg)"
  echo
  echo "=== 本批将被改的五件：当前行数（wc -l 现跑）==="
  for f in $FILES; do
    printf '%s  行数=%s\n' "$f" "$(wc -l < "$f")"
  done
  echo
  echo "=== ⑤ refscan（改件前逐件单跑，rc 直取不经管道）==="
  for f in $FILES; do
    bash scripts/verify/v_citation_refscan.sh "$f" > "$D/_tmp_refscan_one.txt" 2>&1
    rc=$?
    sp=$(grep -E '合计行号引用' "$D/_tmp_refscan_one.txt" | tr -d '\n')
    printf '%s :: %s :: REFSCAN_RC=%s\n' "$f" "$sp" "$rc"
  done
  echo
  echo "=== ① 账构 v_ledger_batch_consistency.sh ==="
  bash scripts/verify/v_ledger_batch_consistency.sh > "$D/_tmp_zhanggou.txt" 2>&1
  echo "rc=$?"
  tail -12 "$D/_tmp_zhanggou.txt"
  echo
  echo "=== ② VCC 按 claim_face 70 面 ==="
  A=$(grep -vE '^ *($|#)' scripts/verify/claim_face.txt | sed 's#^#/home/ros1-slam/#')
  N=$(printf '%s\n' $A | grep -c .)
  echo "面条目数=$N"
  bash scripts/verify/v_citation_check.sh $A > "$D/_tmp_vcc.txt" 2>&1
  echo "rc=$?"
  grep -E '文档=|单点引用|OK=|RED|AMBER|勘误句|非单点|豁免|命中|未命中' "$D/_tmp_vcc.txt" | tail -14
  echo
  echo "=== ③ LCB --check 对 r308 当期基线 ==="
  bash scripts/verify/v_ledger_citation_baseline.sh --check \
    note/进度/SCAN集成/证据/log/raw/r308_claim_face_swap/baseline_note70w40_r308.tsv $A \
    > "$D/_tmp_lcb.txt" 2>&1
  echo "rc=$?"
  tail -14 "$D/_tmp_lcb.txt"
  echo
  echo "=== ④ 规矩⑨报告器（本批将被改五件）==="
  python3 scripts/verify/v_claim_field_audit.py $FILES > "$D/_tmp_audit.txt" 2>&1
  echo "rc=$?"
  tail -8 "$D/_tmp_audit.txt"
  echo
  echo "=== 其他在册计数（现跑）==="
  echo "gate_chain 成员=$(grep -cvE '^ *(#|$)' scripts/verify/gate_chain.txt)"
  echo "syn_face 条目=$(grep -cvE '^ *(#|$)' scripts/verify/syn_face.txt)"
  echo "pyst_face 条目=$(grep -cvE '^ *(#|$)' scripts/verify/pyst_face.txt)"
  echo "parity_face 条目=$(grep -cvE '^ *(#|$)' scripts/verify/parity_face.txt)"
  echo "窗回执非注释行=$(grep -cvE '^ *(#|$)' scripts/verify/_window_receipt.tsv)"
  echo "30_3rd 文件数=$(find ws/src/30_3rd -type f | wc -l)"
  echo "请求单行数=$(wc -l < note/进度/00_裁定请求单_2026-10-09.md)"
  echo "R 补最大号=$(grep -c '^## R 补' note/进度/00_裁定请求单_2026-10-09.md)"
  echo "A 号最大=$(grep -ohE 'A5[0-9]' note/进度/00_裁定请求单_2026-10-09.md | sort -u | tail -3 | tr '\n' ' ')"
  echo "05 追加最大号=$(grep -oE '^### 追加[一二三四五六七八九十零〇]+' note/05_复验作业单.md | tail -2 | tr '\n' ' ')"
  echo "HEAD=$(git rev-parse --short HEAD)"
  echo "git status 非跟踪计数=$(git status --porcelain | grep -c '^??')"
} > "$OUT" 2>&1
echo "WROTE $OUT"
wc -l "$OUT"
