#!/bin/bash
# R285 逐 hunk 未漂移自证（05 追加三十一 第 4 条下半句的可执行形）
# 甲种（被按行号引用，refscan＞0）＝只许 尾部纯追加 或 单行原位替换；乙种（refscan＝0 且有读数留底）＝许中间插入
set -u
cd /home/ros1-slam
JIA="note/04_配置改动diff.md note/00_README.md note/进度/00_总览.md note/进度/SCAN集成/进度文档.md"
YI="note/05_复验作业单.md scripts/verify/run_gate_chain.sh"
total=0; bad=0
echo "== 甲种件（refscan 读数见 00_refscan_pre_change.txt：04＝18／README＝6／00_总览＝4／进度文档＝50）=="
for f in $JIA; do
  H=$(git show HEAD:"$f" | wc -l); C=$(wc -l < "$f")
  echo "-- $f（HEAD=$H 现=$C）"
  while IFS= read -r hunk; do
    total=$((total+1))
    if printf '%s' "$hunk" | grep -qE "^@@ -$H,0 \+$((H+1))"; then
      echo "   [甲·合规＝尾部纯追加] $hunk"
    elif printf '%s' "$hunk" | grep -qE '^@@ -[0-9]+ \+[0-9]+ @@$'; then
      echo "   [甲·合规＝单行原位替换] $hunk"
    else
      echo "   ★[甲·须停手] $hunk"; bad=$((bad+1))
    fi
  done < <(git diff -U0 HEAD -- "$f" | grep '^@@')
done
echo "== 乙种件（refscan＝0 留底：01_refscan_05_and_runner.txt）=="
for f in $YI; do
  H=$(git show HEAD:"$f" | wc -l); C=$(wc -l < "$f")
  echo "-- $f（HEAD=$H 现=$C；乙种依据＝该件合计行号引用=0）"
  while IFS= read -r hunk; do
    total=$((total+1))
    if printf '%s' "$hunk" | grep -qE '^@@ -[0-9]+,0 \+[0-9]+'; then
      echo "   [乙·合规＝纯插入（有 refscan=0 依据）] $hunk"
    elif printf '%s' "$hunk" | grep -qE '^@@ -[0-9]+ \+[0-9]+ @@$'; then
      echo "   [乙·合规＝单行原位] $hunk"
    else
      echo "   ★[乙·须停手：疑似改写既有行内容] $hunk"; bad=$((bad+1))
    fi
  done < <(git diff -U0 HEAD -- "$f" | grep '^@@')
done
echo "=================================================="
echo "hunk 总数=$total｜须停手=$bad"
if [ "$bad" = 0 ]; then echo "HUNKCHECK VERDICT: rc=0 本次改动未移动任何被按行号引用的目标行"; exit 0; fi
echo "HUNKCHECK VERDICT: rc=5 有 $bad 个 hunk 不属允许形，停手逐条核"; exit 5
