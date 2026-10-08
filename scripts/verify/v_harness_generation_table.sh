#!/usr/bin/env bash
# v_harness_generation_table.sh —— 判据载体代际表的生成器与漂移校验器（小明 2026-10-08）
# 为何要件不要纯文：07 档一旦手抄就会与 git 脱钩（她多次因引用者跨代误读而开单，而「表本身对不对」此前无人能判）。
# 用法：
#   bash v_harness_generation_table.sh            # 打印当前阶梯（可重定向覆写 07 的表体）
#   bash v_harness_generation_table.sh --check    # 拿 07 档里在册的 SM sha12 与 git 现算值对账，不符即 FAIL
# 性质：零 ROS 节点、零活体、零 ws 写；只读 git 对象与 note/07 档。
set -u
ROOT=/home/ros1-slam
SM=ws/src/95_simulation/lite3_gazebo_sim/verify/m2_smoke.sh
LIB=ws/src/95_simulation/lite3_gazebo_sim/verify/m2_exitclean_lib.sh
XAC=ws/src/95_simulation/lite3_gazebo_sim/urdf/lite3_planar.urdf.xacro
PLG=ws/src/95_simulation/lite3_gazebo_sim/src/lite3_planar_drive.cpp
DOC=$ROOT/note/07_判据载体代际对照表.md
cd "$ROOT" || exit 5

sha_of() { git show "$1:$2" 2>/dev/null | sha256sum | cut -c1-12; }

if [ "${1:-}" = "--check" ]; then
  bad=0; n=0
  while IFS= read -r line; do
    c=$(printf '%s' "$line" | awk -F'|' '{gsub(/ /,"",$3); print $3}')
    s=$(printf '%s' "$line" | awk -F'|' '{gsub(/ /,"",$4); print $4}')
    lb=$(printf '%s' "$line" | awk -F'|' '{gsub(/^ +| +$/,"",$6); print $6}')
    [ -z "$c" ] && continue
    case "$s" in *[0-9a-f]*) ;; *) continue ;; esac
    n=$((n+1))
    now=$(sha_of "$c" "$SM")
    if [ "$now" != "$s" ]; then echo "[FAIL] $c 档内 SM=$s 现算=$now"; bad=$((bad+1)); fi
    # R47：LIB 列同核（S07 自己要求双 sha，却只验一件＝名实不符）
    if git cat-file -e "$c:$LIB" 2>/dev/null; then
      nlib=$(sha_of "$c" "$LIB")
      [ "$nlib" = "$lb" ] || { echo "[FAIL] $c 档内 LIB=$lb 现算=$nlib"; bad=$((bad+1)); }
    else
      case "$lb" in 无*) : ;; *) echo "[FAIL] $c 该代无库件，档内 LIB 列应为「无…」实为 $lb"; bad=$((bad+1));; esac
    fi
  done <<< "$(grep -E '^\| H[0-9]+ \|' "$DOC")"
  # 防重与穷尽（本轮实测教训：我把表体重复插入成 27 行，逐行核对却仍 PASS——
  #   因为每一行单独都与 git 相符，重复与漏行都不在「逐行」的判据面内。现补两断言：）
  _rows=$(grep -cE '^\| H[0-9]+ \|' "$DOC")
  _uniq=$(grep -oE '^\| H[0-9]+ ' "$DOC" | sort -u | wc -l)
  _gt=$(git log --format=%h -- "$SM" "$LIB" | wc -l)
  [ "$_rows" = "$_uniq" ] || { echo "[FAIL] 表体有重复行：在册 $_rows 行、唯一 H# $_uniq 个"; bad=$((bad+1)); }
  [ "$_uniq" = "$_gt" ] || { echo "[FAIL] 表体不穷尽：唯一 H# $_uniq 个 ≠ git 载体提交数 $_gt"; bad=$((bad+1)); }
  echo "  分母面：在册行=$n 唯一H#=$_uniq git载体提交数=$_gt（重复与漏行同判红）"
  echo "== 07 档对账：在册 $n 行（SM 与 LIB 双列同核，R47），不符 $bad 行 =="
  [ "$bad" = "0" ] && [ "$n" -ge 1 ] || exit 5
  echo "GENERATION TABLE CHECK: PASS（$n 行逐行与 git 现算同值 ⇒ 表未与仓脱钩）"
  exit 0
fi

i=0
printf '| # | commit | SM sha12 | 行数 | 库件 | XAC | PLG |\n|---|---|---|---|---|---|---|\n'
for c in $(git log --format=%h -- "$SM" "$LIB" | tac); do   # R47：轴＝SM 与 LIB 的并集（判据搬库后单看 SM 会漏代）
  i=$((i+1))
  if git cat-file -e "$c:$LIB" 2>/dev/null; then lib="$(sha_of "$c" "$LIB")"; else lib="无（判据在 SM 内）"; fi
  lines=$(git show "$c:$SM" | wc -l)
  printf '| H%d | %s | %s | %s | %s | %s | %s |\n' "$i" "$c" "$(sha_of "$c" "$SM")" "$lines" \
         "$lib" "$(sha_of "$c" "$XAC" | cut -c1-8)" "$(sha_of "$c" "$PLG" | cut -c1-8)"
done
echo "== 共 $i 版载体 =="
