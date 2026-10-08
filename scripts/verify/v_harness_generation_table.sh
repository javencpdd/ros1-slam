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
    [ -z "$c" ] && continue
    case "$s" in *[0-9a-f]*) ;; *) continue ;; esac
    n=$((n+1))
    now=$(sha_of "$c" "$SM")
    if [ "$now" != "$s" ]; then echo "[FAIL] $c 档内 SM=$s 现算=$now"; bad=$((bad+1)); fi
  done <<< "$(grep -E '^\| H[0-9]+ \|' "$DOC")"
  echo "== 07 档对账：在册 $n 行，不符 $bad 行 =="
  [ "$bad" = "0" ] && [ "$n" -ge 1 ] || exit 5
  echo "GENERATION TABLE CHECK: PASS（$n 行逐行与 git 现算同值 ⇒ 表未与仓脱钩）"
  exit 0
fi

i=0
printf '| # | commit | SM sha12 | 行数 | 库件 | XAC | PLG |\n|---|---|---|---|---|---|---|\n'
for c in $(git log --format=%h -- "$SM" | tac); do
  i=$((i+1))
  if git cat-file -e "$c:$LIB" 2>/dev/null; then lib="$(sha_of "$c" "$LIB")"; else lib="无（判据在 SM 内）"; fi
  lines=$(git show "$c:$SM" | wc -l)
  printf '| H%d | %s | %s | %s | %s | %s | %s |\n' "$i" "$c" "$(sha_of "$c" "$SM")" "$lines" \
         "$lib" "$(sha_of "$c" "$XAC" | cut -c1-8)" "$(sha_of "$c" "$PLG" | cut -c1-8)"
done
echo "== 共 $i 版载体 =="
