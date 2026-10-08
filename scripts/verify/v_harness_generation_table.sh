#!/usr/bin/env bash
# v_harness_generation_table.sh —— 判据载体代际表的生成器与漂移校验器（小明 2026-10-08）
# 为何要件不要纯文：07 档一旦手抄就会与 git 脱钩（她多次因引用者跨代误读而开单，而「表本身对不对」此前无人能判）。
# 用法：
#   bash v_harness_generation_table.sh            # 打印当前阶梯（可重定向覆写 07 的表体）
#   bash v_harness_generation_table.sh --check    # 拿 07 档里在册的 SM sha12 与 git 现算值对账，不符即 FAIL
# 性质：零 ROS 节点、零活体、零 ws 写；只读 git 对象与 note/07 档。
set -u
ROOT=/home/ros1-slam
SELF=$(readlink -f "$0")   # --negctl 要用本体递归跑三次（临时表副本作被试）
SM=ws/src/95_simulation/lite3_gazebo_sim/verify/m2_smoke.sh
LIB=ws/src/95_simulation/lite3_gazebo_sim/verify/m2_exitclean_lib.sh
XAC=ws/src/95_simulation/lite3_gazebo_sim/urdf/lite3_planar.urdf.xacro
PLG=ws/src/95_simulation/lite3_gazebo_sim/src/lite3_planar_drive.cpp
DOC=${GTC_DOC:-$ROOT/note/07_判据载体代际对照表.md}   # 只有 --negctl 的自证会覆写它（缺省恒为在册表）
cd "$ROOT" || exit 5

sha_of() { git show "$1:$2" 2>/dev/null | sha256sum | cut -c1-12; }

if [ "${1:-}" = "--negctl" ]; then
  # 05 C4a：新加的核对面必须**真的红一次**且可复跑。做法＝拿在册表的临时副本各改一位（不改原表、不写回），
  #   再让本体去核：R1 改末行 PLG 的一位十六进制（R55 新增的那一列）、R2 改末行 SM 的一位（旧有那一列，作阳性对照）。
  #   两条都必须红；同时 R0 用未改副本证明"红来自改动、不是来自临时件本身"。
  T=$(mktemp -d /tmp/gtc_neg.XXXXXX); trap 'rm -rf "$T"' EXIT
  L=$(grep -E '^\| H[0-9]+ \|' "$DOC" | tail -1)
  [ -n "$L" ] || { echo "取不到末行 ⇒ 前置不成立"; exit 9; }
  _rows0=$(grep -cE '^\| H[0-9]+ \|' "$DOC")
  for col in 4 8; do
    nm=$([ "$col" = 4 ] && echo SM || echo PLG)
    # 先取出该列的**值本身**（去空格），再把末行里这一处整值换掉：
    #   第一版我用 awk 对字段做 sub(/[0-9a-f]$/,...) —— 字段是「 值 」带空格的，末位是空格，改写静默不生效（对照空转）。
    val=$(printf '%s' "$L" | awk -F'|' -v c="$col" '{v=$c; gsub(/ /,"",v); print v}')
    case "$val" in *[0-9a-f]*) ;; *) echo "第 $nm 列取值不是 sha 前缀（值=$val）⇒ 前置不成立"; exit 9;; esac
    repl=$([ "${val: -1}" = "f" ] && echo 0 || echo f)
    mut="${val%?}$repl"
    new="${L/$val/$mut}"
    [ "$new" != "$L" ] || { echo "第 $nm 列改写未生效 ⇒ 对照空转，判 9"; exit 9; }
    awk -v old="$L" -v new="$new" '$0==old{print new; next}{print}' "$DOC" > "$T/doc_c$col.md"
    _rowsn=$(grep -cE '^\| H[0-9]+ \|' "$T/doc_c$col.md")
    [ "$_rowsn" = "$_rows0" ] || { echo "改动后表体行数=$_rowsn（原 $_rows0）⇒ 红可能来自掉行而非列比对，判 9"; exit 9; }
    GTC_DOC="$T/doc_c$col.md" bash "$SELF" --check > "$T/out_c$col.txt" 2>&1; r=$?
    echo "  R$([ "$col" = 4 ] && echo 2 || echo 1) 改末行 $nm（$val→$mut）⇒ 本体 rc=$r（期望 5），红行：$(grep -m1 'FAIL' "$T/out_c$col.txt")"
    if [ "$r" != 5 ] || ! grep -q "$nm" "$T/out_c$col.txt"; then echo "NEGCTL VERDICT: $nm 列改了不红 ⇒ 该核对面是空气，rc=5"; exit 5; fi
  done
  cp "$DOC" "$T/doc_same.md"; GTC_DOC="$T/doc_same.md" bash "$SELF" --check >/dev/null 2>&1; r=$?
  echo "  R0 未改副本 ⇒ 本体 rc=$r（期望 0，证明红来自改动而非临时件本身）"
  [ "$r" = 0 ] || { echo "NEGCTL VERDICT: 未改也红 ⇒ 临时件路径本身触发假红，rc=5"; exit 5; }
  echo "NEGCTL VERDICT: SM 与 PLG 两列都能真红 rc=0"; exit 0
fi

if [ "${1:-}" = "--check" ]; then
  bad=0; n=0
  while IFS= read -r line; do
    c=$(printf '%s' "$line" | awk -F'|' '{gsub(/ /,"",$3); print $3}')
    s=$(printf '%s' "$line" | awk -F'|' '{gsub(/ /,"",$4); print $4}')
    lb=$(printf '%s' "$line" | awk -F'|' '{gsub(/^ +| +$/,"",$6); print $6}')
    xa=$(printf '%s' "$line" | awk -F'|' '{gsub(/ /,"",$7); print $7}')
    pl=$(printf '%s' "$line" | awk -F'|' '{gsub(/ /,"",$8); print $8}')
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
    # R55：XAC／PLG 两列同核（这条是批3 的前置作业——批3 改的正是 PLG 源码；表印 7 列却只验 2 列＝又一名实不符）
    for pair in "XAC:$XAC:$xa" "PLG:$PLG:$pl"; do
      nm=${pair%%:*}; rest=${pair#*:}; path=${rest%%:*}; want=${rest#*:}
      case "$want" in 无*) if git cat-file -e "$c:$path" 2>/dev/null; then echo "[FAIL] $c 档内 $nm 列为「无」但 git 里有该件"; bad=$((bad+1)); fi; continue;; esac
      case "$want" in *[0-9a-f]*) ;; *) echo "[FAIL] $c 档内 $nm 列不是 sha 前缀（值=「$want」）⇒ 列形制变过，先更件再更表"; bad=$((bad+1)); continue;; esac
      if git cat-file -e "$c:$path" 2>/dev/null; then
        got=$(git show "$c:$path" 2>/dev/null | sha256sum | cut -c1-8)
        [ "$got" = "$want" ] || { echo "[FAIL] $c 档内 $nm=$want 现算=$got"; bad=$((bad+1)); }
      else
        echo "[FAIL] $c 该代无 $path（$nm 列在册值 $want）⇒ 表里有 git 里没的件"; bad=$((bad+1))
      fi
    done
  done <<< "$(grep -E '^\| H[0-9]+ \|' "$DOC")"
  # R55 之二：HEAD 与被测件面的**前向漂移**必须可指认（末行的 XAC/PLG 是"该行 commit 时刻"的值；
  #   若 .cpp/.xacro 在末行之后又单独提交过，代际表里没有地方记它 ⇒ 这里打 WARN 并报数，
  #   是否把它升成 FAIL（＝"改被测件面必须伴随一次载体提交"）属**判据变更**，按 05 A3 先挂账待小美/主理人追认，我不擅自升档。）
  _last=$(grep -E '^\| H[0-9]+ \|' "$DOC" | tail -1)
  _lpl=$(printf '%s' "$_last" | awk -F'|' '{gsub(/ /,"",$8); print $8}')
  _lxa=$(printf '%s' "$_last" | awk -F'|' '{gsub(/ /,"",$7); print $7}')
  _hpl=$(git show "HEAD:$PLG" 2>/dev/null | sha256sum | cut -c1-8)
  _hxa=$(git show "HEAD:$XAC" 2>/dev/null | sha256sum | cut -c1-8)
  _drift=0
  [ "$_lpl" != "$_hpl" ] && { echo "  [WARN] 被测件面前向漂移：末行 PLG=$_lpl 而 HEAD PLG=$_hpl（$PLG 在末行之后又单独提交过）"; _drift=$((_drift+1)); }
  [ "$_lxa" != "$_hxa" ] && { echo "  [WARN] 被测件面前向漂移：末行 XAC=$_lxa 而 HEAD XAC=$_hxa"; _drift=$((_drift+1)); }
  [ "$_drift" = 0 ] && echo "  被测件面：末行 XAC/PLG 与 HEAD 同值（无漂移，升 FAIL 与否待裁）"
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
