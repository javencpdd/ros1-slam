#!/usr/bin/env bash
# 01_让路自测.sh —— ★零窗自测★（不取新活体读数）＝把 r498 窗脚本里"重跑让路"那段取出来真触发一次
# 用法＝bash 01_让路自测.sh <原件 00_r498_window.sh 全路径> <scratch 根>
# 取法＝只执行原件第 31—41 行与第 84 行（★逐字节、不重写★），执行前由本件把 R/RUN/DUMP/CSV 指到 scratch；
#       ⇒ 被试代码＝原件那 12 行本身，被改的只是它作用的目录（★真窗一个字不动★）
# 证明两件事＝①旧件在位时★改名＋出声★真发生（防呆有牙）；②让路后新窗志从空起、^STATE= 不可能被上一遍叠成 8
set -u
SRC="${1:?用法: bash 01_ 件 <原件全路径> <scratch 根>}"
R="${2:?scratch 根缺失}"
EX="$R/_extract_sourced.sh"
mkdir -p "$R"
printf '# 被执行件＝下面两份 sed 输出的拼接（第 31—41 行 ‖ 第 84 行）\n' > "$EX"
sed -n '31,41p' "$SRC" >> "$EX"
sed -n '84p'   "$SRC" >> "$EX"

echo "== 〔一〕字节同原件自证（★被试代码不许是我重写过的★）=="
echo "  原件行数=$(wc -l < "$SRC") ‖ 被执行件行数=$(wc -l < "$EX")（＝1 行注＋11＋1）"
diff <(sed -n '31,41p' "$SRC") <(sed -n '2,12p' "$EX") > /dev/null && echo "  第 31—41 行 diff＝空 ⇒ 逐字节同" || echo "  ★第 31—41 行不同⇒被试的不是原件★"
diff <(sed -n '84p' "$SRC") <(sed -n '13p' "$EX") > /dev/null && echo "  第 84 行 diff＝空 ⇒ 逐字节同" || echo "  ★第 84 行不同⇒被试的不是原件★"
echo "  sha 原件第31—41行＝$(sed -n '31,41p' "$SRC" | sha256sum | cut -c1-16) ‖ 被执行件同段＝$(sed -n '2,12p' "$EX" | sha256sum | cut -c1-16)"

seed(){
  rm -rf "$R/seedmark" 2>/dev/null; mkdir -p "$R/seedmark"
  printf 'STATE=REF x\nSTATE=A x\nSTATE=Ap x\nSTATE=C x\n' > "$R/00_window_log.txt"
  echo pre   > "$R/01_preflight.txt"
  echo delta > "$R/03_delta.txt"
  echo cmp   > "$R/05_cmp.txt"
  echo du    > "$R/07_复读.txt"
  mkdir -p "$R/run/dump" "$R/run/csv"; echo x > "$R/run/pub_C.txt"
}

RUN="$R/run"; DUMP="$RUN/dump"; CSV="$RUN/csv"

echo
echo "== 〔二〕正测＝旧件在位时让路真发生 =="
seed
echo "  播种前现读＝$(ls -1 "$R" | tr '\n' ' ')"
echo "  旧窗志 ^STATE＝$(grep -a -c '^STATE=' "$R/00_window_log.txt")（★故意叠 4 枚＝上一遍的形状★）"
. "$EX"
echo "  —— 让路后 —— "
for f in 00_window_log.txt 01_preflight.txt 03_delta.txt 05_cmp.txt 07_复读.txt run; do
  [ -e "$R/$f" ] && echo "  ★还在＝$f（让路没生效）★" || echo "  已让路＝$f"
done
echo "  .prev_ 件数=$(ls -1d "$R"/.prev_* 2>/dev/null | wc -l) ‖ .runs 文件内容=$(cat "$R/.runs" 2>/dev/null) ‖ seedmark 在位＝$([ -d "$R/seedmark" ] && echo Y || echo N)（★不在改名链里的目录＝不受让路保护★）"
echo "  新窗志行数=$(wc -l < "$R/00_window_log.txt") ‖ 新窗志 ^STATE=$(grep -a -c '^STATE=' "$R/00_window_log.txt")（★应为 0＝没把上一遍叠进来★）"
OLDLOG=$(ls -1 "$R"/.prev_*_00_window_log.txt 2>/dev/null | head -1)
echo "  被让开的旧窗志仍在＝$(basename "${OLDLOG:-无}") ‖ 其 ^STATE=$(grep -a -c '^STATE=' "${OLDLOG:-/dev/null}")（★旧遍整份冻存、没被覆写★）"

echo
echo "== 〔三〕第二遍＝同一目录连跑（★遍次标注该自洽的那一遍★）=="
printf 'STATE=REF y\nSTATE=A y\n' > "$R/00_window_log.txt"
. "$EX"
echo "  本遍 .runs=$(cat "$R"/.runs) ‖ .prev_ 件数=$(ls -1d "$R"/.prev_* 2>/dev/null | wc -l) ‖ 新窗志行数=$(wc -l < "$R/00_window_log.txt") ‖ 新窗志 ^STATE=$(grep -a -c '^STATE=' "$R/00_window_log.txt")"

echo
echo "== 〔四〕反测＝手工模拟旧姿势（★不取原件代码、只说明让路值多少★）=="
M="$R/naive"; mkdir -p "$M"; printf 'STATE=REF z\nSTATE=A z\nSTATE=Ap z\nSTATE=C z\n' > "$M/00_window_log.txt"
{ cat "$M/00_window_log.txt"; printf 'STATE=REF w\nSTATE=A w\nSTATE=Ap w\nSTATE=C w\n'; } >> "$M/00_window_log.txt"
echo "  不让路＋续写后 ^STATE=$(grep -a -c '^STATE=' "$M/00_window_log.txt")（★R496 实犯过的 8★；本节是手工模拟、不是被试件的行为）"
