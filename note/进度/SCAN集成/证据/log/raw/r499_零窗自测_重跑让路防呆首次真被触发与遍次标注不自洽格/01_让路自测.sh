#!/usr/bin/env bash
# 01_让路自测.sh（★第二版★＝首版有两处判式不合格、原件与输出已冻存为 04_ 与 03_）
# 零窗自测：不取新活体读数，只把 r498 窗脚本里"重跑让路"那段取出来真触发一次
# 用法＝bash 01_让路自测.sh <原件 00_r498_window.sh 全路径> <scratch 根> <在册 r498 目录>
# 取法＝只执行原件第 31—41 行与第 84 行（★逐字节、不重写★），执行前由本件把 R/RUN/DUMP/CSV 指到 scratch
# 三件事各自要证的数＝〔二〕让路真发生（六枚 .prev_ ＋ 新窗志里不含上一遍的记号）〔三〕遍次标注第二遍起才自洽
#                    〔四〕不让路的旧姿势会把 ^STATE 叠成 8（★手工模拟、明写不是被试件行为★）〔五〕不自洽在仓里已有载体
set -u
SRC="${1:?用法: bash 01_件 <原件全路径> <scratch 根> <在册 r498 目录>}"
R="${2:?scratch 根缺失}"
R498="${3:?在册 r498 目录缺失}"
EX="$R/_extract_sourced.sh"
mkdir -p "$R"
printf '# 被执行件＝下面两份 sed 输出的拼接（第 31—41 行 ‖ 第 84 行）\n' > "$EX"
sed -n '31,41p' "$SRC" >> "$EX"
sed -n '84p'   "$SRC" >> "$EX"

NAMES="00_window_log.txt 01_preflight.txt 03_delta.txt 05_cmp.txt 07_复读.txt run"

echo "== 〔一〕字节同原件自证（★被试代码不许是我重写过的★）=="
echo "  原件行数=$(wc -l < "$SRC") ‖ 被执行件行数=$(wc -l < "$EX")（＝1 行注＋11＋1）"
diff <(sed -n '31,41p' "$SRC") <(sed -n '2,12p' "$EX") > /dev/null && echo "  第 31—41 行 diff＝空 ⇒ 逐字节同" || echo "  ★第 31—41 行不同⇒被试的不是原件★"
diff <(sed -n '84p' "$SRC") <(sed -n '13p' "$EX") > /dev/null && echo "  第 84 行 diff＝空 ⇒ 逐字节同" || echo "  ★第 84 行不同⇒被试的不是原件★"
echo "  sha 原件第31—41行＝$(sed -n '31,41p' "$SRC" | sha256sum | cut -c1-16) ‖ 被执行件同段＝$(sed -n '2,12p' "$EX" | sha256sum | cut -c1-16)"

seed(){
  printf 'STATE=REF s1\nSTATE=A s1\nSTATE=Ap s1\nSTATE=C s1\n' > "$R/00_window_log.txt"
  echo pre-s1   > "$R/01_preflight.txt"
  echo delta-s1 > "$R/03_delta.txt"
  echo cmp-s1   > "$R/05_cmp.txt"
  echo du-s1    > "$R/07_复读.txt"
  mkdir -p "$R/run/dump" "$R/run/csv"; echo x-s1 > "$R/run/pub_C.txt"
}

RUN="$R/run"; DUMP="$RUN/dump"; CSV="$RUN/csv"

echo
echo "== 〔二〕正测＝旧件在位时让路真发生（★判式按内容、不按名字在不在★）=="
seed
echo "  播种＝$NAMES（★上一遍的形状★）"
echo "  旧窗志 ^STATE=$(grep -a -c '^STATE=' "$R/00_window_log.txt") ‖ 旧窗志含记号 s1 的行=$(grep -a -c 's1' "$R/00_window_log.txt")"
. "$EX"
echo "  —— 让路后 —— "
for f in $NAMES; do
  hit=$(ls -1d "$R"/.prev_*_"$f" 2>/dev/null | wc -l)
  echo "  让路件在位？$f ＝ $( [ "$hit" -ge 1 ] && echo "Y（$(basename "$(ls -1d "$R"/.prev_*_"$f" | head -1)")）" || echo '★N＝这枚没被让开★' )"
done
echo "  .prev_ 总数=$(ls -1d "$R"/.prev_* 2>/dev/null | wc -l)（★应＝6★） ‖ .runs=$(cat "$R/.runs") "
echo "  ★新窗志里含上一遍记号 s1 的行数=$(grep -a -c 's1' "$R/00_window_log.txt")（应＝0＝没串接；非 0 ⇒ 让路没挡住）★"
echo "  新窗志行数=$(wc -l < "$R/00_window_log.txt") ‖ 新窗志 ^STATE=$(grep -a -c '^STATE=' "$R/00_window_log.txt")"
OLD=$(ls -1 "$R"/.prev_*_00_window_log.txt 2>/dev/null | head -1)
echo "  被让开的旧窗志整份仍在＝${OLD:-无} ‖ 其 ^STATE=$(grep -a -c '^STATE=' "${OLD:-/dev/null}") ‖ 其 s1 行=$(grep -a -c 's1' "${OLD:-/dev/null}")（★旧遍冻存未覆写★）"

echo
echo "== 〔三〕第二遍＝同一目录连跑（★遍次标注从这一遍起才自洽★）=="
printf 'STATE=REF s2\nSTATE=A s2\n' > "$R/00_window_log.txt"
. "$EX"
echo "  本遍 .runs=$(cat "$R/.runs") ‖ .prev_ 总数=$(ls -1d "$R"/.prev_* 2>/dev/null | wc -l) ‖ 新窗志含 s2 的行=$(grep -a -c 's2' "$R/00_window_log.txt")（应＝0） ‖ 新窗志行数=$(wc -l < "$R/00_window_log.txt")"

echo
echo "== 〔四〕反测＝手工模拟旧姿势（★不取原件代码、只说明让路值多少；本节自身可复跑★）=="
M="$R/naive"; mkdir -p "$M"
printf 'STATE=REF p\nSTATE=A p\nSTATE=Ap p\nSTATE=C p\n' > "$M/prev.txt"
: > "$M/00_window_log.txt"
{ cat "$M/prev.txt"; printf 'STATE=REF q\nSTATE=A q\nSTATE=Ap q\nSTATE=C q\n'; } >> "$M/00_window_log.txt"
echo "  不让路＋续写后 ^STATE=$(grep -a -c '^STATE=' "$M/00_window_log.txt")（★R496 实犯过的 8★）"

echo
echo "== 〔五〕不自洽在仓里已有载体（★不是自测造出来的★）=="
echo "  在册 r498 窗志第 1 行＝$(head -1 "$R498/00_window_log.txt" | cut -c1-150)"
echo "  在册 r498 的 .runs＝$(cat "$R498/.runs" 2>/dev/null) ‖ .prev_ 件＝$(ls -1d "$R498"/.prev_* 2>/dev/null | xargs -r -n1 basename | tr '\n' ' ')"
echo "  ⇒ 若 .runs＝1 而 .prev_ 非空＝该遍改名让路真发生过、但标注写「第 1 遍」＝两数互相打脸"
