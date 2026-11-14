#!/usr/bin/env bash
# v_rd_archive_selftest.sh (RAS) —— retake_driver 的归档/清单面零节点自测（小明 2026-10-08，随 R48 起件）
#
# 为何单立一件：R48 查出的两条缺陷都在「证据可复核」这一面，而不在判据门值上——
#   （一）清单是每段 >> 追加、且 out/ 段条目缺 out/ 前缀 ⇒ 18:23 批实测清单 88 行对 61 件，
#         从归档址跑 sha256sum -c 得 103 行非 OK，即「清单存在但不可验」；
#   （二）各段 cp 完就跑下一段 ⇒ VERDICT 行永远进不了自己的归档副本（归档件 66 行／活体件 68 行）。
# 本件按 STS 同一条纪律：**eval 现役 RD 件里的真函数体**，不复制实现（复制了就不测现役件）。
#
# 用法：bash v_rd_archive_selftest.sh     退出码 0=全过；5=有 FAIL（本件自有 rc 层，不外借）
set -u
ROOT=/home/ros1-slam
RD="$ROOT/ws/src/95_simulation/lite3_gazebo_sim/verify/retake_driver.sh"
NP=0; NF=0
mark(){ if [ "$1" = "0" ]; then NP=$((NP+1)); printf '[PASS] %s\n' "$2"; else NF=$((NF+1)); printf '[FAIL] %s\n' "$2"; fi; }
[ -f "$RD" ] || { echo "[FAIL] 现役 RD 件不在位：$RD"; echo "RAS VERDICT: rc=5"; exit 5; }

TMP=$(mktemp -d /tmp/ras_XXXXXX)
trap 'rm -rf "$TMP"' EXIT

# 取现役件的两个函数体（manifest_build / archive_out）＋ 依赖的 log/now/OUT_UNCLOSED
FN=$(awk '/^manifest_build\(\) \{/{p=1} p{print} p&&/^\}$/{p=0} /^archive_out\(\) \{/{q=1} q{print} q&&/^\}$/{q=0}' "$RD")
[ -n "$FN" ] || { echo "[FAIL] 未能从现役 RD 件取出归档函数体（件面结构变了，本自测随之失效）"; echo "RAS VERDICT: rc=5"; exit 5; }
log(){ printf '%s\n' "$*"; }
OUT_UNCLOSED=0
eval "$FN"
grep -q 'manifest_build' <<<"$FN" && grep -q "sha256sum -c" <<<"$FN" \
  && mark 0 "A0 函数体取自现役件且含清单可验断言（非本地复制的实现）" \
  || mark 1 "A0 函数体取不到可验断言 ⇒ 现役件仍是旧式追加"

run_ao(){ # $1=OUT $2=ARCHIVE_DIR $3=打印落件；**不用命令替换**：$() 会开子 shell，OUT_UNCLOSED 的改动母 shell 读不到（本测试首版正是栽在这里，属假红而非假绿，但同族教训）
  OUT="$1"; ARCHIVE_DIR="$2"; MANIFEST="$2/_sha256_manifest.txt"; OUT_UNCLOSED=0
  archive_out > "$3" 2>&1
}

# A1 三面同数（阳性对照：正常一批件必须全绿且给四个数）
mkdir -p "$TMP/o1" "$TMP/e1"
for i in 1 2 3; do echo "x$i" > "$TMP/o1/f$i.log"; done
mkdir -p "$TMP/o1/sub"; echo y > "$TMP/o1/sub/g.log"
run_ao "$TMP/o1" "$TMP/e1" "$TMP/l1"; L=$(cat "$TMP/l1")
OK1=$(echo "$L" | grep -c 'OUT 收口')
N4=$(echo "$L" | sed -n 's/.*件数=\([0-9]*\) ＝ 归档件数=\([0-9]*\) ｜清单可验=\([0-9]*\) ＝ 全件数=\([0-9]*\)（.*/\1 \2 \3 \4/p')
[ "$N4" = "4 4 4 4" ] && mark 0 "A1a 打印四数同值（OUT=out=可验=全件=4）" || mark 1 "A1a 四数不同值（四数=$N4，应 4 4 4 4）"
[ "$OK1" = "1" ] && [ "$N4" = "4 4 4 4" ] && mark 0 "A1 正常批：收口成立且四数同值（打印=$N4）" \
                 || mark 1 "A1 正常批四数不同值（OK 行=$OK1 四数=$N4，应 4 4 4 4）"

# A2 清单从归档址可验（R48 一 的正面：这条正是旧式跑不通的那一步）
VC=$( cd "$TMP/e1" && sha256sum -c "$TMP/e1/_sha256_manifest.txt" 2>&1 )
BAD=$(echo "$VC" | grep -vc ': OK$')
ENT=$(wc -l < "$TMP/e1/_sha256_manifest.txt")
FILES=$(find "$TMP/e1" -type f ! -name '_sha256_manifest.txt' | wc -l)
[ "$BAD" = "0" ] && [ "$ENT" = "$FILES" ] && [ "$ENT" -ge 1 ] \
  && mark 0 "A2 清单从归档址 -c 全 OK 且条目数＝件数（$ENT＝$FILES）" \
  || mark 1 "A2 清单不可验或条目不符（非 OK 行=$BAD 条目=$ENT 件数=$FILES）"

# A3 路径带 out/ 前缀（旧式缺前缀 ⇒ 跨目录同名件在一张表里不可判读）
HASPFX=$(grep -c '  out/f1[.]log$' "$TMP/e1/_sha256_manifest.txt")
[ "$HASPFX" = "1" ] && mark 0 "A3 条目路径含 out/ 前缀（$(grep 'out/f1.log$' "$TMP/e1/_sha256_manifest.txt" | awk '{print $2}')）" \
                    || mark 1 "A3 条目仍缺 out/ 前缀 ⇒ 跨目录同名件不可判读"

# A4 阴性对照：归档址里有一件读不了 ⇒ 必须打真红（旧式只比 n=m，这一型静默过关）
echo z > "$TMP/e1/unreadable.log"; chmod 000 "$TMP/e1/unreadable.log"
run_ao "$TMP/o1" "$TMP/e1" "$TMP/l4"; L4=$(cat "$TMP/l4")
[ "$OUT_UNCLOSED" = "1" ] && echo "$L4" | grep -q 'OUT 未收口' \
  && mark 0 "A4 不可读件被打真红（OUT_UNCLOSED=1；四数打印见下）" \
  || mark 1 "A4 不可读件竟过关＝清单可验面是假绿灯"
echo "$L4" | sed 's/^/      /'
chmod 644 "$TMP/e1/unreadable.log"; rm -f "$TMP/e1/unreadable.log"

# A5 空集真命题防护：OUT 空 ⇒ 不得记为收口（坑29 同族）
mkdir -p "$TMP/o5" "$TMP/e5"
run_ao "$TMP/o5" "$TMP/e5" "$TMP/l5"; L5=$(cat "$TMP/l5")
[ "$OUT_UNCLOSED" = "1" ] && mark 0 "A5 空 OUT 判不洁（不因 -ge 1 失守而默认绿）" \
                          || mark 1 "A5 空 OUT 被记为收口 ⇒ 空集真命题"

# A6 清单不列自身、不列 .new 残件
SELF=$(grep -c '_sha256_manifest.txt' "$TMP/e1/_sha256_manifest.txt")
NEWF=$(find "$TMP/e1" -name '*.new' | wc -l)
[ "$SELF" = "0" ] && [ "$NEWF" = "0" ] && mark 0 "A6 清单不含自身与 .new 临时件" \
                   || mark 1 "A6 清单含自身=$SELF 或残留 .new=$NEWF"

# A7 终局指认式（tail 用的那条 grep -c）在两种态下分别给 1／0
mkdir -p "$TMP/e7/out"
echo "===== RETAKE VERDICT: rc=0 2026-01-01 00:00:00 =====" > "$TMP/e7/out/driver.log"
V1=$(grep -c 'RETAKE VERDICT' "$TMP/e7/out/driver.log")
echo "nothing here" > "$TMP/e7/out/driver.log"
V0=$(grep -c 'RETAKE VERDICT' "$TMP/e7/out/driver.log")
[ "$V1" = "1" ] && [ "$V0" = "0" ] && mark 0 "A7 终局指认式两态可分（含行=1／不含行=0）" \
                 || mark 1 "A7 终局指认式两态不可分（$V1／$V0）"

echo "=================================================="
echo "RAS VERDICT: PASS=$NP FAIL=$NF"
[ "$NF" = "0" ] && exit 0 || exit 5
