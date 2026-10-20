#!/usr/bin/env bash
# _manifest_build.sh —— 留底清单★建单闸★（常驻、施工侧自有登记面件：★不入门禁串、不改任何在册件的 rc 语义★）
# 起意＝批177（R521）实测：手敲管道建单时列表枚数＝0 ⇒ `sha256sum` 读★标准输入★写出「空串的 sha256（e3b0c442…）＋ `-`」
#        这样一枚★恒条目★、`sha256sum -c` ★照样 rc=0★ ⇒ 三件套里那句「清单在位且 rc=0」在那一回是★空转★的。
#        批176 的在册教训＝★防呆没有常驻代码载体时，只能发「被行使」、发不出「有牙」★ ⇒ 本件把这条变成一件能跑、会翻红的东西。
# 用法＝bash _manifest_build.sh [--build|--check] [--BR_EMPTY] [目录…]（★缺省＝当前目录；--build 是缺省动作★）
#   --check  ＝只验不建（★复核方另用一条独立式子数一遍＝在册姿势★）
#   --BR_EMPTY＝★负例开关★：故意让文件列表为空 ⇒ 本件★必须翻红（rc=5）★并点名「条目枚数＝0」＝证明这枚闸真有牙
# 断言（★三枚并报、缺一即红★）＝①sha 条目枚数＞0 ②条目枚数＝目录内文件数−1（★清单自己不录入★）③`sha256sum -c` rc=0
# 清单方言＝「<64位sha><两个空格><相对路径>」＋★头部注释行以 # 起★（★注释行不计入条目＝批174 修过的那枚口径★）
set -u
# ★打印面★＝★不截目录名★＝本仓目录名多含中文、`cut -c1-90` 会★按字节★截断产生非法 UTF-8（批180 实测：下游 python 解码 UnicodeDecodeError）

base=$(cd "$(dirname "$0")" && pwd)
MODE=build; BR_EMPTY=0; DIRS=""
for a in "$@"; do
  case "$a" in
    --check) MODE=check ;;
    --build) MODE=build ;;
    --BR_EMPTY) BR_EMPTY=1 ;;
    *) DIRS="$DIRS $a" ;;
  esac
done
[ -n "$DIRS" ] || DIRS="."

fail=0
for d in $DIRS; do
  [ -d "$d" ] || { echo "★目录不在位★ $d"; fail=1; continue; }
  mf="$d/_sha256_manifest.txt"
  # ★独立式子①＝文件数（不含清单本体）★
  n=$(find "$d" -type f ! -name '_sha256_manifest.txt' | wc -l)
  if [ "$MODE" = build ]; then
    if [ "$BR_EMPTY" = 1 ]; then
      # 负例开关：复现批177 那枚事故＝列表为空仍去喂 sha256sum
      : | sha256sum > "$mf" 2>/dev/null
      echo "  （★负例开关★＝故意写一枚空输入的 sha 进清单，模拟批177 那次事故）"
    else
      {
        printf '# _sha256_manifest.txt —— ★清单最后建、本清单自己不录入★\n'
        printf '# 建单时刻＝%s（★date 现取、不心算★） ‖ 建单件＝_manifest_build.sh（★建单闸★）\n' "$(date '+%F %T')"
        printf '# ★断言★＝sha 条目枚数＝目录内文件数−1 且＞0；★只报 sha256sum -c rc=0 不算过★\n'
        find "$d" -type f ! -name '_sha256_manifest.txt' -printf '%P\n' | LC_ALL=C sort | sed "s#^#$d/#" | xargs -d '\n' sha256sum \
          | sed "s#  $d/#  #"
      } > "$mf"
    fi
  fi
  [ -f "$mf" ] || { echo "  ★清单不在位★ $mf"; fail=1; continue; }
  # ★独立式子②＝只数 sha 行（^[0-9a-f]{64}[ *]），注释行单列★
  m=$(grep -cE '^[0-9a-f]{64}[ *]' "$mf")
  cm=$(grep -c '^#' "$mf")
  rc=0; (cd "$d" && sha256sum -c --quiet _sha256_manifest.txt) >/dev/null 2>&1 || rc=1
  verdict=OK
  [ "$m" -gt 0 ] || { verdict="★条目枚数＝0＝恒真清单（批177 那枚事故）★"; fail=1; }
  [ "$m" = "$n" ] || { verdict="★漏件/多列：条目 $m ≠ 文件数 $n★"; fail=1; }
  [ "$rc" = 0 ] || { verdict="$verdict ‖ ★sha256sum -c rc=$rc★"; fail=1; }
  printf "  %-92s 文件数(不含清单)=%-4s sha条目=%-4s 注释行=%-3s -c_rc=%s ⇒ %s\n" \
    "$(basename "$d")" "$n" "$m" "$cm" "$rc" "$verdict"
done
if [ "$fail" = 0 ]; then echo "建单闸全过（★三枚断言逐枚真取：条目＞0 ‖ 条目＝文件数 ‖ -c rc=0★）"; exit 0; fi
echo "★建单闸翻红＝登记面自曝（本件只作用于★建单方自己★：不进门禁串、不改任何在册件的 rc 语义）★"
exit 5
