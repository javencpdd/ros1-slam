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
ROOTQ=$(cd "$base/../../../.." && pwd)
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
# ★★G19/R534＝负例开关的写出面必须在★夹具目录★里（旧形＝--BR_EMPTY 往★真目标★写一枚空清单＝
#   本批 14:14 实测＝我从证据面跑 --BR_EMPTY、没带目录参数 ⇒ 在 证据/ 落了一枚 68 字节的空清单
#   ＝批186 自曝④那条铁律（★反控与干跑一律不许指向已入库目录★）在★自家常驻件的自测分支★里成真＝已删、已改形★）
BR_TMP=""
if [ "$BR_EMPTY" = 1 ]; then
  BR_TMP=$(mktemp -d /tmp/manifest_br_empty.XXXXXX)
  case "$BR_TMP" in /tmp/*) : ;; *) echo "★夹具写出面不在 /tmp＝$BR_TMP ⇒ 负例不跑★"; exit 9 ;; esac
  echo "  ★负例开关★＝写出面★强制换到夹具★＝$BR_TMP（★原参数里的目录一枚不写★）"
  DIRS="$BR_TMP"
fi

fail=0
for d in $DIRS; do
  [ -d "$d" ] || { echo "★目录不在位★ $d"; fail=1; continue; }
  d=$(cd "$d" && pwd)   # ★G19 牙根／R534＝目标先归★绝对面★（旧形＝相对参数直接拼＝`${mf#$ROOTQ/}` 削不出相对键⇒闸恒不响⇒本批三次重写已入库 r533 清单都靠 git checkout 还原＝★一枚闸连着两遍没牙、第三遍才对上键★）
  mf="$d/_sha256_manifest.txt"
  # ★独立式子①＝文件数（不含清单本体）★
  n=$(find "$d" -type f ! -name '_sha256_manifest.txt' | wc -l)
  # ★G19 反控②／R534＝已入库清单不许被★重建★覆写。旧形＝`git ls-files --error-unmatch` 拿★绝对路径★查
  #   ⇒ 恒查不中＝这枚闸★没牙★＝本批实测两次：我拿 --build 对已入库的 r533 各跑一次、清单的时刻行被重写两次（都靠 git checkout 还原＝已入库⇒可逆）
  #   正形＝路径先★削掉仓根★再问 git（git 的键是★相对面★）；查不到＝未入库⇒准建（本窗留底就是未入库那支）
  if [ "$MODE" = build ] && [ "$BR_EMPTY" != 1 ] && [ -f "$mf" ]; then
    REL="${mf#$ROOTQ/}"
    if git -C "$ROOTQ" ls-files --error-unmatch "$REL" >/dev/null 2>&1; then
      echo "  ★拒绝＝$d 的清单★已入库★（git 相对面＝$REL 查得到）＝已入库只读不写、清单零回填 ⇒ 要复核请用 --check"
      fail=1
      continue
    fi
  fi
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
[ -n "$BR_TMP" ] && rm -rf "$BR_TMP" && echo "  ★夹具已删＝$BR_TMP 现读 $([ -d "$BR_TMP" ] && echo 在 || echo 无)★"
if [ "$fail" = 0 ]; then echo "建单闸全过（★三枚断言逐枚真取：条目＞0 ‖ 条目＝文件数 ‖ -c rc=0★）"; exit 0; fi
echo "★建单闸翻红＝登记面自曝（本件只作用于★建单方自己★：不进门禁串、不改任何在册件的 rc 语义）★"
exit 5
