#!/usr/bin/env bash
# _manifest_coverage_check.sh —— 留底清单★覆盖面★自检（常驻、★只报不拦★：不入门禁串、不改任何在册件 rc 语义）
# 起意＝R506 落 r505q：★表层清单（find -maxdepth 1）的 `sha256sum -c` 照样 rc=0★（合成反例 3 件／1 条）
#       ⇒ "rc=0" 不覆盖"清单覆盖了哪些件"；本件把这条变成每次批末都会跑的第六枚附面。
# 用法＝bash _manifest_coverage_check.sh [目录 …]（★不给参数＝扫 raw/ 下全部留底目录★）｜[--BR_COV]（★自测面、夹具只在 /tmp★）
# ★批200 落 r541q 反例 C6＝「第六枚是永绿形」★＝旧形三枚缺陷同批改：①那列 `sha256sum-c_rc` 在★调用者工作目录★解析相对条目⇒仓面上全数报 1＝一枚恒假读数（独立普查真数＝0 枚换值）
#   ②判定式根本不含该 rc⇒报了也不参与判定；③末命令是 echo⇒本件★永远 rc=0＝连报的那列都是错的、且没有一枚坏样本能证明它有分辨力★
#   ⇒ 现形＝-c 一律 cd 进目录跑＋★三档并报★（OK／不合＝内容换值／缺件＝清单有盘上无）＋小结并报两枚合计＋名册辖域自报＋--BR_COV 八枚期望（含★换工作目录读数必须逐字同值★那枚反控）
#   ★缺省面 rc 语义一字未动＝仍只报不拦★＝分辨力证据全在自测面（★第十一枚 F5 同族＝检查式自己也要被测★）
# 口径＝n＝目录内文件数（不含清单本体）；m＝清单★条目★数＝以 64 位十六进制开头的行数；★n−m 必须＝0★；另逐条比"磁盘有、清单无"的未列件数
#   ★r514q 修正（批172）＝旧式 m＝非空行数⇒把清单的注释头也计成条目★；判据面交付件的清单带四行注释头（条数写在注释里）
#   ⇒ 同一枚名字在两种清单方言下取的是两枚东西＝假漏件（n−m 为负）。现改为只数 sha 行；★只报不拦、不进任何在册件的 rc★
set -u
# ★打印面★＝★不截目录名★＝本仓目录名多含中文、`cut -c1-90` 会★按字节★截断产生非法 UTF-8（批180 实测：下游 python 解码 UnicodeDecodeError）
RAW_DEFAULT="log/raw"
base=$(cd "$(dirname "$0")" && pwd)
BR_COV=0
if [ "${1:-}" = "--BR_COV" ]; then BR_COV=1; shift; fi
if [ $# -eq 0 ]; then
  dirs=$(cd "$base/$RAW_DEFAULT" && ls -d r*/ 2>/dev/null | sed 's#/$##' | sed "s#^#$base/$RAW_DEFAULT/#")
else dirs="$*"; fi
if [ "$BR_COV" = 1 ]; then
  echo "== ★自测面（夹具只在 /tmp、跑完自报已删；★靶＝批200 落 r541q 反例 C6「永绿形」）★=="
  T=$(mktemp -d /tmp/cov_selftest.XXXXXX)
  mkdir -p "$T/raw/A_干净" "$T/raw/B_换值" "$T/raw/C_缺件" "$T/raw/D_漏件" "$T/elsewhere"
  for i in 1 2; do echo "content-$i" > "$T/raw/A_干净/f$i.txt"; done
  ( cd "$T/raw/A_干净" && LC_ALL=C sha256sum f1.txt f2.txt > _sha256_manifest.txt )
  for i in 1 2; do echo "content-$i" > "$T/raw/B_换值/f$i.txt"; done
  ( cd "$T/raw/B_换值" && LC_ALL=C sha256sum f1.txt f2.txt > _sha256_manifest.txt )
  sed -i '1s/^.\{64\}/0000000000000000000000000000000000000000000000000000000000000000/' "$T/raw/B_换值/_sha256_manifest.txt"
  echo "only" > "$T/raw/C_缺件/f1.txt"
  ( cd "$T/raw/C_缺件" && LC_ALL=C sha256sum f1.txt > _sha256_manifest.txt )
  printf 'aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa  f_gone.txt\n' >> "$T/raw/C_缺件/_sha256_manifest.txt"
  for i in 1 2 3; do echo "x$i" > "$T/raw/D_漏件/f$i.txt"; done
  ( cd "$T/raw/D_漏件" && LC_ALL=C sha256sum f1.txt f2.txt > _sha256_manifest.txt )
  SELF="$base/$(basename "$0")"
  OUT=$(bash "$SELF" "$T/raw/A_干净" "$T/raw/B_换值" "$T/raw/C_缺件" "$T/raw/D_漏件" 2>&1); RC1=$?
  OUT2=$(cd "$T/elsewhere" && bash "$SELF" "$T/raw/A_干净" "$T/raw/B_换值" "$T/raw/C_缺件" "$T/raw/D_漏件" 2>&1)
  SUM1=$(printf '%s\n' "$OUT" | grep '小结')
  SUM2=$(printf '%s\n' "$OUT2" | grep '小结')
  G1=$(printf '%s\n' "$SUM1" | grep -oE '换值条目合计＝[0-9]+' | grep -oE '[0-9]+')
  G2=$(printf '%s\n' "$SUM1" | grep -oE '缺件合计＝[0-9]+' | grep -oE '[0-9]+')
  G3=$(printf '%s\n' "$OUT" | grep -c '换值=1')
  G5=$(printf '%s\n' "$OUT" | grep -c '缺件=1')
  G6=$(printf '%s\n' "$OUT" | grep -c '未列=1')
  G4=$(printf '%s\n' "$OUT" | grep -c '⇒ 覆盖OK')
  echo "  实跑小结＝$SUM1"
  echo "  逐档点名：换值行＝$G3(★1★) ‖ 缺件行＝$G5(★1★) ‖ 漏件行＝$G6(★1★) ‖ 覆盖OK＝$G4(★1★＝只剩干净那枚) ‖ 本件 rc＝$RC1(★0★＝只报不拦、缺省面不动在册 rc 语义)"
  echo "  ★恒假反控★＝换一枚调用者工作目录再跑同一批夹具 ⇒ 小结必须逐字同值；异目录小结＝$SUM2"
  BAD=""
  [ "$G1" = 1 ] || BAD="$BAD ★换值那枚没被抓＝对表式是死的（永绿形未修）★"
  [ "$G2" = 1 ] || BAD="$BAD ★条目指向缺件没被抓＝「清单有、盘上没有」这一档根本没扫★"
  [ "$G3" = 1 ] || BAD="$BAD ★状态串没点出换值＝把所有非 OK 都印成同一种数★"
  [ "$G5" = 1 ] || BAD="$BAD ★状态串没点出缺件★"
  [ "$G6" = 1 ] || BAD="$BAD ★盘上有、清单无（漏件）没被抓★"
  [ "$G4" = 1 ] || BAD="$BAD ★干净夹具被误报＝判定式过宽★"
  [ "$RC1" = 0 ] || BAD="$BAD ★缺省面本件 rc 不再是 0＝动了只报不拦的在册语义★"
  [ "$SUM2" = "$SUM1" ] || BAD="$BAD ★换工作目录读数就变＝那一列测的是调用目录、不是清单★"
  rm -rf "$T"
  echo "  夹具已删＝$([ -d "$T" ] && echo 在 || echo 无)"
  if [ -n "$BAD" ]; then echo "★自测不合期望★＝$BAD"; echo "本件（自测面）rc=5"; exit 5; fi
  echo "  自测合期望＝★八枚期望判定全合（抓 2 ‖ 分档点名 3 ‖ 不误报 1 ‖ 语义 1 ‖ 恒假反控 1）★"
  exit 0
fi
bad=0; tot=0; okc=0; nolist=0; sum_diff=0; sum_gone=0
NTOT=$(find "$base/$RAW_DEFAULT" -maxdepth 1 -mindepth 1 -type d 2>/dev/null | wc -l)
NIN=$(printf '%s\n' $dirs | grep -c .)
[ $# -eq 0 ] && echo "  ★名册辖域★＝raw 下目录总数 $NTOT ‖ 进入本件名册（r*_ 前缀） $NIN ‖ 按前缀跳过 $((NTOT-NIN))＝★辖域自报＝不让人误以为全扫过★"
for d in $dirs; do
  [ -d "$d" ] || { echo "★目录不在位★ $d"; bad=1; continue; }
  mf="$d/_sha256_manifest.txt"
  if [ ! -f "$mf" ]; then
    n=$(find "$d" -type f | wc -l)
    printf "  %-96s ★无清单★（件数 %s）\n" "$(basename "$d")" "$n"; nolist=$((nolist+1)); bad=1; continue
  fi
  m=$(grep -cE '^[0-9a-f]{64}[ *]' "$mf")
  cm=$(grep -c '^#' "$mf")
  n=$(find "$d" -type f ! -name '_sha256_manifest.txt' | wc -l)
  miss=0
  # ★条目路径归一化＝老清单多用 `find . | sha256sum` 生成 ⇒ 条目形如 "./x.log"，去前缀后再比★
  norm=$(sed -E 's/^[0-9a-f]{64}[ *]{1,2}//; s#^\./##' "$mf")
  while read -r f; do
    rel=${f#"$d"/}
    printf '%s\n' "$norm" | grep -qxF "$rel" || miss=$((miss+1))
  done < <(find "$d" -type f ! -name '_sha256_manifest.txt')
  # ★批200 落 r541q 反例 C6＝「第六枚是永绿形」★＝旧形这行在★调用者工作目录★解析相对条目 ⇒ 全数 rc=1＝一枚恒假读数、且判定式根本不含该 rc
  #   ⇒ 现形＝★cd 进目录跑★＋★三档并报★（OK／换值＝内容真漂／缺件＝条目指向盘上没有的件）＝报的那列必须先是真的，才谈得上接进判定
  tout=$(cd "$d" && LC_ALL=C sha256sum -c _sha256_manifest.txt 2>/dev/null)
  n_ok=$(printf '%s\n' "$tout" | grep -c ': OK$')
  n_diff=$(printf '%s\n' "$tout" | grep -cE ': FAILED$')
  n_gone=$(printf '%s\n' "$tout" | grep -c 'FAILED open or read')
  tot=$((tot+1)); sum_diff=$((sum_diff+n_diff)); sum_gone=$((sum_gone+n_gone))
  if [ "$n" = "$m" ] && [ "$miss" = 0 ] && [ "$n_diff" = 0 ] && [ "$n_gone" = 0 ]; then okc=$((okc+1)); st=覆盖OK
  else
    # ★状态串必须点出真因★＝旧形把所有非 OK 都印成「漏件：n−m=0、未列=0」＝一枚不带出处的数＝读者无从分辨换值／缺件／漏件
    rs=""; [ "$n" != "$m" ] && rs="$rs n−m=$((n-m))"; [ "$miss" != 0 ] && rs="$rs 未列=$miss"
    [ "$n_diff" != 0 ] && rs="$rs 换值=$n_diff"; [ "$n_gone" != 0 ] && rs="$rs 缺件=$n_gone"
    st="★非 OK：$rs★"; bad=1; fi
  printf "  %-96s n=%-4s m=%-4s 未列=%-3s 注释行=%-3s OK=%-4s 不合=%-3s 缺件=%-3s ⇒ %s\n" "$(basename "$d")" "$n" "$m" "$miss" "$cm" "$n_ok" "$n_diff" "$n_gone" "$st"
done
echo "  小结＝有清单目录 $tot ‖ 覆盖OK $okc ‖ 无清单 $nolist ‖ ★全列 sha 换值条目合计＝$sum_diff ‖ 条目指向缺件合计＝$sum_gone★ ‖ ★本件只报不拦★"
[ "$bad" = 0 ] && echo "全部覆盖OK" || echo "★有留底目录漏件＝登记面自曝（不拦、不改串、不回填已入库清单）★"
