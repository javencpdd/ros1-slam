#!/usr/bin/env bash
# _manifest_coverage_check.sh —— 留底清单★覆盖面★自检（常驻、★只报不拦★：不入门禁串、不改任何在册件 rc 语义）
# 起意＝R506 落 r505q：★表层清单（find -maxdepth 1）的 `sha256sum -c` 照样 rc=0★（合成反例 3 件／1 条）
#       ⇒ "rc=0" 不覆盖"清单覆盖了哪些件"；本件把这条变成每次批末都会跑的第六枚附面。
# 用法＝bash _manifest_coverage_check.sh [目录 …]（★不给参数＝扫 raw/ 下全部留底目录★）
# 口径＝n＝目录内文件数（不含清单本体）；m＝清单★条目★数＝以 64 位十六进制开头的行数；★n−m 必须＝0★；另逐条比"磁盘有、清单无"的未列件数
#   ★r514q 修正（批172）＝旧式 m＝非空行数⇒把清单的注释头也计成条目★；判据面交付件的清单带四行注释头（条数写在注释里）
#   ⇒ 同一枚名字在两种清单方言下取的是两枚东西＝假漏件（n−m 为负）。现改为只数 sha 行；★只报不拦、不进任何在册件的 rc★
set -u
RAW_DEFAULT="log/raw"
base=$(cd "$(dirname "$0")" && pwd)
if [ $# -eq 0 ]; then
  dirs=$(cd "$base/$RAW_DEFAULT" && ls -d r*/ 2>/dev/null | sed 's#/$##' | sed "s#^#$base/$RAW_DEFAULT/#")
else dirs="$*"; fi
bad=0; tot=0; okc=0; nolist=0
for d in $dirs; do
  [ -d "$d" ] || { echo "★目录不在位★ $d"; bad=1; continue; }
  mf="$d/_sha256_manifest.txt"
  if [ ! -f "$mf" ]; then
    n=$(find "$d" -type f | wc -l)
    printf "  %-96s ★无清单★（件数 %s）\n" "$(basename "$d" | cut -c1-92)" "$n"; nolist=$((nolist+1)); bad=1; continue
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
  rc=0; sha256sum -c "$mf" >/dev/null 2>&1 || rc=1
  tot=$((tot+1))
  if [ "$n" = "$m" ] && [ "$miss" = 0 ]; then okc=$((okc+1)); st=覆盖OK
  else st="★漏件：n−m=$((n-m))、未列=$miss★"; bad=1; fi
  printf "  %-96s n=%-4s m=%-4s 未列=%-3s 注释行=%-3s sha256sum-c_rc=%s ⇒ %s\n" "$(basename "$d" | cut -c1-92)" "$n" "$m" "$miss" "$cm" "$rc" "$st"
done
echo "  小结＝有清单目录 $tot ‖ 覆盖OK $okc ‖ 无清单 $nolist ‖ ★本件只报不拦★"
[ "$bad" = 0 ] && echo "全部覆盖OK" || echo "★有留底目录漏件＝登记面自曝（不拦、不改串、不回填已入库清单）★"
