#!/usr/bin/env bash
# 01_留底清单自检.sh —— ★常驻可跑式（只报不拦、不入门禁串、不改任何在册件 rc 语义）★
# 起意＝R506 落 r505q：表层清单的 sha256sum -c 照样 rc=0（条数 1 vs 文件数 3 的合成反例＝本目录 01_我的复现〔三〕）
# 用法＝bash 01_留底清单自检.sh <留底目录> [<留底目录> …]（★不接 /tmp 副本、只对着仓内留底跑★）
# 口径＝n＝目录内文件数（不含清单本体）；m＝清单条数；★n−m 必须＝0★；另报"清单是否只覆盖表层"＝子目录内有无文件未被列
set -u
[ $# -ge 1 ] || { echo "用法：bash $0 <留底目录> […]" >&2; exit 9; }
bad=0
for d in "$@"; do
  [ -d "$d" ] || { echo "★目录不在位★ $d"; bad=1; continue; }
  m=0; [ -f "$d/_sha256_manifest.txt" ] && m=$(grep -c '[^[:space:]]' "$d/_sha256_manifest.txt")
  n=$(find "$d" -type f ! -name '_sha256_manifest.txt' | wc -l)
  listed=$(cut -d' ' -f3 "$d/_sha256_manifest.txt" 2>/dev/null | sort -u)
  miss=0
  while read -r f; do
    rel=${f#"$d"/}
    echo "$listed" | grep -qxF "$rel" || miss=$((miss+1))
  done < <(find "$d" -type f ! -name '_sha256_manifest.txt')
  rc=0; sha256sum -c "$d/_sha256_manifest.txt" >/dev/null 2>&1 || rc=1
  printf "%-96s n=%-4s m=%-4s n−m=%-4s 未列件=%-3s sha256sum-c_rc=%s ⇒ %s\n" \
     "$(basename "$d" | cut -c1-92)" "$n" "$m" "$((n-m))" "$miss" "$rc" \
     "$([ "$n" = "$m" ] && [ "$miss" = 0 ] && echo '覆盖OK' || echo '★漏件★')"
  { [ "$n" != "$m" ] || [ "$miss" != 0 ]; } && bad=1
done
[ "$bad" = 0 ] && echo "全部覆盖OK" || echo "★有留底目录漏件＝登记面自曝（本件只报不拦）★"
