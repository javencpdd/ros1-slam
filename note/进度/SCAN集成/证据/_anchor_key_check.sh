#!/usr/bin/env bash
# _anchor_key_check.sh —— ★判读面复核件（常驻、施工侧自有登记面件：★不入门禁串、不改任何在册件的 rc 语义★）
# 起意＝批181（R525）实测两枚：
#   ①窗脚本第 294 行把一段字面串塞进 `sed -E` 当★正则★用，其中 `+` 未转义 ⇒ ★两枚时刻 token 一枚也没命中★＝那枚「排除式」是★死码★，
#      而它输出的「除 token 外逐行差＝2」恰好等于「未排除」的数＝★不报命中枚数，读者永不可能识破★（追加一百一十二①／②）。
#   ②判读面的行号改由机读表 `09_锚键.tsv` 生成＝★追加一百一十一⑧ 的执行形★，但这件事当时只在★窗内一次性★实现过；
#      按追加一百一十二⑧「★新件的可用性门槛＝被下游真实调用一次★」，抽成常驻件才算立住。
# 用法＝bash _anchor_key_check.sh [--all] [--BR_MISS] [窗目录…]   （★缺省＝当前目录；--all＝扫 raw/ 下所有带 05_cmp.txt 的目录★）
# 四枚断言（★逐枚并报、缺一即红 rc=5★）＝
#   A1 ★正则自证★＝本件每条它自己要用的式子，先拿被试件里的一枚真样本跑★命中≥1★；命中 0 ⇒ 点名「死码」（★在册新立：不转义的字面串一律 re.escape 或改用字符类★）
#   A2 ★锚表在位★＝目录内 `09_锚键.tsv` 存在且枚数＝6；★不在位＝记「该窗未带锚表」＝只报不拦、★不计红★（历窗没有这枚表）
#   A3 ★表与现跑对表★＝每枚键：cmp 面表值★逐字等★现跑；志面表值★⊂★现跑并报多命中枚数（★表是时点产物＝算完之后窗志才被 tee 追加★）
#   A4 ★排除式自报命中枚数★＝「本件时刻＝」出现枚数 ‖ 在册那枚式子命中枚数 ‖ 转义安全式命中枚数；★出现>命中 ⇒ 该枚判据未经行使＝红★
# ★负例开关 --BR_MISS★＝故意把一枚键值改成不可能命中的数 ⇒ 本件★必须翻红（rc=5）★并点名＝证明 A3 真有牙（★只报不拦的件也要有跑证的反例面★）
set -u
ROOT=$(cd "$(dirname "$0")/../../../.." && pwd)
RAW="$ROOT/note/进度/SCAN集成/证据/log/raw"
MODE=""; BRK=""
DIRS=""
while [ $# -gt 0 ]; do
  case "$1" in
    --all) MODE=all; shift ;;
    --BR_MISS) BRK=1; shift ;;
    *) DIRS="$DIRS $1"; shift ;;
  esac
done
if [ "$MODE" = all ]; then
  DIRS=$(find "$RAW" -maxdepth 1 -mindepth 1 -type d -name 'r*_*' | while read -r d; do [ -f "$d/05_cmp.txt" ] && printf '%s\n' "$d"; done)
fi
[ -n "$DIRS" ] || DIRS="."

python3 - "$RAW" "$BRK" $DIRS <<'PYEOF'
import io, os, re, sys

RAW, BRK, dirs = sys.argv[1], sys.argv[2], sys.argv[3:]
TOK_SAFE = re.compile(r"本件时刻＝[0-9-]{10} [0-9:]{8}（[^）]*）")
TOK_NAIVE_SRC = "本件时刻＝[0-9-]{10} [0-9:]{8}（★容器面 UTC；宿主 CST＝+8★）"
EXP = 6
red = []
warn = []


def lines(p):
    return io.open(p, encoding="utf-8", errors="replace").read().split("\n")


def hits(path, pat):
    if not os.path.exists(path):
        return None
    return [i + 1 for i, l in enumerate(lines(path)) if pat in l]


print("== 判读面复核件（★常驻·只报不拦★）＝锚表 × 现跑 grep × 正则自证 ==")
# A1 ★正则自证★＝先在本目录自己的源码里找一枚真样本跑命中
naive = re.compile(TOK_NAIVE_SRC)
probe = "本件时刻＝2026-10-20 22:23:15（★容器面 UTC；宿主 CST＝+8★）"
hit_naive = len(naive.findall(probe))
hit_safe = len(TOK_SAFE.findall(probe))
print("  A1 正则自证（★在册那枚未转义的式子 vs 安全式★）＝未转义命中★%d★ ‖ 安全式命中%d ‖ 阳性样本＝「%s」" % (hit_naive, hit_safe, probe[:34]))
print("     ★⇒ 未转义式子★对真样本 0 命中＝死码★（`+` 被当量词）＝本件一律用安全式；★这句不是推演、是本件当场跑出来的★")
if hit_safe < 1:
    red.append("A1 安全式对阳性样本 0 命中＝本件自己的式子也是死的")

n_ok = n_bad = n_notable = 0
for d in dirs:
    d = os.path.abspath(d)
    name = os.path.basename(d)
    cmp_p = os.path.join(d, "05_cmp.txt")
    log_p = os.path.join(d, "00_window_log.txt")
    tsv_p = os.path.join(d, "09_锚键.tsv")
    if not os.path.exists(cmp_p):
        continue
    # A4 排除式自报命中枚数
    cl = lines(cmp_p)
    n_appear = sum(1 for x in cl if "本件时刻＝" in x)
    n_hit_naive = sum(1 for x in cl if naive.search(x))
    n_hit_safe = sum(1 for x in cl if TOK_SAFE.search(x))
    if n_appear and n_hit_safe != n_appear:
        red.append("%s A4 安全式命中 %d ≠ 出现 %d" % (name, n_hit_safe, n_appear))
    # A2 锚表
    if not os.path.exists(tsv_p):
        n_notable += 1
        print("  %-46s ‖ A2 ★未带 09_锚键.tsv★＝记★只报不拦★（历窗没有这枚表） ‖ A4 出现%d ‖ 未转义命中%d ‖ 安全式命中%d" % (
            name[:46], n_appear, n_hit_naive, n_hit_safe))
        continue
    rows = [l.split("\t") for l in io.open(tsv_p, encoding="utf-8").read().rstrip("\n").split("\n")]
    if BRK == "1" and rows:
        r = rows[0]
        r[1] = "999"
        rows[0] = r
    okc = okl = True
    miss = []
    extra = []
    if len(rows) != EXP:
        miss.append("表枚数=%d≠%d" % (len(rows), EXP))
    for row in rows:
        if len(row) < 4:
            continue
        k, crows, lrows, pat = row[0], row[1], row[2], row[3]
        c_now = hits(cmp_p, pat) or []
        l_now = hits(log_p, pat) or []
        if crows != ",".join(map(str, c_now)):
            okc = False
            extra.append("%s.cmp(%s≠%s)" % (k, crows, ",".join(map(str, c_now))))
        want = set(int(x) for x in lrows.split(",") if x.strip())
        if not want <= set(l_now):
            okl = False
            extra.append("%s.log(表值不在现跑里)" % k)
        elif len(l_now) > len(want):
            extra.append("%s.log★多命中%d枚★(时点性)" % (k, len(l_now) - len(want)))
    line = "  %-46s ‖ A2 表枚数=%d ‖ A3 cmp面=%s 志面⊂现跑=%s ‖ A4 出现%d ‖ 未转义命中%d ‖ 安全式命中%d" % (
        name[:46], len(rows), "合" if okc else "★不合★", "合" if okl else "★不合★", n_appear, n_hit_naive, n_hit_safe)
    print(line)
    if extra:
        print("      点名＝%s" % " ‖ ".join(extra))
    if okc and okl and len(rows) == EXP:
        n_ok += 1
    else:
        n_bad += 1
        red.append("%s A3 表与现跑不合（见点名）" % name)

print("\n小结＝红面枚数★%d★ ‖ 表合%d ‖ 表不合%d ‖ 未带表%d（★未带表只报不拦★）" % (len(red), n_ok, n_bad, n_notable))
for x in red:
    print("   ★红★＝%s" % x)
if BRK == "1":
    print("   ★负例开关 --BR_MISS 已行使＝本枚预期★红≥1★；不红＝A3 这枚断言没牙★")
RC = 5 if red else 0
print("本件 rc=%s（0＝已带表的窗逐枚对表全合＋安全式命中＝出现枚数；5＝任一名点；★只报不拦、不进任何在册件的 rc★）" % RC)
sys.exit(RC)
PYEOF
