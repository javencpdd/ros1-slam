#!/usr/bin/env bash
# _ledger_form_check.sh —— ★志面与标题禁形闸（常驻、施工侧自有登记面件＝第十一枚：★不入门禁串、不改任何在册件的 rc 语义★）
# 起意＝本 boot 反引号／ASCII 双引号进「会被 shell 解释的通道」★第八次★复发（批197 用 echo 串装工单文本、串里带反引号⇒被当命令替换执行、盘面报「没有那个文件或目录」），
#        同一批我又★手打了一枚不存在的 token★ 让形检的★检查式★自错＝「姿势写在脑子里＝等于没姿势」（追加一百一十九那条老账）。
#        ⇒ 在册规矩（note/ 三面与 .ai/ 卡面禁用反引号与 ASCII 双引号、一律「」；提交标题必须成形）★从来没有一枚可跑式子盯它★＝本件补这格。
# ★被检面分档（★只检★本批新增行★＝历史行照留＝不破「旧文不回改」★）＝
#   A ＝三面（＋可选卡面 --tasks）的★新增行★＝git diff --unified=0 对 HEAD 取 + 行
#   B ＝★提交标题那一行★（--msg 文件；★形检要跑在写文件之前、也要跑在 commit 之前★＝追加七十八④）
#   S ＝★样本通道★（--file 文件）＝★与被检新增行走同一对式子★＝自测面用它、不是另写一枚弱检
# 断言（★逐枚并报、任一不合＝本件 rc=5★）＝
#   F1 ★反引号＝0 枚★ ‖ F2 ★ASCII 双引号＝0 枚★（A 与 S 两档共用）
#   F3 ★标题五枚谓词★＝^x86侧批数字（·补(数字｜一二三四五)）?（…）＝… ‖ 中文括号成对 ‖ 单行 ‖ 无禁形 ‖ 「x86侧批」恰 1 枚
#   F4 ★消费者在位★＝_batch_readout.sh 引用本件枚数≥1（追加一百一十二⑩）＝★只在批末全量模式判★
#   F5 ★检查式自己也要被测★＝自测面先喂一枚★已知含禁形★的样本、★抓不到＝式子坏＝比漏报更坏★（当场红并点名）
# 用法＝bash _ledger_form_check.sh [--msg 文件] [--file 文件] [--tasks] [--BR_FORM]
# 退出码＝0（全过）‖ 5（任一名点）‖ 9（前置缺失＝不在 git 仓内／给的文件读不到）
set -u
ROOT=$(cd "$(dirname "$0")/../../../.." && pwd)
MSG=""; TASKS=0; BRK=0; SAMPLE=""
while [ $# -gt 0 ]; do
  case "$1" in
    --msg) MSG="$2"; shift 2 ;;
    --file) SAMPLE="$2"; shift 2 ;;
    --tasks) TASKS=1; shift ;;
    --BR_FORM) BRK=1; shift ;;
    *) echo "未知参数 $1"; exit 9 ;;
  esac
done
cd "$ROOT" || exit 9
git rev-parse HEAD >/dev/null 2>&1 || { echo "★前置缺失＝无 HEAD 可比★"; exit 9; }
for f in "$MSG" "$SAMPLE"; do
  [ -z "$f" ] || [ -f "$f" ] || { echo "★前置缺失＝给的文件读不到★ $f"; exit 9; }
done

if [ "$BRK" = 1 ]; then
  T=$(mktemp -d /tmp/ledger_form.XXXXXX)
  printf -- '- 合形的一行＝用「引号」与全角括号（这样）★不踩禁形★\n' > "$T/good.txt"
  printf -- '- 踩反引号的一行＝写了一段 `ls -l` 在里面\n' > "$T/bad_tick.txt"
  printf -- '- 踩 ASCII 双引号的一行＝写了一句 "quoted" 在里面\n' > "$T/bad_dq.txt"
  printf -- 'x86侧批999（★好标题★）＝正文成形（括号成对）\n' > "$T/msg_good.txt"
  printf -- 'x86侧批999（坏标题缺右括号＝正文\n' > "$T/msg_bad1.txt"
  printf -- '标题缺前缀（★没写 x86侧批★）＝不成形\n' > "$T/msg_bad2.txt"
  printf -- 'x86侧批999（★坏标题里带反引号 `ls`★）＝不成形\n' > "$T/msg_bad3.txt"
  echo "== ★自测面（夹具只在 /tmp、跑完自报已删）=="
  G1=$(bash "$0" --file "$T/good.txt" 2>&1 | grep -c '\[红\]')
  B1=$(bash "$0" --file "$T/bad_tick.txt" 2>&1 | grep -c 'F1')
  B2=$(bash "$0" --file "$T/bad_dq.txt" 2>&1 | grep -c 'F2')
  bash "$0" --msg "$T/msg_bad1.txt" >/dev/null 2>&1; R1=$?
  bash "$0" --msg "$T/msg_bad2.txt" >/dev/null 2>&1; R2=$?
  bash "$0" --msg "$T/msg_bad3.txt" >/dev/null 2>&1; R3=$?
  OUT4=$(bash "$0" --msg "$T/msg_good.txt" 2>&1); R4=$?
  F3G=$(printf '%s\n' "$OUT4" | grep -c 'F3 标题＝★不过★')
  echo "  两支禁形＝踩反引号被抓 $B1(≥1) ‖ 踩 ASCII 双引号被抓 $B2(≥1) ‖ 合形样本被点名 $G1(★0★)"
  echo "  标题三支坏样本 rc＝$R1/$R2/$R3（★全部必须 5★） ‖ 好标题＝F3 判「不过」枚数 $F3G（期望★0★；★rc=$R4 只可能因 F4／A 档＝本控不判★）"
  BAD=""
  [ "$B1" -ge 1 ] || BAD="$BAD ★F5：F1 没抓到已知反引号＝式子坏＝比漏报更坏★"
  [ "$B2" -ge 1 ] || BAD="$BAD ★F5：F2 没抓到已知 ASCII 双引号＝式子坏★"
  [ "$G1" = 0 ] || BAD="$BAD ★合形样本被误报＝锚过宽（$G1 条红）★"
  [ "$R1" = 5 ] || BAD="$BAD ★缺右括号的标题没翻红（实=$R1）＝F3 没牙★"
  [ "$R2" = 5 ] || BAD="$BAD ★缺前缀的标题没翻红（实=$R2）★"
  [ "$R3" = 5 ] || BAD="$BAD ★标题里的反引号没翻红（实=$R3）★"
  [ "$F3G" = 0 ] || BAD="$BAD ★好标题被判不过＝F3 锚过宽★"
  rm -rf "$T"
  echo "  夹具已删＝$([ -d "$T" ] && echo 在 || echo 无)"
  if [ -n "$BAD" ]; then echo "★自测不合期望★＝$BAD"; echo "本件（自测面）rc=5"; exit 5; fi
  echo "  自测合期望＝★七枚期望判定全合（禁形抓 2 ‖ 不误报 2 ‖ 标题抓 3 ‖ 好标题 1）★"
  echo "本件（自测面）rc=0"; exit 0
fi

python3 - "$ROOT" "$MSG" "$TASKS" "$SAMPLE" <<'PYEOF'
import io, os, re, subprocess, sys
ROOT, MSG, TASKS, SAMPLE = sys.argv[1], sys.argv[2], sys.argv[3] == "1", sys.argv[4]
red = []
FILES = ["note/04_配置改动diff.md", "note/05_复验作业单.md", "note/进度/00_总览.md", "note/00_README.md",
         "note/进度/总览分册/00_档头历史.md"]
if TASKS:
    d = os.path.join(ROOT, ".ai", "tasks")
    if os.path.isdir(d):
        FILES += [os.path.join(".ai", "tasks", f) for f in sorted(os.listdir(d)) if f.endswith(".md")]
TICK = re.compile(r"`")
DQ = re.compile(r'"')
n1 = n2 = 0
def judge(where, l):
    global n1, n2
    t = len(TICK.findall(l)); q = len(DQ.findall(l))
    if t:
        n1 += 1; red.append("F1 %s 含反引号 %d 枚 ‖ 片段＝%s" % (where, t, l[:40]))
    if q:
        n2 += 1; red.append("F2 %s 含 ASCII 双引号 %d 枚 ‖ 片段＝%s" % (where, q, l[:40]))
print("== 志面与标题禁形闸（★第十一枚·只报不拦、不入门禁串★）==")
if SAMPLE:
    p = SAMPLE if os.path.isabs(SAMPLE) else os.path.join(ROOT, SAMPLE)
    for i, l in enumerate(io.open(p, encoding="utf-8", errors="replace").read().split("\n"), 1):
        judge("%s:%d（样本）" % (os.path.basename(SAMPLE), i), l)
    print("  ★模式＝样本通道★（与被检新增行走同一对式子） ‖ F1＝%d ‖ F2＝%d" % (n1, n2))
else:
    def added(path):
        out = subprocess.run(["git", "-c", "core.quotepath=false", "diff", "--unified=0", "--", path],
                             cwd=ROOT, capture_output=True, text=True, errors="replace").stdout
        return [l[1:] for l in out.split("\n") if l.startswith("+") and not l.startswith("+++")]
    tot = 0
    print("  被检面名册枚数＝%d%s（★只检新增行★＝历史行照留）" % (len(FILES), " ‖ ★含卡面 --tasks★" if TASKS else " ‖ 卡面未入（加 --tasks 才扫）"))
    for f in FILES:
        ls = added(f); tot += len(ls)
        for l in ls: judge(f, l)
    print("  新增行合计＝%d 行 ‖ F1 反引号行＝%d 枚 ‖ F2 ASCII 双引号行＝%d 枚" % (tot, n1, n2))
if MSG and not SAMPLE:
    p = MSG if os.path.isabs(MSG) else os.path.join(ROOT, MSG)
    s = io.open(p, encoding="utf-8").read()
    first = s.split("\n")[0]
    bad = []
    if re.match(r"^x86侧批[0-9]+(·补([0-9]+|[一二三四五]))?（.*）＝.*", first, re.S) is None: bad.append("标题形不合（缺前缀或缺「）＝」）")
    if first.count("（") != first.count("）"): bad.append("中文括号不成对（（ %d ‖ ） %d）" % (first.count("（"), first.count("）")))
    if "\n" in s.rstrip("\n"): bad.append("标题含换行（★本仓形＝单行超长★）")
    if TICK.search(first) or DQ.search(first): bad.append("标题含禁形（反引号或 ASCII 双引号）")
    if first.count("x86侧批") != 1: bad.append("「x86侧批」枚数＝%d≠1" % first.count("x86侧批"))
    for b in bad: red.append("F3 标题面 ‖ %s" % b)
    print("  F3 标题＝%s（字符 %d ‖ 五枚谓词不合枚数＝%d）" % ("★不过★" if bad else "过", len(first), len(bad)))
elif not MSG and not SAMPLE:
    print("  F3 标题面＝★本行未给 --msg＝不可证（★不写成「无缺」★）")
d4 = 0
if not SAMPLE:
    readout = os.path.join(ROOT, "note", "进度", "SCAN集成", "证据", "_batch_readout.sh")
    if os.path.isfile(readout):
        d4 = io.open(readout, encoding="utf-8", errors="replace").read().count("_ledger_form_check.sh")
    print("  F4 消费者＝_batch_readout.sh 引用本件枚数＝%d ⇒ %s" % (d4, "在位" if d4 >= 1 else "★死件★"))
    if d4 < 1: red.append("F4 本件无消费者")
print("  ★红档枚数＝%d★（★只报不拦★）" % len(red))
for r in red: print("     [红] %s" % r)
sys.exit(5 if red else 0)
PYEOF
RC=$?
echo "本件 rc=$RC（0＝全过 ‖ 5＝任一名点 ‖ 9＝前置缺失）；★不进任何在册件的 rc、不入门禁串★"
exit $RC
