#!/usr/bin/env bash
# _governance_drift_check.sh —— ★governance 漂移闸（常驻、施工侧自有登记面件＝第十枚：★不入门禁串、不改任何在册件的 rc 语义★）
# 起意＝真人 2026-10-21 指出「tooling/skill 的 SKILL.md 描述、note/00_README 门面行、00_总览 现行格」三处停在 v4／v5.3 时代＝
#       ★同一枚病（02 坑60 的反向亚种）＝「引用带死版本号也会漂」★；本包与门面已改★指针形★⇒ 但没有格子盯它是否又漂回去＝本件补这格。
# 姿势＝★换代留名不等于换代不漂★＝凡「只登记一句规矩、没有可跑式子盯」的到签硬规矩，下次一定再犯（追加一百二十二⑦／追加一百一十九）。
# ★两态面设计（本件最承重的一格）★＝本件扫的面分两档、只有第一档记红＝
#   【红档＝施工侧★有写入通道★的面】`tooling/skill/**/*.md` ＋ 仓根 `README.md` ＋ `note/00_README.md` 的★现行指针节内★
#   【旁报＝施工侧★无写入通道★的面】`note/计划书v7/*.md` 正文（官方表述笔权）＋ `note/00_README.md` 历史行（旧文不回改）
#     ⇒ 理由＝把无通道面记红＝造一枚★永红闸★＝等于关掉报警器；但完全不扫＝漂了没人知道 ⇒ 记旁报、逐枚点名、批末并报枚数。
# 断言（★逐枚并报、任一红档不合＝本件 rc=5★）＝
#   D1 ★现行指针行在位且不老★＝`note/00_README.md` 的「现行指针」节里那行「日期现取＝YYYY-MM-DD」，距令日 ≤ N 天（缺省 21）
#   D2 ★下游不许硬编码现行版本★＝五支牙（只作用红档面）＝
#        A1 「现行」与「v数字」同句共现（双向，＝拿版本号当★现行断言★用）
#        A2 「21 号…v数字」同句共现（★不要求带「按」★＝「与 21 号 v4 的」这一形是真人点出来的漏网形；★只管 21 号＝中心计划书那枚书目号★＝23 号队列与 v6 目录同句是在册许可形）
#        A3 「计划书v数字/<路径>」而★盘上该路径不存在★＝悬空版本路径（换代后必然长出来的形状）
#        A4 「计划书v数字/<路径>」盘上存在、但同句带★断言词★（现行｜真身｜依据｜为准｜生效｜作用于）＝用现存路径断言现行效力
#        A5 「（中心）计划书…真身／唯一真身…v数字」＝★把「真身」这顶帽子扣死在某一个版本号上★（目录树注行就是这一形）
#      ★现存路径的纯书目引用不记红＝进旁报★（23 号队列的真身此刻就在 `note/计划书v6/`＝在册许可，不是漂）
#   D3 ★技能包不许转录门值★＝扫 `tooling/skill/**/*.md` 里 OID_DIST／OCC_TOL／DEV_MAX／DT_MAX／`0\.40`／`0\.10`／`0\.35`／`≥1200` ⇒ 命中即点名
#   D4 ★「终稿」必须带层★＝含「终稿」的行同句须带层词（路线｜判据｜§6.4｜范围化｜非终稿｜带层｜口径）之一；红档面记红、笔权与历史面记旁报
#   D5 ★消费者在位★＝`_batch_readout.sh` 必须真调用本件（★新件要被下游真实调用一次＝追加一百一十二⑩★），否则＝本枚件是死件＝红
#   D6 ★清单＝派生物、漂了必须被机器看见★＝逐枚对表 `tooling/_sha256_manifest.txt`（顶层）＋`tooling/skill/_sha256_manifest.txt`（子）
#      的每一条「sha ↔ 盘上现值」⇒ 不合即点名在册值与现读值两枚、并给修法＝★重生成（`tooling/_build_tooling_manifests.sh`）、不是手改★
#      起意＝本批现读抓到★顶层清单对 README.md 那枚 sha 从 批174·补二（6a01532）起就漂了★（在册 85d8c47e ‖ 现读 9edf5a7a）＝
#      同批只动了 README、没同批刷顶层＝在册规矩 4 的一次漏执行、十四枚在册面里没有任何格子盯它
# 许可括注（★全行豁免★）＝行内含 现读＝／写就时／指针／归档／旧版／不再更新／历史／取代／一字未动／不写死／不硬编码／不得读成
#   ⇒ 那一枚版本号是★快照注记或历史叙述★，不是硬编码引用＝真人 2026-10-21 的指针形判据正是「门面现读为准（本稿写就时＝v7）」这一形。
# 用法＝bash _governance_drift_check.sh [--max-age N] [--root DIR] [--BR_DRIFT]
#   --BR_DRIFT＝★自测面（夹具只在 /tmp、跑完自报已删）★＝六枚坏样本（硬编码现行断言／书目号同句钉版本／真身钉版本／转录门值／终稿不带层／悬空版本路径）必须全被抓进红档
#              ＋三枚好样本（指针形括注、现存版本路径的书目引用、版本并列简写 v3/v4）必须不被记红，
#              另三枚分流对照（现存路径引用★必须落旁报★、门面历史行★必须落旁报不落红★、简写★不进任何档★）；任一不合＝本件自带缺陷⇒ rc=5 并点名
# 退出码＝0（红档全过）‖ 5（红档任一不合）‖ 9（前置缺失＝被扫面不在位）
set -u
ROOT=$(cd "$(dirname "$0")/../../../.." && pwd)
MAXAGE=21; BRK=0; TARGET="$ROOT"
while [ $# -gt 0 ]; do
  case "$1" in
    --max-age) MAXAGE="$2"; shift 2 ;;
    --root) TARGET="$2"; shift 2 ;;
    --BR_DRIFT) BRK=1; shift ;;
    *) echo "未知参数 $1"; exit 9 ;;
  esac
done
[ -d "$TARGET" ] || { echo "★前置缺失＝被扫面不在位★ $TARGET"; exit 9; }
SELF=$(cd "$(dirname "$0")" && pwd)/_batch_readout.sh

if [ "$BRK" = 1 ]; then
  T=$(mktemp -d /tmp/gov_drift.XXXXXX)
  mkdir -p "$T/note/计划书v6" "$T/tooling/skill/x"
  printf -- '## ★现行指针（夹具）★\n- 现行中心计划书＝以本门面现读为准（本稿写就时＝v7）；说「终稿」必须带层（路线｜判据｜§6.4）\n  ★日期现取＝%s★\n## 历史行\n- 一处旧登记＝已把本条列入终稿前置\n' "$(date +%F)" > "$T/note/00_README.md"
  : > "$T/note/计划书v6/23_v6任务队列.md"
  printf -- '- 计划书v4/：21 号计划书 v4 原文（**现行施工蓝本**）\n' > "$T/tooling/skill/x/bad_hard.md"
  printf -- '- 门值 OID_DIST 0.40 ‖ OCC_TOL 0.10（★把数字抄进技能包了★）\n' > "$T/tooling/skill/x/bad_gate.md"
  printf -- '- 本件已定为终稿\n' > "$T/tooling/skill/x/bad_final.md"
  printf -- '- 施工蓝本见 `note/计划书v9/21_中心计划书-v9.md`（★换代后的悬空路径★）\n' > "$T/tooling/skill/x/bad_dangling.md"
  printf -- '├── 计划书v5/                # **计划书唯一真身＝21_中心计划书-v5.3.md**（1157 行）\n' > "$T/tooling/skill/x/bad_pin.md"
  printf -- '你是小张。你把 20 号的模块化架构与 21 号 v4 的目录设计落成产出。\n' > "$T/tooling/skill/x/bad_21hao.md"
  printf -- '- 现行中心计划书＝★指针形★以 `note/00_README.md` 门面现读为准（本稿写就时＝v7），本包不硬编码版本号\n' > "$T/tooling/skill/x/good_ptr.md"
  printf -- '- 任务队列＝`note/计划书v6/23_v6任务队列.md`（★现存路径、纯书目引用★）\n' > "$T/tooling/skill/x/ok_exist.md"
  printf -- '- 三条硬规则（来源：21 号计划书 v3/v4 §3.2/§3.3）\n' > "$T/tooling/skill/x/ok_shorthand.md"
  printf -- '- ★D6 用★一枚在册 sha 故意写错的件\n' > "$T/tooling/skill/x/d6_bad.md"
  GS=$(cd "$T/tooling/skill" && sha256sum x/good_ptr.md | cut -d' ' -f1)
  TS=$(cd "$T/tooling" && sha256sum skill/x/good_ptr.md | cut -d' ' -f1)
  printf -- '%s  x/good_ptr.md\n0000000000000000000000000000000000000000000000000000000000000000  x/d6_bad.md\n' "$GS" > "$T/tooling/skill/_sha256_manifest.txt"
  printf -- '# 顶层清单（夹具）\n%s  skill/x/good_ptr.md\n' "$TS" > "$T/tooling/_sha256_manifest.txt"
  OUT=$(bash "$0" --root "$T" --max-age 21 2>&1); RC=$?
  RED=$(printf '%s\n' "$OUT" | grep '\[红\]')
  SOFT=$(printf '%s\n' "$OUT" | grep '\[旁\]')
  C1=$(printf '%s\n' "$RED" | grep -c bad_hard);  C2=$(printf '%s\n' "$RED" | grep -c bad_gate)
  C3=$(printf '%s\n' "$RED" | grep -c bad_final); C4=$(printf '%s\n' "$RED" | grep -c good_ptr)
  C5=$(printf '%s\n' "$RED" | grep -c ok_exist);  C6=$(printf '%s\n' "$SOFT" | grep -c ok_exist)
  C7=$(printf '%s\n' "$RED" | grep -c bad_dangling)
  C8=$(printf '%s\n' "$RED" | grep -c '终稿前置'); C9=$(printf '%s\n' "$SOFT" | grep -c '终稿前置')
  C10=$(printf '%s\n' "$RED" | grep -c ok_shorthand); C11=$(printf '%s\n' "$SOFT" | grep -c ok_shorthand)
  C12=$(printf '%s\n' "$RED" | grep -c bad_pin); C13=$(printf '%s\n' "$RED" | grep -c bad_21hao)
  C14=$(printf '%s\n' "$RED" | grep -c 'sha 不合.*d6_bad'); C15=$(printf '%s\n' "$RED" | grep -c 'sha 不合.*good_ptr')
  echo "  阳性对照（红档抓＝期望≥1 ‖ 红档不抓＝期望★0★ ‖ 旁报抓＝期望≥1）"
  echo "    硬编码现行版本=$C1(≥1) ‖ 转录门值=$C2(≥1) ‖ 终稿不带层=$C3(≥1) ‖ 悬空版本路径=$C7(≥1)"
  echo "    真身钉版本=$C12(≥1) ‖ 书目号同句钉版本=$C13(≥1) ‖ D6 清单 sha 漂=$C14(≥1)且正条目误报=$C15(★0★)"
  echo "    指针形好样本误报=$C4(0) ‖ 现存路径引用误报=$C5(0)且落旁报=$C6(≥1)"
  echo "    版本并列简写（v3/v4）误报=$C10(★0★)且落旁报=$C11(0＝★简写本就不是路径、不进任何档★)"
  echo "    门面历史行：记红=$C8(★0★) ‖ 记旁报=$C9(≥1)"
  BAD=""
  [ "$C1" -ge 1 ] || BAD="$BAD ★A1 硬编码现行版本没抓到★"
  [ "$C2" -ge 1 ] || BAD="$BAD ★D3 转录门值没抓到★"
  [ "$C3" -ge 1 ] || BAD="$BAD ★D4 终稿不带层没抓到★"
  [ "$C7" -ge 1 ] || BAD="$BAD ★A3 悬空版本路径没抓到★"
  [ "$C12" -ge 1 ] || BAD="$BAD ★A5 真身钉在版本号上没抓到★"
  [ "$C13" -ge 1 ] || BAD="$BAD ★A2 书目号同句钉版本没抓到★"
  [ "$C14" -ge 1 ] || BAD="$BAD ★D6 清单 sha 漂了没抓到★"
  [ "$C15" = 0 ] || BAD="$BAD ★D6 把对得上的条目也报红＝对表式自己有错★"
  [ "$C4" = 0 ] || BAD="$BAD ★指针形好样本被误报＝豁免锚缺★"
  [ "$C5" = 0 ] || BAD="$BAD ★现存路径引用被记红＝会造永红闸★"
  [ "$C6" -ge 1 ] || BAD="$BAD ★现存路径引用没进旁报＝这一档根本没扫★"
  [ "$C10" = 0 ] || BAD="$BAD ★版本并列简写被当路径抓＝锚把非路径形也算进去了★"
  [ "$C8" = 0 ] || BAD="$BAD ★无写入通道的历史行被记红＝闸没有牙只能靠我改旧文★"
  [ "$C9" -ge 1 ] || BAD="$BAD ★历史行没进旁报＝漂了没人知道★"
  rm -rf "$T"
  echo "  夹具已删＝$([ -d $T ] && echo 在 || echo 无)"
  if [ -n "$BAD" ]; then echo "★自测不合期望★＝$BAD"; echo "本件（自测面）rc=5"; exit 5; fi
  echo "  自测合期望＝★十四枚期望判定全合（红档抓 7 ‖ 不误报 4 ‖ 两档分流 3）★"
  echo "本件（自测面）rc=0"; exit 0
fi

python3 - "$TARGET" "$MAXAGE" "$SELF" <<'PYEOF'
import io, os, re, sys, datetime
ROOT, MAXAGE, READOUT = sys.argv[1], int(sys.argv[2]), sys.argv[3]
D = os.path.join(ROOT, "note", "00_README.md")
red, soft = [], []   # ★先定义＝read() 要往里记红档★
def read(p):
    # ★★本件自曝的牙根修★＝第一版夹具缺 README.md 时本件★直接崩（traceback、rc=1）★＝
    #   正是我在册 S34／S27 那族「查不到只有红档算被检到、崩与空值都不算」★我自己又犯了一次★
    #   正形＝取不到 ⇒ 记红档并点名、★绝不 traceback★
    try:
        return io.open(p, encoding="utf-8", errors="replace").read()
    except Exception as e:
        red.append("D0 读不到＝%s（%s）" % (p, type(e).__name__))
        return ""
if not os.path.isfile(D):
    print("★前置缺失＝门面件不在位★ %s" % D); sys.exit(9)
gate = [l for l in read(D).split("\n") if "现行指针" in l or "日期现取" in l]
print("== governance 漂移闸（★第十枚·只报不拦、不入门禁串★）==")
if not gate:
    red.append("D1 门面「现行指针」节不在位"); print("  D1 ★不过★＝门面里没有现行指针节")
else:
    m = re.search(r"日期现取＝(\d{4})-(\d{2})-(\d{2})", "\n".join(gate))
    if not m: red.append("D1 指针行没有日期枚"); print("  D1 ★不过★＝指针行缺「日期现取＝YYYY-MM-DD」")
    else:
        age = (datetime.date.today() - datetime.date(*[int(x) for x in m.groups()])).days
        print("  D1 指针行日期＝%s ‖ 距令日＝%d 天 ‖ 上限＝%d ⇒ %s" % (m.group(0).split("＝")[-1], age, MAXAGE, "过" if age <= MAXAGE else "★过期★"))
        if age > MAXAGE: red.append("D1 指针行过期（%d 天）" % age)
SNAP = re.compile(r"(现读[=＝:]|写就时|指针|归档|旧版|不再更新|历史|取代|一字未动|不写死|不硬编码|不得读成|来源[：: ])")
A1 = re.compile(r"(现行[^。]{0,16}v[0-9]|v[0-9][^。]{0,16}现行)")   # ★双向共现★＝「v4 原文（现行施工蓝本）」那种现行词在后头的形也要抓
A2 = re.compile(r"21 ?号[^。]{0,20}v[0-9]")                          # ★只管 21 号＝中心计划书那枚书目号★（★23 号队列与 v6 目录同句是在册许可形、不该被抓★）
A5 = re.compile(r"(计划书|中心计划书)[^。]{0,14}(唯一真身|真身)[^。]{0,30}v[0-9]")   # ★A5＝把「真身」这顶帽子扣在某个版本号上★（★树注行「计划书v5/ # 唯一真身＝…v5.3.md」就是这一形★）
A34 = re.compile(r"计划书\s*v([0-9]+)/([^v\s`|）)、，；:：][^\s`|）)、，；:：]*)")   # ★斜杠后一位不许是 v★＝「计划书 v3/v4」是★版本并列简写★不是路径
ASSERT = re.compile(r"(现行|真身|依据|为准|生效|作用于)")
GATE = re.compile(r"(OID_DIST|OCC_TOL|DEV_MAX|DT_MAX|0\.40|0\.10|0\.35|≥\s*1200)")
FIN = re.compile(r"终稿")
LAYER = re.compile(r"(路线|判据|§6\.4|范围化|S0—S4|限.*层|不是终稿|非终稿|带层|口径)")
n2r = n2s = n3 = n4r = n4s = 0
def snip(l): return l.strip().replace("\r", "")[:40]   # ★点名必带句＝只给 文件:行 会让自测与下游无法核对被咬的是哪一句★
def add(face, code, msg):
    """★两态落档★＝红档面记 red（计 rc=5）、无通道面记 soft（只点名）＝两档都★逐枚扫、一档不落空★"""
    global n2r, n2s, n3, n4r, n4s
    if code == "D2":
        if face == "red": n2r += 1; red.append("D2 " + msg)
        else: n2s += 1; soft.append("D2旁 " + msg)
    elif code == "D4":
        if face == "red": n4r += 1; red.append("D4 " + msg)
        else: n4s += 1; soft.append("D4旁 " + msg)
    else:
        n3 += 1; red.append(code + " " + msg)
def face_of(rel, line_no):
    """★两态分面★＝返回 'red'／'soft'＝本行此刻施工侧有没有写入通道。"""
    if rel.startswith("tooling/skill") or rel == "README.md": return "red"
    if rel == os.path.join("note", "00_README.md"):
        return "red" if line_no in PTR_LINES else "soft"
    return "soft"          # 计划书正文＝官方表述笔权面
_ptr = read(D).split("\n")
PTR_LINES, _mode = set(), False
for _i, _l in enumerate(_ptr, 1):
    if re.match(r"^#{1,4} ", _l): _mode = "现行指针" in _l   # ★节边界＝标题行到下一个标题行★
    if _mode: PTR_LINES.add(_i)
files = []
for root, dirs, fs in os.walk(os.path.join(ROOT, "tooling", "skill")):
    for f in fs:
        if f.endswith(".md"): files.append(os.path.join(root, f))
files.append(os.path.join(ROOT, "README.md"))   # ★缺面不再崩＝由 read() 记红档★
for p in [D] + files:
    rel = os.path.relpath(p, ROOT)
    for i, l in enumerate(read(p).split("\n"), 1):
        f = face_of(rel, i)
        if SNAP.search(l): continue
        hit = None
        if f == "red":
            if A1.search(l): hit = "A1 硬编码现行断言 ‖ 句＝%s" % snip(l)
            elif A2.search(l): hit = "A2 书目号同句钉版本 ‖ 句＝%s" % snip(l)
            elif A5.search(l): hit = "A5 把「真身」钉在版本号上 ‖ 句＝%s" % snip(l)
            else:
                for m in A34.finditer(l):
                    q = os.path.join(ROOT, "note", m.group(0))
                    if not os.path.exists(q): hit = "A3 悬空版本路径＝%s ‖ 句＝%s" % (m.group(0), snip(l)); break
                    if ASSERT.search(l): hit = "A4 以现存路径断言现行效力＝%s ‖ 句＝%s" % (m.group(0), snip(l)); break
                if hit is None and A34.search(l):
                    add("soft", "D2", "%s:%d 现存版本路径的书目引用（★不记红＝真身此刻就在该目录★）‖ 句＝%s" % (rel, i, snip(l)))
            if hit: add(f, "D2", "%s:%d %s" % (rel, i, hit))
        else:
            if A1.search(l) or A2.search(l) or A5.search(l): add(f, "D2", "%s:%d 硬编码现行引用（★无写入通道面＝旁报★）‖ 句＝%s" % (rel, i, snip(l)))
            else:
                for m in A34.finditer(l):
                    if not os.path.exists(os.path.join(ROOT, "note", m.group(0))):
                        add(f, "D2", "%s:%d 悬空版本路径＝%s（★旁报★）‖ 句＝%s" % (rel, i, m.group(0), snip(l))); break
        if f == "red" and rel.startswith("tooling/skill") and GATE.search(l):
            add("red", "D3", "%s:%d 转录门值 ‖ 句＝%s" % (rel, i, snip(l)))
        if FIN.search(l) and not LAYER.search(l):
            add(f, "D4", "%s:%d 「终稿」未带层 ‖ 句＝%s" % (rel, i, snip(l)))
print("  D2 硬编码现行引用＝红档 %d 枚 ‖ 旁报 %d 枚 ‖ D3 技能包转录门值＝%d 枚 ‖ D4 「终稿」不带层＝红档 %d ‖ 旁报 %d 枚" % (n2r, n2s, n3, n4r, n4s))
print("  ★旁报面＝计划书正文（笔权）＋门面历史行（不回改）＋现存版本路径引用 ⇒ 只点名、不记红、不改任何在册件的 rc★")
d5 = 0
if os.path.isfile(READOUT):
    d5 = read(READOUT).count("_governance_drift_check.sh")
print("  D5 消费者＝_batch_readout.sh 里引用本件枚数＝%d ⇒ %s" % (d5, "在位" if d5 >= 1 else "★死件（没有下游调用）★"))
if d5 < 1: red.append("D5 本件无消费者")
# ★D6＝清单是派生物、漂了必须被机器看见★（本批跑证＝顶层清单对 README.md 那枚 sha 在★批174·补二★之后就漂了、直到今天现读才抓到）
import hashlib
def sha_of(p):
    h = hashlib.sha256()
    with open(p, "rb") as fh:
        for b in iter(lambda: fh.read(65536), b""): h.update(b)
    return h.hexdigest()
for mf, base, tag in ((os.path.join(ROOT, "tooling", "_sha256_manifest.txt"), os.path.join(ROOT, "tooling"), "D6a 顶层"),
                      (os.path.join(ROOT, "tooling", "skill", "_sha256_manifest.txt"), os.path.join(ROOT, "tooling", "skill"), "D6b 子清单")):
    if not os.path.isfile(mf): red.append("%s 清单不在位＝%s" % (tag, mf)); print("  %s ★不在位★" % tag); continue
    ents = re.findall(r"(?m)^([0-9a-f]{64})  (.+)$", read(mf))
    if not ents: red.append("%s 条目枚数＝0（★空清单＝假绿★）" % tag); print("  %s ★条目 0 枚★" % tag); continue
    bad = []
    for s, relp in ents:
        q = os.path.join(base, relp)
        if not os.path.isfile(q): bad.append("缺件 %s" % relp)
        elif sha_of(q) != s: bad.append("sha 不合 %s（在册 %s ‖ 现读 %s）" % (relp, s[:16], sha_of(q)[:16]))
    print("  %s 条目＝%d ‖ 不合＝%d ‖ 缺＝%d" % (tag, len(ents), len(bad), len([b for b in bad if b.startswith("缺件")])))
    for b in bad: red.append("%s %s ⇒ ★修法＝重生成（--build --by-order）、不是手改★" % (tag, b))
print("  ★红档枚数＝%d‖ 旁报枚数＝%d★（★只报不拦★）" % (len(red), len(soft)))
for _r in red: print("     [红] %s" % _r)     # ★逐枚全列＝截前 8 枚会让下游数不到真枚数（追加一百二十二③同族）★
for _s in soft: print("     [旁] %s" % _s)
sys.exit(5 if red else 0)
PYEOF
RC=$?
echo "本件 rc=$RC（0＝红档全过 ‖ 5＝红档任一名点 ‖ 9＝前置缺失）；★不进任何在册件的 rc、不入门禁串★"
exit $RC
