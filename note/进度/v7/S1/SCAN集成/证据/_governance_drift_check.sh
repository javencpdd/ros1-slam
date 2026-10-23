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
#      ★这支牙不受全行豁免管（批200 落 r541q 反例 C1＝F3）★＝旧形把豁免写在三支牙之前、整行 continue ⇒
#      一行「来源：v7 §8」起头＋同句抄满四枚门值＝★门值转录那支牙在同一行里全部失效★；而★带出处恰是技能包抄数字最自然的写法★。
#      在册规矩乙那句「判据／门值只引用索引、一个数字都不抄」★没有例外形★＝所以豁免词在这里不能当通行证。
#   D4 ★「终稿」必须带层★＝含「终稿」的行同句须带层词（路线｜判据｜§6.4｜范围化｜非终稿｜带层｜口径）之一；红档面记红、笔权与历史面记旁报
#   D5 ★消费者在位★＝`_batch_readout.sh` 必须真调用本件（★新件要被下游真实调用一次＝追加一百一十二⑩★），否则＝本枚件是死件＝红
#   D6 ★清单＝派生物、漂了必须被机器看见★＝逐枚对表 `tooling/_sha256_manifest.txt`（顶层）＋`tooling/skill/_sha256_manifest.txt`（子）
#      的每一条「sha ↔ 盘上现值」⇒ 不合即点名在册值与现读值两枚、并给修法＝★重生成（`tooling/_build_tooling_manifests.sh`）、不是手改★
#      起意＝本批现读抓到★顶层清单对 README.md 那枚 sha 从 批174·补二（6a01532）起就漂了★（在册 85d8c47e ‖ 现读 9edf5a7a）＝
#      同批只动了 README、没同批刷顶层＝在册规矩 4 的一次漏执行、十四枚在册面里没有任何格子盯它
#   D7 ★件内路径在盘性★（★R603／批255 新增★）＝扫三档件名册里的★仓内相对路径字面★、逐枚问「盘上有没有」
#      起意＝两枚同族缺陷先后由人眼抓到＝批244（VCC 的 glob 仍写搬迁前路径＝★一面失明 35 批★）‖ 批254（高程合同件第 146 行同样指旧路径、
#      且该件 ROOT 写死容器路径⇒本机恒不可跑＝★W8 段从未执行★）＝在册只有「搬家后要回头扫一遍取路径的件」这句★程序性话★、没有格子盯；
#      ★本支牙盯的是状态（路径在不在盘）、不是程序（有没有人扫过）＝程序那半按追加一百八十第一节挂触发式、不假装能机判★
#      名册＝★派生式、不写死枚数★（＝追加一百八十第一节的第二次行使）＝自有件（红档）‖ 判据件 scripts/verify（旁报＝动它须先裁）‖ 包内 ws/**/verify（旁报＝红线面）
#      三枚排除（★逐档报枚数、不静默★）＝变量拼接（前一位是 $ ％）‖ 夹具行（含 夹具｜mktemp｜正控｜反控）‖ 绝对面（本支只管相对面）
# 许可括注（★全行豁免★）＝行内含 现读＝／写就时／指针／归档／旧版／不再更新／历史／取代／一字未动／不写死／不硬编码／不得读成
#   ⇒ 那一枚版本号是★快照注记或历史叙述★，不是硬编码引用＝真人 2026-10-21 的指针形判据正是「门面现读为准（本稿写就时＝v7）」这一形。
# 用法＝bash _governance_drift_check.sh [--max-age N] [--root DIR] [--BR_DRIFT] [--settlement]
#       --settlement＝问二收口三式（R606／批258 裁、批260 落件）：只报不拦、不立豁免表
#   --BR_DRIFT＝★自测面（夹具只在 /tmp、跑完自报已删）★＝七枚坏样本（硬编码现行断言／书目号同句钉版本／真身钉版本／转录门值／★带出处的门值行＝豁免不能吃掉门值牙★／终稿不带层／悬空版本路径）必须全被抓进红档
#              ＋三枚好样本（指针形括注、现存版本路径的书目引用、版本并列简写 v3/v4）必须不被记红，
#              另三枚分流对照（现存路径引用★必须落旁报★、门面历史行★必须落旁报不落红★、简写★不进任何档★）；任一不合＝本件自带缺陷⇒ rc=5 并点名
# 退出码＝0（红档全过）‖ 5（红档任一不合）‖ 9（前置缺失＝被扫面不在位）
set -u
# ★R589／批242 修 ROOT★＝本件随 SCAN集成 于批208 搬入 note/进度/v7/S1/… ⇒ 实际深 6 级，原式四跳只到 note/进度
#   ⇒ 该件自批208 起★恒报「前置缺失＝不在位／不是 git 仓」★，批末读数面四枚附面同步恒报「件不在位」（本批现跑抓到）
ROOT=$(git rev-parse --show-toplevel 2>/dev/null) || ROOT=$(cd "$(dirname "$0")/../../../../.." && pwd)
MAXAGE=21; BRK=0; SETTLE=0; TARGET="$ROOT"
while [ $# -gt 0 ]; do
  case "$1" in
    --max-age) MAXAGE="$2"; shift 2 ;;
    --root) TARGET="$2"; shift 2 ;;
    --BR_DRIFT) BRK=1; shift ;;
    --settlement) SETTLE=1; shift ;;
    *) echo "未知参数 $1"; exit 9 ;;
  esac
done
[ -d "$TARGET" ] || { echo "★前置缺失＝被扫面不在位★ $TARGET"; exit 9; }
if [ "$SETTLE" = 1 ]; then
  ST=$(mktemp -d "${TMPDIR:-/tmp}/gov_settle.XXXXXX")
  SARGS=""
  [ -n "$MAXAGE" ] && SARGS="$SARGS --max-age $MAXAGE"
  bash "$0" $SARGS --root "$TARGET" > "$ST/out.txt" 2>&1
  SRC=$?
  S1=$(grep -c '^     \[旁\] D7旁 ws/' "$ST/out.txt")
  S2=$(grep -c '^     \[旁\] D7旁 scripts/' "$ST/out.txt")
  S3=$(grep -o '抽得相对路径 [0-9]*' "$ST/out.txt" | grep -o '[0-9]*$' | head -1)
  echo "== 问二收口三式（R606／批258 裁、批260 落件；★只报不拦、不立豁免表★） =="
  echo "  一式＝ws 档旁报＝$S1（期望 2＝丁档两枚按问五永留旁报；甲乙丙八枚改指后归零）"
  echo "  二式＝scripts 档旁报＝$S2（期望 10＝运行时造面、不许降；降了＝有人往排除档塞东西）"
  echo "  三式＝抽得相对路径总数＝$S3（★只增不减★；减＝有人删引用；增须能归因到加指针）"
  echo "  主扫描 rc=$SRC（三式只报不拦，不改本件 rc 语义）"
  rm -rf "$ST"
  exit $SRC
fi

SELF=$(cd "$(dirname "$0")" && pwd)/_batch_readout.sh

if [ "$BRK" = 1 ]; then
  T=$(mktemp -d /tmp/gov_drift.XXXXXX)
  mkdir -p "$T/note/计划书v6" "$T/tooling/skill/x"
  printf -- '## ★现行指针（夹具）★\n- 现行中心计划书＝以本门面现读为准（本稿写就时＝v7）；说「终稿」必须带层（路线｜判据｜§6.4）\n  ★日期现取＝%s★\n## 历史行\n- 一处旧登记＝已把本条列入终稿前置\n' "$(date +%F)" > "$T/note/00_README.md"
  : > "$T/note/计划书v6/23_v6任务队列.md"
  printf -- '- 计划书v4/：21 号计划书 v4 原文（**现行施工蓝本**）\n' > "$T/tooling/skill/x/bad_hard.md"
  printf -- '- 门值 OID_DIST 0.40 ‖ OCC_TOL 0.10（★把数字抄进技能包了★）\n' > "$T/tooling/skill/x/bad_gate.md"
  printf -- '- 按出处转录门值：来源：v7 §8 的 OID_DIST 0.40 ‖ DT_MAX 120（★一行带出处＝旧形在这行被整行豁免吃掉★）\n' > "$T/tooling/skill/x/bad_gate_src.md"
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
  # ★D7 的三枚控（被试面内容一律不带「夹具／正控／反控／mktemp」字样＝否则被豁免锚吃掉＝★这本身就是这支牙的口径★）★
  mkdir -p "$T/note/进度/v7/S1/SCAN集成/证据" "$T/scripts/verify" "$T/note/进度/v7/S1/SCAN集成/设计"
  printf -- '# 被试面甲\n- 引用＝note/计划书v9/21_中心计划书-v9.md（换代后的旧形）\n' > "$T/note/进度/v7/S1/SCAN集成/证据/_d7_own_bad.sh"
  printf -- '# 被试面乙\n- 引用＝note/计划书v6/23_v6任务队列.md（盘上有）\n- 目录形＝note/进度/v7/S1/SCAN集成/设计/*.md（glob 取前缀）\n' > "$T/note/进度/v7/S1/SCAN集成/证据/_d7_own_ok.sh"
  printf -- '# 被试面丙\n- 引用＝note/进度/SCAN集成/证据/log/raw/r278_3d_release/21_xiaozhang_landing.txt\n' > "$T/scripts/verify/v_d7_crit.sh"
  OUT=$(bash "$0" --root "$T" --max-age 21 2>&1); RC=$?
  RED=$(printf '%s\n' "$OUT" | grep '\[红\]')
  SOFT=$(printf '%s\n' "$OUT" | grep '\[旁\]')
  C1=$(printf '%s\n' "$RED" | grep -c bad_hard);  C2=$(printf '%s\n' "$RED" | grep -c 'bad_gate\.md')
  C3=$(printf '%s\n' "$RED" | grep -c bad_final); C4=$(printf '%s\n' "$RED" | grep -c good_ptr)
  C5=$(printf '%s\n' "$RED" | grep -c ok_exist);  C6=$(printf '%s\n' "$SOFT" | grep -c ok_exist)
  C7=$(printf '%s\n' "$RED" | grep -c bad_dangling)
  C8=$(printf '%s\n' "$RED" | grep -c '终稿前置'); C9=$(printf '%s\n' "$SOFT" | grep -c '终稿前置')
  C10=$(printf '%s\n' "$RED" | grep -c ok_shorthand); C11=$(printf '%s\n' "$SOFT" | grep -c ok_shorthand)
  C12=$(printf '%s\n' "$RED" | grep -c bad_pin); C13=$(printf '%s\n' "$RED" | grep -c bad_21hao)
  C14=$(printf '%s\n' "$RED" | grep -c 'sha 不合.*d6_bad'); C15=$(printf '%s\n' "$RED" | grep -c 'sha 不合.*good_ptr')
  C16=$(printf '%s\n' "$RED" | grep -c bad_gate_src)
  C17=$(printf '%s\n' "$RED" | grep -c '_d7_own_bad'); C18=$(printf '%s\n' "$RED" | grep -c '_d7_own_ok')
  C19=$(printf '%s\n' "$SOFT" | grep -c 'v_d7_crit');  C20=$(printf '%s\n' "$RED" | grep -c 'v_d7_crit')
  echo "  阳性对照（红档抓＝期望≥1 ‖ 红档不抓＝期望★0★ ‖ 旁报抓＝期望≥1）"
  echo "    硬编码现行版本=$C1(≥1) ‖ 转录门值=$C2(≥1) ‖ 终稿不带层=$C3(≥1) ‖ 悬空版本路径=$C7(≥1)"
  echo "    ★带出处的门值行（旧形被整行豁免那一枚）＝$C16(≥1＝豁免不能当门值转录的通行证)★"
  echo "    真身钉版本=$C12(≥1) ‖ 书目号同句钉版本=$C13(≥1) ‖ D6 清单 sha 漂=$C14(≥1)且正条目误报=$C15(★0★)"
  echo "    指针形好样本误报=$C4(0) ‖ 现存路径引用误报=$C5(0)且落旁报=$C6(≥1)"
  echo "    版本并列简写（v3/v4）误报=$C10(★0★)且落旁报=$C11(0＝★简写本就不是路径、不进任何档★)"
  echo "    门面历史行：记红=$C8(★0★) ‖ 记旁报=$C9(≥1)"
  echo "    ★D7 件内路径在盘性＝自有件坏样本记红=$C17(≥1) ‖ 好样本（含 glob 取前缀）记红=$C18(★0★) ‖ 判据件坏样本记旁报=$C19(≥1)且不记红=$C20(★0★)★"
  BAD=""
  [ "$C1" -ge 1 ] || BAD="$BAD ★A1 硬编码现行版本没抓到★"
  [ "$C2" -ge 1 ] || BAD="$BAD ★D3 转录门值没抓到★"
  [ "$C16" -ge 1 ] || BAD="$BAD ★D3 被一行出处从句整行绕过＝豁免锚吃掉了门值牙★"
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
  [ "$C17" -ge 1 ] || BAD="$BAD ★D7 自有件里的不在盘路径没被抓＝这支牙根本没接上名册★"
  [ "$C18" = 0 ] || BAD="$BAD ★D7 把在盘路径或 glob 前缀报成不在盘＝会造永红闸★"
  [ "$C19" -ge 1 ] || BAD="$BAD ★D7 判据件的不在盘路径没进旁报＝失明面没人点名★"
  [ "$C20" = 0 ] || BAD="$BAD ★D7 把须先裁的判据件记成红＝越了施工侧的界★"
  rm -rf "$T"
  echo "  夹具已删＝$([ -d $T ] && echo 在 || echo 无)"
  if [ -n "$BAD" ]; then echo "★自测不合期望★＝$BAD"; echo "本件（自测面）rc=5"; exit 5; fi
  echo "  自测合期望＝★十九枚期望判定全合（红档抓 9 ‖ 不误报与不记红 6 ‖ 两档分流 4）★"
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
        if f == "red" and rel.startswith("tooling/skill") and GATE.search(l):
            # ★门值牙先于豁免★＝批200 落 r541q 反例 C1：旧形把 continue 写在它之前＝一行「来源：」就能把四枚门值整行抄进技能包
            add("red", "D3", "%s:%d 转录门值（★这支牙不受全行豁免★）‖ 句＝%s" % (rel, i, snip(l)))
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
# ★D7＝件内路径在盘性★（R603／批255）＝名册＝派生式（★不写死枚数★＝追加一百八十第一节第二次行使）；盯状态、不盯程序
D7_HEAD = re.compile(r"(?<![A-Za-z0-9_.\-])((?:note|scripts|tooling|ws|opt|data)/[^\s'\"`|（）()，,；;：:*<>＝、。\[\]^{}]+)")   # ★前一位允许斜杠＝$ROOT/note/… 这一形必须能判；容器面靠 D7_CONT 挡、不靠斜杠挡★
D7_TAIL = ".,;:-—…/」』】”’"          # ★句末标点与目录尾巴一律先剥掉再判＝否则「路径后面黏一个句号」这一形会假不在盘★
D7_CONT = ("opt/ros", "ws/devel", "opt/ros1-slam")   # ★容器面与编译产物＝本机按定义不存在＝不判、单报一枚数★
D7_JIAJU = re.compile(r"(夹具|mktemp|正控|反控|探针|BR_|\$T)")   # ★含 $T 那一行＝自测夹具的写出面＝路径按 $T 为根、不按仓根判★
def d7_roster(base, keep):
    out = []
    for root, dirs, fs in os.walk(base):
        dirs[:] = [x for x in dirs if x not in (".git", "__pycache__", "log", "run", "build", "devel", "node_modules")]
        for f in fs:
            p = os.path.join(root, f)
            if keep(os.path.relpath(p, ROOT), f): out.append(p)
    return sorted(out)
own_f = d7_roster(os.path.join(ROOT, "note"), lambda r, f: f.startswith("_") and f.endswith(".sh"))
crit_f = d7_roster(os.path.join(ROOT, "scripts"), lambda r, f: f.endswith(".sh") and os.sep + "verify" + os.sep in r)
pkg_f = d7_roster(os.path.join(ROOT, "ws"), lambda r, f: f.endswith(".sh") and os.sep + "verify" + os.sep in r)
d7_tot = d7_ok = d7_own_bad = d7_soft_bad = x_var = x_jiaju = x_cont = x_noext = x_egedu = 0
for p, tier in [(x, "own") for x in own_f] + [(x, "crit") for x in crit_f] + [(x, "pkg") for x in pkg_f]:
    rel = os.path.relpath(p, ROOT)
    for i, l in enumerate(read(p).split("\n"), 1):
        if D7_JIAJU.search(l):
            x_jiaju += len(D7_HEAD.findall(l)); continue
        for m in D7_HEAD.finditer(l):
            prev = l[m.start()-1] if m.start() else ""
            if prev in "$%": x_var += 1; continue
            tok = m.group(0).strip(D7_TAIL)
            if "*" in tok: tok = tok.split("*")[0].strip(D7_TAIL)   # ★glob＝取目录前缀再判★
            if len(tok.split("/")) < 2: continue
            if any(tok.startswith(c) for c in D7_CONT): x_cont += 1; continue   # ★容器面与产物＝不判★
            if "..." in tok or "…" in tok: x_egedu += 1; continue                 # ★省略号＝散文里的例句形＝不判★
            if not os.path.exists(os.path.join(ROOT, tok)) and "." not in os.path.basename(tok):
                x_noext += 1; continue                                          # ★无扩展名又不是目录＝散文里的书名片段＝不判（宁漏不误红）★
            d7_tot += 1
            if os.path.exists(os.path.join(ROOT, tok)): d7_ok += 1; continue
            if tier == "own":
                d7_own_bad += 1; red.append("D7 %s:%d 件内路径不在盘＝%s ‖ 句＝%s" % (rel, i, tok, snip(l)))
            else:
                d7_soft_bad += 1
                soft.append("D7旁 %s:%d 件内路径不在盘＝%s（★%s＝只点名★）‖ 句＝%s" % (
                    rel, i, tok, "判据件＝动它须先裁" if tier == "crit" else "ws 包内＝红线面", snip(l)))
print("  D7 名册（★派生式＝文件系统筛，与 05 追加一百八十那枚 git 索引面式子同职不同面★）＝自有件 %d ‖ 判据件 %d ‖ 包内 %d 枚件" % (len(own_f), len(crit_f), len(pkg_f)))
print("  D7 取数＝抽得相对路径 %d 枚 ‖ 在盘 %d ‖ ★不在盘 %d（自有件记红 %d ‖ 判据与包内记旁报 %d）★ ‖ 排除五档并报＝容器面与产物 %d ‖ 无扩展名散文形 %d ‖ 省略号例句形 %d ‖ 变量拼接 %d ‖ 夹具行 %d" % (
    d7_tot, d7_ok, d7_own_bad + d7_soft_bad, d7_own_bad, d7_soft_bad, x_cont, x_noext, x_egedu, x_var, x_jiaju))
print("  ★红档枚数＝%d‖ 旁报枚数＝%d★（★只报不拦★）" % (len(red), len(soft)))
for _r in red: print("     [红] %s" % _r)     # ★逐枚全列＝截前 8 枚会让下游数不到真枚数（追加一百二十二③同族）★
for _s in soft: print("     [旁] %s" % _s)
sys.exit(5 if red else 0)
PYEOF
RC=$?

echo "本件 rc=$RC（0＝红档全过 ‖ 5＝红档任一名点 ‖ 9＝前置缺失）；★不进任何在册件的 rc、不入门禁串★"
exit $RC
