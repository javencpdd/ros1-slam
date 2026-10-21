#!/usr/bin/env bash
# _rqq_watch.sh —— ★firing 探测件（常驻、施工侧自有登记面件＝第九枚：★不入门禁串、不改任何在册件的 rc 语义★）
# 起意＝真人 2026-10-21 对 VAR 辖域勾「★不扩＋挂触发式★」：一旦出现 `rNNNq_` 族缺清单实例（哪怕一枚、哪怕施工侧自己犯的）
#        ⇒ 立即按 R355 同规格扩辖域、触发当批 04 登记 firing、★不再等句★；★探测义务交给施工侧★。
#        批175／181／182／183 四批这枚都是★手敲两行式子★＝姿势依赖人记（追加一百零六⑥「到签的守卫要自己写式子跑过才算行使」同族）
#        ⇒ 本件把它变成★能跑、会翻红★的东西。
# 丙案口径（★批181 按真人 2026-10-20「参考我的习惯自行勾选」自走、照登在册、真人可另起句改判★）＝
#   ★只有「正文齐而清单缺席★跨过一整批★」才算 firing★；★在途半成品不算★（否则判据面一开始写＝施工侧就自触发一次立法级动作）
#   正文齐 := 该目录内★非清单★件枚数 ≥ 3（判据面在册交付形＝3 枚正文＋清单）
#   跨过一整批 := 同一枚目录名★同时出现在★本条记录与★上一条 phase=post 记录★的「缺清单且正文齐」集合里
# 用法＝bash _rqq_watch.sh [--batch N] [--phase pre|post] [--RO] [--BR_WATCH]
#   ★--RO＝只读面（G15 的另一半）＝判据面（独立实例）跑本件时用＝只报、★不写状态面★、随句带时刻★★＝
#     状态面归★施工侧★写（一枚账本两个写手＝追加一百一十九那条同族）；非 --RO 的行为一字未改
#   ★G15＝firing 读数随句具名★＝「缺清单＝N」这枚数必须同句点名★哪几枚是判据面在途（正文未齐⇒不进集合）★、★哪几枚已正文齐而缺席★
#     ＝不具名时读者无法分辨「在途不算」与「谁漏了该红」＝一枚不带出处的数＝S33 那族
#   --BR_WATCH＝★自测面（正例＋反例各一组、夹具只在 /tmp）★＝
#     ①★阳性对照★＝造一枚「正文齐而清单缺席」的目录、★连跑两遍 phase=post★ ⇒ 第二遍必须 firing=YES（rc=5）＝证明「跨过一整批」这枚比对真有牙
#     ②★阴性对照★＝同一枚夹具把★正文齐★这层门加上（在途＝非清单件 2 枚）连跑两遍 ⇒ 必须★恒 NO★＝证明丙案那枚门真在起作用、不是见缺就红
#     ③★空集字面回归控★（★批188 加＝本批那枚在册缺陷的直接靶★）＝夹具内★零枚缺席★连跑两遍 ⇒ 第二遍打印的「上一次集合枚数」必须＝★0★（★旧版给 1＝把状态面的占位符「无」当成了成员★）且 rc=0
#     ★④G15 具名控★＝夹具内★同时有在途与正文齐缺席各一枚★ ⇒ 读数必须★随句点名★那枚在途、且「★具名★」句必须恰 1 枚
#     ★⑤G15 只读控★（★带对立面＝追加一百二十三②★）＝跑 --RO 前后状态面 sha16★必须同值★、只读句必须打；★同一条夹具再跑一次非 --RO⇒sha 必须变★（不变＝「只读」这句是空话）
#     ★五枚期望任一不合＝本件自己有缺陷 ⇒ 当场红（rc=5）并点名★
# 退出码＝0（无 firing＝含「首次行使无前值＝只报不判」）‖ 5（★firing★ 或 ★自测不合期望★）‖ 9（前置缺失＝被试 raw/ 不在位）
set -u
ROOT=$(cd "$(dirname "$0")/../../../.." && pwd)
RAW="${RQQ_RAW:-$ROOT/note/进度/SCAN集成/证据/log/raw}"
STATE="${RQQ_STATE:-$ROOT/note/进度/SCAN集成/证据/_rqq_watch.state.tsv}"
BATCH="?"; PHASE=pre; RO=0
while [ $# -gt 0 ]; do
  case "$1" in
    --batch) BATCH="$2"; shift 2 ;;
    --phase) PHASE="$2"; shift 2 ;;
    --RO) RO=1; shift ;;
    --BR_WATCH) BRW=1; shift ;;
    *) echo "未知参数 $1"; exit 9 ;;
  esac
done
[ -n "${BRW:-}" ] || BRW=0
[ -d "$RAW" ] || { echo "★前置缺失＝被试 raw/ 不在位★ $RAW"; exit 9; }

if [ "$BRW" = 1 ]; then
  echo "== ★自测面（夹具只在 /tmp、★不碰仓内 raw★）=="
  T=$(mktemp -d /tmp/rqq_watch_selftest.XXXXXX)
  mkdir -p "$T/raw" "$T/state_dir"
  F_OK="$T/raw/r900q_demo_正文齐而清单缺席"
  F_ING="$T/raw/r901q_demo_在途半成品"
  mkdir -p "$F_OK" "$F_ING"
  for i in 1 2 3; do : > "$F_OK/0${i}_件.txt"; done
  for i in 1 2; do : > "$F_ING/0${i}_件.txt"; done
  export RQQ_RAW="$T/raw" RQQ_STATE="$T/state_dir/w.tsv"
  bash "$0" --phase post --batch S1 >/dev/null 2>&1; A=$?
  bash "$0" --phase post --batch S2 >/dev/null 2>&1; B=$?
  echo "  阳性对照（同一枚正文齐缺席、连跑两批）＝第一遍 rc=$A（期望 0）‖ 第二遍 rc=$B（期望 ★5★）"
  rm -rf "$F_OK"
  for i in 1 2; do : > "$F_ING/0${i}_件.txt"; done
  bash "$0" --phase post --batch S3 >/dev/null 2>&1; C=$?
  bash "$0" --phase post --batch S4 >/dev/null 2>&1; D=$?
  echo "  阴性对照（在途＝正文不齐、连跑两批）＝第三遍 rc=$C ‖ 第四遍 rc=$D（两枚期望★均 0★＝丙案那枚门在起作用）"
  # ★第三控＝空集字面回归控★（＝批187 在册那枚「无」被当成员数的★直接靶★）＝夹具里★一枚缺清单目录都没有★、连跑两遍 post
  #   期望＝★第二遍打印的「上一条记录集合枚数」＝0★（旧版给 1）且 rc=0；★这控不过＝状态面读数仍在虚报★
  mkdir -p "$T/raw/r902q_demo_有清单"
  : > "$T/raw/r902q_demo_有清单/00_独立裁定.txt"; : > "$T/raw/r902q_demo_有清单/_sha256_manifest.txt"
  OUT2=$(bash "$0" --phase post --batch S5 2>&1); E5=$?
  N0=$(printf '%s\n' "$OUT2" | grep -oE '集合枚数＝[0-9]+' | grep -oE '[0-9]+' | head -1)
  echo "  空集字面回归控（★夹具内零枚缺席★、连跑两批）＝第五遍打印的上一次集合枚数＝[$N0]（期望★0★）‖ rc=$E5（期望★0★）"
  # ★第四控＝G15 具名控★＝夹具里★同时有一枚正文齐缺席（r900q 已删＝所以用另一枚）＋一枚在途★ ⇒ 读数里必须★随句点名★那枚在途目录
  rm -rf "$T/state_dir/w.tsv"; mkdir -p "$T/raw/r903q_demo_正文齐第二枚"; for i in 1 2 3; do : > "$T/raw/r903q_demo_正文齐第二枚/0${i}_件.txt"; done
  OUT4=$(bash "$0" --phase pre --batch S6 2>&1)
  NA=$(printf '%s\n' "$OUT4" | grep -c 'r901q_demo_在途半成品')       # ★在途那枚必须被★具名★
  NB=$(printf '%s\n' "$OUT4" | grep -c '★具名★＝')                     # 那一句本身必须在
  echo "  ★G15 具名控★＝读数里出现「r901q 在途」枚数＝$NA（期望★≥1★） ‖ 「★具名★」句枚数＝$NB（期望★1★）"
  # ★第五控＝G15 只读控★＝--RO 不许动状态面（对立面＝非 --RO 必须动）＝先取 sha、跑 --RO、再取 sha
  #   ★本控先把「正文齐而缺席」那枚夹具摘掉★＝让 rc 的期望能写死成★0★（第一版把「期望 0」印在★确实 firing★的夹具态上＝
  #   ★印的期望与实测态不符★＝在册「一枚数不带出处」的同族、当场改正＝★夹具态要先造到期望那一档、再印期望★）
  rm -rf "$T/raw/r903q_demo_正文齐第二枚"
  bash "$0" --phase post --batch S7 >/dev/null 2>&1
  S1=$(sha256sum "$T/state_dir/w.tsv" 2>/dev/null | cut -c1-16)
  OUT5=$(bash "$0" --phase post --batch S8 --RO 2>&1); RC5=$?
  S2=$(sha256sum "$T/state_dir/w.tsv" 2>/dev/null | cut -c1-16)
  S3v=$(printf '%s\n' "$OUT5" | grep -c '★只读面（--RO）＝不写状态面★')
  bash "$0" --phase post --batch S9 >/dev/null 2>&1
  S4=$(sha256sum "$T/state_dir/w.tsv" 2>/dev/null | cut -c1-16)
  echo "  ★G15 只读控★（★夹具态＝零枚正文齐缺席⇒rc 期望写死 0★）＝跑 --RO 前状态面 sha16＝[$S1] ‖ 跑后＝[$S2]（★两枚必须同值★） ‖ 只读句枚数＝$S3v（期望★1★） ‖ rc=$RC5（期望★0★）"
  echo "     ★对立面★＝同一条夹具再跑一次★非★ --RO ⇒ sha16 变为 [$S4]（★必须与 [$S1] 不同＝否则「只读」这句是空话★）"
  BAD=""
  [ "$A" = 0 ] || BAD="$BAD 阳性第一遍应0实=$A"
  [ "$B" = 5 ] || BAD="$BAD ★阳性第二遍应5实=$B＝「跨过一整批」这枚比对没牙★"
  [ "$C" = 0 ] && [ "$D" = 0 ] || BAD="$BAD ★阴性应恒0实=$C/$D＝见缺就红＝丙案的门没起作用★"
  [ "$N0" = "0" ] || BAD="$BAD ★空集字面「无」被当成一枚成员＝枚数实=[$N0]★"
  [ "$E5" = 0 ] || BAD="$BAD ★零枚缺席却红＝rc=$E5★"
  [ "$NA" -ge 1 ] || BAD="$BAD ★G15 具名控＝在途目录没被随句点名★"
  [ "$NB" = 1 ] || BAD="$BAD ★G15 具名句枚数＝$NB≠1★"
  [ "$S1" = "$S2" ] || BAD="$BAD ★G15 只读控＝--RO 却动了状态面（$S1→$S2）★"
  [ "$S3v" = 1 ] || BAD="$BAD ★G15 只读句没打＝枚数 $S3v★"
  [ "$RC5" = 0 ] || BAD="$BAD ★G15 只读控 rc 不合＝夹具已摘到零枚缺席、rc 应 0 实=$RC5★"
  [ "$S1" != "$S4" ] || BAD="$BAD ★对立面不合＝非 --RO 也没写状态面⇒「只读」那句是空话★"
  if [ -n "$BAD" ]; then echo "  ★自测不合期望★＝$BAD"; RC=5; else echo "  自测合期望＝★这枚件的牙已被五组对照跑证（阳性／阴性／空集字面回归／G15 具名／G15 只读带对立面）★"; RC=0; fi
  rm -rf "$T"
  echo "本件（自测面）rc=$RC"; exit $RC
fi

python3 - "$RAW" "$STATE" "$BATCH" "$PHASE" "$RO" <<'PYEOF'
import io, os, re, sys, datetime

RAW, STATE, BATCH, PHASE, RO = sys.argv[1:6]
MIN_CONTENT = 3
EMPTY = "无"   # ★状态面空集占位符★＝★只作字面、不作成员★（批187 在册缺陷＝旧版把它 split 成 1 枚成员⇒枚数读数系统性 +1；firing 判定本身未坏＝本集合取自盘上实扫）
print("== firing 探测（★丙案口径·常驻第九枚·只报不拦、不入门禁串★）==")
rows = []
for name in sorted(os.listdir(RAW)):
    p = os.path.join(RAW, name)
    if not (os.path.isdir(p) and re.match(r"^r\d+q_", name)):
        continue
    files = [f for f in os.listdir(p) if os.path.isfile(os.path.join(p, f))]
    has_mf = "_sha256_manifest.txt" in files
    n_content = len([f for f in files if f != "_sha256_manifest.txt"])
    rows.append((name, has_mf, n_content))
missing = [(n, c) for n, hm, c in rows if not hm]
ready = [n for n, c in missing if c >= MIN_CONTENT]
inflight = [(n, c) for n, c in missing if c < MIN_CONTENT]
print("  该族目录枚数＝%d ‖ 缺清单＝%d ‖ 其中★正文齐（非清单件≥%d）★＝%d" % (len(rows), len(missing), MIN_CONTENT, len(ready)))
# ★G15（判据面 r534q 问⑦③给的处置）★＝firing 读数★随句具名★＝把「判据面从建目录到建清单之间那段时间」★点名★出来、
#   否则「缺清单＝1」这枚数读者无法分辨＝判据面在途（不算）还是谁漏了（该红）＝★一枚数不带出处＝另一枚 S33 型缺陷★
print("  ★具名★＝判据面★在途★目录（正文未齐⇒★不进 firing 集合★）＝%s ‖ 判据面★正文齐而清单缺席★＝%s" % (
    " ‖ ".join("%s(非清单件%d)" % (n, c) for n, c in inflight) or "无",
    " ‖ ".join(ready) or "无"))
for n, c in missing:
    print("     缺清单面＝%s ‖ 非清单件=%d ‖ 正文齐=%s" % (n, c, "是" if c >= MIN_CONTENT else "★否（在途＝不计）"))

prev, prev_batch = None, "?"
if os.path.exists(STATE):
    for l in io.open(STATE, encoding="utf-8").read().split("\n"):
        if not l.strip() or l.startswith("#"):
            continue
        f = l.split("\t")
        if len(f) >= 4 and f[1] == "post":
            prev_batch = f[2]
            prev = set(x for x in f[3].split(",") if x and x != EMPTY)
    if prev is not None:
        print("  上一条 phase=post 记录＝批%s ‖ 其「正文齐而缺清单」集合枚数＝%d（★已滤空集字面「%s」＝批187 在册缺陷的修★）" % (prev_batch, len(prev), EMPTY))
cross = sorted(set(ready) & (prev if prev is not None else set()))
if prev is None:
    print("  ★首次行使＝无前值 ⇒「跨过一整批」不可判 ⇒ 只报不判★（★这不等于「无缺」★）")
    FIRE = False
else:
    FIRE = bool(cross)
print("  firing＝%s ‖ 同一枚跨过一整批仍缺＝%s" % ("★YES★" if FIRE else "NO", cross or "无"))

ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
hdr = ("# _rqq_watch.state.tsv —— ★丙案口径的状态面★（每行＝时刻\tphase\t批号\t该批「正文齐而缺清单」目录名集合，逗号分隔或「无」）\n"
       "# ★由 _rqq_watch.sh 本体写＝不是手抄★；★「跨过一整批」＝同一枚出现在本条与上一条 phase=post★；★在途（正文未齐）不进集合★\n")
lines = []
if os.path.exists(STATE):
    lines = [l for l in io.open(STATE, encoding="utf-8").read().split("\n") if l.strip() and not l.startswith("#")]
if RO == "1":
    # ★G15 的另一半＝状态面★由施工侧写★：判据面（独立实例）跑本件时只读、并随句带★时刻★＝它不替施工侧落账、也不污染账本
    print("  ★只读面（--RO）＝不写状态面★（状态面由施工侧写 ‖ 判据面只读并带时刻＝%s ‖ 此刻集合枚数=%d）" % (ts, len(ready)))
else:
    lines.append("\t".join([ts, PHASE, str(BATCH), ",".join(ready) if ready else "无"]))
    io.open(STATE, "w", encoding="utf-8").write(hdr + "\n".join(lines) + "\n")
    print("  状态面已落一行＝phase=%s ‖ 集合枚数=%d" % (PHASE, len(ready)))
RC = 5 if FIRE else 0
print("本件 rc=%s（0＝无 firing‖5＝★firing＝按 R355 同规格扩辖域、触发当批 04 登记、不再等句★‖9＝前置缺失）" % RC)
sys.exit(RC)
PYEOF
