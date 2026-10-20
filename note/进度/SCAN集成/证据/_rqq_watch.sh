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
# 用法＝bash _rqq_watch.sh [--batch N] [--phase pre|post] [--BR_WATCH]
#   --BR_WATCH＝★自测面（正例＋反例各一组、夹具只在 /tmp）★＝
#     ①★阳性对照★＝造一枚「正文齐而清单缺席」的目录、★连跑两遍 phase=post★ ⇒ 第二遍必须 firing=YES（rc=5）＝证明「跨过一整批」这枚比对真有牙
#     ②★阴性对照★＝同一枚夹具把★正文齐★这层门加上（在途＝非清单件 2 枚）连跑两遍 ⇒ 必须★恒 NO★＝证明丙案那枚门真在起作用、不是见缺就红
#     ★两枚期望任一不合＝本件自己有缺陷 ⇒ 当场红（rc=5）并点名★
# 退出码＝0（无 firing＝含「首次行使无前值＝只报不判」）‖ 5（★firing★ 或 ★自测不合期望★）‖ 9（前置缺失＝被试 raw/ 不在位）
set -u
ROOT=$(cd "$(dirname "$0")/../../../.." && pwd)
RAW="${RQQ_RAW:-$ROOT/note/进度/SCAN集成/证据/log/raw}"
STATE="${RQQ_STATE:-$ROOT/note/进度/SCAN集成/证据/_rqq_watch.state.tsv}"
BATCH="?"; PHASE=pre
while [ $# -gt 0 ]; do
  case "$1" in
    --batch) BATCH="$2"; shift 2 ;;
    --phase) PHASE="$2"; shift 2 ;;
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
  BAD=""
  [ "$A" = 0 ] || BAD="$BAD 阳性第一遍应0实=$A"
  [ "$B" = 5 ] || BAD="$BAD ★阳性第二遍应5实=$B＝「跨过一整批」这枚比对没牙★"
  [ "$C" = 0 ] && [ "$D" = 0 ] || BAD="$BAD ★阴性应恒0实=$C/$D＝见缺就红＝丙案的门没起作用★"
  if [ -n "$BAD" ]; then echo "  ★自测不合期望★＝$BAD"; RC=5; else echo "  自测合期望＝★这枚件的牙已被两组对照跑证★"; RC=0; fi
  rm -rf "$T"
  echo "本件（自测面）rc=$RC"; exit $RC
fi

python3 - "$RAW" "$STATE" "$BATCH" "$PHASE" <<'PYEOF'
import io, os, re, sys, datetime

RAW, STATE, BATCH, PHASE = sys.argv[1:5]
MIN_CONTENT = 3
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
print("  该族目录枚数＝%d ‖ 缺清单＝%d ‖ 其中★正文齐（非清单件≥%d）★＝%d" % (len(rows), len(missing), MIN_CONTENT, len(ready)))
for n, c in missing:
    print("     缺清单面＝%s ‖ 非清单件=%d ‖ 正文齐=%s" % (n, c, "是" if c >= MIN_CONTENT else "★否（在途＝不计）"))

prev, prev_batch = None, "?"
if os.path.exists(STATE):
    for l in io.open(STATE, encoding="utf-8").read().split("\n"):
        if not l.strip() or l.startswith("#"):
            continue
        f = l.split("\t")
        if len(f) >= 4 and f[1] == "post":
            prev_batch, prev = f[2], set(x for x in f[3].split(",") if x)
    if prev is not None:
        print("  上一条 phase=post 记录＝批%s ‖ 其「正文齐而缺清单」集合枚数＝%d" % (prev_batch, len(prev)))
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
lines.append("\t".join([ts, PHASE, str(BATCH), ",".join(ready) if ready else "无"]))
io.open(STATE, "w", encoding="utf-8").write(hdr + "\n".join(lines) + "\n")
print("  状态面已落一行＝phase=%s ‖ 集合枚数=%d" % (PHASE, len(ready)))
RC = 5 if FIRE else 0
print("本件 rc=%s（0＝无 firing‖5＝★firing＝按 R355 同规格扩辖域、触发当批 04 登记、不再等句★‖9＝前置缺失）" % RC)
sys.exit(RC)
PYEOF
