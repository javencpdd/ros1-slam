#!/usr/bin/env bash
# _build_tooling_manifests.sh —— ★tooling 两面清单（子清单＋顶层）的法定生成件（施工侧自有登记面件，不入门禁串、不改任何在册件的 rc）★
# 起意＝真人 2026-10-21 明文处置「清单闸」那一格＝★闸防「手改」、不防「重建」⇒ 走法定生命周期★：
#   ①先写完全部 SKILL.md／角色件内容 ②★按生成序重建★ tooling/skill/_sha256_manifest.txt（生成≠编辑，新件落位即换代）
#   ③改动面若触到★顶层★清单覆盖集才同批刷顶层（本件★不碰★顶层清单＝只读现取它一次，用来验证排除面的出处）
#   ④04 段两数并报旧 sha→新 sha ＋当批留底报行
# 生成序（★写死在这一处、别处不许另造★）＝对 `tooling/skill/` 下全部★普通文件★（不含符号链接）取★相对面路径★，
#   以 `LC_ALL=C sort` 定序，逐件 `sha256sum`，输出「<sha>  <相对路径>」两列；★相对面＝清单可移植的唯一形★（绝对面落盘＝换机即漂）。
# 排除面名册（★两枚、各带出处的现取断言★）＝
#   `_sha256_manifest.txt` 自身＝清单不自录（在册铁律＝留底清单最后建、自己不录入）
#   `RESTORE.sh` ＝★由顶层 `tooling/_sha256_manifest.txt` 覆盖★⇒ 本件每次跑都要★现读顶层清单里那一行★★在不在位★，不在＝拒绝生成（rc=9）
#     ⇒ 理由＝"排除"必须是★有出处的移交★，不能是我自己少写一件＝追加一百二十二②「生成面每跑一次重播一遍」的同族
# 默认★拒写★＝没有 --by-order 明令旗一律 rc=9 退出（＝对已入库面的重建必须由明令触发，同 R534 断言③ 的 RERUN=1 形）
# 用法＝bash _build_tooling_manifests.sh [--target skill|top] --check | --build --by-order | --BR_NOFLAG | --BR_OK
#   --check＝只校验（逐条 sha256sum -c 面）＝不合⇒ rc=5 并逐枚点名（★这才是"改完还没重建"的正常红态★）
#   --build --by-order＝按上面的生成序★整体重写★清单（生成物、非手改）
#   --BR_NOFLAG＝★负例开关★＝不带明令旗跑 --build ⇒ 本件必须翻红（rc=9）；带旗的★正控★另由 --BR_OK 跑（生成完立刻自删到 /tmp 的副本，不落仓）
#   --BR_OK＝正控面★四枚★期望＝①名册枚数＝生成枚数 ②路径集合差＝0 ③生成物可 -c ④★序形锚同值★（＝批200 补的第四枚；旧三枚对「序形换了」全绿＝判据面反例 C2 实测、我本次独立复跑亦全绿）
# 退出码＝0（校验全过／生成成功）‖ 5（有条目不合）‖ 9（前置缺失＝面不在位、或无明令旗想写盘、或排除面出处读不到）
set -u
ROOT=$(cd "$(dirname "$0")" && pwd)          # ★tooling/ 由自己位置现取＝不硬编码★
SKILL="$ROOT/skill"
MF="$SKILL/_sha256_manifest.txt"
TOP="$ROOT/_sha256_manifest.txt"
BR_NOFLAG=0; BR_OK=0; MODE=""; ORDERED=0; TARG=skill
while [ $# -gt 0 ]; do
  case "$1" in
    --check) MODE=check; shift ;;
    --build) MODE=build; shift ;;
    --target) TARG="$2"; shift 2 ;;
    --by-order) ORDERED=1; shift ;;
    --BR_NOFLAG) BR_NOFLAG=1; MODE=build; ORDERED=0; shift ;;
    --BR_OK) BR_OK=1; MODE=build; ORDERED=1; shift ;;
    *) echo "未知参数 $1"; exit 9 ;;
  esac
done
[ -n "$MODE" ] || { echo "★用法＝--check | --build --by-order | --BR_NOFLAG | --BR_OK ‖ 可选 --target skill|top★"; exit 9; }
[ "$TARG" = skill ] || [ "$TARG" = top ] || { echo "★未知目标面★ $TARG（只许 skill ‖ top）"; exit 9; }

# ============ ★第四枚验收式＝序形锚（批200 落 r541q 问④裁＋反例 C2＝F4）============
# 在册三枚验收式＝①条目枚数等式 ②路径集合差＝0 ③LC_ALL=C sha256sum -c rc＝0 ⇒ 判据面实测：同一份 136 枚覆盖面在★落盘自然序★与★C 定序★两形下
# 三枚★全绿★、而清单本文行差＝228 枚＝★这三枚对「序形变了」完全无分辨力★（我本次也独立复跑到同一读数）。
# ⇒ 补第四枚＝★路径列序列的 sha16 锚★（只锚序列、不锚内容＝改内容不炸、改序形或改覆盖面必炸）；换代先比、比出差⇒正控面 rc=5 并点名。
#   锚值＝现取（式子见本件两处 SEQ／SEQT 行）；覆盖面合法增删时★必须同批改锚并另起句登记★＝本件缺省面（--check／--build）不受锚约束、不会因此永红。
ANCH_SKILL=7b3113d656b837ac     # tooling/skill 子清单＝136 枚路径列按 C 定序的序列 sha16（对立面＝落盘自然序 5c00e15aba48b34c＝不同值⇒锚真有分辨力）
ANCH_TOP=b914971a1e56f236       # tooling 顶层清单＝13 枚路径列按名册序的序列 sha16（对立面＝改 C 定序 bf0167a5fb85f9bc）

# ============ ★顶层面★＝tooling/_sha256_manifest.txt（头注逐字不动、名册沿用盘上条目行、只重算 sha＝R348 先例形）============
if [ "$TARG" = top ]; then
  [ -f "$TOP" ] || { echo "★前置缺失＝顶层清单不在位★ $TOP"; exit 9; }
  HDR=$(grep -c '^#' "$TOP"); ENT=$(grep -cE '^[0-9a-f]{64}  ' "$TOP")
  [ "$ENT" -gt 0 ] || { echo "★拒绝＝顶层条目枚数取到 0★"; exit 9; }
  if [ "$MODE" = check ]; then
    OUT=$(cd "$ROOT" && LC_ALL=C sha256sum -c _sha256_manifest.txt 2>&1); RC=$?
    OKN=$(printf '%s\n' "$OUT" | grep -c ': OK$'); BADN=$((ENT - OKN))
    echo "== 顶层清单校验（★只读★）头注＝$HDR ‖ 条目＝$ENT ‖ OK＝$OKN ‖ 不合＝$BADN ‖ rc=$RC"
    printf '%s\n' "$OUT" | grep -v ': OK$' | sed 's/^/   ★不合★ /'
    [ "$RC" = 0 ] && [ "$BADN" = 0 ] || { echo "本件 rc=5（＝★条目行与盘上现值不合＝清单这枚派生物漂了★＝按法定生命周期重生成）"; exit 5; }
    echo "本件 rc=0"; exit 0
  fi
  if [ "$ORDERED" != 1 ]; then
    echo "★拒绝重建★＝顶层清单的★整体重写★必须由明令触发 ⇒ 必须带 --by-order"; exit 9
  fi
  NAME=$(mktemp /tmp/top_roster.XXXXXX); NEW=$(mktemp /tmp/top_manifest.XXXXXX)
  grep -E '^[0-9a-f]{64}  ' "$TOP" | awk '{print $2}' > "$NAME"
  grep '^#' "$TOP" > "$NEW"
  while IFS= read -r p; do (cd "$ROOT" && sha256sum "$p") >> "$NEW"; done < "$NAME"
  N2=$(grep -cE '^[0-9a-f]{64}  ' "$NEW"); H2=$(grep -c '^#' "$NEW")
  DIFFP=$(diff <(LC_ALL=C sort "$NAME") <(grep -E '^[0-9a-f]{64}  ' "$NEW" | awk '{print $2}' | LC_ALL=C sort) | grep -c '^[<>]')
  [ "$N2" = "$ENT" ] && [ "$H2" = "$HDR" ] && [ "$DIFFP" = 0 ] || {
    echo "★拒绝落盘★＝条目 $ENT→$N2 ‖ 头注 $HDR→$H2 ‖ 名册差 $DIFFP（★三枚等式任一不合都不许动顶层★）"; rm -f "$NAME" "$NEW"; exit 9; }
  SEQT=$(grep -E '^[0-9a-f]{64}  ' "$NEW" | awk '{print $2}' | sha256sum | cut -c1-16)
  if [ "$BR_OK" = 1 ]; then
    echo "== ★正控（--BR_OK／top）★＝只生成到 /tmp、★不落仓★：条目 $ENT ‖ 头注 $HDR ‖ 名册差 $DIFFP"
    echo "   ★第四枚验收式＝序形锚★＝路径列（名册序）序列 sha16＝$SEQT ‖ 在册锚＝$ANCH_TOP ⇒ $([ "$SEQT" = "$ANCH_TOP" ] && echo 合 || echo '★不合＝序形或覆盖面换了★')"
    rm -f "$NAME" "$NEW"
    [ "$SEQT" = "$ANCH_TOP" ] || { echo "★正控不合期望＝序形锚换枚了★＝换代先比：要么序形被改、要么覆盖面增删未登记 ⇒ 必须同批改锚并另起句登记"; echo "本件（正控面）rc=5"; exit 5; }
    echo "本件（正控面）rc=0"; exit 0
  fi
  cp "$NEW" "$TOP"; rm -f "$NAME" "$NEW"
  OUT=$(cd "$ROOT" && LC_ALL=C sha256sum -c _sha256_manifest.txt 2>&1); RC=$?
  OKN=$(printf '%s\n' "$OUT" | grep -c ': OK$')
  echo "== ★顶层清单已按 R348 形重生成★（头注逐字不动、名册不变、只换 sha）条目＝$ENT ‖ rc=$RC ‖ OK＝$OKN"
  [ "$RC" = 0 ] && [ "$OKN" = "$ENT" ] || { echo "★重生成后自检不合＝本件有缺陷★"; exit 5; }
  echo "本件 rc=0"; exit 0
fi
# ============ ★子清单面★＝tooling/skill/_sha256_manifest.txt（生成序＝find 全量相对面＋LC_ALL=C sort）============
[ -d "$SKILL" ] || { echo "★前置缺失＝技能包镜像面不在位★ $SKILL"; exit 9; }

# ★排除面出处断言★＝RESTORE.sh 必须真在顶层清单里（读不到＝我不许把它从子清单里删掉）
if ! grep -q "skill/RESTORE.sh" "$TOP" 2>/dev/null; then
  echo "★拒绝＝排除面出处读不到★：顶层清单 $TOP 里没有 skill/RESTORE.sh 那一行 ⇒ RESTORE.sh 不许从本清单缺席"; exit 9
fi

if [ "$MODE" = build ] && [ "$ORDERED" != 1 ]; then
  echo "★拒绝重建★＝对已入库面的★整体重写★必须由明令触发 ⇒ 必须带 --by-order（真人 2026-10-21 那句「闸防手改不防重建、走法定生命周期」）"
  echo "   想要的正确动作＝先 --check 看红在哪几枚，再 --build --by-order"
  exit 9
fi

LIST=$(cd "$SKILL" && find . -type f ! -name "_sha256_manifest.txt" ! -name "RESTORE.sh" | sed 's|^\./||' | LC_ALL=C sort)
N=$(printf '%s\n' "$LIST" | grep -c .)
[ "$N" -gt 0 ] || { echo "★拒绝＝待录名册取到 0 枚★（生成面为空＝清单必假）"; exit 9; }

if [ "$MODE" = check ]; then
  # ★本件自曝的一枚★＝第一版用 `grep -c 'FAILED'` 数不合＝本机 sha256sum 说★中文★（「成功」／「失败」）⇒ rc=1 而计数报 0＝
  #   正是我在册那族「查不到不许空值冒充被检到」★我自己在新件里又犯一次★⇒ 正形＝锁 LC_ALL=C ＋★枚数等式★（OK 枚数必须＝条目枚数）
  OUT=$(cd "$SKILL" && LC_ALL=C sha256sum -c _sha256_manifest.txt 2>&1); RC=$?
  ENT=$(grep -c '  ' "$MF")
  OKN=$(printf '%s\n' "$OUT" | grep -c ': OK$')
  BADN=$((ENT - OKN))
  echo "== 子清单校验（★只读★）条目＝$ENT ‖ 待录名册＝$N ‖ OK＝$OKN ‖ 不合＝$BADN ‖ rc=$RC"
  printf '%s\n' "$OUT" | grep -v ': OK$' | sed 's/^/   ★不合★ /'
  [ "$RC" = 0 ] && [ "$BADN" = 0 ] || { echo "本件 rc=5（＝内容已改、清单尚未重建＝★法定生命周期里的正常红态★）"; exit 5; }
  echo "本件 rc=0"; exit 0
fi

NEW=$(mktemp /tmp/skill_manifest.XXXXXX)
( cd "$SKILL" && while IFS= read -r f; do sha256sum "$f"; done <<< "$LIST" ) > "$NEW"
M2=$(grep -c '  ' "$NEW")
SEQ=$(printf '%s\n' "$LIST" | sha256sum | cut -c1-16)
SEQN=$(printf '%s\n' "$LIST" | grep -c .)
if [ "$M2" != "$N" ]; then echo "★拒绝落盘＝生成枚数 $M2 ≠ 名册枚数 $N★"; rm -f "$NEW"; exit 9; fi
# ★集合差断言★＝新清单的路径列与旧清单的路径列差集必须只允许"空"（换代＝同覆盖面换 sha，不许多件或少件；多了要另批显式登记）
DIFFP=$(diff <(awk '{print $2}' "$MF" | LC_ALL=C sort) <(awk '{print $2}' "$NEW" | LC_ALL=C sort) | grep -c '^[<>]')
if [ "$BR_OK" = 1 ]; then
  echo "== ★正控（--BR_OK）★＝只生成到 /tmp、★不落仓★：名册 $N ‖ 生成 $M2 ‖ 与在册路径列差＝$DIFFP"
  echo "   ★第四枚验收式＝序形锚★＝路径列（C 定序、$SEQN 枚）序列 sha16＝$SEQ ‖ 在册锚＝$ANCH_SKILL ⇒ $([ "$SEQ" = "$ANCH_SKILL" ] && echo 合 || echo '★不合＝序形或覆盖面换了★')"
  OLD=$(sha256sum "$NEW" | cut -c1-16); rm -f "$NEW"
  echo "   正控期望＝差＝0（★换代不增不减覆盖面★）；生成物 sha16＝$OLD ‖ 已从 /tmp 删除＝$([ -f "$NEW" ] && echo 在 || echo 无)"
  [ "$DIFFP" = 0 ] || { echo "★正控不合期望＝路径列有增减★"; exit 5; }
  [ "$SEQ" = "$ANCH_SKILL" ] || { echo "★正控不合期望＝序形锚换枚了★＝换代先比：要么序形被改（★三枚旧验收式对此全绿★）、要么覆盖面增删未登记 ⇒ 必须同批改锚并另起句登记"; echo "本件（正控面）rc=5"; exit 5; }
  echo "本件（正控面）rc=0"; exit 0
fi
if [ "$DIFFP" != 0 ]; then
  echo "★拒绝落盘＝覆盖面枚数变了（差 $DIFFP 枚）★＝本次换代必须★同时★登记新增/删除件，不许让清单悄悄改集"
  diff <(awk '{print $2}' "$MF" | LC_ALL=C sort) <(awk '{print $2}' "$NEW" | LC_ALL=C sort) | sed 's/^/   /'
  rm -f "$NEW"; exit 9
fi
cp "$NEW" "$MF"; rm -f "$NEW"
OUT=$(cd "$SKILL" && LC_ALL=C sha256sum -c _sha256_manifest.txt 2>&1); RC=$?
OKN=$(printf '%s\n' "$OUT" | grep -c ': OK$')
echo "== ★子清单已按生成序重建★（生成物、非手改）条目＝$M2 ‖ 复跑 sha256sum -c rc=$RC ‖ OK＝$OKN（★枚数等式＝条目 $M2 − OK $OKN ＝ $((M2 - OKN))★）"
[ "$RC" = 0 ] && [ "$OKN" = "$M2" ] || { echo "★重建后自检不合＝本件有缺陷★"; exit 5; }
echo "本件 rc=0"; exit 0
