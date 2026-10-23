#!/usr/bin/env bash
# _commit_title_numstat.sh —— ★「提交标题声称的动作 vs 该提交 numstat」对表件（★常驻、施工侧自有＝第十二枚；
#   ★不入门禁串、不改任何在册件的 rc 语义、只报不拦★）
#
# 起意＝批200（56759b4）标题自陈「★新规矩第一次行使＝「声称的动作 vs 该提交 numstat」真对表★…
#       ★本批人眼对表过一次、还没有件★」；批201 号位①把它固化成★能跑、会翻红★的东西。
#       ⇒ 此前这枚对表★只存在于人眼与标题散文里★＝姿势依赖人记（追加一百零六⑥「到签的守卫要落成件」同族）。
#
# ★判据三级（★扫哪一档写进件头★＝追加一百二十六：只在脑子里的规矩＝没有规矩）★：
#   A 行数级（最硬）＝标题「<面名>…追加 N 行」且★行紧跟数字★ ⇒ numstat 该面 additions 必须＝N
#       （N 认阿拉伯数字与中文 一二三四五六七八九十两；★不认复合数如「五十」★＝那是追加编号不是行数）
#   B 面级存在性＝标题某单元形如「<面名><≤3 字符>动作词」⇒ numstat 必须出现该面（★缺一即红★）
#       ★仅两面可判★＝04:段（ ｜ 00_总览:档头滚｜档头｜条目区
#       （05／留底／状态面／档头历史 的 B ★已放弃★：引用在册条目与声称动了完全同形、不可区分 ⇒ 见 act_re 注）
#   C 未动形＝标题某单元形如「<面名><≤14 字符>未动」⇒ numstat 必须★不★出现该面
#   D Y15 三枚尺（★批247／R595 上牙★）＝★只对 Y15 落册批及其后的提交生效★（域下限＝5ce95b2，用
#     「git merge-base --is-ancestor」判，形如账构的判定域下限＝早于闸的史文不回填）：
#       D1 首句（第一个全角「＝」之前）≤40 字 ‖ D2 正文（其后）≤400 字
#       D3 段数＝三段＝★做了什么／为什么做／去看哪里★ 三枚段名各恰 1 次★且按序★
#         ★段边界只认「段名＋全角＝」★——正文里把段名当★词★引用不算段（批247 上线第一跑就把自己的
#         提交判红两枚＝拿裸词计数的假红，见 note/02 坑95；该枚提交标题自此留作★假红反例的正控★）
#     ★字数按 Unicode 字符数、不按字节★（件内走 python3 len，绝不在 LC_ALL=C 下用 wc -m＝坑94 同族）
#     ★标题里没有「＝」＝不代判、只印「D 不可证」★（那是形检 F3 的辖域，两枚尺不互相顶替）
#
# ★四枚必守技术点（全部有存量实证）★：
#   ①★必须带 -c core.quotepath=false★（不带时 CJK 路径被 git 转义成八进制、grep 中文全给 0＝★假零★）
#   ②★04／05 面名必须带左右边界★＝`(^|[^0-9A-Za-z_])04([^0-9A-Za-z_]|$)`：
#      否则 ★r205★ 里的子串「05」会被当成面名（批201 存量扫第一版实测假阳 91 条的主因之一）
#   ③★「行」必须紧跟数字★＝否则「05 追加五十（★整行余值…」里的"整行"会被当成行数（同批实测假阳）
#   ④★声称行数按 numstat additions 列★、不按 wc -l（numstat 已把空行算进去）
#
#   E 件级两态对表（★R606／批258 问三硬要求、批260 落件★）＝给定 <件路径>，从 git 取
#     改前版（缺省 base＝HEAD）与工作树改后版各跑一遍，报 rc／输出行数／三档断言计数
#     （[PASS|FAIL|ABSENT]）两态差；★硬要求★＝改后三档计数★不得小于***改前（除非显式
#     给 --allow-shrink 并附理由）；★计数为 0 只对旧态合法***＝改前 0 是常态、不是缺陷。
#     ★与 A—D 的分工★＝A—D 管「提交标题声称 vs numstat」，E 管「改一件件之前 vs 之后」。
# 用法＝bash _commit_title_numstat.sh <sha>            # 对表单枚提交
#       bash _commit_title_numstat.sh --selftest       # 正负控自证（★九枚＝三枚真实提交对表＋六枚 D 级尺★、只读 git、零写盘）
#       bash _commit_title_numstat.sh --scan-all       # 扫存量（★新闸上线必须先扫已入库违例★）
#       bash _commit_title_numstat.sh --two-state <件> [--base <sha>] [--allow-shrink 理由]   # E 级
# 退出码＝0（全部对得上 ‖ 自测全合期望）‖ 5（★有不符★ ‖ 自测不合期望）‖ 9（前置缺失：sha 不存在／git 不可用）
set -u
# ★R589／批242 修 ROOT★＝本件随 SCAN集成 于批208 搬入 note/进度/v7/S1/… ⇒ 实际深 6 级，原式四跳只到 note/进度
#   ⇒ 该件自批208 起★恒报「前置缺失＝不在位／不是 git 仓」★，批末读数面四枚附面同步恒报「件不在位」（本批现跑抓到）
ROOT=$(git rev-parse --show-toplevel 2>/dev/null) || ROOT=$(cd "$(dirname "$0")/../../../../.." && pwd)
cd "$ROOT" || { echo "前置缺失：仓根不在位"; exit 9; }
[ -d .git ] || { echo "前置缺失：不是 git 仓"; exit 9; }

FACES="04 05 00_总览 档头历史 状态面 留底"

face_path() {
  case "$1" in
    04)        echo "note/04_配置改动diff.md" ;;
    05)        echo "note/05_复验作业单.md" ;;
    00_总览)   echo "note/进度/00_总览.md" ;;
    档头历史)  echo "note/进度/总览分册/00_档头历史.md" ;;
    状态面)    echo "note/进度/v7/S1/SCAN集成/证据/_rqq_watch.state.tsv" ;;
    留底)      echo "note/进度/v7/S1/SCAN集成/证据/log/raw/" ;;
  esac
}

# 动作词正则（B 判据用；04 带左右边界）
# ★B 的辖域★只保留两类★最明确★的「动了」形（★批201 存量扫第二版实证★：
#   「R247 刚写进 05 追加二十五」这种★引用在册条目★与「05 追加一百二十七」这种★声称动了★完全同形、
#   无法可靠区分 ⇒ 05／留底／状态面／档头历史 四面的 B ★整体放弃★（它们由 A 行数级与 C 未动形覆盖）；
#   ★第三版又收窄★＝「条目区」也放弃（bb54f6d 实证：「00_总览 条目区『志 220 行』」是★在读一个数★、
#   不是说动了该面）⇒ 现只留「档头滚」与「段（」两形：
#   宁可少判一面，不可把「引用／读数」当「动了」＝假阳比漏判更坏（假阳会训练人忽略本件）★）
act_re() {
  case "$1" in
    04)       echo '(^|[^0-9A-Za-z_])04[^0-9]{0,3}(R[0-9]+ 段（|段（)' ;;
    00_总览)  echo '00_总览[^。；]{0,3}档头滚' ;;
    *)        echo '@@NONE@@' ;;
  esac
}

cn2num() { # ★阿拉伯数字原样返回；中文单字转换；复合数（如「五十」）与空串一律返回空⇒调用方跳过★
  case "$1" in
    一) echo 1 ;; 二) echo 2 ;; 两) echo 2 ;; 三) echo 3 ;; 四) echo 4 ;;
    五) echo 5 ;; 六) echo 6 ;; 七) echo 7 ;; 八) echo 8 ;; 九) echo 9 ;; 十) echo 10 ;;
    '') echo "" ;;
    *) printf '%s' "$1" | grep -qE '^[0-9]+$' && echo "$1" || echo "" ;;
  esac
}

Y15_SINCE="5ce95b2"   # ★D 级域下限★＝批239（Y15 落进正文 §5 的那枚提交）；早于它的史文不回填

d_eval() { # $1＝提交标题（★整个提交信息那一行★）⇒ stdout＝不合行／不可证行；return＝不合枚数
  local raw n
  raw=$(Y15_T="$1" python3 - <<'PY'
import os
t = os.environ.get("Y15_T", "").strip()
red = []; info = []
if not t:
    red.append("  [D 前置不合] 提交标题为空")
elif "＝" not in t:
    info.append("  [D 不可证] 标题无「＝」＝首句与正文分不开（属形检 F3 辖域，本件不代判、不写成无缺）")
else:
    i = t.index("＝"); head, body = t[:i], t[i + 1:]
    if len(head) > 40:
        red.append("  [D1 首句超] 首句 %d 字＞40（Y15 在册上限）" % len(head))
    if len(body) > 400:
        red.append("  [D2 正文超] 正文 %d 字＞400（Y15 在册上限）" % len(body))
    segs = ["做了什么", "为什么做", "去看哪里"]
    pos = []; miss = []
    for s in segs:
        # ★段边界只认「段名＋全角＝」这一种形★：正文里把三枚段名当★词★引用（如「三段（做了什么／为什么做／
        # 去看哪里）各恰 1 次」）不该算段＝批247 的标题被第一版判红两枚，就是拿裸词计数造的假红（坑95）
        c = body.count(s + "＝")
        if c == 0:
            miss.append(s)
        elif c > 1:
            red.append("  [D3 段名重] 段边界「%s＝」出现 %d 次＞1（Y15 只三段）" % (s, c))
        pos.append(body.find(s + "＝"))
    if miss:
        red.append("  [D3 段缺] 缺段边界：%s（Y15 三段＝做了什么／为什么做／去看哪里，★只认后跟全角＝的那一处★）" % "、".join(miss))
    elif pos != sorted(pos):
        red.append("  [D3 段序乱] 三枚段边界出现位 %s 非升序（Y15 按段排）" % pos)
for x in red:
    print("R\t%s" % x)
for x in info:
    print("I\t%s" % x)
print("N\t%d" % len(red))
PY
)
  n=$(printf '%s\n' "$raw" | awk -F'\t' '$1=="N"{print $2}')
  printf '%s\n' "$raw" | awk -F'\t' '$1!="N"{print $2}'
  case "$n" in ''|*[!0-9]*) n=0 ;; esac
  return "$n"
}

d_in_domain() { git merge-base --is-ancestor "$Y15_SINCE" "$1" 2>/dev/null; }

get_title()   { git -c core.quotepath=false log -1 --format=%s "$1" 2>/dev/null; }
get_numstat() { git -c core.quotepath=false show --numstat --format='' "$1" 2>/dev/null; }

ns_add() { printf '%s\n' "$1" | awk -v p="$2" '$3==p {print $1; f=1} END{if(!f) print "-"}'; }

check_one() { # $1=sha ⇒ stdout=不合行；return=不合枚数（9＝前置缺失）
  local sha="$1" title numstat bad=0
  title=$(get_title "$sha"); numstat=$(get_numstat "$sha")
  [ -n "$title" ] || { echo "前置缺失：取不到 $sha 的标题"; return 9; }

  local units u face path re n want got frag has any_unit
  units=$(printf '%s' "$title" | sed 's/｜/\n/g; s/‖/\n/g; s/；/\n/g; s/。/\n/g; s/＝/\n/g; s/＋/\n/g')

  # ★快速排除★：标题不含「未动」⇒ C 判据整体跳过（省掉 6 次全标题 grep）
  local has_static=0
  case "$title" in *未动*) has_static=1 ;; esac

  # ★只留可能相关的单元★（含六个面名之一 或 含未动）⇒ 单元数从几十降到个位数
  local slim=""
  while IFS= read -r u; do
    [ -n "$u" ] || continue
    case "$u" in
      *04*|*05*|*00_总览*|*档头历史*|*状态面*|*留底*|*未动*) slim="$slim$u"$'\n' ;;
    esac
  done <<EOF
$units
EOF

  for face in $FACES; do
    path=$(face_path "$face"); re=$(act_re "$face")
    # ---- C 未动形（★面名也要带边界★＝否则 r505q 里的「05」会被当成面名：批201 第三版实证 c54db29 假阳）----
    if [ "$has_static" = 1 ]; then
      local cre
      case "$face" in
        04|05) cre='(^|[^0-9A-Za-z_])'$face'[^。；]{0,14}未动' ;;
        *)     cre="$face[^。；]{0,14}未动" ;;
      esac
      if printf '%s\n' "$slim" | grep -qE "$cre"; then
        got=$(ns_add "$numstat" "$path")
        if [ "$got" != "-" ]; then
          echo "  [C 未动形不合] 标题称「$face … 未动」但 numstat 有该面：additions=$got"
          bad=$((bad+1))
        fi
      fi
    fi
    # ---- 逐单元：A 行数级 ＋ B 面级存在 ----
    while IFS= read -r u; do
      [ -n "$u" ] || continue
      # ★先确认本单元含此面（04／05 带左右边界，否则 r205 里的「05」会被当成面名）★
      case "$u" in
        *"$face"*) ;;
        *) continue ;;
      esac
      has=0
      case "$face" in
        04|05) printf '%s' "$u" | grep -qE '(^|[^0-9A-Za-z_])'$face'([^0-9A-Za-z_]|$)' && has=1 ;;
        *)     has=1 ;;
      esac
      [ "$has" = 1 ] || continue
      # A：追加 N 行（★宽松取片段、再由 cn2num 收口★：复合数如「五十」返回空⇒自动跳过＝那是追加编号不是行数）
      frag=$(printf '%s' "$u" | grep -oE '追加[^。；]{0,6}行' | head -1)
      if [ -n "$frag" ]; then
        n=$(printf '%s' "$frag" | grep -oE '[0-9一二三四五六七八九十两]+' | head -1)
        want=$(cn2num "$n")
        if [ -n "$want" ]; then
          got=$(ns_add "$numstat" "$path")
          if [ "$got" != "$want" ]; then
            echo "  [A 行数级不合] 标题称「$face 追加 $n 行」(=$want) 但 numstat additions=$got"
            bad=$((bad+1))
          fi
        fi
      fi
      # B：面级存在（面名后紧跟动作词；★该单元若含「未动」则归 C 辖域、B 不重复判★）
      case "$u" in *未动*) continue ;; esac
      if printf '%s' "$u" | grep -qE "$re"; then
        got=$(ns_add "$numstat" "$path")
        if [ "$got" = "-" ]; then
          echo "  [B 面级不合] 标题称动了「$face」但 numstat 无该面"
          bad=$((bad+1))
        fi
      fi
    done <<EOF
$slim
EOF
  done
  # ---- D：Y15 三枚尺（★只对域内提交生效★＝5ce95b2 及其后；域外只印一行、不写成无缺）----
  local dout drc
  if d_in_domain "$sha"; then
    dout=$(d_eval "$title"); drc=$?
    if [ -n "$dout" ]; then printf '%s\n' "$dout"; fi
    bad=$((bad+drc))
  else
    echo "  [D 域外] 早于 Y15 落册批 $Y15_SINCE ⇒ 三枚尺不判（史文不回填）"
  fi
  return $bad
}

MODE="${1:-}"

# ---------- E 级：件级两态对表（问三硬要求） ----------
two_state() {
  local f="" base="HEAD" allow="" why=""
  while [ $# -gt 0 ]; do
    case "$1" in
      --base) base="$2"; shift 2 ;;
      --allow-shrink) allow=1; why="$2"; shift 2 ;;
      *) if [ -z "$f" ]; then f="$1"; fi; shift ;;
    esac
  done
  [ -n "$f" ] || { echo "E 级要一个件路径"; return 9; }
  [ -f "$f" ] || { echo "件不在位：$f"; return 9; }
  local td; td=$(mktemp -d "${TMPDIR:-/tmp}/ts.XXXXXX") || return 9
  git -C "$(git rev-parse --show-toplevel 2>/dev/null || echo .)" show "$base:$f" > "$td/before.sh" 2>/dev/null
  if [ ! -s "$td/before.sh" ]; then echo "改前版取不到：$base:$f"; rm -rf "$td"; return 9; fi
  local ROOTD; ROOTD=$(git rev-parse --show-toplevel 2>/dev/null || echo .)
  local rb ra rcb rca lb la cb ca
  rb=$(cd "$ROOTD" && bash "$td/before.sh" 2>&1); rcb=$?
  ra=$(cd "$ROOTD" && bash "$f" 2>&1); rca=$?
  lb=$(printf '%s\n' "$rb" | wc -l); la=$(printf '%s\n' "$ra" | wc -l)
  cb=$(printf '%s\n' "$rb" | grep -c -E '\[(PASS|FAIL|ABSENT)\]')
  ca=$(printf '%s\n' "$ra" | grep -c -E '\[(PASS|FAIL|ABSENT)\]')
  echo "E 两态对表 $f（base=$base）"
  echo "  改前 rc=$rcb 行=$lb 三档=$cb ‖ 改后 rc=$rca 行=$la 三档=$ca"
  if [ "$cb" -eq 0 ]; then echo "  ★旧态三档 0＝常态（该件此前不执行）★ 不计缺陷"; fi
  if [ "$ca" -lt "$cb" ]; then
    if [ -n "$allow" ]; then echo "  ★三档变小但已显式批准★ 理由＝$why"; rm -rf "$td"; return 0; fi
    echo "  X 改后三档小于改前＝覆盖面受损（问三硬要求）⇒ 须给 --allow-shrink 与理由"; rm -rf "$td"; return 5
  fi
  echo "  OK 改后三档不小于改前"
  rm -rf "$td"; return 0
}

case "$MODE" in
  --selftest)
    echo "== 自测（★九枚＝三枚真实提交对表＋六枚 D 级尺★、只读 git、零写盘）=="
    echo "   ★判读★＝不合枚数 0 ⇒ 期望绿；≠0 ⇒ 期望翻红（单枚对表模式统一映射为退出码 5）"
    fail=0
    out=$(check_one 578b186); rc=$?
    echo "负控 578b186（批199·补，裸·补形）：不合枚数=$rc"
    [ -n "$out" ] && printf '%s\n' "$out"
    if [ "$rc" -eq 0 ]; then echo "  X 期望翻红（≠0），实得 0 ⇒ 本件无牙"; fail=$((fail+1));
    else echo "  ✔ 翻红如期望"; fi
    out=$(check_one 56759b4); rc=$?
    echo "正控一 56759b4（批200·补一）：不合枚数=$rc"
    [ -n "$out" ] && printf '%s\n' "$out"
    if [ "$rc" -ne 0 ]; then echo "  X 期望绿（0），实得 $rc"; fail=$((fail+1));
    else echo "  ✔ 全绿如期望"; fi
    out=$(check_one 13594c7); rc=$?
    echo "正控二 13594c7（批200）：不合枚数=$rc"
    [ -n "$out" ] && printf '%s\n' "$out"
    if [ "$rc" -ne 0 ]; then echo "  X 期望绿（0），实得 $rc"; fail=$((fail+1));
    else echo "  ✔ 全绿如期望"; fi
    echo "-- D 级六枚（★正控取真提交的那一行标题、负控与反控取合成串＝只测尺、不掺 A/B/C★）--"
    out=$(d_eval "$(get_title d414fd2)"); rc=$?
    echo "D 正控 d414fd2（批243 真提交标题）：不合枚数=$rc"
    [ -n "$out" ] && printf '%s\n' "$out"
    if [ "$rc" -ne 0 ]; then echo "  X 期望绿（0），实得 $rc"; fail=$((fail+1)); else echo "  ✔ 全绿如期望"; fi
    out=$(d_eval "$(get_title a3710d6)"); rc=$?
    echo "D 正控二 a3710d6（批247 真提交★＝正文里把三枚段名当词引用＝第一版假红的那一枚★）：不合枚数=$rc"
    [ -n "$out" ] && printf '%s\n' "$out"
    if [ "$rc" -ne 0 ]; then echo "  X 期望绿（0），实得 $rc ⇒ 段边界计数仍把词引用算成段"; fail=$((fail+1)); else echo "  ✔ 全绿如期望（词引用不算段边界）"; fi
    out=$(d_eval "x86侧批999（这一枚提交把三件事都写了进去所以首句故意写得很长很长超过四十字）＝做了什么＝甲‖为什么做＝乙‖去看哪里＝丙"); rc=$?
    echo "D 负控一（首句＞40）：不合枚数=$rc"
    [ -n "$out" ] && printf '%s\n' "$out"
    if [ "$rc" -eq 0 ]; then echo "  X 期望翻红（≠0），实得 0 ⇒ 尺无牙"; fail=$((fail+1)); else echo "  ✔ 翻红如期望"; fi
    out=$(d_eval "x86侧批999（少了一段）＝做了什么＝甲‖为什么做＝乙"); rc=$?
    echo "D 负控二（缺段名「去看哪里」）：不合枚数=$rc"
    [ -n "$out" ] && printf '%s\n' "$out"
    if [ "$rc" -eq 0 ]; then echo "  X 期望翻红（≠0），实得 0 ⇒ 尺无牙"; fail=$((fail+1)); else echo "  ✔ 翻红如期望"; fi
    out=$(d_eval "x86侧批999（正文超长反控）＝做了什么＝甲‖为什么做＝乙‖去看哪里＝$(python3 -c 'print("字"*401)')"); rc=$?
    echo "D 反控三（正文＞400）：不合枚数=$rc"
    [ -n "$out" ] && printf '%s\n' "$out"
    if [ "$rc" -eq 0 ]; then echo "  X 期望翻红（≠0），实得 0 ⇒ 尺无牙"; fail=$((fail+1)); else echo "  ✔ 翻红如期望"; fi
    out=$(d_eval "x86侧批999 没有等号的一枚标题"); rc=$?
    echo "D 不可证（无「＝」）：不合枚数=$rc"; [ -n "$out" ] && printf '%s\n' "$out"
    if [ "$rc" -ne 0 ]; then echo "  X 期望 0（不代判、不冒红），实得 $rc"; fail=$((fail+1));
    else case "$out" in *不可证*) echo "  ✔ 判 0 且印「不可证」＝没把不判冒充无缺" ;; *) echo "  X 期望印出不可证一行，实没印"; fail=$((fail+1));; esac; fi
    echo "--"
    if [ "$fail" -eq 0 ]; then echo "✔ 自测九枚全合期望 ⇒ 本件有牙"; exit 0
    else echo "X 自测有 $fail 枚不合 ⇒ 本件有缺陷"; exit 5; fi
    ;;
  --two-state)
    shift
    out=$(two_state "$@"); rc=$?
    echo "== E 级 =="
    printf '%s\n' "$out"
    if [ "$rc" -eq 0 ]; then echo "✔ 两态对表合（rc=0）"; exit 0; fi
    echo "X 两态对表不合 rc=$rc"; exit 5 ;;
  --scan-all)
    echo "== 存量扫（★新闸上线必须先扫已入库违例、否则绿会被读成从来没违例过＝假绿★）=="
    total=0; bad_n=0
    for sha in $(git -c core.quotepath=false log --format=%H); do
      total=$((total+1))
      out=$(check_one "$sha"); rc=$?
      if [ "$rc" -ne 0 ] && [ "$rc" -ne 9 ]; then
        bad_n=$((bad_n+1)); echo "X $sha"
        [ -n "$out" ] && printf '%s\n' "$out"
      fi
    done
    echo "--"
    echo "存量提交=$total ‖ 不合=$bad_n"
    if [ "$bad_n" -eq 0 ]; then echo "（零不合＝存量无违例）"; exit 0; else exit 5; fi
    ;;
  "")
    echo "用法：bash _commit_title_numstat.sh <sha> | --selftest | --scan-all | --two-state <件> [--base <sha>]"; exit 9 ;;
  *)
    out=$(check_one "$MODE"); rc=$?
    echo "== 对表 $MODE =="
    if [ "$rc" -eq 9 ]; then printf '%s\n' "$out"; exit 9; fi
    if [ "$rc" -eq 0 ]; then echo "✔ 全部对得上（rc=0）"; exit 0; fi
    echo "X 不合 $rc 枚："; printf '%s\n' "$out"; exit 5 ;;
esac
