#!/usr/bin/env bash
# v_ledger_citation_baseline.sh —— 志文引用**计数基线对比器**（**候选件，未入门禁串**；小明 2026-10-11）
#   为什么要有它：R181 抓到"我把豁免档收留的指针逐字抄了三处"靠的**不是颜色**（RED／AMBER 一个没动），
#   而是 VCC 的 [计数] 总数从 634→635、豁免红从 3→4。那个"改动前后逐数对齐"的动作今天只存在于我脑子里 ⇒
#   按坑63／坑46 的口径，**没有件盯着的规矩第四次以后还会犯**，本件把它变成可复跑的东西。
#   边界（写死，防被当成闸）：**只比计数、不判红绿语义**——计数变动本身不是错，"这次改志文该不该变"仍由人判；
#   与 v_ledger_batch_consistency.sh 同族（报告器；入门与阈值归 A26／#12 那族，施工侧不自落）。
#   用法：
#     bash v_ledger_citation_baseline.sh --snap  <基线文件> <志文…>   # 立基线
#     bash v_ledger_citation_baseline.sh --check <基线文件> <志文…>   # 比对：全等 rc=0／有变动 rc=5／前置缺失 rc=9
#     bash v_ledger_citation_baseline.sh --selftest                  # 四例，**每例都必须能单独失败**
#   环境变量：VCC＝改指另一个 VCC 件（自测注桩用）。**本件不使用 FORCE**（在册 O-5：FORCE 禁用于判定）。
set -u

VCC="${VCC:-$(dirname "$0")/v_citation_check.sh}"

# 归一字段名 ⇒ 基线与现值两边都走同一张表，杜绝"基线里写 未命中=、现值里写 豁免表未命中="这种对不上账
FIELD_KEYS=(文档 单点引用 OK RED AMBER 勘误句 非单点形状 豁免红 豁免表命中 豁免表未命中)
VCC_PATTERNS=('文档=' '单点引用=' 'OK=' 'RED=' 'AMBER=' '勘误句=' '非单点形状=' '豁免红=' '豁免表命中=' '未命中=')

pick() {  # pick <行> <前缀> ⇒ 该前缀后面紧跟的第一个整数；取不到就打印空串（调用方必须判空，不得当 0）
    printf '%s' "$1" | grep -oE "$2[0-9]+" | head -1 | sed -E "s/^$2//"
}

# 输出一行一个 `字段=值`；**任一字段解析不出就整轮失败**（宁可 rc=9，也不让"少解析一项"悄悄变成"这项等于 0"＝坑45）
count_all() {
    local line
    line=$(bash "$VCC" "$@" 2>&1 | grep '^\[计数\]' | tail -1)
    [ -n "$line" ] || { echo "VCC 没有打印 [计数] 行 ⇒ 前置缺失（件坏了／被检面为空），不判" >&2; return 1; }
    local i name val
    for i in "${!FIELD_KEYS[@]}"; do
        name="${FIELD_KEYS[$i]}"; val=$(pick "$line" "${VCC_PATTERNS[$i]}")
        [ -n "$val" ] || { echo "字段「$name」在 [计数] 行里解析不出 ⇒ 本件与 VCC 口径不符，不判" >&2; return 1; }
        printf '%s=%s\n' "$name" "$val"
    done
}

base_val() {  # base_val <基线文件> <字段>
    grep -m1 -E "^$2=" "$1" 2>/dev/null | cut -d= -f2
}

snap() {
    local bf="$1"; shift
    [ "$#" -gt 0 ] || { echo "未给被检志文 ⇒ 空集不得判绿（坑29）" >&2; return 9; }
    local c
    if ! c=$(count_all "$@"); then
        echo "计数失败 ⇒ **不写基线**（写了就是把缺失固化成'正常'）" >&2; return 9
    fi
    { printf '# v_ledger_citation_baseline 基线｜时间：%s\n' "$(date '+%F %T')"
      printf '# 被检面：%s\n' "$*"
      printf '%s\n' "$c"; } > "$bf"
    echo "已写基线 $bf："
    printf '%s\n' "$c" | sed 's/^/  /'
}

check() {
    local bf="$1"; shift
    { [ -f "$bf" ] && [ -s "$bf" ]; } || { echo "基线文件不在位或为空：$bf ⇒ 前置缺失，不判（rc=9）" >&2; return 9; }
    local cur
    if ! cur=$(count_all "$@"); then
        echo "现值计数失败 ⇒ 不判（rc=9）" >&2; return 9
    fi
    local i name a b changed=0
    echo "字段              基线 → 现值"
    for i in "${!FIELD_KEYS[@]}"; do
        name="${FIELD_KEYS[$i]}"
        a=$(base_val "$bf" "$name"); b=$(printf '%s' "$cur" | grep -m1 -E "^$name=" | cut -d= -f2)
        { [ -n "$a" ] && [ -n "$b" ]; } || { echo "  「$name」基线或现值取不到 ⇒ 口径不符，不判（rc=9）"; return 9; }
        if [ "$a" = "$b" ]; then
            printf '  %-14s %s = %s  同值\n' "$name" "$a" "$b"
        else
            printf '  %-14s %s → %s  **变动 %+d**\n' "$name" "$a" "$b" "$(( b - a ))"
            changed=1
        fi
    done
    if [ "$changed" -eq 1 ]; then
        echo "LCB VERDICT: 计数有变动（**只报数**——变动本身不是错；该不该变由人判，入门与阈值归 A26／#12 那族）"
        return 5
    fi
    echo "LCB VERDICT: 与基线逐项同值"
    return 0
}

selftest() {
    command -v python3 >/dev/null 2>&1 || { echo "自测需 python3 造志文桩 ⇒ 前置缺失（rc=9）" >&2; return 9; }
    [ -f "$VCC" ] || { echo "VCC 不在位：$VCC ⇒ 自测没有对象，不判（rc=9）" >&2; return 9; }
    local tmp npass=0 nfail=0 rc
    tmp=$(mktemp -d)
    # 桩志文里引用的都是**盘上真件的真行**，避免自测自己造出假红（坑58：先自证再断言）
    printf -- '- 桩甲：`v_citation_check.sh:1` bash 在首行。\n' > "$tmp/a.md"
    printf -- '- 桩乙：本行不含任何单点引用。\n' > "$tmp/b.md"

    snap "$tmp/base.tsv" "$tmp/a.md" "$tmp/b.md" >/dev/null 2>&1; rc=$?
    [ "$rc" -eq 0 ] && { npass=$((npass+1)); echo "[PASS] T0 立基线成功"; } || { nfail=$((nfail+1)); echo "[FAIL] T0 立基线失败 rc=$rc"; }

    # T1 同面重测 ⇒ 必须 rc=0
    check "$tmp/base.tsv" "$tmp/a.md" "$tmp/b.md" > "$tmp/t1.log" 2>&1; rc=$?
    if [ "$rc" -eq 0 ]; then npass=$((npass+1)); echo "[PASS] T1 同面重测 rc=0（全等）";
    else nfail=$((nfail+1)); echo "[FAIL] T1 同面重测 rc=$rc ⇒ 件自己不稳，不可用：$(tail -2 "$tmp/t1.log" | tr '\n' ' ')"; fi

    # T2 阳性对照：新增一处引用 ⇒ **必须**被报成变动。这一例是整个件的存在理由——它不成，件就是空控（02 ⑩）
    # T2 故意引**同一个目标:行**的第二处——这正是 R181 那个盲区：RED／AMBER 一根不变，只有总数会动
    printf -- '- 桩甲再添一处：`v_citation_check.sh:1` bash 同一目标的第二处引用。\n' >> "$tmp/a.md"
    check "$tmp/base.tsv" "$tmp/a.md" "$tmp/b.md" > "$tmp/t2.log" 2>&1; rc=$?
    if [ "$rc" -eq 5 ] && grep -q '单点引用' "$tmp/t2.log" && grep -q '变动' "$tmp/t2.log"; then
        npass=$((npass+1)); echo "[PASS] T2 阳性对照：新增引用被报为变动（rc=5）"
    else nfail=$((nfail+1)); echo "[FAIL] T2 阳性对照失效 rc=$rc ⇒ 本件没有分辨力：$(tail -3 "$tmp/t2.log" | tr '\n' ' ')"; fi

    # T3 基线缺失 ⇒ 必须 rc=9，**不得**当成"没有变化"而判绿
    check "$tmp/nonexistent.tsv" "$tmp/a.md" > "$tmp/t3.log" 2>&1; rc=$?
    if [ "$rc" -eq 9 ]; then npass=$((npass+1)); echo "[PASS] T3 基线缺失 rc=9（不判）";
    else nfail=$((nfail+1)); echo "[FAIL] T3 基线未退 9（rc=$rc）⇒ 空基线被当成同值"; fi

    # T4 VCC 坏掉（桩不打印 [计数]）⇒ 必须 rc=9，不得静默返回全零（坑45：把"没读到"读成"读到 0"）
    printf '#!/usr/bin/env bash\necho "我什么都不报"\n' > "$tmp/stub.sh"
    VCC="$tmp/stub.sh" bash "$0" --check "$tmp/base.tsv" "$tmp/a.md" > "$tmp/t4.log" 2>&1; rc=$?
    if [ "$rc" -eq 9 ]; then npass=$((npass+1)); echo "[PASS] T4 VCC 不打印 [计数] 时 rc=9（不冒充全零）";
    else nfail=$((nfail+1)); echo "[FAIL] T4 未退 9（rc=$rc）：$(tail -2 "$tmp/t4.log" | tr '\n' ' ')"; fi

    rm -rf "$tmp"
    echo "LCB SELFTEST VERDICT: PASS=$npass FAIL=$nfail（T2 是本件的分辨力所在，它红＝整件不可用）"
    [ "$nfail" -eq 0 ] || return 5
    return 0
}

MODE="${1:-}"
case "$MODE" in
    --snap|--check) BASE="${2:-}"; [ -n "$BASE" ] || { echo "$MODE 需要基线文件路径" >&2; exit 9; }
                    [ "$#" -ge 3 ] || { echo "$MODE 需要至少一个被检志文 ⇒ 空集不判（坑29）" >&2; exit 9; }
                    if [ "$MODE" = "--snap" ]; then snap "$BASE" "${@:3}"; else check "$BASE" "${@:3}"; fi ;;
    --selftest) selftest ;;
    *) echo "用法：--snap <基线> <志文…> | --check <基线> <志文…> | --selftest（候选件，未入门禁串）" >&2; exit 9 ;;
esac
