#!/usr/bin/env bash
# v_verdict_field_parity.sh —— 「结论行 vs 字段值」并排核（**候选件原型，未入门禁串**；小明 2026-10-11，R207 建）
# 为什么要有它：02 ㉕ 抓到那次 aborted 窗的读数件里 result_returned 为真、而同件结论行写"仍无 result ⇒ 升为已证"。
#   这类"结论由人手写、字段由件写 ⇒ 两者不同源"的缺陷**今天没有任何件盯着**（坑62 族；#14／#29 那两件一直没等到同批）。
# 边界（写死，防被当成闸）：**只列"并排疑点"、不自判红绿、不改任何读数**——哪条真是错、要不要改判＝人判（与 A26 报告器同族；
#   入门与阈值归 #12／#19／A32 那一族，施工侧不自落）。判据本体在本件里**不存在**。
# 用法：
#   bash v_verdict_field_parity.sh <目录或 json 文件 ...>    # 缺省＝不判（见 rc=9）
#   bash v_verdict_field_parity.sh --selftest                # 三例：阳性须报出／阴性须不报／缺前置须 rc=9
# rc：0 无疑点；5 有疑点（**仍只是"请看这一行"，不是判红**）；9 前置缺失（没给件／JSON 解析不了／件里根本没有被并排的键）。
# 环境变量（**缺省＝现行规则，本件不自定档**）：
#   VP_RULES=off      ⇒ 关掉全部规则、只做并排计数（自测注桩用，也是候选「丙」的形状）
#   VP_RULESET=cur    ⇒ 缺省：两条规则都逐条列（首扫实测＝承重 5＋噪音 7）
#   VP_RULESET=A      ⇒ 候选「甲」：claim 级（已证／推翻）只在**同一行同时提到本件的字段名或件名**时才报（治跨对象误伤）
#   VP_RULESET=B      ⇒ 候选「乙」：claim 级**只进计数、不逐条列**（nosay 级仍逐条列，保证承重条仍可指认）
#   这三档是给 #12／#19／小美**定档用的对照面**，不是三个都算通过；哪一档成为件的行为＝主理人／小美明文，施工侧不自落。
# **本件不使用 FORCE**（在册 O-5：FORCE 禁用于判定）。
set -u

if [ "${1:-}" = "--selftest" ]; then
  T=$(mktemp -d /tmp/vp_selftest.XXXXXX)
  mkdir -p "$T/pos" "$T/neg" "$T/empty"
  python3 - "$T" <<'PYEOF'
import json, os, sys
T = sys.argv[1]
# 阳性例：字段说"回了话"，同目录文本说"无 result 且升为已证" ⇒ 必须被列
json.dump({"attempts": [{"result_returned": True, "wall_s": 60.2, "outcome": None}],
           "patience": {"before_set": {"planner_patience": 3.0}, "restored": {"planner_patience": 60.0}}},
          open(os.path.join(T, "pos", "b18.json"), "w"))
open(os.path.join(T, "pos", "NOTE.md"), "w").write("verdict: 该对仍无 result ⇒ 在册『不可终止』升为『已证』\n")
# 阴性例：字段与文本同向（确实没回话），且 restored 等于 before_set ⇒ 不该被列
json.dump({"attempts": [{"result_returned": False, "wall_s": 90.0}],
           "patience": {"before_set": {"planner_patience": 3.0}, "restored": {"planner_patience": 3.0}}},
          open(os.path.join(T, "neg", "b18.json"), "w"))
open(os.path.join(T, "neg", "ANALYSIS.md"), "w").write("客户端预算内没有收到 result\n")
PYEOF
  mkdir -p "$T/mix"
  python3 - "$T" <<'PYFIX'
import json, os, sys
T = sys.argv[1]
# 混合例：件既无回话也无 outcome ⇒ claim 级规则会开火；两份文本一份**提到字段名**、一份**不提**
json.dump({"attempts": [{"result_returned": False, "wall_s": 90.0, "outcome": None}]},
          open(os.path.join(T, "mix", "b18.json"), "w"))
open(os.path.join(T, "mix", "NAMED.md"), "w").write("该对 outcome 缺失，故在册那句被实证推翻\n")
open(os.path.join(T, "mix", "UNNAMED.md"), "w").write("另一件事另论：那一句的说法已被推翻\n")
PYFIX
  r1=0; bash "$0" "$T/pos"  >/dev/null 2>&1 || r1=$?
  r2=0; bash "$0" "$T/neg"  >/dev/null 2>&1 || r2=$?
  r3=0; bash "$0" "$T/empty" >/dev/null 2>&1 || r3=$?
  n1=$(bash "$0" "$T/pos" 2>/dev/null | grep -c '^\[疑点\]')
  n2=$(bash "$0" "$T/neg" 2>/dev/null | grep -c '^\[疑点\]')
  n_cur=$(bash "$0" "$T/mix" 2>/dev/null | grep -c '^\[疑点\]')
  n_A=$(VP_RULESET=A bash "$0" "$T/mix" 2>/dev/null | grep -c '^\[疑点\]')
  n_AB=$(VP_RULESET=AB bash "$0" "$T/mix" 2>/dev/null | grep -c '^\[疑点\]')
  n_B=$(VP_RULESET=B bash "$0" "$T/mix" 2>/dev/null | grep -c '^\[疑点\]')
  c_B=$(VP_RULESET=B bash "$0" "$T/mix" 2>/dev/null | grep -oE 'claim只计数=[0-9]+' | cut -d= -f2)
  r6=0; VP_RULES=off bash "$0" "$T/pos" >/dev/null 2>&1 || r6=$?
  n6=$(VP_RULES=off bash "$0" "$T/pos" 2>/dev/null | grep -c '^\[疑点\]')
  rm -rf "$T"
  ok1=$([ "$r1" = "5" ] && [ "$n1" -ge 2 ] && echo 1 || echo 0)
  ok2=$([ "$r2" = "0" ] && [ "$n2" = "0" ] && echo 1 || echo 0)
  ok3=$([ "$r3" = "9" ] && echo 1 || echo 0)
  echo "[C1] 阳性例须 rc=5 且至少列出两条并排疑点：实得 rc=$r1 疑点=$n1"
  echo "[C2] 阴性例须 rc=0 且疑点=0：实得 rc=$r2 疑点=$n2"
  echo "[C3] 缺前置（目录里没有 json）须 rc=9：实得 rc=$r3"
  ok4=$([ "$n_cur" = "2" ] && [ "$n_A" = "1" ] && [ "$n_AB" = "1" ] && [ "$n_B" = "0" ] && [ "$c_B" = "2" ] && echo 1 || echo 0)
  ok5=$([ "$r6" = "0" ] && [ "$n6" = "0" ] && echo 1 || echo 0)
  echo "[C4] 改档必须真改变行为（混合例须 cur=2／甲=1／合档=1 只留提到字段名那条／乙=0 打印而 claim只计数=2）：实得 cur=$n_cur A=$n_A AB=$n_AB B=$n_B B计数=$c_B"
  echo "[C5] VP_RULES=off 须不判任何疑点且 rc=0（防「关掉规则还报红」）：实得 rc=$r6 疑点=$n6"
  npass=0; for x in $ok1 $ok2 $ok3 $ok4 $ok5; do [ "$x" = "1" ] && npass=$((npass+1)); done
  echo "PARITY SELFTEST VERDICT: PASS=$npass FAIL=$((5-npass))"
  [ "$npass" = "5" ] || exit 5
  exit 0
fi

[ $# -ge 1 ] || { echo "[FAIL] 未给被检件 ⇒ 空集不得判绿（R46 一；与坑29 同族）" >&2; exit 9; }

python3 - "$@" <<'PYEOF'
import glob, json, os, re, sys
paths = sys.argv[1:]
jsons, dirs = [], []
for p in paths:
    if os.path.isdir(p):
        dirs.append(p)
        jsons += sorted(glob.glob(os.path.join(p, '**', '*.json'), recursive=True))
    elif p.endswith('.json'):
        jsons.append(p)
        dirs.append(os.path.dirname(p))
    else:
        print('[跳过] 非 json：%s' % p)
if not jsons:
    print('[前置缺失] 给定的件里没有任何可解析的 json ⇒ 不判（rc=9）')
    sys.exit(9)

RULESET = os.environ.get('VP_RULESET', 'cur') or 'cur'
RULES_OFF = (os.environ.get('VP_RULES', '') == 'off')
print('[规则档] %s（**缺省＝cur；改档需主理人／小美明文，本件不自定**）' % ('off（只做并排计数）' if RULES_OFF else RULESET))
NOSAY = re.compile(r'(无\s*result|没有\s*result|未收到\s*result|未回话|仍无规划|没有任何\s*result)')
CLAIM = re.compile(r'(升为.{0,4}已证|已证|不可终止|被实证推翻|推翻)')

def scal(d, pre=''):
    out = {}
    for k, v in d.items():
        if isinstance(v, dict):
            out.update(scal(v, pre + k + '.'))
        elif isinstance(v, list):
            for i, it in enumerate(v):
                if isinstance(it, dict):
                    out.update(scal(it, '%s%s[%d].' % (pre, k, i)))
        else:
            out[pre + k] = v
    return out

nflag = 0
claim_only = 0
used_txt = 0
# 去重：给定的目录可能互相嵌套（父目录的递归 glob 会把子目录的件再收一遍）⇒ 按绝对路径去重，
#   否则 [计数] 与疑点条数会被同一件重复贡献（R208 实测：同一份件被数了两次）。
seen, uniq = set(), []
for j in jsons:
    a = os.path.abspath(j)
    if a in seen:
        continue
    seen.add(a)
    uniq.append(j)
dup = len(jsons) - len(uniq)
jsons = uniq
for j in jsons:
    try:
        d = json.load(open(j, encoding='utf-8', errors='ignore'))
    except Exception as e:
        print('[前置缺失] 解析不了 %s（%s）⇒ 该件不判' % (j, str(e)[:40]))
        continue
    f = scal(d)
    d_dir = os.path.dirname(j)
    txts = []
    for ext in ('*.md', '*.txt'):
        txts += sorted(glob.glob(os.path.join(d_dir, ext)))
    used_txt += len(txts)
    rr_true = [k for k, v in f.items() if k.endswith('result_returned') and v is True]
    rr_false = [k for k, v in f.items() if k.endswith('result_returned') and v is False]
    has_outcome = any(k for k in f if k.endswith('outcome') and f[k] not in (None, ''))
    pb = f.get('patience.before_set.planner_patience')
    pr = f.get('patience.restored.planner_patience')
    if RULES_OFF:
        continue
    for t in txts:
        for ln, line in enumerate(open(t, encoding='utf-8', errors='ignore').read().splitlines(), 1):
            if not (NOSAY.search(line) or CLAIM.search(line)):
                continue
            hits = []
            if rr_true and NOSAY.search(line):
                hits.append('字段里 result_returned 为真的键=%s，文本却说"没有 result"' % rr_true[:2])
            claim_hit = bool(CLAIM.search(line)) and not has_outcome and not rr_true
            if claim_hit:
                # 筛「该行是否同时提到本件的字段名或本件文件名」——甲／合档用它判"是不是在讲别的对象"
                leaf = set()
                for k in f:
                    leaf.add(k.rsplit('.', 1)[-1].split('[')[0])
                leaf.add(os.path.splitext(os.path.basename(j))[0])
                named = any(x and x in line for x in leaf)
                if RULESET == 'A' and not named:
                    claim_hit = False                                   # 甲：跨对象误伤 ⇒ 不报也不计数
                elif RULESET == 'B':
                    claim_hit = False; claim_only += 1                  # 乙：claim 级一律只计数
                elif RULESET == 'AB' and not named:
                    claim_hit = False; claim_only += 1                  # 合档：讲别的对象的只计数
            if claim_hit:
                hits.append('文本下了"已证／推翻"级结论，但该件既无 outcome 码也无回话记录 ⇒ 结论强度请人判')
            if hits:
                nflag += 1
                print('[疑点] %s ↔ %s（第 %d 行）：%s' % (os.path.basename(j), os.path.relpath(t), ln, '；'.join(hits)))
                print('        原文：%s' % line.strip()[:100])
    if pb is not None and pr is not None and pb != pr:
        nflag += 1
        print('[疑点] %s：还原未还原（before_set=%s 而 restored=%s）' % (os.path.basename(j), pb, pr))
print('[计数] json=%d 并排文本件=%d 疑点=%d 去重跳过=%d claim只计数=%d 规则档=%s（**只列不判**）' % (
    len(jsons), used_txt, nflag, dup, claim_only, 'off' if RULES_OFF else RULESET))
sys.exit(5 if nflag else 0)
PYEOF
