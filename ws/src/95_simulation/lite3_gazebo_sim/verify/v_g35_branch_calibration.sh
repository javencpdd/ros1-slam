#!/usr/bin/env bash
# v_g35_branch_calibration.sh —— 辅门 35（R20 的 span>=35）的**零节点半标定**（小美 M2R8 路由 R37 的无窗那一半；小明 2026-10-09）
#
# 她要把「辅门 35 未标定」的亏欠拆两半：活体那半＝R41 的两条真反例（要窗），零节点这半＝不等窗、现在就做。
# 关键纪律（与 STS／DWRS／BRS 同一条）：**判据逻辑住 m2_smoke.sh，本件不复制实现**——
#   四个式子（_b_abs／_b_span／ok_press／branch 标签）从**现役 SM** 里按配对括号抽出来 eval，
#   所以本件过 ＝ 在跑的那份判据的分辨力被证，而不是证了我抄的一份副本（05 B2）。
#   因此本件**不改 SM**、不触 SM/LIB 指纹 ⇒ 不跳代际（代际仍 H14）；抽不到式子即 rc=9 不判被试。
#
# 对表四态（在册原文 m2_smoke.sh:338-345 的注释口径：OR 式，但 sim2>=60 一支必须同过辅门 span>=35）：
#   abs+span ／ span ／ NONE ／ unreached。辅门的**咬合处**是 fixture F2：sim2=61 而 span=34 ⇒
#   若无辅门会算「达标」，在册式必须给 NONE＋ok_press=False（#12(a) 的「环境慢起使 sim2 自然过 60」那一型）。
#   这不是空口断言：T3 把抽出的式子里 `and _span >= 35.0` **抠掉**再跑 F2 ⇒ 必判 abs+span/达标，
#   即「阴性对照先真的红一次」（05 C4a），证明辅门确实在判据链上、不是注释里的装饰。
# 顺带把三个门值（60.0／35.0／40.0）按字面钉住：小美若改档，本件先红 ⇒ 属**判据漂移探测器**，修法是她定档后我更表，不就地放宽。
#
# 用法：bash v_g35_branch_calibration.sh      退出码（本件自有层，不与 SM／STS／DWRS 各层互认）：
#   0=对表全过；5=有 FAIL；9=抽不到现役式子或被检式子不完整（前置不成立，不判被试）
set -u
ROOT=/home/ros1-slam
SELF=$(readlink -f "$0")

LIVE_SM=$ROOT/ws/src/95_simulation/lite3_gazebo_sim/verify/m2_smoke.sh
# 第二份定义（加压档辅门的纯函数版，为 H15 接线预备）：路径由 shell 传入，python 侧不重复硬编码
PG=${G35_PGATE:-$ROOT/ws/src/95_simulation/lite3_gazebo_sim/verify/m2_press_gate.py}
# --postwiring：接线（SM 改调 press_gate()）落地后的**收尾断言**——不是"新件通过"，而是"旧那份内联定义从盘上消失"。
#   今日跑必然判红（SM 里内联式仍在）⇒ 这条红就是它以后真能拦人的凭据（05 C4a 同一口径），也是 02 坑43 的收尾判据。
if [ "${1:-}" = "--postwiring" ]; then
  SMP=${G35_SM:-$LIVE_SM}     # 与本件其余模式同一条口子（G35_SM），此处必须用 LIVE_SM：SM= 的赋值在本段之后
  [ -f "$SMP" ] || { echo "缺现役件：$SMP"; exit 9; }
  echo "=== G35 接线后收尾断言（SM 内不应再有第二份内联定义）｜被检件=$SMP sha=$(sha256sum "$SMP" | cut -c1-12) ==="
  LEFT=0
  for pat in '_b_abs = (' '_b_span = (' 'ok_press = bool(_r)'; do
    n_=$(grep -cF "$pat" "$SMP")
    if [ "$n_" = 0 ]; then echo "  [PASS] SM 内已无「$pat」（命中 0）"; else echo "  [FAIL] SM 内仍有「$pat」命中 $n_ 处 ⇒ 第二份定义未消除"; LEFT=$((LEFT+1)); fi
  done
  if grep -q 'press_gate' "$SMP"; then echo "  [PASS] SM 已引用 press_gate()"; else echo "  [FAIL] SM 未引用 press_gate() ⇒ 接线未落地"; LEFT=$((LEFT+1)); fi
  echo "=================================================="
  if [ "$LEFT" = 0 ]; then echo "POSTWIRING VERDICT: 单一源达成 rc=0（此后 T6 逐格互检退化为同源自检，可保留作回归）"; exit 0; fi
  echo "POSTWIRING VERDICT: 尚有 $LEFT 处未收口 rc=1（现状未接线时本断言就该红，不是件坏）"; exit 1
fi
SM=${G35_SM:-$LIVE_SM}     # 只有 --negctl 的自证会覆写它（缺省恒为现役件；生效性由阴性对照本身来证）

if [ "${1:-}" = "--negctl" ]; then
  # 05 C4a：新 gate 必须**真的红过一次**，而且要可复跑。做法＝拿现役 SM 的副本、抠掉辅门子句，
  #   再以该副本为被试重跑本体：F2/F6 一类断言必须翻不过去 ⇒ 本体判 rc=5。若它照样绿，说明本件测的是空气。
  [ -f "$SM" ] || { echo "缺现役件：$SM"; exit 9; }
  T=$(mktemp -d /tmp/g35_negctl.XXXXXX); trap 'rm -rf "$T"' EXIT
  sed 's/ and _span >= 35\.0//' "$SM" > "$T/m2_smoke_mut.sh"
  _n0=$(grep -cF '_span >= 35.0' "$SM"); _n1=$(grep -cF '_span >= 35.0' "$T/m2_smoke_mut.sh")
  echo "=== G35 阴性对照（被试＝抠掉辅门的 SM 副本）：在册件含辅门 $_n0 处 ⇒ 副本含 $_n1 处 ==="
  if [ "$_n0" -lt 1 ] || [ "$_n1" -ne 0 ]; then
    echo "改写未生效（$_n0→$_n1）⇒ 阴性对照空转，本模式判前置不成立 rc=9"; exit 9
  fi
  G35_SM="$T/m2_smoke_mut.sh" bash "$SELF" > "$T/red.log" 2>&1; r=$?
  echo "  被试副本判定行：$(grep -E '^    _b_abs ' "$T/red.log")"
  echo "  本体实得 rc=$r（期望 5＝有 FAIL），失败断言数=$(grep -c '^\[FAIL\]' "$T/red.log")"
  grep '^\[FAIL\]' "$T/red.log" | sed 's/^/      /'
  if [ "$r" = 5 ] && [ "$(grep -c '^\[FAIL\]' "$T/red.log")" -ge 3 ]; then
    echo "NEGCTL VERDICT: 本件能真红 rc=0（辅门确在对表面前起作用，不是注释装饰）"; exit 0
  fi
  echo "NEGCTL VERDICT: 本件抠掉辅门仍不红 ⇒ 对表无分辨力，判 rc=5"; exit 5
fi
echo "=== 辅门 35 零节点标定（SM sha=$(sha256sum "$SM" | cut -c1-12)）==="

python3 - "$SM" "$PG" <<'PYEOF'
import re, sys
sm = open(sys.argv[1], encoding='utf-8').read()
PG_PATH = sys.argv[2]

def paren_expr(text, start):
    """从 start 起找第一个 '('，返回**配对括号**内的完整表达式；收不拢返回 None。"""
    j = text.find('(', start)
    if j < 0: return None
    d = 0
    for k in range(j, len(text)):
        c = text[k]
        if c == '(': d += 1
        elif c == ')':
            d -= 1
            if d == 0: return text[j:k + 1]
    return None

# **窗口收口**：四个式子只在「SMOKE gate_press_late 回显行」前后 1500/600 字节的段内找——
#   不在全文里 find。理由：SM 里 `ok_press =` 这类赋值不止一处（别的 gate 也叫这名），全文找会把
#   **别的判据的式子**抽来对表，那测的就不是辅门 35 了（05 B2 的反面：抽错段＝测了副本）。
_a = sm.find("print('SMOKE gate_press_late=")
if _a < 0:
    print("抽不到在册回显行（print('SMOKE gate_press_late=…) ⇒ 前置不成立，不判被试（rc=9）"); sys.exit(9)
win = sm[max(0, _a - 1500):_a + 600]

def pick_paren(anchor):
    i = win.rfind(anchor)
    return paren_expr(win, i) if i >= 0 else None

def pick_rhs(anchor):
    """取窗口内**最后一次**该赋值、并取**整行等号右边的表达式**。
       为什么不能按「首个括号组」抽：在册 `ok_press = bool(_r) and (_b_abs or _b_span)` 不以括号开头，
       按括号抽会拿到 `(_r)`（＝只抽到 bool 的实参），对表就成了测另一件事——本件第一次跑就是这么被结构校核拦下的。
       为什么取最后一次：同名赋值在更早的行还有别家 gate 的（实测 SM:320 另有一条 ok_press = True 的未开档恒真赋值），
       取首个会把别条判据抽来对表。"""
    i = win.rfind(anchor)
    if i < 0: return None
    j = win.find('\n', i)
    line = win[i:j if j >= 0 else len(win)]
    if '=' not in line: return None
    r = line.split('=', 1)[1].strip()
    if '#' in r: r = r.split('#', 1)[0].strip()      # 在册这几行无行内注释；真有则不参与求值
    return r or None

E = {'_b_abs': pick_rhs('_b_abs ='), '_b_span': pick_rhs('_b_span ='),
     'ok_press': pick_rhs('ok_press ='), 'branch': pick_paren("('unreached'")}
miss = [k for k, v in E.items() if not v]
if miss:
    print("抽不到现役式子：%s ⇒ 前置不成立，不判被试（rc=9）" % ",".join(miss)); sys.exit(9)
# 结构校核（坑34 的口径：口子/抽取的生效性本身必须是被测项）：抽来的 ok_press 必须真的同时引用两支支旗，
#   且 branch 必须引用 reached 与两支——否则对表就是在测我另写的一套，本件直接判 9 而不是硬测。
_str = []
if '_b_abs' not in E['ok_press'] or '_b_span' not in E['ok_press']: _str.append('ok_press 未同引两支支旗')
if 'not _r' not in E['branch'] or '_b_abs' not in E['branch'] or '_b_span' not in E['branch']: _str.append('branch 未同引 reached 与两支')
if _str:
    print("抽取式结构不符（抽到了不相干的式子）：%s ⇒ 不判被试（rc=9）" % "；".join(_str)); sys.exit(9)
print("  抽取四面（逐字来自现役 SM，非本件自写）：")
for k in ('_b_abs', '_b_span', 'ok_press', 'branch'):
    print("    %-9s = %s" % (k, E[k]))

# 门值按字面在册（判据漂移探测）
gate = [(w, w in (E['_b_abs'] or '') + (E['_b_span'] or '')) for w in ('60.0', '35.0', '40.0')]

def judge(r, sim2, span, src=None):
    """把抽来的式子放进同一个命名空间 eval——判定逻辑与被试同源；src 供阴性对照改写用。"""
    s = dict(E); s.update(src or {})
    ns = {'_r': r, '_sim2': sim2, '_span': span}
    exec('_b_abs = %s'   % s['_b_abs'], ns)
    exec('_b_span = %s'  % s['_b_span'], ns)
    exec('_ok = %s'      % s['ok_press'], ns)
    exec('_br = %s'      % s['branch'], ns)
    return ns['_br'], bool(ns['_ok'])

T = 0; F = 0; CP = 0
def chk(name, want, got):
    global T, F, CP
    T += 1
    if got == want: CP += 1; print("[PASS] %s ⇒ %s" % (name, got))
    else: F += 1; print("[FAIL] %s 期望=%s 实得=%s" % (name, want, got))

print("--- T1 四态对表（abs+span／span／NONE／unreached 必须各归各位）---")
tbl = [
    ("F1 sim2=61 span=36 → abs+span 且达标",                  True, 61.0, 36.0,  'abs+span', True),
    ("F2 sim2=61 span=34 → NONE 且达标 False（辅门咬合处）",   True, 61.0, 34.0,  'NONE', False),
    ("F3 sim2=50 span=41 → span 且达标",                      True, 50.0, 41.0,  'span', True),
    ("F4 sim2=50 span=20 → NONE 且达标 False",                True, 50.0, 20.0,  'NONE', False),
    ("F5 reached=False → unreached（一票否决，读数再高也不达标）", False, 99.0, 99.0, 'unreached', False),
    ("F6 边界 sim2=60 span=35 → abs+span（>= 含等号）",        True, 60.0, 35.0,  'abs+span', True),
    ("F7 边界 sim2=59.99 span=34.99 → NONE",                  True, 59.99, 34.99, 'NONE', False),
    ("F8 边界 sim2=59.99 span=40.0 → span（OR 另一支单独成立）", True, 59.99, 40.0, 'span', True),
]
for name, r, s2, sp, wb, wok in tbl:
    chk(name, (wb, wok), judge(r, s2, sp))

print("--- T2 辅门可指认：F1(36) 与 F2(34) 同 sim2，判定必须翻转（否则辅门不在链上）---")
b1, o1 = judge(True, 61.0, 36.0); b2, o2 = judge(True, 61.0, 34.0)
chk("T2 同 sim2 下 span 跨 35 使 branch 与达标同时翻转", True, (b1 != b2) and (o1 != o2))

print("--- T3 阴性对照（抠掉 `and _span >= 35.0` 后重跑 F2 ⇒ 期望 abs+span/达标，即原表在此必红）---")
mut_expr = re.sub(r'\s*and\s+_span\s*>=\s*35\.0', '', E['_b_abs'])
if mut_expr == E['_b_abs']:
    chk("T3 改写生效（抽出的 _b_abs 里应能找到辅门子句）", True, False)
else:
    print("    改写后（仅供阴性对照，不写回任何件）：%s" % mut_expr)
    mb, mok = judge(True, 61.0, 34.0, src={'_b_abs': mut_expr})
    chk("T3 抠辅门后 F2 翻成 abs+span 且达标（说明在册式里辅门确实在判）", ('abs+span', True), (mb, mok))
    chk("T3 与正表同输入不同结论（分辨力自证）", True, (mb, mok) != (b2, o2))

print("--- T4 门值字面在位（60.0／35.0／40.0 任一被改 ⇒ 本件先红，更表须由小美定档后做）---")
for w, hit in gate:
    chk("T4 门值 %s 在现役式子里" % w, True, hit)

print("--- T5 同源锚（抽取式必须与在册回显行同处一段，防「我抽的是另一段」）---")
chk("T5 抽取段确为回显行所在那一段（窗口收口生效）", True,
    ("branch=%s" in win) and ("('unreached'" in win) and ("_b_abs =" in win))

print("--- T6 双定义逐格同值（H15 接线落地前的防漂移闸）---")
# 现状：同一判据有**两处在册定义**——SM 的内联式（H14 现役、活体真正走的那份）与 m2_press_gate.press_gate()
#   （纯函数版，其件头自记「SM 接线到本件后此表才算活体同一份式子」⇒ 接线前两者可各自漂移而各自的表都绿）。
#   G35 的 T1～T5 只核 SM 那一处、m2_press_gate 的 11 组只核它自己一处 ⇒ 补这组逐格差分，把「两处同值」变成断言而不是期望。
import importlib.util, os, tempfile
def load_pg(path, tag):
    spec = importlib.util.spec_from_file_location("m2_press_gate_%s" % tag, path)
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m); return m
if not os.path.exists(PG_PATH):
    chk("T6 第二份定义在位", True, False); print("    [读数] 找不到 %s" % PG_PATH)
else:
    import hashlib
    print("    两处定义：SM=%s（sha %s）｜%s（sha %s）" % (
        os.path.basename(sys.argv[1]), hashlib.sha256(sm.encode()).hexdigest()[:12],
        os.path.basename(PG_PATH), hashlib.sha256(open(PG_PATH,'rb').read()).hexdigest()[:12]))
    GRID = [(r, s2, sp) for r in (True, False)
            for s2 in (20.0, 59.9, 60.0, 61.0, 62.0, 99.0)
            for sp in (10.0, 34.9, 35.0, 36.0, 39.9, 40.0, 41.0)]
    def diff(mod):
        bad = []
        for r, s2, sp in GRID:
            b1, o1 = judge(r, s2, sp)          # SM 抽出来的式子（branch, ok）
            o2, b2 = mod.press_gate(r, s2, sp)  # 纯函数版（ok, branch）
            if (o1, b1) != (o2, b2): bad.append((r, s2, sp, (o1, b1), (o2, b2)))
        return bad
    mism = diff(load_pg(PG_PATH, "real"))
    chk("T6 逐格同值（%d 格）不一致=0" % len(GRID), 0, len(mism))
    for m in mism[:4]: print("      不一致格 reached=%s sim2=%.1f span=%.2f｜SM=%s 纯函数=%s" % (m[0], m[1], m[2], m[3], m[4]))
    # T6b 阴性对照：把纯函数版的辅门 35.0 改成 30.0（只在临时副本上，不回写任何在册件）⇒ T6 必须报不一致
    src = open(PG_PATH, encoding='utf-8').read()
    mut = src.replace("TOL_SUB_SPAN = 35.0", "TOL_SUB_SPAN = 30.0")
    if mut == src:
        chk("T6b 改写生效（找到辅门行）", True, False)
    else:
        tfd = tempfile.mkdtemp(prefix="g35_t6b_")
        tp = os.path.join(tfd, "pg_mut.py"); open(tp, "w", encoding='utf-8').write(mut)
        mism2 = diff(load_pg(tp, "mut"))
        chk("T6b 辅门改一档后必须报不一致（本组=%d 格）" % len(mism2), True, len(mism2) > 0)
        print("      不一致样例：reached=%s sim2=%.1f span=%.2f｜SM=%s 改后=%s" % tuple(mism2[0]) if mism2 else "      （无）")
        import shutil; shutil.rmtree(tfd, ignore_errors=True)

print("==================================================")
print("G35 CALIBRATION: PASS=%d FAIL=%d / %d" % (CP, F, T))
print("  覆盖面：四态对表（T1）／辅门翻转可指认（T2）／抠辅门必真红的阴性对照（T3）／门值字面在册（T4）／抽取式与回显行同源（T5）／**双定义逐格同值与改档必红（T6／T6b）**")
print("  活体那半（辅门在真跑里被非零态读过）仍留 R41 两条真反例，本件不冒充已验")
sys.exit(0 if F == 0 else 5)
PYEOF
rc=$?
echo "G35 rc=$rc"
exit $rc
