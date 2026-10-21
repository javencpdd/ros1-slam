#!/usr/bin/env python3
# r534_kit.py —— ★批190＝新活体窗 R534 的件面★：复制 r530 四件→落 G1—G12→★全在 /tmp★先验、通过才入仓
import io, os, re, shutil, subprocess, glob

R = "/home/jack/ysc-code/ros1-slam"
SRCW = glob.glob(os.path.join(R, "note/进度/SCAN集成/证据/log/raw/r530_*"))[0]
DIRNAME = "r534_活体窗落G1到G12件内改形_exit实参与键校验给红档与字段非数字给红档与段标题挂条件与case一处导出喂本体与面判定真取与正控进志与差集进窗与不变量进合取项与两遍同输入"
print("新目录名字节长＝", len(DIRNAME.encode()))
OUT = "/tmp/r534_kit/" + DIRNAME
shutil.rmtree("/tmp/r534_kit", ignore_errors=True)
os.makedirs(OUT)
rd = lambda p: io.open(p, encoding="utf-8", errors="replace").read()
wr = lambda p, s: io.open(p, "w", encoding="utf-8").write(s)

def patch(tag, s, pairs, must=True):
    for a, b in pairs:
        if s.count(a) != 1:
            print("★未命中★", tag, repr(a[:46]), "count＝", s.count(a))
            if must: raise SystemExit(2)
        s = s.replace(a, b, 1)
    print("过", tag)
    return s

# ══════ 1）01_复读件_r534.py ＝ G1（退出点入断言）＋ G2（键校验给红档）══════
recheck = rd(os.path.join(SRCW, "01_复读件_r530.py"))
recheck = patch("复读件·G1＋G2", recheck, [
 ('RC_RED = RC_BY_BR[BR_BY_SWAP]',
  'def bye(br):\n'
  '    """★G1／G2/R534＝退出点＝唯一一枚函数★＝①键不在名册⇒★红档 5＋点名★（旧形＝KeyError＝崩）'
  '②★在退出那一行上★断言「名里含本支码」＝实参表达式本身被证（旧形＝断言只到变量层、'
  '判据面 H10／H11 跑到印 9 退 0 零翻红）"""'
  '    if br == "zero":\n        sys.exit(0)\n'
  '    if br not in RC_BY_BR:\n'
  '        print("   ★件自身缺陷＝分支名不在名册＝%s ⇒ 按红档退出（★G2＝不崩★）★" % br)\n        sys.exit(5)\n'
  '    code = RC_BY_BR[br]\n'
  '    assert str(code) in NAME_BY_BR[br], "★退出点＝名不含本支码＝名↔码在实参层脱钩（G1/R534）★"\n'
  '    assert code in (5, 9), "★退出点＝码不属在册档（G1/R534）★"\n'
  '    sys.exit(code)\n\n\nRC_RED = RC_BY_BR[BR_BY_SWAP]'),
 ('    sys.exit(RC_RED)', '    bye(BR_BY_SWAP)'),
 ('sys.exit(0 if PREMISS == 0 else PREMISS)', 'bye("zero" if PREMISS == 0 else "premiss")'),
])
wr(os.path.join(OUT, "01_复读件_r534.py"), recheck)

# ══════ 2）_cmp_r534.py ＝ G5（段标题挂条件）＋ G11（rc_不变量 进合取项）══════
cmpy = rd(os.path.join(SRCW, "_cmp_r530.py"))
old_title = [l for l in cmpy.split("\n") if l.startswith('print("\\n===〔一b〕')][0]
new_title = ('INV_DONE = sum(1 for a, b in [("A", "Ap"), ("A", "C"), ("A", "REF"), ("REF", "A")] '
             'if (S[a]["ox"] == S[b]["ox"] and S[a]["oy"] == S[b]["oy"]))   '
             '# ★G5/R534＝把 INV_DONE 算到★标题之前★＝标题里那句话必须由它导出（旧形＝标题硬编码「四对全异架⇒未行使」＝S35）\n'
             'print("\\n===〔一b〕★S22/R530＋G5/R534＝不变量自带消费者★（r521q：`inv_ok` 算而无消费＝死变量；'
             '★标题这句由 INV_DONE 现取＝%s★）===" % ("★四对全异架⇒本窗未行使★" if INV_DONE == 0 else "★本窗行使 %d 对⇒不得再打「未行使」★" % INV_DONE))')
cmpy = patch("对照件·G5", cmpy, [(old_title, new_title)])
old_line = [l for l in cmpy.split("\n") if l.startswith("INV_DONE = sum(")][1]
cmpy = cmpy.replace(old_line, "# ★G5/R534＝这行已上移到标题之前★（留此注释＝★旧位置不再重复计算★）", 1)
cmpy = patch("对照件·G11", cmpy, [
 ('RCM = [("合取一', 'RCM = [("合取七★rc_不变量＝0★（G11/R534＝不变量枚★进合取项★、不再只在打印面）", RC_INV == 0), ("合取一'),
 ('★0＝下表六枚合取项逐枚真取到且全真', '★0＝下表★七枚★合取项逐枚真取到且全真（G11/R534 起＝六枚→七枚）'),
])
wr(os.path.join(OUT, "_cmp_r534.py"), cmpy)

# ══════ 3）06_判读件_r534.py ＝ G3（字段非数字给红档）＋ G4（哨兵＋全文断言不含 None）＋ G8（面判定真取）══════
probe = rd(os.path.join(SRCW, "06_判读件_r530.py"))
probe = patch("判读件·G3", probe, [
 ('    okl = set(int(x) for x in lrows.split(",")) <= set(ln)',
  '    okl = rows_of(lrows, k) <= set(ln)   # ★G3/R534＝字段非数字⇒记红档、不 int 崩★（旧形＝ValueError＠件:67）'),
 ('def grab(path, pat, field):',
  'def rows_of(txt, keyname):\n'
  '    """★G3/R534＝表字段里出现非数字（如「无」「缺」）⇒★红档点名★、绝不 ValueError 崩★"""\n'
  '    v = set()\n'
  '    for x in txt.split(","):\n'
  '        if x.isdigit():\n            v.add(int(x))\n'
  '        else:\n            miss.append("★表字段非数字＝%s ‖ 键＝%s★（G3＝按红档、不按崩）" % (x, keyname))\n'
  '    return v\n\n\n'
  'def grab(path, pat, field):'),
])
probe = patch("判读件·G4", probe, [
 ('        return [None]\n    return h',
  '        return ["★缺★"]   # ★G4/R534＝哨兵值、不是 None★（旧形＝打印面出「第 None 行」＝S27 残留）\n    return h'),
 ('txt = "\\n".join(O) + "\\n"',
  'txt = "\\n".join(O) + "\\n"\n'
  'assert "None" not in txt, "★G4/R534＝输出串里出现 None＝查不到被当成读数★（旧形＝件:34 返回 [None]→件:90 打印）"'),
])
probe = patch("判读件·G8", probe, [
 ('"宿主" if "jack" in ROOT else "容器"',
  'FACE_TAG'),
 ('P("=== 06_判读（',
  'import datetime\n'
  '_off = datetime.datetime.now().astimezone().utcoffset()\n'
  '_hrs = int(_off.total_seconds() // 3600) if _off else 0\n'
  '# ★G8/R534＝面判定★真取★＝取本机 UTC 偏移（＋8＝宿主 CST、0＝容器 UTC），★不再用路径子串猜★（旧形＝"jack" in ROOT＝S42）\n'
  'FACE_TAG = ("宿主（UTC+%%d）" %% _hrs) if _hrs == 8 else ("容器（UTC%%+d）" %% _hrs if _hrs == 0 else "★偏移非 8/0＝面未定＝不发面名★")\n'
  'P("=== 06_判读（'),
])
wr(os.path.join(OUT, "06_判读件_r534.py"), probe)

# ══════ 4）00_r534_window.sh ＝ G6 G7 G9 G10 G12 ＋ 件名改指 ══════
win = rd(os.path.join(SRCW, "00_r530_window.sh")).replace("_r530.py", "_r534.py").replace("R530 改形", "R534 改形")
case_line_s1 = [l for l in win.split("\n") if l.startswith('case " $YIELD_LIST " in *" run "*|*" 05_cmp.txt "*')][0]
case_line_98 = [l for l in win.split("\n") if l.startswith('case " $YIELD_LIST " in *" run "*|*" 00_window_log.txt "*')][0]
win = patch("窗件·G6", win, [
 (case_line_s1, 'blacklist_hit "★S1★"; if [ $? = 1 ]; then echo "★S1 守卫不过＝让路清单里还有承重件＝当场中止★" >&2; exit 8; fi'),
 (case_line_98, 'blacklist_hit "★步98★"; if [ $? = 1 ]; then echo "★步98 守卫不过＝让路清单里含其它承重件＝当场中止★" >&2; exit 8; fi'),
 ('SKIPPED=""',
  '# ★G6/R534＝黑名单★一处导出★＝两处守卫与自测★都喂这同一枚函数★（旧形＝件:123 ‖ 349 ‖ 358 ★三份复写★＝'
  '判据面 S38；施工侧另证＝复写那枚自测★拦 8/8、比本体 7/8 ‖ 6/8 还宽★＝复写会给出「过严」的假象）\n'
  'BL_NAMES="run 05_cmp.txt 00_window_log.txt 01_preflight.txt 03_delta.txt 07_复读.txt"\n'
  'blacklist_hit(){\n  local FROM="$1" f hit=0\n'
  '  for f in $BL_NAMES; do case " $YIELD_LIST " in *" $f "*) hit=1;; esac; done\n'
  '  [ "$hit" = 1 ] && echo "★黑名单命中（★本体函数★＝$FROM 那一道守卫）★" >&2\n  return $hit\n}\nSKIPPED=""'),
])
bl_test_line = [l for l in win.split("\n") if l.startswith("BL_TEST=0; for nm")][0]
win = patch("窗件·G6 自测喂本体", win, [
 (bl_test_line,
  'BL_TEST=0; for nm in 01_preflight.txt 03_delta.txt 07_复读.txt; do SAVED_YL="$YIELD_LIST"; YIELD_LIST="$nm"; blacklist_hit "★自测★" >/dev/null 2>&1; [ $? = 1 ] && BL_TEST=$((BL_TEST+1)); YIELD_LIST="$SAVED_YL"; done   '
  '# ★G6/R534＝自测★喂本体★（旧形＝喂第三枚复写的 case＝S38）'),
])
win = patch("窗件·G9", win, [
 ('[ "$N_HIT" -ge 1 ] && [ "$N_HIT" = "$N_APPEAR" ] || echo "★F5 元字符正控不过',
  'F5BAD=0; { [ "$N_HIT" -ge 1 ] && [ "$N_HIT" = "$N_APPEAR" ]; } || { F5BAD=1; echo "★F5 元字符正控不过'),
])
i = win.find('F5BAD=1; echo "★F5 元字符正控不过')
j = win.find("\n", i)
win = win[:j] + "\n[ \"$F5BAD\" = 1 ] && echo \"★F5 正控报警★（同上句）\" | tee -a \"$R/00_window_log.txt\" > /dev/null   # ★G9/R534＝报警★既 tee 进窗志、又留 F5BAD 位★（旧形＝只 echo stderr、窗志 0 枚＝S41）" + win[j:]
win = patch("窗件·G12", win, [
 ('LOGSHA_BEFORE=$(sha256sum "$R/00_window_log.txt" 2>/dev/null | cut -c1-16)',
  'cp "$R/00_window_log.txt" "$R/00_window_log.冻结输入.txt"   # ★G12/R534＝第二遍复跑读★同一份冻结输入★（旧形＝两遍读的窗志不同份＝S21）\nLOGSHA_BEFORE=$(sha256sum "$R/00_window_log.txt" 2>/dev/null | cut -c1-16)'),
])
k = win.find("python3 -u \"$R/_cmp_r534.py\"")
line_end = win.find("\n", k)
first_call = win[k:line_end]
second = win.find("python3 -u \"$R/_cmp_r534.py\"", line_end + 1)
if second < 0:
    print("★注意★＝复跑那遍的调用形不同，需人工定位"); 
else:
    le2 = win.find("\n", second)
    line2 = win[second:le2]
    assert '"$R/00_window_log.txt"' in line2, "★G12 复跑那遍的入参形不同＝未改到＝不发★"
    win = win[:second] + line2.replace('"$R/00_window_log.txt"', '"$R/00_window_log.冻结输入.txt"', 1) + win[le2:]
    print("过 窗件·G12 复跑入参改指冻结件")
win = patch("窗件·G10", win, [
 ('RESTSAME=$([ "$L1R" = "$L2R" ] && echo True || echo False)',
  'RESTSAME=$([ "$L1R" = "$L2R" ] && echo True || echo False)\n'
  '# ★G10/R534＝「差集 ⊆ 命中行」那枚对表★进窗★（旧形＝只在离窗判读件里算＝S23）★＝用★同一枚★ TOK_TIME 取命中行、再用行位比较差集行\n'
  'HITROWS=$(grep -nE "$TOK_TIME" "$R/05_cmp.txt" | cut -d: -f1 | paste -sd, -)\n'
  'D0ROWS=$(diff "$R/05_cmp.txt" "$R/05_cmp_复跑.txt" | grep -oE "^[0-9]+" | sort -u | paste -sd, -)\n'
  'SUBSET=$([ -n "$D0ROWS" ] && [ -z "$(comm -23 <(echo "$D0ROWS" | tr , \\n | sort) <(echo "$HITROWS" | tr , \\n | sort))" ] && echo True || echo False)'),
 ('say "  对照件第一遍 rc=$CRC',
  'say "  ★G10/R534＝差集所在行（$D0ROWS）⊆ 排除式命中行（$HITROWS）＝$SUBSET★（★不相等⇒差集落在别处＝这枚排除式没盖住真差★）"\nsay "  对照件第一遍 rc=$CRC'),
])
win = patch("窗件·G7＋文案元字符", win, [
 ('★F7a/R530＝该数取自★被试命令★的 $?（上一行）', '★F7a/R534＝该数取自★上一行那条 SRC=$? 的被试命令本身★'),
 ('（期望 ★0★＝件:328 已改取变量本身', '（期望 ★0★＝本窗 r534 件:331 一带已改取变量本身；★旧句里的「件:328」＝r525 那窗的行号＝跨窗引用＝G7/R534 起必须带窗名'),
])
wr(os.path.join(OUT, "00_r534_window.sh"), win)
print("★件已生成★＝", sorted(os.listdir(OUT)))
