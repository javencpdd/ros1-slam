#!/usr/bin/env python3
# _kit_r521.py —— 批177 窗套件生成器＝从 r517 目录复制 12 枚链接件、改名（r517/R517 两大小写都替、r517q 字面先保护）、
#   再把★六枚可件内修的缺席格★落进件内（r517q 缺席清单 S5 ‖ S6 ‖ S2 ‖ S3 ‖ S10 ‖ S1）。
# ★姿势照在册★＝逐枚按★锚点行★定位并断言★恰好一枚命中★再落盘；改完必过 bash -n ／ ast.parse ／ 件内 R= 路径＝盘上路径 三段自证。
import ast
import io
import os
import re
import shutil
import subprocess

RAW = "/home/jack/ysc-code/ros1-slam/note/进度/SCAN集成/证据/log/raw"
SRC_NAME = ("r517_活体窗按r514q缺席清单落六枚件内修_真调让路函数并报分支命中枚数与自测挪到roscore之前与9档拆名与"
            "sha随句带行数与栏形新名与源面从句")
DST_NAME = ("r521_活体窗按r517q缺席清单落六枚件内修_两支红档名由一枚码查表导出与独立来源从句改整枚data与盲区枚数带遍历域与"
            "复跑判据改除时刻行外逐行同与判读锚由件内现取与真目录撞名让路载体")
SRC = os.path.join(RAW, SRC_NAME)
DST = os.path.join(RAW, DST_NAME)
assert os.path.isdir(SRC), "源目录不在位"
assert DST_NAME.count("r517q") == 1, "新目录名应含 r517q 恰一枚（被保护的判据面引用）"
if os.path.exists(DST):
    junk = os.listdir(DST)
    assert "run" not in junk and "00_window_log.txt" not in junk and "_sha256_manifest.txt" not in junk, \
        "目标目录已有产物＝不覆盖（先人工处置）：" + str(junk)
    shutil.rmtree(DST)

KEEP = [
    "00_r517_window.sh", "01_复读件_r517.py", "02_make_copies_r517.py", "03_sub_elev_count.py",
    "_cmp_r517.py", "pub_rect_probe_r517.py", "sub_costmap_dump_r517.py", "sim_nav_smoke_r517.sh",
    "global_costmap_params_A_r517.yaml", "global_costmap_params_C_r517.yaml",
    "mbf_navigation_r517.launch", "mode_nav_sim_r517.launch",
]
for f in KEEP:
    assert os.path.isfile(os.path.join(SRC, f)), "源件缺失：" + f
os.makedirs(DST)
for f in KEEP:
    shutil.copyfile(os.path.join(SRC, f), os.path.join(DST, f.replace("r517", "r521")))
print("复制 %d 件 -> %s" % (len(KEEP), DST_NAME))


def rd(p):
    return io.open(p, encoding="utf-8").read()


def wr(p, s):
    io.open(p, "w", encoding="utf-8").write(s)


# ---------- 2. 改名（★大小写两端都替★＝批170 那枚的对偶：本批首跑栽在只替小写；★r517q 先保护★＝防替换链咬到同族子串） ----------
for fn in sorted(os.listdir(DST)):
    p = os.path.join(DST, fn)
    if not os.path.isfile(p):
        continue
    t = rd(p)
    assert "r517q" not in t and "R517q" not in t, "复制件 %s 内含 r517q/R517q（替换链会咬到同族子串）" % fn
    t = t.replace("r517q", "@@KEEPQ@@").replace("R517q", "@@KEEPQ@@")
    t = t.replace("r517", "r521").replace("R517", "R521")
    t = t.replace("@@KEEPQ@@", "r517q")
    wr(p, t)
print("改名：内容 r517/R517 -> r521/R521（r517q 字面在复制件内 0 枚＝已断言）")

# ---------- 2b. 件内整串路径 → 新目录名（★先通用替换、再整串换名★＝倒过来会把新名里的 r517 咬成 r521 之外的东西） ----------
OLD_DIRNAME = SRC_NAME.replace("r517", "r521")
hit = 0
for fn in sorted(os.listdir(DST)):
    p = os.path.join(DST, fn)
    if not os.path.isfile(p):
        continue
    t = rd(p)
    n = t.count(OLD_DIRNAME)
    if n:
        wr(p, t.replace(OLD_DIRNAME, DST_NAME))
        hit += n
        print("  路径串改写：%s 内 %d 处旧目录名 -> 新目录名" % (fn, n))
assert hit >= 1, "★没有任何一处旧目录名被改写＝件内路径与盘上目录名会不一致（这一枚当年真翻红过）★"


def line_edit(path, needle_start, needle_end, new_lines, tag, whole=False):
    """★按锚点行定位★：断言 needle 各命中恰一枚，再整段替换 [start..end]（whole=True 时 end 忽略）。"""
    lines = rd(path).split("\n")
    si = [i for i, l in enumerate(lines) if needle_start in l]
    assert len(si) == 1, "改形[%s] 起始锚命中 %d 次（应为 1）｜%s" % (tag, len(si), needle_start[:60])
    if whole:
        span = [si[0]]
    else:
        ei = [i for i, l in enumerate(lines) if needle_end in l]
        assert len(ei) == 1, "改形[%s] 结束锚命中 %d 次（应为 1）｜%s" % (tag, len(ei), needle_end[:60])
        assert ei[0] >= si[0], "改形[%s] 锚序倒了" % tag
        span = [si[0], ei[0]]
    out = lines[:span[0]] + new_lines + (lines[span[-1] + 1:] if not whole else lines[span[0] + 1:])
    wr(path, "\n".join(out))
    print("  改形[%s] 落盘 ✔（原 %d 行 -> 新 %d 行）" % (tag, len(span), len(new_lines)))


W = os.path.join(DST, "00_r521_window.sh")
P1 = os.path.join(DST, "01_复读件_r521.py")
PB = os.path.join(DST, "pub_rect_probe_r521.py")
CM = os.path.join(DST, "_cmp_r521.py")

# ---------- 3. 六枚件内修 ----------
# ★S5★＝两支红档的「打印名」与 sys.exit 原本共★四枚手写字面量★（名两处＋码两处）⇒ 改成★码与名共一枚变量、名由码查表导出★
line_edit(P1, "if not os.path.exists(cited):", "sys.exit(9)", [
    'RC_NAME = {5: "负例分支＝替换自测（设计来翻红的那支）", 9: "前置缺失档＝被引件或清单出处件不在位"}',
    'RC_RED = 5 if SWAP else 9   # ★S5（r517q 缺席清单 S5）＝红档的★码★与★名★共一枚变量；名由码查表导出、不再两处手写★',
    "if not os.path.exists(cited):",
    '    assert RC_RED in RC_NAME, "★退出码 %s 不在名册里＝打印名无从导出★" % RC_RED',
    '    print("   被引件不在位＝%s ⇒ rc_复读=%s（★%s＝%s★；★S5/R521＝这一枚名由码查表导出、不靠人工同步★）" % (cited, RC_RED, RC_RED, RC_NAME[RC_RED]))',
    '    print("   ★本件即将以 rc=%s 退出（R517 立的两枚：负例红进码＝5 ‖ 前置缺失＝9；★R521 改形①＝名与码同源★）★" % RC_RED)',
    "    sys.exit(RC_RED)",
], "S5 名由码查表导出")

# ★S6★＝独立来源二那句从句写的是「T 段格集」，式子实为★整枚已发布数组★⇒ 从句改口（不改式子、不改语义）
line_edit(PB, "lethal_recount = sum(1 for v in data if v == 100)", None, [
    "    lethal_recount = sum(1 for v in data if v == 100)   # ★独立来源二＝对已发布数组逐格重数★（★S6/R521 辖域从句更正★＝本件进程内、data ★序列化前那一刻的整枚已发布数组★（宽×高全格、★不按 T 段过滤★）；★rect≡T 是打印面 n_T==N 那枚的产物、不是这枚式子的辖域★；★仍不构成对 ROS 传输／订阅面的第二证★）",
], "S6 从句改整枚 data", whole=True)

# ★S2★＝「盲区枚数」那行只说「两态任一下标越界的格」、不带遍历域 ⇒ 域随句并报（式子里的遍历根＝规范架 1000×1000）
line_edit(CM, 'print("     盲区枚数＝%d', None, [
    '    print("     盲区枚数＝%d（★遍历域＝规范架 1000×1000＝1000000 格（S2/R521＝域随句并报、本检查★只在该域内★可见）‖ 该域内两态任一下标越界的格、本检查对其不可见★）" % blind)',
], "S2 盲区枚数带遍历域", whole=True)

# ★S3★＝件把 wall-clock 打进自己的输出 ⇒「两遍 sha 相同＝真同值」★永不为真★；判据改成★除时刻行外逐行差★并把时刻行行位随句报出
line_edit(W, 'DIFFN=$(diff "$R/05_cmp.txt" "$R/05_cmp_复跑.txt"', None, [
    'DIFFN=$(diff "$R/05_cmp.txt" "$R/05_cmp_复跑.txt" 2>/dev/null | grep -c "^[<>]")',
    "# ★S3（r517q 缺席清单 S3）＝本件把★遍次与时刻★打进自己的输出 ⇒ 两遍 sha★必然不同★、「sha 相同＝真同值」这枚判据★永不为真★",
    "#   ⇒ 判据换成这一枚＝★除时刻行外逐行差★，并把★时刻行的行位★随句报出（差在哪一行不再靠人眼找）",
    "TSROW=$(grep -n '★遍次（件自身带）' \"$R/05_cmp.txt\" | cut -d: -f1 | paste -sd, -)",
    "TSROW2=$(grep -n '★遍次（件自身带）' \"$R/05_cmp_复跑.txt\" | cut -d: -f1 | paste -sd, -)",
    "DIFFX=$(diff <(grep -v '★遍次（件自身带）' \"$R/05_cmp.txt\") <(grep -v '★遍次（件自身带）' \"$R/05_cmp_复跑.txt\") 2>/dev/null | grep -c \"^[<>]\")",
], "S3 前置变量", whole=True)

line_edit(W, 'say "  对照件第一遍 rc=$CRC', None, [
    'say "  对照件第一遍 rc=$CRC ‖ ★离窗复跑第二遍 rc=$CRC2★ ‖ 两遍逐行差＝$DIFFN 行（★行位口径＝diff 两侧各计一行★） ‖ ★S3/R521＝除时刻行外逐行差＝$DIFFX 行★（★判据取这一枚；sha 相同这枚永不为真★） ‖ 时刻行行位＝第一遍[$TSROW] ‖ 第二遍[$TSROW2]（★两枚行位相同才算差只长在时刻串上、不长在别处★）"',
], "S3 判据句子", whole=True)

# ★S10★＝判读面的行号锚★由件内现取★并打进窗志（历版手写错过：八栏被抄成 05_cmp 第 32—35 行）
line_edit(W, 'if [ "$DIFFN" != "0" ]; then diff', None, [
    'say "== 步10c ★S10/R521＝判读锚由件内现取★（手写锚错过一次：八栏被抄成第 32—35 行）⇒ 下面这些行号由★本窗件★打出，判读面只准抄这里、不准凭记忆写★★"',
    'for pat in "──【P100 行】" "──【P90 行】" "G1 闸" "判词" "注：皆致命" "★遍次（件自身带）"; do',
    '  CROW=$(grep -n -F "$pat" "$R/05_cmp.txt" 2>/dev/null | cut -d: -f1 | paste -sd, -)',
    '  LROW=$(grep -n -F "$pat" "$R/00_window_log.txt" 2>/dev/null | cut -d: -f1 | paste -sd, -)',
    '  say "  锚＝$pat ‖ 05_cmp.txt 行号＝${CROW:-（无）} ‖ 窗志行号＝${LROW:-（无）}（★两枚载体不互顶、各报各的★）"',
    "done",
    'if [ "$DIFFN" != "0" ]; then diff "$R/05_cmp.txt" "$R/05_cmp_复跑.txt" | head -14 | sed \'s/^/    差｜/\' | tee -a "$R/00_window_log.txt"; fi',
], "S10 判读锚由件内现取", whole=True)

# ★S1★＝撞名让路在★真 $R 目录★上从未被走过一次 ⇒ 用★非承重探针件★在真目录上跑两圈（承重件先被踢出让路清单、并有中止守卫）
line_edit(W, 'say "== 步0 重跑让路★', None, [
    'say "== 步0 重跑让路★＝本遍＝第 $((RUNS+1)) 遍 ‖〈上一遍遗留〉已改名让路＝${SKIPPED:-（无・本目录首次跑）} ‖〈本遍自建〉＝${BUILT:-（无）}（★甲已修＝建目录在让路之后⇒这一枚不进让路清单★）‖ 遍次来源＝$R/.runs（★预检通过才自增★）★"',
    "# ★S1（r517q 缺席清单 S1）★＝「撞名让路」此前只在★/tmp scratch★上走过、★真 $R 目录★上 0 枚载体 ⇒ 本步在★真目录★上跑两圈",
    "#   ★清单临时只含一枚非承重探针件★＝六枚承重件（含 00_window_log.txt 与 run/）此刻★不在★让路清单里；守卫不过⇒当场 exit 8（★绝不允许把窗志搬走★）",
    "#   ★命令替换里的函数对全局 YIELDED 的赋值不回父壳★⇒ 这一格★只报 rc 与命中、不报「让开枚数」★（追加一百零七⑤）",
    'S1_SAVED_LIST="$YIELD_LIST"',
    'YIELD_LIST="zz_probe.txt"',
    '[ "$(printf "%s\\n" $YIELD_LIST | wc -l)" = 1 ] || { echo "★S1 守卫不过＝让路清单不止一枚★" >&2; exit 8; }',
    'case " $YIELD_LIST " in *" run "*|*" 05_cmp.txt "*|*" 00_window_log.txt "*) echo "★S1 守卫不过＝让路清单里还有承重件＝当场中止★" >&2; exit 8;; esac',
    'touch "$R/zz_probe.txt"',
    'S1A_ERR=$(yield_prev "$R" "$STAMP" 2>&1 >/dev/null); S1A=$?',
    'S1A_HITS=$(printf "%s\\n" "$S1A_ERR" | grep -c "撞名即停")',
    'touch "$R/zz_probe.txt"',
    'S1B_ERR=$(yield_prev "$R" "$STAMP" 2>&1 >/dev/null); S1B=$?',
    'S1B_HITS=$(printf "%s\\n" "$S1B_ERR" | grep -c "撞名即停")',
    'S1B_HITSO=$(printf "%s\\n" "$S1B_ERR" | grep -o "撞名即停" | wc -l)',
    'S1PREV=$(ls -a "$R" | grep -c "^\\.prev_.*zz_probe")',
    'YIELD_LIST="$S1_SAVED_LIST"',
    '[ -f "$R/00_window_log.txt" ] && [ -d "$R/run" ] || { echo "★S1 后检不过＝承重件被让路函数搬走了★" >&2; exit 8; }',
    'say "  ★S1 真目录两圈（目录＝$R 本身 ‖ 清单临时只含 zz_probe.txt ‖ STAMP 同枚）★＝第一圈 rc=$S1A（命中行=$S1A_HITS・★期望 0＝真让路★） ‖ ★第二圈 rc=$S1B（命中行=$S1B_HITS ‖ 命中次=$S1B_HITSO・期望 6／1＝真停★）"',
    'say "  ★S1 判读尺★＝第二圈 rc=6 ⇒ ★撞名让路在真 $R 目录上第一次有载体★；rc=0 ⇒ 本窗仍未走到那条分支＝★延二那一格继续半填、枚名不摘★"',
    'say "  ★S1 stderr 原文（亲手搬进窗志）＝$S1B_ERR ‖ 盘上留证＝$S1PREV 枚 .prev_*_zz_probe.txt（★留在本目录作载体、不删★） ‖ 清单已还原＝$YIELD_LIST"',
    'say "  ★S1 结构上限现测★＝★命中枚数恒 ≤1★（件内首枚撞名即 return 6 ⇒ 循环不继续）＝这枚是布尔、不是计数；★行数口径与次数口径随句并报、不得互顶★"',
    'say "  ★S1 两道守卫★＝①让路清单临时只含★一枚★探针件（不止一枚⇒exit 8）②★跑完后窗志与 run/ 仍在位★（被动走⇒exit 8）＝走到下面这一行＝★两枚守卫都过★"',
], "S1 真目录撞名载体", whole=True)

# ---------- 4. 三段自证（bash -n ／ ast.parse ／ 件内 R= 路径＝盘上路径） ----------
sh = subprocess.run(["bash", "-n", W], capture_output=True, text=True)
assert sh.returncode == 0, "bash -n 未过：\n" + sh.stderr
for py in sorted(os.listdir(DST)):
    if py.endswith(".py"):
        ast.parse(rd(os.path.join(DST, py)), filename=py)
print("bash -n 通过 ‖ ast.parse 通过（%d 枚 py）" % len([f for f in os.listdir(DST) if f.endswith(".py")]))

sw = rd(W)
m = re.search(r"^R=(/opt/ros1-slam/\S+)$", sw, re.M)
assert m, "窗脚本内未找到 R= 路径行"
inner = m.group(1)
host_p = inner.replace("/opt/ros1-slam", "/home/jack/ysc-code/ros1-slam", 1)
assert os.path.isdir(host_p), "件内 R= 路径在宿主面不存在：" + host_p
assert host_p == DST, "件内 R=%s 与盘上目录 %s 不一致" % (host_p, DST)
print("件内 R= 路径＝盘上路径 ✔ ＝%s" % DST_NAME)

# ★本窗新落六枚改形的★可执行层★断言（辖域只管可执行行、注释里的历史描述不算＝追加一百零三⑤ 的断言亚种）
exe = "\n".join(l for l in sw.split("\n") if not l.strip().startswith("#"))
assert "RC_NAME" in rd(P1), "S5 未落"
assert "不按 T 段过滤" in rd(PB), "S6 未落"
assert "规范架 1000×1000" in rd(CM), "S2 未落"
assert "DIFFX" in exe and "TSROW" in exe, "S3 未落（可执行层）"
assert "步10c" in exe, "S10 未落（可执行层＝打印行，注释不算）"
assert "S1_B" in exe or "S1B" in exe, "S1 未落（可执行层）"
assert "CLRC=" not in exe, "★上一窗删掉的镜像重演 CLRC 硬赋值不得回潮★"
print("六枚改形逐枚命中 ✔")
