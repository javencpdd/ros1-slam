#!/usr/bin/env python3
# _kit_r525.py —— 批181 窗套件生成器＝从 r521 复制 12 件＋改名（★r521/R525 两大小写都替、r521q 先保护★）
#   再落★六枚件内修★（r521q 新抓 S13 ‖ S14 ‖ S15 ‖ S18 ‖ S20 ‖ S22）＋★S1 许可形★（STAMP env 口＋步98 单枚承重名预置撞名）
# 姿势＝逐枚按锚点断言恰一枚命中再落盘；改完必过 bash -n ／ ast.parse ／ 件内 R= 路径＝盘上路径；★本批新增＝建单闸由 _manifest_build.sh 建★
import ast
import io
import os
import re
import shutil
import subprocess

RAW = "/home/jack/ysc-code/ros1-slam/note/进度/SCAN集成/证据/log/raw"
SRC_NAME = ("r521_活体窗按r517q缺席清单落六枚件内修_两支红档名由一枚码查表导出与独立来源从句改整枚data与盲区枚数带遍历域与"
            "复跑判据改除时刻行外逐行同与判读锚由件内现取与真目录撞名让路载体")
DST_NAME = ("r525_活体窗按r521q缺席清单落六枚件内修_两支红档同名册自证与志面dollar问机检与时刻token行内排除与锚键入表与"
            "不变量自带消费者与真目录单枚承重名撞名载体")
SRC = os.path.join(RAW, SRC_NAME)
DST = os.path.join(RAW, DST_NAME)
assert os.path.isdir(SRC), "源目录不在位"
assert DST_NAME.count("r521q") == 1, "新目录名应含 r521q 恰一枚"
if os.path.exists(DST):
    junk = os.listdir(DST)
    assert "run" not in junk and "00_window_log.txt" not in junk and "_sha256_manifest.txt" not in junk, \
        "目标目录已有产物＝不覆盖：" + str(junk)
    shutil.rmtree(DST)

KEEP = ["00_r521_window.sh", "01_复读件_r521.py", "02_make_copies_r521.py", "03_sub_elev_count.py",
        "_cmp_r521.py", "pub_rect_probe_r521.py", "sub_costmap_dump_r521.py", "sim_nav_smoke_r521.sh",
        "global_costmap_params_A_r521.yaml", "global_costmap_params_C_r521.yaml",
        "mbf_navigation_r521.launch", "mode_nav_sim_r521.launch"]
for f in KEEP:
    assert os.path.isfile(os.path.join(SRC, f)), "源件缺失：" + f
os.makedirs(DST)
for f in KEEP:
    shutil.copyfile(os.path.join(SRC, f), os.path.join(DST, f.replace("r521", "r525")))
print("复制 %d 件 -> %s" % (len(KEEP), DST_NAME))

rd = lambda p: io.open(p, encoding="utf-8").read()
wr = lambda p, s: io.open(p, "w", encoding="utf-8").write(s)

# ---------- 2. 改名（大小写两端都替、r521q 先保护） ----------
for fn in sorted(os.listdir(DST)):
    p = os.path.join(DST, fn)
    if not os.path.isfile(p):
        continue
    t = rd(p)
    assert "r521q" not in t and "R521q" not in t, "复制件 %s 内含 r521q/R521q" % fn
    t = t.replace("r521q", "@@KQ@@").replace("R521q", "@@KQ@@")
    t = t.replace("r521", "r525").replace("R521", "R525")
    t = t.replace("@@KQ@@", "r521q")
    wr(p, t)
OLD_DIRNAME = SRC_NAME.replace("r521", "r525")
hit = 0
for fn in sorted(os.listdir(DST)):
    p = os.path.join(DST, fn)
    if os.path.isfile(p):
        t = rd(p)
        if OLD_DIRNAME in t:
            hit += t.count(OLD_DIRNAME)
            wr(p, t.replace(OLD_DIRNAME, DST_NAME))
assert hit >= 1, "★没有任何一处旧目录名被改写＝件内路径与盘上目录名会不一致★"
print("改名与整串路径改写完成（命中 %d 处）" % hit)
for fn in sorted(os.listdir(DST)):
    p = os.path.join(DST, fn)
    if os.path.isfile(p):
        t = rd(p)
        assert "r525q" not in t.replace("r521q", ""), "★r521q 保护链被改名打断★"
        assert "R521" not in t, "★大写旧窗号残留＝改名规则被打掉（批170 那枚对偶、这次栽在生成器代码行）★：%s" % fn
print("改名后置检通过＝★大写 R521 残留 0 枚★")


def line_edit(path, needle_start, new_lines, tag, needle_end=None):
    lines = rd(path).split("\n")
    si = [i for i, l in enumerate(lines) if needle_start in l]
    assert len(si) == 1, "改形[%s] 起始锚命中 %d 次｜%s" % (tag, len(si), needle_start[:60])
    a = si[0]
    b = a if needle_end is None else [i for i, l in enumerate(lines) if needle_end in l][0]
    if needle_end is not None:
        assert len([i for i, l in enumerate(lines) if needle_end in l]) == 1, "改形[%s] 结束锚不唯一" % tag
        assert b >= a, "改形[%s] 锚序倒了" % tag
    out = lines[:a] + new_lines + lines[b + 1:]
    wr(path, "\n".join(out))
    print("  改形[%s] 落盘 ✔" % tag)


W = os.path.join(DST, "00_r525_window.sh")
P1 = os.path.join(DST, "01_复读件_r525.py")
CM = os.path.join(DST, "_cmp_r525.py")

# ================= S13 ‖ S14＝两张表把「分支↔码↔名」绑死＋名册自证 =================
line_edit(P1, 'RC_NAME = {5: "负例分支＝替换自测（设计来翻红的那支）", 9: "前置缺失档＝被引件或清单出处件不在位"}', [
    'BR_BY_SWAP = "swap" if SWAP else "premiss"   # ★S14/R525＝先定★分支★，码与名都从分支查出来★',
    'RC_BY_BR = {"swap": 5, "premiss": 9}',
    'NAME_BY_BR = {"swap": "负例分支＝替换自测（设计来翻红的那支）", "premiss": "前置缺失档＝被引件或清单出处件不在位"}',
    'RC_DOC = " ‖ ".join("%s＝%d＝%s" % (k, RC_BY_BR[k], NAME_BY_BR[k]) for k in ("swap", "premiss"))',
    'assert RC_BY_BR["swap"] == 5 and RC_BY_BR["premiss"] == 9, "★名册被对调＝分支↔码不再同源（r521q S14 的那枚反例）★"',
    'assert RC_BY_BR.keys() == NAME_BY_BR.keys() and sorted(RC_BY_BR.values()) == [5, 9], "★两张表不同构＝一名一码不许复用★"',
    'RC_RED = RC_BY_BR[BR_BY_SWAP]',
], "S14 分支↔码↔名同名册")

line_edit(P1, '    assert RC_RED in RC_NAME, "★退出码 %s 不在名册里＝打印名无从导出★" % RC_RED', [
    '    assert BR_BY_SWAP in RC_BY_BR and BR_BY_SWAP in NAME_BY_BR and RC_RED == RC_BY_BR[BR_BY_SWAP], "★分支↔码↔名三者不同源＝r521q S14 那枚对调★"',
], "S14 assert 换新名册")
line_edit(P1, '    print("   被引件不在位＝%s ⇒ rc_复读=%s（★%s＝%s★；★S5/R525＝这一枚名由码查表导出、不靠人工同步★）" % (cited, RC_RED, RC_RED, RC_NAME[RC_RED]))',
          ['    print("   被引件不在位＝%s ⇒ rc_复读=%s（★分支＝%s ‖ 名＝%s★；★S13/S14/R525＝码与名都由★同一枚分支变量★查出来★）" % (cited, RC_RED, BR_BY_SWAP, NAME_BY_BR[BR_BY_SWAP]))'],
          "S13 第一支改由分支导出")
line_edit(P1, '    print("   ★本件即将以 rc=%s 退出（R517 立的两枚：负例红进码＝5 ‖ 前置缺失＝9；★R525 改形①＝名与码同源★）★" % RC_RED)',
          ['    print("   ★本件即将以 rc=%s 退出（★R525 自证＝这句里的 5／9 与名册★同源★；名册＝%s★）★" % (RC_RED, RC_DOC))'],
          "S13 自陈行同源")
line_edit(P1, '    print("   ★清单出处件不在位 ⇒ rc_勾对_复读件=5（不猜清单）‖ 并记「前置缺失」档＝9★（★r505q 点过的那支没进码、本批补上★）")',
          ['    print("   ★清单出处件不在位 ⇒ rc_勾对_复读件=%s（不猜清单）‖ 并记「%s」★（★S13/R525＝★第二支★的名与码也从同一张表导出★）★" % (RC_BY_BR["swap"], NAME_BY_BR["premiss"]))'],
          "S13 第二支入册")
line_edit(P1, 'print("   ★本件退出码档＝%s（0＝正测取到全读数 ‖ 5＝负例分支红（仅 SWAP）‖ 9＝前置缺失＝被引件或清单出处件不在位)★" % (0 if PREMISS == 0 else PREMISS))',
          ['print("   ★本件退出码档＝%s（0＝正测取到全读数 ‖ %s★）★" % (0 if PREMISS == 0 else PREMISS, RC_DOC))'],
          "S13 尾部档说明同源")

# ================= S15＝say 串里的 $? 改预取变量＋志面回潮机检（ASCII 记号定位、防自打自证） =================
line_edit(W, 'SRC=$?', ['SRC=$?', 'SRC_FROM=$?   # ★S15/R525＝★紧跟调用现取★，下面那句 say 里★不再出现裸 $?★（r521 那版被当场展开成 0）'], "S15 预取变量")
line_edit(W, 'say "  ★SWAP 遍 进程 rc 回读＝$SRC（★这枚数是从 $? 读回来的、不是打印串里的自陈★）',
          ['say "  ★SWAP 遍 进程 rc 回读＝$SRC（★这枚数是从★同一枚★变量 SRC_FROM＝$SRC_FROM 读回来的、不是打印串里的自陈★） ‖ 证件内自陈行＝$(grep -c \'即将以 rc=5 退出\' "$R/08_复读_swap负例.txt") 枚 ‖ 该件不进任何在册件的 rc★"'],
          "S15 打印句去裸 $?" , needle_end='该件不进任何在册件的 rc★"')

# ================= S18＝时刻串改成★行内 token★排除＋断言被排除行的非时刻部分两遍逐字同 =================
line_edit(W, 'DIFFX=$(diff <(grep -v ',
          ['TOK_TIME="本件时刻＝[0-9-]{10} [0-9:]{8}（★容器面 UTC；宿主 CST＝+8★）"',
           'L1R=$(sed -E "s/$TOK_TIME/★时刻已排除★/" "$R/05_cmp.txt" | grep -F "★遍次（件自身带）" | LC_ALL=C sort)',
           'L2R=$(sed -E "s/$TOK_TIME/★时刻已排除★/" "$R/05_cmp_复跑.txt" | grep -F "★遍次（件自身带）" | LC_ALL=C sort)',
           'RESTSAME=$([ "$L1R" = "$L2R" ] && echo True || echo False)',
           'DIFFX=$(diff <(sed -E "s/$TOK_TIME/★时刻已排除★/" "$R/05_cmp.txt") <(sed -E "s/$TOK_TIME/★时刻已排除★/" "$R/05_cmp_复跑.txt") 2>/dev/null | grep -c "^[<>]")'],
          "S18 行内 token 排除")
line_edit(W, 'say "  对照件第一遍 rc=$CRC ‖ ★离窗复跑第二遍 rc=$CRC2★',
          ['say "  对照件第一遍 rc=$CRC ‖ ★离窗复跑第二遍 rc=$CRC2★ ‖ 两遍逐行差＝$DIFFN 行（行位口径＝diff 两侧各计一行） ‖ ★S18/R525＝除★时刻 token★ 外逐行差＝$DIFFX 行★（★排除从「整行」改成「行内 token」＝r521q S18 那两枚反例的堵法★） ‖ ★被排除行的非时刻部分两遍逐字同＝$RESTSAME★（★假绿面：遍次／共几遍／出处都变了这枚必 False★） ‖ 时刻 token 行位＝第一遍[$TSROW] ‖ 第二遍[$TSROW2]"'],
          "S18 判据句子")

# ================= S20＝锚打成★键值表★（机读）＋人读句，判读件只准从表里生成 =================
line_edit(W, 'say "== 步10c ★S10/R525＝判读锚由件内现取★',
          ['say "== 步10c ★S20/R525＝锚★入表★（机读键值）＋人读句★（r521q S20＝上一版锚只印在志里⇒抄错无人拦、且锚串本身进志使同一 grep 多命中）=="',
           ': > "$R/09_锚键.tsv"',
           'for kv in "P100_BAR:──【P100 行】" "P90_BAR:──【P90 行】" "G1:G1 闸" "PANC:判词" "ZHI:注：皆致命" "BIANCI:★遍次（件自身带）"; do',
           '  K="${kv%%:*}"; PAT="${kv#*:}"',
           '  CROW=$(grep -n -F "$PAT" "$R/05_cmp.txt" 2>/dev/null | cut -d: -f1 | paste -sd, -)',
           '  LROW=$(grep -n -F "$PAT" "$R/00_window_log.txt" 2>/dev/null | awk -v k="$PAT" \'index($0,k)>0 && $0 !~ /^.*[0-9]\\]\\s+锚＝/{print $1}\' | cut -d: -f1 | paste -sd, -)',
           '  printf \'%s\\t%s\\t%s\\t%s\\n\' "ANCHOR_${K}_cmp" "${CROW:-无}" "${LROW:-无}" "$PAT" >> "$R/09_锚键.tsv"',
           '  say "  锚＝$PAT ‖ 05_cmp.txt 行号＝${CROW:-（无）} ‖ 窗志行号＝${LROW:-（无）}（★两枚载体不互顶★）"',
           'done',
           'say "  ★09_锚键.tsv 枚数＝$(wc -l < "$R/09_锚键.tsv")（期望 6）★ ‖ ★判读件里的行号一律由本表生成、★不得手写★（追加一百一十一⑧）★"'],
          "S20 锚键入表", needle_end='（★两枚载体不互顶、各报各的★）"')

# ★S20 的收尾＝旧步10c 的 `done` 是循环的另一半、不在我的锚跨度里⇒当场摘掉孤立那一枚★
_lines = rd(W).split("\n")
_i = [k for k, l in enumerate(_lines) if '09_锚键.tsv 枚数' in l]
assert len(_i) == 1, ("锚键枚数行不唯一", len(_i))
_j = _i[0] + 1
assert _lines[_j].strip() == "done", ("孤立 done 预期落空", _lines[_j][:40])
del _lines[_j]
wr(W, "\n".join(_lines))
print("  摘除孤立 done ✔（★循环的两半必须一起换＝锚要覆盖到结构的末尾★）")

# ================= S22＝给 inv_ok 一个★消费者★：新枚 rc_不变量＋行使枚数随句并报 =================
line_edit(CM, 'print("\\n===〔二〕栏一＋栏二（判定面，★与谓词无关★）===")',
          ['print("\\n===〔一b〕★S22/R525＝不变量自带消费者★（r521q：`inv_ok` 算而无消费＝死变量；四对全异架⇒★本窗未行使★）===")',
           'INV_DONE = sum(1 for a, b in [("A", "Ap"), ("A", "C"), ("A", "REF"), ("REF", "A")] if (S[a]["ox"] == S[b]["ox"] and S[a]["oy"] == S[b]["oy"]))',
           'RC_INV = 0 if inv_ok else 5',
           'print("   行使枚数（同 origin 的对数）＝%d ‖ 不适用枚数（异 origin）＝%d ‖ ★rc_不变量＝%s★（★0＝所有★已行使★的对都 raw≡aligned；★行使 0 枚≠红★＝全异架是合法态，但★必须随句并报行使枚数★，否则读者会把「代码在场」读成「跑过并通过」）" % (INV_DONE, 4 - INV_DONE, RC_INV))',
           'print("   ★本窗不变量★未行使★（当 INV_DONE=0 时）＝发放形永久带这句★ ‖ 历窗行使面另行全扫、★不得跨窗引用★")',
           'print("\\n===〔二〕栏一＋栏二（判定面，★与谓词无关★）===")'],
          "S22 inv_ok 消费者")

# ================= S1 许可形＝STAMP env 口＋步98 单枚承重名预置撞名（前后目录指纹自证） =================
s = rd(W)
old_stamp = 'STAMP=$(date +%Y%m%d-%H%M%S.$(date +%N | cut -c1-6))'
assert s.count(old_stamp) == 1, ("STAMP 锚", s.count(old_stamp))
s = s.replace(old_stamp, 'STAMP="${WSTAMP:-$(date +%Y%m%d-%H%M%S.$(date +%N | cut -c1-6))}"   # ★S1/R525＝许可形要求的那枚★ env 口★（r521q：无口⇒同戳重跑不可外部指定）★')
wr(W, s)

step98 = [
    'say "== 步98 ★S1 许可形＝在★真目录★上对★一枚★承重名预置让路件再撞一次★（★清单只含那一枚＝不搬动其余五枚★）=="',
    'S98_SAVED="$YIELD_LIST"',
    'YIELD_LIST="05_cmp.txt"',
    'case " $YIELD_LIST " in *" run "*|*" 00_window_log.txt "*|*" 01_preflight.txt "*|*" 03_delta.txt "*|*" 07_复读.txt "*) echo "★步98 守卫不过＝让路清单里含其它承重件＝当场中止★" >&2; exit 8;; esac',
    ': > "$R/.prev_${STAMP}_05_cmp.txt"',
    'S98B=$(ls -a "$R" | LC_ALL=C sort | md5sum | cut -c1-16)',
    'S98ERR=$(yield_prev "$R" "$STAMP" 2>&1 >/dev/null); S98=$?',
    'S98HITS=$(printf "%s\\n" "$S98ERR" | grep -c "撞名即停")',
    'S98A=$(ls -a "$R" | LC_ALL=C sort | md5sum | cut -c1-16)',
    'YIELD_LIST="$S98_SAVED"',
    'say "  ★步98 读数★＝rc=$S98（★期望 6＝撞名即停★） ‖ 命中行=$S98HITS（期望 1） ‖ stderr 原文（★件内插桩、非人抄★）＝$S98ERR"',
    'say "  ★步98 自证＝其余承重件一根手指都没动★＝前后目录指纹（★预置件已建之后再取★）＝$S98B ‖ $S98A ‖ 相同＝$([ "$S98B" = "$S98A" ] && echo True || echo False)（★False＝真目录被动过＝本步的守卫目的就是把这枚变成可点数★）"',
    'say "  ★步98 判读尺★＝rc=6 且 前后指纹相同 ⇒ ★承重名上的撞名让路第一次有窗内载体、且未损坏留底★；rc=0 ⇒ ★预置没生效或判定式没走到★；指纹不同 ⇒ ★这一步本身就把承重件搬走了＝按★否决形★处理（不静默）★"',
    'say "  ★S15 形检★＝全志里『从 ★0★ 读回来』字样枚数＝$(grep -c "从 ★0★ 读回来" "$R/00_window_log.txt")（期望 ★0★；★记号含★星号★＝本行不含被检字面＝防自打自证＝追加一百零四① 同族★）"',
]
s = rd(W)
anchor = 'say "== 步99 清盘与计数同条命令 =='
assert s.count(anchor) == 1, ("步99 锚", s.count(anchor))
s = s.replace(anchor, "\n".join(step98) + "\n" + anchor)
wr(W, s)
print("  改形[S1 许可形＋STAMP env 口＋S15 形检] 落盘 ✔")

# ---------- 三段自证 ----------
sh = subprocess.run(["bash", "-n", W], capture_output=True, text=True)
assert sh.returncode == 0, "bash -n 未过：\n" + sh.stderr
for py in sorted(os.listdir(DST)):
    if py.endswith(".py"):
        ast.parse(rd(os.path.join(DST, py)), filename=py)
print("bash -n 通过 ‖ ast.parse 通过（%d 枚 py）" % len([f for f in os.listdir(DST) if f.endswith(".py")]))
sw = rd(W)
m = re.search(r"^R=(/opt/ros1-slam/\S+)$", sw, re.M)
assert m, "窗脚本内未找到 R= 路径行"
host_p = m.group(1).replace("/opt/ros1-slam", "/home/jack/ysc-code/ros1-slam", 1)
assert os.path.isdir(host_p), "件内 R= 路径在宿主面不存在：" + host_p
assert host_p == DST, "件内 R=%s 与盘上目录 %s 不一致" % (host_p, DST)
print("件内 R= 路径＝盘上路径 ✔")
exe = "\n".join(l for l in sw.split("\n") if not l.strip().startswith("#"))
assert 'STAMP="${WSTAMP:-' in exe, "STAMP env 口未落"
assert "步98" in exe and "S98B=" in exe, "S1 许可形未落（可执行层）"
assert "ANCHOR_" in exe and "09_锚键.tsv" in exe, "S20 未落（可执行层）"
p1 = rd(P1)
assert "RC_BY_BR" in p1 and "assert RC_BY_BR[\"swap\"] == 5" in p1, "S14 名册自证未落"
assert "RC_NAME" not in p1, "★旧名册 RC_NAME 残留＝两本名册并存★"
cm = rd(CM)
assert "rc_不变量" in cm and "INV_DONE" in cm, "S22 未落"
assert "RC_INV" in cm, "S22 消费者未落"
print("六枚改形逐枚命中 ✔")
