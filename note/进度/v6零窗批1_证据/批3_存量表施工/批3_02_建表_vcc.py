#!/usr/bin/env python3
# 批3 建表生成器之一：VCC 存量表（#112 乙，主理人 2026-10-15 16:05 明令第 9 件）
# 数据全部从文件读入（_vcc_full.txt／claim_face.txt），源码内不放正文（动作㉕）。
# ★键形与「一条命令复验」用的是同一条推导（同一个 sed/awk 切法）——否则表与复验各说各话＝R320 键污染那一型。
#   键＝「文档 basename|目标串:行」；全路径放出处列（面内 进度文档.md 有两件，basename 不唯一 ⇒ 不放进键）。
import os, re, subprocess, collections

ROOT = "/home/ros1-slam"
D = os.path.join(ROOT, "note/进度/v6零窗批1_证据/批3_存量表施工")
FACE_FILE = "scripts/verify/claim_face.txt"
raw = open(os.path.join(D, "_vcc_full.txt"), encoding="utf-8").read().splitlines()
face = [l.strip() for l in open(os.path.join(ROOT, FACE_FILE), encoding="utf-8")
        if l.strip() and not l.strip().startswith("#")]
amber = [l for l in raw if l.startswith("[AMBER] ")]
_q = [l for l in raw if l.startswith("[四元组]")]
VCC_SHA = (re.search(r"sha=([0-9a-f]{12})", _q[0]).group(1)
           if _q and re.search(r"sha=([0-9a-f]{12})", _q[0]) else "件未自报")
_cnt = [l for l in raw if l.startswith("[计数] ")]


def blame(doc, ln):
    out = subprocess.run(["git", "-C", ROOT, "blame", "-L", "%d,%d" % (ln, ln), "--porcelain", doc],
                         capture_output=True, text=True, errors="ignore").stdout.splitlines()
    if not out:
        return "blame空", ""
    h = out[0].split()[0]
    if h.count("0") == len(h):
        return "未入库行", ""
    s = subprocess.run(["git", "-C", ROOT, "log", "-1", "--format=%s", h],
                       capture_output=True, text=True).stdout.strip()
    m = re.match(r"(R\d+)", s)
    return (m.group(1) if m else s[:16]), h[:7]


def anchor(doc, ln):
    ls = open(os.path.join(ROOT, doc), encoding="utf-8", errors="ignore").read().splitlines()
    s = ls[ln - 1] if 0 < ln <= len(ls) else ""
    return s.replace("\t", " ").strip()[:60]


# 与复验式同一条键推导：'[AMBER] 文档 ⇒ 目标' → 去理由句 → '文档|目标:行'
def make_key(a):
    body = a[len("[AMBER] "):].split(" 键名「")[0]
    body = re.split(r"（", body, 1)[0].rstrip()
    doc, tgt = body.split(" ⇒ ", 1)
    return "%s|%s" % (doc.strip(), tgt.strip())


def classify(a):
    if " 键名「" in a:
        return "键名不在该行"
    if "（**仓内根本没有这个文件名**" in a:
        return "目标文件不在仓内"
    if "同名多份" in a:
        return "同名多份未写全路径"
    return "其他（未列 reason，按行号档）"


groups = collections.OrderedDict()
for a in amber:
    k = make_key(a)
    groups.setdefault(k, {"n": 0, "cls": classify(a), "raw": a})
    groups[k]["n"] += 1

rows = []
for k, v in groups.items():
    doc_base, tgt = k.split("|", 1)
    m = re.match(r"(.*):(\d+)$", tgt)
    tb, tl = (m.group(1), m.group(2)) if m else (tgt, "0")
    hits = []
    for f in face:
        if os.path.basename(f) != doc_base:
            continue
        if not os.path.isfile(os.path.join(ROOT, f)):
            continue
        for i, line in enumerate(open(os.path.join(ROOT, f), encoding="utf-8", errors="ignore").read().splitlines(), 1):
            if "%s:%s" % (os.path.basename(tb), tl) in line:
                b, h7 = blame(f, i)
                hits.append("%s:%d〔blame=%s｜commit=%s｜锚=%s〕" % (f, i, b, h7, anchor(f, i)))
    loc = "　".join(hits) if hits else "★定位失败：该 basename 的候选文档内未搜到「%s:%s」字样（不删行，照实登记）" % (os.path.basename(tb), tl)
    rows.append("\t".join([k, str(v["n"]), v["cls"], loc]))

CHECK = ("T=scripts/verify/vcc_known_findings.tsv; [ -f \"$T\" ] || { echo '缺表⇒不判 rc=9'; exit 9; }; "
         "K() { cd /home/ros1-slam; VCC_MAX=200 bash scripts/verify/v_citation_check.sh $(grep -vE '^ *(#|$)' scripts/verify/claim_face.txt) 2>/dev/null "
         "| sed -n '/^\\[AMBER\\]/{s/^\\[AMBER\\] //;s/ 键名「.*//;s/（.*//;s/[[:space:]]*$//;p}' "
         "| awk '{split($0,a,\" ⇒ \"); print a[1]\"|\"a[2]}' | sort -u; }; "
         "N=$(comm -13 <(grep -v '^#' \"$T\" | cut -f1 | sort -u) <(K) | grep -c .); "
         "S=$(comm -23 <(grep -v '^#' \"$T\" | cut -f1 | sort -u) <(K) | grep -c .); "
         "echo \"VCC存量复验 新增=$N 陈旧=$S\"; [ \"$N\" = 0 ] && exit 0 || exit 5")

hdr = [
    "# vcc_known_findings.tsv —— VCC 存量表（第 9 件门禁成员 v_citation_check.sh 的存量档）",
    "# 档位＝Z5 四件套之一＝#112 乙（存量表随件入门）；明令出处＝主理人 2026-10-15 16:05 勾复 A-8：「第 9 件＝VCC（#112 乙，rc 语义一字不改、只报不判）」（盘上原文＝note/计划书v6/23_v6任务队列.md 的 A-8 行）",
    "# 门用面写死（Z5 之三）＝%s 现跑 %d 件｜窗宽＝±40（件内缺省，可被 VCC_FIXUP_CHARS 覆盖）｜明细上限 VCC_MAX=200（★只放宽打印条数，RED/AMBER/OK 判据本体一字未动）" % (FACE_FILE, len(face)),
    "# 件版本＝v_citation_check.sh 本件现身 sha 前 12=%s｜建表时点件现跑读数＝%s" % (VCC_SHA, (_cnt[0] if _cnt else "件未自报计数行")),
    "# 键形＝「文档 basename|目标串:行」＝与下面复验式同一条 sed/awk 推导；★键内不含件给的理由句／引号／制表符（R320 那两张废表的病根＝把「键名「…」不在该行」整段并进键、且半角引号写坏）",
    "# ★basename 不唯一声明＝面内 note/进度/…/进度文档.md 有两件同名（本批实测）⇒ 键按 basename 归并、两处若引同一目标则合为一行且处数照加；全路径逐条在本表第 4 列",
    "# 列＝键｜处数｜严档类｜逐条带出处（引用在志文中的实测行号＋git blame 批次＋commit 前 7＋该行锚片段；★行号随志文增长而漂，定位以锚片段为准）",
    "# 一条命令复验（新增才点名、陈旧只报不销；★缺表⇒打印「缺表⇒不判 rc=9」并以 9 退出）：",
    "#   " + CHECK,
    "# 复验成本＝现跑实测（见批留底 批3_04）：本复验式跑一次 VCC 全 70 面约 1.8 秒；★若改成逐件调用则约 43 秒（本批实测两种跑法都跑过，取前者）",
    "# ★消费方＝批内动作与串读数的对照档，不在件内：VCC 件只有豁免表输入口（VCC_EXEMPT→citation_known_pseudo_reds.tsv），无存量表输入口；把件改成消费方＝改件＝超出本次明令 ⇒ 已按停手条件呈报",
]
dst = os.path.join(ROOT, "scripts/verify/vcc_known_findings.tsv")
with open(dst, "w", encoding="utf-8") as fh:
    fh.write("\n".join(hdr) + "\n")
    for r in rows:
        fh.write(r + "\n")
fails = sum(1 for r in rows if "定位失败" in r)
print("AMBER明细=%d 表行=%d 总处数=%d 定位失败=%d 表=%s 行数=%d 字节=%d" % (
    len(amber), len(rows), sum(v["n"] for v in groups.values()), fails,
    os.path.relpath(dst, ROOT), len(hdr) + len(rows), os.path.getsize(dst)))
