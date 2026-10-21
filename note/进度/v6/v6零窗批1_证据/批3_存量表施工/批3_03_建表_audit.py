#!/usr/bin/env python3
# 批3 建表生成器之二：规矩⑨ 结论行报告器的存量表（#114 甲，明令第 10 件「同批配存量表」）
# 数据从文件读入（_audit_full.txt），源码内不放正文（动作㉕）。
# 键形＝报告行所在「那一行的内容」的 sha256 前 12 ⇒ 志文变长、行号漂了也不产生假新增（R320 废表的另一病根＝键里塞理由句）
import os, re, subprocess, hashlib

ROOT = "/home/ros1-slam"
D = os.path.join(ROOT, "note/进度/v6/v6零窗批1_证据/批3_存量表施工")
raw = open(os.path.join(D, "_audit_full.txt"), encoding="utf-8").read().splitlines()

rep = []
for l in raw:
    if not l.strip().startswith("[报]"):
        continue
    body = l.strip()[len("[报]"):].strip()
    loc, rest = body.split("｜", 1)
    doc, ln = loc.rsplit(":", 1)
    word = re.search(r"词=([^｜\s]+)", rest)
    rep.append((doc, int(ln), word.group(1) if word else "未给", rest))

face_line = [l for l in raw if l.startswith("面＝")]
cnt_line = [l for l in raw if l.startswith("[计数]")]

def blame(doc, ln):
    rel = os.path.relpath(doc, ROOT)
    out = subprocess.run(["git", "-C", ROOT, "blame", "-L", "%d,%d" % (ln, ln), "--porcelain", rel],
                         capture_output=True, text=True, errors="ignore").stdout.splitlines()
    if not out:
        return "blame空", ""
    h = out[0].split()[0]
    if h.count("0") == len(h):
        return "未入库行", ""
    s = subprocess.run(["git", "-C", ROOT, "log", "-1", "--format=%s", h],
                       capture_output=True, text=True).stdout.strip()
    m = re.match(r"(R\d+)", s)
    return (m.group(1) if m else s[:14]), h[:7]

rows = []
for doc, ln, word, rest in rep:
    ls = open(doc, encoding="utf-8", errors="ignore").read().splitlines()
    content = ls[ln - 1] if 0 < ln <= len(ls) else ""
    fp = hashlib.sha256(content.strip().encode("utf-8")).hexdigest()[:12]
    b, h7 = blame(doc, ln)
    snip = content.replace("\t", " ").strip()[:64]
    rows.append("\t".join([fp, os.path.relpath(doc, ROOT), str(ln), word, b, h7, snip]))

CHECK = ("T=scripts/verify/claim_audit_known_lines.tsv; [ -f \"$T\" ] || { echo '缺表⇒不判 rc=9'; exit 9; }; "
         "cd /home/ros1-slam; K() { python3 scripts/verify/v_claim_field_audit.py 2>/dev/null "
         "| sed -n '/^ *\\[报\\]/{s/^ *\\[报\\] //;p}' | awk -F'｜' '{print $1}' "
         "| while IFS= read -r loc; do f=${loc%:*}; n=${loc##*:}; sed -n \"${n}p\" \"$f\" "
         "| python3 -c 'import sys,hashlib;print(hashlib.sha256(sys.stdin.read().strip().encode(\"utf-8\")).hexdigest()[:12])'; done | sort -u; }; "
         "N=$(comm -13 <(grep -v '^#' \"$T\" | cut -f1 | sort -u) <(K) | grep -c .); "
         "S=$(comm -23 <(grep -v '^#' \"$T\" | cut -f1 | sort -u) <(K) | grep -c .); "
         "echo \"规矩⑨存量复验 新增=$N 陈旧=$S\"; [ \"$N\" = 0 ] && exit 0 || exit 5")

hdr = [
    "# claim_audit_known_lines.tsv —— 规矩⑨（结论行报告器 v_claim_field_audit.py）存量表",
    "# 档位＝#114 甲＝报告器入门＋同批配存量表；明令出处＝主理人 2026-10-15 16:05 勾复 A-8「第 10 件＝规矩⑨结论行报告器（#114 甲，同批配存量表）」（原文见 note/计划书v6/23_v6任务队列.md A-8 行）",
    "# 门用面写死＝scripts/verify/claim_face.txt（现跑 %s）" % (face_line[0].split("／")[0].replace("面＝", "") if face_line else "未知"),
    "# 件版本＝v_claim_field_audit.py sha 前 12=%s｜件读数出处＝本批现跑：%s ／ %s" % (
        subprocess.run(["sha256sum", os.path.join(ROOT, "scripts/verify/v_claim_field_audit.py")],
                       capture_output=True, text=True).stdout.split()[0][:12],
        cnt_line[0] if cnt_line else "件未自报计数行", "跑法：python3 scripts/verify/v_claim_field_audit.py（缺省面）"),
    "# 键形＝「被报那一行的整行内容」sha256 前 12 ⇒ 行号漂不产生假新增；★键内不含引号／制表符／件给的理由句",
    "# 列＝键（行内容指纹）｜文档相对路径｜实测行号（会漂，定位以指纹＋锚片段为准）｜强度词｜git blame 批次｜commit 前 7｜该行锚片段（前 64 字）",
    "# ★本表消费方＝批内一条命令复验（现跑 [报] 行逐行取指纹 → 与本表键集 diff）；报告器件本身无存量表输入口（件只报不拦、rc 恒 0／缺面 9），改件成消费方＝超出本次明令 ⇒ 已按停手条件呈报",
    "# 一条命令复验（新增才点名、陈旧只报不销；缺表⇒下表末命令自行打印「缺表⇒不判 rc=9」并以 9 退出）：",
    "#   " + CHECK,
    "# 缺表行为＝复验式先判文件在位，缺⇒打印「不判 rc=9」并以 9 退出（照 Z5 第四件套语义；★不是报告器自身的 rc）",
]
dst = os.path.join(ROOT, "scripts/verify/claim_audit_known_lines.tsv")
with open(dst, "w", encoding="utf-8") as fh:
    fh.write("\n".join(hdr) + "\n")
    for r in rows:
        fh.write(r + "\n")
print("报行=%d 表行=%d（含头 %d 行）" % (len(rep), len(hdr) + len(rows), len(hdr)))
for r in rows:
    c = r.split("\t")
    print("  %s ｜ %s:%s ｜ 词=%s ｜ 出处=%s" % (c[0], c[1], c[2], c[3], c[4]))
