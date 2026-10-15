#!/usr/bin/env python3
# R331 批2 第四档生成器：余 15 条逐条确证（判据式逐字取自 ap 稿；数据源＝证据目录原件；先核清单 sha 再算）
# 三态口径＝作业单 22 号批2 ＋ 主理人 2026-10-16 令⑤：原件所在目录无 _sha256_manifest.txt ⇒ 只能记 N3（不降级、不豁免）
import os, re, subprocess

ROOT = "/home/ros1-slam"
RAW = os.path.join(ROOT, "note/进度/SCAN集成/证据/log/raw")
AP = os.path.join(ROOT, "note/进度/SCAN集成/设计/ap_在册判据清单_A54扫完定义输入.md")
SELF = re.compile(r"^(r31[4-9]|r32\d|r33\d|batch|_tmp)")  # 施工侧自产枚举/留底目录不算历史原件

ap = {}
for line in open(AP, encoding="utf-8", errors="ignore").read().splitlines():
    if re.match(r"^\| *[0-9]+ *\|", line):
        p = [x.strip() for x in line.split("|")]
        ap[int(p[1])] = (p[2], p[3])

# 每行：判据式里的关键字段（★不是判据本文，是"去证据树里找原件"的检索式）
PAT = {
    8:  [r"13\.2\s*Hz", r"hz=13"],
    9:  [r"264\.4", r"hz=264"],
    10: [r"飞点\s*0\.0000", r"硬门[^，。\n]{0,12}0\.5"],
    11: [r"A④"],
    12: [r"A⑤"],
    13: [r"state=3", r"stageB[^\n]{0,40}teb"],
    14: [r"A0 RTF"],
    15: [r"13 CHECK", r"CHECK 全"],
    17: [r"话题频率", r"/livox/lidar", r"rslidar"],
    19: [r"mbf"],
    20: [r"SUP 心跳", r"心跳[^\n]{0,14}1 Hz"],
    21: [r"maze20", r"post2d"],
    22: [r"X2"],
    23: [r"M5 判据一", r"判据一[^\n]{0,20}PASS"],
    24: [r"dt[=≤ ][0-9.]+s?[^\n]{0,10}60", r"60s"],
}
PROBE_ONLY = {17, 22}          # ★本体依赖活体／硬件 ⇒ 直接 N3（在册读法）
EXT = (".log", ".txt", ".json", ".out")
_dirs = sorted(d for d in os.listdir(RAW) if os.path.isdir(os.path.join(RAW, d)))
mcache = {}


def manifest_state(d):
    if d in mcache:
        return mcache[d]
    dd = os.path.join(RAW, d)
    mf = os.path.join(dd, "_sha256_manifest.txt")
    if not os.path.isfile(mf):
        mcache[d] = ("无清单", 0)
        return mcache[d]
    r = subprocess.run("sha256sum -c _sha256_manifest.txt 2>/dev/null | grep -c ': OK$'",
                       shell=True, cwd=dd, capture_output=True, text=True)
    n = int((r.stdout or "0").strip() or 0)
    bad = subprocess.run("sha256sum -c _sha256_manifest.txt 2>/dev/null | grep -vc ': OK$'",
                         shell=True, cwd=dd, capture_output=True, text=True)
    nb = int((bad.stdout or "0").strip() or 0)
    mcache[d] = (("有清单·OK=%d/非OK=%d" % (n, nb)) if nb == 0 else "有清单·校验有非OK", n)
    return mcache[d]


def find(n):
    hits = []
    for d in _dirs:
        if SELF.match(d):
            continue
        dd = os.path.join(RAW, d)
        for f in sorted(os.listdir(dd)):
            fp = os.path.join(dd, f)
            if not os.path.isfile(fp) or not f.endswith(EXT):
                continue
            try:
                txt = open(fp, encoding="utf-8", errors="ignore").read()
            except Exception:
                continue
            for pat in PAT[n]:
                m = re.search(pat, txt)
                if m:
                    i = m.start()
                    snip = txt[max(0, i - 30):i + 46].replace("\n", "⏎").replace("\t", " ")
                    hits.append((d, f, pat, snip))
                    break
            if len(hits) >= 6:
                return hits
    return hits


print("| 判据号 | 判据式（逐字，取自 ap 稿） | 原件检索结果（目录＋文件＋清单校验态） | 命中字段 | 本档结论 |")
print("|---|---|---|---|---|")
stat = {}
for n in sorted(PAT):
    nm, expr = ap.get(n, ("?", "?"))
    hs = find(n)
    ok = [h for h in hs if manifest_state(h[0])[0].startswith("有清单·OK")]
    if n in PROBE_ONLY:
        concl = "N3＝不可离线确证（本体依赖活体窗／硬件现役）"
    elif ok:
        concl = "★命中带清单且校验 OK 的原件 ⇒ 待人工逐值判定（本档只列位置，确证句在登记件内）"
    elif hs:
        concl = "N3＝不可离线确证（原件所在目录无 _sha256_manifest.txt＝sha 前置缺失；★按令⑤不补、不降级）"
    else:
        concl = "N3＝未定位（证据树内无该字段的历史原件；在册出处若是志文则属转抄值、不作确证）"
    stat[n] = concl
    loc = "；".join("%s/%s［%s］" % (h[0], h[1], manifest_state(h[0])[0]) for h in hs[:2]) or "（无）"
    fld = hs[0][3] if hs else "—"
    print("| #%s %s | %s | %s | %s | %s |" % (n, nm, expr[:70], loc[:150], fld[:60], concl))
open(os.path.join(ROOT, "note/进度/v6零窗批1_证据/批2_A5确证/_第四档_行表.md"), "w",
     encoding="utf-8").write("\n".join(
    ["%s\t%s" % (k, v) for k, v in sorted(stat.items())]) + "\n")
