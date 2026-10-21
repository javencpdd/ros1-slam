#!/usr/bin/env python3
# _cmp_r456.py —— §8.3 段 B 对照件：★只比（CHECK 名，判定）二元组★（C6），数字一律不比
# 用法：_cmp_r456.py <在册形输出> <副本形输出> <在册形 launch 日志> <副本形 launch 日志>
# ★本件只报数★：同值／不同值由 06 判读按 C6＋C11 判；★不解释为什么★。
import sys


def parse(path):
    seq = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            if line.startswith("CHECK|"):
                p = line.rstrip("\n").split("|")
                if len(p) >= 3:
                    seq.append((p[1], p[2]))
    return seq


def plugins(path):
    out = []
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            for line in f:
                if "Using plugin" in line:
                    out.append(line.strip().split("Using plugin")[-1].strip())
    except Exception:
        return [], 0, 0
    rec = sum(1 for l in open(path, encoding="utf-8", errors="replace") if "Received a" in l) if 1 else 0
    return out, len(out), rec


a = parse(sys.argv[1])
b = parse(sys.argv[2])
print("== 段 B 对照（★在册形 REG vs 副本形 CP★）==")
print("REG＝%d 条｜PASS=%d FAIL=%d" % (len(a), sum(1 for _, v in a if v == "PASS"), sum(1 for _, v in a if v == "FAIL")))
print("CP ＝%d 条｜PASS=%d FAIL=%d" % (len(b), sum(1 for _, v in b if v == "PASS"), sum(1 for _, v in b if v == "FAIL")))
n = max(len(a), len(b))
diff = []
for i in range(n):
    x = a[i] if i < len(a) else ("<缺第%d条>" % (i + 1), "-")
    y = b[i] if i < len(b) else ("<缺第%d条>" % (i + 1), "-")
    same = (x == y)
    if not same:
        diff.append((i + 1, x, y))
    print("  %2d｜REG=%-22s %-5s ‖ CP=%-22s %-5s ⇒ 同＝%s" % (i + 1, x[0], x[1], y[0], y[1], same))
print("★逐行同值（名＋判定）＝", "True" if (len(a) == len(b) and not diff) else "False", "｜差异条数＝%d" % len(diff))
for i, x, y in diff:
    print("  差异第 %d 条：REG=%s|%s ‖ CP=%s|%s" % (i, x[0], x[1], y[0], y[1]))
sa = set(n for n, _ in a)
sb = set(n for n, _ in b)
print("名集差（CP−REG）＝%s｜（REG−CP）＝%s" % (sorted(sb - sa), sorted(sa - sb)))
fa = set(n for n, v in a if v == "FAIL")
fb = set(n for n, v in b if v == "FAIL")
print("FAIL 集：REG＝%s ‖ CP＝%s ‖ 新增＝%s ‖ 消失＝%s" % (sorted(fa), sorted(fb), sorted(fb - fa), sorted(fa - fb)))
for tag, path in (("REG", sys.argv[3]), ("CP", sys.argv[4])):
    names, cnt, rec = plugins(path)
    print("C11｜%s Using plugin 行=%d 层名=%s ‖ Received a 行=%d" % (tag, cnt, names, rec))
print("== 本件只报数；判读见 06 件 ==")
