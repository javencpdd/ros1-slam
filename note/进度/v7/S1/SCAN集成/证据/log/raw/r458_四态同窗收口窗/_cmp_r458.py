#!/usr/bin/env python3
# _cmp_r458.py —— 四态对照件：★两条栏并列★（r456q 点名的半步）
#   栏一＝（CHECK 名，判定）二元组向量（C6 允许的比法）＝逐位置比、报"异行数"
#   栏二＝C4 同因同码＝对每条★共同 FAIL★行比 outcome 数字＋message 串＋连带句（★逐字★）＝报"不等条数"
# 用法：_cmp_r458.py <A.txt> <Ap.txt> <B.txt> <C.txt> [dumpA.txt dumpB.txt dumpC.txt]
# ★本件只报数、不判读★（判读写在 06 件）
import sys


def parse(path):
    seq = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            if line.startswith("CHECK|"):
                p = line.rstrip("\n").split("|")
                if len(p) >= 4:
                    seq.append((p[1], p[2], "|".join(p[3:])))
    return seq


def vec(seq):
    return [(n, v) for n, v, _ in seq]


def fails(seq):
    return dict((n, m) for n, v, m in seq if v == "FAIL")


def pairdiff(base, other, tag):
    vb, vo = vec(base), vec(other)
    n = max(len(vb), len(vo))
    diff = [i + 1 for i in range(n) if (vb[i] if i < len(vb) else None) != (vo[i] if i < len(vo) else None)]
    fb, fo = fails(base), fails(other)
    common = sorted(set(fb) & set(fo))
    c4bad = [k for k in common if fb[k] != fo[k]]
    print("── %s vs 基准：CHECK 条数 %d／%d ‖ ★栏一（名，判定）异行数＝%d★ %s" % (
        tag, len(vb), len(vo), len(diff), diff or ""))
    print("   ★栏二 C4（共同 FAIL 行逐字比 outcome／message／连带句）＝%d 条共同 FAIL、不等条数＝%d★" % (len(common), len(c4bad)))
    for k in c4bad:
        print("     C4 不等｜%s｜基准=%s" % (k, fb[k][:110]))
        print("             %s｜对照=%s" % (" " * len(k), fo[k][:110]))
    print("   新增红（对照−基准）＝%s｜消失红（基准−对照）＝%s" % (
        sorted(set(fo) - set(fb)), sorted(set(fb) - set(fo))))


names = ["A", "Ap", "B", "C"]
seqs = {}
for i, p in enumerate(sys.argv[1:5]):
    seqs[names[i]] = parse(p)
for k in names:
    s = seqs[k]
    print("%s＝%d 条｜PASS=%d FAIL=%d" % (k, len(s), sum(1 for _, v, _ in s if v == "PASS"), sum(1 for _, v, _ in s if v == "FAIL")))
print("名集（四态）逐字同＝", all(vec(seqs[x]) and [n for n, _ in vec(seqs[x])] == [n for n, _ in vec(seqs["A"])] for x in names))
pairdiff(seqs["A"], seqs["Ap"], "Ap（在册形第二遍＝同窗噪声地板）")
pairdiff(seqs["A"], seqs["B"], "B（副本形＝加一枚同源 MAX 层）")
pairdiff(seqs["A"], seqs["C"], "C（副本形＋给第二实例喂刻意抬高代价的图＝r448q C7 原式靶形）")
if len(sys.argv) > 7:
    print("== 参数面语义等价（★只报命中行数、不解释★）==")
    for tag, a, b in (("A↔B", sys.argv[5], sys.argv[6]), ("B↔C", sys.argv[6], sys.argv[7])):
        la = [l.rstrip("\n") for l in open(a, encoding="utf-8", errors="replace")]
        lb = [l.rstrip("\n") for l in open(b, encoding="utf-8", errors="replace")]
        only_a = [x for x in la if x not in lb]
        only_b = [x for x in lb if x not in la]
        print("  %s｜只在 A 侧 %d 行｜只在 B 侧 %d 行" % (tag, len(only_a), len(only_b)))
        for x in only_a[:6]:
            print("    − %s" % x[:130])
        for x in only_b[:6]:
            print("    ＋ %s" % x[:130])
print("== 本件只报数；判读见 06 件 ==")
