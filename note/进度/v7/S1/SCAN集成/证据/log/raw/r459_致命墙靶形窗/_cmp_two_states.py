#!/usr/bin/env python3
# _cmp_two_states.py —— 两态对照件（★栏一（名，判定）＋栏二 C4 同因同码 两条并列★）
# 用法：_cmp_two_states.py <基准.txt> <对照.txt> <基准dump.yaml> <对照dump.yaml>
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


base = parse(sys.argv[1])
other = parse(sys.argv[2])
vb = [(n, v) for n, v, _ in base]
vo = [(n, v) for n, v, _ in other]
fb = dict((n, m) for n, v, m in base if v == "FAIL")
fo = dict((n, m) for n, v, m in other if v == "FAIL")
n = max(len(vb), len(vo))
diff = [i + 1 for i in range(n) if (vb[i] if i < len(vb) else None) != (vo[i] if i < len(vo) else None)]
common = sorted(set(fb) & set(fo))
c4bad = [k for k in common if fb[k] != fo[k]]
for tag, s in (("基准", base), ("对照", other)):
    print("%s＝%d 条｜PASS=%d FAIL=%d" % (tag, len(s), sum(1 for _, v, _ in s if v == "PASS"), sum(1 for _, v, _ in s if v == "FAIL")))
print("名集逐字同＝", [x[0] for x in vb] == [x[0] for x in vo])
print("★栏一（名，判定）异行数＝%d★ %s" % (len(diff), diff or ""))
print("★栏二 C4＝%d 条共同 FAIL、不等条数＝%d★" % (len(common), len(c4bad)))
for k in c4bad:
    print("  C4 不等｜%s｜基准=%s" % (k, fb[k][:120]))
    print("           对照=%s" % fo[k][:120])
print("新增红（对照−基准）＝%s｜消失红（基准−对照）＝%s" % (sorted(set(fo) - set(fb)), sorted(set(fb) - set(fo))))
la = [l.rstrip("\n") for l in open(sys.argv[3], encoding="utf-8", errors="replace")]
lb = [l.rstrip("\n") for l in open(sys.argv[4], encoding="utf-8", errors="replace")]
print("参数面 live dump｜只在基准侧 %d 行｜只在对照侧 %d 行" % (len([x for x in la if x not in lb]), len([x for x in lb if x not in la])))
for x in [x for x in la if x not in lb][:6]:
    print("  − %s" % x[:130])
for x in [x for x in lb if x not in la][:6]:
    print("  ＋ %s" % x[:130])
print("== 本件只报数；判读见 06 件 ==")
