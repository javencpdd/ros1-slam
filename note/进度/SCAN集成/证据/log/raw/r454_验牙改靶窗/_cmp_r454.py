#!/usr/bin/env python3
# _cmp_r454.py —— 只报数、不判读（判读写在 06 件里，由我按在册 C7 定义判）
# 用法：_cmp_r454.py <控制跑输出> <打断前快照> <注入跑输出>
import sys

def parse(path):
    seq = []
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            for line in f:
                if line.startswith("CHECK|"):
                    p = line.rstrip("\n").split("|")
                    if len(p) >= 3:
                        seq.append((p[1], p[2]))
    except Exception as e:
        return None, "READ_FAIL %s" % e
    return seq, None

def fails(seq):
    return sorted(set(n for n, v in seq if v == "FAIL"))

def names(seq):
    return [n for n, _ in seq]

ct, e1 = parse(sys.argv[1])
pre, e2 = parse(sys.argv[2]) if len(sys.argv) > 2 else (None, "NO_ARG")
inj, e3 = parse(sys.argv[3]) if len(sys.argv) > 3 else (None, "NO_ARG")

print("控制跑＝%s 条｜PASS=%s FAIL=%s" % (
    len(ct) if ct is not None else "None",
    sum(1 for _, v in ct if v == "PASS") if ct else "-",
    sum(1 for _, v in ct if v == "FAIL") if ct else "-"))
if ct:
    print("  控制跑 FAIL 名集＝%s" % fails(ct))
    print("  控制跑名序＝%s" % names(ct))

if pre is None:
    print("★打断前快照不可用（%s）⇒ 本窗＝注入未发生，C7 判 INVALID、不得据此说有牙或没牙★" % (e2 or "EMPTY"))
    sys.exit(0)
if inj is None:
    print("★注入跑输出不可用（%s）⇒ INVALID★" % (e3 or "EMPTY"))
    sys.exit(0)

print("打断前＝%s 条｜名序＝%s" % (len(pre), names(pre)))
print("注入跑＝%s 条｜PASS=%s FAIL=%s" % (
    len(inj),
    sum(1 for _, v in inj if v == "PASS"),
    sum(1 for _, v in inj if v == "FAIL")))
print("  注入跑 FAIL 名集＝%s" % fails(inj))

cf, nf = set(fails(ct)) if ct else set(), set(fails(inj))
print("新增 FAIL（注入跑 − 控制跑）＝%s" % sorted(nf - cf))
print("消失 FAIL（控制跑 − 注入跑）＝%s" % sorted(cf - nf))

pre_set = set(names(pre))
pend = [n for n in names(ct) if n not in pre_set]
print("控制跑里打断后才会印的名＝%s" % pend)
print("其中在注入跑印出的＝%s｜其中注入跑判 FAIL 的＝%s" % (
    [n for n in pend if n in set(names(inj))],
    [n for n in pend if n in nf]))
print("注入跑名序＝%s" % names(inj))
print("== 本件只报数：有没有牙由 06 判读按 C7 定义判 ==")
