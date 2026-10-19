#!/usr/bin/env python3
# _cmp_r462.py —— ★三式收口窗的对照件★：栏一（名，判定）＋栏二 C4 同因同码＋★C1-C 键路径展平（含每态 T1／T2 自等）★＋★C7-B 内容栏逐格★
# 用法：_cmp_r462.py <REF> <A> <Ap> <B> <C> <dump目录> <csv目录>
# ★只报数、不判读★；★数字比较一律带 r459q 在册容差（占据格 ±600／点云 ±1.25Hz／恒等字段 0／端口不比）★
import glob
import os
import re
import sys

import yaml

TOL_OCC = 600
TOL_HZ = 1.25


def parse(path):
    seq = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            if line.startswith("CHECK|"):
                p = line.rstrip("\n").split("|")
                if len(p) >= 4:
                    seq.append((p[1], p[2], "|".join(p[3:])))
    return seq


def vec(s):
    return [(n, v) for n, v, _ in s]


def fails(s):
    return dict((n, m) for n, v, m in s if v == "FAIL")


def pair(base, other, tag):
    vb, vo = vec(base), vec(other)
    n = max(len(vb), len(vo))
    d1 = [i + 1 for i in range(n) if (vb[i] if i < len(vb) else None) != (vo[i] if i < len(vo) else None)]
    fb, fo = fails(base), fails(other)
    common = sorted(set(fb) & set(fo))
    bad = [k for k in common if fb[k] != fo[k]]
    print("── %s：栏一（名，判定）异行数＝%d%s ‖ 栏二 C4＝共同 FAIL %d 条、不等 %d 条%s ‖ 新增红 %s 消失红 %s" % (
        tag, len(d1), ("(第 %s 条)" % d1 if d1 else ""), len(common), len(bad), ("→ %s" % bad if bad else ""),
        sorted(set(fo) - set(fb)), sorted(set(fb) - set(fo))))
    for k in bad:
        print("     C4 不等｜%s｜基准=%s" % (k, fb[k][:100]))
        print("              %s｜对照=%s" % (" " * len(k), fo[k][:100]))


def flat(path):
    d = yaml.safe_load(open(path, encoding="utf-8"))
    out = []

    def walk(node, pre):
        if isinstance(node, dict):
            for k, v in node.items():
                walk(v, pre + "/" + str(k))
        elif isinstance(node, list):
            for i, v in enumerate(node):
                walk(v, pre + "[%d]" % i)
        else:
            out.append("%s=%s" % (pre, node))
    walk(d, "")
    return sorted(out)


def cells(path):
    with open(path, encoding="utf-8") as f:
        meta = f.readline().strip().split(",")
        data = f.readline().strip().split(",")
    w, h, res, ox, oy = int(meta[1]), int(meta[2]), float(meta[3]), float(meta[4]), float(meta[5])
    m = {}
    i = 0
    for r in range(h):
        for c in range(w):
            wx = round(ox + (c + 0.5) * res, 2)
            wy = round(oy + (r + 0.5) * res, 2)
            m[(wx, wy)] = int(data[i])
            i += 1
    return m, w, h


def band_read(pa, pc, lo, hi):
    """★靶区专项：把两侧格按世界 x 落在 [lo,hi] 的格挑出来，数 C 里致命而 A 里不致命的格数★（★全局 D 会被层栈与 STVL 时变盖过，这一栏才是靶的直接读数★）"""
    ma, _, _ = cells(pa)
    mc, _, _ = cells(pc)
    ks = [k for k in set(ma) & set(mc) if lo <= k[0] <= hi]
    up = sum(1 for k in ks if mc[k] >= 90 > ma[k])
    dn = sum(1 for k in ks if ma[k] >= 90 > mc[k])
    both_l = sum(1 for k in ks if ma[k] >= 90 and mc[k] >= 90)
    print("   靶区专项｜x∈[%.2f,%.2f] 交集格=%d ‖ ★C 致命而 A 不致命=%d 格★ ‖ A 致命而 C 不致命=%d ‖ 两侧皆致命=%d" % (lo, hi, len(ks), up, dn, both_l))


def content(pa, pb, tag):
    ma, wa, ha = cells(pa)
    mb, wb, hb = cells(pb)
    both = set(ma) & set(mb)
    d = sum(1 for k in both if ma[k] != mb[k])
    la = sum(1 for v in ma.values() if v >= 90)
    lb = sum(1 for v in mb.values() if v >= 90)
    print("   %s｜X_基准=%d X_对照=%d Δ=%+d（容差 ±%d ⇒ %s）‖ 逐格差异 D=%d ‖ 格集交集=%d 仅基准=%d 仅对照=%d ‖ 几何 %dx%d vs %dx%d" % (
        tag, la, lb, lb - la, TOL_OCC, "在容差内" if abs(lb - la) <= TOL_OCC else "★超容差★",
        d, len(both), len(set(ma) - set(mb)), len(set(mb) - set(ma)), wa, ha, wb, hb))
    return d


ddir, cdir = sys.argv[6], sys.argv[7]
D = os.path.dirname(os.path.dirname(os.path.abspath(ddir)))
root_dir = sys.argv[8] if len(sys.argv) > 8 else ""
seqs = dict((n, parse(sys.argv[i + 1])) for i, n in enumerate(["REF", "A", "Ap", "B", "C"]))
for k in ["REF", "A", "Ap", "B", "C"]:
    s = seqs[k]
    print("%s＝%d 条｜PASS=%d FAIL=%d" % (k, len(s), sum(1 for _, v, _ in s if v == "PASS"), sum(1 for _, v, _ in s if v == "FAIL")))
print("名集（五态）逐字同＝", all([n for n, _ in vec(seqs[k])] == [n for n, _ in vec(seqs["A"])] for k in seqs))
print("REF vs A（★在册形 vs A 侧同形重排副本＝语义等价的判定面复核★）")
pair(seqs["REF"], seqs["A"], "REF vs A")
pair(seqs["A"], seqs["Ap"], "Ap vs A（★C6 同窗噪声地板★）")
pair(seqs["A"], seqs["B"], "B vs A（★加一枚同源 MAX 层★）")
pair(seqs["A"], seqs["C"], "C vs A（★C7-B 靶＝致命墙喂第二枚层★）")

print("== C1-C：参数面键路径展平（★每态 T1／T2 先自等★）==")
for st in ["A", "Ap", "B", "C"]:
    p1 = os.path.join(ddir, "dump_%s_T1.yaml" % st)
    p2 = os.path.join(ddir, "dump_%s_T2.yaml" % st)
    if not (os.path.exists(p1) and os.path.exists(p2)):
        print("  %s｜★T1／T2 不齐★＝%s %s" % (st, os.path.exists(p1), os.path.exists(p2)))
        continue
    f1, f2 = flat(p1), flat(p2)
    print("  %s｜T1 行数=%d T2 行数=%d ‖ ★自等＝%s★（差 %d 项：%s）" % (
        st, len(f1), len(f2), f1 == f2, len(set(f1) ^ set(f2)), sorted(set(f1) ^ set(f2))[:4]))
ref = flat(os.path.join(ddir, "dump_A_T1.yaml"))
for st in ["B", "C"]:
    cur = flat(os.path.join(ddir, "dump_%s_T1.yaml" % st))
    only_c = [x for x in cur if x not in ref]
    only_r = [x for x in ref if x not in cur]
    # ★键路径以 dump 的命名空间为根（我 dump 的就是 global_costmap）⇒ 前缀里没有 /global_costmap；
    #   白名单判定按★展平后的路径起始★做，不按"整串包含"（上一版用包含式 ⇒ 20 项全被误报成非白名单＝★又一个截断/形不对的自伤★）
    okp = lambda k: k.startswith("/plugins[") or k.startswith("/elevation_layer/")
    bad = [x for x in only_c if not okp(x.split("=")[0])]
    bad += [x for x in only_r if not okp(x.split("=")[0])]
    print("  A↔%s｜只在 %s 侧 %d 项｜只在 A 侧 %d 项｜★非白名单项=%d★%s" % (
        st, st, len(only_c), len(only_r), len(bad), ("→ %s" % bad[:3] if bad else "")))
    for x in only_c[:6]:
        print("     ＋ %s" % x[:130])


print("== C1-B 载体面＝★路径归一式★（r459q 要求：载体副本只改指向 ⇒ 把所有路径字面换成同一记号后应逐行同）==")


def norm(line):
    s = line.strip()
    s = re.sub(r"\$\(find [A-Za-z0-9_]+\)/[^\s\"\']+", "@PATH@", s)
    s = re.sub(r"\$\(env [A-Za-z0-9_]+\)", "@PATH@", s)
    # ★上一版这里用 [A-Za-z0-9_./\-] 的字符类 ⇒ 含中文的绝对路径段根本没被换成记号（mode／smoke 各报 1 行"归一后仍不同"＝工具的形不对、不是副本改错）
    s = re.sub(r"/[^\s\"'<>]*\.(?:ya?ml|launch|sh)", "@PATH@", s)
    s = re.sub(r"[A-Za-z0-9_]+\.(?:ya?ml|launch|sh)", "@PATH@", s)   # ★第三轮补＝"裸件名.扩展名"也是路径字面（在册那行是 `lite3_bringup mode_nav_sim.launch`＝包名+裸件名、不带斜杠，前两轮的字符类都盖不住它）
    return s


for tag, reg, cp in (("mbf", "ws/src/40_navigation/lite3_navigation/launch/mbf_navigation.launch", "mbf_navigation_r462.launch"),
                     ("mode", "ws/src/70_bringup/lite3_bringup/launch/mode_nav_sim.launch", "mode_nav_sim_r462.launch"),
                     ("smoke", "ws/src/70_bringup/lite3_bringup/verify/sim_nav_smoke.sh", "sim_nav_smoke_r462.sh")):
    try:
        a = open(os.path.join(root_dir, reg), encoding="utf-8", errors="replace").read().splitlines()
        b = open(os.path.join(D, cp), encoding="utf-8", errors="replace").read().splitlines()
    except Exception as e:
        print("  %s｜★取样不可用＝%s★" % (tag, e))
        continue
    dline = []
    if len(a) != len(b):
        dline.append("行数不等 %d vs %d" % (len(a), len(b)))
    for k in range(min(len(a), len(b))):
        if norm(a[k]) != norm(b[k]):
            dline.append("第%d行｜在=%s ‖ 副=%s" % (k + 1, norm(a[k])[:80], norm(b[k])[:80]))
    raw = [k + 1 for k in range(min(len(a), len(b))) if a[k] != b[k]]
    print("  %s｜★归一后差异行数=%d★（%s）‖ 归一前的字面差异行=%s" % (tag, len(dline), "; ".join(dline[:2]) or "逐行同", raw))

print("== C7-B：内容栏（★世界坐标对齐的逐格差异★；D_floor＝同参数面双跑）==")
try:
    content(os.path.join(cdir, "cm_A.csv"), os.path.join(cdir, "cm_Ap.csv"), "D_floor｜A vs Ap")
    content(os.path.join(cdir, "cm_A.csv"), os.path.join(cdir, "cm_B.csv"), "A vs B")
    content(os.path.join(cdir, "cm_A.csv"), os.path.join(cdir, "cm_C.csv"), "A vs C（★靶在这一栏留下多少逐格差异★）")
    band_read(os.path.join(cdir, "cm_A.csv"), os.path.join(cdir, "cm_C.csv"), 0.25, 1.35)
    band_read(os.path.join(cdir, "cm_A.csv"), os.path.join(cdir, "cm_B.csv"), 0.25, 1.35)
except Exception as e:
    print("  ★内容栏取样不可用＝%s★" % e)
print("== 本件只报数；判读见 06 件 ==")
