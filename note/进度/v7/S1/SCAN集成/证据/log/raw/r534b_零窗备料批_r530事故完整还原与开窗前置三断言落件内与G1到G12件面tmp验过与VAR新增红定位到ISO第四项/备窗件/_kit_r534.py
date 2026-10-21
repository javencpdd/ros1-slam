#!/usr/bin/env python3
# 批186（R530）套件生成件＝从 r525 套件复制＋★落 F1—F9 九枚件内改形★（★r525 全套一字不动＝不破已入库清单★）
# 自证四段＝①字节>0 ②bash -n／ast.parse 语法过 ③★每枚 F 的替换点计数＝1（替换失败即中止）★ ④★开窗前把新加的三枚自测件单独跑一遍★（正控＋反控）
import io, os, re, shutil, subprocess, sys

R = "/home/jack/ysc-code/ros1-slam/"
RAW = R + "note/进度/SCAN集成/证据/log/raw"
SRC = [d for d in os.listdir(RAW) if d.startswith("r525_活")][0]
NAME = "r534_活体窗落九枚件内改形_F1钉打印数与退出码同枚与F2名码同表与F3条件挂条件与F4查不到有红档与F5三数并报与F6黑名单同构与F7语义锚与F8不凑三枚与F9事后复现"  # ★213 字节★（首版 296 字节＝ext4 单段名上限 255＝撞墙＝★命名面也按字节计★）
DST = os.path.join(RAW, NAME)
if os.path.exists(DST):
    shutil.rmtree(DST)
shutil.copytree(os.path.join(RAW, SRC), DST)
# 清掉窗产物（新目录只留★件本体★＋让路载体不带走）
for drop in ("run", "00_window_log.txt", "05_cmp.txt", "05_cmp_复跑.txt", "07_复读.txt", "08_复读_swap负例.txt",
             "09_锚键.tsv", "06_判读.txt", "06_判读件_r525.py", ".runs", "_sha256_manifest.txt",
             "01_preflight.txt", "03_delta.txt", "zz_probe.txt", "02_make_copies_r534.py"):
    p = os.path.join(DST, drop)
    if os.path.isdir(p):
        shutil.rmtree(p)
    elif os.path.isfile(p):
        os.remove(p)
for f in os.listdir(DST):
    if f.startswith(".prev_") or f.startswith("global_costmap_params_") or f.endswith("_r525.launch") \
       or f.endswith("_r525.yaml") or f.startswith("sim_nav_smoke_r525") or f.startswith("sub_elev") \
       or f.startswith("mode_nav_sim_r525"):
        p = os.path.join(DST, f)
        shutil.rmtree(p) if os.path.isdir(p) else os.remove(p)

WS = [p for p in os.listdir(DST)]
print("新目录件清单＝", sorted(WS))
N4 = "F1—F9"


def repl(path, reps, tag):
    p = os.path.join(DST, path)
    s = io.open(p, encoding="utf-8").read()
    for old, new in reps:
        n = s.count(old)
        assert n == 1, ("★%s 替换点计数＝%d（须＝1）★" % (tag, n), old[:70])
        s = s.replace(old, new)
    io.open(p, "w", encoding="utf-8").write(s)
    print("  ✔ %s ‖ 替换 %d 处" % (tag, len(reps)))


# ───────── ① 复读件＝F1 两半同批＋断言钉「打印数＝退出数」＋F2 名↔码同表 ─────────
repl("01_复读件_r525.py", [
    ('RC_BY_BR = {"swap": 5, "premiss": 9}\nNAME_BY_BR = {"swap": "负例分支＝替换自测（设计来翻红的那支）", "premiss": "前置缺失档＝被引件或清单出处件不在位"}',
     'BR_TABLE = {"swap": (5, "负例分支＝替换自测（设计来翻红的那支、退码＝5）"),\n'
     '            "premiss": (9, "前置缺失档＝被引件或清单出处件不在位（退码＝9）")}\n'
     'RC_BY_BR = {k: v[0] for k, v in BR_TABLE.items()}\n'
     'NAME_BY_BR = {k: v[1] for k, v in BR_TABLE.items()}   # ★F2/R530＝名与码★同一张表的同一行★＝H5 那类「只把两枚名对调」结构上不可造★'),
    ('assert RC_BY_BR.keys() == NAME_BY_BR.keys() and sorted(RC_BY_BR.values()) == [5, 9], "★两张表不同构＝一名一码不许复用★"',
     'assert RC_BY_BR.keys() == NAME_BY_BR.keys() and sorted(RC_BY_BR.values()) == [5, 9], "★两张表不同构＝一名一码不许复用★"\n'
     'assert all(str(RC_BY_BR[k]) in NAME_BY_BR[k] for k in BR_TABLE), "★名里没有自己的码＝名↔码配对无断言（r525q S26／H5 那一族）★"'),
    ('RC_RED = RC_BY_BR[BR_BY_SWAP]\nRC_RED = 5 if SWAP else 9   # ★S5（r517q 缺席清单 S5）＝红档的★码★与★名★共一枚变量；名由码查表导出、不再两处手写★',
     'RC_RED = RC_BY_BR[BR_BY_SWAP]   # ★F1/R530＝删掉★被字面量覆盖的那一行★（r525q 的 J 变异证明它是★死码★）★'),
    ('    print("   ★清单出处件不在位 ⇒ rc_勾对_复读件=%s（不猜清单）‖ 并记「%s」★（★S13/R525＝★第二支★的名与码也从同一张表导出★）★" % (RC_BY_BR["swap"], NAME_BY_BR["premiss"]))\n    PREMISS = 9',
     '    PRN = RC_BY_BR["premiss"]   # ★F1/R530＝★键★也取自分支名册（r525 取的是★swap★行＝印 5 退 9）★\n'
     '    print("   ★清单出处件不在位 ⇒ rc_勾对_复读件=%s（不猜清单）‖ 并记「%s」★（★F1/R530＝打印的数与件尾退码★同枚★导出＋下面当场断言钉相等★）★" % (PRN, NAME_BY_BR["premiss"]))\n'
     '    PREMISS = RC_BY_BR["premiss"]   # ★F1/R530＝退码不再手写字面量（r525q m5：9→7 零翻红＝无牙）★\n'
     '    assert PRN == PREMISS, "★件尾打印的数≠实际退码＝名实又脱钩（S13 那一族的★牙★）★"'),
], "复读件 F1／F2")
# ───────── ② 文件名与件内自称★窗号★同步（★历史引用串 r525q／r525 那版★保持原形★） ─────────
for f in sorted(os.listdir(DST)):
    fp = os.path.join(DST, f)
    if os.path.isfile(fp) and (f.endswith((".py", ".sh"))):
        t = io.open(fp, encoding="utf-8").read()
        t = t.replace(SRC, NAME).replace("_r525", "_r534").replace("R525", "R530").replace("r525_yield_", "r534_yield_")
        io.open(fp, "w", encoding="utf-8").write(t)
        nf = f.replace("_r525", "_r534")
        if nf != f:
            os.rename(fp, os.path.join(DST, nf))
print("  ✔ 件内自称窗号与文件名已同步（★只动★窗号形★，r525q 等历史串未动★）")
for f in sorted(os.listdir(DST)):
    fp = os.path.join(DST, f)
    if os.path.isfile(fp):
        for n, l in enumerate(io.open(fp, encoding="utf-8", errors="replace").read().split("\n"), 1):
            if "r525" in l.replace("r525q", "").replace("_r534", "").replace("r534", ""):
                print("     ★残留 r525 形★＝%s:%d ‖ %s" % (f, n, l.strip()[:110]))

# ───────── ④ 语法与四段自证 ─────────
sh = os.path.join(DST, "00_r534_window.sh")
r = subprocess.run(["bash", "-n", sh], capture_output=True, text=True)
assert r.returncode == 0, ("bash -n 不过", r.stderr[:300])
print("  ✔ bash -n rc=0 ‖ 行数＝", io.open(sh, encoding="utf-8").read().count("\n"))
for f in os.listdir(DST):
    if f.endswith(".py"):
        import ast
        ast.parse(io.open(os.path.join(DST, f), encoding="utf-8", errors="replace").read())
print("  ✔ 全部 .py ast.parse 过")
for f in sorted(os.listdir(DST)):
    print("     %-34s ‖ %d 字节 ‖ %d 行" % (f, os.path.getsize(os.path.join(DST, f)),
                                            io.open(os.path.join(DST, f), encoding="utf-8", errors="replace").read().count("\n")))
print("新目录＝", DST)
