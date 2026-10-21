#!/usr/bin/env python3
# r534_dryrun.py —— ★开窗前★把 R534 三枚件件面各跑一遍（★全在 /tmp、仓内只读★）＋★新牙控★
import io, os, re, sys, glob, shutil, subprocess

R = "/home/jack/ysc-code/ros1-slam"
REL = "note/进度/SCAN集成/证据/log/raw"
K = glob.glob("/tmp/r534_kit/r534_*")[0]
W530 = glob.glob(os.path.join(R, REL, "r530_*"))[0]
T = "/tmp/r534_dry"; shutil.rmtree(T, ignore_errors=True)
raw = os.path.join(T, "fr", REL); os.makedirs(raw)
DATA = os.path.join(T, "data"); shutil.copytree(W530, DATA)          # ★窗数据副本＝写盘都落这里★
os.symlink(DATA, os.path.join(raw, os.path.basename(DATA)))
os.symlink(W530, os.path.join(raw, "r472_收口窗重跑规范架与出处随行与勾对表"))  # 被引窗＝只读符号链接
def run(cmd): return subprocess.run(cmd, capture_output=True, text=True, errors="replace")
def show(tag, r, extra=""):
    nums = sorted(set(x for tup in re.findall(r"rc_勾对_复读件=(\d+)|即将以 rc=(\d+) 退出|按红档退出", r.stdout) for x in tup if x))
    print("%-10s｜rc=%s ‖ 打印的数=%s ‖ Assert=%s ‖ KeyError=%s ‖ %s%s" % (
        tag, r.returncode, nums or "无", "红" if "AssertionError" in r.stderr else "无",
        "有" if "KeyError" in r.stderr else "无", extra,
        ("｜末行＝" + r.stderr.strip().split("\n")[-1][:52]) if r.stderr.strip() else ""))
SRC = os.path.basename(DATA)
RECHECK = os.path.join(K, "01_复读件_r534.py")
print("── A＝复读件（G1＋G2）──")
show("正测", run([sys.executable, RECHECK, os.path.join(T, "fr"), SRC, SRC]))
show("swap", run([sys.executable, RECHECK, os.path.join(T, "fr"), SRC, SRC, "--swap"]))
base = io.open(RECHECK, encoding="utf-8").read()
def m(tag, a, b, args):
    p = os.path.join(T, "mm.py"); io.open(p, "w", encoding="utf-8").write(base.replace(a, b, 1))
    assert base != io.open(p, encoding="utf-8").read(), tag + " 变异未命中"
    show(tag, run([sys.executable, p, os.path.join(T, "fr"), SRC, SRC] + args))
m("G2键拼错", 'BR_BY_SWAP = "swap" if SWAP else "premiss"', 'BR_BY_SWAP = "sw" if SWAP else "premiss"', ["--swap"])
m("名对调", '"swap": (5, "负例分支＝替换自测（设计来翻红的那支、退码＝5）")', '"swap": (5, "前置缺失档＝被引件或清单出处件不在位（退码＝9）")', [])
m("G1逃逸", 'bye("zero" if PREMISS == 0 else "premiss")', 'sys.exit(0)', [])
print("── B＝对照件（G5＋G11）──")
CMP = os.path.join(K, "_cmp_r534.py")
args = [sys.executable, CMP] + [os.path.join(DATA, "run", "smoke_%s.txt" % s) for s in ("REF", "A", "Ap", "C")] + [
    os.path.join(DATA, "run", "dump"), os.path.join(DATA, "run", "csv"), os.path.join(DATA, "run"),
    os.path.join(DATA, "00_window_log.txt"), R]
r = run(args)
o = r.stdout
print("  rc=%s ‖ 标题这句由 INV_DONE 导出＝%s ‖ 合取项枚数=%s ‖ rc_机检=%s" % (
    r.returncode, re.findall(r"★标题这句由 INV_DONE 现取＝(.{0,34})", o), len(re.findall(r"合取项｜", o)),
    re.findall(r"rc_机检＝(\S+)（", o)[:1]))
print("  ★不变量枚是否进了合取项＝%s★ ‖ 全文「未行使」枚数=%d" % ("rc_不变量" in o and len(re.findall(r"合取七", o)), len(re.findall("未行使", o))))
shutil.copy(CMP, os.path.join(DATA, "_cmp_r534.py")); shutil.copy(RECHECK, os.path.join(DATA, "01_复读件_r534.py"))
print("── C＝判读件（G3＋G4＋G8）──")
PR = os.path.join(K, "06_判读件_r534.py")
shutil.copy(PR, os.path.join(DATA, "06_判读件_r534.py"))
r = run([sys.executable, os.path.join(DATA, "06_判读件_r534.py"), os.path.join(T, "fr"), SRC])
print("  正测 rc=%s ‖ 面标签＝%s ‖ 红档枚数=%s" % (r.returncode, re.findall(r"跑在★(.{0,18}?)★面", r.stdout), re.findall(r"红档枚数＝(\d+) 枚", r.stdout)))
tp = os.path.join(DATA, "09_锚键.tsv"); rows = io.open(tp, encoding="utf-8").read().rstrip("\n").split("\n")
f = rows[4].split("\t"); f[2] = "无"; rows[4] = "\t".join(f)
io.open(tp, "w", encoding="utf-8").write("\n".join(rows) + "\n")
r = run([sys.executable, os.path.join(DATA, "06_判读件_r534.py"), os.path.join(T, "fr"), SRC])
print("  ★G3＝表字段「无」★ rc=%s ‖ ValueError=%s ‖ 红档枚数=%s ‖ 点名=%s" % (
    r.returncode, "ValueError" in r.stderr, re.findall(r"红档枚数＝(\d+) 枚", r.stdout),
    re.findall(r"表字段非数字[^★]*★?", r.stdout)[:1]))
io.open(tp, "w", encoding="utf-8").write("\n".join(io.open(PR, encoding="utf-8").read() and rows) + "\n") if False else None
lg = os.path.join(DATA, "00_window_log.txt"); L = [l for l in io.open(lg, encoding="utf-8").read().split("\n") if "事后复现" not in l]
io.open(lg, "w", encoding="utf-8").write("\n".join(L) + "\n")
r = run([sys.executable, os.path.join(DATA, "06_判读件_r534.py"), os.path.join(T, "fr"), SRC])
print("  ★G4＝删事后复现句★ rc=%s ‖ 断言(不含 None)红=%s ‖ 输出含「第 None 行」=%s ‖ 输出含「第 ★缺★ 行」=%s" % (
    r.returncode, "AssertionError" in r.stderr and "None" in r.stderr, "第 None 行" in r.stdout, "★缺★ 行" in r.stdout))
