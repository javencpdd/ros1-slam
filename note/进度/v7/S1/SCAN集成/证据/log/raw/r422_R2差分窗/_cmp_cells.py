import io
import os
import re
import sys
# ★只读取两件 cost 原始件做逐格对照（解析式与在册先例 r419 run/_parse.py 逐字同源：data: 之后取前 1600 个整数）
G6 = os.environ.get("G6", "/opt/ros1-slam/note/进度/SCAN集成/证据/log/raw/r419_M3归因窗/run/cost_x2_g6.txt")
D2 = os.environ.get("D2", "/opt/ros1-slam/note/进度/SCAN集成/证据/log/raw/r422_R2差分窗/run/cost_x2_r422_d2.txt")
FR, FC = (18, 21), (29, 32)


def load(p, tag):
    t = io.open(p, encoding="utf-8", errors="ignore").read()
    if "data:" not in t:
        print("★%s 无 data: 面（未采到）⇒ 本段记 N3" % tag); return None
    v = [int(x) for x in re.findall(r"-?\d+", t.split("data:", 1)[1])][:1600]
    if len(v) != 1600:
        print("★%s 格数=%d（须 1600）⇒ 本段记 N3" % (tag, len(v))); return None
    print("  %s 格数=%d ‖ 值集=%s" % (tag, len(v), sorted(set(v))))
    return v


a = load(G6, "g6（R419 四插件参照）")
b = load(D2, "d2（本窗★去 stvl）")
if a is None or b is None:
    sys.exit(5)
g = lambda v, r, c: v[r * 40 + c]
print("\n足印 rows18-21 × cols29-32 逐格（★痕迹 9 格就在这一带）：")
print("      " + "".join("  c%-2d:g6/d2" % c for c in range(FC[0], FC[1] + 1)))
for r in range(FR[0], FR[1] + 1):
    print("r%-3d " % r + "".join("  %3d/%-3d" % (g(a, r, c), g(b, r, c)) for c in range(FC[0], FC[1] + 1)))
ta = [i for i in range(1600) if (i // 40) in range(FR[0], FR[1] + 1) and (i % 40) in range(FC[0], FC[1] + 1) and a[i] not in (0,)]
tb = [i for i in range(1600) if (i // 40) in range(FR[0], FR[1] + 1) and (i % 40) in range(FC[0], FC[1] + 1) and b[i] not in (0,)]
print("\n★二值判定量（★交判定件自报，本件只并报数、不自判顶档）＝足印内非零格 g6=%d ‖ d2=%d" % (len(ta), len(tb)))
print("  全图差异格数=%d／1600 ‖ g6 非零=%d ‖ d2 非零=%d" % (sum(1 for i in range(1600) if a[i] != b[i]),
      sum(1 for x in a if x != 0), sum(1 for x in b if x != 0)))
print("  值 99 格数 g6=%d ‖ d2=%d（★叠加项族，见 R386 逐格加强判）" % (a.count(99), b.count(99)))
