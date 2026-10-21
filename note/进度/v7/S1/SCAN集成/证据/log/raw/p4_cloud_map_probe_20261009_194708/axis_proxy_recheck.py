"""axis_proxy_recheck.py —— 把 §四 用到的两个代理显式量化（只读诊断，不进 gate、不改任何判据）。

对 R57＋R63 九跑，逐跑算三种"注入点相对机体"的代理方位：
  b_axis ＝起点→注入点方位 对世界轴(0/90/180/270)的最小夹角（§四 用过的那个数）
  b_path ＝起点→注入点方位 减 起点→终点弦向（注入点在路径前方多少度）
  b_at→last＝注入点→终点方位 减 起点→终点弦向（注入点处轨迹还朝哪拐）
并打印 seen 读数，用来判"方位踩进盲区"这一说法在**只有起点/终点**的在册数据里能不能站住。
"""
import io
import json
import math
import os

ROOT = "/home/ros1-slam"
DIRS = ["note/进度/SCAN集成/证据/log/raw/scan_p4_retake_newso_20261009_0407",
        "note/进度/SCAN集成/证据/log/raw/scan_p4_sampling_n6_20261009_1037"]


def norm(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def bearing(a, b):
    return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))


def to_axis(b):
    a = abs(b) % 90.0
    return min(a, 90.0 - a)


rows = []
for rel in DIRS:
    for run in sorted(os.listdir(os.path.join(ROOT, rel))):
        f = os.path.join(ROOT, rel, run, "p4.json")
        if not os.path.isfile(f):
            continue
        line = [l for l in io.open(f, encoding="utf-8").read().split("\n") if l.startswith("{")]
        if not line:
            continue
        d = json.loads(line[0])
        inj = d.get("inject") or {}
        tb = d.get("traj_before") or {}
        if not inj.get("at") or not tb.get("first") or not tb.get("last"):
            rows.append((run, inj.get("occ_pts_near"), None, None, None, None))
            continue
        first, last, at = tb["first"], tb["last"], inj["at"]
        ch = bearing(first, last)
        rows.append((run, inj.get("occ_pts_near"), to_axis(bearing(first, at)),
                     norm(bearing(first, at) - ch), norm(bearing(at, last) - ch), ch))

print("%-6s %-6s %-9s %-9s %-11s %-9s" % ("run", "seen", "b_axis", "b_path", "b_at→last", "chord"))
for r in rows:
    if r[2] is None:
        print("%-6s %-6s  缺位姿（链未就绪）" % (r[0], r[1]))
    else:
        print("%-6s %-6s %8.1f° %8.1f° %9.1f° %8.1f°" % (r[0], r[1], r[2], r[3], r[4], r[5]))

GROUPS = (("seen<10（未见/勉强）", lambda s: s < 10),
          ("seen>=10（已见）", lambda s: s >= 10))
for tag, pred in GROUPS:
    sel = [r for r in rows if r[2] is not None and r[1] is not None and pred(r[1])]
    if not sel:
        print("\n[%s] 无样本" % tag)
        continue
    print("\n[%s] n=%d runs=%s" % (tag, len(sel), ",".join(r[0] for r in sel)))
    for k, name in ((2, "b_axis"), (3, "b_path"), (4, "b_at→last")):
        vs = [r[k] for r in sel]
        print("    %s 范围 %.1f°–%.1f°  中位 %.1f°" % (name, min(vs), max(vs), sorted(vs)[len(vs) // 2]))

g0 = [r for r in rows if r[2] is not None and r[1] is not None and r[1] < 10]
g1 = [r for r in rows if r[2] is not None and r[1] is not None and r[1] >= 10]
if g0 and g1:
    for k, name in ((2, "b_axis"), (3, "b_path")):
        a0 = [r[k] for r in g0]
        a1 = [r[k] for r in g1]
        disjoint = max(a0) < min(a1) or max(a1) < min(a0)
        print("\n可分性检查 %s：未见组 [%.1f°, %.1f°] vs 已见组 [%.1f°, %.1f°] ⇒ %s"
              % (name, min(a0), max(a0), min(a1), max(a1),
                 "区间不相交（该代理可分）" if disjoint else "区间相交 ⇒ 该代理**不能**区分未见/已见"))
