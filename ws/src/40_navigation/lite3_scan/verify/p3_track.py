#!/usr/bin/env python3
"""p3_track.py —— A7 判据③「动态闭环跟踪走完」取证与判定（小明 2026-10-08，随 roadmap C #3 的 P3 波）。

判据原文（note/进度/仿真测试台/设计稿/A_SCAN松耦合集成设计.md §65）：
  ③ 到达判据 FSM reached，终点 odom 偏差 ≤0.35m（对齐 teb 基线门）
  ④ 全程 dt≤120s
  ⑤ 3 连全过（沿用「三连任一未过即停」红线）
本件把「FSM reached」钉在小张为 O-1 补的到达实体上（scripts/scan_arrival_monitor.py，/lite3/scan/arrived
latch + /lite3/scan/arrival_dist），因为盘上 SCAN 对外无到达信号（FSM 只看轨迹时间耗尽、controller 的
finish_dist 只把 cmd 置零）——拿 bspline 末点当到达必假绿，那是设计件事实 A 明令禁止的。

口径选择（都在读数里打出来，供验收改判，不藏）：
  · dt 用**仿真时间**（/clock），与 M2 在册 dt 口径同族；墙钟一并报，两者不同源时以 sim 为准。
  · 「到没到」只看到达实体的 latch，不看 FSM 内部状态（不消费即不引用）。
  · 0.35m 门值与 120s 门值照原文取，**本件不自设新门**；位移/路径长度只作读数不作判据。
fail-closed：任一环节异常即记 FAIL 并给原因；跑不动与跑不过分开打（坑29／R40 一族），跑不动 rc=9。
零写入：只发 goal、只订阅。
"""
import json
import math
import os
import sys
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32
from scipy.spatial import cKDTree

DEV_MAX = float(os.environ.get("P3_DEV_MAX", "0.35"))     # 判据③ 终点 odom 偏差门（原文值）
DT_MAX = float(os.environ.get("P3_DT_MAX", "120.0"))      # 判据④ 全程 dt 门（原文值，仿真时间）
RUNS = int(os.environ.get("P3_RUNS", "3"))                # 判据⑤ 三连
GOAL_MIN, GOAL_MAX = 2.5, 4.0                             # 与 P2 同族：占据滑窗约 9m 见方
FREE_CLEAR = 0.40
OCC_MIN = 500


def axes(msg, name):
    for f in msg.fields:
        if f.name != name or f.datatype != 7:
            continue
        stride = msg.point_step // 4
        start = f.offset // 4
        n = msg.width * msg.height
        need = (n - 1) * stride + start + 1
        if n <= 0 or start >= stride or need > len(msg.data) // 4:
            return None
        v = np.frombuffer(msg.data, dtype=np.float32, count=need)
        return np.asarray(v[start:start + n * stride:stride], dtype=np.float64)
    return None


def occ_points(msg):
    x, y = axes(msg, "x"), axes(msg, "y")
    if x is None or y is None or len(x) == 0:
        return None
    m = np.isfinite(x) & np.isfinite(y)
    return np.column_stack([x[m], y[m]])


class Sim(object):
    """读数容器：/clock、odom、到达实体、cmd 流量都只在这里采样一次。"""

    def __init__(self):
        self.clock = None
        self.odom = None
        self.odom_t = None
        self.arrived = False
        self.dist = None
        self.cmd_n = 0
        self.pts = []

    def on_clock(self, m):
        self.clock = m.secs + m.nsecs * 1e-9

    def on_odom(self, m):
        p = m.pose.pose.position
        self.odom = (p.x, p.y)
        self.odom_t = time.time()

    def on_arrived(self, m):
        self.arrived = bool(m.data)

    def on_dist(self, m):
        self.dist = float(m.data)

    def on_cmd(self, m):
        self.cmd_n += 1


def pick_goal(tree, occ, odom, idx=1):
    # 逐跑换 seed：同一 seed 三连会挑到同一个 goal（"三连"退化成同任务重放三次），
    #   这里让每跑必不同（可复现：seed=P3_SEED*1000+跑序），要同 goal 复现由调用方显式改回。
    rng = np.random.default_rng(int(os.environ.get("P3_SEED", "7")) * 1000 + idx)
    for _ in range(400):
        th = rng.uniform(-math.pi, math.pi)
        r = rng.uniform(GOAL_MIN, GOAL_MAX)
        gx, gy = odom[0] + r * math.cos(th), odom[1] + r * math.sin(th)
        d, _i = tree.query([gx, gy])
        if d >= FREE_CLEAR:
            return gx, gy, r, float(d)
    return None


def run_cycle(sim, occ_msg, idx):
    c = {"run": idx, "fail": []}
    pts = occ_points(occ_msg)
    if pts is None or len(pts) < OCC_MIN:
        c["fail"].append("占据图点数不足（前置未成立，非判定失败）")
        c["invalid"] = True
        return c
    odom0 = sim.odom
    if odom0 is None:
        c["fail"].append("odom 未就绪（前置未成立）")
        c["invalid"] = True
        return c
    g = pick_goal(cKDTree(pts), pts, odom0, idx)
    if g is None:
        c["fail"].append("400 次采样内找不到自由 goal（前置未成立）")
        c["invalid"] = True
        return c
    gx, gy, gr, gclear = g
    pub = rospy.Publisher("/scan_planner/goal", PoseStamped, queue_size=1)
    m = PoseStamped()
    m.header.frame_id = occ_msg.header.frame_id or "world"   # 与 P2 同口径：goal 帧取占据图 frame_id（实测 world），不硬写
    m.header.stamp = rospy.Time.now()
    m.pose.position.x, m.pose.position.y = gx, gy
    m.pose.orientation.w = 1.0
    sim.arrived = False
    sim.dist = None
    sim.cmd_n = 0
    t0w = time.time()
    c0 = sim.clock
    pub.publish(m)
    rospy.loginfo("P3 run%d goal=(%.2f,%.2f) r=%.2f clear=%.2f", idx, gx, gy, gr, gclear)
    # 超时**只按仿真时间判**（判据④ 的 dt 是 sim 口径）；墙钟只作不可动运动的兜底。
    #   首版把墙钟兜底写成 DT_MAX+60 秒＝180s，而本平台 RTF≈0.28 ⇒ 120s sim 要 ~430s 墙钟，
    #   等于「判据还没到门就被判死」——低 RTF 平台下的第三型假红（坑32／O-5 同族），故分开两级。
    SIM_WAIT = DT_MAX + 15.0
    WALL_CAP = float(os.environ.get("P3_WALL_CAP", "900"))
    seen_cmd = False
    sim_over = False
    t_lim_w = t0w + WALL_CAP
    while time.time() < t_lim_w:
        ds = (sim.clock - c0) if (sim.clock is not None and c0 is not None) else 0.0
        if ds > 6.0 and sim.cmd_n > 0:
            seen_cmd = True
        if sim.arrived and sim.dist is not None:
            break
        if ds > SIM_WAIT:
            sim_over = True
            break
        rospy.sleep(0.2)
    else:
        c["fail"].append("墙钟兜底 %.0fs 到而 sim 时基未见推进（RTF 塌陷，属算力面非判据面）" % WALL_CAP)
        c["invalid"] = True
    c1 = sim.clock
    dtw = time.time() - t0w
    dts = (c1 - c0) if (c0 is not None and c1 is not None) else None
    c.update(goal=[round(gx, 3), round(gy, 3)], goal_r=round(gr, 3), goal_clear=round(gclear, 3),
             odom0=[round(odom0[0], 3), round(odom0[1], 3)],
             odom1=[round(sim.odom[0], 3), round(sim.odom[1], 3)] if sim.odom else None,
             dist_end=round(sim.dist, 3) if sim.dist is not None else None,
             arrived=bool(sim.arrived), cmd_pub_count=int(sim.cmd_n), cmd_seen_after_6s=seen_cmd,
             dt_sim=round(dts, 2) if dts is not None else None, dt_wall=round(dtw, 2))
    if sim.odom and dts is not None and sim.dist is not None:
        disp = math.hypot(sim.odom[0] - odom0[0], sim.odom[1] - odom0[1])
        c["displacement"] = round(disp, 3)          # 只作读数，不设门
    # ---- 判据本体 ----
    if sim_over:
        c["fail"].append("判据④ 在 sim %.1fs 内未到达（门 %.0fs，超时即判，不再等墙钟）" % ((sim.clock - c0) if (sim.clock and c0) else -1, DT_MAX))
    if not sim.arrived:
        c["fail"].append("判据③ 到达实体未置位（arrived=False）")
    else:
        if sim.dist is None:
            c["fail"].append("判据③ 到达但距离不可指认")
        elif sim.dist > DEV_MAX:
            c["fail"].append("判据③ 终点 odom 偏差 %.3f > %.2fm" % (sim.dist, DEV_MAX))
    if dts is None:
        c["fail"].append("判据④ dt 不可指认（/clock 未就绪）")
    elif dts > DT_MAX:
        c["fail"].append("判据④ 全程 dt=%.1fs（sim）> %.0fs" % (dts, DT_MAX))
    if not seen_cmd:
        c["fail"].append("闭环未闭：goal 后 6s 起 /scan_planner/cmd_vel 计数=%d（跟踪层没在发命令）" % sim.cmd_n)
    return c


def main():
    rospy.init_node("p3_track", anonymous=True)
    sim = Sim()
    from rosgraph_msgs.msg import Clock
    rospy.Subscriber("/clock", Clock, sim.on_clock, queue_size=5, buff_size=2 ** 22)
    rospy.Subscriber("/LIO/odom_vehicle", Odometry, sim.on_odom, queue_size=5, buff_size=2 ** 22)
    rospy.Subscriber("/lite3/scan/arrived", Bool, sim.on_arrived, queue_size=5)
    rospy.Subscriber("/lite3/scan/arrival_dist", Float32, sim.on_dist, queue_size=5)
    rospy.Subscriber("/scan_planner/cmd_vel", rospy.AnyMsg, sim.on_cmd, queue_size=5, buff_size=2 ** 20)

    occ = {}

    def occ_cb(m):
        occ["msg"] = m
    rospy.Subscriber("/grid_map/occupancy", PointCloud2, occ_cb, queue_size=2, buff_size=2 ** 26)

    t0 = time.time()
    while time.time() - t0 < 90 and ("msg" not in occ or sim.odom is None or sim.clock is None):
        rospy.sleep(0.3)
    if "msg" not in occ or sim.odom is None or sim.clock is None:
        print("P3 判据③ 结论 = INVALID|前置未成立（占据图=%s odom=%s clock=%s）" %
              ("msg" in occ, sim.odom is not None, sim.clock is not None))
        return 9

    rows = []
    for i in range(1, RUNS + 1):
        r = run_cycle(sim, occ["msg"], i)
        rows.append(r)
        print(json.dumps(r, ensure_ascii=False))
        if r.get("invalid"):
            print("P3 判据③ 结论 = INVALID|%s（第 %d 跑前置未成立，不判被试、不占连击）" % (";".join(r["fail"]), i))
            return 9
        if r["fail"]:
            print("P3 判据③ 结论 = FAIL|%s（第 %d 跑，红线＝三连任一未过即停）" % (";".join(r["fail"]), i))
            return 1
    print("P3 判据③ 结论 = PASS|全部 %d 跑到达实体置位、终点偏差≤%.2fm、dt(sim)≤%.0fs" % (RUNS, DEV_MAX, DT_MAX))
    return 0


if __name__ == "__main__":
    sys.exit(main())
