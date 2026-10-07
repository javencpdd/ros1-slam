#!/usr/bin/env python3
"""p2_goal_plan.py —— A7 判据②「goal 后 5s 内 FSM 出轨迹、路径无穿墙」取证与判定。

流程：等占据图收敛 → 建占据 KD 树 → 在自由空间选 goal → 实证 goal remap 活体归属
     → 发 goal → 等 scan_planner/Bspline → de Boor 求值曲线 → 逐采样点查最近占据距离。
fail-closed：任一环节异常即记 FAIL，不默认通过。零写入，只订阅/发布 goal 话题。
"""
import json
import subprocess
import sys
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2

from scan_planner.msg import Bspline

OCC_TOL = 0.10      # 硬穿墙阈值（米）：轨迹采样点距最近占据点 <此值 即判穿墙
GOAL_MIN, GOAL_MAX = 2.5, 4.0   # goal 距机范围（占据图滑窗约 9m 见方）
FREE_CLEAR = 0.40   # goal 自身需离最近占据点 ≥ 此值
REACT_S = 5.0       # 判据②：goal 后出轨迹的时间门


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


def wait_occ(timeout=60):
    box = {}

    def cb(m):
        p = occ_points(m)
        if p is not None and len(p) > 500:
            box["pts"] = p
            box["frame"] = m.header.frame_id

    rospy.Subscriber("/grid_map/occupancy", PointCloud2, cb, queue_size=2, buff_size=2 ** 26)
    t0 = time.time()
    while time.time() - t0 < timeout and not rospy.is_shutdown():
        if "pts" in box:
            time.sleep(3.0)          # 再稳一稳，取到的是收敛后的图
            return box
        time.sleep(0.5)
    return box if "pts" in box else None


def de_boor(t, ctrl, knots, order=None):
    """教科书式 de Boor。次数 p 优先由节点向量长度反推（合法 B 样条 len(knots)=n+p+1），
    不信任调用方给的 order，以免对 SCAN 语义的猜测出错。"""
    n = len(ctrl)
    if n == 0 or len(knots) < 3:
        return None
    p = len(knots) - n - 1
    if p < 1:
        p = max(1, (order or 2) - 1)
    if n < p + 1 or len(knots) < n + p + 1:
        return None
    U = knots
    lo, hi = U[p], U[n]
    if hi <= lo or not (lo - 1e-9 <= t <= hi + 1e-9):
        return None
    if t >= hi - 1e-12:
        i = n - 1
    else:
        i = p
        while i < n - 1 and not (U[i] <= t < U[i + 1]):
            i += 1
    d = [np.array(ctrl[i - p + j], dtype=float) for j in range(p + 1)]
    for r in range(1, p + 1):
        for j in range(p, r - 1, -1):
            left, right = U[i - p + j], U[i + 1 + j - r]
            den = right - left
            a = 0.0 if abs(den) < 1e-12 else (t - left) / den
            a = min(max(a, 0.0), 1.0)
            d[j] = (1 - a) * d[j - 1] + a * d[j]
    return d[p]


def eval_curve(bs):
    ctrl = np.array([[q.x, q.y] for q in bs.pos_pts], dtype=float)
    knots = list(bs.knots)
    n = ctrl.shape[0]
    if n == 0 or len(knots) < 3:
        return None
    p = len(knots) - n - 1
    if p < 1:
        p = max(1, int(bs.order) - 1)
    if n < p + 1 or len(knots) < n + p + 1:
        return None
    lo, hi = knots[p], knots[n]
    if hi <= lo:
        return None
    ts = np.linspace(lo, hi, 120)
    pts = [de_boor(t, ctrl, knots, bs.order) for t in ts]
    pts = [q for q in pts if q is not None]
    return np.array(pts) if len(pts) >= 2 else None


def topic_subs(topic):
    try:
        out = subprocess.run(["rostopic", "info", topic], capture_output=True,
                             text=True, timeout=15).stdout
    except Exception:
        return None
    tail = out.split("Subscribers:")
    if len(tail) < 2:
        return []
    seg = tail[1].split("Publishers:")[0]        # Subscribers 段在 Publishers 段之后
    names = []
    for line in seg.splitlines():
        s = line.strip()
        if s.startswith("*"):
            names.append(s.lstrip("* ").split()[0].lstrip("/"))
    return names


def main():
    out = {"checks": {}, "notes": {}}
    chk = out["checks"]

    rospy.init_node("p2_goal_plan", anonymous=True)
    occ = wait_occ(60)
    if not occ:
        chk["①占据图就绪"] = False
        out["verdict"] = "FAIL"
        print(json.dumps(out, ensure_ascii=False)); return
    chk["①占据图就绪"] = True
    P = occ["pts"]
    tree = cKDTree(P)
    out["notes"]["occ_n"] = int(len(P))
    out["notes"]["occ_frame"] = occ.get("frame")

    # 机人位置：取 /Odometry（camera_init 系，与 world 由恒等 TF 对齐）
    try:
        m = rospy.wait_for_message("/Odometry", __import__("nav_msgs.msg", fromlist=["Odometry"]).Odometry, timeout=20)
        robot = np.array([m.pose.pose.position.x, m.pose.pose.position.y])
    except Exception as e:
        chk["②取到机器人位姿"] = False
        out["notes"]["odom_err"] = type(e).__name__
        out["verdict"] = "FAIL"; print(json.dumps(out, ensure_ascii=False)); return
    chk["②取到机器人位姿"] = True
    out["notes"]["robot"] = [round(float(robot[0]), 3), round(float(robot[1]), 3)]

    # 在自由空间选 goal：候选 = 以机器人为心、GOAL_MIN..MAX 半径、36 方位 × 3 半径
    best = None
    for r in np.linspace(GOAL_MIN, GOAL_MAX, 3):
        for a in np.linspace(0, 2 * np.pi, 36, endpoint=False):
            g = robot + np.array([r * np.cos(a), r * np.sin(a)])
            d, _ = tree.query(g)
            if d >= FREE_CLEAR:
                best = (g, float(d), float(r))
                break
        if best:
            break
    if not best:
        chk["③选出自由 goal"] = False
        out["verdict"] = "FAIL"; print(json.dumps(out, ensure_ascii=False)); return
    goal, gclear, gr = best
    chk["③选出自由 goal"] = True
    out["notes"]["goal"] = [round(float(goal[0]), 3), round(float(goal[1]), 3)]
    out["notes"]["goal_dist_to_occ"] = round(gclear, 3)
    out["notes"]["goal_radius"] = round(gr, 2)

    # goal remap 活体归属（§C 勘误项的话题级确认）
    s_priv = topic_subs("/scan_planner/goal")
    s_rviz = topic_subs("/move_base_simple/goal")
    out["notes"]["subs_scan_planner_goal"] = s_priv
    out["notes"]["subs_move_base_simple_goal"] = s_rviz
    if s_priv is None:
        chk["④goal remap 活体归属"] = False
    else:
        has = any("scan_planner_node" in x for x in s_priv)
        notdup = not (s_rviz and any("scan_planner_node" in x for x in s_rviz))
        chk["④goal remap 活体归属"] = bool(has and notdup)

    # 发 goal 并等轨迹
    got = {}

    def bcb(m):
        got.setdefault("t_first", time.time())
        got["msg"] = m

    rospy.Subscriber("/planning/bspline", Bspline, bcb, queue_size=5)
    time.sleep(1.0)
    got.clear()

    pub = rospy.Publisher("/scan_planner/goal", PoseStamped, queue_size=1)
    time.sleep(1.5)
    ps = PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = occ.get("frame") or "world"
    ps.pose.position.x = float(goal[0]); ps.pose.position.y = float(goal[1])
    ps.pose.orientation.w = 1.0
    t_pub = time.time()
    for _ in range(3):
        pub.publish(ps)
        time.sleep(0.4)

    deadline = t_pub + REACT_S
    while time.time() < deadline and "msg" not in got and not rospy.is_shutdown():
        time.sleep(0.1)

    if "msg" not in got:
        chk["⑤5s 内 FSM 出轨迹"] = False
        out["verdict"] = "FAIL"; print(json.dumps(out, ensure_ascii=False)); return
    dt = got["t_first"] - t_pub
    chk["⑤5s 内 FSM 出轨迹"] = bool(dt <= REACT_S)
    out["notes"]["react_s"] = round(dt, 2)

    bs = got["msg"]
    out["notes"]["bspline"] = {"order": int(bs.order), "n_ctrl": len(bs.pos_pts),
                               "n_knots": len(bs.knots), "traj_id": int(bs.traj_id)}
    curve = eval_curve(bs)
    if curve is None or len(curve) < 2:
        chk["⑥曲线可求值"] = False
        out["verdict"] = "FAIL"; print(json.dumps(out, ensure_ascii=False)); return
    chk["⑥曲线可求值"] = True
    d, _ = tree.query(curve)
    out["notes"]["traj_min_dist_m"] = round(float(d.min()), 3)
    out["notes"]["traj_median_dist_m"] = round(float(np.median(d)), 3)
    out["notes"]["n_samples"] = int(len(curve))
    out["notes"]["n_inside_tol"] = int((d < OCC_TOL).sum())
    chk["⑦轨迹无穿墙(>=%.2fm)" % OCC_TOL] = bool(d.min() >= OCC_TOL)

    end = curve[-1]
    out["notes"]["endpoint"] = [round(float(end[0]), 3), round(float(end[1]), 3)]
    out["notes"]["endpoint_err_m"] = round(float(np.linalg.norm(end - goal)), 3)

    out["verdict"] = "PASS" if all(v for v in chk.values()) else "FAIL"
    print(json.dumps(out, ensure_ascii=False))


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        import traceback
        print(json.dumps({"verdict": "FAIL", "checks": {}, "exception": type(e).__name__,
                          "tb": traceback.format_exc().splitlines()[-3:]}, ensure_ascii=False))
