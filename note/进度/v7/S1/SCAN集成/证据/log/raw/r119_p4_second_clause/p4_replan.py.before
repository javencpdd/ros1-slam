#!/usr/bin/env python3
"""p4_replan.py —— A7 P4「重规划（丢 1 块动态障碍）」取证与判定（小明 2026-10-08）。

设计原文（note/进度/仿真测试台/设计稿/A_SCAN松耦合集成设计.md §64）：执行序 P3 动态闭环 → **P4 重规划（丢 1 块动态障碍）**。
§65 的验证判据里没有单独编号的「P4 判据」，所以本件不自立编号，结论行写作「P4 重规划结论 = …」，
  判据本体只用在册既有阈值：穿墙阈 OCC_TOL=0.10m（P2 原文）、终点偏差 0.35m 与 dt 120s（A7 判据③④ 原文）、到达实体 J1。

三档分明（坑29／R40 那一族，绝不把「跑不动」报成「跑不过」）：
  INVALID＝扰动没打进世界／扰动未被占据图看见／goal 未被 FSM 消费／首规没出来 ⇒ 前置未成立，不判被试；
  FAIL   ＝扰动已被看见，但没有任何新轨迹、或新轨迹穿上新障碍、或最终未到达／超时；
  PASS   ＝障碍被感知 ⇒ 出现新轨迹（轨迹身份可指认地变了）⇒ 新轨迹不撞新障碍 ⇒ 仍以 ≤0.35m 到达。

世界只改**运行时位姿**（/gazebo/set_model_state 搬 obs_5），m1_world.world 文件一字不动，退出前必还原并自证。
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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from p2_goal_plan import eval_curve, occ_points          # 复用现役 P2 件里的曲线求值与点云解析（不复制实现）
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

INJECT_MODEL = os.environ.get("P4_MODEL", "obs_5")       # 世界里现成的 0.8³ 箱（runtime 搬动，不改世界件）
INJECT_Z = 0.40
SEE_RADIUS = 0.55                                        # 「扰动已被看见」的取样半径
SEE_MIN_PTS = 20                                         # 看见判据：新增占据点数下限（取证件自用，非被测门值）
GOAL_MIN, GOAL_MAX = 2.5, 4.0
FREE_CLEAR = 0.40
OCC_TOL = 0.10                                           # P2 原文硬穿墙阈，本件不改值
AVOID_DIST = float(os.environ.get("P4_AVOID_DIST", "0.60"))   # 「算绕开」的距离＝箱半宽 0.40＋裕度 0.20（施工口径，待小美定档）
DEV_MAX = 0.35                                           # A7 判据③ 原文
DT_MAX = 120.0                                           # A7 判据④ 原文
REACT_S = float(os.environ.get("P4_REACT_MAX", "15.0"))  # 看见障碍后允许的重规响应窗（施工口径，可改）


class St(object):
    def __init__(self):
        self.clock = None
        self.odom = None
        self.arrived = False
        self.dist = None
        self.bs = None
        self.occ = None
        self.cmd_n = 0

    def on_clock(self, m):
        self.clock = m.clock.secs + m.clock.nsecs * 1e-9

    def on_odom(self, m):
        p = m.pose.pose.position
        self.odom = (p.x, p.y)

    def on_arrived(self, m):
        self.arrived = bool(m.data)

    def on_dist(self, m):
        self.dist = float(m.data)

    def on_bs(self, m):
        self.bs = m

    def on_occ(self, m):
        p = occ_points(m)
        if p is not None and len(p) > 200:
            self.occ = p

    def on_cmd(self, m):
        self.cmd_n += 1


def bs_ident(bs):
    """轨迹身份：start_time＋段数＋控制点首末（可指认地比对「是不是换了一条」）。"""
    if bs is None:
        return None
    st = bs.start_time.secs + bs.start_time.nsecs * 1e-9
    pts = np.array([[q.x, q.y] for q in bs.pos_pts], dtype=float)
    return {"start_time": round(st, 3), "n_ctrl": int(len(pts)),
            "first": [round(pts[0][0], 3), round(pts[0][1], 3)] if len(pts) else None,
            "last": [round(pts[-1][0], 3), round(pts[-1][1], 3)] if len(pts) else None}


def los_free(tree, a, b, step=0.5, clearance=FREE_CLEAR):
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    n = max(2, int(np.linalg.norm(b - a) / step) + 1)
    for t in np.linspace(0.0, 1.0, n):
        if tree.query(a + (b - a) * t)[0] < clearance:
            return False
    return True


def pick_goal(tree, odom, idx=1):
    rng = np.random.default_rng(int(os.environ.get("P4_SEED", "11")) * 1000 + idx)
    for _ in range(400):
        th = rng.uniform(-math.pi, math.pi)
        r = rng.uniform(GOAL_MIN, GOAL_MAX)
        gx, gy = odom[0] + r * math.cos(th), odom[1] + r * math.sin(th)
        d = tree.query([gx, gy])[0]
        if d >= FREE_CLEAR and los_free(tree, odom, (gx, gy)):
            return gx, gy, r, float(d)
    return None


def dump(out, verdict):
    """任何出口都把读数打全——21:55 那次 FAIL 只留了一行结论，取证数据（轨迹身份、离障距离分布）全丢了，
       这正是「失败时最难复查」的形态；自此 FAIL／INVALID 与 PASS 同等留件。"""
    print(verdict, flush=True)
    print(json.dumps(out, ensure_ascii=False), flush=True)


def main():
    out = {"steps": [], "fail": [], "invalid": False}
    rospy.init_node("p4_replan", anonymous=True)
    st = St()
    from rosgraph_msgs.msg import Clock
    rospy.Subscriber("/clock", Clock, st.on_clock, queue_size=5, buff_size=2 ** 22)
    rospy.Subscriber("/LIO/odom_vehicle", Odometry, st.on_odom, queue_size=5, buff_size=2 ** 22)
    rospy.Subscriber("/lite3/scan/arrived", Bool, st.on_arrived, queue_size=5)
    rospy.Subscriber("/lite3/scan/arrival_dist", Float32, st.on_dist, queue_size=5)
    from scan_planner.msg import Bspline
    rospy.Subscriber("/planning/bspline", Bspline, st.on_bs, queue_size=5, buff_size=2 ** 24)
    rospy.Subscriber("/grid_map/occupancy", PointCloud2, st.on_occ, queue_size=2, buff_size=2 ** 26)
    rospy.Subscriber("/scan_planner/cmd_vel", rospy.AnyMsg, st.on_cmd, queue_size=5, buff_size=2 ** 20)

    def step(k, v):
        out["steps"].append("%s=%s" % (k, v))

    t0 = time.time()
    while time.time() - t0 < 120 and (st.occ is None or st.odom is None or st.clock is None):
        rospy.sleep(0.3)
    if st.occ is None or st.odom is None or st.clock is None:
        dump(out, "P4 重规划结论 = INVALID|链未就绪（占据图=%s odom=%s clock=%s）" %
             (st.occ is not None, st.odom is not None, st.clock is not None))
        return 9

    tree = cKDTree(st.occ)
    g = pick_goal(tree, st.odom)
    if g is None:
        dump(out, "P4 重规划结论 = INVALID|找不到沿途自由的 goal（滑窗未知区过大）")
        return 9
    gx, gy, gr, gclear = g
    out.update(goal=[round(gx, 3), round(gy, 3)], goal_r=round(gr, 3), goal_clear=round(gclear, 3))

    # 还原用：读当前 obs 位姿（从世界件里取的静态值不必再查，直接用在册 pose）
    orig = [5.8, -1.0, 0.4]
    svc = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    restored = False

    def restore():
        nonlocal restored
        try:
            ms = ModelState()
            ms.model_name = INJECT_MODEL
            ms.reference_frame = "world"
            ms.pose.position.x, ms.pose.position.y, ms.pose.position.z = orig
            ms.pose.orientation.w = 1.0
            svc(ms)
            restored = True
        except Exception as e:
            step("restore_err", repr(e)[:80])

    pub = rospy.Publisher("/scan_planner/goal", PoseStamped, queue_size=1)
    time.sleep(1.5)
    m = PoseStamped()
    m.header.frame_id = "world"
    m.header.stamp = rospy.Time.now()
    m.pose.position.x, m.pose.position.y = gx, gy
    m.pose.orientation.w = 1.0
    t_send = time.time()
    for _ in range(3):
        pub.publish(m)
        rospy.sleep(0.4)
    while time.time() - t_send < 25.0 and st.dist is None:      # 投递阳性对照（P3 同款）
        rospy.sleep(0.2)
    if st.dist is None:
        restore()
        dump(out, "P4 重规划结论 = INVALID|goal 未被消费（25s 内无 arrival_dist 读数）")
        return 9
    step("goal_delivered", True)

    # 首规出来
    bs0 = st.bs
    t_r = time.time()
    while bs0 is None and time.time() - t_r < 20.0:
        rospy.sleep(0.2)
        bs0 = st.bs
    if bs0 is None:
        restore()
        dump(out, "P4 重规划结论 = INVALID|首规未出现（goal 已投递但无 /planning/bspline）")
        return 9
    id0 = bs_ident(bs0)
    c0 = eval_curve(bs0)
    out["traj_before"] = id0
    step("traj_pts", 0 if c0 is None else int(len(c0)))
    if c0 is None or len(c0) < 5:
        restore()
        dump(out, "P4 重规划结论 = INVALID|首规曲线不可求值")
        return 9

    # 把障碍搬到轨迹中段前方（机器人还没走到的地方）
    ip = c0[int(len(c0) * 0.40)]
    ix, iy = float(ip[0]), float(ip[1])
    n_before = int(len(st.occ))          # 注入前的占据图点数，供「看见」面读数对照
    ms = ModelState()
    ms.model_name = INJECT_MODEL
    ms.reference_frame = "world"
    ms.pose.position.x, ms.pose.position.y, ms.pose.position.z = ix, iy, INJECT_Z
    ms.pose.orientation.w = 1.0
    try:
        svc(ms)
    except Exception as e:
        restore()
        dump(out, "P4 重规划结论 = INVALID|/gazebo/set_model_state 调用失败 %r" % (repr(e)[:80],))
        return 9
    out["inject"] = {"model": INJECT_MODEL, "at": [round(ix, 3), round(iy, 3)], "z": INJECT_Z,
                     "pts_before": n_before}
    step("injected", "%s→(%.2f,%.2f)" % (INJECT_MODEL, ix, iy))

    # 扰动可见性阳性对照：占据图必须在 SEE_RADIUS 内多出点
    t_i = time.time()
    seen = 0
    while time.time() - t_i < 30.0:
        if st.occ is not None:
            d = np.linalg.norm(st.occ - np.array([ix, iy]), axis=1)
            seen = int(np.sum(d < SEE_RADIUS))
            if seen >= SEE_MIN_PTS:
                break
        rospy.sleep(0.3)
    out["inject"]["occ_pts_near"] = seen
    if seen < SEE_MIN_PTS:
        restore()
        dump(out, "P4 重规划结论 = INVALID|扰动未被占据图看见（%.2fm 内新增占据点=%d＜%d）" %
             (SEE_RADIUS, seen, SEE_MIN_PTS))
        return 9
    step("disturbance_seen", seen)

    # 等新轨迹（身份变了）
    base = id0["start_time"]
    new_id = None
    t_w = time.time()
    while time.time() - t_w < REACT_S + 25.0:
        cur = bs_ident(st.bs)
        if cur and (abs(cur["start_time"] - base) > 0.05 or cur["last"] != id0["last"]):
            new_id = cur
            break
        rospy.sleep(0.2)
    out["traj_after"] = new_id
    if new_id is None:
        restore()
        dump(out, "P4 重规划结论 = FAIL|障碍已被看见（注入处附近占据点 %d）但 %ss 内无新轨迹身份 ⇒ 未重规划" %
             (seen, int(REACT_S + 25)))
        return 1
    step("replanned", True)

    # 重规中途会连出多条（半程曲线／未收敛的过渡轨迹），拿瞬间快照定罪会冤枉被测件 ⇒
    #   先取「身份稳定 ≥3s 的那条」再判（同一读数口径也回头给首规算一遍，作对照，好让 0.055m 这种值可比）
    def stable_bs(timeout=30.0):
        last = bs_ident(st.bs)
        t_s = time.time()
        while time.time() - t_s < timeout:
            rospy.sleep(0.5)
            cur = bs_ident(st.bs)
            if cur and last and cur["start_time"] == last["start_time"] and cur["n_ctrl"] == last["n_ctrl"]:
                if time.time() - t_s > 3.0:
                    return st.bs, cur
            last = cur
        return st.bs, bs_ident(st.bs)

    bsN, new_id = stable_bs()
    out["traj_after"] = new_id
    cn = eval_curve(bsN)
    b0 = eval_curve(bs0)
    if b0 is not None and st.occ is not None:
        t0tree = cKDTree(st.occ)
        out["traj_before_control_min_dist"] = round(float(min(t0tree.query(q)[0] for q in b0)), 3)
    if cn is None or len(cn) < 5:
        restore()
        dump(out, "P4 重规划结论 = INVALID|新轨迹不可求值")
        return 9
    tree2 = cKDTree(st.occ)
    dmin = float(min(tree2.query(p)[0] for p in cn))
    dout_obs = float(np.linalg.norm(np.array([ix, iy]) - cn, axis=1).min())
    out["new_traj"] = {"n_pts": int(len(cn)), "min_dist_to_any_occ": round(dmin, 3),
                       "min_dist_to_injected": round(dout_obs, 3)}
    out["clearance"] = {"min_dist_to_any_occ": round(dmin, 3), "min_dist_to_injected": round(dout_obs, 3),
                        "gate_occ": OCC_TOL}
    # 不在这里 return：21:55 与 21:59 两轮把「离占据最近 0.458m（在册阈 0.10m 判不穿墙）」与「离注入箱心 0.031m
    #   （曲线从箱体中心穿过）」同时成立摆了出来——这是**判据分辨力缺口**，不能由我直接定罪。
    #   两条都记入件，继续跑完取物理事实（机器人有没有真撞进去、后面有没有换绕行轨迹）。

    # 收敛：仍要到达（J1 实体）且在 0.35m／120s 门内
    t_c = time.time()
    dts0 = st.clock
    robot_min = 99.0
    traj_min = dout_obs
    while time.time() - t_c < 300.0:
        if st.odom is not None:
            robot_min = min(robot_min, float(np.linalg.norm(np.array(st.odom) - np.array([ix, iy]))))
        cur = bs_ident(st.bs)
        if cur and new_id and abs(cur["start_time"] - new_id["start_time"]) > 0.05:
            new_id = cur
            cx = eval_curve(st.bs)
            if cx is not None and len(cx) > 4:
                traj_min = float(np.linalg.norm(np.array([ix, iy]) - cx, axis=1).min())
        if st.arrived and st.dist is not None:
            break
        if st.clock is not None and dts0 is not None and (st.clock - dts0) > DT_MAX + 15.0:
            break
        rospy.sleep(0.2)
    dts = (st.clock - dts0) if (st.clock is not None and dts0 is not None) else None
    out["physical"] = {"robot_min_to_injected": round(robot_min, 3), "box_half_width": 0.40,
                      "traj_min_to_injected_final": round(traj_min, 3)}
    out["arrive"] = {"arrived": bool(st.arrived), "dist_end": round(st.dist, 3) if st.dist is not None else None,
                     "dt_sim_since_replan": round(dts, 2) if dts is not None else None,
                     "cmd_count": int(st.cmd_n), "displacement": round(math.hypot(st.odom[0] - gx, st.odom[1] - gy), 3)}
    restore()
    out["restored"] = restored
    collide = robot_min <= 0.40
    avoided = traj_min >= AVOID_DIST
    if not st.arrived:
        dump(out, "P4 重规划结论 = FAIL|扰动后未到达（sim %.1fs，dist=%s；物理离箱心最近 %.2fm，轨迹 %.2fm）" %
             (dts if dts is not None else -1, out["arrive"]["dist_end"], robot_min, traj_min))
        return 1
    if collide:
        dump(out, "P4 重规划结论 = FAIL|机器人撞进注入障碍（物理离箱心 %.2fm ≤ 箱半宽 0.40m；轨迹离箱心 %.2fm）⇒ 未有效避让" %
             (robot_min, traj_min))
        return 1
    if not avoided:
        dump(out, "P4 重规划结论 = FAIL|到达但未观测到避让（轨迹离箱心最近 %.2fm ＜ %.2fm；物理最近 %.2fm）⇒ 判据分辨力缺口一并上报" %
             (traj_min, AVOID_DIST, robot_min))
        return 1
    if st.dist > DEV_MAX or (dts is not None and dts > DT_MAX):
        dump(out, "P4 重规划结论 = FAIL|到达但超门（偏差 %.3fm／dt_sim %.1fs）" % (st.dist, dts or -1))
        return 1
    dump(out, "P4 重规划结论 = PASS|障碍被看见(%d 点)⇒扰动后轨迹离箱心最近 %.2fm≥%.2fm（避让在位）⇒未撞（物理最近 %.2fm）⇒到达偏差 %.3fm≤%.2fm｜还原=%s" %
         (seen, traj_min, AVOID_DIST, robot_min, st.dist, DEV_MAX, restored))
    return 0


if __name__ == "__main__":
    sys.exit(main())
