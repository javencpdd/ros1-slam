#!/usr/bin/env python3
"""p4_replan.py —— A7 P4「重规划（丢 1 块动态障碍）」取证与判定（小明 2026-10-08）。

设计原文（note/进度/仿真测试台/设计稿/A_SCAN松耦合集成设计.md §64）：执行序 P3 动态闭环 → **P4 重规划（丢 1 块动态障碍）**。
§65 的验证判据里没有单独编号的「P4 判据」，所以本件不自立编号，结论行写作「P4 重规划结论 = …」，
  判据本体只用在册既有阈值：穿墙阈 OCC_TOL=0.10m（P2 原文）、终点偏差 0.35m 与 dt 120s（A7 判据③④ 原文）、到达实体 J1。

三档分明（坑29／R40 那一族，绝不把「跑不动」报成「跑不过」）：
  INVALID＝扰动没打进世界／扰动未被占据图看见／goal 未被 FSM 消费／首规没出来 ⇒ 前置未成立，不判被试；
            （A24「测量污染」一档曾于 2026-10-11 并入，主理人 2026-10-12 裁撤销 ⇒ 超幅值的 odom 系框失真
             回到 `FAIL|扰动后未到达`，代价与更正面见 04 R220；这类轮次今后**不得**再当行为结论引用）
  FAIL   ＝扰动已被看见，但没有任何新轨迹、或新轨迹穿上新障碍、或最终未到达／超时；
  PASS   ＝障碍被感知 ⇒ 出现新轨迹（轨迹身份可指认地变了）⇒ 新轨迹不撞新障碍 ⇒ 仍以 ≤0.35m 到达。

输出一件（R220 撤销 R135 的分件，主理人 2026-10-12 裁"两者都撤"）：**判据行与 JSON 同走 stdout**；
  机读字段 `grade／invalid／fail／pass` 仍一律由判据行反推（单一真值保留，不两处各写各的）。

世界只改**运行时位姿**（/gazebo/set_model_state 搬 obs_5），m1_world.world 文件一字不动，退出前必还原并自证。
"""
import json
import math
import os
import re
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
from gazebo_msgs.srv import SetModelState, GetModelState

INJECT_MODEL = os.environ.get("P4_MODEL", "obs_5")       # 世界里现成的 0.8³ 箱（runtime 搬动，不改世界件）
INJECT_Z = 0.40
SEE_RADIUS = 0.55                                        # 「扰动已被看见」的取样半径
SEE_MIN_PTS = 20                                         # 看见判据：新增占据点数下限（取证件自用，非被测门值）
GOAL_MIN, GOAL_MAX = 2.5, 4.0
FREE_CLEAR = 0.40
OCC_TOL = 0.10                                           # P2 原文硬穿墙阈，本件不改值
AVOID_DIST = float(os.environ.get("P4_AVOID_DIST", "0.40"))   # 2026-10-11 主理人裁 A5＝定档 0.40（＝箱半宽；语义「轨迹不得进入箱体」，擦边算过）
#   旧默认 0.60（箱半宽 0.40＋裕度 0.20）属施工口径、当时标「待小美定档」；要复算裁前各轮，设 P4_AVOID_DIST=0.60 即可逐字重放
#   注意本件另有两条同值不同义的边界：collide 用 robot_min <= 0.40（本体贴边即算撞，含等号），avoided 用 traj_min >= 0.40（轨迹贴边算过，含等号）
DEV_MAX = 0.35                                           # A7 判据③ 原文
DT_MAX = 120.0                                           # A7 判据④ 原文
# A23 纳入判定（v4.7 稿 Z6，主理人 2026-10-12 裁）：轨迹离**任意占据格**的最近距离低于此阈即判穿墙。
#   门值 0.10m 沿用 P2 原文 OCC_TOL 同值但**不同义**（OCC_TOL 讲的是本体贴边即撞，本条讲的是**轨迹线**离占据格）；
#   单一真身在本行（env `P4_OCC_TOL` 可覆盖，供离线阳性对照"关掉判据"用）。
#   升级条件（Z6 原文）：出现"同一批跨多张不同分辨率图反复扫"再请示升 yaml，届时本行改为读参数、不在代码里写死。
OCC_BREACH_TOL = float(os.environ.get("P4_OCC_TOL", "0.10"))
TWO_COL_FMT = "两列并报 avoid@%.2f=%.2f／breach@%.2f=%.2f"
# #103（R210 已裁返工）：还原面。旧实现两处病＝①`orig` 抄的是**世界件里的静态值**、不是开窗前从 Gazebo 读回来的实际位姿
#   （跨窗时无从发现上一窗没还原干净）；②`restore()` 只看"调用没抛异常"就记 restored=True（**没有回读断言**）。
#   两条门值单一真身在本两行，env 覆盖供离线阳性对照「撤掉断言必须转绿」用。
RESTORE_TOL = float(os.environ.get("P4_RESTORE_TOL", "0.02"))      # 位置回读容差（m）
RESTORE_QTOL = float(os.environ.get("P4_RESTORE_QTOL", "0.001"))   # 姿态四元数分量回读容差（无量纲）

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


def a23_breach(min_occ, tol=None):
    """A23：轨迹离任意占据格最近距离是否 breaches 阈值。返回 (是否 breach, 用的门值)。
    判定链与离线阳性对照**共用本函数**——把同一判断抄两份就是坑62（两处各自维护）。"""
    t = OCC_BREACH_TOL if tol is None else float(tol)
    if min_occ is None:
        return False, t
    return (float(min_occ) < t), t


def a23_selftest():
    """离线合成三段对照（不开 ROS、不起窗）：明知穿墙必须判 breach、关掉判据必须转绿、安全轨迹必须不 breach。"""
    cases = [
        ("例一 合成穿墙（轨迹离占据格 0.02m）", 0.020, None, True),
        ("例二 同数据但门值关到 0（＝撤掉判据）", 0.020, 0.0, False),
        ("例三 安全轨迹（离占据格 0.50m）", 0.500, None, False),
        ("例四 边界：恰等于门值 0.10 不算 breach（严格小于才拦）", 0.100, None, False),
        ("例五 无读数（None）须放行不得瞎判", None, None, False),
    ]
    npass = 0
    for name, dmin, tol, want in cases:
        got, t = a23_breach(dmin, tol)
        ok = (got == want)
        npass += ok
        print("[%s] %s：期望 breach=%s／实得 %s（门值 %.2f）" % ("PASS" if ok else "FAIL", name, want, got, t))
    print("A23 SELFTEST VERDICT: PASS=%d/%d（含「撤掉判据必须转绿」这一反向例——不反向就证明不了判据在起作用）"
          % (npass, len(cases)))
    return 0 if npass == len(cases) else 9


def restore_verdict(before, after, tol=None, qtol=None):
    """#103 的还原断言＝纯函数（判定链与离线自测共用同一份，抄两遍就是坑62）。

    before／after＝(x, y, z, qx, qy, qz, qw) 七元组或 None；返回 dict：
    ok（写回后是否真在原位）／delta_m（位置差模长）／qdelta（四元数逐分量最大差）／tol_m／why。
    缺读数一律 ok=False 并在 why 里写明——「取不到」不等于"还原成功"（坑64 那一族：工具故障不得写成正经结论）。
    """
    t = RESTORE_TOL if tol is None else float(tol)
    q = RESTORE_QTOL if qtol is None else float(qtol)
    if before is None or after is None:
        return {u"ok": False, u"delta_m": None, u"qdelta": None, u"tol_m": round(t, 4),
                u"why": u"缺读数不得判还原（before=%s／after=%s）" % (before is not None, after is not None)}
    dm = math.sqrt(sum((float(before[i]) - float(after[i])) ** 2 for i in range(3)))
    qd = max(abs(float(before[3 + i]) - float(after[3 + i])) for i in range(4))
    return {u"ok": bool(dm <= t and qd <= q), u"delta_m": round(dm, 4), u"qdelta": round(qd, 6),
            u"tol_m": round(t, 4), u"qtol": round(q, 6),
            u"why": u"" if (dm <= t and qd <= q) else u"位置或姿态超出容差"}


def restore_selftest():
    """离线五例（不开 ROS、不起窗）：一致须放行／位移超容差须拦住／撤宽容差必须转绿（反向例）／
       位置同姿态不同须拦住／缺读数不得瞎判。"""
    b = (5.8, -1.0, 0.4, 0.0, 0.0, 0.0, 1.0)
    cases = [
        (u"例一 写回后逐位相同", b, b, None, None, True),
        (u"例二 位移 0.05m＞容差 0.02m 必须拦住", b, (5.85, -1.0, 0.4, 0.0, 0.0, 0.0, 1.0), None, None, False),
        (u"例三 同例二数据但容差撤到 10m（＝关掉断言）必须转绿", b, (5.85, -1.0, 0.4, 0.0, 0.0, 0.0, 1.0), 10.0, None, True),
        (u"例四 位置同、姿态差 90°（qw 1.0→0.707）必须拦住", b, (5.8, -1.0, 0.4, 0.0, 0.0, 0.707, 0.707), None, None, False),
        (u"例五 回读取不到（None）不得判成已还原", b, None, None, None, False),
    ]
    npass = 0
    for name, bef, aft, tol, qtol, want in cases:
        r = restore_verdict(bef, aft, tol, qtol)
        ok = (r[u"ok"] == want)
        npass += ok
        print(u"[%s] %s：期望 ok=%s／实得 %s（delta_m=%s／qdelta=%s／tol_m=%s%s）"
              % (u"PASS" if ok else u"FAIL", name, want, r[u"ok"], r[u"delta_m"], r[u"qdelta"], r[u"tol_m"],
                 u"" if r[u"why"] == u"" else u"／why=" + r[u"why"]))
    print(u"RESTORE SELFTEST VERDICT: PASS=%d/%d（含「撤掉断言必须转绿」反向例——不反向就证明不了回读断言在起作用）"
          % (npass, len(cases)))
    return 0 if npass == len(cases) else 9


def dump(out, verdict):
    """任何出口都把读数打全——21:55 那次 FAIL 只留了一行结论，取证数据（轨迹身份、离障距离分布）全丢了，
       这正是「失败时最难复查」的形态；自此 FAIL／INVALID 与 PASS 同等留件。

       结构性修法保留一条、撤销一条（主理人 2026-10-12 裁"15 号两者都撤"，R220）：
       ①**保留·单一真值**：档位只从判据行解析一次，机读字段一律由它反推——这条属坑62 的修法，
          撤掉就会让 R124／R125 那种 `"invalid": false` 与判据行 `INVALID|…` 反向回来，故不在撤销范围内。
       ②**撤销·两份输出分件**：曾由件写 `$P4_JSON` 指向的文件、判据行只走 stdout；自本批起恢复
          **判据行与 JSON 同走 stdout**（⇒ 收下来的 `p4.json` 首行是中文结论句、整份不是合法 JSON——
          这是主理人明知代价后的裁定，代价见 04 R220 与 v4.7 补丁稿 Z2，不是本件的缺陷）。
       撤销②的同时 harness 侧的同向自证也必须撤（它要比两份东西，现在只剩一份）；单一真值在，
       "两处各自维护"这一型结构上不可能发生 ⇒ 防线没有因为撤自证而消失。
    """
    m = re.match(r"P4 重规划结论 = (PASS|FAIL|INVALID)\|", verdict)
    grade = m.group(1) if m else "INVALID"
    out["verdict"] = verdict
    out["grade"] = grade
    out["invalid"] = (grade == "INVALID")
    out["pass"] = (grade == "PASS")
    out["fail"] = (grade == "FAIL")
    print(verdict, flush=True)
    print(json.dumps(out, ensure_ascii=False), flush=True)
    return grade


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

    # 还原用：**开窗前先从 Gazebo 读回实际位姿**（#103 返工之一、二：旧版把世界件静态值 [5.8,-1.0,0.4] 抄成 orig，
    #   既看不出上一窗没还原干净，也没有回读断言）
    get_svc = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    svc = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    orig = None
    try:
        rsp0 = get_svc(INJECT_MODEL, "world")
        if rsp0.success:
            p0, q0 = rsp0.state.pose.position, rsp0.state.pose.orientation
            orig = (p0.x, p0.y, p0.z, q0.x, q0.y, q0.z, q0.w)
    except Exception as e:
        step("pose_before_err", repr(e)[:80])
    if orig is None:
        dump(out, "P4 重规划结论 = INVALID|开窗前读不到 %s 的当前位姿（/gazebo/get_model_state 未成功）"
                  "⇒ 没有基线可还原，本窗不自证「世界没被我改」" % (INJECT_MODEL,))
        return 9
    out["pose_before"] = [round(float(v), 4) for v in orig]
    step("pose_before", out["pose_before"])
    restored = False
    restore_report = {u"ok": False, u"delta_m": None, u"qdelta": None, u"tol_m": round(RESTORE_TOL, 4),
                      u"why": u"尚未执行还原"}

    def restore():
        """写回**开窗前读到的原七元组**（含姿态），再回读一次交给 restore_verdict 断言。"""
        nonlocal restored, restore_report
        try:
            ms = ModelState()
            ms.model_name = INJECT_MODEL
            ms.reference_frame = "world"
            ms.pose.position.x, ms.pose.position.y, ms.pose.position.z = orig[0], orig[1], orig[2]
            ms.pose.orientation.x, ms.pose.orientation.y = orig[3], orig[4]
            ms.pose.orientation.z, ms.pose.orientation.w = orig[5], orig[6]
            svc(ms)
            rospy.sleep(0.4)
            rsp2 = get_svc(INJECT_MODEL, "world")
            if not rsp2.success:
                restore_report = {u"ok": False, u"delta_m": None, u"qdelta": None, u"tol_m": round(RESTORE_TOL, 4),
                                  u"why": u"回读失败（get_model_state success=False／%s）" % (rsp2.status_message[:40],)}
            else:
                p2, q2 = rsp2.state.pose.position, rsp2.state.pose.orientation
                after = (p2.x, p2.y, p2.z, q2.x, q2.y, q2.z, q2.w)
                out["pose_after"] = [round(float(v), 4) for v in after]
                restore_report = restore_verdict(orig, after)
            restored = bool(restore_report[u"ok"])
            step("restore_delta_m", restore_report[u"delta_m"])
        except Exception as e:
            step("restore_err", repr(e)[:80])
            restore_report = {u"ok": False, u"delta_m": None, u"qdelta": None, u"tol_m": round(RESTORE_TOL, 4),
                              u"why": u"写回或回读抛异常 %s" % (repr(e)[:60],)}

    def rn():
        """还原面读数（每次现取）——凡在 restore() 之后打出的结论行都带这一串，
           否则 INVALID／FAIL 那几条早退路径就成了"世界还没还原干净却没人知道"的洞。"""
        return "还原=restored(%s)｜delta_m=%s／tol_m=%s／qdelta=%s／why=%s" % (
            restored, restore_report["delta_m"], restore_report["tol_m"], restore_report["qdelta"],
            restore_report["why"] if restore_report["why"] else "无")

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
        dump(out, "P4 重规划结论 = INVALID|goal 未被消费（25s 内无 arrival_dist 读数）｜%s" % rn())
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
        dump(out, "P4 重规划结论 = INVALID|首规未出现（goal 已投递但无 /planning/bspline）｜%s" % rn())
        return 9
    id0 = bs_ident(bs0)
    c0 = eval_curve(bs0)
    out["traj_before"] = id0
    step("traj_pts", 0 if c0 is None else int(len(c0)))
    if c0 is None or len(c0) < 5:
        restore()
        dump(out, "P4 重规划结论 = INVALID|首规曲线不可求值｜%s" % rn())
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
        dump(out, "P4 重规划结论 = INVALID|/gazebo/set_model_state 调用失败 %r｜%s" % (repr(e)[:80], rn()))
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
        dump(out, "P4 重规划结论 = INVALID|扰动未被占据图看见（%.2fm 内新增占据点=%d＜%d）｜%s" %
             (SEE_RADIUS, seen, SEE_MIN_PTS, rn()))
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
        dump(out, "P4 重规划结论 = INVALID|新轨迹不可求值｜%s" % rn())
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
    out["restore"] = restore_report
    collide = robot_min <= 0.40
    avoided = traj_min >= AVOID_DIST
    if not st.arrived:
        dump(out, "P4 重规划结论 = FAIL|扰动后未到达（sim %.1fs，dist=%s；物理离箱心最近 %.2fm，轨迹 %.2fm）｜%s" %
             (dts if dts is not None else -1, out["arrive"]["dist_end"], robot_min, traj_min, rn()))
        return 1
    if collide:
        dump(out, "P4 重规划结论 = FAIL|机器人撞进注入障碍（物理离箱心 %.2fm ≤ 箱半宽 0.40m；轨迹离箱心 %.2fm）⇒ 未有效避让｜%s" %
             (robot_min, traj_min, rn()))
        return 1
    if not avoided:
        dump(out, "P4 重规划结论 = FAIL|到达但未观测到避让（轨迹离箱心最近 %.2fm ＜ %.2fm；物理最近 %.2fm）⇒ 判据分辨力缺口一并上报｜%s" %
             (traj_min, AVOID_DIST, robot_min, rn()))
        return 1
    breach, btol = a23_breach(dmin)
    out["a23"] = {"min_dist_to_any_occ": round(dmin, 3), "breach_gate": btol, "breach": bool(breach),
                  "avoid_gate": AVOID_DIST, "avoid": bool(avoided)}
    if breach:
        dump(out, "P4 重规划结论 = FAIL|轨迹离任意占据格最近 %.3fm ＜ %.2fm（A23 纳入判定，Z6）⇒ 判穿墙；%s｜%s" %
             (dmin, btol, TWO_COL_FMT % (AVOID_DIST, traj_min, btol, dmin), rn()))
        return 1
    if st.dist > DEV_MAX or (dts is not None and dts > DT_MAX):
        dump(out, "P4 重规划结论 = FAIL|到达但超门（偏差 %.3fm／dt_sim %.1fs）｜%s" % (st.dist, dts or -1, rn()))
        return 1
    if not restored:
        dump(out, "P4 重规划结论 = INVALID|避让与到达全部成立但**世界没还原干净**（%s）⇒ 本窗读数不可引用；"
                  "这条门自 #103 起才真落在件里（v_scan_p4.sh 头注早已诺「还原不成就把本面判 INVALID」，此前无任何代码执行它）" % (rn(),))
        return 9
    dump(out, "P4 重规划结论 = PASS|障碍被看见(%d 点)⇒扰动后轨迹离箱心最近 %.2fm≥%.2fm（避让在位）⇒未撞（物理最近 %.2fm）⇒到达偏差 %.3fm≤%.2fm｜%s｜%s" %
         (seen, traj_min, AVOID_DIST, robot_min, st.dist, DEV_MAX,
         TWO_COL_FMT % (AVOID_DIST, traj_min, btol, dmin), rn()))
    return 0


if __name__ == "__main__":
    if "--a23-selftest" in sys.argv:
        sys.exit(a23_selftest())
    if "--restore-selftest" in sys.argv:
        sys.exit(restore_selftest())
    sys.exit(main())
