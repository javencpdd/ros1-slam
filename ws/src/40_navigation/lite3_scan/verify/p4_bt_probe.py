#!/usr/bin/env python3
"""p4_bt_probe.py — SCAN P4 dynamic replan through the v8 BT orchestration.

Chain (started by v_scan_bt.sh with SCAN_BT_PROBE=this file):
  m2_gazebo → m2_mapping → 60_safety → 70_scan → scan_bt_nav(BT + bridge)

Unlike p4_replan.py (goal sent straight to /scan_planner/goal), this probe sends
the goal to the BT's /v8/bt/goal and lets RequestPath forward it to SCAN; arrival
is the BT's outcome=reached (fed by SCAN's /lite3/scan/arrived through the
bridge).  Everything else follows the in-force P4 gates: injected obs_5, seen by
the occupancy, new B-spline identity, avoid ≥0.40 m, physical >0.40 m, A23 3D
clearance ≥0.10 m, endpoint ≤0.35 m, dt ≤120 s.
Exit: 0 PASS, 1 FAIL, 9 INVALID.
"""
import json
import math
import os
import sys
import time

import numpy as np
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from scipy.spatial import cKDTree
from std_msgs.msg import Bool, Float32, String
from scan_planner.msg import Bspline

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from p2_goal_plan import axes, eval_curve, occ_points
from p4_replan import (AVOID_DIST, DEV_MAX, DT_MAX, INJECT_MODEL, INJECT_Z,
                       a23_breach, bs_ident, pick_goal, restore_verdict)

SEE_RADIUS = 0.55
SEE_MIN_PTS = 20
INJECT_FRAC = float(os.environ.get("P4_INJECT_FRAC", "0.60"))
# 斜坡注入时箱心要抬到坡面之上；平地保持 0.40。调用方按坡面高度覆盖。
INJECT_Z = float(os.environ.get("P4_INJECT_Z", str(INJECT_Z)))


class State(object):
    def __init__(self):
        self.bt = None
        self.bt_at = None
        self.path_status = None
        self.bs = None
        self.occ = None
        self.occ_z = None
        self.odom = None
        self.odom_z = None
        self.odom_yaw = None
        self.arrived = None
        self.dist = None
        self.cmd_n = 0

    def on_bt(self, m):
        try:
            self.bt = json.loads(m.data)
            self.bt_at = time.monotonic()
        except ValueError:
            pass

    def on_path_status(self, m):
        try:
            self.path_status = json.loads(m.data)
        except ValueError:
            pass

    def on_bs(self, m):
        self.bs = m

    def on_occ(self, m):
        x, y, z = axes(m, "x"), axes(m, "y"), axes(m, "z")
        if x is None or y is None or z is None:
            return
        mask = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        if mask.sum() > 200:
            self.occ = np.column_stack([x[mask], y[mask]])
            self.occ_z = z[mask]

    def on_odom(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        self.odom = (p.x, p.y)
        self.odom_z = p.z
        self.odom_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_arrived(self, m):
        self.arrived = bool(m.data)

    def on_dist(self, m):
        self.dist = float(m.data)

    def on_cmd(self, _m):
        self.cmd_n += 1


def main():
    rospy.init_node("p4_bt_probe", anonymous=True)
    st = State()
    rospy.Subscriber("/v8/bt/status", String, st.on_bt, queue_size=5)
    rospy.Subscriber("/global_path/status", String, st.on_path_status, queue_size=5)
    rospy.Subscriber("/planning/bspline", Bspline, st.on_bs, queue_size=5, buff_size=2 ** 24)
    rospy.Subscriber("/grid_map/occupancy", PointCloud2, st.on_occ, queue_size=2, buff_size=2 ** 26)
    rospy.Subscriber("/LIO/odom_vehicle", Odometry, st.on_odom, queue_size=5)
    rospy.Subscriber("/lite3/scan/arrived", Bool, st.on_arrived, queue_size=5)
    rospy.Subscriber("/lite3/scan/arrival_dist", Float32, st.on_dist, queue_size=5)
    rospy.Subscriber("/scan_planner/cmd_vel", rospy.AnyMsg, st.on_cmd, queue_size=5)

    goal_pub = rospy.Publisher("/v8/bt/goal", PoseStamped, queue_size=1)
    out = {"steps": []}

    def step(key, value):
        out["steps"].append("%s=%s" % (key, value))
        rospy.loginfo("[p4-bt] %s=%s", key, value)

    def rn():
        return "bt=%s/%s outcome=%s seq=%s path=%s/%s" % (
            (st.bt or {}).get("event"), (st.bt or {}).get("root"),
            (st.bt or {}).get("outcome"), (st.bt or {}).get("outcome_seq"),
            (st.bt or {}).get("path_state"), (st.bt or {}).get("path_points"))

    t0 = time.time()
    while time.time() - t0 < 120 and (st.occ is None or st.odom is None or st.bt is None):
        rospy.sleep(0.3)
    if st.occ is None or st.odom is None or st.bt is None:
        print(json.dumps({"verdict": "INVALID", "reason": "occupancy/odom/BT status not ready",
                          "steps": out["steps"]}, ensure_ascii=False))
        return 9

    tree = cKDTree(st.occ)
    fixed = os.environ.get("P4_GOAL_XY")
    if fixed:
        # 斜坡等走廊场景：固定目标（LIO/SCAN 系），避开随机目标选到坡侧。
        gx, gy = [float(v) for v in fixed.split(",")]
        gr, gclear = math.hypot(gx - st.odom[0], gy - st.odom[1]), None
    else:
        pick = pick_goal(tree, st.odom)
        if pick is None:
            print(json.dumps({"verdict": "INVALID", "reason": "no free goal", "steps": out["steps"]},
                             ensure_ascii=False))
            return 9
        gx, gy, gr, gclear = pick
    out["goal"] = [round(gx, 3), round(gy, 3), round(gr, 3),
                   None if gclear is None else round(gclear, 3)]

    get_svc = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    set_svc = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    def pose_tuple(pose):
        p, q = pose.position, pose.orientation
        return (p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    try:
        rsp0 = get_svc(INJECT_MODEL, "world")
        if not rsp0.success:
            raise RuntimeError("get_model_state success=False: %s" % rsp0.status_message[:60])
        orig = pose_tuple(rsp0.pose)
    except Exception as error:  # noqa: BLE001 - fail closed with the raw error
        print(json.dumps({"verdict": "INVALID", "reason": "GetModelState failed: %r" % (error,),
                          "steps": out["steps"]}, ensure_ascii=False))
        return 9

    def restore():
        try:
            ms = ModelState()
            ms.model_name = INJECT_MODEL
            ms.reference_frame = "world"
            ms.pose.position.x, ms.pose.position.y, ms.pose.position.z = orig[0], orig[1], orig[2]
            ms.pose.orientation.x, ms.pose.orientation.y = orig[3], orig[4]
            ms.pose.orientation.z, ms.pose.orientation.w = orig[5], orig[6]
            set_svc(ms)
            rsp = get_svc(INJECT_MODEL, "world")
            return restore_verdict(orig, pose_tuple(rsp.pose))
        except Exception as error:  # noqa: BLE001
            return {"ok": False, "why": repr(error)[:120]}

    goal = PoseStamped()
    goal.header.frame_id = "world"
    goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = gx, gy, 0.0
    goal.pose.orientation.w = 1.0
    goal_pub.publish(goal)
    t_goal = time.monotonic()
    t_goal_sim = rospy.Time.now().to_sec()   # A7 判据④ 的 dt 是 sim 时间
    step("bt_goal_sent", "%.2f,%.2f" % (gx, gy))

    # 1) wait for the BT to reach path-ready (bridge status) and SCAN's first bspline
    t_first = time.time()
    first = None
    while time.time() - t_first < 45:
        if st.bs is not None and (st.path_status or {}).get("ok"):
            first = bs_ident(st.bs)
            break
        rospy.sleep(0.25)
    out["traj_before"] = first
    step("path_status", json.dumps(st.path_status, ensure_ascii=False))
    if first is None:
        out["restore"] = restore()
        print(json.dumps({"verdict": "INVALID", "reason": "no BT path-ready / first bspline",
                          "detail": out, "steps": out["steps"]}, ensure_ascii=False))
        return 9

    # 2) inject obs_5 on the first curve, same geometry as p4_replan
    c0 = eval_curve(st.bs)
    if c0 is None or len(c0) < 5:
        out["restore"] = restore()
        print(json.dumps({"verdict": "INVALID", "reason": "first curve not evaluable",
                          "detail": out, "steps": out["steps"]}, ensure_ascii=False))
        return 9
    ip = c0[int(len(c0) * INJECT_FRAC)]
    ix, iy = float(ip[0]), float(ip[1])
    ms = ModelState()
    ms.model_name = INJECT_MODEL
    ms.reference_frame = "world"
    # 注入点 (ix, iy) 是 SCAN/LIO 系坐标；SetModelState 要 world 系。用机器人
    # 当前 Gazebo world 位姿与 LIO 位姿求 2D 刚体变换 T：world = R*lio + t。
    try:
        rsp_robot = get_svc("lite3_planar", "world")
        if not rsp_robot.success:
            raise RuntimeError("get_model_state(lite3_planar) success=False")
        wp = rsp_robot.pose.position
        wq = rsp_robot.pose.orientation
        wyaw = math.atan2(2.0 * (wq.w * wq.z + wq.x * wq.y),
                          1.0 - 2.0 * (wq.y * wq.y + wq.z * wq.z))
        dth = wyaw - (st.odom_yaw or 0.0)
        c, s_ = math.cos(dth), math.sin(dth)
        tx = wp.x - (c * st.odom[0] - s_ * st.odom[1])
        ty = wp.y - (s_ * st.odom[0] + c * st.odom[1])
        wx, wy = c * ix - s_ * iy + tx, s_ * ix + c * iy + ty
    except Exception as error:  # noqa: BLE001
        out["restore"] = restore()
        print(json.dumps({"verdict": "INVALID", "reason": "world transform failed: %r" % (error,),
                          "detail": out, "steps": out["steps"]}, ensure_ascii=False))
        return 9
    ms.pose.position.x, ms.pose.position.y, ms.pose.position.z = wx, wy, INJECT_Z
    ms.pose.orientation.w = 1.0
    try:
        set_svc(ms)
    except Exception as error:  # noqa: BLE001
        out["restore"] = restore()
        print(json.dumps({"verdict": "INVALID", "reason": "SetModelState failed: %r" % (error,),
                          "detail": out, "steps": out["steps"]}, ensure_ascii=False))
        return 9
    out["inject"] = {"model": INJECT_MODEL, "lio_at": [round(ix, 3), round(iy, 3)],
                     "world_at": [round(wx, 3), round(wy, 3)], "frac": INJECT_FRAC}
    step("injected", "%.2f,%.2f" % (ix, iy))

    # 3) disturbance seen by the occupancy
    seen = 0
    t_i = time.time()
    while time.time() - t_i < 30:
        if st.occ is not None:
            seen = int(np.sum(np.linalg.norm(st.occ - np.array([ix, iy]), axis=1) < SEE_RADIUS))
            if seen >= SEE_MIN_PTS:
                break
        rospy.sleep(0.3)
    out["inject"]["occ_pts_near"] = seen
    if seen < SEE_MIN_PTS:
        out["restore"] = restore()
        print(json.dumps({"verdict": "INVALID", "reason": "disturbance not seen (%d<20)" % seen,
                          "detail": out, "steps": out["steps"]}, ensure_ascii=False))
        return 9
    step("disturbance_seen", seen)

    # 4) wait for a new B-spline identity after the injection
    def stable(timeout=30.0):
        last = bs_ident(st.bs)
        start = time.time()
        while time.time() - start < timeout:
            rospy.sleep(0.5)
            cur = bs_ident(st.bs)
            if (cur and last and cur["start_time"] == last["start_time"]
                    and cur["n_ctrl"] == last["n_ctrl"] and time.time() - start > 3.0):
                return cur
            last = cur
        return bs_ident(st.bs)

    new_id = stable()
    out["traj_after"] = new_id
    replanned = bool(new_id and first and new_id["start_time"] != first["start_time"])
    step("replanned", replanned)
    if not replanned:
        out["restore"] = restore()
        print(json.dumps({"verdict": "FAIL", "reason": "no new trajectory identity after injection",
                          "detail": out, "steps": out["steps"]}, ensure_ascii=False))
        return 1

    # 5) clearance of the new curve (2D + 3D A23)
    cn = eval_curve(st.bs)
    occ_snapshot, occ_z_snapshot = st.occ, st.occ_z
    if cn is None or len(cn) < 5 or occ_snapshot is None:
        out["restore"] = restore()
        print(json.dumps({"verdict": "INVALID", "reason": "new curve/occupancy not evaluable",
                          "detail": out, "steps": out["steps"]}, ensure_ascii=False))
        return 9
    occ3 = np.column_stack([occ_snapshot, occ_z_snapshot]) if (
        occ_z_snapshot is not None and len(occ_z_snapshot) == len(occ_snapshot)) else None
    tree2 = cKDTree(occ_snapshot)
    dmin2 = float(min(tree2.query(p)[0] for p in cn))
    dmin3 = float(min(np.sqrt((occ3[:, 0] - p[0]) ** 2 + (occ3[:, 1] - p[1]) ** 2
                              + occ3[:, 2] ** 2).min() for p in cn)) if occ3 is not None else dmin2
    dbox = float(np.linalg.norm(np.array([ix, iy]) - cn, axis=1).min())
    out["clearance"] = {"min_dist_to_any_occ_2d": round(dmin2, 3),
                        "min_dist_to_any_occ_3d": round(dmin3, 3),
                        "min_dist_to_injected": round(dbox, 3),
                        "avoid_gate": AVOID_DIST, "occ_gate": 0.10}

    # 6) watch the BT outcome and the physical distance to the injected box
    robot_min = 99.0
    deadline = time.time() + 300.0
    while time.time() < deadline:
        if st.odom is not None:
            robot_min = min(robot_min, float(np.linalg.norm(np.array(st.odom) - np.array([ix, iy]))))
        if (st.bt or {}).get("outcome"):
            break
        rospy.sleep(0.2)
    outcome = (st.bt or {}).get("outcome")
    dist = st.dist if st.dist is not None else (
        math.hypot(st.odom[0] - gx, st.odom[1] - gy) if st.odom else None)
    dt = rospy.Time.now().to_sec() - t_goal_sim
    dt_wall = time.monotonic() - t_goal
    out["bt"] = {"outcome": outcome, "outcome_seq": (st.bt or {}).get("outcome_seq"),
                 "event": (st.bt or {}).get("event"),
                 "path_points": (st.bt or {}).get("path_points"),
                 "planner_mode": (st.bt or {}).get("planner_mode")}
    out["arrive"] = {"dist_end": None if dist is None else round(dist, 3),
                     "dt_sim": round(dt, 2), "dt_wall": round(dt_wall, 2),
                     "cmd_count": st.cmd_n}
    out["physical"] = {"robot_min_to_injected": round(robot_min, 3), "box_half_width": 0.40}
    out["restore"] = restore()
    out["scan_arrived"] = st.arrived

    verdict = "PASS"
    reason = "BT outcome=reached after dynamic replan"
    if outcome != "reached":
        verdict, reason = "FAIL", "BT outcome=%s" % outcome
    elif dist is None or dist > DEV_MAX:
        verdict, reason = "FAIL", "endpoint %s > %.2f" % (dist, DEV_MAX)
    elif dbox < AVOID_DIST:
        verdict, reason = "FAIL", "trajectory %.3f < avoid %.2f" % (dbox, AVOID_DIST)
    elif robot_min <= 0.40:
        verdict, reason = "FAIL", "physical %.3f <= box half 0.40" % robot_min
    elif a23_breach(dmin3)[0]:
        verdict, reason = "FAIL", "A23 3D %.3f < 0.10" % dmin3
    elif dt > DT_MAX:
        verdict, reason = "FAIL", "dt_sim %.1f > %.0f" % (dt, DT_MAX)
    elif not (out.get("restore") or {}).get("ok"):
        verdict, reason = "INVALID", "world restore not verified"
    out["verdict"] = verdict
    out["reason"] = reason
    out["pass"] = verdict == "PASS"
    out["fail"] = verdict == "FAIL"
    out["invalid"] = verdict == "INVALID"
    print(json.dumps(out, ensure_ascii=False))
    return 0 if verdict == "PASS" else (9 if verdict == "INVALID" else 1)


if __name__ == "__main__":
    sys.exit(main())
