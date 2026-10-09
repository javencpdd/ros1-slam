#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""p4_body_frame_probe.py —— 判「近场轴向盲区」属**机体轴**还是**世界/地图轴**（#57／请求单 B9）

前情：`p4_inject_probe` 证注入面清白；`p4_cloud_map_probe` 在**机体静止（偏航≈0）**时测到
世界系 0°/90° 云图双零、四个对角方位全看见。但偏航≈0 时**机体轴与世界轴重合**，
所以那批读数**天然无法区分**两种成因：
  ①机体系（机体自身遮挡／MID360 插件图案随机体转） ⇒ 对策在传感器与装机面（与 B6／O-10 同线）；
  ②世界/地图系（建图侧体素格、局部系选择或占据更新方向） ⇒ 对策在 faster-lio／SCAN 面。
两者的处置完全不同，所以必须让**机体偏航离开 0** 再量一次。

做法：把 obs_5 固定在**世界系** 8 个方位×1.0m，逐档把机体**原地转**到偏航 0/30/60/90°
（`set_model_state` 只改朝向、不改位置，逐档 `get_model_state` 回读验证），每档测同一组方位的
`/cloud_registered` 与 `/grid_map/occupancy` 在 0.55m 半径内的点数。
  · 双零方位集合**不随偏航变**（还是世界系那几个方位）⇒ 判**世界/地图系**；
  · 双零方位集合**随偏航平移**（换到机体系里看是同一组）⇒ 判**机体系**。

三道自锁（不过就 INVALID，不判 FAIL）：①每档偏航回读误差 >3° 或位置漂 >5cm ⇒ 该档作废；
②LIO 没跟上（`/LIO/odom_vehicle` 的偏航与指令差 >25°）⇒ 该档读数不作为证据（云/图都建在 LIO 系里）；
③每档都跑阳性（obs_1 原位，图里必须 >0）与阴性（世界外 20,20，必须全 0）对照。

零改点：不动 `p4_replan.py`、不动 SEE_RADIUS/SEE_MIN_PTS/AVOID/到达门、世界件一字不动；
**不投 goal、不发 /cmd_vel**（M6 是唯一写者，本件全程不碰它——`ModelState` 里的 `twist` 是 gazebo 的
模型速度字段，不是 ROS 话题）；`axis()` 复用 `p1_occ_sample`（不另写解析，02 坑43）。

rc：0=判出**机体系**（盲区随机体转）1=判出**世界/地图系**（盲区固定）9=前置/自锁不过
"""
import json
import math
import os
import sys
import time

import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from p1_occ_sample import axis                                  # noqa: E402

BOX = os.environ.get("P4_MODEL", "obs_5")
ROBOT = os.environ.get("P4_ROBOT", "lite3_planar")
Z_BOX = 0.40
R = float(os.environ.get("PROBE_R", "0.55"))
DIST = float(os.environ.get("PROBE_D", "1.0"))
HOME = (5.8, -1.0)
POS_CTRL = (1.5, 1.2)
NEG_CTRL = (20.0, 20.0)
YAWS = [float(v) for v in os.environ.get("PROBE_YAWS", "0,30,60,90").split(",")]
HEADINGS = (0, 45, 90, 135, 180, 225, 270, 315)
SETTLE_S = 3.0
SAMPLE_S = 6.0
YAW_TOL = 3.0
POS_TOL = 0.05
LIO_TOL = 25.0


def wrap(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def yaw_of(q):
    return math.degrees(2.0 * math.atan2(math.sin(0.5 * q.z), math.cos(0.5 * q.z)))


def q_yaw(deg):
    h = math.radians(deg) / 2.0
    return Quaternion(0.0, 0.0, math.sin(h), math.cos(h))


def main():
    out = os.environ.get("BF_OUT")
    log = []

    def say(s):
        print(s)
        log.append(s)

    rospy.init_node("p4_body_frame_probe", anonymous=True)
    odom = {}
    buf = {"cloud": [], "map": []}

    def on_odom(m):
        odom["x"] = m.pose.pose.position.x
        odom["y"] = m.pose.pose.position.y
        odom["yaw"] = yaw_of(m.pose.pose.orientation)
    rospy.Subscriber("/LIO/odom_vehicle", Odometry, on_odom, queue_size=5)

    def mk(key):
        def cb(m):
            if odom:
                buf[key].append(m)
        return cb
    rospy.Subscriber("/cloud_registered", PointCloud2, mk("cloud"), queue_size=2, buff_size=2 ** 26)
    rospy.Subscriber("/grid_map/occupancy", PointCloud2, mk("map"), queue_size=2, buff_size=2 ** 26)

    for _ in range(60):
        if odom and buf["map"]:
            break
        rospy.sleep(0.5)
    if not odom or not buf["map"]:
        say("P4 机体系归属探针 = INVALID|odom 或占据图未就绪 ⇒ 前置不成立")
        return 9

    get = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    setst = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    rospy.wait_for_service("/gazebo/set_model_state", timeout=60)

    def pts_near(msg, cx, cy):
        x, y, z = axis(msg, "x"), axis(msg, "y"), axis(msg, "z")
        if x is None or y is None:
            return 0
        m = np.hypot(x - cx, y - cy) <= R
        if z is not None and len(z) == len(x):
            m = m & (z > -0.05) & (z < 0.95)
        return int(np.sum(m))

    def measure(cx, cy):
        buf["cloud"].clear()
        buf["map"].clear()
        t0 = time.time()
        best = {"cloud": 0, "map": 0}
        while time.time() - t0 < SAMPLE_S:
            rospy.sleep(0.4)
            for k in ("cloud", "map"):
                for m in buf[k]:
                    best[k] = max(best[k], pts_near(m, cx, cy))
        return best

    def move_box(x, y):
        setst(ModelState(model_name=BOX, reference_frame="world",
                         pose=Pose(position=Point(x, y, Z_BOX)), twist=Twist()))

    home_st = get(ROBOT, "world")
    if not home_st.success:
        say("P4 机体系归属探针 = INVALID|读不到机体模型 %s（%s）" % (ROBOT, home_st.status_message))
        return 9
    hx, hy, hz = home_st.pose.position.x, home_st.pose.position.y, home_st.pose.position.z
    say("机体世界原位 (%.2f, %.2f, %.2f) 偏航 %.1f°；LIO 报 (%.2f, %.2f) 偏航 %.1f°"
        % (hx, hy, hz, yaw_of(home_st.pose.orientation), odom["x"], odom["y"], odom.get("yaw", float("nan"))))

    steps = []
    for yaw in YAWS:
        st = ModelState(model_name=ROBOT, reference_frame="world",
                        pose=Pose(position=Point(hx, hy, hz), orientation=q_yaw(yaw)), twist=Twist())
        setst(st)
        rospy.sleep(1.0)
        act = get(ROBOT, "world")
        dyaw = wrap(yaw_of(act.pose.orientation) - yaw) if act.success else 180.0
        dpos = math.hypot(act.pose.position.x - hx, act.pose.position.y - hy) if act.success else 9.9
        lio_err = wrap(odom.get("yaw", 0.0) - yaw)
        ok_yaw, ok_pos, ok_lio = abs(dyaw) <= YAW_TOL, dpos <= POS_TOL, abs(lio_err) <= LIO_TOL
        say("== 偏航 %5.1f° 档｜回读误差 %.1f°、位置漂 %.2fm｜LIO 偏航 %.1f°（差 %.1f°）⇒ 回读%s、LIO%s"
            % (yaw, dyaw, dpos, odom.get("yaw", float("nan")), lio_err,
               "过" if (ok_yaw and ok_pos) else "不过", "跟上" if ok_lio else "没跟上"))
        if not (ok_yaw and ok_pos) or not ok_lio:
            steps.append({"yaw": yaw, "usable": False, "readback_err": round(dyaw, 2),
                          "pos_drift": round(dpos, 3), "lio_err": round(lio_err, 2)})
            continue

        pc = measure(*POS_CTRL)
        nc = measure(*NEG_CTRL)
        say("  对照｜阳性(1.5,1.2) cloud=%d map=%d｜阴性(20,20) cloud=%d map=%d"
            % (pc["cloud"], pc["map"], nc["cloud"], nc["map"]))
        if not (pc["map"] > 0 and nc["map"] == 0 and nc["cloud"] == 0):
            steps.append({"yaw": yaw, "usable": False, "why": "本档对照面不成立",
                          "pos_ctrl": pc, "neg_ctrl": nc})
            say("  ⇒ 本档作废（对照面不成立，计数口径不可信）")
            continue

        rows = []
        for h in HEADINGS:
            tx, ty = hx + DIST * math.cos(math.radians(h)), hy + DIST * math.sin(math.radians(h))
            move_box(tx, ty)
            rospy.sleep(SETTLE_S)
            got = measure(tx, ty)
            a = get(BOX, "world")
            dev = math.hypot(a.pose.position.x - tx, a.pose.position.y - ty) if a.success else 9.9
            if dev > POS_TOL:
                say("    方位 %3.0f° 注入未落地（偏 %.2fm）⇒ 该方位作废" % (h, dev))
                rows.append({"heading": h, "landed": False})
                continue
            bucket = ("双零" if got["cloud"] == 0 and got["map"] == 0 else
                      "云有图无" if got["cloud"] > 0 and got["map"] == 0 else
                      "都看见" if got["cloud"] > 0 and got["map"] > 0 else "图有云无")
            say("    方位 %3.0f°（机体系 %5.1f°）cloud=%3d map=%3d ｜ %s"
                % (h, wrap(h - yaw), got["cloud"], got["map"], bucket))
            rows.append({"heading": h, "heading_body": round(wrap(h - yaw), 1), "target": [round(tx, 3), round(ty, 3)],
                         "landed": True, "bucket": bucket, **got})
            move_box(*HOME)
            rospy.sleep(1.5)
        steps.append({"yaw": yaw, "usable": True, "readback_err": round(dyaw, 2), "pos_drift": round(dpos, 3),
                      "lio_err": round(lio_err, 2), "pos_ctrl": pc, "neg_ctrl": nc, "rows": rows})

    move_box(*HOME)
    rospy.sleep(1.5)
    fin = get(ROBOT, "world")
    fb = get(BOX, "world")
    say("还原：机体回读偏航 %.1f°（原位 %.1f°）、箱子回读 (%.2f, %.2f) 原位 (%.1f, %.1f)"
        % (yaw_of(fin.pose.orientation) if fin.success else float("nan"), yaw_of(home_st.pose.orientation),
           fb.pose.position.x if fb.success else float("nan"), fb.pose.position.y if fb.success else float("nan"),
           HOME[0], HOME[1]))

    usable = [s for s in steps if s.get("usable") and s.get("rows")]
    if len(usable) < 2 or len(set(s["yaw"] for s in usable)) < 2:
        say("P4 机体系归属探针 = INVALID|可用偏航档 %d 个（<2 或全同偏航）⇒ 无鉴别力，不判归属"
            % len(usable))
        rc = 9
    else:
        zf_world, zf_body = {}, {}
        for s in usable:
            z = sorted(r["heading"] for r in s["rows"] if r.get("landed") and r["bucket"] == "双零")
            zf_world[s["yaw"]] = z
            zf_body[s["yaw"]] = sorted(int(wrap(h - s["yaw"])) % 360 for h in z)
            say("  偏航 %5.1f°｜双零（世界系）%s｜同一组换算到机体系 %s" % (s["yaw"], z, zf_body[s["yaw"]]))
        nonempty = [s["yaw"] for s in usable if zf_world[s["yaw"]]]
        same_world = len(set(tuple(v) for k, v in zf_world.items() if v)) == 1 and len(nonempty) == len(usable)
        same_body = len(set(tuple(v) for k, v in zf_body.items() if v)) == 1 and len(nonempty) == len(usable)
        moved = len(set(tuple(v) for k, v in zf_world.items() if v)) > 1
        if not nonempty:
            say("P4 机体系归属探针 = INVALID|全部可用档都没有复现「双零」方位 ⇒ 本次条件下盲区不存在，"\
                "归属问题不成立（差异可能在收敛窗/时序，另批）")
            rc = 9
        elif same_body and moved:
            say("P4 机体系归属探针 = PASS|盲区**随机体转**（世界系双零集合在变、机体系里是同一组）⇒ "\
                "O-9 的轴向盲区归**机体系**（机体遮挡／插件图案面，与 B6 同线），不归建图面")
            rc = 0
        elif same_world and not moved:
            say("P4 机体系归属探针 = FAIL|盲区**固定在地图/世界系**（偏航变了、双零方位不变）⇒ "\
                "归建图侧（体素格／局部系选择／占据更新方向），不归传感器遮挡")
            rc = 1
        else:
            say("P4 机体系归属探针 = INVALID|两假说都不干净（世界系集合=%s 机体系集合=%s）⇒ 需加密偏航档或方位"
                % ({k: v for k, v in zf_world.items()}, {k: v for k, v in zf_body.items()}))
            rc = 9
        say("档位明细：可用档 %d／总 %d" % (len(usable), len(steps)))

    if out:
        with open(out, "w") as fh:
            fh.write(json.dumps({"rc": rc, "robot_home": [hx, hy, hz], "dist": DIST, "radius": R,
                                 "yaws": YAWS, "steps": steps}, ensure_ascii=False, indent=2) + "\n")
        with open(out + ".log", "w") as fh:
            fh.write("\n".join(log) + "\n")
    say("分档自证：判据行首段与 rc 必须同向（PASS↔0／FAIL↔1／INVALID↔9）；本次 rc=%d" % rc)
    return rc


if __name__ == "__main__":
    sys.exit(main())
