#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""p4_cloud_map_probe.py —— 把「扰动没被看见」拆成两层量：云里有没有 vs 图里有没有（#55 第二刀）

前情：p4_inject_probe 已证 **注入面清白**（R63 三个"未见"点全部 0.0–0.1s 落地、20s 不回位、
阴性对照能识破不存在的模型）。于是 O-9 只剩两种可能：
  ①雷达点云里就没有（感知面：机体遮挡/近场/滤波）；
  ②云里有、但 SCAN 的占据图没长出来（建图面：滑窗/阈值/raytrace/更新周期）。
这两种的处置完全不同，所以必须分层量。

做法：机体不动（不投 goal ⇒ 无轨迹无运动），把 obs_5 依次放到机体周围 8 个方位 × 1.0m，
每处 settle 3s 后采 6s，分别数 `/cloud_registered`（faster-lio 去畸变后的云）与
`/grid_map/occupancy`（SCAN 占据点）在目标点 0.55m 半径内的点数——半径与 SEE_RADIUS 同值，**不改任何门值**。
两道对照：阳性＝量世界里的现成障碍 obs_1（图里必须＞0，否则计数代码是结构性零）；
阴性＝量世界外一点 (20,20)（必须＝0，否则半径/坐标系算错）。

零改点：只运行时搬模型并还原；`axis()` 直接 import p1_occ_sample（**不另写一份点云解析**，02 坑43）。
"""
import json
import math
import os
import sys
import time

import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry          # Odometry 在 nav_msgs，不在 geometry_msgs（首跑 ImportError 就是这个）
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from p1_occ_sample import axis                                  # noqa: E402  复用同一份解析

MODEL = os.environ.get("P4_MODEL", "obs_5")
Z_BOX = 0.40
R = float(os.environ.get("PROBE_R", "0.55"))                    # 与 p4_replan.SEE_RADIUS 同值（不放宽）
DIST = float(os.environ.get("PROBE_D", "1.0"))                  # 注入点到机体的距离
HEADINGS = (0, 45, 90, 135, 180, 225, 270, 315)
SETTLE_S = 3.0
SAMPLE_S = 6.0
HOME = (5.8, -1.0)                                              # 世界件里 obs_5 的原位
POS_CTRL = (1.5, 1.2)                                           # obs_1 原位：图里必须＞0
NEG_CTRL = (20.0, 20.0)                                         # 世界外：必须＝0

cnt = {"cloud": 0, "map": 0}


def pts_near(msg, cx, cy):
    x, y = axis(msg, "x"), axis(msg, "y")
    z = axis(msg, "z")
    if x is None or y is None:
        return 0
    d = np.hypot(x - cx, y - cy)
    m = d <= R
    if z is not None and len(z) == len(x):
        m = m & (z > -0.05) & (z < 0.95)                        # 只看箱体所在的高度带
    return int(np.sum(m))


def main():
    out = os.environ.get("CM_OUT")
    log = []

    def say(s):
        print(s)
        log.append(s)

    rospy.init_node("p4_cloud_map_probe", anonymous=True)
    pose = {}

    def on_odom(m):
        pose["x"] = m.pose.pose.position.x
        pose["y"] = m.pose.pose.position.y
    rospy.Subscriber("/LIO/odom_vehicle", Odometry, on_odom, queue_size=5)

    buf = {"cloud": [], "map": []}

    def mk(key):
        def cb(m):
            if "x" in pose:
                buf[key].append(m)
        return cb
    rospy.Subscriber("/cloud_registered", PointCloud2, mk("cloud"), queue_size=2, buff_size=2 ** 26)
    rospy.Subscriber("/grid_map/occupancy", PointCloud2, mk("map"), queue_size=2, buff_size=2 ** 26)

    for _ in range(60):
        if "x" in pose and buf["map"]:
            break
        rospy.sleep(0.5)
    if "x" not in pose or not buf["map"]:
        say("P4 云/图分层探针 = INVALID|odom 或 /grid_map/occupancy 未就绪 ⇒ 前置不成立（不判任何一侧）")
        return 9
    say("机体位姿 (%.2f, %.2f)；云帧=%d 图帧=%d" % (pose["x"], pose["y"], len(buf["cloud"]), len(buf["map"])))

    get = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    setst = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    rospy.wait_for_service("/gazebo/set_model_state", timeout=60)

    def move(x, y):
        ms = ModelState(model_name=MODEL, reference_frame="world",
                        pose=Pose(position=Point(x, y, Z_BOX)), twist=Twist())
        setst(ms)

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

    say("== 对照面 ==")
    pc = measure(*POS_CTRL)
    say("  阳性（obs_1 原位 %.1f,%.1f）：cloud=%d map=%d ｜ 图里必须＞0，否则计数是结构性零" % (POS_CTRL + (pc["cloud"], pc["map"])))
    nc = measure(*NEG_CTRL)
    say("  阴性（世界外 20,20）：cloud=%d map=%d ｜ 必须全 0" % (nc["cloud"], nc["map"]))
    pos_ok = pc["map"] > 0
    neg_ok = nc["map"] == 0 and nc["cloud"] == 0
    if not (pos_ok and neg_ok):
        say("P4 云/图分层探针 = INVALID|对照面不成立（阳性 map=%d 需＞0、阴性 map=%d cloud=%d 需＝0）"
            "⇒ 计数口径本身不可信，不出结论" % (pc["map"], nc["map"], nc["cloud"]))
        return 9

    say("== 八方位注入（机体不动，距离 %.2fm，半径 %.2fm）==" % (DIST, R))
    rows = []
    for h in HEADINGS:
        tx = pose["x"] + DIST * math.cos(math.radians(h))
        ty = pose["y"] + DIST * math.sin(math.radians(h))
        move(tx, ty)
        rospy.sleep(SETTLE_S)
        got = measure(tx, ty)
        act = get(MODEL, "world")
        dev = math.hypot(act.pose.position.x - tx, act.pose.position.y - ty)
        if dev > 0.05:
            say("  方位 %3d° 注入未落地（偏差 %.2fm）⇒ 该方位作废" % (h, dev))
            rows.append({"heading": h, "landed": False})
            continue
        cls = ("云无图无＝感知面盲区" if got["cloud"] == 0 and got["map"] == 0 else
               "云有图无＝建图面丢点" if got["cloud"] > 0 and got["map"] == 0 else
               "云有图有＝看得见（与 seen 跑同型）" if got["map"] > 0 else "云无图有＝异常（图点非本次注入）")
        say("  方位 %3d° (%6.2f,%6.2f) cloud=%3d map=%3d ｜ %s" % (h, tx, ty, got["cloud"], got["map"], cls))
        rows.append({"heading": h, "target": [round(tx, 3), round(ty, 3)], "landed": True, **got, "cls": cls})
        move(*HOME)
        rospy.sleep(1.5)

    # 四桶必须互斥——首版把「云无图有」也计进感知盲区，于是 8 个方位汇总成"落地 10 方位"，
    # 判定行也跟着虚高（正是 #30「一个失因吞掉并存失因」的镜像：一个计数把两种态吞在一起）。
    landed = [r for r in rows if r.get("landed")]
    n_sense = sum(1 for r in landed if r["cloud"] == 0 and r["map"] == 0)
    n_map = sum(1 for r in landed if r["cloud"] > 0 and r["map"] == 0)
    n_ok = sum(1 for r in landed if r["cloud"] > 0 and r["map"] > 0)
    n_lag = sum(1 for r in landed if r["cloud"] == 0 and r["map"] > 0)
    say("汇总：落地 %d 方位｜云图都无（感知盲区）=%d｜云有图无（建图丢点）=%d｜都看见=%d｜"
        "图有云无（累积图 vs 单帧云不同步，不属盲区也不属丢点）=%d"
        % (len(landed), n_sense, n_map, n_ok, n_lag))
    if n_map and not n_sense:
        say("P4 云/图分层探针 = FAIL|O-9 根因落在**建图面**（云里有、图里没长出来）：%d 个方位如此" % n_map)
        rc = 1
    elif n_sense and not n_map:
        say("P4 云/图分层探针 = FAIL|O-9 根因落在**感知面**（云里就没有）：%d 个方位云图双零"
            "（另有 %d 方位属图有云无的两层不同步，不计盲区）" % (n_sense, n_lag))
        rc = 1
    elif n_sense and n_map:
        say("P4 云/图分层探针 = FAIL|两层都有：%d 方位感知缺、%d 方位建图丢 ⇒ 需分别处置" % (n_sense, n_map))
        rc = 1
    else:
        say("P4 云/图分层探针 = PASS|八方位全部看得见 ⇒ 本探针条件下不复现 O-9（差异在机体运动/时序，"
            "不在静态可见性）")
        rc = 0
    if out:
        with open(out, "w") as fh:
            fh.write(json.dumps({"rc": rc, "pose": [pose.get("x"), pose.get("y")], "dist": DIST, "radius": R,
                                 "pos_ctrl": pc, "neg_ctrl": nc, "rows": rows}, ensure_ascii=False, indent=2) + "\n")
        with open(out + ".log", "w") as fh:
            fh.write("\n".join(log) + "\n")
    return rc


if __name__ == "__main__":
    sys.exit(main())
