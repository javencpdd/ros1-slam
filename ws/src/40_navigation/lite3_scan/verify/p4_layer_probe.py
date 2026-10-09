#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""p4_layer_probe.py —— 把「注入体没被看见」分到**三层**：原始雷达 → LIO 云 → SCAN 占据图（#58 前置／R72）

R70 两层探针（云 vs 图）留下一个没答的问题：四个**轴向**方位（0/90/180/270）在**单帧云**里也是 0 点
（0°/90° 是云图双零，180°/270° 只是累积图里留着早先的点）。云已经是 faster-lio 之后的产物，
所以「传感器图案本身就没打到」与「LIO 的体素降采样把小目标抹掉」这两案**还没分开**。
本件加最前面一层：`/rslidar_points`（插件原始输出，frame_id=rslidar，LIO 之前）。

三层的读法（互斥，不重叠计数）：
  raw=0  且 cloud=0 且 map=0 ⇒ **传感器/图案面**（射线根本没打到，与 LIO 和 SCAN 无关）
  raw>0  且 cloud=0          ⇒ **LIO 面**（打到了但被降采样/滤波抹掉）
  cloud>0 且 map=0           ⇒ **建图/更新面**（云里有、占据图没长出来）
  其余                       ⇒ 看得见，或与累积时序有关（图有云无）

坐标系声明（本件只在**机体静止、偏航≈0** 时成立，且把这句话写在脸上）：
  spawn 参数 `-x 0 -y 0 -z 0.15` 且无 `-Y` ⇒ 机体出生在原点、朝向 0 ⇒ 传感器系与世界/地图系只差 (0,0,0.38) 的平移，
  x/y 可直接比（LIO 报的机体位 (0.01,0.02) 与原点差 2cm ≪ 半径 0.55m）。
  **R71 已证：外部写的机体偏航挂不住、且 LIO 报的朝向不跟随 ⇒ 本件不做任何需要机体系的判断。**

零改点：不动 `p4_replan.py`、不动任何门值（半径 0.55 与 SEE_RADIUS 同值不放宽）、世界件不动、
不投 goal、不发 /cmd_vel；`axis()` 复用 `p1_occ_sample`。
rc：0=三层都通（本条件下不复现）1=定位到某一层（判据行里写哪一层）9=前置/对照不过
"""
import json
import math
import os
import sys
import time

import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from p1_occ_sample import axis                                  # noqa: E402

BOX = os.environ.get("P4_MODEL", "obs_5")
Z_BOX = 0.40
R = float(os.environ.get("PROBE_R", "0.55"))
DIST = float(os.environ.get("PROBE_D", "1.0"))
HOME = (5.8, -1.0)
POS_CTRL = (1.5, 1.2)
NEG_CTRL = (20.0, 20.0)
HEADINGS = (0, 45, 90, 135, 180, 225, 270, 315)
SETTLE_S = 3.0
SAMPLE_S = 6.0
LAYERS = ("raw", "cloud", "map")
TOPICS = {"raw": "/rslidar_points", "cloud": "/cloud_registered", "map": "/grid_map/occupancy"}


def main():
    out = os.environ.get("LY_OUT")
    log = []

    def say(s):
        print(s)
        log.append(s)

    rospy.init_node("p4_layer_probe", anonymous=True)
    pose = {}

    def on_odom(m):
        pose["x"] = m.pose.pose.position.x
        pose["y"] = m.pose.pose.position.y
    rospy.Subscriber("/LIO/odom_vehicle", Odometry, on_odom, queue_size=5)

    buf = dict((k, []) for k in LAYERS)

    def mk(key):
        def cb(m):
            if "x" in pose:
                buf[key].append(m)
        return cb
    for k in LAYERS:
        rospy.Subscriber(TOPICS[k], PointCloud2, mk(k), queue_size=2, buff_size=2 ** 26)

    for _ in range(60):
        if "x" in pose and all(buf[k] for k in LAYERS):
            break
        rospy.sleep(0.5)
    missing = [k for k in LAYERS if not buf[k]]
    if "x" not in pose or missing:
        say("P4 三层分离探针 = INVALID|前置未就绪（odom %s，缺层 %s）⇒ 不判任何一层"
            % ("有" if "x" in pose else "无", missing or "无"))
        return 9
    say("机体位姿 (%.2f, %.2f)；帧数 raw=%d cloud=%d map=%d"
        % (pose["x"], pose["y"], len(buf["raw"]), len(buf["cloud"]), len(buf["map"])))

    def pts_near(msg, cx, cy):
        x, y, z = axis(msg, "x"), axis(msg, "y"), axis(msg, "z")
        if x is None or y is None:
            return 0
        m = np.hypot(x - cx, y - cy) <= R
        if z is not None and len(z) == len(x):
            m = m & (z > -0.05) & (z < 0.95)
        return int(np.sum(m))

    def measure(cx, cy):
        for k in LAYERS:
            buf[k].clear()
        t0 = time.time()
        best = dict((k, 0) for k in LAYERS)
        while time.time() - t0 < SAMPLE_S:
            rospy.sleep(0.4)
            for k in LAYERS:
                for m in buf[k]:
                    best[k] = max(best[k], pts_near(m, cx, cy))
        return best

    get = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    setst = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    rospy.wait_for_service("/gazebo/set_model_state", timeout=60)

    def move(x, y):
        setst(ModelState(model_name=BOX, reference_frame="world",
                         pose=Pose(position=Point(x, y, Z_BOX)), twist=Twist()))

    say("== 对照面（三层都要过）==")
    pc = measure(*POS_CTRL)
    nc = measure(*NEG_CTRL)
    say("  阳性（obs_1 原位）raw=%d cloud=%d map=%d｜阴性（世界外）raw=%d cloud=%d map=%d"
        % (pc["raw"], pc["cloud"], pc["map"], nc["raw"], nc["cloud"], nc["map"]))
    if not (pc["map"] > 0 and pc["raw"] > 0 and
            all(nc[k] == 0 for k in LAYERS)):
        say("P4 三层分离探针 = INVALID|对照面不成立（阳性 raw=%d map=%d 需>0；阴性 %s 需全 0）"
            "⇒ 计数口径不可信，不出结论" % (pc["raw"], pc["map"], {k: nc[k] for k in LAYERS}))
        return 9

    say("== 八方位注入（机体不动，距离 %.2fm，半径 %.2fm）==" % (DIST, R))
    rows = []
    for h in HEADINGS:
        tx = pose["x"] + DIST * math.cos(math.radians(h))
        ty = pose["y"] + DIST * math.sin(math.radians(h))
        move(tx, ty)
        rospy.sleep(SETTLE_S)
        got = measure(tx, ty)
        act = get(BOX, "world")
        dev = math.hypot(act.pose.position.x - tx, act.pose.position.y - ty)
        if dev > 0.05:
            say("  方位 %3d° 注入未落地（偏差 %.2fm）⇒ 该方位作废" % (h, dev))
            rows.append({"heading": h, "landed": False})
            continue
        if got["raw"] == 0:
            cls = "传感器/图案面（射线就没打到）"
        elif got["cloud"] == 0:
            cls = "LIO 面（打到了，云里没有）"
        elif got["map"] == 0:
            cls = "建图/更新面（云里有，图里没长）"
        elif got["map"] > 0:
            cls = "三层都通（看得见）"
        else:
            cls = "异常态（不参与分类）"
        say("  方位 %3d° (%6.2f,%6.2f) raw=%3d cloud=%3d map=%3d ｜ %s"
            % (h, tx, ty, got["raw"], got["cloud"], got["map"], cls))
        rows.append({"heading": h, "target": [round(tx, 3), round(ty, 3)], "landed": True, "cls": cls, **got})
        move(*HOME)
        rospy.sleep(1.5)

    landed = [r for r in rows if r.get("landed")]
    n_raw = sum(1 for r in landed if r["raw"] == 0)
    n_lio = sum(1 for r in landed if r["raw"] > 0 and r["cloud"] == 0)
    n_map = sum(1 for r in landed if r["cloud"] > 0 and r["map"] == 0)
    n_ok = sum(1 for r in landed if r["cloud"] > 0 and r["map"] > 0)
    say("汇总：落地 %d 方位｜传感器缺=%d｜LIO 抹掉=%d｜建图没长=%d｜三层都通=%d（四桶互斥）"
        % (len(landed), n_raw, n_lio, n_map, n_ok))
    if not landed:
        say("P4 三层分离探针 = INVALID|八方位全部未落地 ⇒ 注入面本次不可用，不判层")
        rc = 9
    elif n_raw and not (n_lio or n_map):
        say("P4 三层分离探针 = FAIL|O-9 落在**传感器/图案面**：%d 个方位原始点云里就没有 ⇒ "
            "与 faster-lio 体素、SCAN 更新都无关" % n_raw)
        rc = 1
    elif n_lio and not (n_raw or n_map):
        say("P4 三层分离探针 = FAIL|O-9 落在**LIO 面**：原始云有回波、去畸变/降采样后没了 ⇒ %d 个方位" % n_lio)
        rc = 1
    elif n_map and not (n_raw or n_lio):
        say("P4 三层分离探针 = FAIL|O-9 落在**建图/更新面**：云里有、占据图没长出来 ⇒ %d 个方位" % n_map)
        rc = 1
    elif (n_raw + n_lio + n_map) == 0:
        say("P4 三层分离探针 = PASS|八方位三层全通 ⇒ 本条件下不复现 O-9（差异在机体运动/时序，另批）")
        rc = 0
    else:
        say("P4 三层分离探针 = FAIL|多层的并存：传感器缺=%d LIO 抹掉=%d 建图没长=%d ⇒ 需分别处置，不合并归因"
            % (n_raw, n_lio, n_map))
        rc = 1

    move(*HOME)
    if out:
        with open(out, "w") as fh:
            fh.write(json.dumps({"rc": rc, "pose": [pose.get("x"), pose.get("y")], "dist": DIST,
                                 "radius": R, "topics": TOPICS, "pos_ctrl": pc, "neg_ctrl": nc,
                                 "rows": rows,
                                 "tally": {"raw": n_raw, "lio": n_lio, "map": n_map, "ok": n_ok}},
                                ensure_ascii=False, indent=2) + "\n")
        with open(out + ".log", "w") as fh:
            fh.write("\n".join(log) + "\n")
    say("分档自证：判据行首段与 rc 必须同向（PASS↔0／FAIL↔1／INVALID↔9）；本次 rc=%d" % rc)
    return rc


if __name__ == "__main__":
    sys.exit(main())
