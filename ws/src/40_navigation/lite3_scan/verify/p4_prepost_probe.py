#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""p4_prepost_probe.py —— 四层刷分：把 faster-lio 内部的**哪一步**吃掉回波分开（#59 后续／R74）

R72 已证 O-9 的失因层在 `/rslidar_points` → `/cloud_registered` 之间（原始云八方位都有 1659–2221 点，
LIO 输出在四个轴向一律 0 点）。代码级阅读（见 `设计/L_faster-lio轴向丢点候选环节.md`）给出三个候选环节：
  A `preprocess`：`point_filter_num` 抽样（默认 2）＋`blind` 最小距离（在册 sim 值 **0.2** ⇒ 1.0m 处的箱子不该被它吃掉）
  B `voxel_scan_`（`filter_size_surf: 0.3` 的 PCL VoxelGrid）：0.1m 的小箱体与**地面/墙**点同格时，
    输出的那一个代表点落在 0.55m 半径之外
  C 发布面：`scan_bodyframe_pub_en: true` 已经把 **pre-voxel、post-preprocess** 的
    `/cloud_registered_body` 发出来了 ⇒ **不用改任何配置**就能把 A 与 B 分开

分层口径（互斥）：
  raw>0 且 body=0            ⇒ A（preprocess 抽样／blind／VELO32 分支处理）
  body>0 且 cloud=0          ⇒ B（0.3m 体素降采样把小目标并进别处的代表点）
  cloud>0 且 map=0           ⇒ 下游 SCAN 建图面
  全 >0                      ⇒ 本条件下不复现

帧自证（防止把帧搞错当结论，坑45/坑47 族）：`/cloud_registered_body` 的 frame_id、以及该消息**全体点**的
z 分位（p5/p50/p95）一并打印——若 z 中位在 −0.38 附近 ⇒ 该帧原点就是**雷达光心**（机体 0.38m 高），
若在 0 附近 ⇒ 原点在机体/地图面。半径计数用世界系目标心，但 **z 带随 frame_id 换算**（见 zband）。

零改点：不动 `p4_replan.py`、不动任何门值、**不动任何 faster-lio 参数或配置副本**（C 案的存在正是为了不用改）、
世界件不动、不投 goal、不发 /cmd_vel；`axis()` 复用 `p1_occ_sample`。
rc：0=不复现 1=定位到某一步（判据行写哪一步）9=前置或对照不过
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
LAYERS = ("raw", "body", "cloud", "map")
TOPICS = {"raw": "/rslidar_points", "body": "/cloud_registered_body",
          "cloud": "/cloud_registered", "map": "/grid_map/occupancy"}


def main():
    out = os.environ.get("PP_OUT")
    log = []

    def say(s):
        print(s)
        log.append(s)

    rospy.init_node("p4_prepost_probe", anonymous=True)
    pose = {}

    def on_odom(m):
        pose["x"] = m.pose.pose.position.x
        pose["y"] = m.pose.pose.position.y
    rospy.Subscriber("/LIO/odom_vehicle", Odometry, on_odom, queue_size=5)

    buf = dict((k, []) for k in LAYERS)
    meta = {}

    def mk(key):
        def cb(m):
            if "x" in pose:
                buf[key].append(m)
                if key not in meta:
                    zs = axis(m, "z")
                    meta[key] = {"frame_id": m.header.frame_id,
                                 "n": 0 if zs is None else len(zs),
                                 "z_p": None if zs is None or not len(zs) else
                                        [round(float(np.percentile(zs, q)), 3) for q in (5, 50, 95)]}
        return cb
    for k in LAYERS:
        rospy.Subscriber(TOPICS[k], PointCloud2, mk(k), queue_size=2, buff_size=2 ** 26)

    for _ in range(60):
        if "x" in pose and all(buf[k] for k in LAYERS):
            break
        rospy.sleep(0.5)
    missing = [k for k in LAYERS if not buf[k]]
    if "x" not in pose or missing:
        say("P4 四层刷分探针 = INVALID|前置未就绪（odom %s，缺层 %s）⇒ 不判任何一步"
            % ("有" if "x" in pose else "无", missing or "无"))
        return 9
    say("机体位姿 (%.2f, %.2f)" % (pose["x"], pose["y"]))
    for k in LAYERS:
        say("  层 %-5s topic=%-24s frame_id=%-12s 点数=%s z分位(p5/p50/p95)=%s"
            % (k, TOPICS[k], meta[k]["frame_id"], meta[k]["n"], meta[k]["z_p"]))
    body_oz = meta["body"]["z_p"][1] if meta["body"]["z_p"] else 0.0
    z_off = 0.0 if abs(body_oz + 0.38) < 0.15 else 0.0
    say("  帧自证：body 层 z 中位 %.3f ⇒ %s（若 −0.38 附近＝原点在雷达光心，z 带整体上移 0.38）"
        % (body_oz, "雷达光心系" if abs(body_oz + 0.38) < 0.15 else "非雷达光心系（按地图/机体面处理）"))
    if abs(body_oz + 0.38) < 0.15:
        z_off = -0.38

    def pts_near(msg, cx, cy, zlo, zhi):
        x, y, z = axis(msg, "x"), axis(msg, "y"), axis(msg, "z")
        if x is None or y is None:
            return 0
        m = np.hypot(x - cx, y - cy) <= R
        if z is not None and len(z) == len(x):
            m = m & (z > zlo) & (z < zhi)
        return int(np.sum(m))

    def measure(cx, cy):
        for k in LAYERS:
            buf[k].clear()
        t0 = time.time()
        best = dict((k, 0) for k in LAYERS)
        while time.time() - t0 < SAMPLE_S:
            rospy.sleep(0.4)
            for k in LAYERS:
                zo = z_off if k == "body" else 0.0
                for m in buf[k]:
                    best[k] = max(best[k], pts_near(m, cx, cy, -0.05 + zo, 0.95 + zo))
        return best

    get = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    setst = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    rospy.wait_for_service("/gazebo/set_model_state", timeout=60)

    def move(x, y):
        setst(ModelState(model_name=BOX, reference_frame="world",
                         pose=Pose(position=Point(x, y, Z_BOX)), twist=Twist()))

    say("== 对照面（四层都要过）==")
    pc = measure(*POS_CTRL)
    nc = measure(*NEG_CTRL)
    say("  阳性 raw=%d body=%d cloud=%d map=%d｜阴性 raw=%d body=%d cloud=%d map=%d"
        % (pc["raw"], pc["body"], pc["cloud"], pc["map"],
           nc["raw"], nc["body"], nc["cloud"], nc["map"]))
    if not (pc["raw"] > 0 and pc["body"] > 0 and pc["map"] > 0 and all(nc[k] == 0 for k in LAYERS)):
        say("P4 四层刷分探针 = INVALID|对照面不成立（阳性 %s 需 raw/body/map>0；阴性 %s 需全 0）"
            "⇒ 四层计数口径不可信，不出结论" % (pc, nc))
        return 9

    say("== 八方位注入（机体不动，距离 %.2fm，半径 %.2fm）==" % (DIST, R))
    rows = []
    for h in HEADINGS:
        tx = pose["x"] + DIST * math.cos(math.radians(h))
        ty = pose["y"] + DIST * math.sin(math.radians(h))
        move(tx, ty)
        rospy.sleep(SETTLE_S)
        got = measure(tx, ty)
        a = get(BOX, "world")
        dev = math.hypot(a.pose.position.x - tx, a.pose.position.y - ty)
        if dev > 0.05:
            say("  方位 %3d° 注入未落地（偏差 %.2fm）⇒ 作废" % (h, dev))
            rows.append({"heading": h, "landed": False})
            continue
        if got["raw"] == 0:
            step = "更早：插件/话题面（raw 就没点，与 R72 矛盾需复查）"
        elif got["body"] == 0:
            step = "A preprocess（抽样/blind/VELO32 分支）"
        elif got["cloud"] == 0:
            step = "B voxel_scan_ 0.3m 降采样（代表点跑到半径外）"
        elif got["map"] == 0:
            step = "C 下游 SCAN 建图/更新面"
        else:
            step = "四层全通（看得见）"
        say("  方位 %3d° (%6.2f,%6.2f) raw=%4d body=%4d cloud=%3d map=%3d ｜ %s"
            % (h, tx, ty, got["raw"], got["body"], got["cloud"], got["map"], step))
        rows.append({"heading": h, "target": [round(tx, 3), round(ty, 3)], "landed": True,
                     "step": step, **got})
        move(*HOME)
        rospy.sleep(1.5)

    landed = [r for r in rows if r.get("landed")]
    nA = sum(1 for r in landed if r["raw"] > 0 and r["body"] == 0)
    nB = sum(1 for r in landed if r["body"] > 0 and r["cloud"] == 0)
    nC = sum(1 for r in landed if r["cloud"] > 0 and r["map"] == 0)
    nOk = sum(1 for r in landed if r["cloud"] > 0 and r["map"] > 0)
    say("汇总：落地 %d 方位｜A preprocess 吃=%d｜B 体素降采样吃=%d｜C 建图面=%d｜四层全通=%d（互斥）"
        % (len(landed), nA, nB, nC, nOk))
    if not landed:
        say("P4 四层刷分探针 = INVALID|八方位全部未落地 ⇒ 注入面本次不可用")
        rc = 9
    elif nB and not (nA or nC):
        say("P4 四层刷分探针 = FAIL|失因环节＝**B `voxel_scan_`（filter_size_surf 0.3 的 PCL VoxelGrid）**："
            "pre-voxel 的 body 层有 %d 方位非零、voxel 之后为 0 ⇒ 小目标被并进同格代表点，与 blind/抽样无关" % nB)
        rc = 1
    elif nA and not (nB or nC):
        say("P4 四层刷分探针 = FAIL|失因环节＝**A preprocess**（抽样 `point_filter_num`／`blind`／VELO32 分支）："
            "raw 有、pre-voxel 的 body 层就没了 ⇒ %d 方位" % nA)
        rc = 1
    elif nC and not (nA or nB):
        say("P4 四层刷分探针 = FAIL|失因环节＝**C 下游 SCAN 建图面**（云里有、图里没长）：%d 方位" % nC)
        rc = 1
    elif (nA + nB + nC) == 0:
        say("P4 四层刷分探针 = PASS|八方位四层全通 ⇒ 本条件下不复现 R72（差异在时序/运动，另批）")
        rc = 0
    else:
        say("P4 四层刷分探针 = FAIL|多环节并存：A=%d B=%d C=%d ⇒ 不合并归因，需分别处置" % (nA, nB, nC))
        rc = 1

    move(*HOME)
    if out:
        with open(out, "w") as fh:
            fh.write(json.dumps({"rc": rc, "pose": [pose.get("x"), pose.get("y")], "dist": DIST,
                                 "radius": R, "topics": TOPICS, "meta": meta, "body_z_offset": z_off,
                                 "pos_ctrl": pc, "neg_ctrl": nc, "rows": rows,
                                 "tally": {"A_preprocess": nA, "B_voxel": nB, "C_map": nC, "ok": nOk}},
                                ensure_ascii=False, indent=2) + "\n")
        with open(out + ".log", "w") as fh:
            fh.write("\n".join(log) + "\n")
    say("分档自证：判据行首段与 rc 必须同向（PASS↔0／FAIL↔1／INVALID↔9）；本次 rc=%d" % rc)
    return rc


if __name__ == "__main__":
    sys.exit(main())
