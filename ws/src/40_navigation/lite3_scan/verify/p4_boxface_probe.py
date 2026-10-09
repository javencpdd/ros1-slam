#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""p4_boxface_probe.py —— 用**箱体自身 OBB** 计数重测四层（R76；修 R70/R72/R74/R75 共用的口径错）

错在哪（一句话）：在册注入体 `obs_5` 是 **0.8×0.8×0.8 的箱子**（`m1_world.world:99`，`<pose>5.8 -1.0 0.4 0 0 -0.6</pose>`
⇒ 心在 z=0.40、半宽 0.40、**还带 −0.6 rad 偏航**），而我前四批都按"0.1m 见方的小目标"用**圆柱/球半径**去数点：
0.55m 圆柱把箱体自身、地面、背景墙混成一锅；R75 的 0.25m 贴箱球又**整批读 0**（因为回波都在距心 0.40m 的表面上，
球半径比箱体半宽还小）。⇒ 那两批的层结论我已在 X22 里主动挂起。

本件的口径：**只数落在该箱 OBB 内的点**（把点换进箱子自身坐标系，三轴都 ≤0.40 才算）。
  · 雷达只能打到一个面的**表面**上，表面点距心 ≤ 半对角 0.69、且三轴偏差 ≤0.40 ⇒ OBB 正好包住所有合法回波；
  · 地面（心下方 0.40）与背景墙（箱外）都被排除；
  · 同时另数一个 **shell 环带**（距心 0.8–1.6m）作为"背景有多少点"的对照，把旧口径的污染源量化在明面上。
分层判据用 `in`（OBB 内点数）：raw_in=0 ⇒ 传感器/插件面；raw_in>0 且 body_in=0 ⇒ A preprocess；
body_in>0 且 cloud_in=0 ⇒ B 体素降采样；cloud_in>0 且 map_in=0 ⇒ C 下游建图；全 >0 ⇒ 不复现。

两道自锁（不过判 INVALID，不降格成 0）：
  ①**帧旋转检查**：OBB 用世界系偏航直接算，前提是该消息 frame 与 world **只差平移**。件里读 TF 的旋转角，
    |yaw| > 3° 即判该层不可用（我们机体静止、`world→camera_init` 是 identity 静态边，预期为 0，但**不假设、实测**）。
  ②**landmark 实测**：阳性对照用现成的 `obs_1`（同样 0.8³、偏航 0.4 rad），要求 raw 层 OBB 内 ≥ 50 点，
    否则说明"OBB＋声明偏移"这套几何本身没对准，直接 INVALID 不出结论。

零改点：不动 `p4_replan.py`、不动四道门值、不动任何 faster-lio 参数/副本、世界件不动、不投 goal、不发 /cmd_vel；
`axis()` 复用 `p1_occ_sample`（不另写解析，02 坑43）。带 `--selftest` 零节点几何自测（S-1…S-6），起工前必须先绿（坑48 处置④）。
rc：0=不复现 1=定位到某一层 5=selftest 不过 9=前置/自锁不过
"""
import json
import math
import os
import sys
import time

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Point, PointStamped, Pose, Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from p1_occ_sample import axis                                  # noqa: E402

BOX = os.environ.get("P4_MODEL", "obs_5")
Z_BOX = 0.40
HALF = 0.40                                                        # 世界件 obs_5/obs_1 的 box size 0.8 ⇒ 半宽 0.40
EPS = 0.01                                                       # 面点恰在 ±half 边界上，浮点会把它们挤出去（selftest 实测丢 212 点）
Z_FLOOR = -0.35                                                  # OBB 下界留 5cm 挡地面（箱子底面恰在地面上）
SHELL = (0.8, 1.6)                                                 # 背景环带（距箱心），量化旧口径的污染源
DIST = float(os.environ.get("PROBE_D", "1.0"))
HOME = (5.8, -1.0)
POS_CTRL = (1.5, 1.2)                                              # obs_1 原位（0.8³、yaw 0.4 rad）
POS_CTRL_YAW = 0.4
NEG_CTRL = (20.0, 20.0)
HEADINGS = (0, 45, 90, 135, 180, 225, 270, 315)
SETTLE_S = 3.0
SAMPLE_S = 6.0
LAYERS = ("raw", "body", "cloud", "map")
TOPICS = {"raw": "/rslidar_points", "body": "/cloud_registered_body",
          "cloud": "/cloud_registered", "map": "/grid_map/occupancy"}
DECL_Z = {"rslidar": 0.38}                                         # xacro:55 base_link→rslidar 静态边
listener = None


def obb_counts(x, y, z, cx, cy, cz, yaw, half=HALF, shell=SHELL):
    """返回 (OBB 内点数, shell 环带点数, OBB 内最近点距离)。OBB＝箱子自身坐标系三轴 ≤half。

    z 用 [Z_FLOOR, +half] 而不是 ±half：**箱子就坐在地上**（pose z=0.4、半宽 0.4 ⇒ 底面恰在 z=0 的地面），
    地面点会正好落在 OBB 的下边界上被算进来（selftest 实测虚高 ~370 点）。留 5cm 余量把地面平面挡掉，
    箱侧壁 z_local>−0.35 的回波全部保留。
    """
    dx, dy, dz = np.asarray(x) - cx, np.asarray(y) - cy, np.asarray(z) - cz
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx, ly = c * dx - s * dy, s * dx + c * dy
    inside = (np.abs(lx) <= half + EPS) & (np.abs(ly) <= half + EPS) & (dz >= Z_FLOOR) & (dz <= half + EPS)
    r = np.sqrt(dx * dx + dy * dy + dz * dz)
    ring = (r >= shell[0]) & (r <= shell[1])
    n = int(np.sum(inside))
    rr = np.sqrt(np.asarray(x) ** 2 + np.asarray(y) ** 2 + np.asarray(z) ** 2)   # 距**该消息 frame 原点**＝本层自己的量程
    near = float(r[inside].min()) if n else float("nan")
    rmin = float(rr[inside].min()) if n else float("nan")
    rmax = float(rr[inside].max()) if n else float("nan")
    return n, int(np.sum(ring)), near, rmin, rmax


def selftest():
    """零节点几何自测：0.8³ 箱子表面点＋地面＋背景墙，验 OBB 口径的六件事（坑48 处置④：起工前先绿）。"""
    ok = True
    yaw = -0.6
    face = []
    for _ in range(600):                                           # 顶面 z=+0.4（箱子最高面）
        face.append((np.random.uniform(-.4, .4), np.random.uniform(-.4, .4), .4))
    for _ in range(400):                                           # 侧面
        face.append((.4, np.random.uniform(-.4, .4), np.random.uniform(-.4, .4)))
    fx = np.array([p[0] for p in face]) * math.cos(yaw) - np.array([p[1] for p in face]) * math.sin(yaw) + 1.0
    fy = np.array([p[0] for p in face]) * math.sin(yaw) + np.array([p[1] for p in face]) * math.cos(yaw) + 0.0
    fz = np.array([p[2] for p in face]) + 0.4
    gx = np.random.uniform(-3, 5, 60000)
    gy = np.random.uniform(-3, 5, 60000)
    gz = np.zeros(len(gx))                                          # 地面 z=0（正卡在箱子底面上）
    wx = np.full(4000, 3.0)                                        # 3m 外一面墙
    wy = np.random.uniform(-3, 3, 4000)
    wz = np.random.uniform(0, 1, 4000)
    X, Y, Z = np.concatenate([fx, gx, wx]), np.concatenate([fy, gy, wy]), np.concatenate([fz, gz, wz])
    Xb, Yb, Zb = np.concatenate([fx, gx]), np.concatenate([fy, gy]), np.concatenate([fz, gz])
    base = obb_counts(fx, fy, fz, 1.0, 0.0, 0.4, yaw)[0]            # 只有箱面点时的基准数（解析＝1000）
    withg = obb_counts(X, Y, Z, 1.0, 0.0, 0.4, yaw)[0]              # 加地面/墙之后
    for name, cond, got in (
        ("S-1 纯箱面点基准＝解析 1000", lambda g: 940 <= g <= 1000, base),
        ("S-2 加 6 万地面点＋墙后 OBB 几乎不变（挡得住地面）", lambda g: g <= base + 40, withg),
        ("S-3 3m 外的墙不进 OBB", lambda g: g <= base + 40, obb_counts(X, Y, Z, 1.0, 0.0, 0.4, yaw)[0]),
        ("S-4 shell 环带确实抓到背景（≫OBB，旧口径污染源）", lambda g: g[1] > 3 * g[0], obb_counts(X, Y, Z, 1.0, 0.0, 0.4, yaw)),
        ("S-5 偏航用错（按 0 算）会漏掉旋转箱的回波", lambda g: g[0] < 0.8 * base, obb_counts(X, Y, Z, 1.0, 0.0, 0.4, 0.0)),
        ("S-6 目标心挪 1.6m（沿 y）后 OBB 应掉空", lambda g: g[0] == 0, obb_counts(X, Y, Z, 1.0, 1.6, 0.4, yaw)),
        ("S-7 无地面时与有地面一致（证 S-2 不是稀疏度假象）", lambda g: abs(g - base) <= 40, obb_counts(Xb, Yb, Zb, 1.0, 0.0, 0.4, yaw)[0])):
        print("%-52s %s  值=%s" % (name, "PASS" if cond(got) else "FAIL", got))
        ok = ok and bool(cond(got))
    print("SELFTEST %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 5


def main():
    global listener
    out = os.environ.get("BX_OUT")
    log = []

    def say(s):
        print(s)
        log.append(s)

    rospy.init_node("p4_boxface_probe", anonymous=True)
    listener = tf.TransformListener()
    pose = {}

    def on_odom(m):
        pose["x"] = m.pose.pose.position.x
        pose["y"] = m.pose.pose.position.y
    rospy.Subscriber("/LIO/odom_vehicle", Odometry, on_odom, queue_size=5)

    buf = dict((k, []) for k in LAYERS)
    meta = {}

    def mk(key):
        def cb(m):
            if pose:
                buf[key].append(m)
                if key not in meta:
                    xs = axis(m, "x")
                    meta[key] = {"frame_id": m.header.frame_id, "n": 0 if xs is None else len(xs)}
        return cb
    for k in LAYERS:
        rospy.Subscriber(TOPICS[k], PointCloud2, mk(k), queue_size=2, buff_size=2 ** 26)

    for _ in range(60):
        if pose and all(buf[k] for k in LAYERS):
            break
        rospy.sleep(0.5)
    missing = [k for k in LAYERS if not buf[k]]
    if not pose or missing:
        say("P4 箱体OBB探针 = INVALID|前置未就绪（odom %s，缺层 %s）" % ("有" if pose else "无", missing or "无"))
        return 9
    for k in LAYERS:
        say("  层 %-5s %-24s frame_id=%-12s 整帧点数=%d" % (k, TOPICS[k], meta[k]["frame_id"], meta[k]["n"]))

    def frame_shift(frame):
        """该 frame 相对 world 的平移与旋转角；旋转 >3° ⇒ 不能用"世界偏航＋平移"的简化（自锁①）。"""
        tr, q = listener.lookupTransform(frame, "world", rospy.Time(0))
        yaw = math.degrees(math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] * q[1] + q[2] * q[2])))
        return tuple(tr), yaw

    shifts, bad_rot = {}, []
    for k in LAYERS:
        fr = meta[k]["frame_id"]
        if fr == "rslidar":
            shifts[k] = (pose["x"], pose["y"], DECL_Z["rslidar"])   # 声明偏移（rslidar 不接 world TF 树），由 landmark 实测校验
            continue
        try:
            off, yaw = frame_shift(fr)
        except Exception as e:
            say("P4 箱体OBB探针 = INVALID|TF 解析失败 %s（%s）⇒ 不判层，绝不把解析失败当 0" % (k, type(e).__name__))
            return 9
        shifts[k] = off
        if abs(yaw) > 3.0:
            bad_rot.append("%s:%.1f°" % (k, yaw))
    if bad_rot:
        say("P4 箱体OBB探针 = INVALID|层 %s 的 frame 相对 world 有 >3° 旋转 ⇒ OBB 简化不成立，不判层" % bad_rot)
        return 9

    def measure(cx, cy, cz, yaw):
        for k in LAYERS:
            buf[k].clear()
        t0 = time.time()
        best = dict((k, [0, 0, float("nan"), float("nan"), float("nan")]) for k in LAYERS)
        while time.time() - t0 < SAMPLE_S:
            rospy.sleep(0.4)
            for k in LAYERS:
                ox, oy, oz = shifts[k]
                for m in buf[k]:
                    x, y, z = axis(m, "x"), axis(m, "y"), axis(m, "z")
                    if x is None or y is None or len(x) == 0:
                        continue
                    if z is None or len(z) != len(x):
                        z = np.zeros(len(x))
                    got = obb_counts(np.asarray(x) + ox, np.asarray(y) + oy, np.asarray(z) + oz, cx, cy, cz, yaw)
                    if got[0] >= best[k][0]:
                        best[k] = list(got)
        return best

    get = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    setst = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    rospy.wait_for_service("/gazebo/set_model_state", timeout=60)

    def move(x, y):
        setst(ModelState(model_name=BOX, reference_frame="world",
                         pose=Pose(position=Point(x, y, Z_BOX)), twist=Twist()))

    say("== 对照面（OBB 口径）==")
    pc = measure(POS_CTRL[0], POS_CTRL[1], Z_BOX, POS_CTRL_YAW)
    say("  阳性 obs_1(1.5,1.2,yaw0.4) " + " ".join("%s=%d/%d" % (k, pc[k][0], pc[k][1]) for k in LAYERS)
        + "（格式 OBB内/背景环带）")
    say("  阳性各层 OBB 内点的**距本层原点量程 rmin–rmax**（blind 的直接证据面）："
        + " ".join("%s=%.2f–%.2f" % (k, pc[k][3], pc[k][4]) for k in LAYERS if pc[k][0]))
    if pc["raw"][0] < 50:
        say("P4 箱体OBB探针 = INVALID|landmark 不过：raw 层在 obs_1 的 OBB 内只有 %d 点（需 ≥50）"
            "⇒ 几何口径本身没对准，不出层结论" % pc["raw"][0])
        return 9
    nc = measure(NEG_CTRL[0], NEG_CTRL[1], Z_BOX, 0.0)
    say("  阴性 (20,20) " + " ".join("%s=%d/%d" % (k, nc[k][0], nc[k][1]) for k in LAYERS))
    if any(nc[k][0] for k in LAYERS):
        say("P4 箱体OBB探针 = INVALID|世界外仍数到点（%s）⇒ 口径漏了，不出结论" % {k: nc[k][0] for k in LAYERS})
        return 9

    say("== 八方位注入（机体不动，心距 %.2fm，OBB 半宽 %.2fm）==" % (DIST, HALF))
    rows = []
    for h in HEADINGS:
        tx = pose["x"] + DIST * math.cos(math.radians(h))
        ty = pose["y"] + DIST * math.sin(math.radians(h))
        move(tx, ty)
        rospy.sleep(SETTLE_S)
        got = measure(tx, ty, Z_BOX, -0.6)
        a = get(BOX, "world")
        dev = math.hypot(a.pose.position.x - tx, a.pose.position.y - ty)
        if dev > 0.05:
            say("  方位 %3d° 注入未落地（偏差 %.2fm）⇒ 作废" % (h, dev))
            rows.append({"heading": h, "landed": False})
            continue
        if got["raw"][0] == 0:
            step = "传感器/插件面：箱体内一个原始点都没有"
        elif got["body"][0] == 0:
            step = "A preprocess"
        elif got["cloud"][0] == 0:
            step = "B 体素降采样"
        elif got["map"][0] == 0:
            step = "C 下游 SCAN 建图面"
        else:
            step = "四层全通（看得见）"
        say("  方位 %3d° (%6.2f,%6.2f) raw=%d/%d body=%d/%d cloud=%d/%d map=%d/%d ｜ %s"
            % (h, tx, ty, got["raw"][0], got["raw"][1], got["body"][0], got["body"][1],
               got["cloud"][0], got["cloud"][1], got["map"][0], got["map"][1], step))
        rows.append({"heading": h, "target": [round(tx, 3), round(ty, 3)], "landed": True, "step": step,
                     **dict((k, got[k]) for k in LAYERS)})
        say("      距本层原点 rmin–rmax：" + " ".join("%s=%.2f–%.2f" % (k, got[k][3], got[k][4])
            for k in LAYERS if got[k][0]))
        move(*HOME)
        rospy.sleep(1.5)

    landed = [r for r in rows if r.get("landed")]
    n_s = sum(1 for r in landed if r["raw"][0] == 0)
    nA = sum(1 for r in landed if r["raw"][0] > 0 and r["body"][0] == 0)
    nB = sum(1 for r in landed if r["body"][0] > 0 and r["cloud"][0] == 0)
    nC = sum(1 for r in landed if r["cloud"][0] > 0 and r["map"][0] == 0)
    nOk = sum(1 for r in landed if r["cloud"][0] > 0 and r["map"][0] > 0)
    say("汇总（OBB 口径）：落地 %d 方位｜传感器缺=%d｜A preprocess=%d｜B 体素=%d｜C 建图=%d｜四层全通=%d"
        % (len(landed), n_s, nA, nB, nC, nOk))
    if not landed:
        say("P4 箱体OBB探针 = INVALID|八方位全部未落地 ⇒ 注入面不可用")
        rc = 9
    elif n_s and not (nA or nB or nC):
        say("P4 箱体OBB探针 = FAIL|失因层＝**传感器/插件面**：%d 方位箱体内原始点为 0 ⇒ 与 faster-lio 无关" % n_s)
        rc = 1
    elif nA and not (n_s or nB or nC):
        say("P4 箱体OBB探针 = FAIL|失因环节＝**A preprocess**（raw 有、pre-voxel 没有）：%d 方位" % nA)
        rc = 1
    elif nB and not (n_s or nA or nC):
        say("P4 箱体OBB探针 = FAIL|失因环节＝**B 体素降采样**（pre-voxel 有、voxel 后没有）：%d 方位" % nB)
        rc = 1
    elif nC and not (n_s or nA or nB):
        say("P4 箱体OBB探针 = FAIL|失因环节＝**C 下游 SCAN 建图面**：%d 方位" % nC)
        rc = 1
    elif (n_s + nA + nB + nC) == 0:
        say("P4 箱体OBB探针 = PASS|八方位四层全通 ⇒ 按真实几何重测后不复现 O-9 的「层」说法，"
            "前四批的层结论确认作废（差异在运动/时序，另批）")
        rc = 0
    else:
        say("P4 箱体OBB探针 = FAIL|多层并存：传感器缺=%d A=%d B=%d C=%d ⇒ 不合并归因" % (n_s, nA, nB, nC))
        rc = 1

    move(*HOME)
    if out:
        with open(out, "w") as fh:
            fh.write(json.dumps({"rc": rc, "pose": [pose.get("x"), pose.get("y")], "dist": DIST,
                                 "half": HALF, "shell": list(SHELL), "topics": TOPICS, "meta": meta,
                                 "shifts": {k: list(v) for k, v in shifts.items()},
                                 "pos_ctrl": pc, "neg_ctrl": nc, "rows": rows,
                                 "tally": {"sensor": n_s, "A": nA, "B": nB, "C": nC, "ok": nOk}},
                                ensure_ascii=False, indent=2) + "\n")
        with open(out + ".log", "w") as fh:
            fh.write("\n".join(log) + "\n")
    say("分档自证：判据行首段与 rc 必须同向（PASS↔0／FAIL↔1／INVALID↔9）；本次 rc=%d" % rc)
    return rc


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    sys.exit(main())
