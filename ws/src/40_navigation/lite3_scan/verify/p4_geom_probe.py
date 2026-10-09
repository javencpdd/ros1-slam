#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""p4_geom_probe.py —— 帧对齐＋球内计数重测四层（R75；修 R74 自我暴露的计数缺陷）

R74 的帧自证查出：四层各自的 z 原点根本不同（raw=`rslidar` 在雷达光心 0.38m 高、
`/cloud_registered`=`camera_init` 在起始机体高、map=`world`），而我此前对四层**一律用同一条 z 带且不加帧偏移**
⇒ "原始云有 1659–2221 点 ⇒ 传感器看得见箱子"这句话**不成立**：那数主要是同一 0.55m **圆柱**里的背景。
本件把几何口径一次性修对：
  ①**用 TF 把目标心换算到每条消息自己的 frame**（`lookupTransform(world → msg.header.frame_id)`），
  ②**球内计数**（三维距离 ≤ 半径），**不设 z 带**，
  ③同时给两个半径：R=0.55（与在册 SEE_RADIUS 同值，不放宽）与 R=0.25（贴着箱子：箱体半对角≈0.087m，
     而箱底到地面只有 0.40m ⇒ 0.25m 球把地面/墙排除在外）。两个半径一起看，半径敏感性就摊在明面上。

分层判读（同口径下才准比）：
  raw=0 且 其余=0        ⇒ 传感器/插件面就没打到箱子
  raw>0 且 body=0        ⇒ A preprocess（抽样/VELO32 分支/blind）
  body>0 且 cloud=0      ⇒ B 体素降采样（0.3m 叶子的代表点）
  cloud>0 且 map=0       ⇒ C 下游 SCAN 建图面
  全 >0                  ⇒ 本条件下不复现

零改点：不动 `p4_replan.py`、不动门值（0.55 原值）、不动任何 faster-lio 参数/副本、世界件不动、
不投 goal、不发 /cmd_vel；`axis()` 复用 `p1_occ_sample`。TF 解析不出来 ⇒ **INVALID**，不降格成"数到 0"。
rc：0=不复现 1=定位到某一层 9=前置/对照/TF 不过
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
RADII = (0.55, 0.25)                                            # 0.55＝在册 SEE_RADIUS 同值；0.25＝贴箱体
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
listener = None


def frame_target(cx, cy, frame, stamp):
    """把世界系目标心 (cx,cy,Z_BOX) 用 TF 换算进 frame；失败即抛（调用方判 INVALID，绝不静默当 0）。

    本探针机体**静止不投 goal** ⇒ 用 Time(0)（最新变换）而不是消息时刻：雷达与机体此刻不动，
    消息时刻反而常因 TF 发布滞后触发 ExtrapolationException（那是"跑不动"不是"数到 0"）。
    """
    ps = PointStamped()
    ps.header.frame_id = "world"
    ps.header.stamp = rospy.Time(0)
    ps.point = Point(cx, cy, Z_BOX)
    return listener.transformPoint(frame, ps).point


DECL = {"rslidar": 0.38}      # xacro:55 base_link→rslidar 的静态边 origin xyz="0 0 0.38"（TF 里 rslidar 在 base_link 树下、
                              # 与 world/camera_init/body **不同树**（lio_relay_sim.launch:39-42 只连 world→camera_init），
                              # 所以 raw 层拿不到 TF 时只能用这条声明偏移——并且必须被实测校验（见 landmark_check），不默认它成立。


def sphere_counts(x, y, z, off):
    """纯几何：给点阵与该系下的目标心，返回 [R=0.55 内点数, R=0.25 内点数]。抽成独立函数是为了能在
    **零节点**下自测（G-1…G-4），不必烧一条 10 分钟的链——R75 前三跑全死在没做这一步（新坑，入 02）。"""
    d = np.sqrt((np.asarray(x) - off[0]) ** 2 + (np.asarray(y) - off[1]) ** 2 + (np.asarray(z) - off[2]) ** 2)
    i = int(np.argmin(d))
    return [int(np.sum(d <= r)) for r in RADII], (float(d[i]), float(x[i]), float(y[i]), float(z[i]))


def selftest():
    """零节点几何自测：箱体＋地面点，验球口径的两件事——贴箱球排除地面、大球把地面纳进来（＝R72/R74 的污染源）。"""
    bx, by, bz = 1.0, 0.0, 0.02                      # 箱子在 rslidar 系里的预期心（世界 1.0m、箱高 0.40 减 0.38）
    xs = np.random.uniform(bx - 0.05, bx + 0.05, 400)
    ys = np.random.uniform(by - 0.05, by + 0.05, 400)
    zs = np.random.uniform(-0.05, 0.07, 400)
    gx = np.random.uniform(-3, 3, 60000)             # 地面密到与沙箱同量级（20000/帧 ⇒ ~10^5/m²），
    gy = np.random.uniform(-3, 3, 60000)             #   否则"大球吞进多少地面"是稀疏度假象，G-2 就不是它要防的那个错
    gz = np.full(60000, -0.38)
    x, y, z = np.concatenate([xs, gx]), np.concatenate([ys, gy]), np.concatenate([zs, gz])
    dz = 0.40                                        # 箱心 z 到地面的竖直距离（0.02 −(−0.38)）
    dens = len(gx) / 36.0                             # 60000 点铺在 6×6 m²
    EXP_G = int(dens * math.pi * max(0.0, RADII[0] ** 2 - dz ** 2))   # 解析值：球缺截地面 ⇒ 圆面积×密度
    print("G-0 解析期望：大球比贴箱球多吞地面 ≈%d 点（密度 %.0f/m²、截圆半径 %.3fm）"
          % (EXP_G, dens, math.sqrt(max(0.0, RADII[0] ** 2 - dz ** 2))))
    ok = True
    for name, cond, got in (
            ("G-1 贴箱球只数到箱体", lambda g: g[1] in range(300, 401), sphere_counts(x, y, z, (bx, by, bz))[0]),
            ("G-2 大球吞进的地面数应≈解析值（防稀疏度假象）", lambda g: abs((g[0] - g[1]) - EXP_G) < 0.2 * EXP_G, sphere_counts(x, y, z, (bx, by, bz))[0]),
            ("G-3 目标偏移 0.3m 时贴箱球应掉空", lambda g: g[1] < 30, sphere_counts(x, y, z, (bx + 0.3, by, bz))[0]),
            ("G-4 最近点距离可报（landmark 校验用）", lambda g: g[0] < 0.2, sphere_counts(x, y, z, (bx, by, bz))[1])):
        print("%-40s %s  值=%s" % (name, "PASS" if cond(got) else "FAIL", got))
        ok = ok and cond(got)
    print("SELFTEST %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 5


def count(msg, centre_world, use_declared=None, body=None):
    x, y, z = axis(msg, "x"), axis(msg, "y"), axis(msg, "z")
    if x is None or y is None or len(x) == 0:
        return None, None
    if z is None or len(z) != len(x):
        z = np.zeros(len(x))
    frame = msg.header.frame_id
    if use_declared is not None:
        ox, oy, oz = body[0], body[1], DECL[use_declared]   # 机体平面位置由调用方传入（不闭包 main 局部量）
    else:
        off = frame_target(centre_world[0], centre_world[1], frame, msg.header.stamp)
        ox, oy, oz = off.x, off.y, off.z
    return sphere_counts(x, y, z, (ox, oy, oz))


def main():
    global listener
    out = os.environ.get("GM_OUT")
    log = []

    def say(s):
        print(s)
        log.append(s)

    rospy.init_node("p4_geom_probe", anonymous=True)
    listener = tf.TransformListener()   # noetic 实签：TransformerROS.__init__(args=None, cache_time=10.0)——只接 cache_time，queue_size/buffer_size 都会 TypeError（R75 连崩两次于此，签名已在本机 introspect 核实，不再靠猜）
    pose = {}

    def on_odom(m):
        pose["x"] = m.pose.pose.position.x
        pose["y"] = m.pose.pose.position.y
        pose["z"] = m.pose.pose.position.z
    rospy.Subscriber("/LIO/odom_vehicle", Odometry, on_odom, queue_size=5)

    buf = dict((k, []) for k in LAYERS)
    meta = {}

    def mk(key):
        def cb(m):
            if pose:
                buf[key].append(m)
                if key not in meta:
                    xs = axis(m, "x")
                    n = 0 if xs is None else len(xs)   # 不能写 `axis(...) or []`：返回 ndarray 时真值判断直接抛 ValueError（R75 第三崩）
                    meta[key] = {"frame_id": m.header.frame_id, "n": n}
        return cb
    for k in LAYERS:
        rospy.Subscriber(TOPICS[k], PointCloud2, mk(k), queue_size=2, buff_size=2 ** 26)

    for _ in range(60):
        if pose and all(buf[k] for k in LAYERS):
            break
        rospy.sleep(0.5)
    missing = [k for k in LAYERS if not buf[k]]
    if not pose or missing:
        say("P4 几何对齐探针 = INVALID|前置未就绪（odom %s，缺层 %s）⇒ 不判任何一层"
            % ("有" if pose else "无", missing or "无"))
        return 9
    for k in LAYERS:
        say("  层 %-5s %-24s frame_id=%-12s 整帧点数=%d" % (k, TOPICS[k], meta[k]["frame_id"], meta[k]["n"]))
    say("  preprocess 削减比：body/raw = %d/%d = %.1f%%（R74 同型观测，本件不据此下结论）"
        % (meta["body"]["n"], meta["raw"]["n"], 100.0 * meta["body"]["n"] / max(1, meta["raw"]["n"])))

    say("  几何口径：raw 层用**声明偏移**（xacro:55 base_link→rslidar 静态边 0.38m，TF 里 rslidar 不接 world）"
        "并须过 landmark 实测；body/cloud/map 三层用 **TF**（world→camera_init 静态边由 lio_relay_sim.launch:41 提供）")
    geom_paths = {"raw": "declared_offset_0.38", "body": "TF", "cloud": "TF", "map": "TF"}
    get = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    setst = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    rospy.wait_for_service("/gazebo/set_model_state", timeout=60)

    def move(x, y):
        setst(ModelState(model_name=BOX, reference_frame="world",
                         pose=Pose(position=Point(x, y, Z_BOX)), twist=Twist()))

    tf_fail = []

    def measure(cx, cy):
        for k in LAYERS:
            buf[k].clear()
        t0 = time.time()
        best = dict((k, [0, 0]) for k in LAYERS)
        seen = dict((k, False) for k in LAYERS)
        while time.time() - t0 < SAMPLE_S:
            rospy.sleep(0.4)
            for k in LAYERS:
                for m in buf[k]:
                    try:
                        c, _ = count(m, [cx, cy, Z_BOX],
                                     use_declared="rslidar" if k == "raw" else None,
                                     body=(pose["x"], pose["y"]))
                    except Exception as e:
                        tf_fail.append("%s:%s" % (k, type(e).__name__))
                        continue
                    if c is None:
                        continue
                    seen[k] = True
                    best[k] = [max(best[k][i], c[i]) for i in (0, 1)]
        return best, seen

    say("== 对照面（球内、帧对齐）==")
    pc, _ = measure(*POS_CTRL)
    lm = None                     # 声明偏移的实测校验：按 DECL 预测箱子在 rslidar 系里的位置，看原始点里最近的一粒离它多远
    for m in buf["raw"]:
        _c, near = count(m, [POS_CTRL[0], POS_CTRL[1], Z_BOX], use_declared="rslidar",
                         body=(pose["x"], pose["y"]))
        if near is not None:
            lm = near
            break
    say("  raw 层声明偏移实测校验（obs_1 原位）：离预测点最近原始点 %.3fm @ (%.2f,%.2f,%.2f)｜偏移表=%s"
        % (lm[0], lm[1], lm[2], lm[3], DECL) if lm else "  raw 层校验：拿不到 raw 消息")
    nc, _ = measure(*NEG_CTRL)
    say("  阳性(1.5,1.2) " + " ".join("%s=%d/%d" % (k, pc[k][0], pc[k][1]) for k in LAYERS))
    say("  阴性(20,20)  " + " ".join("%s=%d/%d" % (k, nc[k][0], nc[k][1]) for k in LAYERS))
    if tf_fail:
        say("P4 几何对齐探针 = INVALID|TF 解析失败 %d 次（%s）⇒ 帧换算不可信，不判层（绝不把解析失败当 0）"
            % (len(tf_fail), sorted(set(tf_fail))[:3]))
        return 9
    if not (lm is not None and lm[0] <= 0.25):
        say("P4 几何对齐探针 = INVALID|raw 层的声明偏移未被实测确认（最近原始点 %.2fm > 0.25m）⇒ raw 层不可用，"
            "A 与传感器面无法分开（body/cloud/map 三层仍按 TF 口径有效）" % (lm[0] if lm else -1.0))
        return 9
    if not (pc["raw"][1] > 0 and pc["map"][1] > 0 and all(nc[k][0] == 0 for k in LAYERS)):
        say("P4 几何对齐探针 = INVALID|对照面不成立（阳性 raw=%d map=%d 需>0，阴性 %s 需全 0）"
            % (pc["raw"][1], pc["map"][1], {k: nc[k] for k in LAYERS}))
        return 9

    say("== 八方位注入（机体不动，距离 %.2fm，球半径 %s）==" % (DIST, list(RADII)))
    rows = []
    for h in HEADINGS:
        tx = pose["x"] + DIST * math.cos(math.radians(h))
        ty = pose["y"] + DIST * math.sin(math.radians(h))
        move(tx, ty)
        rospy.sleep(SETTLE_S)
        got, seen = measure(tx, ty)
        a = get(BOX, "world")
        dev = math.hypot(a.pose.position.x - tx, a.pose.position.y - ty)
        if dev > 0.05:
            say("  方位 %3d° 注入未落地（偏差 %.2fm）⇒ 作废" % (h, dev))
            rows.append({"heading": h, "landed": False})
            continue
        if got["raw"][1] == 0:
            step = "传感器/插件面：贴箱球内原始点就为 0（箱子确实没进云）"
        elif got["body"][1] == 0:
            step = "A preprocess"
        elif got["cloud"][1] == 0:
            step = "B 体素降采样"
        elif got["map"][1] == 0:
            step = "C 下游建图面"
        else:
            step = "四层全通（看得见）"
        say("  方位 %3d° (%6.2f,%6.2f) raw=%d/%d body=%d/%d cloud=%d/%d map=%d/%d ｜ %s"
            % (h, tx, ty, got["raw"][0], got["raw"][1], got["body"][0], got["body"][1],
               got["cloud"][0], got["cloud"][1], got["map"][0], got["map"][1], step))
        rows.append({"heading": h, "target": [round(tx, 3), round(ty, 3)], "landed": True,
                     "step": step, **dict((k, got[k]) for k in LAYERS)})
        move(*HOME)
        rospy.sleep(1.5)

    landed = [r for r in rows if r.get("landed")]
    n_sense = sum(1 for r in landed if r["raw"][1] == 0)
    nA = sum(1 for r in landed if r["raw"][1] > 0 and r["body"][1] == 0)
    nB = sum(1 for r in landed if r["body"][1] > 0 and r["cloud"][1] == 0)
    nC = sum(1 for r in landed if r["cloud"][1] > 0 and r["map"][1] == 0)
    nOk = sum(1 for r in landed if r["cloud"][1] > 0 and r["map"][1] > 0)
    say("汇总（贴箱球 R=0.25 口径）：落地 %d 方位｜传感器缺=%d｜A preprocess=%d｜B 体素=%d｜C 建图=%d｜全通=%d"
        % (len(landed), n_sense, nA, nB, nC, nOk))
    if tf_fail:
        say("P4 几何对齐探针 = INVALID|跑中出现 TF 失败 %d 次 ⇒ 部分读数不可信（%s）"
            % (len(tf_fail), sorted(set(tf_fail))[:3]))
        rc = 9
    elif not landed:
        say("P4 几何对齐探针 = INVALID|八方位全部未落地 ⇒ 注入面不可用")
        rc = 9
    elif n_sense and not (nA or nB or nC):
        say("P4 几何对齐探针 = FAIL|失因层＝**传感器/插件面**：%d 个方位贴箱球内原始点为 0 ⇒ 与 faster-lio 无关" % n_sense)
        rc = 1
    elif nA and not (n_sense or nB or nC):
        say("P4 几何对齐探针 = FAIL|失因环节＝**A preprocess**：原始有、pre-voxel 没有（%d 方位）" % nA)
        rc = 1
    elif nB and not (n_sense or nA or nC):
        say("P4 几何对齐探针 = FAIL|失因环节＝**B 体素降采样**：pre-voxel 有、voxel 后没有（%d 方位）" % nB)
        rc = 1
    elif nC and not (n_sense or nA or nB):
        say("P4 几何对齐探针 = FAIL|失因环节＝**C 下游 SCAN 建图面**：%d 方位" % nC)
        rc = 1
    elif (n_sense + nA + nB + nC) == 0:
        say("P4 几何对齐探针 = PASS|八方位四层全通 ⇒ 本条件下不复现（R74 的层结论受 z 带缺陷影响，以本件为准）")
        rc = 0
    else:
        say("P4 几何对齐探针 = FAIL|多层并存：传感器缺=%d A=%d B=%d C=%d ⇒ 不合并归因" % (n_sense, nA, nB, nC))
        rc = 1

    move(*HOME)
    if out:
        with open(out, "w") as fh:
            fh.write(json.dumps({"rc": rc, "pose": [pose.get("x"), pose.get("y"), pose.get("z")],
                                 "dist": DIST, "radii": list(RADII), "topics": TOPICS, "meta": meta,
                                 "geom_paths": geom_paths, "declared": DECL,
                                 "landmark_raw_nearest_m": (round(lm[0], 3) if lm else None),
                                 "pos_ctrl": {k: pc[k] for k in LAYERS},
                                 "neg_ctrl": {k: nc[k] for k in LAYERS},
                                 "tf_fail": len(tf_fail), "rows": rows,
                                 "tally": {"sensor": n_sense, "A": nA, "B": nB, "C": nC, "ok": nOk}},
                                ensure_ascii=False, indent=2) + "\n")
        with open(out + ".log", "w") as fh:
            fh.write("\n".join(log) + "\n")
    say("分档自证：判据行首段与 rc 必须同向（PASS↔0／FAIL↔1／INVALID↔9）；本次 rc=%d" % rc)
    return rc


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    sys.exit(main())
