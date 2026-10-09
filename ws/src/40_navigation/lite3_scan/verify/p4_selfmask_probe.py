#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""p4_selfmask_probe.py —— 量「机体自遮挡到底占多大」＋「球形 blind 浪费了多少点」（R82；主理人 2026-10-10 提出换盒）

为什么单独起一件（而不是继续用 p4_boxface_probe）：
  在册那件是**搬箱子**去扫距离（`move()` 调 SetModelState 动的是 obs_5）；主理人这次的口径是
  **「障碍物的位置不变」**⇒ 必须反过来**搬机体**。两件取样几何因此不可互换，本件一字不改那件。
  本件还把原始帧落盘（float16，厘米级足够）⇒ 此后任何过滤方案（盒／盒∪小球／任意外扩）都能离线重算，
  **不再需要开链**——这是把"要不要 90 分钟整批重取"的裁定成本降下来的唯一办法。

三个只出数、不裁决的问题：
  Q1 仿真里雷达会不会打到自己躯干？——数**雷达系原始点**落在躯干盒内的个数。
     依据：插件 `livox_ode_multiray_shape.cpp:38` 只把**传感器所挂 link（rslidar）**的 space id 排掉，
     而 rslidar 的 collision 已按 D-M1-5④ 整块删除；躯干是**另一个 link TORSO** ⇒ 按代码读应该会打到，
     但那是我从 ODE space 机制推的，**必须实测**（不实测就又是"按注释推"，坑43 同族）。
  Q2 现网 `blind=1.0` 的球里，0.56–1.0m 这一层实际有多少点？（体积比 146 倍不等于点数比）
  Q3 换成躯干盒（或盒∪小球）后，同一个障碍物的回波能多留下多少点？

生效剔除点（本件与它同口径）：`pointcloud_preprocess.cc:208`（**C16Handler**，`lidar_type: 4`）
  `if (range < blind_*blind_) continue;`，比的是**雷达系原始 x/y/z** ⇒ 盒在雷达系是常量、不需 TF。

零改点：不动任何源码/配置/门值；**不动 obs_* 世界件位置**；不投 goal、不发 /cmd_vel；
  faster-lio 不起（本件只要传感器原始流＋Gazebo 状态服务）；跑完把机体搬回起始位（朝向也回读后原样还）。
自锁（不过判 INVALID，不降格成 0）：
  ① landmark：obs_1 原位（0.8³、yaw 0.4）在 raw 层 OBB 内 ≥50 点，否则几何口径本身没对准；
  ② 每站用 GetModelState **回读**的实际位姿算世界系（R71 已证指令 yaw 不成立，绝不拿指令值当真）；
  ③ 每帧整帧点数 ≥10000（在册插件 20000 点/包，掉一半说明流不稳）。
`--selftest` 零节点几何自测（T-1…T-6），起工前必须先绿（坑48 处置④）。
rc：0=测量成立 1=站全废 5=selftest 不过 9=前置/自锁不过
"""
import json
import math
import os
import sys
import time

import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from p1_occ_sample import axis                                  # noqa: E402

# ── 机体几何（每条给来源，不用记忆值）────────────────────────────────────────
HALF = (0.210, 0.155, 0.110)      # TORSO 碰撞盒 0.42×0.31×0.22 的半宽　　lite3_planar.urdf.xacro:26/31
LID_OFF_Z = 0.380                 # base_link→rslidar 纯平移、rpy=0　　　 同文件 :52-55
BOX_C = (0.0, 0.0, -LID_OFF_Z)    # ⇒ 躯干盒在**雷达系**的盒心（与机体 yaw/俯仰无关，本件的核心便利）
MARGIN = float(os.environ.get("SM_MARGIN", "0.05"))              # 盒外扩（真机要留给腿与支架）
EPS = 0.01
SPHERE_NOW = 1.0                  # 现网在册值　　　　　faster_lio_outdoor.yaml:32
SPHERE_SIM_YAML = 0.2             # 在册但**不被 m2 链加载**的那份　faster_lio_sim.yaml:38（差异⑥）
SPHERE_COMBO = 0.3                # 组合案的小球（与 render min_raylength 同量级）
BANDS = (0.0, 0.27, 0.56, 1.00, 1.50, 3.00, 1e9)                 # 0.27＝躯干最近面　0.56＝躯干最远角
FILTER_KEYS = ("F1", "F2", "F3", "F4")
OBST = os.environ.get("SM_MODEL", "obs_5")                       # 在册注入体（0.8³、yaw −0.6）
OBST_HALF, OBST_Z, Z_FLOOR = 0.40, 0.40, -0.35
LANDMARK = ("obs_1", 1.5, 1.2, 0.4)                              # 阳性对照原位（沿用 #64 的 landmark）
DIRS = {"face": 0.0, "diag": 45.0}                               # 相对障碍物自身偏航的进近方向
SURF = tuple(float(v) for v in os.environ.get("SM_SURF", "0.35,0.5,0.6,0.7,0.85,1.0,1.3").split(","))
FRAMES = 3
MIN_PTS = 10000
SETTLE_S = 3.0
RAW_TOPIC = os.environ.get("SM_RAW_TOPIC", "/rslidar_points")
OUT_PATH = os.environ.get("SM_OUT")


def filter_legend():
    return {"F1": "球形 blind=%.1f（现网在册，什么都不改）" % SPHERE_NOW,
            "F2": "球形 blind=%.1f（sim yaml 在册值，但那份不被 m2 链加载）" % SPHERE_SIM_YAML,
            "F3": "机体长方体（半宽 %s、外扩 %.2f、盒心 %s，全在雷达系）" % (list(HALF), MARGIN, list(BOX_C)),
            "F4": "长方体 ∪ 小球 %.1f（组合案）" % SPHERE_COMBO}


def in_selfbox(x, y, z):
    """雷达系原始点是否落在躯干盒（含外扩）内。三轴比较，无需 TF、无需机体位姿。"""
    hx, hy, hz = HALF
    cx, cy, cz = BOX_C
    return ((np.abs(np.asarray(x) - cx) <= hx + MARGIN + EPS)
            & (np.abs(np.asarray(y) - cy) <= hy + MARGIN + EPS)
            & (np.abs(np.asarray(z) - cz) <= hz + MARGIN + EPS))


def band_counts(r):
    """按量程分带计数（互斥，和＝总数）。"""
    return [int(np.sum((r >= BANDS[i]) & (r < BANDS[i + 1]))) for i in range(len(BANDS) - 1)]


def world_of(x, y, z, mp):
    """雷达系点 → world：world = R(q)·(p + (0,0,0.38)) + t，q/t 用**回读**的机体位姿。"""
    qx, qy, qz, qw = mp["qx"], mp["qy"], mp["qz"], mp["qw"]
    vx, vy, vz = np.asarray(x), np.asarray(y), np.asarray(z) + LID_OFF_Z
    tx, ty, tz = 2.0 * (qy * vz - qz * vy), 2.0 * (qz * vx - qx * vz), 2.0 * (qx * vy - qy * vx)
    return (vx + (qw * tx + (qy * tz - qz * ty)) + mp["x"],
            vy + (qw * ty + (qz * tx - qx * tz)) + mp["y"],
            vz + (qw * tz + (qx * ty - qy * tx)) + mp["z"])


def obb_mask(x, y, z, cx, cy, cz, yaw, half):
    """目标物自身 OBB（与 #64 同口径：EPS 边界容差＋下界挡地面）。"""
    dx, dy, dz = np.asarray(x) - cx, np.asarray(y) - cy, np.asarray(z) - cz
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx, ly = c * dx - s * dy, s * dx + c * dy
    return (np.abs(lx) <= half + EPS) & (np.abs(ly) <= half + EPS) & (dz >= Z_FLOOR) & (dz <= half + EPS)


def filters(x, y, z, r):
    """四种候选过滤各自的**保留**掩码；键固定为 F1–F4（数值走 filter_legend，避免键名漂移）。"""
    sb = in_selfbox(x, y, z)
    return {"F1": r >= SPHERE_NOW, "F2": r >= SPHERE_SIM_YAML,
            "F3": ~sb, "F4": (~sb) & (r >= SPHERE_COMBO)}


def station_name(dist, dirkey):
    return "s%03d_%s" % (int(round(dist * 100)), dirkey)


def pose_to_mp(st):
    o = st.pose.orientation
    return {"x": st.pose.position.x, "y": st.pose.position.y, "z": st.pose.position.z,
            "qx": o.x, "qy": o.y, "qz": o.z, "qw": o.w}


def yaw_of(mp):
    return 2.0 * math.atan2(mp["qz"], mp["qw"])


def selftest():
    """零节点自测（坑48 处置④：起工前先绿）。"""
    bad = []
    tmp = OUT_PATH if OUT_PATH else "/tmp"

    # T-1 躯干盒必须抓到顶面点与最远角点，且不能吞盒外点
    if not np.all(in_selfbox(np.array([0.0, 0.21]), np.array([0.0, 0.155]), np.array([-0.27, -0.49]))):
        bad.append("T-1a 躯干盒漏掉自身表面点")
    if np.any(in_selfbox(np.array([0.6, 0.0]), np.array([0.0, 0.0]), np.array([-0.38, 0.30]))):
        bad.append("T-1b 躯干盒误吞盒外点（0.6m 外或正上方）")

    # T-2 分带互斥且闭合
    r = np.array([0.10, 0.30, 0.60, 1.20, 2.0, 99.0])
    if sum(band_counts(r)) != len(r):
        bad.append("T-2 分带不闭合 %s" % band_counts(r))

    # T-3 单调性：r=0.9 的盒外点 ⇒ F1 必丢、F2/F3/F4 必留
    k = filters(np.array([0.9]), np.array([0.0]), np.array([0.0]), np.array([0.9]))
    got = tuple(bool(k[f][0]) for f in FILTER_KEYS)
    if got != (False, True, True, True):
        bad.append("T-3 四过滤在 r=0.9 盒外点上的期望 (F,T,T,T)，实得 %s" % (got,))

    # T-4 机体朝向旋转：q=yaw90°、机体在 (1,2,0.15) ⇒ 雷达系点 (0.5,0,0)
    #      先加安装偏移 (0,0,0.38) 得 (0.5,0,0.38)，再绕 z 转 90°（x→y）得 (0,0.5,0.38)，最后加平移
    #      ⇒ world (1.0, 2.5, 0.53)。这一例的期望值我第一次写成 (1.0,1.5,2.35) 是**算错**（把 x→−y
    #      与"先转后加偏移"混了），代码本身没错——自检存在的意义就是把这种错在开链前逼出来。
    mp = {"x": 1.0, "y": 2.0, "z": 0.15, "qx": 0.0, "qy": 0.0,
          "qz": math.sin(math.pi / 4), "qw": math.cos(math.pi / 4)}
    wx, wy, wz = world_of(np.array([0.5]), np.array([0.0]), np.array([0.0]), mp)
    if not (abs(wx[0] - 1.0) < 1e-6 and abs(wy[0] - 2.5) < 1e-6 and abs(wz[0] - 0.53) < 1e-6):
        bad.append("T-4 yaw90 旋转不对：得 (%.3f,%.3f,%.3f) 期望 (1.000,2.500,0.530)" % (wx[0], wy[0], wz[0]))

    # T-5 OBB 口径（与 #64 同口径）：**雷达只能打在箱子的表面上**，所以合法回波的
    #      箱体局部坐标 |l| ≤ 半宽（近面恰在 l_x = +0.40，靠 EPS 才收得进）；而箱子前方 0.9m 的
    #      **空气**（l_x=1.3）与背景墙（l_x=2.6）都必须落在 OBB 外。第一次我把"近面点"写成
    #      了"距心 1.3m 的点"＝把空气当目标，是概念错不是代码错。
    cx, cy, yaw = 5.8, -1.0, -0.6
    for lx, ly, want, tag in ((OBST_HALF, 0.10, True, "T-5a 表面点"),
                              (OBST_HALF + 0.9, 0.0, False, "T-5b 前方空气"),
                              (2.6, 0.0, False, "T-5c 背景墙")):
        fx = cx + lx * math.cos(yaw) - ly * math.sin(yaw)
        fy = cy + lx * math.sin(yaw) + ly * math.cos(yaw)
        got = bool(obb_mask(np.array([fx]), np.array([fy]), np.array([OBST_Z]),
                            cx, cy, OBST_Z, yaw, OBST_HALF)[0])
        if got != want:
            bad.append("%s 期望 in_obb=%s 实得 %s" % (tag, want, got))

    # T-6 float16 落盘回读误差 <2cm（否则离线重算会自己造出假差异）
    fp = os.path.join(tmp, "sm_selftest.npy")
    ref = np.array([[1.234, -5.678, 0.321]])
    np.save(fp, ref.astype(np.float16))
    if float(np.max(np.abs(np.load(fp).astype(np.float64) - ref))) > 0.02:
        bad.append("T-6 float16 回读误差 >2cm")
    os.remove(fp)

    for b in bad:
        print("SELFTEST FAIL: " + b)
    print("P4 自遮挡探针自检 = %s（6 例）" % ("PASS" if not bad else "FAIL"))
    return 0 if not bad else 5


def grab(n_want, topic):
    """收 n_want 帧原始云 → [(x,y,z)]；拿不到返回短列表。"""
    got = []

    def cb(m):
        if len(got) >= n_want:
            return
        x, y, z = axis(m, "x"), axis(m, "y"), axis(m, "z")
        if x is None or len(x) == 0 or y is None or z is None:
            return
        if len(y) != len(x) or len(z) != len(x):
            return
        got.append((np.asarray(x, dtype=np.float64), np.asarray(y, dtype=np.float64),
                    np.asarray(z, dtype=np.float64)))

    sub = rospy.Subscriber(topic, PointCloud2, cb, queue_size=4, buff_size=2 ** 26)
    t0 = time.time()
    while len(got) < n_want and time.time() - t0 < 40:
        rospy.sleep(0.3)
    sub.unregister()
    return got


def one_station(tag, frames, obst_pose, yaw_o):
    """一站的汇总；帧全废返回 None（不降格成 0）。"""
    per, dead = [], 0
    for (x, y, z) in frames:
        if len(x) < MIN_PTS:
            dead += 1
            continue
        r = np.sqrt(x * x + y * y + z * z)
        sb = in_selfbox(x, y, z)
        ww = world_of(x, y, z, obst_pose["robot"])
        om = obb_mask(ww[0], ww[1], ww[2], obst_pose["obs_x"], obst_pose["obs_y"], OBST_Z, yaw_o, OBST_HALF)
        rr = np.sqrt(ww[0] ** 2 + ww[1] ** 2 + ww[2] ** 2)
        keep = filters(x, y, z, r)
        rec = {"n": int(len(x)), "self_in_box": int(np.sum(sb)),
               "self_rmin": float(np.min(r[sb])) if np.any(sb) else None,
               "self_rmax": float(np.max(r[sb])) if np.any(sb) else None,
               "bands": band_counts(r), "obst_in_obb": int(np.sum(om)),
               "obst_by_band": band_counts(rr[om])}
        for fk in FILTER_KEYS:
            rec["keep_" + fk] = int(np.sum(keep[fk] & om))
        per.append(rec)
    if not per:
        print("  站 %-16s 帧全不可用（整帧 <10000 点，废 %d 帧）⇒ 不降格成 0" % (tag, dead))
        return None
    agg = {"frames": len(per), "dead_frames": dead}
    for k in ("n", "self_in_box", "obst_in_obb") + tuple("keep_" + f for f in FILTER_KEYS):
        agg[k] = sum(p[k] for p in per) / len(per)
    agg["bands"] = [sum(p["bands"][i] for p in per) / len(per) for i in range(len(BANDS) - 1)]
    agg["obst_by_band"] = [sum(p["obst_by_band"][i] for p in per) / len(per) for i in range(len(BANDS) - 1)]
    rv = [p for p in per if p["self_rmin"] is not None]
    agg["self_r"] = [min(p["self_rmin"] for p in rv), max(p["self_rmax"] for p in rv)] if rv else [None, None]
    return agg


def main():
    if "--selftest" in sys.argv:
        return selftest()
    if not OUT_PATH:
        print("P4 自遮挡探针 = INVALID|没给 SM_OUT 证据目录 ⇒ 不落盘就没法离线重算")
        return 9
    rospy.init_node("p4_selfmask_probe", anonymous=True)
    get = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    setst = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    try:
        rospy.wait_for_service("/gazebo/get_model_state", timeout=30)
    except Exception:
        print("P4 自遮挡探针 = INVALID|gazebo 状态服务不可用 ⇒ 读不到机体位姿")
        return 9

    def read(name):
        st = get(name, "world")
        if not st.success:
            raise RuntimeError("读不到 %s 的 model state" % name)
        return pose_to_mp(st)

    home = read("lite3_planar")
    obst = read(OBST)
    yaw_o = yaw_of(obst)
    print("机体起始位姿（回读）x=%.3f y=%.3f z=%.3f yaw=%.1f°" % (home["x"], home["y"], home["z"], math.degrees(yaw_of(home))))
    print("注入体 %s 原位 x=%.3f y=%.3f z=%.3f yaw=%.1f°（本件**不动它**）"
          % (OBST, obst["x"], obst["y"], obst["z"], math.degrees(yaw_o)))
    print("躯干盒（雷达系）盒心 %s 半宽 %s 外扩 %.2f；量程带 %s" % (list(BOX_C), list(HALF), MARGIN, list(BANDS[:-1])))

    frames = grab(2, RAW_TOPIC)
    if not frames:
        print("P4 自遮挡探针 = INVALID|%s 收不到原始帧 ⇒ 传感器面就没通" % RAW_TOPIC)
        return 9
    x0, y0, z0 = frames[0]
    ww = world_of(x0, y0, z0, home)
    lm = int(np.sum(obb_mask(ww[0], ww[1], ww[2], LANDMARK[1], LANDMARK[2], OBST_Z, LANDMARK[3], OBST_HALF)))
    print("  landmark obs_1 OBB 内 raw 点=%d（需 ≥50）整帧=%d" % (lm, len(x0)))
    if lm < 50:
        print("P4 自遮挡探针 = INVALID|landmark 不过 ⇒ 世界系换算/几何口径本身没对准，不出 Q1–Q3")
        return 9

    rows = {}
    for dirkey, ddeg in DIRS.items():
        ang = yaw_o + math.radians(ddeg)
        ux, uy = math.cos(ang), math.sin(ang)
        for s in SURF:
            tag = station_name(s, dirkey)
            dc = s + OBST_HALF                                    # 近面 s ⇒ 到箱心 s＋半宽
            setst(ModelState(model_name="lite3_planar", reference_frame="world",
                             pose=Pose(position=Point(obst["x"] + dc * ux, obst["y"] + dc * uy, home["z"]),
                                       orientation=Quaternion(home["qx"], home["qy"], home["qz"], home["qw"]))))
            rospy.sleep(SETTLE_S)
            mp = read("lite3_planar")
            fr = grab(FRAMES, RAW_TOPIC)
            np.save(os.path.join(OUT_PATH, tag + ".npy"), np.stack(fr[0][:3]).astype(np.float16))
            agg = one_station(tag, fr, {"robot": mp, "obs_x": obst["x"], "obs_y": obst["y"]}, yaw_o)
            if agg is None:
                continue
            agg["pose"] = mp
            rows[tag] = agg
            print("  站 %-16s 机体(%7.2f,%7.2f) 整帧=%6.0f 躯干盒内=%5.0f 障碍OBB内=%6.1f "
                  "留点 F1=%6.1f F2=%6.1f F3=%6.1f F4=%6.1f"
                  % (tag, mp["x"], mp["y"], agg["n"], agg["self_in_box"], agg["obst_in_obb"],
                     agg["keep_F1"], agg["keep_F2"], agg["keep_F3"], agg["keep_F4"]))

    if not rows:
        print("P4 自遮挡探针 = FAIL|没有任何一站可用 ⇒ 本次量不出 Q1–Q3（站全废，须查前置，不是跑不过）")
        return 1

    n = len(rows)
    ns = [v["self_in_box"] for v in rows.values()]
    rr = [v["self_r"] for v in rows.values() if v["self_r"][0] is not None]
    q1 = ("有自回波（%d/%d 站盒内非零，均值 %.0f 点，距雷达 %.2f–%.2f m）"
          % (sum(1 for v in ns if v > 0), n, sum(ns) / n,
             min(x[0] for x in rr), max(x[1] for x in rr))) if rr \
        else "无自回波（全部 %d 站、每帧 2 万点里落在躯干盒内的都是 0）⇒ 仿真侧 blind 砍掉的不是自己，是真实障碍" % n
    bd = [sum(v["bands"][i] for v in rows.values()) / n for i in range(len(BANDS) - 1)]
    tn = sum(bd) or 1.0
    f1 = sum(v["keep_F1"] for v in rows.values())
    f2 = sum(v["keep_F2"] for v in rows.values())
    f3 = sum(v["keep_F3"] for v in rows.values())
    f4 = sum(v["keep_F4"] for v in rows.values())
    ob = sum(v["obst_in_obb"] for v in rows.values())
    print("  Q1 仿真里雷达打不打到自己躯干：%s" % q1)
    print("  Q2 整帧点按量程分带（各站均值）：%s" % " ".join(
        "%.2f–%s:%6.0f(%.1f%%)" % (BANDS[i], "∞" if BANDS[i + 1] > 1e8 else "%.2f" % BANDS[i + 1],
                                    bd[i], 100.0 * bd[i] / tn) for i in range(len(BANDS) - 1)))
    print("     ⇒ 纯浪费球壳 0.56–1.00m 里 = %.0f 点/帧（占整帧 %.1f%%）；躯干最近面 0.27m、最远角 0.56m"
          % (bd[2], 100.0 * bd[2] / tn))
    print("  Q3 障碍 OBB 内保留点合计（%d 站）：现网球1.0=%.0f ｜ 球0.2=%.0f ｜ 盒=%.0f ｜ 盒∪球0.3=%.0f"
          % (n, f1, f2, f3, f4))
    print("     传感器本来就看得到 %.0f 点，现网配置留下 %.0f 点 ⇒ **换盒可救回 ×%.2f（%.0f 点）**"
          % (ob, f1, (f3 / f1) if f1 else float("inf"), ob - f1))
    verdict = "P4 自遮挡探针 = PASS|%s；%d 站可用（障碍物位置未动，动的是机体）" % (q1, n)

    # 顺序要紧：**先落 json（数据），再搬回机体（清理）**。上一版把清理排在前面，而清理那行把
    # `twist` 塞进了 `Pose`（twist 其实属 ModelState）⇒ AttributeError 一崩就把 14 站的位姿全丢了，
    # 只留下 14 个 .npy（原始点本身保住了）。记为坑50：**清理动作永远排在数据落盘之后**。
    with open(os.path.join(OUT_PATH, "sm.json"), "w") as fh:
        fh.write(json.dumps({"verdict": verdict, "home": home, "obstacle": obst, "q1": q1,
                             "bands_mean": bd, "keep_sum": {"F1": f1, "F2": f2, "F3": f3, "F4": f4},
                             "obst_sum": ob,
                             "geom": {"half": list(HALF), "lid_off_z": LID_OFF_Z, "margin": MARGIN,
                                      "box_c": list(BOX_C), "bands": list(BANDS)},
                             "filters": filter_legend(), "surf": list(SURF), "dirs": DIRS, "rows": rows},
                            ensure_ascii=False, indent=2) + "\n")
    try:
        setst(ModelState(model_name="lite3_planar", reference_frame="world",
                         pose=Pose(position=Point(home["x"], home["y"], home["z"]),
                                   orientation=Quaternion(home["qx"], home["qy"], home["qz"], home["qw"])),
                         twist=Twist()))
        print("  机体已搬回起始位 (%.2f, %.2f, %.2f)，朝向用回读值原样还（不下 yaw 指令，R71 已证指令 yaw 不成立）"
              % (home["x"], home["y"], home["z"]))
    except Exception as e:
        print("  【注意】搬回起始位失败（%s）：不影响本次结论，但下一件不得以『机体在起始位』当前提"
              % type(e).__name__)
    print(verdict)
    return 0


if __name__ == "__main__":
    sys.exit(main())
