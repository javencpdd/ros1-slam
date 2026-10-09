#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""stairstudy/fixtures.py —— 楼梯通行「第四通道」离线预检的几何与传感模型（方向一 · 落地顺序 (a) 步）

为何存在：G 稿 §5 判别力矩阵断言「坡度与台阶高都分不开『可上楼梯 vs 矮墙/梯侧壁』，能分的是
踏步深度＋周期性」，而这句话到 2026-10-09 为止只有论证、没有数值。本件把它变成可跑的样本。
性质：**纯离线**。零 ROS 节点、零在册端口、零 gazebo/roslaunch、不改任何在册件；
生成的 world 只写进本目录 worlds/，**一律不被任何 launch 引用**（在册 m1_world.world 为红线件，一字不动）。

实体模型：每个场景 =  footprint（x∈[x0,x1], y∈[y0,y1]）上的「上表面高程函数」top_z(x,y)，
材质占据 0 ≤ z ≤ top_z ——足够表达台阶／矮墙／斜坡／侧壁／带堵头的楼梯，且射线求交可以 marches 出来。
"""
import math

WALK_Y = (-0.6, 0.6)          # 可走带宽 1.2m（与在册世界障碍尺度同量级）


def _stairs(riser, tread, n, x0=0.0, y0=WALK_Y[0], width=WALK_Y[1] - WALK_Y[0]):
    xs = x0 + tread * n

    def f(x, y):
        if x < x0 or x >= xs or y < y0 or y >= y0 + width:
            return 0.0
        k = int((x - x0) / tread) + 1          # 第 k 级踏面（1..n）
        return k * min(riser, 0.85)            # 厂商 step_z_max 量级封顶，纯为数值稳定
    return {"kind": "stairs", "riser": riser, "tread": tread, "steps": n,
            "x0": x0, "x1": xs, "y0": y0, "y1": y0 + width, "top_z": f,
            "note": "踏步深 %.2fm 级高 %.2fm 共 %d 级" % (tread, riser, n)}


def _wall(h, thick, x0=0.0, y0=WALK_Y[0], width=WALK_Y[1] - WALK_Y[0]):
    xs = x0 + thick

    def f(x, y):
        if x0 <= x < xs and y0 <= y < y0 + width:
            return h
        return 0.0
    return {"kind": "wall", "h": h, "x0": x0, "x1": xs, "y0": y0, "y1": y0 + width, "top_z": f,
            "note": "整面高 %.2fm 厚 %.2fm 无踏步" % (h, thick)}


def _ramp(run, rise, x0=0.0, y0=WALK_Y[0], width=WALK_Y[1] - WALK_Y[0]):
    xs = x0 + run

    def f(x, y):
        if x0 <= x < xs and y0 <= y < y0 + width:
            return rise * (x - x0) / run
        return 0.0
    return {"kind": "ramp", "run": run, "rise": rise, "x0": x0, "x1": xs, "y0": y0,
            "y1": y0 + width, "top_z": f,
            "note": "连续斜面 包络与楼梯同（run %.2f rise %.2f）无竖直段" % (run, rise)}


def _sidewall(run, rise, x0=0.0, thick=0.10):
    """梯侧壁：z 剖面与斜坡全等（同 run/rise），但 y 向只有一片薄板 ⇒ 上不去也不可过。"""
    y0 = WALK_Y[0]

    def f(x, y):
        if x0 <= x < x0 + run and y0 <= y < y0 + thick:
            return rise * (x - x0) / run
        return 0.0
    return {"kind": "sidewall", "run": run, "rise": rise, "thick": thick, "x0": x0,
            "x1": x0 + run, "y0": y0, "y1": y0 + thick, "top_z": f,
            "note": "侧壁：剖面与全宽斜坡逐点同值，差别只在 y 向厚度 %.2f" % thick}


def _stairs_blocked(riser, tread, n, x0=0.0, y0=WALK_Y[0], width=WALK_Y[1] - WALK_Y[0]):
    """楼梯中间压一根立柱：踏步周期性仍在，但通道被局部打断（判据必须能报「不可过」而不是「可上」）。"""
    base = _stairs(riser, tread, n, x0, y0, width)
    orig = base["top_z"]                    # 先取名再改 base，否则闭包里自己指自己（实测爆栈一次）
    px0, px1 = x0 + tread * (n // 2), x0 + tread * (n // 2) + 0.25
    py0, py1 = y0 + 0.45, y0 + 0.70

    def f(x, y):
        z = orig(x, y)
        if px0 <= x < px1 and py0 <= y < py1:
            return max(z, 0.50)
        return z
    base = dict(base)
    base.update({"kind": "stairs_blocked", "pillar": (px0, px1, py0, py1), "top_z": f,
                 "note": "同几何楼梯＋第 %d 级处一根 0.25m 立柱（高 0.50m）" % (n // 2 + 1)})
    return base


def scenarios():
    """八个体：三档可上楼梯（浅/中/陡）＋两档矮墙＋斜坡＋侧壁＋带堵头楼梯。
    关键配对：stairs_r010_d028 与 ramp 的**包络逐点同量**（rise 0.80／run 2.24）⇒ 分不开就无话可说。"""
    return [
        ("stairs_easy", _stairs(0.06, 0.30, 13)),
        ("stairs_mid", _stairs(0.10, 0.28, 8)),
        ("stairs_steep", _stairs(0.18, 0.22, 5)),
        ("wall_h080", _wall(0.80, 0.20)),
        ("wall_h045", _wall(0.45, 0.20)),
        ("ramp_envelope", _ramp(2.24, 0.80)),
        ("sidewall_envelope", _sidewall(2.24, 0.80)),
        ("stairs_blocked_mid", _stairs_blocked(0.10, 0.28, 8)),
    ]


# --- 传感模型 -----------------------------------------------------------------
# 在册仿真的点云源＝Livox MID360 插件（lite3_planar.urdf.xacro：竖向 50 样本、横向 100 样本、
# 量程 0.1–40m、gaussian stddev=0、sensor pose z=0.38），经 faster-lio 去畸变后以
# /LIO/clouds_lidar 进 SCAN 的 plan_env（见 70_scan.launch:56）。xacro 里竖向写的是 ±π，
# 插件内部按非重复图案解释该范围；这里取 **±90° 均匀 50 线** 作等效覆盖下界
# ——真实 MID360 竖向 ±52°，已被 ±90° 包含 ⇒ 本模型的垂直覆盖只会比真的更宽，结论方向保守。
# HD32_ELEV 是**对照臂**（真机若用 32 线 ±15.5°），用来说「换成窄垂直 FOV 会怎样」。
HD32_ELEV = [i * 1.0 - 15.5 for i in range(32)]      # 对照臂：±15° 级（32 线，1° 间隔）
MID360_ELEV = [-90.0 + i * (180.0 / 49.0) for i in range(50)]
# 方位照 xacro 的 samples=100 忠实取 100 个（每帧 100×50＝5000 点，update_rate=10 ⇒ 5万点/秒）。
# 曾一度改成 360 方位以「与 32 线臂等密」，那反而失真：本件要答的是**在册仿真现在给不给得出**，
# 两臂每帧点数由 precheck 的密度自证行原样打出（5000 级 vs 11520 级，差＜2 倍，比较可用）。
MID360_AZ = [-180.0 + i * 3.6 for i in range(100)]
MID360_MOUNT_Z = 0.38


def cloud_mid360(fx, ox, oy, oz=MID360_MOUNT_Z, noisy=0.0, seed=0):
    """在册仿真雷达近似：100×50 仿球面、安装高 0.38m、量程到 8m（本场景够用）。"""
    import random
    rng = random.Random(seed)
    pts = []
    for a in MID360_AZ:
        for e in MID360_ELEV:
            hit = cast_rays(fx, ox, oy, oz, a, e, max_range=8.0)
            if hit:
                if noisy:
                    hit = (hit[0], hit[1], max(0.0, hit[2] + rng.gauss(0.0, noisy)))
                pts.append(hit)
    return pts


def cast_rays(fx, ox, oy, oz, azim_deg, elev_deg, max_range=6.0, ds=0.01, ground=True):
    """从 (ox,oy,oz) 沿方位/俯角打一条射线，marching 出与实体的首个交点。
    返回 (x,y,z) 命中点或 None（未命中）。实体是「0..top_z 的柱体」，命中条件＝射线高度 ≤ 该处 top_z。
    ground=True 时射线降到 z≤0 记为**地面回波**（返回该点、高度 0）。
    这一条是补的：早先 z<0 即弃＝世界里没有地面，结果所有低角线在够到实体之前就消失，
    实测把 stairs_mid 的 65% 列抹成无回波，那是建模缺陷而不是传感器极限（坑：拿影子当对象）。"""
    ca, ce = math.cos(math.radians(azim_deg)), math.cos(math.radians(elev_deg))
    sa, se = math.sin(math.radians(azim_deg)), math.sin(math.radians(elev_deg))
    dx, dy, dz = ce * ca, ce * sa, se
    s = ds
    while s < max_range:
        x, y, z = ox + dx * s, oy + dy * s, oz + dz * s
        t = fx["top_z"](x, y)
        if t > 0.0 and z <= t + 1e-9:
            return (x, y, z)
        if z <= 0.0:
            return (x, y, 0.0) if ground else None
        s += ds
    return None


def cloud(fx, ox, oy, oz=0.40, azim0=0.0, azim1=360.0, azstep=2.0, noisy=0.0, seed=0):
    """整圈点云（多线）。ox/oy＝机体位置；noisy>0 时按该标准差加高斯噪声（rng 固定 seed，可复算）。"""
    import random
    rng = random.Random(seed)
    pts = []
    a = azim0
    while a <= azim1 + 1e-9:
        for e in HD32_ELEV:
            hit = cast_rays(fx, ox, oy, oz, a, e)
            if hit:
                if noisy:
                    hit = (hit[0], hit[1], max(0.0, hit[2] + rng.gauss(0.0, noisy)))
                pts.append(hit)
        a += azstep
    return pts


def cloud_planar_single_height(fx, ox, oy, oz=0.40, azim0=330.0, azim1=390.0, azstep=0.5):
    """S1 最坏假设：只有一条水平扫描线（单高 z=oz），无 elev 维。
    默认方位窗 330–390°＝正对 −x 侧的实体（机体在实体 −x 侧时 az=0° 指向实体）。"""
    pts = []
    a = azim0
    while a <= azim1 + 1e-9:
        hit = cast_rays(fx, ox, oy, oz, a, 0.0)
        if hit:
            pts.append(hit)
        a += azstep
    return pts


# --- 参数化 world 生成（落地顺序 (a) 的产物本体） --------------------------------
def sdf_model(name, fx, z0=0.0, mx=0.0, my=0.0):
    """把一个场景写成 Gazebo SDF：沿 x 按 0.02m 切片堆 box（对任意 top_z 都成立，含台阶与斜面）。
    注意：本函数**只为离线预检与后续真仿窗口的候选件**服务，产物不被任何在册 launch 引用。"""
    sl, x = [], fx["x0"]
    yc = (fx["y0"] + fx["y1"]) / 2.0
    wdt = fx["y1"] - fx["y0"]
    while x < fx["x1"] - 1e-9:
        h = max(fx["top_z"](x + 1e-6, yc), 0.0)
        if h > 1e-6:
            sl.append((x, h))
        x += 0.02
    body = []
    for i, (sx, h) in enumerate(sl):
        body.append("""        <link name="seg%d">
          <pose>%.3f %.3f %.3f 0 0 0</pose>
          <collision name="c%d"><geometry><box><size>0.02 %.3f %.3f</size></box></geometry></collision>
          <visual name="v%d"><geometry><box><size>0.02 %.3f %.3f</size></box></geometry></visual>
        </link>""" % (i, sx + 0.01 - fx["x0"] + mx, yc + my, h / 2.0 + z0, i, wdt, h, i, wdt, h))
    return """    <model name="%s">
      <static>1</static>
      <pose>%.3f %.3f 0 0 0 0</pose>
%s
    </model>
""" % (name, fx["x0"], 0.0, "".join(body))


WORLD_TMPL = """<?xml version="1.0"?>
<sdf version="1.6">
  <world name="stair_lab">
    <!-- stair_lab_%s —— 由 stairstudy/fixtures.py 生成（离线候选件，**不被任何在册 launch 引用**）
         场景：%s
         请勿与 worlds/m1_world.world 混用：后者是红线件（各跑 world_before/after.sha 对它取指纹）。 -->
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
%s
  </world>
</sdf>
"""


def emit_worlds(dirpath):
    import os
    if not os.path.isdir(dirpath):
        os.makedirs(dirpath)
    out = []
    for name, fx in scenarios():
        w = WORLD_TMPL % (name, fx["note"], sdf_model(name, fx))
        p = os.path.join(dirpath, "stair_lab_%s.world" % name)
        with open(p, "w", encoding="utf-8") as fh:
            fh.write(w)
        out.append((p, w.count("<link name=")))
    return out


if __name__ == "__main__":
    import os, sys
    here = os.path.dirname(os.path.abspath(__file__))
    if "--emit" in sys.argv:
        for p, n in emit_worlds(os.path.join(here, "worlds")):
            print("WROTE %s links=%d" % (os.path.relpath(p, here), n))
    else:
        for name, fx in scenarios():
            print("%-18s kind=%-14s x=[%.2f,%.2f] y=[%.2f,%.2f] zmax=%.2f｜%s"
                  % (name, fx["kind"], fx["x0"], fx["x1"], fx["y0"], fx["y1"],
                     max(fx["top_z"](x * 0.01 + fx["x0"], (fx["y0"] + fx["y1"]) / 2.0)
                         for x in range(int(round((fx["x1"] - fx["x0"]) * 100)) + 1)), fx["note"]))
