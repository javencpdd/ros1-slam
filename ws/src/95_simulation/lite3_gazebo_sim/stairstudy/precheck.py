#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""stairstudy/precheck.py —— 「第四通道（踏步深度＋周期性）」能否从可观测数据算出的离线预检

要回答的一句话（G 稿 §5 的断言，此前只有论证没有数值）：
  「坡度与台阶高分不开『可上楼梯 vs 矮墙/梯侧壁』，能分的是踏步深度＋周期性。」
本件不实现判据，只做**判据可行性预检**：给定 机体距离 × 传感模型 × 栅格分辨率，
逐特征看两实体的读数差有没有余量，并把「数据根本没被看见」单独成一态。

已知简化（写清楚，不当事实用）：地面回波已建模（射线降到 z≤0 记为高度 0 的地面点），
因此可见性拆成两面读：`ret_frac`＝该列有**任何**回波（遮挡/超程面），`surf_frac`＝该列有**实体面**（z>0.05）回波（结构铺开面）；再加 `run_len_m`＝最长连续有面长度。
实体面按无厚度竖直段处理（踏步前沿是一条面而非有宽度的立面），栅格化后会有 1 列的量化展宽。

三态口径：可分（有门可写）／不可分（读数重叠）／前件不成立（该格没回波，INVALID 类，不计入分辨力）。

调用姿势自证：`python3 stairstudy/precheck.py` —— 零 ROS 依赖、**不需要 env 三步**（不 rospack、不起节点）；
只 import numpy 与同目录 fixtures.py。rc：0＝自检全过，5＝自检有 FAIL。
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fixtures as FX                                       # noqa: E402

MOUNT_Z = 0.40                     # 对照臂（32 线）与单高平面用的安装高
MOUNT_Z_MID = FX.MID360_MOUNT_Z     # 在册仿真雷达安装高 0.38（xacro 实测）
FOV_LO_DEG = min(FX.HD32_ELEV)      # 对照臂最低线俯角（−15.5°）
FOV_LO_MID = min(FX.MID360_ELEV)    # 在册臂最低线俯角（−90°，即有一条垂直向下）
POSES = (("d1.5", -1.5), ("d1.0", -1.0), ("d0.6", -0.6))    # 机体 x（实体前缘在 x=0）
RES = (0.05, 0.10, 0.15, 0.30)
KINDS = ("planar", "hdl32", "mid360", "mid360n")   # mid360＝在册仿真臂；hdl32＝真机窄 FOV 对照臂
SEP = 0.50                                                  # 「可分」判据：相对差 ≥ 0.50
XWIN = (0.0, 4.0)             # 公共 x 窗：所有实体都从 x=0 起、最长 3.9m。各按自身 x 跨度建格会让
                              # 0.2m 厚的矮墙只有 4 列，「竖直段 0 vs 9」里就混进了尺寸伪影。
YWIN = (-0.8, 0.8)            # 公共可走带 1.6m：走廊类特征必须在同一 y 窗内量——
                              # 按各自 bbox 归一会把「只占一侧的薄侧壁」读成「占满」，那是假可分性
SURF_Z = 0.05                 # 高于此视作「实体面」；地面回波（z≈0）不得进剖面/走廊/高差三个特征
FACE_THRS = (0.03, 0.05, 0.10)  # 竖直段计数用的固定跳升阈值（候选门值量级，非在册门值）
MIN_SD = 0.02                   # 残差低于此就谈不上周期（连续斜面归零，防噪声自相关冒充结构）


def grid(pts, fx, r):
    """点云 → （每格最大高程 g，是否有回波 hit）。**无回波格一律标 UNKNOWN，不得当 0 高程用**：
    早先版本把空格当 0，于是「剖面从 0 跳到 0.7m」被读成一个 0.7m 的大台阶——那是把盲区的形状当成障碍。"""
    nx = int(round((XWIN[1] - XWIN[0]) / r)) + 1
    ny = int(round((YWIN[1] - YWIN[0]) / r)) + 1
    g = np.zeros((nx, ny))
    hit = np.zeros((nx, ny), dtype=bool)
    if not pts:
        return g, hit
    a = np.asarray(pts, dtype=float)
    ix = np.clip(((a[:, 0] - XWIN[0]) / r).astype(int), 0, nx - 1)
    iy = np.clip(((a[:, 1] - YWIN[0]) / r).astype(int), 0, ny - 1)
    np.maximum.at(g, (ix, iy), a[:, 2])
    hit[ix, iy] = True
    return g, hit


def longest_run(mask):
    best = cur = bidx = 0
    for i, v in enumerate(mask):
        cur = cur + 1 if v else 0
        if cur > best:
            best, bidx = cur, i - cur + 1
    return bidx, best


FEATS = [("slope_deg", "剖面坡度°"), ("max_rise", "级高读数m"), ("faces", "竖直段根数@0.05"),
         ("peri_amp", "周期性强度"), ("peri_lag", "周期长度m"),
         ("width_frac", "有面y列比"), ("h_spread", "同列y向高差极大m"),
         ("ret_frac", "有回波列比"), ("run_len_m", "最长连续有面长度m")]


def feats(g, hit, fx, r):
    n = g.shape[0]
    x = np.arange(n) * r + XWIN[0] + r / 2.0
    surf = hit & (g > SURF_Z)                          # 「实体面」格：地面回波一律排除在外
    colret = hit.any(axis=1)
    colsurf = surf.any(axis=1)
    ret = float(colret.mean())
    sf = float(colsurf.mean())
    width = float(surf.any(axis=0).mean())
    hs = 0.0
    for i in range(n):
        col = g[i][surf[i]]
        if col.size >= 2:
            hs = max(hs, float(col.max() - col.min()))
    j0, ln = longest_run(colsurf)
    z = dict((k, 0.0) for k, _ in FEATS)
    z.update(dict((k, 0.0) for k in ("faces@0.03", "faces@0.05", "faces@0.10")))  # 早退也要有这三键
    z.update({"ret_frac": ret, "surf_frac": sf, "width_frac": width, "h_spread": hs,
              "run_len_m": ln * r, "surf_first": float(x[j0]) if ln else None})
    if ln < 4:
        return z
    xs = x[j0:j0 + ln]
    zs = np.asarray([float(np.max(g[i][surf[i]])) if surf[i].any() else np.nan
                     for i in range(j0, j0 + ln)], dtype=float)
    ok = ~np.isnan(zs)
    if ok.sum() < 4:
        return z
    xs, zs = xs[ok], zs[ok]
    k, b0 = np.polyfit(xs, zs, 1)
    d = np.diff(zs)
    resid = zs - (k * xs + b0)
    faces_t = dict((t, int(np.sum(d > t))) for t in FACE_THRS)
    sd = float(np.std(resid))
    lag = amp = 0.0
    if sd >= MIN_SD and len(zs) >= 6:
        det = resid - resid.mean()
        m = len(zs)
        ac = np.correlate(det, det, "full")[m - 1:]
        if ac[0] > 0:
            lo = max(1, int(round(0.10 / r)))
            hi = min(max(lo + 1, int(round(0.45 / r))), m - 1)
            if hi > lo:
                jj = lo + int(np.argmax(ac[lo:hi + 1]))
                lag, amp = jj * r, float(ac[jj] / ac[0])
    z.update({"slope_deg": float(math.degrees(math.atan(k))), "max_rise": float(np.max(d)),
              "faces": faces_t[0.05],
              "faces@0.03": faces_t[0.03], "faces@0.05": faces_t[0.05], "faces@0.10": faces_t[0.10],
              "peri_lag": lag, "peri_amp": amp, "resid_sd": sd, "run_len": int(len(zs))})
    return z


# --- 传感模型 → 点集 -------------------------------------------------------
def points(kind, fx, ox, azstep=None):
    oy = 0.0
    oz = MOUNT_Z_MID if kind.startswith("mid360") else MOUNT_Z
    if kind == "planar":              # 最坏假设：一条水平扫描线（与 P4 注入体 z=0.40 同族）
        # 机体在实体 −x 侧 ⇒ 正对实体是 az=0°（dx=cos az）。曾误取 150–210°＝背对，
        # 结果 planar 恒 0 点、对照会因「传感器朝反」而假绿（坑29/坑41 要防的零命中判绿）。
        return FX.cloud_planar_single_height(fx, ox, oy, oz, 330.0, 390.0, azstep or 0.25)
    st = azstep or (1.0 if kind == "hdl32" else 2.0)
    if kind == "hdl32":
        return FX.cloud(fx, ox, oy, oz, 0.0, 360.0, st)
    if kind == "mid360":
        return FX.cloud_mid360(fx, ox, oy, oz)
    if kind == "mid360n":
        return FX.cloud_mid360(fx, ox, oy, oz, noisy=0.02, seed=7)
    if kind == "hdl32n":              # 同上线数＋2cm 高度噪声（seed 固定，可复算）
        return FX.cloud(fx, ox, oy, oz, 0.0, 360.0, azstep or 1.0, noisy=0.02, seed=7)
    raise ValueError("未知传感模型 " + kind)


def relgap(fa, fb):
    return dict((k, abs(fa[k] - fb[k]) / (abs(fa[k]) + abs(fb[k]) + 1e-9)) for k, _ in FEATS)


PAIRS = [("stairs_mid", "wall_h080", "可上楼梯 vs 等高矮墙（G 稿主断言）"),
         ("stairs_easy", "wall_h045", "浅台阶 vs 半高矮墙"),
         ("stairs_mid", "ramp_envelope", "楼梯 vs 同包络连续斜坡"),
         ("ramp_envelope", "sidewall_envelope", "斜坡 vs 同剖面薄侧壁（差别只在占不占道）"),
         ("stairs_mid", "stairs_blocked_mid", "楼梯 vs 楼梯中段压立柱（通道被局部打断）")]


NDENSE = {}


def build(fxmap, pose_names=("d1.5",), kinds=KINDS, res=RES, azstep=None):
    """射线只按（场景, 传感, 机体位姿）打一遍；分辨率只做栅格化（曾误按 96 次全周打）。"""
    c = {}
    for pname, ox in POSES:
        if pname not in pose_names:
            continue
        for name, fx in fxmap.items():
            for kd in kinds:
                pts = points(kd, fx, ox, azstep)
                NDENSE[(name, pname, kd)] = len(pts)
                for r in res:
                    g, hit = grid(pts, fx, r)
                    c[(name, pname, kd, r)] = (g, hit, feats(g, hit, fx, r))
    return c


def cell(c, a, b, pose, kd, r):
    fa, fb = c[(a, pose, kd, r)][2], c[(b, pose, kd, r)][2]
    if min(fa["ret_frac"], fb["ret_frac"]) < 0.05:          # 一方基本没有任何回波 ⇒ 前件不成立，不进分辨力面
        return None
    return relgap(fa, fb)


def selftest(fxmap, c, cnear):
    bad = 0

    def ck(name, cond, detail):
        nonlocal bad
        print("[%s] %s ｜ %s" % ("PASS" if cond else "FAIL", name, detail))
        if not cond:
            bad += 1

    F = lambda nm, ps, kd, r: c[(nm, ps, kd, r)][2]        # noqa: E731  读数快捷方式
    ck("T0 实体齐", len(fxmap) == 8, "场景数=%d（应为 8）" % len(fxmap))

    # T1 主断言的承重面：楼梯↔等高矮墙在**在册臂 mid360** 的四个分辨率上都要至少有一个特征拉开 ≥SEP
    #    （run_len＝最长连续有面长度是候选承重特征，故逐档附出，但不要求它独自扛住每一档）
    runs, mx = [], []
    for r in RES:
        rr = cell(c, "stairs_mid", "wall_h080", "d1.5", "mid360", r)
        runs.append(0.0 if rr is None else rr["run_len_m"])
        mx.append(0.0 if rr is None else max(rr.values()))
    ck("T1 主断言有数值支撑（在册臂四档全可分）", all(v >= SEP - 1e-9 for v in mx),
       "最大相对差逐档=%s；run_len 逐档=%s（阈 %.2f）"
       % (["%.2f" % v for v in mx], ["%.2f" % v for v in runs], SEP))

    # T2 阴性对照：级高→1mm 的「墙式楼梯」必须丢掉竖直段与周期性（否则 T1 的绿是空的）
    fake = FX._stairs(0.001, 0.28, 8)
    g, h = grid(points("mid360", fake, -1.5), fake, 0.05)
    ff = feats(g, h, fake, 0.05)
    fm = F("stairs_mid", "d1.5", "mid360", 0.05)
    ck("T2 级高归零则特征归零", ff["faces"] < fm["faces"] and ff["run_len_m"] <= fm["run_len_m"],
       "faces %d→%d、run_len %.2f→%.2f" % (fm["faces"], ff["faces"], fm["run_len_m"], ff["run_len_m"]))

    # T3 阈值建议表的算术面：每踏步占几列＝tread/r，不足 2 列就无从测踏步深度
    cpr_fine, cpr_coarse = 0.30 / 0.05, 0.30 / 0.30
    ck("T3 每踏步列数（0.05 档≥5 列、0.30 档≤1 列）",
       cpr_fine >= 5 and cpr_coarse <= 1, "浅台阶 0.30m 踏步：0.05 档 %.1f 列 / 0.30 档 %.1f 列"
       % (cpr_fine, cpr_coarse))

    # T4 前件不成立对照：单高平面（一条线）几乎给不出结构，但必须仍有阳性读数
    fp = F("stairs_easy", "d1.5", "planar", 0.05)
    f1 = F("stairs_easy", "d1.5", "mid360", 0.05)
    ck("T4 单高平面无结构面（且须仍有阳性）", 0.0 < fp["surf_frac"] < f1["surf_frac"],
       "浅台阶 surf 平面=%.2f vs 在册臂=%.2f" % (fp["surf_frac"], f1["surf_frac"]))

    # T5 确定性：同 seed 同参必同值（噪声臂不得靠运气）
    ga, _ = grid(points("mid360n", fxmap["stairs_mid"], -1.5), fxmap["stairs_mid"], 0.05)
    gb, _ = grid(points("mid360n", fxmap["stairs_mid"], -1.5), fxmap["stairs_mid"], 0.05)
    ck("T5 同 seed 可复算", bool(np.array_equal(ga, gb)), "两算栅格逐元全等")

    # T6 不可分性也得是可复现的结论：堵头那一对在各臂各档都不得被"侥幸分开"
    sep = [cell(c, "stairs_mid", "stairs_blocked_mid", "d1.5", kd, r) is not None
           and max(cell(c, "stairs_mid", "stairs_blocked_mid", "d1.5", kd, r).values()) >= SEP
           for kd in ("mid360", "hdl32") for r in (0.05, 0.10, 0.15)]
    ck("T6 堵头不可分且稳定复现（登记为需语义通道＝方向四）", all(not v for v in sep),
       "八格全不可分=%s（若某格忽然可分，先怀疑特征伪影而不是新能力）" % all(not v for v in sep))

    # T7 竖直段计数对阈值敏感 ⇒ 不能单独作承重特征；承重位由 run_len/坡度承担（这里把敏感性量出来给小张看）
    sens = [fm["faces@0.03"], fm["faces@0.05"], fm["faces@0.10"]]
    ck("T7 竖直段计数随阈值单调（故不得单独承重）",
       sens[0] >= sens[1] >= sens[2], "faces@0.03/0.05/0.10 = %d/%d/%d" % tuple(sens))

    # T8 在册仿真图案在楼梯方向上**更稀**：100×50 全向把竖向间隔摊到 3.67°，
    #    落在楼梯立面仰角带内的只有约 8 条线；32 线臂是 1° 间隔 ⇒ 结构面读数反而更密。
    #    （原假设「宽垂直 FOV 就看得更多」被实测推翻，方向记录在案，不改口径粉饰）
    worse = []
    for nm in ("stairs_easy", "stairs_mid", "stairs_steep"):
        a, b = F(nm, "d1.5", "mid360", 0.05), F(nm, "d1.5", "hdl32", 0.05)
        worse.append(a["surf_frac"] < b["surf_frac"])
    ck("T8 仿真图案在楼梯立面上比 32 线更稀（保真度缺口）", all(worse),
       "surf 三梯逐只：在册臂 vs 32线 = %s" % ["%s %.2f/%.2f" % (nm, F(nm, "d1.5", "mid360", 0.05)["surf_frac"],
                                                                 F(nm, "d1.5", "hdl32", 0.05)["surf_frac"])
                                                for nm in ("stairs_easy", "stairs_mid", "stairs_steep")])

    # T9 自相关不得在短序列上冒充周期：可见长度不足两个踏步时，peri_amp 必须待在小数区
    tREAD = 0.28
    viol = []
    for kd in ("mid360", "hdl32"):
        for r in RES:
            for nm in ("stairs_easy", "stairs_mid", "stairs_steep"):
                f = c[(nm, "d1.5", kd, r)][2]
                if f["run_len_m"] < 2 * tREAD and f["peri_amp"] >= 0.5:
                    viol.append("%s/%s@%.2f run=%.2f peri=%.2f" % (nm, kd, r, f["run_len_m"], f["peri_amp"]))
    ck("T9 短序列自相关不冒充周期", not viol,
       "违规格=%d（可见长度＜2 个踏步却报出 peri_amp≥0.5 即为伪峰）%s" % (len(viol), "; ".join(viol[:3])))
    return bad


def main():
    fxmap = dict(FX.scenarios())
    c = build(fxmap, pose_names=("d1.5",))
    cnear = build(fxmap, pose_names=("d1.0", "d0.6"), kinds=("mid360",), res=(0.10, 0.30))

    print("=== 射线密度自证（每扫描落在公共窗内的命中点数；两臂若差＞2 倍，则配对差不得归因于视场形状）===")
    for kd in KINDS:
        print("  %-8s %s" % (kd, "  ".join("%s=%d" % (nm, NDENSE[(nm, "d1.5", kd)])
                                          for nm in ("stairs_easy", "stairs_mid", "wall_h080"))))
    print("\n=== 分辨力矩阵 @d1.5（+＝至少一特征相对差≥%.2f；-＝全重叠；n/a＝前件不成立）===" % SEP)
    print("配对".ljust(31) + "".join("  %s@%.2f" % (kd, r) for kd in KINDS for r in RES))
    for a, b, _ in PAIRS:
        row = []
        for kd in KINDS:
            for r in RES:
                rr = cell(c, a, b, "d1.5", kd, r)
                row.append(" %s" % ("n/a" if rr is None else ("+" if max(rr.values()) >= SEP else "-")))
        print(("%s↔%s" % (a, b)).ljust(31) + "".join(row))

    print("\n=== 可分特征明细（在册臂 mid360 @0.05 @d1.5）===")
    for a, b, why in PAIRS:
        fa, fb = c[(a, "d1.5", "mid360", 0.05)][2], c[(b, "d1.5", "mid360", 0.05)][2]
        rr = relgap(fa, fb)
        win = ["%s %.3f↔%.3f(差%.2f)" % (nm, fa[k], fb[k], rr[k]) for k, nm in FEATS if rr[k] >= SEP]
        print("%s ↔ %s｜%s\n   可分：%s"
              % (a, b, why, "；".join(win) if win else "（无 ⇒ 现特征不可分）"))

    print("\n=== 可见性面（ret＝有回波列比；surf＝有实体面列比；run＝最长连续有面长度）===")
    for name in ("stairs_easy", "stairs_mid", "stairs_steep", "wall_h080"):
        for kd in ("planar", "hdl32", "mid360"):
            line = []
            for r in (0.05, 0.10, 0.15, 0.30):
                f = c[(name, "d1.5", kd, r)][2]
                line.append("r%.2f ret=%.2f surf=%.2f run=%.2fm faces=%d peri=%.2f"
                            % (r, f["ret_frac"], f["surf_frac"], f["run_len_m"],
                               f["faces"], f["peri_amp"]))
            print("  %-12s %-7s %s" % (name, kd, " ｜ ".join(line)))

    print("\n=== 距离面（在册臂 mid360@0.10）：走近到多少米才有踏步结构 ===")
    for name in ("stairs_easy", "stairs_mid", "stairs_steep"):
        row = []
        for pn, ox in POSES:
            src = c if (name, pn, "mid360", 0.10) in c else cnear
            if (name, pn, "mid360", 0.10) not in src:
                continue
            f = src[(name, pn, "mid360", 0.10)][2]
            row.append("%s ret=%.2f surf=%.2f run=%.2fm faces=%d peri=%.2f slope=%.1f"
                       % (pn, f["ret_frac"], f["surf_frac"], f["run_len_m"],
                          f["faces"], f["peri_amp"], f["slope_deg"]))
        print("  %-12s %s" % (name, " ｜ ".join(row)))

    print("\n=== 视场几何（两臂并列；安装高：在册臂 %.2fm、对照臂 %.2fm）===" % (MOUNT_Z_MID, MOUNT_Z))
    for tag, mz, elevs in (("在册臂 MID360 近似（50 竖向 ±90°、100 方位）", MOUNT_Z_MID, FX.MID360_ELEV),
                           ("对照臂 32 线（1° 竖向间隔 ±15.5°）", MOUNT_Z, FX.HD32_ELEV)):
        flo = min(elevs)
        band = [e for e in elevs if abs(e) <= 15.5]
        step_deg = (max(elevs) - min(elevs)) / (len(elevs) - 1)
        if -flo >= 60.0:                       # 近乎垂直向下有线 ⇒ 没有地面盲区，"距离带"这种说法不适用
            print("  %s：最低线 %.0f° ⇒ **无地面盲区**（机体脚下就有线）；"
                  "但竖向间隔 %.2f° ⇒ 楼梯立面所在的 ±15.5° 仰角带内只有 %d 条线"
                  % (tag, flo, step_deg, len(band)))
            continue
        tg = math.tan(math.radians(-flo))
        dg = mz / tg
        print("  %s：最低线 %.1f° ⇒ 地面盲区半径 d_ground=%.2fm；±15.5° 带内 %d 条线（间隔 %.1f°）"
              % (tag, flo, dg, len(band), step_deg))
        for h in (0.06, 0.10, 0.18, 0.30):
            print("      踏步高 %.2fm ⇒ 站距梯脚 %.2f–%.2fm（带宽 %.2fm）才看得见这级立面；更近＝整锥从其上方越过"
                  % (h, max(0.0, dg - h / tg), dg, h / tg))
    print("\n=== 阈值建议表（供 A8/O-8/C1/C4 输入，**不是门值**）===")
    print("  在册参照：SCAN 栅格 grid_map/resolution=0.05（30_3rd/plan_manage/launch/advanced_param.xml:51）；"
          "costmap resolution=0.1（40_navigation/lite3_navigation/config/costmap_common.yaml:14）")
    print("  两轴不得混谈：踏步**深度**（水平周期）下限＝2 列＝2r；台阶**高度**（竖直跳升）下限＝max(栅格量化 r, 3σ=0.06)")
    for r in RES:
        tag = "  ←＝SCAN 现值" if abs(r - 0.05) < 1e-9 else ("  ←＝costmap 现值" if abs(r - 0.1) < 1e-9 else "")
        print("  r=%.2f%s ⇒ 踏步深≥%.2fm 才可测｜级高≥%.2fm 才可测｜"
              "§4.3 未标定施工值（台阶高 0.15m）在本档%s｜该档可测的最浅楼梯踏步深＝%.2fm"
              % (r, tag, 2 * r, max(r, 0.06), "判得出" if 0.15 >= max(r, 0.06) else "判不出", 2 * r))
    print("\n=== 自检 ===")
    bad = selftest(fxmap, c, cnear)
    print("PRECHECK SELFTEST: FAIL=%d" % bad)
    return 0 if bad == 0 else 5


if __name__ == "__main__":
    sys.exit(main())
