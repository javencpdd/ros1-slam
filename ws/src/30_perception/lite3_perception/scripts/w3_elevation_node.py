#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""w3_elevation_node —— 高程表征件骨架（S1）

形状依据＝部署架构师-小张 R278 落点裁定＋`spike3d_elevation_costmap_design.md` §4。
职责边界（设计件 §4.4）：只做"点云 → 每列高度统计 → 三值栅格"；不做状态估计、不碰 /cmd_vel、零 ros2qnx 引用。
摄取路径按 ag §1 预检② 的向量化形状写死＝四条 bincount，禁逐格 python 循环。
零覆盖列⇒UNKNOWN，且默认分支不得产出 FREE（#46 三态语义在册未销，故此处只收紧不放宽）。
"""
import math
import sys
import time

import numpy as np

UNKNOWN = -1
FREE = 0
BLOCKED = 100
Z_BINS = 32


class ElevGrid(object):
    """每格累加器：计数／z 和／z 平方和／每格 Z_BINS 箱 z 直方图（全向量化）。

    z 箱界＝固定 z_range（跨帧不变）；若随当帧极值浮动，先前累计的箱会被悄悄重划＝同一格两帧同值不同读。
    """

    def __init__(self, resolution, x_min, x_max, y_min, y_max, z_min=-2.0, z_max=2.0):
        self.res = float(resolution)
        self.x_min = float(x_min)
        self.y_min = float(y_min)
        self.nx = int(math.floor((float(x_max) - self.x_min) / self.res))
        self.ny = int(math.floor((float(y_max) - self.y_min) / self.res))
        if self.nx < 1 or self.ny < 1:
            raise ValueError("window spans less than one cell")
        self.z_lo = float(z_min)
        self.z_hi = float(z_max)
        if not self.z_hi > self.z_lo:
            raise ValueError("z_range must be increasing")
        self.span = self.z_hi - self.z_lo
        self.count = np.zeros((self.ny, self.nx), dtype=np.int64)
        self.z_sum = np.zeros((self.ny, self.nx), dtype=np.float64)
        self.z_sq = np.zeros((self.ny, self.nx), dtype=np.float64)
        self.hist = np.zeros((self.ny, self.nx, Z_BINS), dtype=np.int64)
        self.frames = 0
        self.clipped = 0

    def _cells(self, xy):
        col = np.floor((xy[:, 0] - self.x_min) / self.res).astype(np.int64)
        row = np.floor((xy[:, 1] - self.y_min) / self.res).astype(np.int64)
        keep = (col >= 0) & (col < self.nx) & (row >= 0) & (row < self.ny)
        return row[keep], col[keep]

    def add_points(self, xyz):
        if xyz is None or len(xyz) == 0:
            self.frames += 1
            return 0
        xyz = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
        good = np.isfinite(xyz).all(axis=1)
        xyz = xyz[good]
        if len(xyz) == 0:
            self.frames += 1
            return 0
        row, col = self._cells(xyz[:, :2])
        if len(row) == 0:
            self.frames += 1
            return 0
        z = xyz[:, 2]
        flat = (row * self.nx + col).astype(np.int64)
        n_cells = self.ny * self.nx
        self.count += np.bincount(flat, minlength=n_cells)[:n_cells].reshape(self.ny, self.nx)
        self.z_sum += np.bincount(flat, weights=z, minlength=n_cells)[:n_cells].reshape(self.ny, self.nx)
        self.z_sq += np.bincount(flat, weights=z * z, minlength=n_cells)[:n_cells].reshape(self.ny, self.nx)
        zb_raw = ((z - self.z_lo) / self.span * Z_BINS).astype(np.int64)
        self.clipped += int(((zb_raw < 0) | (zb_raw >= Z_BINS)).sum())
        zb = np.clip(zb_raw, 0, Z_BINS - 1)
        hbin = np.bincount(flat * Z_BINS + zb, minlength=n_cells * Z_BINS)
        self.hist += hbin.reshape(n_cells, Z_BINS).reshape(self.ny, self.nx, Z_BINS)
        self.frames += 1
        return len(row)

    def p50(self):
        """每格 z 中位数（由直方图取，精度≈span/Z_BINS）；零覆盖格返回 NaN。"""
        out = np.full((self.ny, self.nx), np.nan, dtype=np.float64)
        edges = self.z_lo + (np.arange(Z_BINS) + 0.5) / Z_BINS * self.span
        total = self.hist.sum(axis=2)
        live = total > 0
        if not live.any():
            return out
        cum = np.cumsum(self.hist, axis=2)
        half = (total + 1) // 2
        idx = (cum >= half[..., None]).argmax(axis=2)
        out[live] = edges[idx[live]]
        return out

    def rms(self):
        cnt = np.maximum(self.count, 1)
        mean = self.z_sum / cnt
        var = np.maximum(self.z_sq / cnt - mean * mean, 0.0)
        out = np.sqrt(var)
        out[self.count == 0] = np.nan
        return out

    def slope(self, surface):
        gx = np.zeros_like(surface)
        gy = np.zeros_like(surface)
        gx[:, 1:] = (surface[:, 1:] - surface[:, :-1]) / self.res
        gy[1:, :] = (surface[1:, :] - surface[:-1, :]) / self.res
        return np.hypot(np.nan_to_num(gx, nan=0.0), np.nan_to_num(gy, nan=0.0))

    def step(self, surface):
        pad = np.full((self.ny + 2, self.nx + 2), np.nan, dtype=np.float64)
        pad[1:-1, 1:-1] = surface
        best = np.zeros((self.ny, self.nx), dtype=np.float64)
        valid = np.isfinite(surface)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy == 0 and dx == 0:
                    continue
                nb = pad[1 + dy:1 + dy + self.ny, 1 + dx:1 + dx + self.nx]
                diff = np.abs(surface - nb)
                best = np.where(np.isfinite(diff), np.maximum(best, diff), best)
        best[~valid] = np.nan
        return best

    def coverage(self):
        covered = int((self.count > 0).sum())
        return 1.0 - float(covered) / float(self.count.size), covered


def cost_map(grid, slope_max_rad, step_max, rough_max):
    """三通道判据（设计件 §4.3）：slope／step／roughness 任一越界⇒BLOCKED；零覆盖⇒UNKNOWN；其余⇒FREE。

    源图只承诺三值 −1/0/100（v5.3 §16.8：trinary_costmap 开启时中间代价不得入契约）。
    三个阈值现均为**未标定施工值（实测锚＝0 处，不得作真机能力上限）**，见同批 elevation_costmap.yaml。
    """
    surface = grid.p50()
    sl = grid.slope(surface)
    st = grid.step(surface)
    rg = grid.rms()
    out = np.full((grid.ny, grid.nx), UNKNOWN, dtype=np.int8)
    live = grid.count > 0
    blocked = live & ((sl > math.tan(slope_max_rad)) | (st > step_max) | (rg > rough_max))
    out[live] = FREE
    out[blocked] = BLOCKED
    return out, surface


def selftest():
    """离线自检（不连 ROS）。★含 #46 负面对照：摘掉 unknown 标注必须被识破。"""
    fails = []
    ok = []
    rng = np.random.default_rng(7)

    def chk(name, cond):
        (ok if cond else fails).append(name)

    flat = np.column_stack([rng.uniform(0, 4, 400), rng.uniform(0, 4, 400), np.zeros(400)])
    g = ElevGrid(0.5, 0.0, 4.0, 0.0, 4.0)
    g.add_points(flat)
    cm, surf = cost_map(g, math.radians(25.0), 0.15, 0.05)
    chk("T0 平地全覆盖⇒无未知列", int((cm == UNKNOWN).sum()) == 0)
    chk("T1 平地⇒全 FREE", int((cm == FREE).sum()) == cm.size)
    chk("T2 覆盖计数守恒", int(g.count.sum()) == 400)

    g2 = ElevGrid(0.5, 0.0, 4.0, 0.0, 4.0)
    g2.add_points(flat)
    wall = np.column_stack([rng.uniform(0, 1, 600), rng.uniform(0, 4, 600), rng.uniform(0.0, 1.2, 600)])
    g2.add_points(wall)
    cm2, _ = cost_map(g2, math.radians(25.0), 0.15, 0.05)
    chk("T3 陡变列⇒出现 BLOCKED", int((cm2 == BLOCKED).sum()) > 0)
    chk("T4 源图只出三值", set(np.unique(cm2).tolist()) <= set([UNKNOWN, FREE, BLOCKED]))

    g3 = ElevGrid(0.5, 0.0, 4.0, 0.0, 4.0)
    g3.add_points(np.array([[1.0, 1.0, 0.0], [float("nan"), 0.5, 0.0], [3.5, 3.5, 0.0]]))
    cm3, _ = cost_map(g3, math.radians(25.0), 0.15, 0.05)
    chk("T5 零覆盖列＝UNKNOWN", int((cm3 == UNKNOWN).sum()) > 0)
    chk("T6 未覆盖处绝不判 FREE", int((cm3 == FREE).sum()) == 2)

    rows, cols = g._cells(flat[:, :2])
    acc = {}
    for r_i, c_i, z_v in zip(rows, cols, flat[:, 2]):
        acc.setdefault((int(r_i), int(c_i)), []).append(float(z_v))
    med = g.p50()
    tol = (g.z_hi - g.z_lo) / Z_BINS + 1e-9
    bad = 0
    for key, vals in acc.items():
        if abs(med[key[0], key[1]] - float(np.median(np.asarray(vals)))) > tol:
            bad += 1
    chk("T7 直方图 P50 与逐格中位数同档（向量化不改语义，容差＝span/Z_BINS）", bad == 0)

    tampered = cm3.copy()
    tampered[tampered == UNKNOWN] = FREE
    chk("T8 #46 负面对照：摘 unknown 后必失未知语义（可被本件识破）",
        int((tampered == UNKNOWN).sum()) == 0 and int((cm3 == UNKNOWN).sum()) > 0)

    g4 = ElevGrid(0.5, 0.0, 4.0, 0.0, 4.0)
    g4.add_points(np.array([[50.0, 50.0, 1.0]]))
    chk("T9 窗口外点不计入（防包络外污染）", int(g4.count.sum()) == 0)

    print("  ok=%d fail=%d" % (len(ok), len(fails)))
    for n in ok:
        print("  [OK]   %s" % n)
    for n in fails:
        print("  [FAIL] %s" % n)
    print("W3_ELEV_SELFTEST PASS=%d FAIL=%d" % (len(ok), len(fails)))
    return 0 if not fails else 5


def main(argv):
    if "--selftest" in argv:
        return selftest()
    import rospy
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String

    rospy.init_node("w3_elevation_node", anonymous=False)
    res = float(rospy.get_param("~elevation/resolution", 0.1))
    x_min = float(rospy.get_param("~elevation/x_min", -22.0))
    x_max = float(rospy.get_param("~elevation/x_max", 22.0))
    y_min = float(rospy.get_param("~elevation/y_min", -12.0))
    y_max = float(rospy.get_param("~elevation/y_max", 12.0))
    slope_max = math.radians(float(rospy.get_param("~elevation/slope_max_deg", 25.0)))
    step_max = float(rospy.get_param("~elevation/step_max_m", 0.15))
    rough_max = float(rospy.get_param("~elevation/roughness_max_m", 0.05))
    nan_warn = float(rospy.get_param("~elevation/nan_warn_ratio", 0.30))
    grid = ElevGrid(res, x_min, x_max, y_min, y_max,
                    z_min=float(rospy.get_param("~elevation/z_min", -2.0)),
                    z_max=float(rospy.get_param("~elevation/z_max", 2.0)))
    pub_grid = rospy.Publisher("/elevation_grid", OccupancyGrid, queue_size=2, latch=True)
    pub_stats = rospy.Publisher("/elevation_stats", String, queue_size=2)

    def on_cloud(msg):
        t0 = time.time()
        n = msg.point_step // 4
        if n <= 3:
            rospy.logwarn("point_step too small: %s", msg.point_step)
            return
        pts = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, n)
        grid.add_points(pts[:, :3].astype(np.float64))
        cm, _surf = cost_map(grid, slope_max, step_max, rough_max)
        ratio, covered = grid.coverage()
        occ = OccupancyGrid()
        occ.header.frame_id = "map"
        occ.header.stamp = msg.header.stamp
        occ.info.resolution = res
        occ.info.width = grid.nx
        occ.info.height = grid.ny
        occ.info.origin.position.x = x_min
        occ.info.origin.position.y = y_min
        occ.data = cm.reshape(-1).tolist()
        pub_grid.publish(occ)
        pub_stats.publish(String(data="uncovered_ratio=%.4f covered=%d/%d frames=%d ms=%.1f warn=%s"
                                 % (ratio, covered, cm.size, grid.frames,
                                    (time.time() - t0) * 1e3, "YES" if ratio > nan_warn else "no")))

    rospy.Subscriber("/rslidar_points", PointCloud2, on_cloud, queue_size=2)
    rospy.loginfo("w3_elevation_node up res=%.3f cells=%dx%d unknown=%d（阈值为未标定施工值）",
                  res, grid.ny, grid.nx, UNKNOWN)
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
