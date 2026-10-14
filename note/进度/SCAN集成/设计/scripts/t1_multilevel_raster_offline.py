#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# t1_multilevel_raster_offline.py —— 离线「多高度列」栅格化器（T1；主理人 A47 答"丙：先证形状"获准）
#
# 性质：本件只出**表征**（每格一列 z 板），★不出代价映射、不出"可上/不可上"判据
#       ——三通道映射与 costmap 投影属已搁置面（S2 线）与消费时刻之后的事，均不在本件内。
# 边界：零窗、零编译、不触 costmap_2d 插件链、不新建包；★本件与它的输出话题名一律不用 `/elevation_grid`
#       （v4.3 X1 纯载体禁令：多层不得复用该载体）。
# 分档：本件产出＝**甲-离**（离线真数据读数），★不得顶替甲-活（05 追加三十二）。
# 参数依据：分辨率 0.10 有在册依据（spike3d §4.3 那句「resolution=0.1（与 sim_map_bridge/static 层同格）」）；
#           z 窗 -0.45 / +0.75 与小美给的 z 板厚 0.10 ＝**丙档设计值**（由在册雷达高 38cm 推导），本件不称实测。
# P8：计划书未定稿、现行以 v5.3（含 R294 收编段）为准、本件不是终稿。

import argparse
import sys
import time

import numpy as np

UNKNOWN, FREE, BLOCKED = -1, 0, 100   # #46 三态在册常量（本件只用 UNKNOWN/覆盖两态，不产出 FREE）
RES = 0.10
Z_MIN, Z_MAX = -0.45, 0.75
Z_SLAB = 0.10


# ---------- 输入面：PCD 读取（binary 与 ascii 两种 DATA）----------
def read_pcd(path):
    """返回 (N,4) 的 x,y,z,intensity float32；纯 numpy，不依赖 open3d/pcl（实测二者未装）。"""
    fields, sizes, types, counts = None, None, None, None
    width = height = npoints = None
    data_mode = None
    with open(path, 'rb') as fh:
        while True:
            line = fh.readline()
            if not line:
                raise ValueError('PCD 头未收口：没读到 DATA 行')
            key = line.split(0x20.to_bytes(1, 'big'))[0].decode('ascii', 'ignore').strip().upper()
            val = line.decode('ascii', 'ignore').strip()
            if key == 'FIELDS':
                fields = val.split()[1:]
            elif key == 'SIZE':
                sizes = [int(v) for v in val.split()[1:]]
            elif key == 'TYPE':
                types = val.split()[1:]
            elif key == 'COUNT':
                counts = [int(v) for v in val.split()[1:]]
            elif key == 'WIDTH':
                width = int(val.split()[1])
            elif key == 'HEIGHT':
                height = int(val.split()[1])
            elif key == 'POINTS':
                npoints = int(val.split()[1])
            elif key == 'DATA':
                data_mode = val.split()[1].lower()
                break
        offset = fh.tell()
    counts = counts or [1] * len(fields)
    if any(c != 1 for c in counts):
        raise ValueError('本件只支持 COUNT 全 1 的点云（实测 lite3.pcd 即如此）；COUNT=%r' % (counts,))
    names = [f.lower() for f in fields]
    for need in ('x', 'y', 'z'):
        if need not in names:
            raise ValueError('PCD 缺字段 %r，实际字段=%r' % (need, names))
    typech = {'F': 'f', 'U': 'u', 'I': 'i'}
    dt = np.dtype({'names': names, 'formats': ['%s%d' % (typech[t.upper()], s) for t, s in zip(types, sizes)]})
    n = npoints if npoints else int(width) * int(height or 1)
    if data_mode == 'binary':
        raw = np.fromfile(path, dtype=dt, count=n, offset=offset)
        if raw.size != n:
            raise ValueError('读到的点数 %d ≠ 声明的 POINTS %d' % (raw.size, n))
    elif data_mode == 'ascii':
        raw = np.loadtxt(path, dtype=dt, skiprows=offset, max_rows=n, comments='')
    else:
        raise ValueError('本件只支持 DATA=binary/ascii，实测＝%r' % data_mode)
    cols = [raw[f].astype(np.float64) for f in ('x', 'y', 'z')]
    if 'intensity' in names:
        cols.append(raw['intensity'].astype(np.float64))
    else:
        cols.append(np.zeros(n, dtype=np.float64))
    return np.column_stack(cols), {'fields': names, 'mode': data_mode, 'declared': n}


# ---------- 独立显式一步：占位零点剔除（小张重锚句：★必须在摄取阶段、先于任何统计）----------
def drop_placeholder_zeros(pts):
    """返回 (清洗后的点, 被剔的零点数)。这一步不得靠 blind 球形状顺手删。"""
    iszero = (pts[:, 0] == 0.0) & (pts[:, 1] == 0.0) & (pts[:, 2] == 0.0)
    removed = int(iszero.sum())
    return pts[~iszero], removed


# ---------- 表征：多高度列栅格 ----------
def rasterize(pts, res=RES, z_min=Z_MIN, z_max=Z_MAX, slab=Z_SLAB, x_lo=None, x_hi=None, y_lo=None, y_hi=None):
    """每 xy 格存一个 z 板位掩码（bit k ＝第 k 个 z 板被占）。返回 dict。"""
    nz = int(round((z_max - z_min) / slab))
    nslabs = 1 << nz
    popcount = np.array([bin(i).count('1') for i in range(nslabs)], dtype=np.int16)
    if pts.shape[0] == 0:
        raise ValueError('没有点可栅格化')
    x_lo = float(pts[:, 0].min()) if x_lo is None else x_lo
    x_hi = float(pts[:, 0].max()) if x_hi is None else x_hi
    y_lo = float(pts[:, 1].min()) if y_lo is None else y_lo
    y_hi = float(pts[:, 1].max()) if y_hi is None else y_hi
    nx = int(np.floor((x_hi - x_lo) / res)) + 1
    ny = int(np.floor((y_hi - y_lo) / res)) + 1
    cx = np.floor((pts[:, 0] - x_lo) / res).astype(np.int64)
    cy = np.floor((pts[:, 1] - y_lo) / res).astype(np.int64)
    cz = np.floor((pts[:, 2] - z_min) / slab).astype(np.int64)
    inside = (cx >= 0) & (cx < nx) & (cy >= 0) & (cy < ny) & (cz >= 0) & (cz < nz)
    cx, cy, cz = cx[inside], cy[inside], cz[inside]
    cell = cy * nx + cx
    n_out_of_z = int(pts.shape[0] - inside.sum())
    pairs = np.unique(np.column_stack([cell, cz]), axis=0)   # 去重：一格一板只算一次
    pcell, psz = pairs[:, 0], pairs[:, 1]
    mask = np.zeros(nx * ny, dtype=np.int64)
    np.add.at(mask, pcell, 1 << psz)
    cov = np.zeros(nx * ny, dtype=np.int16)
    np.add.at(cov, cell, 1)                                   # 每格落点数（剔零之后）
    nslab_occ = popcount[mask]
    low = np.where(mask > 0, _lowest_bit(mask), -1).astype(np.int16)
    high = np.where(mask > 0, _highest_bit(mask, nz), -1).astype(np.int16)
    contiguous = (mask > 0) & (nslab_occ == (high - low + 1))
    multi = (nslab_occ >= 2) & ~contiguous                    # ★中间有空板 ⇒ 多层／桥洞候选
    susp = (low > 0) & (mask > 0)                             # ★最下层板之上还有底空 ⇒ 悬挑候选
    return {'res': res, 'z_min': z_min, 'z_max': z_max, 'slab': slab, 'nz': nz,
            'nx': nx, 'ny': ny, 'x_lo': x_lo, 'x_hi': x_hi, 'y_lo': y_lo, 'y_hi': y_hi,
            'mask': mask, 'covered': cov, 'n_slab': nslab_occ, 'low': low, 'high': high,
            'multi_layer': multi, 'suspended': susp,
            'n_points_in': int(pts.shape[0]), 'n_points_used': int(inside.sum()),
            'n_points_out_of_z': n_out_of_z}


def _lowest_bit(mask):
    m = mask.astype(np.int64, copy=True)
    out = np.zeros_like(m)
    hit = m > 0
    lsb = np.where(hit, m & (-m), 0)
    out[hit] = np.log2(lsb[hit].astype(np.float64)).astype(np.int64)
    return out


def _highest_bit(mask, nz):
    m = mask.astype(np.int64, copy=True)
    hit = m > 0
    idx = np.full_like(m, -1)
    for k in range(nz - 1, -1, -1):
        idx = np.where(hit & ((m >> k) & 1).astype(bool), k, idx)
    return idx


# ---------- 自测（合成分档：每条都带"怎么当场验"）----------
def _cloud(pairs):
    return np.array(pairs, dtype=np.float64)


def selftest():
    ok = fail = 0
    lines = []

    def check(tag, cond, detail=''):
        nonlocal ok, fail
        if cond:
            ok += 1
            lines.append('  [PASS] %s %s' % (tag, detail))
        else:
            fail += 1
            lines.append('  [FAIL] %s %s' % (tag, detail))

    # T-a 单层地面 ⇒ 无多层、无悬挑
    g = np.column_stack([np.repeat(np.arange(0.0, 1.0, 0.1), 3),
                         np.tile(np.arange(0.0, 1.0, 0.1), 3),
                         np.concatenate([np.full(3, -0.40)] * 10)])
    r = rasterize(_after_zero(g))
    check('T-a 单层地面 multi_layer=0', int(r['multi_layer'].sum()) == 0, '格数=%d' % int((r['covered'] > 0).sum()))
    check('T-a 单层地面 suspended=0', int(r['suspended'].sum()) == 0)

    # T-b 悬挑：同一格只有高位板（下面空）
    b = _cloud([[0.5, 0.5, 0.30, 0], [0.5, 0.5, 0.35, 0], [0.55, 0.5, 0.30, 0]])
    r = rasterize(b)
    check('T-b 悬挑 suspended>0', int(r['suspended'].sum()) > 0, '最低板>0 的格=%d' % int(r['suspended'].sum()))

    # T-c 桥洞／双层：同一格底＋顶、中间空 ⇒ multi_layer
    c = _cloud([[1.5, 1.5, -0.40, 0], [1.5, 1.5, -0.20, 0], [1.5, 1.5, 0.60, 0]])
    r = rasterize(c)
    check('T-c 双层 multi_layer>0', int(r['multi_layer'].sum()) > 0, '中间空板的格=%d' % int(r['multi_layer'].sum()))
    check('T-c 三值不塌陷：有覆盖格数==格数>0', int((r['covered'] > 0).sum()) >= 1)

    # T-d 占位零点被独立一步剔掉（复现 #176 的塌格形状）
    mixed = np.vstack([_cloud([[0.0, 0.0, 0.0, 0]] * 500), _cloud([[2.5, 2.5, 0.10, 0]])])
    cleaned, removed = drop_placeholder_zeros(mixed)
    check('T-d 剔零计数 removed=500', removed == 500, 'removed=%d' % removed)
    r = rasterize(cleaned)
    idx = _cell_of(r, 0.0, 0.0)
    inb = 0 <= idx < r['nx'] * r['ny']
    check('T-d 剔零后 (0,0) 处不成格（零点不再撑出栅格范围）', (not inb) or int(r['covered'][idx]) == 0,
          '栅格=%dx%d 总格=%d 覆盖格=%d' % (r['nx'], r['ny'], r['nx'] * r['ny'], int((r['covered'] > 0).sum())))
    check('T-d 剔零后覆盖格==1（只剩那个真点）', int((r['covered'] > 0).sum()) == 1)

    # M1 阴性对照：★不剔零 ⇒ 上面两条必须失效（证明剔除步不是白计、且零点会自己撑出一格）
    r2 = rasterize(mixed)
    cell00b = _cell_of(r2, 0.0, 0.0)
    check('M1 关掉剔零后 (0,0) 格被塌占（阴性对照须成立）', 0 <= cell00b < r2['nx'] * r2['ny'] and int(r2['covered'][cell00b]) > 0,
          '该格落点=%d' % (int(r2['covered'][cell00b]) if 0 <= cell00b < r2['nx'] * r2['ny'] else -1))
    check('M1 关掉剔零后覆盖格==2（虚高一格）', int((r2['covered'] > 0).sum()) == 2, '实=%d' % int((r2['covered'] > 0).sum()))
    check('M1 关掉剔零后总点数虚高 500', r2['n_points_in'] - cleaned.shape[0] == 500)

    # M2 阴性对照：判据换成"只要有板就算多层" ⇒ 单层地面必须误判 ⇒ 证明 multi_layer 有分辨力
    naive = (rasterize(_after_zero(g))['n_slab'] >= 1)
    check('M2 朴素判据在单层地面上误判（故判据须要"中间空板"这一件）', int(naive.sum()) > 0 and int(rasterize(_after_zero(g))['multi_layer'].sum()) == 0)

    # T-e z 窗外的点被计数、不静默丢
    e = _cloud([[3.5, 3.5, 5.0, 0], [3.5, 3.5, -9.0, 0], [3.5, 3.5, 0.0, 1]])
    r = rasterize(np.vstack([e, _cloud([[0.1, 0.1, 0.0, 0]])]))
    check('T-e 窗外点数 n_points_out_of_z=2', r['n_points_out_of_z'] == 2, 'out=%d used=%d' % (r['n_points_out_of_z'], r['n_points_used']))

    # T-f 空输入必须报错而不是静默产出空面
    try:
        rasterize(np.zeros((0, 4)))
        check('T-f 空点云须抛错', False)
    except ValueError:
        check('T-f 空点云须抛错', True)

    lines.append('  合计 PASS=%d FAIL=%d' % (ok, fail))
    return ok, fail, lines


def _after_zero(pts):
    cleaned, _ = drop_placeholder_zeros(pts)
    return cleaned


def _cell_of(r, x, y):
    cx = int(np.floor((x - r['x_lo']) / r['res']))
    cy = int(np.floor((y - r['y_lo']) / r['res']))
    return cy * r['nx'] + cx


def main(argv=None):
    ap = argparse.ArgumentParser(description='离线多高度列栅格化器（T1，零窗）')
    ap.add_argument('--selftest', action='store_true')
    ap.add_argument('--pcd')
    ap.add_argument('--res', type=float, default=RES)
    ap.add_argument('--min-z', type=float, default=Z_MIN)
    ap.add_argument('--max-z', type=float, default=Z_MAX)
    ap.add_argument('--slab', type=float, default=Z_SLAB)
    ap.add_argument('--anchor', choices=['given', 'ground'], default='given',
                    help="given＝照 --min-z/--max-z（★那组是雷达系值，地面≈-0.38）；ground＝把同一 z 宽度下沿贴到本件自报的地面分位（★map 系必用此档）")
    ap.add_argument('--ground-pct', type=float, default=5.0, help='地面代理分位（%%），默认 P5')
    ap.add_argument('--quiet', action='store_true')
    a = ap.parse_args(argv)
    rc = 0
    if a.selftest:
        ok, fail, lines = selftest()
        for ln in lines:
            print(ln)
        print('T1_SELFTEST PASS=%d FAIL=%d rc=%d' % (ok, fail, 0 if fail == 0 else 1))
        rc = 0 if fail == 0 else 1
    if a.pcd:
        t0 = time.time()
        pts, meta = read_pcd(a.pcd)
        raw_total = int(pts.shape[0])
        pts, removed = drop_placeholder_zeros(pts)
        zpct = np.percentile(pts[:, 2], [0, a.ground_pct, 50, 95, 99, 100])
        span = a.max_z - a.min_z
        zmin, zmax = (a.min_z, a.max_z) if a.anchor == 'given' else (float(zpct[1]), float(zpct[1]) + span)
        r = rasterize(pts, res=a.res, z_min=zmin, z_max=zmax, slab=a.slab)
        cov = int((r['covered'] > 0).sum())
        print('T1_ZDIST z 分位（单位：m，map 系）MIN=%.2f P%.0f=%.2f P50=%.2f P95=%.2f P99=%.2f MAX=%.2f'
              % (zpct[0], a.ground_pct, zpct[1], zpct[2], zpct[3], zpct[4], zpct[5]))
        print('T1_ZWIN 锚定=%s ⇒ 实开窗 [%.2f, %.2f]（宽度 %.2f m；★窗口宽度沿用设计值，只换下沿锚；雷达系那组值直接用在 map 系会把大部分点判成窗外）'
              % (a.anchor, zmin, zmax, zmax - zmin))
        print('T1_PCD 文件=%s 声明点数=%d 实读=%d 剔零=%d 入窗=%d 窗外z=%d（单位：点；模式=%s）'
              % (a.pcd, meta['declared'], raw_total, removed, r['n_points_used'], r['n_points_out_of_z'], meta['mode']))
        print('T1_PCD 栅格=%dx%d 格（分辨率 %.2f m）z 窗 [%.2f, %.2f] 板厚 %.2f ⇒ 每格 %d 个 z 板（掩码态空间 %d＝2^%d）'
              % (r['nx'], r['ny'], r['res'], r['z_min'], r['z_max'], r['slab'], r['nz'], 1 << r['nz'], r['nz']))
        print('T1_PCD 覆盖格=%d／总格=%d（覆盖率 %.2f%%）；z 板数分位 P50=%d P95=%d MAX=%d（单位：板／格）'
              % (cov, r['nx'] * r['ny'], 100.0 * cov / (r['nx'] * r['ny']),
                 int(np.percentile(r['n_slab'][r['covered'] > 0], 50)) if cov else -1,
                 int(np.percentile(r['n_slab'][r['covered'] > 0], 95)) if cov else -1,
                 int(r['n_slab'].max()) if cov else -1))
        print('T1_PCD ★多层格（中间有空板）=%d 格 ★悬挑格（最低板之下为空）=%d 格 ⇒ 这两类是 2.5D 单高度表达不了的场景'
              % (int(r['multi_layer'].sum()), int(r['suspended'].sum())))
        print('T1_PCD 耗时=%.1f s（本机单次，★不据此推 Tegra 结论：spike3d §4.3 那句 0.5 s/帧 门槛属活体吞吐判据、本批不测）'
              % (time.time() - t0))
        print('T1_PCD 分档＝甲-离（离线真数据；★不得顶替甲-活）；本件不出代价映射、不出可上判据（#46 三态：未覆盖＝未知，绝不塌陷成可上）')
    if not a.selftest and not a.pcd:
        print('用法：--selftest 或 --pcd <路径>（两者皆不传 ⇒ rc=9 前置缺失）')
        rc = 9
    return rc


if __name__ == '__main__':
    sys.exit(main())
