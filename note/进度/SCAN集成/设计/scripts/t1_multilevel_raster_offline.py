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
#           z 窗 -0.45 / +0.75 与小美给的 z 板厚 0.10 ＝**丙档设计值**（由在册雷达高 38cm 推导），本件不称实测。（★R377 到签后的当期口径另起一句、本句保留为历史时点：模块常量已回写＝**甲档 `Z_MIN, Z_MAX = -0.55, 0.75`＝13 板**，且**引用甲档必须 `--anchor given`**、`ground` 形只可旁报 ⇒ 上面那句 -0.45／12 板只适用于 R377 之前的读数与丙档对照。）
# P8：计划书未定稿、现行以 v5.3（含 R294 收编段）为准、本件不是终稿。

import argparse
import sys
import time

import numpy as np

UNKNOWN, FREE, BLOCKED = -1, 0, 100   # #46 三态在册常量（本件只用 UNKNOWN/覆盖两态，不产出 FREE）
RES = 0.10
# ★R416（真人 2026-10-19 全部到签＝「向后兼容＋不留技术债」）：甲档 13 板 → **档 C 18 板**。
#   下沿 −0.55 → −1.05（增 5 板，容到前向 2 m 工作距）、上沿 +0.75 与板厚 0.10 一字不动。
#   排除：档 B（15 板）只容 1 m ⇒ 盖不住楼梯判读常用 2 m 工作距、迟早二次改档＝新债；
#         档 D（23 板）地面自占 10 板 ⇒ grounded／suspended 区分度被吃掉＝判据退化；
#         档 E（窗不动、喂链前绕 y 回转 15° 做 pitch 补偿）⇒ 判定链与真机链路不等价（真机点云未补偿）且仍须全表重跑＝隐性债。
#   ★新档自批64（R409 乙 落地批）起用；历史 gen0 读数不回溯、仍按 13 板记。
#   ★旧句保留（不回改）＝「R354 甲档在册值（★真人 2026-10-18 G-1 到签取形状甲；★引用甲档必须 --anchor given）」——该句的适用时点＝R377—R415。
Z_MIN, Z_MAX = -1.05, 0.75   # R416 档 C＝18 板（★引用仍必须 --anchor given，见下 --anchor 的 help）
Z_SLAB = 0.10

def gmark(anchor):
    u"""★R389（小美复验 Q2 残留）：ground 形的禁令要贴到**读数行本身**，given 形不贴。"""
    return u'｜★本行＝ground 旁报形、不得入判据读数（G-1 到签④）' if anchor == 'ground' else u''


def zwin_attribution(tag, how):
    """★R366 打印面补句（★纯口径声明、不改任何数）。

    在册硬句真身＝v6 §B.1.4 行「**任何 z 窗引用须连带安装位 (h,θ) 与站立态**（R310 在册硬句）」。
    R310 三值＝h=0.45 m／θ=15°／站立态；在册几何式＝正下方地面雷达系 z=−0.4347 m。
    本件不产 (h,θ)（推窗式在 t3 的 --zwin），故这里只把定档真身**并排报出**、★不冒充本件窗的来源。
    """
    _nz = int(round((Z_MAX - Z_MIN) / Z_SLAB))
    print('%s_ZWIN_ATTR ★z 窗引用须连带安装位 (h,θ)＋站立态（v6 §B.1.4 在册硬句／R310）：'
          'R310 三值＝h=0.45 m ‖ θ=15° ‖ 站立态（几何式在册＝正下方地面雷达系 z=−0.4347 m）；'
          '★定档档 C [%+.2f, %+.2f]（%d 板；★R416 起＝档 C，旧句「甲档 [−0.55,+0.75] 13 板」见 04 R377 段、不回改）现跑式在 t3 --zwin；'
          '本件此行窗★非由 (h,θ) 推得 ⇒ 两式★不得互顶、★不得据本行读作"本件窗已带口径"；'
          '本行窗的来路＝%s' % (tag, Z_MIN, Z_MAX, _nz, how))


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
def rasterize(pts, res=RES, z_min=Z_MIN, z_max=Z_MAX, slab=Z_SLAB, z_ground=None, x_lo=None, x_hi=None, y_lo=None, y_hi=None):
    """每 xy 格存一个 z 板位掩码（bit k ＝第 k 个 z 板被占）。返回 dict。

    z_ground＝本件数据自身的地面高程（建议取剔零后 z 的 P5）。给了它，"悬挑"判据由
    `low > 0` 改为 `low > k0`，`k0 = floor((z_ground - z_min) / slab)`（部署架构师-小张 R296 答复③）；
    窗口下沿高于地面（`z_min > z_ground`）⇒ 整份标 `truncated`，此时两个旗标**只算候选**。
    不传 z_ground ⇒ 沿用旧式 `k0 = 0`（合成自测里的地面就在第 0 板）。
    """
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
    cov = np.zeros(nx * ny, dtype=np.int32)              # ★int32：int16 会在单格 >32767 点时绕回（R302 由 T2 抓到：208836 显示成 12228）
    np.add.at(cov, cell, 1)                                   # 每格落点数（剔零之后）
    nslab_occ = popcount[mask]
    low = np.where(mask > 0, _lowest_bit(mask), -1).astype(np.int16)
    high = np.where(mask > 0, _highest_bit(mask, nz), -1).astype(np.int16)
    contiguous = (mask > 0) & (nslab_occ == (high - low + 1))
    k0 = 0 if z_ground is None else int(np.floor((float(z_ground) - z_min) / slab))
    truncated = bool(z_ground is not None and z_min > float(z_ground) - slab + 1e-9)
    multi = (nslab_occ >= 2) & ~contiguous                    # ★中间有空板 ⇒ 多层／桥洞候选
    susp = (low > k0) & (mask > 0)                            # ★最低板在"地面板 k0"之上才算悬挑候选（小张 R296③）
    return {'res': res, 'z_min': z_min, 'z_max': z_max, 'slab': slab, 'nz': nz,
            'z_ground': None if z_ground is None else float(z_ground), 'k0': k0, 'truncated': truncated,
            'contiguous': contiguous,
            'nx': nx, 'ny': ny, 'x_lo': x_lo, 'x_hi': x_hi, 'y_lo': y_lo, 'y_hi': y_hi,
            'mask': mask, 'covered': cov, 'n_slab': nslab_occ, 'low': low, 'high': high,
            'multi_layer': multi, 'suspended': susp,
            'n_points_in': int(pts.shape[0]), 'n_points_used': int(inside.sum()),
            'n_points_out_of_z': n_out_of_z}


def derived_counts(r):
    """真数据打印面的派生读数（★判据真身在此、`main()` 只打印＝真人 2026-10-18 到签①"抽函数＋补断言"）。

    返回 (接地多层候选格数, 其中空板>=2 板者格数, 每格空板数数组)。小张建议的桥洞可引口径；
    ★不改任何在册旗标定义（`multi_layer`／`suspended` 仍是 A50 待裁旗标）、★不改窗常量与门值。
    """
    n_gap = (r['high'] - r['low'] + 1) - r['n_slab']
    mg = (r['n_slab'] >= 2) & ~r['contiguous'] & (r['low'] <= r['k0'])
    return int(mg.sum()), int((mg & (n_gap >= 2)).sum()), n_gap


REF_LINE_FMT = (u'T1_PCD 参考行（★非在册旗标、A50 待裁）＝接地多层候选（有空板且 low<=k0）=%d 格、'
                u'其中空板>=2 板者=%d 格（单位：格；小张建议的桥洞可引口径，本批不改旗标定义）｜★本行 k0 随锚定形%s')


def ref_line_text(r, anchor_mark=u''):
    """参考行整行文本＝★让排版与判据同源（真人 2026-10-18 到签甲；抓第五位实例的 `D9`＝打印参数对调）。

    数字直接取 `derived_counts()`，故 `main()` 侧不再有可单独对调的参数位。
    """
    mg_n, mg2_n, _ngap = derived_counts(r)
    return REF_LINE_FMT % (mg_n, mg2_n, anchor_mark)


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
    # ★升序：最后一次写入才落在**最高**置位板（降序版每次都覆盖，末次是最低板 ⇒ high≡low，R299 勘误）
    for k in range(nz):
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

    # T-a 单层地面 ⇒ 无多层、无悬挑（另有 T-b 悬挑／T-c 双层／T-d 剔零／T-e 窗外计数／T-f 空输入／
    #      T-g 地面锚定后的悬挑判据／T-h truncated 护栏正反两例／M1—M3 三条阴性对照）
    g = np.column_stack([np.repeat(np.arange(0.0, 1.0, 0.1), 3),
                         np.tile(np.arange(0.0, 1.0, 0.1), 3),
                         np.concatenate([np.full(3, -0.40)] * 10)])
    r = rasterize(_after_zero(g), z_min=-0.45, z_max=0.75, slab=0.10)   # ★R377：窗显式钉回甲档回写前的旧值（G-1 到签⑥＝fixture 字面不动；缺省已随回写漂移，不显式钉住这条机械回归件会假红）
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

    # T-j ★单格点数溢出护栏（R302：cov 曾为 int16，208836 绕回成 12228＝T2 抓出来的）
    big = _cloud([[7.5, 7.5, 0.10, 0]] * 40000)
    r_big = rasterize(big, z_min=-0.45, z_max=0.75, slab=0.10)
    i_big = _cell_of(r_big, 7.5, 7.5)
    check('T-j 单格 40000 点计数不绕回（int32 生效）', int(r_big['covered'][i_big]) == 40000,
          '实测=%d（int16 下会绕成 %d）' % (int(r_big['covered'][i_big]), 40000 - 65536))

    # T-f 空输入必须报错而不是静默产出空面
    try:
        rasterize(np.zeros((0, 4)))
        check('T-f 空点云须抛错', False)
    except ValueError:
        check('T-f 空点云须抛错', True)

    # T-g 地面锚定（小张 R296 答复③）：窗口下沿低于数据地面时，旧式 low>0 会把"贴地"判成悬挑
    gg = np.column_stack([np.repeat(np.arange(0.0, 1.0, 0.1), 3),
                          np.tile(np.arange(0.0, 1.0, 0.1), 3),
                          np.concatenate([np.full(3, 0.40)] * 10)])
    r_old = rasterize(_after_zero(gg), z_min=-0.45, z_max=0.75, slab=0.10)
    r_new = rasterize(_after_zero(gg), z_min=-0.45, z_max=0.75, slab=0.10, z_ground=0.40)
    check('T-g 给地面锚后单层贴地 suspended=0', int(r_new['suspended'].sum()) == 0,
          'k0=%d（同数据不给锚时旧式误判=%d 格）' % (r_new['k0'], int(r_old['suspended'].sum())))
    check('T-g 地面锚不动多层判据 multi_layer=0', int(r_new['multi_layer'].sum()) == 0)

    # M3 阴性对照：不给地面锚 ⇒ 同一组"贴地"数据必须被误判成悬挑（证明 k0 这一步确有分辨力）
    check('M3 不给地面锚时贴地数据误判为悬挑（阴性对照须成立）', int(r_old['suspended'].sum()) > 0,
          '误判=%d／覆盖格=%d' % (int(r_old['suspended'].sum()), int((r_old['covered'] > 0).sum())))

    # T-h 护栏（★小张新修正④：要求地面所在**整板**在窗内 ⇒ 判据是 z_min > z_ground − slab）
    hh = _cloud([[0.5, 0.5, 0.60, 0], [0.6, 0.5, 0.90, 0], [0.5, 0.6, 0.60, 0]])
    r_h = rasterize(hh, z_min=0.50, z_max=1.70, slab=0.10, z_ground=0.30)
    check('T-h z_min 远高于地面 ⇒ truncated=True', r_h['truncated'] is True,
          'z_min=%.2f z_ground=%.2f' % (r_h['z_min'], r_h['z_ground']))
    r_h2 = rasterize(hh, z_min=0.30, z_max=1.50, slab=0.10, z_ground=0.30)
    check('T-h 下沿==地面（不垫板）⇒ 现按④判 truncated=True（地面那板被截半）', r_h2['truncated'] is True,
          'z_min=%.2f 要求 z_min<=%.2f' % (r_h2['z_min'], 0.30 - 0.10))
    r_h3 = rasterize(hh, z_min=0.20, z_max=1.40, slab=0.10, z_ground=0.30)
    check('T-h 下沿垫一板 ⇒ truncated=False（这是推荐的贴地法）', r_h3['truncated'] is False,
          'z_min=%.2f k0=%d' % (r_h3['z_min'], r_h3['k0']))

    # T-i ★本批补的实现级对照（R299 勘误：_highest_bit 曾把 high 写成 low，致 multi_layer 退化）
    r_hw = rasterize(_cloud([[0.05, 0.05, 0.00, 1], [0.05, 0.05, 0.10, 1], [0.05, 0.05, 0.20, 1],
                             [0.55, 0.55, 0.00, 1], [0.55, 0.55, 0.20, 1]]),
                    z_min=-0.45, z_max=0.75, slab=0.10)
    i_full = _cell_of(r_hw, 0.05, 0.05)
    i_gap = _cell_of(r_hw, 0.55, 0.55)
    check('T-i high/low 直测（连续三板 high 应为最低+2）',
          (int(r_hw['low'][i_full]), int(r_hw['high'][i_full])) == (4, 6),
          'low=%d high=%d（旧缺陷下＝4 4）' % (int(r_hw['low'][i_full]), int(r_hw['high'][i_full])))
    check('T-i 连续三板的格**不得**算多层（正例，旧缺陷下必误判）', bool(r_hw['multi_layer'][i_full]) is False,
          'n_slab=%d contiguous=%s' % (int(r_hw['n_slab'][i_full]), bool(r_hw['contiguous'][i_full])))
    check('T-i 0/2 有、1 空的格**必须**算多层（反例）', bool(r_hw['multi_layer'][i_gap]) is True,
          'n_slab=%d low=%d high=%d' % (int(r_hw['n_slab'][i_gap]), int(r_hw['low'][i_gap]), int(r_hw['high'][i_gap])))

    # T-k ★档形断言（真人 G-1 到签⑥；R310 锚＝正下方地面雷达系 z=−0.4347）
    #   ★R416 另起句（真人 2026-10-19 全部到签＝档 C）：原断「甲档 nz=13 ‖ k0=1 ‖ truncated=False」＝R377—R415 时点、不回改；
    #   R416 起档 C 18 板 ⇒ 现断 nz=18 ‖ k0=6 ‖ truncated=False（k0 1→6＝窗下沿降 5 板的直接结果，★不是回归）。
    r_jia = rasterize(_after_zero(gg), z_min=Z_MIN, z_max=Z_MAX, slab=Z_SLAB, z_ground=-0.4347)
    check('T-k 档 C 窗形 nz=18 ‖ k0=6 ‖ truncated=False（vs R310 锚 −0.4347；★R377—R415 的甲档 13 板／k0=1 见 04 R377 段）',
          r_jia['nz'] == 18 and r_jia['k0'] == 6 and r_jia['truncated'] is False,
          '窗[%.2f, %.2f] nz=%d k0=%d truncated=%s' % (Z_MIN, Z_MAX, r_jia['nz'], r_jia['k0'], r_jia['truncated']))
    # ★阳性对照（证上一条有牙）：旧丙档窗形在同一锚上必须判成 truncated=True（＝R352 在册回核结论）
    r_bing = rasterize(_after_zero(gg), z_min=-0.45, z_max=0.75, slab=Z_SLAB, z_ground=-0.4347)
    check('T-k 对照：丙档窗 [-0.45,0.75] 同锚必 truncated=True（★证上一条不是恒真摆设）',
          r_bing['nz'] == 12 and r_bing['truncated'] is True,
          'nz=%d k0=%d truncated=%s' % (r_bing['nz'], r_bing['k0'], r_bing['truncated']))

    # T-l ★k0 取整口径两侧对照（R392 小美四轮 03 要件二＝抓 N10 floor→round、N12 floor→int() 截断）
    #   存活根因（她的话）＝现形所有 fixture 的 (z_ground−z_min)/slab 比值全落三者同值位 ⇒ 取整口径改坏无人拦；本枚两例并钉。
    hl = _cloud([[0.50, 0.50, 0.60, 0], [0.55, 0.50, 0.90, 0]])
    r_lA = rasterize(hl, z_min=0.05, z_max=1.25, slab=0.10, z_ground=0.00)    # 比值＝−0.5
    check('T-l A 负半板比值 −0.5 ⇒ k0=−1（floor；round／int() 截断都得 0 ⇒ 取整口径一改坏本枚必红）',
          r_lA['k0'] == -1,
          'k0=%d truncated=%s（★该例 truncated=True 属几何事实、★不得当"窗可用"引用，与 T-h 护栏例分行）'
          % (r_lA['k0'], r_lA['truncated']))
    r_lB = rasterize(hl, z_min=-0.05, z_max=1.15, slab=0.10, z_ground=0.10)   # 比值＝+1.5
    check('T-l B 正半板比值 +1.5 ⇒ k0=1（floor；round 得 2 ⇒ 本枚必红）',
          r_lB['k0'] == 1, 'k0=%d truncated=%s' % (r_lB['k0'], r_lB['truncated']))

    # T-m ★main() 面派生读数的断言（真人 2026-10-18 到签①＝抽函数＋补断言；抓 r392 全表最后两枚存活 N17／N18）
    #   夹具＝一格贴地多层（low==k0、空板 2）＋一格台阶多层（low==k0+1）＋一格连续填充柱（★抓"漏掉非连续护栏"那族）
    #        ＋★一格"接地、恰 1 空板"的多层格（R397 第五位实例要件＝破 `mg_n==mg2_n` 打平 ⇒ 抓 D8 返回元组对调）。
    r_dm = rasterize(_cloud([[0.00, 0.00, 0.10, 0], [0.00, 0.00, 0.40, 0],
                             [1.00, 1.00, 0.20, 0], [1.00, 1.00, 0.50, 0],
                             [2.00, 2.00, 0.10, 0], [2.00, 2.00, 0.20, 0], [2.00, 2.00, 0.30, 0],
                             [3.00, 3.00, 0.10, 0], [3.00, 3.00, 0.35, 0]]),
                     res=0.05, z_min=0.0, z_max=1.2, slab=0.10, z_ground=0.1)
    mg_n, mg2_n, ngap = derived_counts(r_dm)
    i_p = _cell_of(r_dm, 0.00, 0.00)
    i_q = _cell_of(r_dm, 1.00, 1.00)
    i_s = _cell_of(r_dm, 2.00, 2.00)
    i_t = _cell_of(r_dm, 3.00, 3.00)
    check('T-m 派生判据有牙：接地多层候选=2 ‖ 空板>=2 者=1（★两数不得相等＝抓 D8 返回序对调；台阶格与连续柱须被排除＝抓 N17／N19）',
          mg_n == 2 and mg2_n == 1,
          'mg=%d mg2=%d 贴地两层格 low=%d 台阶格 low=%d 连续柱 n_slab=%d 接地单层空板格 low=%d k0=%d'
          % (mg_n, mg2_n, int(r_dm['low'][i_p]), int(r_dm['low'][i_q]),
             int(r_dm['n_slab'][i_s]), int(r_dm['low'][i_t]), r_dm['k0']))
    check('T-m 夹具几何前提自证（★证上一条不是恒真摆设：空板式带 +1＝抓 N18／D4；阈值 2 与 3 有别＝抓 N20／D6）',
          int(ngap[i_p]) == 2 and int(ngap[i_q]) == 2 and int(ngap[i_s]) == 0 and int(ngap[i_t]) == 1
          and int(r_dm['low'][i_p]) == r_dm['k0'] and int(r_dm['low'][i_q]) == r_dm['k0'] + 1
          and int(r_dm['low'][i_t]) == r_dm['k0'],
          '空板数[贴地两层]=%d [台阶两层]=%d [连续柱]=%d [接地单层空板]=%d'
          % (int(ngap[i_p]), int(ngap[i_q]), int(ngap[i_s]), int(ngap[i_t])))

    # T-n ★排版与判据同源的断言（真人 2026-10-18 到签甲＝抽打印函数；抓第五位实例的 D9＝打印参数对调）
    _mn, _m2, _ng = derived_counts(r_dm)
    check('T-n 同源断言：参考行串内两数顺序须等于 derived_counts 返回值（★抓 D9；对调后串里成 =1 格／=2 格 ⇒ 本枚必红）',
          (u'=%d 格、其中空板>=2 板者=%d 格' % (_mn, _m2)) in ref_line_text(r_dm) and _mn == 2 and _m2 == 1,
          '串内片段＝…=%d 格、其中空板>=2 板者=%d 格…（★两数不等＝对调可见）' % (_mn, _m2))
    check('T-n 对照：旁报形尾巴只随 anchor_mark 出（★given 形不得带旁报禁令＝防"两边都贴"造假禁令）',
          (u'不得入判据读数' not in ref_line_text(r_dm))
          and (u'不得入判据读数' in ref_line_text(r_dm, gmark('ground'))),
          'given 串带禁令＝False（要求）‖ ground 串带禁令＝True（要求）')

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
                    help="given＝照 --min-z/--max-z（★G-1 到签＝**引用甲档必须走本形**；缺省即 given）｜同一组字面若走 ground＝只控 span、下沿改由实测地面分位锚定 ⇒ 产出**只可作旁报、★不得入判据读数**｜（given 那组是雷达系值，地面≈-0.38）；ground＝把同一 z 宽度下沿贴到本件自报的地面分位（★map 系必用此档）")
    ap.add_argument('--ground-pct', type=float, default=5.0, help='地面代理分位（%%），默认 P5')
    ap.add_argument('--ground-pad', type=int, default=1,
                    help='锚定=ground 时把窗下沿再垫几板（小张修正④的等价做法；默认 1＝保证地面所在整板在窗内；0＝复现 R295／R297 那两次读数）')
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
        zmin, zmax = (a.min_z, a.max_z) if a.anchor == 'given' else (float(zpct[1]) - a.ground_pad * a.slab,
                                                                      float(zpct[1]) - a.ground_pad * a.slab + span)
        zground = float(zpct[1])
        r = rasterize(pts, res=a.res, z_min=zmin, z_max=zmax, slab=a.slab, z_ground=zground)
        cov = int((r['covered'] > 0).sum())
        print('T1_ZDIST z 分位（单位：m，map 系）MIN=%.2f P%.0f=%.2f P50=%.2f P95=%.2f P99=%.2f MAX=%.2f'
              % (zpct[0], a.ground_pct, zpct[1], zpct[2], zpct[3], zpct[4], zpct[5]))
        print('T1_ZWIN 锚定=%s 垫板=%d 二元式=(z_ground=%.2f, z_range=%.2fm) ⇒ 实开窗 [%.2f, %.2f]（k0=%d 板；truncated=%s；★窗口宽度沿用设计值，只换下沿锚；雷达系那组值直接用在 map 系会把大部分点判成窗外）%s'
              % (a.anchor, a.ground_pad if a.anchor == 'ground' else 0, zground, zmax - zmin, zmin, zmax, r['k0'], r['truncated'], gmark(a.anchor)))
        zwin_attribution('T1', '锚定=%s／垫板=%s／窗宽=max_z−min_z（件内常量回写后＝1.30 m‖13 板，R377 甲档）%s'
                            % (a.anchor, a.ground_pad if a.anchor == 'ground' else 0,
                               gmark(a.anchor)))
        print('T1_PCD 文件=%s 声明点数=%d 实读=%d 剔零=%d 入窗=%d 窗外z=%d（单位：点；模式=%s）'
              % (a.pcd, meta['declared'], raw_total, removed, r['n_points_used'], r['n_points_out_of_z'], meta['mode']))
        print('T1_PCD 栅格=%dx%d 格（分辨率 %.2f m）z 窗 [%.2f, %.2f] 板厚 %.2f ⇒ 每格 %d 个 z 板（掩码态空间 %d＝2^%d）%s'
              % (r['nx'], r['ny'], r['res'], r['z_min'], r['z_max'], r['slab'], r['nz'], 1 << r['nz'], r['nz'],
                 gmark(a.anchor)))
        print('T1_PCD 覆盖格=%d／总格=%d（覆盖率 %.2f%%）；z 板数分位 P50=%d P95=%d MAX=%d（单位：板／格）'
              % (cov, r['nx'] * r['ny'], 100.0 * cov / (r['nx'] * r['ny']),
                 int(np.percentile(r['n_slab'][r['covered'] > 0], 50)) if cov else -1,
                 int(np.percentile(r['n_slab'][r['covered'] > 0], 95)) if cov else -1,
                 int(r['n_slab'].max()) if cov else -1))
        print('T1_PCD ★多层格（中间真有空板＝不连续）=%d 格 ★悬挑格（最低板>地面板 k0=%d）=%d 格 ⇒ 这两类是 2.5D 单高度表达不了的场景%s%s'
              % (int(r['multi_layer'].sum()), r['k0'], int(r['suspended'].sum()),
                 '；★truncated=True（窗下沿截掉了地面那半板）⇒ 本行两旗标只算候选、不得引用' if r['truncated'] else '',
                 gmark(a.anchor)))
        print('T1_PCD 另有（不是多层）＝板数>=2 的格=%d、连续填充的格=%d（单位：格；★R295／R297 那两个"多层"数用的是这个 >=2 板口径、当时 high 有缺陷，见 R299 勘误）'
              % (int((r['n_slab'] >= 2).sum()), int(r['contiguous'].sum())))
        print(ref_line_text(r, gmark(a.anchor)))   # ★文案与判据同源（R398 到签甲）：参数位不在 main、对调不了
        print('T1_PCD 耗时=%.1f s（本机单次，★不据此推 Tegra 结论：spike3d §4.3 那句 0.5 s/帧 门槛属活体吞吐判据、本批不测）'
              % (time.time() - t0))
        print('T1_PCD 分档＝甲-离（离线真数据；★不得顶替甲-活）；本件不出代价映射、不出可上判据（#46 三态：未覆盖＝未知，绝不塌陷成可上）')
    if not a.selftest and not a.pcd:
        print('用法：--selftest 或 --pcd <路径>（两者皆不传 ⇒ rc=9 前置缺失）')
        rc = 9
    return rc


if __name__ == '__main__':
    sys.exit(main())
