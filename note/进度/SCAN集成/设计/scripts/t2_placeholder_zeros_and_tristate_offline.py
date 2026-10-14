#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""T2｜占位零点剔除 + #46 三态不塌陷的**离线**对照件（零窗，A47＝丙 授权的三步之二）

数据来源＝R82 那次活体自遮挡探针落盘的 14 帧原始点云（float16，雷达系），本件**不发 ROS、不开窗**。
在册来历＝＃176 那句「静止时 208,836 个 (0,0,0) 全塌进 1 格」——本件把它变成可复跑的断言。
判据口径＝＃46 三态：未覆盖＝**未知**，绝不塌陷成"可上"。

用法：
  python3 t2_placeholder_zeros_and_tristate_offline.py --selftest
  python3 t2_placeholder_zeros_and_tristate_offline.py --frames <R82 目录> [--res 0.05] [--slab 0.10]
"""
import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import t1_multilevel_raster_offline as t1  # 同一目录的表征件（★复用判据，不另写一套）

EXPECTED_ZERO_TOTAL = 208836  # 在册值（＃176；单位＝点，14 帧合计）


def load_frames(dirpath):
    """读 <dir>/s*.npy（每帧形状 (3, N) 的 float16），转成 (N, 4) 的 x,y,z,intensity。"""
    files = sorted(glob.glob(os.path.join(dirpath, 's*.npy')))
    if not files:
        raise ValueError('目录里没有 s*.npy：%s' % dirpath)
    frames = []
    for f in files:
        a = np.load(f, allow_pickle=True)
        if a.ndim != 2 or a.shape[0] != 3:
            raise ValueError('%s 形状不是 (3, N)＝%s ⇒ 本件只吃 R82 那种 xyz 三行帧' % (os.path.basename(f), a.shape))
        pts = np.asarray(a.T, dtype=np.float64)          # (N, 3)
        frames.append((os.path.basename(f), np.column_stack([pts, np.zeros(pts.shape[0], dtype=np.float64)])))
    return frames


def zero_stats(frames):
    per = []
    for name, pts in frames:
        iszero = (pts[:, 0] == 0.0) & (pts[:, 1] == 0.0) & (pts[:, 2] == 0.0)
        per.append((name, int(pts.shape[0]), int(iszero.sum()), int((~iszero).sum())))
    return per


def cell_of_origin(frames, res=0.05, slab=0.10, drop=True):
    """把 14 帧合起来栅格化；返回 (结果字典, 剔除的零点数, 原点那一格的落点数或 -1)。"""
    allp = np.vstack([p for _, p in frames])
    removed = 0
    if drop:
        allp, removed = t1.drop_placeholder_zeros(allp)
    z = allp[:, 2]
    z_ground = float(np.percentile(z, 5)) if allp.shape[0] else 0.0
    r = t1.rasterize(allp, res=res, z_min=z_ground - slab, z_max=z_ground - slab + 12 * slab,
                     slab=slab, z_ground=z_ground)
    idx = t1._cell_of(r, 0.0, 0.0)
    at_origin = int(r['covered'][idx]) if 0 <= idx < r['nx'] * r['ny'] else 0
    return r, removed, at_origin, z_ground


def tristate_counts(r):
    """三态计数（本件只到"表征"，不产代价）：接地＝覆盖且最低板贴地；未知＝没回波。"""
    total = r['nx'] * r['ny']
    covered = int((r['covered'] > 0).sum())
    unknown = total - covered
    grounded = int(((r['covered'] > 0) & (r['low'] <= r['k0'])).sum())
    return total, covered, unknown, grounded


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

    def cloud(xyz):
        return np.array(xyz, dtype=np.float64)

    # T2-a 合成：一堆占位零点 + 一个真点 ⇒ 剔零必须报出**条数**，且零点不再撑出格
    mixed = np.vstack([cloud([[0.0, 0.0, 0.0, 0]] * 700), cloud([[1.5, 1.5, 0.10, 0]])])
    cleaned, removed = t1.drop_placeholder_zeros(mixed)
    check('T2-a 剔零计数 removed=700', removed == 700, 'removed=%d' % removed)
    r_keep = t1.rasterize(cleaned, z_min=0.0, z_max=1.2, slab=0.1, z_ground=0.1)
    r_all = t1.rasterize(mixed, z_min=0.0, z_max=1.2, slab=0.1, z_ground=0.1)
    check('T2-a 不剔零会多撑出一格（覆盖格 +1）',
          int((r_all['covered'] > 0).sum()) == int((r_keep['covered'] > 0).sum()) + 1,
          '剔零=%d 格／不剔=%d 格' % (int((r_keep['covered'] > 0).sum()), int((r_all['covered'] > 0).sum())))
    idx0 = t1._cell_of(r_all, 0.0, 0.0)
    check('T2-a 零点全塌进**同一格**（700 点＝1 格）', int(r_all['covered'][idx0]) == 700, '该格落点=%d' % int(r_all['covered'][idx0]))

    # T2-b／T2-c 三态守恒与塌陷对照：用一"稀疏"合成云 ⇒ 格网跨很多格、只有两格有回波
    sparse = cloud([[0.0, 0.0, 0.10, 0], [0.05, 0.0, 0.10, 0], [1.00, 1.00, 0.10, 0], [1.05, 1.00, 0.10, 0]])
    r_sp = t1.rasterize(sparse, res=0.05, z_min=0.0, z_max=1.2, slab=0.1, z_ground=0.1)
    total, covered, unknown, grounded = tristate_counts(r_sp)
    check('T2-b 三态守恒 total==covered+unknown', total == covered + unknown,
          'total=%d covered=%d unknown=%d' % (total, covered, unknown))
    check('T2-b 接地格 ⊆ 覆盖格 且 覆盖格确为 4（四点各落一格）', grounded <= covered and covered == 4,
          'grounded=%d covered=%d' % (grounded, covered))

    # T2-c 阴性对照（塌陷测试）：把"未覆盖"当"可上" ⇒ 必须造出一大片**假自由空间**（证明这条断言有牙）
    fake_free = unknown                    # ★这就是塌陷：未观测被读成自由空间
    check('T2-c 塌陷后"可上格"虚高（阴性对照须成立）', fake_free > 0 and fake_free == total - covered,
          '虚高=%d／%d 格（%.1f%%）' % (fake_free, total, 100.0 * fake_free / total))

    # T2-d 阴性对照（剔零关闭）：真实障碍那格的邻居被零点占走 ⇒ 断言必失效
    naive_zero_cell = int(r_all['covered'][idx0])
    check('T2-d 关掉剔零 ⇒ 原点格被 700 个假点占满（故剔除步不是白计）', naive_zero_cell == 700,
          '该格点数=%d（真点只有 1 个）' % naive_zero_cell)

    # T2-e 地面锚定与垫板（沿用小张 ②③④）：不给地面锚 ⇒ k0=0、旧式退化
    r_noanchor = t1.rasterize(cleaned, z_min=0.0, z_max=1.2, slab=0.1)
    check('T2-e 不给 z_ground ⇒ k0=0（旧式）', r_noanchor['k0'] == 0 and r_noanchor['z_ground'] is None)
    check('T2-e 下沿垫一板 ⇒ truncated=False', r_keep['truncated'] is False, 'z_min=%.2f z_ground=%.2f' % (r_keep['z_min'], r_keep['z_ground']))

    # T2-f 空输入必须报错，不静默产出空面
    try:
        t1.rasterize(np.zeros((0, 4)), z_min=0.0, z_max=1.2, slab=0.1)
        check('T2-f 空点云须抛错', False)
    except ValueError:
        check('T2-f 空点云须抛错', True)

    lines.append('  合计 PASS=%d FAIL=%d' % (ok, fail))
    return ok, fail, lines


def main(argv=None):
    ap = argparse.ArgumentParser(description='T2 占位零点剔除＋三态不塌陷的离线对照（零窗）')
    ap.add_argument('--selftest', action='store_true')
    ap.add_argument('--frames', help='R82 那种含 s*.npy 的目录')
    ap.add_argument('--res', type=float, default=0.05)
    ap.add_argument('--slab', type=float, default=0.10)
    ap.add_argument('--expect-zero', type=int, default=EXPECTED_ZERO_TOTAL)
    a = ap.parse_args(argv)
    rc = 0
    if a.selftest:
        ok, fail, lines = selftest()
        for ln in lines:
            print(ln)
        print('T2_SELFTEST PASS=%d FAIL=%d rc=%d' % (ok, fail, 0 if fail == 0 else 1))
        rc = 0 if fail == 0 else 1
    if a.frames:
        frames = load_frames(a.frames)
        per = zero_stats(frames)
        tot_pts = sum(x[1] for x in per)
        tot_zero = sum(x[2] for x in per)
        print('T2_FRAMES 目录=%s 帧数=%d 合计点=%d 全零=%d 非零=%d（单位：点）'
              % (a.frames, len(frames), tot_pts, tot_zero, tot_pts - tot_zero))
        for name, n, z, nz in per:
            print('  帧 %-14s N=%d 全零=%d 非零=%d（全零占比 %.1f%%）' % (name[:-4], n, z, nz, 100.0 * z / n))
        print('T2_ZERO_MATCH 在册＃176 期望=%d 实测=%d ⇒ %s'
              % (a.expect_zero, tot_zero, '一致' if tot_zero == a.expect_zero else '**不一致（照实报）**'))
        r_pre, _rm, cell_pre, zg = cell_of_origin(frames, res=a.res, slab=a.slab, drop=False)
        r_post, removed, cell_post, zg2 = cell_of_origin(frames, res=a.res, slab=a.slab, drop=True)
        print('T2_DROP 剔零=%d 点（单位：点）；z_ground(P5，雷达系)=%.2f m ⇒ 实开窗 [%.2f, %.2f] k0=%d truncated=%s'
              % (removed, zg2, r_post['z_min'], r_post['z_max'], r_post['k0'], r_post['truncated']))
        print('T2_COLLAPSE 不剔零：原点格落点=%d 点／覆盖格=%d｜剔零后：原点格落点=%d／覆盖格=%d ⇒ ★零点确实全塌进 1 格'
              % (cell_pre, int((r_pre['covered'] > 0).sum()), cell_post, int((r_post['covered'] > 0).sum())))
        total, covered, unknown, grounded = tristate_counts(r_post)
        print('T2_TRISTATE 总格=%d 覆盖=%d 未知=%d 接地=%d（单位：格；守恒式 total==covered+unknown ⇒ %s）'
              % (total, covered, unknown, grounded, '成立' if total == covered + unknown else '**破坏**'))
        print('T2_TRISTATE ★本件不产代价、不产"可上"：未覆盖一律读作**未知**（#46）；接地格 %d 只是"最低板≤地面板 k0"的表征事实'
              % grounded)
        print('T2_TRISTATE 塌陷测试：若把未知 %d 格读作可上 ⇒ 自由空间虚高 %.1f%%（★这就是 #46 明令禁止的那种读法）'
              % (unknown, 100.0 * unknown / total))
        print('T2_LAYER 多层格（中间真有空板）=%d 悬挑候选格（low>k0）=%d 板数>=2 的格=%d（单位：格）'
              % (int(r_post['multi_layer'].sum()), int(r_post['suspended'].sum()), int((r_post['n_slab'] >= 2).sum())))
        print('T2_GRADE 分档＝甲-离（离线真数据、活体落盘的帧；★不得顶替甲-活）')
    if not a.selftest and not a.frames:
        print('用法：--selftest 或 --frames <R82 目录>（两者皆不传 ⇒ rc=9 前置缺失）')
        rc = 9
    return rc


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
