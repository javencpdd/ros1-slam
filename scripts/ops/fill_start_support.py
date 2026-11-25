#!/usr/bin/env python3
"""在既有支撑图上补一小块"站得住"的平台（对应框架自带的 fill_start_platform 语义）。

为什么需要：支撑图由点云栅格化而来，雷达正下方与遮挡区天生没有点 ⇒ 该格没有面 ⇒
局部规划器以 `E_LOCAL_SUPPORT`（"start requires a matching z support"）拒绝规划。
`framework/tools/map_from_pcd.py:269` 已有同名函数，但它工作在生成期的内存结构上；
本工具直接作用于**已生成的地图 JSON**，只在该点半径内补最低那个面的格子，其余一概不动。

用法：
  python3 scripts/ops/fill_start_support.py data/maps/generated/v8_m1_hdl_v1.json 5.2 2.45 --radius 0.5
  ... --dry-run 只报会补多少格，不写文件
"""
import argparse
import json
import math
import os
import sys


def decode_runs(surface):
    cells = set()
    enc = surface['cells']
    if isinstance(enc, dict) and enc.get('encoding') == 'rle':
        for c, r, n in enc['runs']:
            for k in range(n):
                cells.add((c, r + k))
    else:  # 兼容已是列表的形态
        cells = {(int(c), int(r)) for c, r in enc}
    return cells


def encode_runs(cells):
    """按 (col, row) 升序做 RLE：同一列内连续行合并为一段。"""
    runs = []
    for c in sorted({c for c, _ in cells}):
        rows = sorted(r for cc, r in cells if cc == c)
        if not rows:
            continue
        start = prev = rows[0]
        for r in rows[1:]:
            if r == prev + 1:
                prev = r
                continue
            runs.append([c, start, prev - start + 1])
            start = prev = r
        runs.append([c, start, prev - start + 1])
    return {'encoding': 'rle', 'runs': runs}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map_json')
    ap.add_argument('x', type=float)
    ap.add_argument('y', type=float)
    ap.add_argument('--radius', type=float, default=0.5, help='补面半径（米）')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    d = json.load(open(args.map_json, encoding='utf-8'))
    surfaces = d.get('surfaces') or []
    if not surfaces:
        sys.exit('[fill] 该地图没有 surfaces，拒绝操作')
    # 只补最低的那个面（地面），避免把平台挂到桌面/台阶上
    surf = min(surfaces, key=lambda s: s.get('z_reference', 0.0))

    res = float(d.get('resolution', 0.1))
    ox, oy = float(d['origin'][0]), float(d['origin'][1])
    ext = d.get('extent') or {}
    w, h = int(ext.get('width', 0)), int(ext.get('height', 0))

    cells = decode_runs(surf)
    before = len(cells)

    cx, cy = int(round((args.x - ox) / res)), int(round((args.y - oy) / res))
    rad_cells = int(math.ceil(args.radius / res))
    added = 0
    for dc in range(-rad_cells, rad_cells + 1):
        for dr in range(-rad_cells, rad_cells + 1):
            if math.hypot(dc * res, dr * res) > args.radius:
                continue
            c, r = cx + dc, cy + dr
            if w and not (0 <= c < w):
                continue
            if h and not (0 <= r < h):
                continue
            if (c, r) not in cells:
                cells.add((c, r))
                added += 1

    print('[fill] map=%s z_ref=%.3f target=(%.2f,%.2f) idx=(%d,%d) radius=%.2fm'
          % (os.path.basename(args.map_json), surf.get('z_reference', 0.0),
             args.x, args.y, cx, cy, args.radius))
    print('[fill] cells %d -> %d (added %d)' % (before, len(cells), added))
    if args.dry_run:
        print('[fill] DRY-RUN 未写文件')
        return 0
    if added == 0:
        print('[fill] 无需改动')
        return 0
    surf['cells'] = encode_runs(cells)
    tmp = args.map_json + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as fh:
        json.dump(d, fh, ensure_ascii=False, indent=1)
        fh.write('\n')
    os.replace(tmp, args.map_json)
    print('[fill] 已写回 %s' % args.map_json)
    return 0


if __name__ == '__main__':
    sys.exit(main())
