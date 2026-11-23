#!/usr/bin/env python3
"""算自由空间的最大净宽：障碍点栅格化 -> 膨胀 -> 多源距离变换 -> 可达区内最大内切圆。

用途：在选导航路线之前，先定量回答「这个世界有没有净宽 > X 米的通道」。
判据只用点云与地图，不引入 Gazebo 真值。

净宽定义：自由栅格到最近障碍栅格的距离 d，2*d 即该点能容纳的最大圆直径。
膨胀已含 radius_m + clearance_m + 速度余量，所以 2*d 是**留给机体中心**的可用直径。

用法：
  free_space_width.py --pcd data/maps/mapped_m1.pcd --start 1.0,-4.25 --need 0.70
"""
import argparse
import json
import math
import os
import sys
from collections import deque

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def load_pcd(path):
    raw = open(path, 'rb').read()
    marker = raw.find(b'DATA binary')
    if marker < 0:
        raise SystemExit('只支持 binary PCD: %s' % path)
    start = marker + len(b'DATA binary')
    while raw[start:start + 1] in (b'\n', b'\r'):
        start += 1
    import re
    header = raw[:start].decode('ascii', 'replace')
    points = int(re.search(r'POINTS (\d+)', header).group(1))
    fields = len(re.search(r'FIELDS (.*)', header).group(1).split())
    data = np.frombuffer(raw[start:start + points * fields * 4], dtype=np.float32)
    return data.reshape(-1, fields)[:, :3].astype(float)


def grid(points, res, lo, hi):
    nx = int(math.ceil((hi[0] - lo[0]) / res))
    ny = int(math.ceil((hi[1] - lo[1]) / res))
    grid_mask = np.zeros((nx, ny), dtype=bool)
    ix = np.floor((points[:, 0] - lo[0]) / res).astype(int)
    iy = np.floor((points[:, 1] - lo[1]) / res).astype(int)
    ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    grid_mask[ix[ok], iy[ok]] = True
    return grid_mask, nx, ny


def dilate(mask, cells):
    """方形结构元膨胀 cells 格（与碰撞判定的方形膨胀同形）。"""
    out = mask.copy()
    for _ in range(cells):
        grown = out.copy()
        grown[1:, :] |= out[:-1, :]
        grown[:-1, :] |= out[1:, :]
        grown[:, 1:] |= out[:, :-1]
        grown[:, :-1] |= out[:, 1:]
        out = grown
    return out


def distance_transform(free):
    """多源 BFS：每个自由格到最近障碍格（含已膨胀边界外）的格距。"""
    nx, ny = free.shape
    dist = np.full((nx, ny), -1, dtype=np.int32)
    queue = deque()
    for i in range(nx):
        for j in range(ny):
            if not free[i, j]:
                dist[i, j] = 0
                queue.append((i, j))
    while queue:
        i, j = queue.popleft()
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ni, nj = i + di, j + dj
            if 0 <= ni < nx and 0 <= nj < ny and dist[ni, nj] < 0:
                dist[ni, nj] = dist[i, j] + 1
                queue.append((ni, nj))
    return dist


def reachable(free, start_cell):
    nx, ny = free.shape
    seen = np.zeros((nx, ny), dtype=bool)
    si, sj = start_cell
    if not (0 <= si < nx and 0 <= sj < ny) or not free[si, sj]:
        return seen
    queue = deque([(si, sj)])
    seen[si, sj] = True
    while queue:
        i, j = queue.popleft()
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ni, nj = i + di, j + dj
            if 0 <= ni < nx and 0 <= nj < ny and free[ni, nj] and not seen[ni, nj]:
                seen[ni, nj] = True
                queue.append((ni, nj))
    return seen


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pcd', required=True)
    ap.add_argument('--res', type=float, default=0.10)
    ap.add_argument('--start', default='1.0,-4.25')
    ap.add_argument('--inflate', type=float, default=0.456,
                    help='膨胀距离：radius 0.15 + clearance 0.10 + 速度余量约 0.206')
    ap.add_argument('--min-z', type=float, default=0.15)
    ap.add_argument('--max-z', type=float, default=1.60)
    ap.add_argument('--need', type=float, default=0.70)
    ap.add_argument('--out', default='')
    ap.add_argument('--mask-map', default='',
                    help='support map JSON；只统计它覆盖的格子，排除地图外空白')
    ap.add_argument('--any', action='store_true',
                    help='不算可达性，直接给全图自由空间的最大内切圆（用于判断世界本身有没有空间）')
    args = ap.parse_args()

    points = load_pcd(args.pcd)
    obstacles = points[(points[:, 2] >= args.min_z) & (points[:, 2] <= args.max_z)]
    lo = (points[:, 0].min() - 0.5, points[:, 1].min() - 0.5)
    hi = (points[:, 0].max() + 0.5, points[:, 1].max() + 0.5)
    occ, nx, ny = grid(obstacles, args.res, lo, hi)
    cells = int(round(args.inflate / args.res))
    inflated = dilate(occ, cells)
    free = ~inflated

    sx, sy = [float(v) for v in args.start.split(',')]
    si = int(math.floor((sx - lo[0]) / args.res))
    sj = int(math.floor((sy - lo[1]) / args.res))
    si = min(max(si, 0), nx - 1)
    sj = min(max(sj, 0), ny - 1)
    if not args.any and not free[si, sj]:
        print(json.dumps({'ok': False, 'reason': 'start_inside_inflated_obstacle',
                          'start': [sx, sy], 'inflate_m': args.inflate}, ensure_ascii=False))
        return 2

    if args.mask_map:
        import json as _json
        sys.path.insert(0, os.path.join(REPO, 'ws/src/80_algorithm/v8_algorithm/representation'))
        import support_map as rep
        map_value = _json.load(open(args.mask_map))
        support = np.zeros_like(free)
        res_map = map_value['resolution']
        for surface in map_value['surfaces']:
            cells = rep.surface_cells(surface)
            for (ci, cj) in cells:
                x, y = (ci + 0.5) * res_map, (cj + 0.5) * res_map
                gi = int(math.floor((x - lo[0]) / args.res))
                gj = int(math.floor((y - lo[1]) / args.res))
                if 0 <= gi < nx and 0 <= gj < ny:
                    support[gi, gj] = True
        free = free & support

    dist = distance_transform(free)
    if args.any:
        seen = free.copy()
        reach = np.where(free, dist, -1)
    else:
        seen = reachable(free, (si, sj))
        reach = np.where(seen, dist, -1)
    best = int(reach.max())
    width = 2.0 * best * args.res
    idx = np.unravel_index(int(np.argmax(reach)), reach.shape)
    centre = (lo[0] + (idx[0] + 0.5) * args.res, lo[1] + (idx[1] + 0.5) * args.res)

    result = {
        'ok': width >= args.need,
        'pcd': os.path.basename(args.pcd),
        'resolution_m': args.res,
        'inflate_m': args.inflate,
        'obstacle_points': int(len(obstacles)),
        'grid': [nx, ny],
        'start': [sx, sy],
        'reachable_cells': int(seen.sum()),
        'free_cells': int(free.sum()),
        'max_inscribed_diameter_m': round(width, 3),
        'widest_point': [round(centre[0], 2), round(centre[1], 2)],
        'need_m': args.need,
        'verdict': 'PASS' if width >= args.need else 'FAIL',
    }
    print(json.dumps(result, ensure_ascii=False, indent=1))
    if args.out:
        open(args.out, 'w').write(json.dumps(result, ensure_ascii=False) + '\n')
    return 0 if result['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
