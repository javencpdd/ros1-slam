#!/usr/bin/env python3
"""Regrid a point cloud into a 2D occupancy map (map_server PGM+YAML).

This is the console's "regenerate 2D grid" action; it never modifies the source
PCD. Output goes to data/maps/console/<name>/ (map.pgm, map.yaml, preview.png,
meta.json). Occupied = cells with at least --min-points returns in the z band;
unknown = no returns; free is not inferred (a 2D projection cannot tell free).

Usage:
  python3 scripts/ops/regrid_map.py --pcd data/maps/mapped_m1.pcd \
      --name m1_r05 --resolution 0.05 --min-z -1.0 --max-z 2.0
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import math
from pathlib import Path
import sys

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_maps_index as bmi  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pcd', required=True)
    parser.add_argument('--name', required=True)
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--min-z', type=float, default=-1.0)
    parser.add_argument('--max-z', type=float, default=2.0)
    parser.add_argument('--min-points', type=int, default=2)
    args = parser.parse_args()
    pcd = Path(args.pcd)
    if not pcd.is_absolute():
        pcd = ROOT / pcd
    xyz = bmi.read_pcd(pcd)
    if xyz is None or len(xyz) < 10:
        print('no points in %s' % pcd)
        return 2
    mask = (xyz[:, 2] >= args.min_z) & (xyz[:, 2] <= args.max_z)
    xyz = xyz[mask]
    if len(xyz) < 10:
        print('no points after z band')
        return 2
    res = args.resolution
    lo = xyz.min(axis=0)
    hi = xyz.max(axis=0)
    width = max(1, int(math.ceil((hi[0] - lo[0]) / res)))
    height = max(1, int(math.ceil((hi[1] - lo[1]) / res)))
    cells = np.zeros((height, width), dtype=np.int32)
    ix = np.clip(((xyz[:, 0] - lo[0]) / res).astype(int), 0, width - 1)
    iy = np.clip(((xyz[:, 1] - lo[1]) / res).astype(int), 0, height - 1)
    np.add.at(cells, (iy, ix), 1)
    occupied = cells >= max(1, args.min_points)
    pgm = np.full((height, width), 205, dtype=np.uint8)   # unknown
    pgm[occupied] = 0
    pgm = pgm[::-1]          # PGM/map_server row 0 is the top (max y)
    occupied = occupied[::-1]
    out_dir = ROOT / 'data/maps/console' / args.name
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / 'map.pgm').write_bytes(
        b'P5\n%d %d\n255\n' % (width, height) + pgm.tobytes())
    (out_dir / 'map.yaml').write_text(
        'image: map.pgm\nresolution: %.6f\norigin: [%.6f, %.6f, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'
        % (res, float(lo[0]), float(lo[1])), encoding='utf-8')
    rgb = np.full((height, width, 3), (205, 205, 205), dtype=np.uint8)
    rgb[occupied] = (20, 24, 30)
    image = Image.fromarray(rgb)
    image.save(out_dir / 'preview.png', optimize=True)
    meta = {
        'name': args.name,
        'source_pcd': str(pcd.relative_to(ROOT)) if pcd.is_relative_to(ROOT) else str(pcd),
        'resolution': res,
        'min_z': args.min_z,
        'max_z': args.max_z,
        'min_points': args.min_points,
        'width': width,
        'height': height,
        'origin': [float(lo[0]), float(lo[1]), 0.0],
        'occupied_cells': int(occupied.sum()),
        'points_used': int(len(xyz)),
        'created_at': dt.datetime.now().astimezone().isoformat(timespec='seconds'),
    }
    (out_dir / 'meta.json').write_text(json.dumps(meta, ensure_ascii=False, indent=1),
                                       encoding='utf-8')
    print('wrote %s (%dx%d, occupied=%d, points=%d)' % (
        out_dir, width, height, occupied.sum(), len(xyz)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
