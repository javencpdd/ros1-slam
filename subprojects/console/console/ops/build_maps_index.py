#!/usr/bin/env python3
"""Build the console map library: preview PNGs + maps.json (read-only).

Sources (all on disk; no ROS):
  data/maps/lite3.yaml + lite3.pgm         2D occupancy grid (map_server)
  data/maps/mapped_m1.pcd                  SLAM point cloud (binary PCD)
  data/maps/m1_world_synth_v1.pcd          synthetic world cloud
  data/maps/mapped_nist.pcd                NIST cloud
  data/maps/*support.json                  v8 support maps (RLE surfaces)
  ws/src/.../two_floor_nav/maps/open.json  two_floor support map

Outputs (served by the console agent, not the docs site):
  scripts/console/ui/assets/data/maps.json
  scripts/console/ui/assets/maps/<id>.png

The mission page uses origin/resolution/width/height to turn a canvas click
into map coordinates. Point clouds are rendered top-down with a height colour;
support maps are painted surface by surface (higher z on top).  The canonical
world/map binding comes from scripts/console/world_registry.json; maps.json
carries both the display label and world_id.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import math
from pathlib import Path
import re
import os
import struct
import subprocess

import numpy as np
from PIL import Image

from world_registry import load_registry, map_bindings

CONSOLE = Path(__file__).resolve().parents[1]
_P3 = CONSOLE.parent
REPO = Path(os.environ.get('V8_REPO_ROOT') or (_P3.parents[1])).resolve()
SIM = Path(os.environ.get('V8_SIM_ROOT') or (_P3.parent / 'simulation')).resolve()
UI = CONSOLE / 'ui'


def rel(path):
    path = Path(path)
    for base in (REPO, SIM, CONSOLE):
        try:
            return str(path.relative_to(base))
        except ValueError:
            pass
    return str(path)
OUT = UI / 'assets/maps'
BASE = (7, 12, 18)


def colour(t):
    """Blue -> cyan -> green -> yellow -> red for t in [0, 1]."""
    t = max(0.0, min(1.0, t))
    stops = [(0.0, (30, 60, 140)), (0.3, (30, 170, 190)), (0.55, (80, 200, 90)),
             (0.78, (230, 200, 60)), (1.0, (230, 80, 60))]
    for (a, ca), (b, cb) in zip(stops, stops[1:]):
        if a <= t <= b:
            f = (t - a) / (b - a)
            return tuple(int(ca[i] + (cb[i] - ca[i]) * f) for i in range(3))
    return stops[-1][1]


def read_pcd(path: Path, max_points=300000):
    header = {}
    with path.open('rb') as stream:
        while True:
            line = stream.readline().decode('ascii', errors='replace').strip()
            if not line or line.startswith('#'):
                continue
            key, *rest = line.split()
            header[key] = rest
            if key == 'DATA':
                break
        data = stream.read()
    fields = header.get('FIELDS', [])
    sizes = [int(v) for v in header.get('SIZE', [])]
    types = header.get('TYPE', [])
    counts = [int(v) for v in header.get('COUNT', ['1'] * len(fields))]
    points = int(header.get('POINTS', ['0'])[0])
    if not ('x' in fields and 'y' in fields and 'z' in fields) or not points:
        return None
    dtype = {'F': 'f', 'I': 'i', 'U': 'u'}
    formats, names, offsets = [], [], []
    off = 0
    for name, size, typ, count in zip(fields, sizes, types, counts):
        code = dtype.get(typ, 'f') + str(size)
        formats.append((code, (count,)) if count > 1 else code)
        names.append(name)
        offsets.append(off)
        off += size * count
    kind = header.get('DATA', [''])[0]
    if kind == 'binary':
        np_dtype = np.dtype({'names': names, 'formats': formats,
                             'offsets': offsets, 'itemsize': off})
        arr = np.frombuffer(data, dtype=np_dtype, count=points)
        xyz = np.column_stack([arr['x'], arr['y'], arr['z']])
    else:
        rows = [line.split() for line in data.decode('ascii', errors='replace').splitlines()
                if line and not line.startswith('#')]
        xyz = np.array([[float(r[i]) for i in (fields.index('x'), fields.index('y'),
                                               fields.index('z'))] for r in rows])
    xyz = np.asarray(xyz, dtype=np.float64)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if len(xyz) > max_points:
        xyz = xyz[::max(1, len(xyz) // max_points)]
    return xyz


def cloud_preview(path: Path, map_id: str):
    xyz = read_pcd(path)
    if xyz is None or len(xyz) < 10:
        return None
    lo = xyz.min(axis=0)
    hi = xyz.max(axis=0)
    span = max(hi[0] - lo[0], hi[1] - lo[1], 1e-6)
    res = span / 900.0
    width = max(2, int(math.ceil((hi[0] - lo[0]) / res)))
    height = max(2, int(math.ceil((hi[1] - lo[1]) / res)))
    height = min(height, 1000)
    zmin, zmax = float(np.percentile(xyz[:, 2], 2)), float(np.percentile(xyz[:, 2], 98))
    if zmax <= zmin:
        zmax = zmin + 1e-3
    grid = np.full((height, width), np.nan)
    ix = np.clip(((xyz[:, 0] - lo[0]) / res).astype(int), 0, width - 1)
    iy = np.clip(((hi[1] - xyz[:, 1]) / res).astype(int), 0, height - 1)
    z = xyz[:, 2]
    order = np.argsort(z)
    for i in order:
        grid[iy[i], ix[i]] = z[i]
    rgb = np.zeros((height, width, 3), dtype=np.uint8)
    rgb[:] = BASE
    mask = np.isfinite(grid)
    rgb[mask] = np.array([colour((v - zmin) / (zmax - zmin)) for v in grid[mask]],
                         dtype=np.uint8)
    image = Image.fromarray(rgb)
    OUT.mkdir(parents=True, exist_ok=True)
    preview = OUT / (map_id + '.png')
    image.save(preview, optimize=True)
    stride = max(1, len(xyz) // 6000)
    sample = xyz[::stride]
    points_path = OUT / (map_id + '.points.json')
    points_path.write_text(json.dumps({
        'z_range': [round(zmin, 3), round(zmax, 3)],
        'points': [[round(float(v), 3) for v in row] for row in sample],
    }, separators=(',', ':')) + '\n', encoding='utf-8')
    return {
        'kind': 'pointcloud',
        'preview': 'assets/maps/' + preview.name,
        'points3d': 'assets/maps/' + points_path.name,
        'width': width,
        'height': height,
        'resolution': round(res, 6),
        'origin': [round(float(lo[0]), 4), round(float(lo[1]), 4)],
        'bbox': [[round(float(v), 3) for v in lo], [round(float(v), 3) for v in hi]],
        'points': int(len(xyz)),
        'z_range': [round(zmin, 3), round(zmax, 3)],
    }


def read_pgm(path: Path):
    with path.open('rb') as stream:
        magic = stream.readline().strip()
        if magic != b'P5':
            return None
        line = stream.readline()
        while line.startswith(b'#'):
            line = stream.readline()
        width, height = map(int, line.split())
        maxval = int(stream.readline().strip())
        data = np.frombuffer(stream.read(), dtype=np.uint8, count=width * height)
    return data.reshape((height, width)), maxval


def grid_preview(yaml_path: Path, map_id: str, image_path: Path):
    text = yaml_path.read_text(encoding='utf-8', errors='replace')
    res = float(re.search(r'^resolution:\s*([0-9.]+)', text, re.M).group(1))
    origin = [float(v) for v in re.search(r'^origin:\s*\[\s*([^\]]+)\]', text, re.M)
              .group(1).replace(',', ' ').split()]
    pgm = read_pgm(image_path)
    if pgm is None:
        return None
    data, _ = pgm
    height, width = data.shape
    rgb = np.full((height, width, 3), (210, 210, 210), dtype=np.uint8)
    rgb[data < 128] = (20, 24, 30)       # occupied
    rgb[data >= 250] = (245, 245, 245)   # free
    image = Image.fromarray(rgb)
    OUT.mkdir(parents=True, exist_ok=True)
    preview = OUT / (map_id + '.png')
    image.save(preview, optimize=True)
    return {
        'kind': '2d_grid',
        'preview': 'assets/maps/' + preview.name,
        'width': width,
        'height': height,
        'resolution': res,
        'origin': [round(origin[0], 4), round(origin[1], 4), round(origin[2], 4)],
        'files': [rel(yaml_path), rel(image_path)],
    }


def support_preview(path: Path, map_id: str):
    data = json.loads(path.read_text(encoding='utf-8'))
    extent = data.get('extent') or {}
    width, height = int(extent.get('width', 0)), int(extent.get('height', 0))
    res = float(data.get('resolution', 0.0))
    origin = data.get('origin') or [0, 0, 0]
    if not width or not height or not res:
        return None
    surfaces = []
    zs = [float(s.get('z_reference', 0) or 0) for s in data.get('surfaces', [])]
    zmin, zmax = min(zs) if zs else 0.0, max(zs) if zs else 0.0
    span = max(zmax - zmin, 1e-6)
    rgb = np.zeros((height, width, 3), dtype=np.uint8)
    rgb[:] = BASE
    for surface in sorted(data.get('surfaces', []), key=lambda s: float(s.get('z_reference', 0) or 0)):
        z = float(surface.get('z_reference', 0) or 0)
        runs = ((surface.get('cells') or {}).get('runs') or [])
        colour_z = colour((z - zmin) / span)
        for run in runs:
            if not (isinstance(run, list) and len(run) >= 3):
                continue
            row, col, length = int(run[0]) + height // 2, int(run[1]) + width // 2, int(run[2])
            px = height - 1 - row
            if 0 <= px < height:
                a, b = max(0, col), min(width, col + length)
                if b > a:
                    rgb[px, a:b] = colour_z
        fp = surface.get('footprint')
        if not runs and isinstance(fp, dict):
            # NIST-style support maps carry only footprint bounds; paint the
            # supported rectangle at its z_reference.
            c0 = int(round((float(fp['min_x']) - float(origin[0])) / res))
            c1 = int(round((float(fp['max_x']) - float(origin[0])) / res))
            r0 = int(round((float(fp['min_y']) - float(origin[1])) / res))
            r1 = int(round((float(fp['max_y']) - float(origin[1])) / res))
            for row in range(max(0, r0), min(height, r1)):
                px = height - 1 - row
                a, b = max(0, c0), min(width, c1)
                if b > a:
                    rgb[px, a:b] = colour_z
        surfaces.append({
            'id': surface.get('surface_id'),
            'z': round(z, 4),
            'footprint': surface.get('footprint'),
            'traversability': surface.get('traversability'),
        })
    image = Image.fromarray(rgb)
    OUT.mkdir(parents=True, exist_ok=True)
    preview = OUT / (map_id + '.png')
    image.save(preview, optimize=True)
    pts = []
    for surface in data.get('surfaces', []):
        z = float(surface.get('z_reference', 0) or 0)
        runs = ((surface.get('cells') or {}).get('runs') or [])
        for run in runs:
            if not (isinstance(run, list) and len(run) >= 3):
                continue
            row0, col0, length = int(run[0]) + height // 2, int(run[1]) + width // 2, int(run[2])
            for col in range(col0, col0 + length):
                pts.append([round(float(origin[0]) + (col + 0.5) * res, 3),
                            round(float(origin[1]) + (row0 + 0.5) * res, 3), round(z, 3)])
        fp = surface.get('footprint')
        if not runs and isinstance(fp, dict):
            c0 = int(round((float(fp['min_x']) - float(origin[0])) / res))
            c1 = int(round((float(fp['max_x']) - float(origin[0])) / res))
            r0 = int(round((float(fp['min_y']) - float(origin[1])) / res))
            r1 = int(round((float(fp['max_y']) - float(origin[1])) / res))
            for row in range(max(0, r0), min(height, r1)):
                for col in range(max(0, c0), min(width, c1)):
                    pts.append([round(float(origin[0]) + (col + 0.5) * res, 3),
                                round(float(origin[1]) + (row + 0.5) * res, 3), round(z, 3)])
    stride = max(1, len(pts) // 8000)
    sample = pts[::stride]
    points_path = OUT / (map_id + '.points.json')
    points_path.write_text(json.dumps({
        'z_range': [round(zmin, 3), round(zmax, 3)], 'points': sample,
    }, separators=(',', ':')) + '\n', encoding='utf-8')
    return {
        'kind': 'support_map',
        'points3d': 'assets/maps/' + points_path.name,
        'preview': 'assets/maps/' + preview.name,
        'width': width,
        'height': height,
        'resolution': res,
        'origin': [round(float(origin[0]), 4), round(float(origin[1]), 4), round(float(origin[2]), 4)],
        'surfaces': surfaces,
        'frame': data.get('frame'),
        'files': [rel(path)],
    }


def maps():
    registry = load_registry()
    bindings = map_bindings(registry)
    found = []
    lite3_yaml = REPO / 'data/maps/lite3.yaml'
    lite3_pgm = REPO / 'data/maps/lite3.pgm'
    if lite3_yaml.exists() and lite3_pgm.exists():
        info = grid_preview(lite3_yaml, 'lite3', lite3_pgm)
        if info:
            found.append({'id': 'lite3', 'label': 'Lite3 2D 栅格（真机 SLAM 图）',
                          'frame': 'map', 'world': '真机/仿真 2D 栅格',
                          'notes': 'map_server PGM + YAML；2D MBF 可用',
                          **info})
    for name, label, notes in (
        ('mapped_m1', 'm1 建图 PCD（SLAM）', 'faster-lio / GT 位姿投影点云；只读预览'),
        ('m1_world_synth_v1', 'm1 合成世界点云', 'm1 世界点云；只读预览'),
        ('mapped_nist', 'NIST 建图 PCD', 'NIST 楼梯场景点云；只读预览'),
    ):
        pcd = REPO / 'data/maps' / (name + '.pcd')
        if not pcd.exists():
            continue
        info = cloud_preview(pcd, name)
        if info:
            found.append({'id': name, 'label': label, 'frame': 'map',
                          'notes': notes, 'files': [rel(pcd)], **info})
    support_paths = [
        (SIM / 'ws/src/lite3_gazebo_sim/scenarios/two_floor_nav/maps/open.json',
         'two_floor_nav', 'two_floor_nav 支撑图（3D）',
         '声明几何支撑图（open.world；不是 demo_two_floor）；3D 全链目标/起点可选用'),
        (REPO / 'data/maps/nist_stairs_120_s020x170_support.json',
         'nist_stairs', 'NIST 楼梯支撑图（3D）',
         'NIST 楼梯支撑图；只读预览'),
    ]
    for path, map_id, label, notes in support_paths:
        if not path.exists():
            continue
        info = support_preview(path, map_id)
        if info:
            found.append({'id': map_id, 'label': label, 'frame': 'map',
                          'notes': notes, **info})
    console_dir = REPO / 'data/maps/console'
    for yaml_path in sorted(console_dir.glob('*/map.yaml')):
        map_id = 'console_' + yaml_path.parent.name
        image_path = yaml_path.parent / 'map.pgm'
        if not image_path.exists():
            continue
        info = grid_preview(yaml_path, map_id, image_path)
        if not info:
            continue
        meta_path = yaml_path.parent / 'meta.json'
        meta = {}
        if meta_path.exists():
            try:
                meta = json.loads(meta_path.read_text(encoding='utf-8'))
            except ValueError:
                meta = {}
        info['files'] = [rel(yaml_path), rel(image_path)]
        found.append({'id': map_id, 'label': '重栅格化 / ' + yaml_path.parent.name,
                      'frame': 'map', 'world': meta.get('source_pcd', '—'),
                      'notes': '由控制台重栅格化；源 ' + meta.get('source_pcd', '—'),
                      'source_pcd': meta.get('source_pcd'),
                      'regrid': True, **info})
    imported_dir = REPO / 'data/maps/imported'
    for pcd in sorted(imported_dir.glob('*.pcd')):
        if pcd.stat().st_size > 20 * 1024 * 1024:
            continue
        map_id = 'imported_' + pcd.stem
        info = cloud_preview(pcd, map_id)
        if info:
            found.append({'id': map_id, 'label': '导入 / ' + pcd.stem,
                          'frame': 'map', 'world': '—',
                          'notes': '从 data/maps/imported/ 导入的点云',
                          'files': [rel(pcd)], **info})
    for item in found:
        binding = bindings.get(item['id'])
        if binding:
            item['world_id'] = binding['world_id']
            item['world'] = binding['label']
        else:
            item.setdefault('world', '—')
            item['world_id'] = None
    return found


def git_head():
    try:
        return subprocess.check_output(
            ['git', '-C', str(REPO), 'rev-parse', '--short', 'HEAD'],
            text=True, stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.SubprocessError):
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', default=str(UI / 'assets/data/maps.json'))
    args = parser.parse_args()
    items = maps()
    payload = {
        'schema': 'v8_maps/v1',
        'generated_at': dt.datetime.now().astimezone().isoformat(timespec='seconds'),
        'git_head': git_head(),
        'maps': items,
    }
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, ensure_ascii=False, indent=1) + '\n', encoding='utf-8')
    print('wrote %s' % out)
    for item in items:
        print('%-22s %-12s %sx%s res=%s' % (item['id'], item['kind'],
                                           item.get('width'), item.get('height'),
                                           item.get('resolution')))


if __name__ == '__main__':
    main()
