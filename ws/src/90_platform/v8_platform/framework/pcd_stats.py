#!/usr/bin/env python3
"""PCD statistics and post-hoc LIO-map path clearance.

The clearance is a sensor/map check, not a physics collision proof: it transforms
the GT path into the LIO map frame using the first-pose offset/yaw and measures
the nearest map point in a horizontal band around the robot.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

try:
    from scipy.spatial import cKDTree
except Exception:  # pragma: no cover - optional acceleration
    cKDTree = None

_SIZE_TYPE = {
    'F': {4: '<f4', 8: '<f8'},
    'U': {1: '<u1', 2: '<u2', 4: '<u4', 8: '<u8'},
    'I': {1: '<i1', 2: '<i2', 4: '<i4', 8: '<i8'},
}


def read_pcd(path):
    """Return an (N, 3) float array from an ascii or binary PCD."""
    raw = Path(path).read_bytes()
    data_pos = raw.find(b'DATA')
    if data_pos < 0:
        raise ValueError('PCD has no DATA line')
    line_end = raw.find(b'\n', data_pos)
    if line_end < 0:
        raise ValueError('PCD DATA line is not terminated')
    header = raw[:line_end].decode('ascii', 'replace')
    fields, sizes, types, counts = [], [], [], []
    points = None
    data_mode = None
    for line in header.splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        parts = line.split()
        key = parts[0].upper()
        if key == 'FIELDS':
            fields = parts[1:]
        elif key == 'SIZE':
            sizes = [int(value) for value in parts[1:]]
        elif key == 'TYPE':
            types = parts[1:]
        elif key == 'COUNT':
            counts = [int(value) for value in parts[1:]]
        elif key == 'POINTS':
            points = int(parts[1])
        elif key == 'DATA':
            data_mode = parts[1].lower()
    if not fields or points is None or data_mode is None:
        raise ValueError('PCD header is incomplete')
    if len(sizes) != len(fields) or len(types) != len(fields):
        raise ValueError('PCD field metadata mismatch')
    if not counts:
        counts = [1] * len(fields)
    for name in ('x', 'y', 'z'):
        if name not in fields:
            raise ValueError('PCD is missing field ' + name)
    data = raw[line_end + 1:]
    if data_mode == 'ascii':
        rows = [line.split() for line in data.decode('ascii', 'replace').splitlines()
                if line.strip()]
        table = np.array(rows, dtype=float)
        if table.ndim != 2 or table.shape[1] < len(fields):
            raise ValueError('ASCII PCD row width mismatch')
        return table[:, [fields.index('x'), fields.index('y'), fields.index('z')]]
    if data_mode != 'binary':
        raise ValueError('unsupported PCD DATA mode: ' + data_mode)
    if any(count != 1 for count in counts):
        raise ValueError('PCD COUNT != 1 is not supported')
    dtype = []
    for name, size, kind in zip(fields, sizes, types):
        code = _SIZE_TYPE.get(kind.upper(), {}).get(size)
        if code is None:
            raise ValueError('unsupported PCD field type %s/%s' % (kind, size))
        dtype.append((name, code))
    table = np.frombuffer(data, dtype=np.dtype(dtype), count=points)
    return np.column_stack([table['x'], table['y'], table['z']]).astype(float)


def stats(points, cell_m=0.5, z_band_m=0.1):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if not len(points):
        return {'points': 0}
    low, high = points.min(axis=0), points.max(axis=0)
    area = float((high[0] - low[0]) * (high[1] - low[1]))
    xy_cells = np.unique(np.floor(points[:, :2] / cell_m).astype(np.int64), axis=0)
    z_bands = np.unique(np.floor((points[:, 2] - low[2]) / z_band_m).astype(np.int64))
    occupancy = (len(xy_cells) * cell_m * cell_m / area) if area > 0 else None
    return {
        'points': int(len(points)),
        'bbox_min': [float(v) for v in low],
        'bbox_max': [float(v) for v in high],
        'xy_bbox_area_m2': area,
        'z_min': float(low[2]),
        'z_max': float(high[2]),
        'z_bands_0_1m': int(len(z_bands)),
        'xy_cells_0_5m': int(len(xy_cells)),
        'xy_cell_coverage_fraction': min(1.0, occupancy) if occupancy is not None else None,
        'point_density_per_m2': (len(points) / area) if area > 0 else None,
    }


def _load_path(path):
    value = json.loads(Path(path).read_text(encoding='utf-8'))
    points = value['waypoints']
    if len(points) < 2:
        raise ValueError('path needs at least two waypoints')
    return points


def _sample_path(waypoints, sample_m):
    samples = []
    for first, second in zip(waypoints, waypoints[1:]):
        dx = second['x'] - first['x']
        dy = second['y'] - first['y']
        dz = second['z'] - first['z']
        length = math.hypot(dx, dy)
        count = max(1, int(math.ceil(length / sample_m)))
        for index in range(count):
            ratio = index / float(count)
            samples.append((first['x'] + ratio * dx, first['y'] + ratio * dy,
                            first['z'] + ratio * dz))
    last = waypoints[-1]
    samples.append((last['x'], last['y'], last['z']))
    return np.asarray(samples, dtype=float)


def clearance(points, path, compare=None, radius_m=0.35,
              sample_m=0.1, z_min_m=0.15, z_max_m=1.2):
    """Nearest XY distance from sampled GT path to the LIO map points."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    waypoints = _load_path(path)
    path_xyz = _sample_path(waypoints, sample_m)
    compare = compare or {}
    offset = compare.get('origin_offset') or [0.0, 0.0, 0.0]
    dyaw = float(compare.get('origin_yaw_rad') or 0.0)
    gt0 = np.array([waypoints[0]['x'], waypoints[0]['y'], waypoints[0]['z']], dtype=float)
    est0 = gt0 + np.asarray(offset, dtype=float)
    cos_y, sin_y = math.cos(dyaw), math.sin(dyaw)
    rel = path_xyz[:, :2] - gt0[:2]
    path_map = np.column_stack([
        cos_y * rel[:, 0] - sin_y * rel[:, 1] + est0[0],
        sin_y * rel[:, 0] + cos_y * rel[:, 1] + est0[1],
        path_xyz[:, 2] + offset[2],
    ])
    band = ((points[:, 2] >= est0[2] + z_min_m)
            & (points[:, 2] <= est0[2] + z_max_m))
    band_points = points[band]
    if not len(band_points):
        return {'samples': len(path_map), 'map_points_used': 0,
                'ok': False, 'reason': 'no map points in z band'}
    if cKDTree is not None:
        distances, _ = cKDTree(band_points[:, :2]).query(path_map[:, :2])
    else:  # pragma: no cover
        distances = np.array([
            np.min(np.hypot(band_points[:, 0] - x, band_points[:, 1] - y))
            for x, y in path_map[:, :2]
        ])
    return {
        'samples': int(len(path_map)),
        'map_points_used': int(len(band_points)),
        'radius_m': radius_m,
        'sample_m': sample_m,
        'z_band_m': [z_min_m, z_max_m],
        'min_clearance_m': float(np.min(distances)),
        'p05_clearance_m': float(np.percentile(distances, 5)),
        'below_radius_samples': int(np.sum(distances < radius_m)),
        'ok': bool(np.min(distances) >= radius_m),
        'boundary': 'post-hoc LIO-map clearance, not physics collision proof',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pcd', required=True)
    parser.add_argument('--path')
    parser.add_argument('--compare', help='survey JSON with compare.origin_offset/origin_yaw_rad')
    parser.add_argument('--radius', type=float, default=0.35)
    parser.add_argument('--sample-m', type=float, default=0.1)
    parser.add_argument('--z-min', type=float, default=0.15)
    parser.add_argument('--z-max', type=float, default=1.2)
    parser.add_argument('--out', default='')
    args = parser.parse_args()
    points = read_pcd(args.pcd)
    result = {'pcd': str(Path(args.pcd).resolve()), 'stats': stats(points)}
    if args.path:
        compare = None
        if args.compare:
            raw = json.loads(Path(args.compare).read_text(encoding='utf-8'))
            compare = raw.get('compare')
        result['clearance'] = clearance(
            points, args.path, compare=compare, radius_m=args.radius,
            sample_m=args.sample_m, z_min_m=args.z_min, z_max_m=args.z_max)
    text = json.dumps(result, ensure_ascii=False, indent=1) + '\n'
    if args.out:
        Path(args.out).write_text(text, encoding='utf-8')
    print(text)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
