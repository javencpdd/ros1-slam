#!/usr/bin/env python3
"""Stair/step look-ahead on a support map (pure Python, no ROS).

Walks the planned path for a bounded distance and reports the first explicit step
edge ahead: height, distance and direction. This is the perception half of an
automatic gait switch; the gait command itself belongs to the chassis bridge.
"""
import argparse
import json
import math
import sys
from pathlib import Path


def _inside(surface, x, y, margin=0.0):
    box = surface['footprint']
    return (box['min_x'] + margin <= x <= box['max_x'] - margin
            and box['min_y'] + margin <= y <= box['max_y'] - margin)


def _surface_at(surfaces, x, y, z, z_tol=0.30):
    best, best_dz = None, None
    for surface in surfaces:
        if not _inside(surface, x, y):
            continue
        dz = abs(surface['z_reference'] - z)
        if dz <= z_tol and (best_dz is None or dz < best_dz):
            best, best_dz = surface, dz
    return best


def _top_surface_at(surfaces, x, y, z_limit):
    """Highest supported surface under this XY that the robot could step onto."""
    best = None
    for surface in surfaces:
        if not _inside(surface, x, y):
            continue
        if surface['z_reference'] > z_limit:
            continue
        if best is None or surface['z_reference'] > best['z_reference']:
            best = surface
    return best


def _has_edge(surface, target_id):
    return any(edge.get('to') == target_id and edge.get('kind') in ('step', 'slope')
               for edge in surface.get('edges', []))


def _path_points(pose, path):
    if path:
        return [tuple(point[:3]) for point in path]
    return [(pose[0], pose[1], pose[2])]


def stair_ahead(map_value, pose, path=None, lookahead_m=2.0, min_step_m=0.05,
                sample_m=0.05):
    """Return the first step edge within lookahead along the path.

    pose/path are (x, y, z) in map frame. Result:
    {ahead, distance_m, step_m, direction, from_surface, to_surface, at}
    """
    surfaces = [s for s in map_value['surfaces'] if s.get('traversability') == 'supported']
    points = _path_points(pose, path)
    if not points:
        return {'ahead': False, 'reason': 'empty_path'}
    start = _surface_at(surfaces, pose[0], pose[1], pose[2])
    if start is None:
        return {'ahead': False, 'reason': 'pose_off_support'}
    current = start
    travelled = 0.0
    for index in range(1, len(points)):
        a, b = points[index - 1], points[index]
        segment = math.hypot(b[0] - a[0], b[1] - a[1])
        steps = max(1, int(math.ceil(segment / sample_m)))
        for step in range(1, steps + 1):
            ratio = step / float(steps)
            x = a[0] + (b[0] - a[0]) * ratio
            y = a[1] + (b[1] - a[1]) * ratio
            travelled += segment / steps
            if travelled > lookahead_m + 1e-9:
                break
            here = _top_surface_at(surfaces, x, y, current['z_reference'] + 0.5)
            if here is None or here['surface_id'] == current['surface_id']:
                continue
            rise = here['z_reference'] - current['z_reference']
            if abs(rise) < min_step_m or not _has_edge(current, here['surface_id']):
                continue
            return {'ahead': True, 'distance_m': round(travelled, 3),
                    'step_m': round(abs(rise), 4),
                    'direction': 'up' if rise > 0 else 'down',
                    'from_surface': current['surface_id'],
                    'to_surface': here['surface_id'],
                    'at': [round(x, 3), round(y, 3)]}
        if travelled >= lookahead_m:
            break
    return {'ahead': False, 'reason': 'no_step_within_lookahead',
            'scanned_m': round(min(travelled, lookahead_m), 3)}


def _load_path(path):
    if not path:
        return None
    value = json.loads(Path(path).read_text(encoding='utf-8'))
    if isinstance(value, dict):
        value = value.get('path') or value.get('poses') or []
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True)
    parser.add_argument('--pose', required=True, help='x,y,z')
    parser.add_argument('--path', default='')
    parser.add_argument('--lookahead', type=float, default=2.0)
    parser.add_argument('--min-step', type=float, default=0.05)
    args = parser.parse_args()
    map_value = json.loads(Path(args.map).read_text(encoding='utf-8'))
    pose = tuple(float(v) for v in args.pose.split(','))
    result = stair_ahead(map_value, pose, _load_path(args.path),
                         lookahead_m=args.lookahead, min_step_m=args.min_step)
    print(json.dumps(result, ensure_ascii=False, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
