#!/usr/bin/env python3
"""Turn a faster_lio PCD map into a support_map JSON and an interactive 3D app.

The PCD is expected in the LIO camera_init frame; for a session started at the
Gazebo origin, camera_init and map coincide. This is an offline mapper, not an
online perception node: the operator maps with keyboard teleop, stops the SLAM
node to flush the PCD, then runs this script and starts app_runner.
"""
import argparse
import hashlib
import math
import io
import json
import re
import sys
from pathlib import Path

import numpy as np

import paths

HERE = Path(__file__).resolve().parent
ROOT = paths.repo_root(__file__)
PLATFORM = ROOT / 'ws/src/90_platform/v8_platform'
sys.path.insert(0, str(PLATFORM / 'representation'))
sys.path.insert(0, str(PLATFORM / 'regression'))
import gazebo3d_mapping as g3d  # noqa: E402
import support_map as rep  # noqa: E402


def read_pcd(path):
    raw = Path(path).read_bytes()
    header, position = {}, 0
    while True:
        end = raw.find(b'\n', position)
        if end < 0:
            raise ValueError('PCD header has no DATA line')
        line = raw[position:end].decode('ascii', 'ignore').strip()
        position = end + 1
        if not line or line.startswith('#'):
            continue
        parts = line.split()
        if parts[0].upper() == 'DATA':
            data_type = parts[1].lower()
            break
        header[parts[0].upper()] = parts[1:]
    count = int(header['POINTS'][0])
    fields = header['FIELDS']
    sizes = [int(value) for value in header['SIZE']]
    types = header['TYPE']
    counts = [int(value) for value in header['COUNT']]
    if data_type == 'ascii':
        table = np.loadtxt(io.BytesIO(raw[position:]), usecols=(0, 1, 2), ndmin=2)
        return np.asarray(table, dtype=float)
    if data_type != 'binary':
        raise ValueError('unsupported PCD DATA type: ' + data_type)
    formats = {'F': '<f4', 'U': '<u4', 'I': '<i4'}
    names, formats_list, offsets, offset = [], [], [], 0
    for name, size, kind, repeat in zip(fields, sizes, types, counts):
        names.append(name)
        formats_list.append(formats[kind] if size == 4 else ('<f8' if kind == 'F' else '<u8'))
        offsets.append(offset)
        offset += size * repeat
    dtype = np.dtype({'names': names, 'formats': formats_list,
                      'offsets': offsets, 'itemsize': offset})
    table = np.frombuffer(raw[position:position + count * dtype.itemsize],
                          dtype=dtype, count=count)
    return np.column_stack([table['x'], table['y'], table['z']]).astype(float)


def prepare(points, voxel, drop_self, z_min, z_max, x_min, x_max, y_min, y_max,
            nist_profile=False):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    if x_min is not None:
        points = points[points[:, 0] >= x_min]
    if x_max is not None:
        points = points[points[:, 0] <= x_max]
    if y_min is not None:
        points = points[points[:, 1] >= y_min]
    if y_max is not None:
        points = points[points[:, 1] <= y_max]
    if z_min is not None:
        points = points[points[:, 2] >= z_min]
    if z_max is not None:
        points = points[points[:, 2] <= z_max]
    if nist_profile:
        # NIST stair geometry: keep the front ground strip and the stair treads;
        # drop ground-plane points underneath the staircase.
        x, y, z = points[:, 0], points[:, 1], points[:, 2]
        region = ((np.abs(x) <= 1.1) & (y >= -3.2) & (y <= 1.7)
                  & (z >= -0.15) & (z <= 1.05))
        ground = z < 0.08
        ground &= y <= -1.7
        stairs = z >= 0.08
        stairs &= (np.abs(x) <= 1.15) & (y >= -1.7) & (y <= 1.15)
        points = points[region & (ground | stairs)]
    if drop_self:
        inside = ((np.abs(points[:, 0]) <= 0.35) & (np.abs(points[:, 1]) <= 0.35)
                  & (points[:, 2] >= -0.20) & (points[:, 2] <= 0.50))
        points = points[~inside]
    return g3d.voxel_downsample(points, voxel)


def _blocked(point, obstacles, margin):
    for obstacle in obstacles or []:
        lo, hi = obstacle['min'], obstacle['max']
        if (lo[0] - margin <= point[0] <= hi[0] + margin
                and lo[1] - margin <= point[1] <= hi[1] + margin):
            return True
    return False


def choose_start_goal(surface, start_xy=None, obstacles=None, margin=0.55,
                      resolution=None, max_goal_m=4.0):
    box = surface['footprint']
    z = surface['z_reference']
    start = None
    cells = rep.surface_cells(surface) or set()
    if cells and resolution:
        from collections import deque
        free = {}
        for i, j in cells:
            x, y = (i + 0.5) * resolution, (j + 0.5) * resolution
            if not _blocked((x, y, z), obstacles or [], margin):
                free[(i, j)] = (x, y)
        target = start_xy
        if target is None:
            target = ((0.0, 0.0)
                      if box['min_x'] <= 0.0 <= box['max_x']
                      and box['min_y'] <= 0.0 <= box['max_y']
                      else ((box['min_x'] + box['max_x']) / 2.0,
                            (box['min_y'] + box['max_y']) / 2.0))
        if free:
            start_cell = min(free, key=lambda cell: (free[cell][0] - target[0]) ** 2
                             + (free[cell][1] - target[1]) ** 2)
            seen = {start_cell}
            queue = deque([start_cell])
            while queue:
                i, j = queue.popleft()
                for step in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nxt = (i + step[0], j + step[1])
                    if nxt in free and nxt not in seen:
                        seen.add(nxt)
                        queue.append(nxt)
            sx, sy = free[start_cell]
            candidates = [cell for cell in seen
                          if math.hypot(free[cell][0] - sx, free[cell][1] - sy) <= max_goal_m]
            goal_cell = max(candidates, key=lambda cell: math.hypot(
                free[cell][0] - sx, free[cell][1] - sy))
            start = (sx, sy, z)
            goal = (free[goal_cell][0], free[goal_cell][1], z)
            return start, goal
    if start_xy is not None:
        start = (float(start_xy[0]), float(start_xy[1]), z)
    elif box['min_x'] <= 0.0 <= box['max_x'] and box['min_y'] <= 0.0 <= box['max_y']:
        start = (0.0, 0.0, z)
    else:
        start = ((box['min_x'] + box['max_x']) / 2.0,
                 (box['min_y'] + box['max_y']) / 2.0, z)
    candidates = [
        (box['min_x'] + 0.4, start[1], z), (box['max_x'] - 0.4, start[1], z),
        (start[0], box['min_y'] + 0.4, z), (start[0], box['max_y'] - 0.4, z),
        ((box['min_x'] + box['max_x']) / 2.0, (box['min_y'] + box['max_y']) / 2.0, z),
    ]
    inside = [point for point in candidates if box['min_x'] <= point[0] <= box['max_x']
              and box['min_y'] <= point[1] <= box['max_y']]
    goal = max(inside, key=lambda point: (point[0] - start[0]) ** 2
               + (point[1] - start[1]) ** 2) if inside else start
    distance = math.hypot(goal[0] - start[0], goal[1] - start[1])
    if distance > 4.0:
        scale = 4.0 / distance
        goal = (start[0] + (goal[0] - start[0]) * scale,
                start[1] + (goal[1] - start[1]) * scale, z)
    if obstacles and _blocked(goal, obstacles, margin):
        steps = 40
        for index in range(steps + 1):
            ratio = 1.0 - index / float(steps)
            candidate = (start[0] + (goal[0] - start[0]) * ratio,
                         start[1] + (goal[1] - start[1]) * ratio, z)
            if not _blocked(candidate, obstacles, margin):
                goal = candidate
                break
    return start, goal


def write_app(map_id, version, map_path, task_path, app_path, world, model_path):
    app = {
        'schema': 'navigation_app/v1',
        'name': '%s_3d' % map_id,
        'graph': 'ws/src/90_platform/v8_platform/framework/fixtures/graphs/graph_3d_gazebo_app_mapped.yaml',
        'map': {'id': map_id, 'version': version},
        'task': str(task_path.relative_to(ROOT)),
        'localization': {'source': 'carrier'},
        'carrier': {
            'backend': 'gazebo', 'model_name': 'app_carrier', 'center_z': 0.1,
            'snap_m': 0.03, 'spawn': True,
            'world_launch': 'ws/src/95_simulation/lite3_gazebo_sim/launch/app_world.launch',
            'world': world, 'model_path': model_path,
        },
        'visualization': {'markers': True, 'rviz': False},
        'global': {'budget': 20000, 'min_clearance_m': 0.4},
    }
    app_path.write_text(json.dumps(app, ensure_ascii=False, indent=1) + '\n', encoding='utf-8')


def update_registry(map_id, version, map_path, sha, surfaces, label):
    registry_path = ROOT / 'data/maps/registry.json'
    registry = json.loads(registry_path.read_text(encoding='utf-8'))
    entries = [entry for entry in registry['maps']
               if not (entry['map_id'] == map_id and int(entry['version']) == version)]
    entries.append({
        'map_id': map_id, 'version': version,
        'path': str(map_path.relative_to(ROOT)), 'frame': 'map',
        'surfaces': surfaces, 'sha256': sha, 'label': label,
    })
    registry['maps'] = entries
    registry_path.write_text(json.dumps(registry, ensure_ascii=False, indent=1) + '\n',
                             encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pcd', default='')
    parser.add_argument('--map-id', default='mapped_m1')
    parser.add_argument('--version', type=int, default=1)
    parser.add_argument('--resolution', type=float, default=0.10)
    parser.add_argument('--voxel', type=float, default=0.03)
    parser.add_argument('--z-min', type=float, default=None)
    parser.add_argument('--z-max', type=float, default=None)
    parser.add_argument('--x-min', type=float, default=None)
    parser.add_argument('--x-max', type=float, default=None)
    parser.add_argument('--y-min', type=float, default=None)
    parser.add_argument('--y-max', type=float, default=None)
    parser.add_argument('--drop-self', action='store_true', default=True)
    parser.add_argument('--nist-profile', action='store_true',
                        help='keep only NIST front ground and stair treads')
    parser.add_argument('--min-horizontal', type=float, default=0.8)
    parser.add_argument('--min-side', type=float, default=0.20)
    parser.add_argument('--min-area', type=float, default=0.30)
    parser.add_argument('--xy-gap', type=float, default=0.10)
    parser.add_argument('--max-z-step', type=float, default=0.25)
    parser.add_argument('--z-quantize', type=float, default=0.0,
                        help='quantize z to this step before surface extraction; '
                             'real lidar z noise needs it, synthetic clouds do not')
    parser.add_argument('--surface-z-min', type=float, default=-0.06,
                        help='support band lower bound (default tuned for synthetic scenes)')
    parser.add_argument('--surface-z-max', type=float, default=1.05,
                        help='support band upper bound (default tuned for synthetic scenes)')
    parser.add_argument('--min-cells', type=int, default=4,
                        help='minimum connected cells per surface (coarse cells need fewer)')
    parser.add_argument('--goal-margin', type=float, default=0.55,
                        help='clearance from obstacle boxes when picking start/goal')
    parser.add_argument('--goal-max', type=float, default=4.0,
                        help='maximum straight-line start->goal distance for the generated task')
    parser.add_argument('--obstacle-min-z', type=float, default=None,
                        help='turn rejected surfaces at or above this z into obstacle boxes '
                             '(real scenes: furniture tops must not become ceilings)')
    parser.add_argument('--obstacle-mode', choices=('none', 'surfaces', 'full', 'both'),
                        default='surfaces',
                        help='surfaces=refined rejected surfaces; full=full-cloud layer '
                             '(walls included); both=union')
    parser.add_argument('--obstacle-cell', type=float, default=0.20,
                        help='full-cloud obstacle XY cell size')
    parser.add_argument('--obstacle-min-points', type=int, default=5)
    parser.add_argument('--obstacle-band', default='',
                        help='full-cloud obstacle z band lo,hi (default: surface_z_min+0.10, surface_z_max+1.45)')
    parser.add_argument('--obstacle-band-m', type=float, default=0.40,
                        help='full-cloud vertical layer thickness')
    parser.add_argument('--z-align', action='store_true',
                        help='shift every surface/obstacle so the lowest support is z=0')
    parser.add_argument('--absorb-near-floor', type=float, default=0.0,
                        help='merge supports this close above the floor into the floor surface')
    parser.add_argument('--map', default='', help='existing map JSON for --task-only')
    parser.add_argument('--task-only', action='store_true',
                        help='write a task for an existing map (start from --start-x/--start-y)')
    parser.add_argument('--out', default='')
    parser.add_argument('--register', action='store_true')
    parser.add_argument('--write-app', action='store_true')
    parser.add_argument('--world',
                        default='ws/src/95_simulation/lite3_gazebo_sim/worlds/m1_world.world')
    parser.add_argument('--model-path', default='data/maps/worlds')
    parser.add_argument('--start-x', type=float, default=None)
    parser.add_argument('--start-y', type=float, default=None)
    args = parser.parse_args()
    if not re.fullmatch(r'[a-z0-9_]+', args.map_id):
        raise SystemExit('map_id must be [a-z0-9_]+')
    if args.task_only:
        if not args.map:
            raise SystemExit('--task-only needs --map')
        map_path = Path(args.map).resolve()
        value = json.loads(map_path.read_text(encoding='utf-8'))
        largest = max(value['surfaces'],
                      key=lambda surface: (surface['footprint']['max_x'] - surface['footprint']['min_x'])
                      * (surface['footprint']['max_y'] - surface['footprint']['min_y']))
        start_xy = ((args.start_x, args.start_y)
                    if args.start_x is not None and args.start_y is not None else None)
        start, goal = choose_start_goal(largest, start_xy, obstacles=value.get('obstacles'),
                                        margin=args.goal_margin,
                                        resolution=value['resolution'],
                                        max_goal_m=args.goal_max)
        out = Path(args.out or (map_path.with_name(
            '%s_v%d_live_task.json' % (args.map_id, args.version))))
        task = {
            'schema_version': 1, 'task_id': '%s_v%d_live' % (args.map_id, args.version),
            'map': {'map_id': args.map_id, 'map_version': args.version}, 'frame': 'map',
            'goal_reference': 'support_surface',
            'start': {'xyz': list(start), 'yaw': 0},
            'goal': {'xyz': list(goal), 'yaw': 0,
                     'tolerance': {'position_m': 0.15, 'yaw_rad': 0.1, 'source': 'operator'},
                     'surface_hint': largest['surface_id'],
                     'surface_disambiguation': 'explicit_hint'},
            'timeout': {'budget_s': 120, 'harness_deadline_s': 180, 'source': 'operator'},
        }
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(task, ensure_ascii=False, indent=1) + '\n', encoding='utf-8')
        print(json.dumps({'ok': True, 'task': str(out), 'start': start, 'goal': goal},
                         ensure_ascii=False, indent=1))
        return
    if not args.pcd:
        raise SystemExit('--pcd is required unless --task-only')
    pcd_path = Path(args.pcd).resolve()
    points = read_pcd(pcd_path)
    points = prepare(points, args.voxel, args.drop_self, args.z_min, args.z_max,
                     args.x_min, args.x_max, args.y_min, args.y_max,
                     nist_profile=args.nist_profile)
    if not len(points):
        raise SystemExit('no points after filtering')
    horizontal, info = g3d.filter_horizontal(points, threshold=args.min_horizontal)
    if not len(horizontal):
        raise SystemExit('no horizontal points after normal filter')
    if args.z_quantize > 0:
        # elevation.extract_surfaces groups cells by exact z_mean equality; real lidar
        # z noise makes every cell unique, so quantize after the normal filter.
        horizontal = horizontal.copy()
        horizontal[:, 2] = np.round(horizontal[:, 2] / args.z_quantize) * args.z_quantize
        info['z_quantize'] = args.z_quantize
    band = None
    if args.obstacle_band:
        parts = [float(item) for item in args.obstacle_band.split(',')]
        if len(parts) != 2 or parts[0] >= parts[1]:
            raise SystemExit('--obstacle-band must be "lo,hi" with lo < hi')
        band = (parts[0], parts[1])
    source = {'uri': ('pcd://%s?points=%d&voxel=%.3f&horizontal=%.2f&obstacles=%s&z_align=%d'
                      % (pcd_path.name, len(points), args.voxel, args.min_horizontal,
                         args.obstacle_mode, int(args.z_align))),
              'sha256': hashlib.sha256(pcd_path.read_bytes()).hexdigest()}
    value, build = g3d.build_map_from_points(
        horizontal, resolution=args.resolution, map_id=args.map_id,
        map_version=args.version, source=source, min_side=args.min_side,
        min_area=args.min_area, min_cells=args.min_cells,
        xy_gap=args.xy_gap, max_z_step=args.max_z_step,
        surface_z_min=args.surface_z_min, surface_z_max=args.surface_z_max,
        obstacle_min_z=args.obstacle_min_z,
        obstacle_mode=args.obstacle_mode, full_points=points,
        obstacle_cell=args.obstacle_cell, obstacle_min_points=args.obstacle_min_points,
        obstacle_band=band, obstacle_band_m=args.obstacle_band_m,
        z_align=args.z_align, absorb_near_floor_m=args.absorb_near_floor)
    out = Path(args.out) if args.out else (
        ROOT / 'data/maps/generated' / ('%s_v%d.json' % (args.map_id, args.version)))
    out = out.resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(value, ensure_ascii=False, indent=1) + '\n', encoding='utf-8')
    sha = hashlib.sha256(out.read_bytes()).hexdigest()
    largest = max(value['surfaces'],
                  key=lambda surface: (surface['footprint']['max_x'] - surface['footprint']['min_x'])
                  * (surface['footprint']['max_y'] - surface['footprint']['min_y']))
    start_xy = (args.start_x, args.start_y) if args.start_x is not None and args.start_y is not None else None
    start, goal = choose_start_goal(largest, start_xy, obstacles=value.get('obstacles'),
                                     margin=args.goal_margin, resolution=args.resolution,
                                     max_goal_m=args.goal_max)
    task_path = out.with_name('%s_v%d_task.json' % (args.map_id, args.version))
    task = {
        'schema_version': 1, 'task_id': '%s_v%d' % (args.map_id, args.version),
        'map': {'map_id': args.map_id, 'map_version': args.version}, 'frame': 'map',
        'goal_reference': 'support_surface',
        'start': {'xyz': list(start), 'yaw': 0},
        'goal': {'xyz': list(goal), 'yaw': 0,
                 'tolerance': {'position_m': 0.15, 'yaw_rad': 0.1, 'source': 'operator'},
                 'surface_hint': largest['surface_id'],
                 'surface_disambiguation': 'explicit_hint'},
        'timeout': {'budget_s': 120, 'harness_deadline_s': 180, 'source': 'operator'},
    }
    task_path.write_text(json.dumps(task, ensure_ascii=False, indent=1) + '\n', encoding='utf-8')
    app_path = out.with_name('%s_v%d_app.yaml' % (args.map_id, args.version))
    write_app(args.map_id, args.version, out, task_path, app_path, args.world, args.model_path)
    if args.register:
        update_registry(args.map_id, args.version, out, sha, len(value['surfaces']),
                        'mapped from %s' % pcd_path.name)
    print(json.dumps({
        'ok': True, 'map': str(out), 'sha256': sha, 'surfaces': len(value['surfaces']),
        'start': start, 'goal': goal, 'app': str(app_path), 'task': str(task_path),
        'points': len(points), 'horizontal': len(horizontal), 'build': build,
        'normal_filter': info,
    }, ensure_ascii=False, indent=1))
    print('NEXT: python3 -B %s --app %s --execute --interactive --rviz'
          % (HERE / 'app_runner.py', app_path))


if __name__ == '__main__':
    main()
