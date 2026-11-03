#!/usr/bin/env python3
"""Gazebo lidar + odometry -> support/elevation map for the scaled NIST stairs.

The survey teleports the kinematic lite3_planar model to known poses, captures
/rslidar_points at each pose, transforms the sensor-local cloud with /sim/odom_gt
and the URDF base->rslidar mount, and runs the elevation -> support_map pipeline.
The declared NIST map is used only as an evaluation truth, never as planner input.

Scope: Gazebo ray sensor + kinematic pose sink, no contact dynamics/no real robot.
"""
import argparse
import hashlib
import json
import math
import random
import sys
import time
from collections import defaultdict
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / 'representation'))
sys.path.insert(0, str(ROOT / 'contracts'))
import elevation  # noqa: E402
import support_map as rep  # noqa: E402

LEVELS = [0.0, 0.198798, 0.371688, 0.544578, 0.717468, 0.890358]
LIDAR_Z = 0.34
LIDAR_PITCH_DEG = 15.0
BASE_CENTER_Z = 0.11
CROP = {'min_x': -1.1, 'max_x': 1.1, 'min_y': -3.2, 'max_y': 1.7,
        'min_z': -0.15, 'max_z': 1.05}
STAIR_X = 1.1
GROUND_FRONT = -1.0

# Survey poses: (x, y, base_z). The robot faces +y so the forward-biased
# Mid360 pattern looks up the stairs; x offsets widen tread coverage.
# The Gazebo ray collision of the NIST mesh resolves to observed tread heights
# 0.117/0.219/0.321/0.423/0.508 m with 0.42 m treads (the visual SDF scale is
# 0.51/1.70; see evidence for the collision-vs-visual scaling note).
STEP_LEVELS = [0.117, 0.219, 0.321, 0.423, 0.508]
SURVEY_FULL = (
    [(0.0, y, 0.0) for y in (-3.0, -2.6, -2.2, -1.8, -1.4)]
    + [(-0.7, y, 0.0) for y in (-2.7, -2.0, -1.5)]
    + [(0.7, y, 0.0) for y in (-2.7, -2.0, -1.5)]
    + [(x, y, level) for y, level in zip((-0.79, -0.40, 0.0, 0.40, 0.79), STEP_LEVELS)
       for x in (-0.6, 0.0, 0.6)]
)
SURVEY_GROUND = SURVEY_FULL[:11]


def quat_matrix(q):
    x, y, z, w = (float(v) for v in q)
    n = x * x + y * y + z * z + w * w
    if n <= 0:
        raise ValueError('zero quaternion')
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def yaw_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def mount_rotation(pitch_deg=LIDAR_PITCH_DEG):
    th = math.radians(pitch_deg)
    return np.array([[math.cos(th), 0.0, math.sin(th)],
                     [0.0, 1.0, 0.0],
                     [-math.sin(th), 0.0, math.cos(th)]])


def transform_points(points, base_xyz, base_quat, lidar_z=LIDAR_Z, pitch_deg=LIDAR_PITCH_DEG):
    """sensor-local points -> world using odom base pose and the fixed mount."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if not len(points):
        return points
    sensor = points @ mount_rotation(pitch_deg).T + np.array([0.0, 0.0, lidar_z])
    return sensor @ quat_matrix(base_quat).T + np.asarray(base_xyz, dtype=float)


def crop_points(points, crop=CROP):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    mask = ((points[:, 0] >= crop['min_x']) & (points[:, 0] <= crop['max_x'])
            & (points[:, 1] >= crop['min_y']) & (points[:, 1] <= crop['max_y'])
            & (points[:, 2] >= crop['min_z']) & (points[:, 2] <= crop['max_z']))
    return points[mask]


def voxel_downsample(points, size):
    if not len(points):
        return np.zeros((0, 3))
    keys = np.floor(np.asarray(points) / size).astype(np.int64)
    _, index = np.unique(keys, axis=0, return_index=True)
    return np.asarray(points)[np.sort(index)]


def filter_horizontal(points, radius=0.16, k=10, threshold=0.8, voxel=0.03):
    """Keep near-horizontal points; riser/wall normals are rejected before mapping."""
    from scipy.spatial import cKDTree
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if not len(points):
        return points, {'input': 0, 'kept': 0}
    keys = np.floor(points / voxel).astype(np.int64)
    _, index = np.unique(keys, axis=0, return_index=True)
    sample = points[np.sort(index)]
    tree = cKDTree(sample)
    count = min(k, len(sample))
    distances, indices = tree.query(sample, k=count)
    normals = np.zeros_like(sample)
    for row in range(len(sample)):
        neighbours = indices[row]
        spread = distances[row].max() if count > 1 else 0.0
        if count < 3 or spread > radius:
            normals[row, 2] = 0.0
            continue
        cloud = sample[neighbours] - sample[neighbours].mean(axis=0)
        covariance = cloud.T @ cloud / len(neighbours)
        values, vectors = np.linalg.eigh(covariance)
        normals[row] = vectors[:, 0]
    horizontal = np.abs(normals[:, 2]) >= threshold
    keep = sample[horizontal]
    return keep, {'input': int(len(points)), 'sampled': int(len(sample)),
                  'horizontal': int(horizontal.sum()),
                  'normal_radius_m': radius, 'normal_k': count,
                  'normal_threshold': threshold}


def _footprint_gap(fa, fb):
    dx = max(fa['min_x'] - fb['max_x'], fb['min_x'] - fa['max_x'], 0.0)
    dy = max(fa['min_y'] - fb['max_y'], fb['min_y'] - fa['max_y'], 0.0)
    return math.hypot(dx, dy)


def consolidate_surfaces(surfaces, z_merge=0.05, xy_gap=0.12):
    """Merge near-coplanar fragments; never merges distinct layers > z_merge."""
    clusters = []
    for surface in sorted(surfaces, key=lambda item: item['z_reference']):
        fp = surface['footprint']
        area = max(1e-9, (fp['max_x'] - fp['min_x']) * (fp['max_y'] - fp['min_y']))
        clusters.append({'z_reference': surface['z_reference'],
                         'footprint': dict(fp), 'weight': area,
                         'cells': [list(cell) for cell in surface.get('cells', [])],
                         'sources': [surface['surface_id']]})
    changed = True
    while changed:
        changed = False
        for left in range(len(clusters)):
            if changed:
                break
            for right in range(left + 1, len(clusters)):
                a, b = clusters[left], clusters[right]
                if (abs(a['z_reference'] - b['z_reference']) > z_merge
                        or _footprint_gap(a['footprint'], b['footprint']) > xy_gap):
                    continue
                weight = a['weight'] + b['weight']
                a['z_reference'] = ((a['z_reference'] * a['weight']
                                     + b['z_reference'] * b['weight']) / weight)
                for key in ('min_x', 'min_y'):
                    a['footprint'][key] = min(a['footprint'][key], b['footprint'][key])
                for key in ('max_x', 'max_y'):
                    a['footprint'][key] = max(a['footprint'][key], b['footprint'][key])
                a['weight'] = weight
                a['cells'].extend(b['cells'])
                a['sources'].extend(b['sources'])
                clusters.pop(right)
                changed = True
                break
    for index, cluster in enumerate(clusters):
        cluster.pop('weight')
        cluster['surface_id'] = 'm%d' % index
        cluster['z_reference'] = round(cluster['z_reference'], 6)
        cluster['data_uri'] = 'elevation://%s' % cluster['surface_id']
        cluster['cells'] = sorted({tuple(cell) for cell in cluster['cells']})
    return clusters


def classify_surfaces(surfaces, resolution, min_side=0.15, min_area=0.10,
                      z_min=-0.06, z_max=1.05):
    supported, rejected = [], []
    for surface in surfaces:
        fp = surface['footprint']
        width = fp['max_x'] - fp['min_x']
        depth = fp['max_y'] - fp['min_y']
        area = width * depth
        reason = None
        if surface['z_reference'] < z_min or surface['z_reference'] > z_max:
            reason = 'z_out_of_band'
        elif min(width, depth) < min_side:
            reason = 'thin_footprint'
        elif area < min_area:
            reason = 'small_area'
        if reason:
            rejected.append({'surface_id': surface['surface_id'],
                             'z_reference': surface['z_reference'],
                             'footprint': surface['footprint'],
                             'cells': [list(cell) for cell in surface.get('cells', [])],
                             'reason': reason})
            continue
        item = dict(surface)
        item['traversability'] = 'supported'
        item['thickness_m'] = 0.05
        item['representation'] = 'elevation_grid'
        item['clearance_min_m'] = 0.4
        item['edges'] = []
        supported.append(item)
    return supported, rejected


def promote_edges(surfaces, max_xy_gap=0.10, max_z_step=0.25):
    """Mapper-side confirmation of geometric adjacency into explicit step edges."""
    edges, rejected = [], []
    for left, right in _pairs(surfaces):
        gap = _footprint_gap(left['footprint'], right['footprint'])
        drop = abs(right['z_reference'] - left['z_reference'])
        if drop <= 1e-6:
            continue
        if gap <= max_xy_gap and drop <= max_z_step:
            edges.append({'from': left['surface_id'], 'to': right['surface_id'],
                          'kind': 'step', 'max_step_height_m': round(drop + 1e-4, 6),
                          'bidirectional': True, 'xy_gap': round(gap, 4)})
        else:
            rejected.append({'from': left['surface_id'], 'to': right['surface_id'],
                             'xy_gap': round(gap, 4), 'z_drop': round(drop, 4),
                             'reason': 'gap' if gap > max_xy_gap else 'rise'})
    by_id = {surface['surface_id']: surface for surface in surfaces}
    for edge in edges:
        payload = {'to': edge['to'], 'kind': 'step',
                   'max_step_height_m': edge['max_step_height_m'],
                   'bidirectional': True}
        back = {'to': edge['from'], 'kind': 'step',
                'max_step_height_m': edge['max_step_height_m'],
                'bidirectional': True}
        by_id[edge['from']]['edges'].append(payload)
        by_id[edge['to']]['edges'].append(back)
    return edges, rejected


def _pairs(items):
    for index, left in enumerate(items):
        for right in items[index + 1:]:
            yield left, right


def connected_component(surfaces, start_id=None):
    by_id = {surface['surface_id']: surface for surface in surfaces}
    start = start_id or min(surfaces, key=lambda item: item['z_reference'])['surface_id']
    seen, stack = {start}, [start]
    while stack:
        current = stack.pop()
        for edge in by_id[current]['edges']:
            if edge['to'] not in seen:
                seen.add(edge['to'])
                stack.append(edge['to'])
    return seen


def _rectangles(cells):
    """Row runs merged vertically into maximal rectangles (i0, i1, j0, j1)."""
    rows = {}
    for i, j in cells:
        rows.setdefault(int(j), []).append(int(i))
    runs = []
    for j, xs in rows.items():
        xs = sorted(set(xs))
        start = previous = xs[0]
        for x in xs[1:]:
            if x == previous + 1:
                previous = x
                continue
            runs.append((start, previous, j))
            start = previous = x
        runs.append((start, previous, j))
    runs.sort(key=lambda run: (run[0], run[1], run[2]))
    used = [False] * len(runs)
    rectangles = []
    for index, run in enumerate(runs):
        if used[index]:
            continue
        i0, i1, j0 = run
        j_hi = j0
        used[index] = True
        changed = True
        while changed:
            changed = False
            for other, candidate in enumerate(runs):
                if (used[other] or candidate[0] != i0 or candidate[1] != i1
                        or candidate[2] != j_hi + 1):
                    continue
                used[other] = True
                j_hi = candidate[2]
                changed = True
                break
        rectangles.append((i0, i1, j0, j_hi))
    return rectangles


def refined_surface_obstacles(rejected, resolution, z_floor, obstacle_min_z):
    """Split rejected surfaces at their real cells instead of one cluster AABB.

    Full-height columns are kept on purpose: the horizontal-only cloud does not
    see table legs, so letting the body pass under a tabletop would be unsound.
    """
    out = []
    for item in rejected:
        if item['z_reference'] < obstacle_min_z:
            continue
        cells = {tuple(cell) for cell in item.get('cells') or []}
        if cells:
            for i0, i1, j0, j1 in _rectangles(cells):
                out.append({'obstacle_id': 'o%d' % len(out),
                            'min': [i0 * resolution, j0 * resolution, float(z_floor)],
                            'max': [(i1 + 1) * resolution, (j1 + 1) * resolution,
                                    float(item['z_reference']) + 0.05]})
        else:
            footprint = item['footprint']
            out.append({'obstacle_id': 'o%d' % len(out),
                        'min': [footprint['min_x'], footprint['min_y'], float(z_floor)],
                        'max': [footprint['max_x'], footprint['max_y'],
                                float(item['z_reference']) + 0.05]})
    return out


def full_cloud_obstacles(points, z_band, cell_m=0.20, min_points=5, band_m=0.40,
                         min_voxel_points=3, exclude=None, exclude_resolution=None,
                         exclude_tol=0.10):
    """Walls and furniture from the full cloud, with their real vertical extent.

    Per-cell (x, y) vertical ranges are grouped into ``band_m`` layers and
    merged into rectangles; each box keeps the actual z span of its cells, so
    the local planner can tell a wall (floor to ceiling) from a tabletop.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    points = points[(points[:, 2] >= z_band[0]) & (points[:, 2] <= z_band[1])]
    if not len(points):
        return []
    excluded = {}
    for surface in exclude or []:
        for i, j in surface.get('cells') or []:
            excluded.setdefault((int(i), int(j)), []).append(surface['z_reference'])
    profiles = {}
    for x, y, z in points:
        if excluded and exclude_resolution:
            key0 = (int(math.floor(x / exclude_resolution)),
                    int(math.floor(y / exclude_resolution)))
            levels = excluded.get(key0)
            if levels and any(abs(z - level) <= exclude_tol for level in levels):
                continue
        key = (int(math.floor(x / cell_m)), int(math.floor(y / cell_m)))
        # Sparse returns scattered up a wall must not extrude a box down to the
        # floor: keep only 0.2 m z bins with at least min_voxel_points hits.
        bins = profiles.get(key)
        if bins is None:
            profiles[key] = {int(math.floor(z / 0.20)): [z, z, 1]}
            continue
        bucket = bins.get(int(math.floor(z / 0.20)))
        if bucket is None:
            bins[int(math.floor(z / 0.20))] = [z, z, 1]
        else:
            bucket[0] = min(bucket[0], z)
            bucket[1] = max(bucket[1], z)
            bucket[2] += 1
    layers = {}
    for (i, j), bins in profiles.items():
        dense = [value for value in bins.values() if value[2] >= min_voxel_points]
        if not dense or sum(value[2] for value in dense) < min_points:
            continue
        z0 = min(value[0] for value in dense)
        z1 = max(value[1] for value in dense)
        band = int(math.floor(((z0 + z1) / 2.0) / band_m))
        layers.setdefault(band, {})[(i, j)] = (z0, z1)
    out = []
    for band in sorted(layers):
        layer = layers[band]
        for i0, i1, j0, j1 in _rectangles(layer.keys()):
            z0 = min(layer[(i, j)][0] for i in range(i0, i1 + 1) for j in range(j0, j1 + 1)
                     if (i, j) in layer)
            z1 = max(layer[(i, j)][1] for i in range(i0, i1 + 1) for j in range(j0, j1 + 1)
                     if (i, j) in layer)
            out.append({'obstacle_id': 'o%d' % len(out),
                        'min': [i0 * cell_m, j0 * cell_m, float(z0)],
                        'max': [(i1 + 1) * cell_m, (j1 + 1) * cell_m,
                                float(max(z1, z0 + 0.02))]})
    return out


def build_map_from_points(points, resolution=0.05, map_id='nist_sensor',
                          map_version=1, source=None, obstacles=None,
                          z_tol=0.05, min_cells=1, z_merge=0.05,
                          xy_gap=0.10, max_z_step=0.25, min_side=0.15,
                          min_area=0.10, surface_z_min=-0.06, surface_z_max=1.05,
                          obstacle_min_z=None, z_align=False,
                          absorb_near_floor_m=0.0, obstacle_mode='surfaces',
                          full_points=None, obstacle_cell=0.20,
                          obstacle_min_points=5, obstacle_band=None,
                          obstacle_band_m=0.40):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    result = elevation.mapping_pipeline(points, resolution, z_tol=z_tol,
                                        min_cells=min_cells, advisory_gap=xy_gap,
                                        advisory_step=max_z_step)
    merged = consolidate_surfaces(result['surfaces'], z_merge=z_merge, xy_gap=xy_gap)
    supported, rejected = classify_surfaces(merged, resolution, min_side=min_side,
                                            min_area=min_area,
                                            z_min=surface_z_min, z_max=surface_z_max)
    absorbed = []
    if absorb_near_floor_m > 0 and len(supported) > 1:
        floor = min(supported, key=lambda item: item['z_reference'])
        for surface in list(supported):
            if surface is floor:
                continue
            if surface['z_reference'] - floor['z_reference'] > absorb_near_floor_m + 1e-9:
                continue
            floor['cells'] = sorted({tuple(cell) for cell in floor.get('cells', [])}
                                    | {tuple(cell) for cell in surface.get('cells', [])})
            for key in ('min_x', 'min_y'):
                floor['footprint'][key] = min(floor['footprint'][key],
                                              surface['footprint'][key])
            for key in ('max_x', 'max_y'):
                floor['footprint'][key] = max(floor['footprint'][key],
                                              surface['footprint'][key])
            absorbed.append(surface['surface_id'])
        supported = [surface for surface in supported
                     if surface['surface_id'] not in absorbed]
    if not supported:
        raise rep.RepresentationError('E_REP_NO_SURFACES', 'all extracted surfaces rejected')
    edges, edge_rejected = promote_edges(supported, max_xy_gap=xy_gap,
                                         max_z_step=max_z_step)
    component = connected_component(supported)
    keep = [surface for surface in supported if surface['surface_id'] in component]
    dropped = [surface['surface_id'] for surface in supported
               if surface['surface_id'] not in component]
    for surface in keep:
        surface['edges'] = [edge for edge in surface['edges'] if edge['to'] in component]
    if not keep:
        raise rep.RepresentationError('E_REP_NO_SURFACES', 'no connected support component')
    obstacle_boxes = list(obstacles or [])
    if obstacle_mode in ('surfaces', 'both') and obstacle_min_z is not None:
        obstacle_boxes.extend(refined_surface_obstacles(
            rejected, resolution, surface_z_min, obstacle_min_z))
    if obstacle_mode in ('full', 'both'):
        if full_points is None:
            raise rep.RepresentationError('E_REP_OBSTACLES',
                                          'obstacle_mode %s needs full_points' % obstacle_mode)
        band = obstacle_band or (float(surface_z_min) + 0.10, float(surface_z_max) + 1.45)
        obstacle_boxes.extend(full_cloud_obstacles(
            full_points, band, cell_m=obstacle_cell, min_points=obstacle_min_points,
            band_m=obstacle_band_m, exclude=keep,
            exclude_resolution=resolution))
    for index, box in enumerate(obstacle_boxes):
        box['obstacle_id'] = 'o%d' % index
    z_shift = 0.0
    if z_align:
        z_shift = -min(surface['z_reference'] for surface in keep)
        for surface in keep:
            surface['z_reference'] = round(surface['z_reference'] + z_shift, 6)
        for box in obstacle_boxes:
            box['min'][2] = float(box['min'][2]) + z_shift
            box['max'][2] = float(box['max'][2]) + z_shift
    min_x = min(surface['footprint']['min_x'] for surface in keep)
    min_y = min(surface['footprint']['min_y'] for surface in keep)
    origin = [min_x, min_y, 0.0]
    payload_source = dict(source or {})
    payload_source.setdefault('uri', 'gazebo://lite3_planar/rslidar_points+sim_odom_gt')
    value = rep.build_support_map(map_id, resolution, origin, keep, edges, payload_source,
                                  map_version=map_version, obstacles=obstacle_boxes or None)
    info = {
        'points': int(len(points)),
        'columns': len(result['columns']),
        'raw_surfaces': len(result['surfaces']),
        'consolidated_surfaces': len(merged),
        'supported_surfaces': len(keep),
        'absorbed_surfaces': absorbed,
        'z_shift': round(z_shift, 6),
        'dropped_clusters': result['dropped_clusters'],
        'rejected_surfaces': rejected,
        'disconnected_surfaces': dropped,
        'promoted_edges': edges,
        'rejected_edges': edge_rejected,
        'obstacle_mode': obstacle_mode,
        'obstacles': len(obstacle_boxes),
        'map_version': map_version,
        'levels': sorted({surface['z_reference'] for surface in keep}),
    }
    return value, info


def score_against_truth(value, truth, xy_tolerance=0.25, z_tolerance=0.04):
    """Match generated surfaces to the declared NIST truth by z then XY overlap."""
    rows, matched = [], set()
    for surface in value['surfaces']:
        candidates = []
        for index, target in enumerate(truth['surfaces']):
            if index in matched:
                continue
            z_error = abs(surface['z_reference'] - target['z_reference'])
            if z_error > z_tolerance:
                continue
            overlap = _overlap(surface['footprint'], target['footprint'])
            candidates.append((z_error, -overlap, index))
        if not candidates:
            rows.append({'surface_id': surface['surface_id'],
                         'z_reference': surface['z_reference'],
                         'matched': None, 'z_error': None, 'overlap': 0.0})
            continue
        z_error, neg_overlap, index = min(candidates)
        matched.add(index)
        rows.append({'surface_id': surface['surface_id'],
                     'z_reference': surface['z_reference'],
                     'matched': truth['surfaces'][index]['surface_id'],
                     'truth_z': truth['surfaces'][index]['z_reference'],
                     'z_error': round(z_error, 6),
                     'overlap': round(-neg_overlap, 4)})
    truth_hit = {row['matched'] for row in rows if row['matched']}
    missing = [surface['surface_id'] for index, surface in enumerate(truth['surfaces'])
               if index not in matched]
    generated_levels = sorted(surface['z_reference'] for surface in value['surfaces'])
    truth_levels = sorted(surface['z_reference'] for surface in truth['surfaces'])
    level_errors = []
    for level in truth_levels:
        if not generated_levels:
            break
        nearest = min(generated_levels, key=lambda z: abs(z - level))
        if abs(nearest - level) <= z_tolerance:
            level_errors.append(abs(nearest - level))
    expected_edges = set()
    for surface in truth['surfaces']:
        for edge in surface.get('edges', []):
            expected_edges.add(tuple(sorted((surface['surface_id'], edge['to']))))
    generated_edges = set()
    for surface in value['surfaces']:
        for edge in surface.get('edges', []):
            generated_edges.add(tuple(sorted((surface['surface_id'], edge['to']))))
    return {
        'rows': rows,
        'matched_truth': len(truth_hit),
        'truth_surfaces': len(truth['surfaces']),
        'missing_truth': missing,
        'level_mae_m': round(sum(level_errors) / len(level_errors), 6) if level_errors else None,
        'level_max_error_m': round(max(level_errors), 6) if level_errors else None,
        'levels_covered': len(level_errors),
        'generated_edges': len(generated_edges),
        'truth_edges': len(expected_edges),
    }


def _overlap(fa, fb):
    dx = max(0.0, min(fa['max_x'], fb['max_x']) - max(fa['min_x'], fb['min_x']))
    dy = max(0.0, min(fa['max_y'], fb['max_y']) - max(fa['min_y'], fb['min_y']))
    return dx * dy


def _read_pose(odom):
    q = odom.pose.pose.orientation
    p = odom.pose.pose.position
    return (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)


def _point_array(message):
    import sensor_msgs.point_cloud2 as pc2
    return np.array([[p[0], p[1], p[2]] for p in
                     pc2.read_points(message, field_names=('x', 'y', 'z'),
                                     skip_nans=True)], dtype=float)


def survey(args):
    import rospy
    from gazebo_msgs.msg import ModelState
    from gazebo_msgs.srv import SetModelState
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2

    rospy.init_node('gazebo3d_mapping', anonymous=True)
    rospy.wait_for_service('/gazebo/set_model_state', timeout=20.0)
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    pose_box, cloud_box = {'odom': None}, {'cloud': []}

    def on_odom(message):
        pose_box['odom'] = message

    def on_cloud(message):
        cloud_box['cloud'].append(message)

    rospy.Subscriber('/sim/odom_gt', Odometry, on_odom, queue_size=20)
    rospy.Subscriber('/rslidar_points', PointCloud2, on_cloud, queue_size=20)
    seed = int(args.seed)
    rng = random.Random(seed)
    poses = SURVEY_FULL if args.survey == 'full' else SURVEY_GROUND
    world_points, records = [], []
    try:
        for index, (x, y, level) in enumerate(poses):
            state = ModelState()
            state.model_name, state.reference_frame = args.model, 'world'
            state.pose.position.x, state.pose.position.y = x, y
            state.pose.position.z = level + BASE_CENTER_Z
            q = yaw_quaternion(math.pi / 2.0)
            state.pose.orientation.x, state.pose.orientation.y = q[0], q[1]
            state.pose.orientation.z, state.pose.orientation.w = q[2], q[3]
            def hold(seconds):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    result = set_state(state)
                    if not result.success:
                        raise RuntimeError('set_model_state failed: ' + result.status_message)
                    rospy.sleep(1.0 / args.hold_hz)
            hold(args.settle_s)
            cloud_box['cloud'] = []
            deadline = time.monotonic() + args.pose_timeout_s
            while time.monotonic() < deadline and len(cloud_box['cloud']) < args.scans:
                hold(1.0 / args.hold_hz)
            if len(cloud_box['cloud']) < args.scans:
                raise RuntimeError('scan timeout at pose %d' % index)
            base_xyz, base_quat = _read_pose(pose_box['odom'])
            base_xyz = (base_xyz[0] + rng.gauss(0.0, args.odom_noise_m),
                        base_xyz[1] + rng.gauss(0.0, args.odom_noise_m),
                        base_xyz[2] + rng.gauss(0.0, args.odom_noise_m))
            yaw = math.atan2(2.0 * (base_quat[3] * base_quat[2]),
                             1.0 - 2.0 * base_quat[2] * base_quat[2])
            yaw += math.radians(rng.gauss(0.0, args.odom_noise_deg))
            base_quat = yaw_quaternion(yaw)
            captured = len(cloud_box['cloud'])
            for message in cloud_box['cloud'][-args.scans:]:
                local = _point_array(message)
                ranges = np.linalg.norm(local, axis=1)
                local = local[(ranges >= args.min_range_m) & (ranges <= args.max_range_m)]
                base = local @ mount_rotation().T + np.array([0.0, 0.0, LIDAR_Z])
                if args.drop_self:
                    inside = ((np.abs(base[:, 0]) <= 0.30) & (np.abs(base[:, 1]) <= 0.25)
                              & (base[:, 2] >= -0.18) & (base[:, 2] <= 0.18))
                    base = base[~inside]
                world = base @ quat_matrix(base_quat).T + np.asarray(base_xyz)
                world_points.append(crop_points(world))
            records.append({'index': index, 'commanded': [x, y, level],
                            'odom': list(base_xyz), 'yaw_deg': math.degrees(yaw),
                            'scans': captured, 'points': int(sum(len(p) for p in world_points[-captured:]))})
    finally:
        pass
    points = np.concatenate(world_points, axis=0) if world_points else np.zeros((0, 3))
    points = voxel_downsample(points, args.voxel_m)
    ground = points[(points[:, 2] < 0.08) & (points[:, 1] <= GROUND_FRONT)]
    stairs = points[(points[:, 2] >= 0.08) & (np.abs(points[:, 0]) <= STAIR_X + 0.05)
                    & (points[:, 1] <= STAIR_X + 0.05)]
    points = np.concatenate([ground, stairs], axis=0)
    points, normal_info = filter_horizontal(points, threshold=args.min_horizontal)
    records = {'poses': records, 'normal_filter': normal_info,
               'ground_points': int(len(ground)), 'stair_points': int(len(stairs))}
    return points, records


def _sha256_points(points):
    digest = hashlib.sha256()
    digest.update(np.asarray(points, dtype='<f8').tobytes())
    return digest.hexdigest()


def _write_json(path, payload):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(payload, ensure_ascii=False, indent=1) + '\n',
                          encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--survey', choices=['full', 'ground'], default='full')
    parser.add_argument('--model', default='lite3_planar')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--scans', type=int, default=4)
    parser.add_argument('--pose-timeout-s', type=float, default=5.0)
    parser.add_argument('--settle-s', type=float, default=0.6)
    parser.add_argument('--hold-hz', type=float, default=40.0)
    parser.add_argument('--min-horizontal', type=float, default=0.8)
    parser.add_argument('--odom-noise-m', type=float, default=0.0)
    parser.add_argument('--odom-noise-deg', type=float, default=0.0)
    parser.add_argument('--min-range-m', type=float, default=0.3)
    parser.add_argument('--drop-self', action='store_true', default=True)
    parser.add_argument('--max-range-m', type=float, default=12.0)
    parser.add_argument('--voxel-m', type=float, default=0.02)
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--z-max', type=float, default=0.0,
                        help='drop points above this z after scaling (0 disables)')
    parser.add_argument('--point-scale', type=float, default=1.0,
                        help='Gazebo ray-collision to scene scale calibration')
    parser.add_argument('--map-version', type=int, default=1)
    parser.add_argument('--map-id', default='nist_sensor')
    parser.add_argument('--obstacle-json', default='',
                        help='JSON list of map/v2 obstacles injected from a simulated dynamic box')
    parser.add_argument('--truth', default='data/maps/nist_stairs_120_s020x170_support.json')
    parser.add_argument('--points-npz', default='')
    parser.add_argument('--survey-poses', type=int, default=0)
    parser.add_argument('--out-points', default='')
    parser.add_argument('--out-map', required=True)
    parser.add_argument('--out-json', required=True)
    args = parser.parse_args()
    points_path = args.points_npz
    if points_path:
        points = np.load(points_path)['points']
        records = [{'source': points_path, 'points': int(len(points))}]
    else:
        points, records = survey(args)
    obstacles = json.loads(Path(args.obstacle_json).read_text()) if args.obstacle_json else None
    survey_meta = {'mode': args.survey,
                   'poses': args.survey_poses or (len(records['poses']) if isinstance(records, dict) else len(records)),
                   'scans_per_pose': args.scans,
                   'seed': args.seed, 'odom_noise_m': args.odom_noise_m,
                   'odom_noise_deg': args.odom_noise_deg,
                   'min_range_m': args.min_range_m, 'max_range_m': args.max_range_m,
                   'voxel_m': args.voxel_m, 'point_scale': args.point_scale,
                   'z_max': args.z_max, 'survey_poses': args.survey_poses}
    source = {
        'uri': ('gazebo://lite3_planar/rslidar_points+sim_odom_gt?'
                + '&'.join('%s=%s' % (key, survey_meta[key])
                           for key in ('mode', 'poses', 'scans_per_pose', 'seed',
                                       'odom_noise_m', 'odom_noise_deg',
                                       'min_range_m', 'max_range_m', 'voxel_m', 'point_scale', 'z_max',
                                       'survey_poses'))),
        'sha256': _sha256_points(points),
    }
    if args.point_scale != 1.0:
        points = points * args.point_scale
    if args.z_max > 0:
        points = points[points[:, 2] <= args.z_max]
    if args.out_points:
        np.savez_compressed(args.out_points, points=points)
    value, info = build_map_from_points(points, resolution=args.resolution,
                                        map_id=args.map_id,
                                        map_version=args.map_version,
                                        source=source, obstacles=obstacles)
    truth = json.loads(Path(args.truth).read_text(encoding='utf-8'))
    score = score_against_truth(value, truth)
    payload = {'ok': True, 'points_sha256': source['sha256'], 'survey': records,
               'map_info': info, 'score': score, 'map': value, 'args': vars(args)}
    _write_json(args.out_map, value)
    _write_json(args.out_json, payload)
    print(json.dumps({'ok': True, 'map': args.out_map, 'levels': info['levels'],
                      'surfaces': info['supported_surfaces'], 'score': score},
                     ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
