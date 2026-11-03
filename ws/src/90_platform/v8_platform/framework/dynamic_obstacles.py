"""Dynamic obstacle extraction from a registered point cloud (pure Python).

Pipeline: ROI crop -> above-ground filter -> voxel downsample -> grid connectivity
clustering -> box + velocity estimate -> payload for /v8/local/obstacles.

No ROS import: the node wrapper feeds numpy arrays in and gets JSON-able boxes out.
"""
import math

import numpy as np


def roi_filter(points, center_xy, floor_z, forward_m, lateral_m, min_h, max_h):
    """Keep points inside a forward window around the robot and above the floor."""
    if not len(points):
        return points
    dx = points[:, 0] - center_xy[0]
    dy = points[:, 1] - center_xy[1]
    dz = points[:, 2] - floor_z
    mask = ((dx >= -0.5) & (dx <= forward_m)
            & (np.abs(dy) <= lateral_m)
            & (dz >= min_h) & (dz <= max_h))
    return points[mask]


def self_filter(points, center_xy, radius_m):
    if not len(points):
        return points
    d2 = (points[:, 0] - center_xy[0]) ** 2 + (points[:, 1] - center_xy[1]) ** 2
    return points[d2 >= radius_m * radius_m]


def voxel_downsample(points, size_m):
    """Return (voxel_centers, keys) with one representative per occupied voxel."""
    if not len(points):
        return points, np.empty((0, 3), dtype=int)
    # 1e-6 m epsilon: 2.4/0.1 evaluates to 23.999999999999996 and would floor to 23,
    # leaving a one-cell gap against 25 and splitting one object into columns.
    keys = np.floor(points / size_m + 1e-6).astype(np.int64)
    uniq, inverse = np.unique(keys, axis=0, return_inverse=True)
    counts = np.bincount(inverse, minlength=len(uniq)).astype(float)
    sums = np.zeros((len(uniq), 3), dtype=float)
    for axis in range(3):
        sums[:, axis] = np.bincount(inverse, weights=points[:, axis],
                                    minlength=len(uniq))
    return sums / counts[:, None], uniq


def cluster_boxes(points, voxel_keys, min_points=8, max_objects=32):
    """XY-connected components over occupied voxel cells -> axis-aligned boxes.

    Clustering in XY (not 3D) keeps a sparse vertical scan of one person in one
    cluster; the box still carries the full z range of its points.
    """
    if not len(voxel_keys):
        return []
    cells = {}
    for index, key in enumerate(voxel_keys.tolist()):
        cell = (key[0], key[1])
        cells.setdefault(cell, []).append(index)
    seen = set()
    boxes = []
    for cell in cells:
        if cell in seen:
            continue
        stack = [cell]
        seen.add(cell)
        members = []
        while stack:
            current = stack.pop()
            members.extend(cells[current])
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    neighbour = (current[0] + dx, current[1] + dy)
                    if neighbour in cells and neighbour not in seen:
                        seen.add(neighbour)
                        stack.append(neighbour)
        if len(members) < min_points:
            continue
        cloud = points[members]
        boxes.append({
            'min': [float(cloud[:, 0].min()), float(cloud[:, 1].min()), float(cloud[:, 2].min())],
            'max': [float(cloud[:, 0].max()), float(cloud[:, 1].max()), float(cloud[:, 2].max())],
            'points': int(len(members)),
        })
        if len(boxes) >= max_objects:
            break
    return boxes


def _center(box):
    return ((box['min'][0] + box['max'][0]) / 2.0,
            (box['min'][1] + box['max'][1]) / 2.0,
            (box['min'][2] + box['max'][2]) / 2.0)


def associate(previous, current, dt, gate_m=0.8, max_speed_mps=1.5):
    """Nearest-centroid velocity estimate; unmatched boxes get zero velocity."""
    velocities = []
    for box in current:
        center = _center(box)
        best, best_d = None, None
        for old in previous or []:
            old_center = _center(old)
            d = math.hypot(center[0] - old_center[0], center[1] - old_center[1])
            if best_d is None or d < best_d:
                best, best_d = old, d
        if best is None or best_d is None or best_d > gate_m or dt <= 0:
            velocities.append((0.0, 0.0, 0.0))
            continue
        old_center = _center(best)
        vx = (center[0] - old_center[0]) / dt
        vy = (center[1] - old_center[1]) / dt
        speed = math.hypot(vx, vy)
        if speed > max_speed_mps and speed > 0:
            vx *= max_speed_mps / speed
            vy *= max_speed_mps / speed
        velocities.append((vx, vy, 0.0))
    return velocities


def to_payload(boxes, velocities, stamp, ttl_s=1.0):
    """Local planner obstacle message: min/max + velocity + time window."""
    payload = []
    for box, velocity in zip(boxes, velocities):
        payload.append({
            'min': [round(value, 3) for value in box['min']],
            'max': [round(value, 3) for value in box['max']],
            'velocity': [round(value, 3) for value in velocity],
            'points': box.get('points'),
            'stamp': round(stamp, 3),
            'active_from': round(stamp - ttl_s, 3),
            'active_until': round(stamp + ttl_s, 3),
        })
    return payload
