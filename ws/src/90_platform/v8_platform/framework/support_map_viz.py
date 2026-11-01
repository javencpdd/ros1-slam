"""Pure visualization specs for a support_map (ROS node builds MarkerArray)."""
PALETTE = [
    (0.55, 0.55, 0.55, 0.45),
    (0.20, 0.65, 0.90, 0.45),
    (0.95, 0.65, 0.20, 0.45),
    (0.35, 0.80, 0.40, 0.45),
    (0.80, 0.40, 0.85, 0.45),
    (0.95, 0.30, 0.30, 0.45),
]


def surface_specs(map_value):
    """Return deterministic surface marker specs in map coordinates."""
    specs = []
    for index, surface in enumerate(map_value.get('surfaces', [])):
        box = surface['footprint']
        color = PALETTE[index % len(PALETTE)]
        z = float(surface['z_reference'])
        specs.append({
            'kind': 'surface', 'surface_id': surface['surface_id'], 'z_reference': z,
            'x': (box['min_x'] + box['max_x']) / 2.0,
            'y': (box['min_y'] + box['max_y']) / 2.0, 'z': z - .01,
            'sx': max(.01, box['max_x'] - box['min_x']),
            'sy': max(.01, box['max_y'] - box['min_y']), 'sz': .02,
            'color': color,
            'label': '%s z=%.3f' % (surface['surface_id'], z),
        })
    return specs


def obstacle_specs(map_value):
    specs = []
    for index, obstacle in enumerate(map_value.get('obstacles') or []):
        lo, hi = obstacle['min'], obstacle['max']
        specs.append({
            'kind': 'obstacle', 'obstacle_id': obstacle['obstacle_id'],
            'x': (lo[0] + hi[0]) / 2.0, 'y': (lo[1] + hi[1]) / 2.0,
            'z': (lo[2] + hi[2]) / 2.0,
            'sx': max(.01, hi[0] - lo[0]), 'sy': max(.01, hi[1] - lo[1]),
            'sz': max(.01, hi[2] - lo[2]), 'color': (0.9, 0.1, 0.1, 0.55),
            'label': obstacle['obstacle_id'],
        })
    return specs


def trajectory_points(payload):
    """local_native trajectory JSON -> [(x,y,z), ...]."""
    if not isinstance(payload, dict):
        return []
    points = []
    for point in payload.get('trajectory') or []:
        if isinstance(point, (list, tuple)) and len(point) >= 3:
            points.append((float(point[0]), float(point[1]), float(point[2])))
        elif isinstance(point, dict) and {'x', 'y', 'z'} <= set(point):
            points.append((float(point['x']), float(point['y']), float(point['z'])))
    return points
