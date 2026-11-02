#!/usr/bin/env python3
"""Live LiDAR clearance probe for a GT teleport path.

Teleports the model to sampled path points and measures the nearest horizontal
LiDAR return in a body-height band. This is a sensor-visibility check, not a
physics collision proof.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2


def load_path(path):
    value = json.loads(Path(path).read_text(encoding='utf-8'))
    return value['waypoints']


def sample_path(waypoints, sample_m):
    samples = []
    for first, second in zip(waypoints, waypoints[1:]):
        dx, dy, dz = second['x'] - first['x'], second['y'] - first['y'], second['z'] - first['z']
        count = max(1, int(math.ceil(math.hypot(dx, dy) / sample_m)))
        for index in range(count):
            ratio = index / float(count)
            samples.append((first['x'] + ratio * dx, first['y'] + ratio * dy,
                            first['z'] + ratio * dz))
    last = waypoints[-1]
    samples.append((last['x'], last['y'], last['z']))
    return samples


def scan_clearance(message, z_min, z_max, ignore_m=0.08):
    points = np.array(list(point_cloud2.read_points(
        message, field_names=('x', 'y', 'z'), skip_nans=True)), dtype=float)
    if not len(points):
        return None
    band = points[(points[:, 2] >= z_min) & (points[:, 2] <= z_max)]
    if not len(band):
        return None
    distances = np.hypot(band[:, 0], band[:, 1])
    distances = distances[distances > ignore_m]
    return float(np.min(distances)) if len(distances) else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--path', required=True)
    parser.add_argument('--out', default='/tmp/path_clearance_live.json')
    parser.add_argument('--model', default='lite3_planar')
    parser.add_argument('--sample-m', type=float, default=0.2)
    parser.add_argument('--radius', type=float, default=0.35)
    parser.add_argument('--z-min', type=float, default=-0.1)
    parser.add_argument('--z-max', type=float, default=0.8)
    parser.add_argument('--settle-s', type=float, default=0.25)
    parser.add_argument('--scan-timeout', type=float, default=3.0)
    args = parser.parse_args()
    waypoints = load_path(args.path)
    samples = sample_path(waypoints, args.sample_m)
    rospy.init_node('path_clearance_live', anonymous=True, disable_signals=True)
    rospy.wait_for_service('/gazebo/set_model_state', timeout=20.)
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    results = []
    for index, (x, y, z) in enumerate(samples):
        state = ModelState()
        state.model_name, state.reference_frame = args.model, 'world'
        state.pose.position.x, state.pose.position.y, state.pose.position.z = x, y, z
        state.pose.orientation.w = 1.0
        set_state(state)
        rospy.sleep(args.settle_s)
        try:
            message = rospy.wait_for_message('/rslidar_points', PointCloud2,
                                             timeout=args.scan_timeout)
            distance = scan_clearance(message, args.z_min, args.z_max)
        except Exception as error:  # noqa: BLE001 - report probe failure
            distance = None
            results.append({'index': index, 'xyz': [x, y, z], 'error': str(error)})
            continue
        results.append({'index': index, 'xyz': [x, y, z], 'clearance_m': distance})
    values = [item['clearance_m'] for item in results if item.get('clearance_m') is not None]
    result = {
        'scope': 'path_clearance_live',
        'path': str(Path(args.path).resolve()),
        'samples': len(samples), 'scanned': len(values),
        'radius_m': args.radius,
        'z_band_m': [args.z_min, args.z_max],
        'min_clearance_m': min(values) if values else None,
        'p05_clearance_m': float(np.percentile(values, 5)) if values else None,
        'below_radius_samples': sum(1 for value in values if value < args.radius),
        'ok': bool(values) and min(values) >= args.radius,
        'samples_detail': results,
        'boundary': 'live LiDAR visibility; not physics collision proof',
    }
    Path(args.out).write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n',
                              encoding='utf-8')
    print(json.dumps({key: result[key] for key in
                      ('ok', 'samples', 'scanned', 'min_clearance_m',
                       'p05_clearance_m', 'below_radius_samples')}, ensure_ascii=False))
    return 0 if result['ok'] else 6


if __name__ == '__main__':
    raise SystemExit(main())
