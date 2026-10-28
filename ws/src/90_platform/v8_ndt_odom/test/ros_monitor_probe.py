"""Real NDT/ROS monitor interface test using declared synthetic point clouds.

This validates acceptance, rejection freshness and reset. It is not a physical
sensor accuracy trial and does not promote NDT to a navigation backend.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import tempfile
import time


def wait_for(predicate, label, seconds=15):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.02)
    raise RuntimeError('NDT observation timeout: ' + label)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', default='/tmp/v8-ndt-build/devel/lib/v8_ndt_odom/ndt_odom_node')
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    import rospy
    from sensor_msgs.msg import PointCloud2, PointField
    from sensor_msgs import point_cloud2
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, Header
    from std_srvs.srv import Trigger
    rospy.init_node('ndt_monitor_probe', anonymous=True)
    output = Path(args.out)
    output.parent.mkdir(parents=True, exist_ok=True)
    # Several unequal intersecting planes avoid a symmetric/degenerate target.
    points = []
    for i in range(41):
        for j in range(31):
            a, b = i * .05, j * .04
            points.extend([(a, -1., b, 1.), (a, 1.3, b, 1.),
                           (2.4, a - 1., b, 1.), (a, b - .6, -.2, 1.)])
    records = {'odom': [], 'health': []}
    subs = [rospy.Subscriber('/r841_ndt/odom', Odometry, lambda msg: records['odom'].append(
        {'stamp': msg.header.stamp.to_sec(), 'xyz': [msg.pose.pose.position.x,
          msg.pose.pose.position.y, msg.pose.pose.position.z],
         'covariance_diagonal': [msg.pose.covariance[i * 7] for i in range(6)]})),
        rospy.Subscriber('/r841_ndt/healthy', Bool, lambda msg: records['health'].append(
            {'wall': time.monotonic(), 'healthy': bool(msg.data)}))]
    publisher = rospy.Publisher('/r841_ndt/cloud', PointCloud2, queue_size=1)
    fields = [PointField(name, i * 4, PointField.FLOAT32, 1)
              for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
    result = {'domain': 'synthetic_pointcloud_ROS_real_NDT_monitor',
              'accuracy_backend_switch_tested': False, 'cases': [], 'records': records}
    with tempfile.TemporaryDirectory(prefix='ndt_monitor_') as temporary:
        pcd = Path(temporary) / 'map.pcd'
        header = ('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n'
                  'TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\n'
                  'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n' % (len(points), len(points)))
        pcd.write_text(header + '\n'.join(' '.join(map(str, p)) for p in points))
        with (output.parent / 'ndt-monitor-node.log').open('wb') as log:
            process = subprocess.Popen([args.binary, '__name:=r841_ndt',
                '_map_path:=' + str(pcd), '_cloud_topic:=/r841_ndt/cloud',
                '_ext_z:=0.0', '_ext_pitch:=0.0', '_resolution:=0.5',
                '_score_max:=0.05', '_max_iterations:=30'], stdout=log, stderr=subprocess.STDOUT)
            try:
                wait_for(lambda: publisher.get_num_connections() > 0 and bool(records['health']), 'node ready')

                def send(stamp, dx=0., count=None):
                    data = [(x - dx, y, z, intensity) for x, y, z, intensity in points]
                    if count is not None:
                        data = data[:count]
                    publisher.publish(point_cloud2.create_cloud(
                        Header(stamp=rospy.Time.from_sec(stamp), frame_id='lidar'), fields, data))

                for stamp, dx in ((1., 0.), (2., .05), (3., .10)):
                    before = len(records['odom'])
                    send(stamp, dx)
                    wait_for(lambda: len(records['odom']) > before, 'accepted cloud %.1f' % stamp)
                    observed = records['odom'][-1]
                    result['cases'].append({'case': 'accepted_' + str(stamp),
                        'ok': abs(observed['xyz'][0] - dx) < .05,
                        'observed': observed, 'expected_x': dx})
                before_odom, before_health = len(records['odom']), len(records['health'])
                send(4., count=10)
                wait_for(lambda: len(records['health']) > before_health, 'sparse rejection')
                time.sleep(.1)
                result['cases'].append({'case': 'sparse_rejection_not_fresh',
                    'ok': len(records['odom']) == before_odom and not records['health'][-1]['healthy']})
                before_health = len(records['health'])
                send(2., .05)
                wait_for(lambda: len(records['health']) > before_health, 'old stamp rejection')
                time.sleep(.1)
                result['cases'].append({'case': 'old_stamp_not_fresh',
                    'ok': len(records['odom']) == before_odom and not records['health'][-1]['healthy']})
                before_health = len(records['health'])
                response = rospy.ServiceProxy('/r841_ndt/reset', Trigger)()
                wait_for(lambda: len(records['health']) > before_health, 'reset health')
                result['cases'].append({'case': 'reset_revokes_health',
                    'ok': response.success and not records['health'][-1]['healthy']})
                send(.5, 0.)  # Reset must clear the former timestamp/history.
                wait_for(lambda: len(records['odom']) > before_odom, 'accept after reset')
                result['cases'].append({'case': 'reset_clears_timestamp',
                    'ok': records['odom'][-1]['stamp'] == .5 and records['health'][-1]['healthy']})
                result['ok'] = all(row['ok'] for row in result['cases'])
            except Exception as error:
                result.update(ok=False, error=str(error))
            finally:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
                for sub in subs:
                    sub.unregister()
                publisher.unregister()
    output.write_text(json.dumps(result, indent=2))
    print(json.dumps({'ok': result['ok'], 'cases': result['cases'], 'error': result.get('error')}))
    return 0 if result['ok'] else 6


if __name__ == '__main__':
    raise SystemExit(main())
