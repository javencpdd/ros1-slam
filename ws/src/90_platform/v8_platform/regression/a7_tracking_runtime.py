"""SC11 ROS tracking runtime: reference, clean tracking, lateral deviation, stop latch.

The reference is produced by the support planner from the SC11 scenario. The
owned monitor node consumes JSON reference and PoseStamped samples; the probe
never sends a fabricated planar command. No Gazebo or real robot is claimed.
"""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
for sub in ('scenarios', 'planner', 'validator'):
    sys.path.insert(0, str(ROOT / sub))
import scenarios as scn  # noqa: E402
import support_planner as pln  # noqa: E402


def reference_for(scenario='SC11'):
    case = scn.scenario(scenario)
    route = pln.plan(case['map'], case['start'], case['goal'])
    return [[round(float(v), 6) for v in point[:3]] for point in route['path']]


def wait_for(predicate, timeout, label):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(.01)
    raise RuntimeError('observation timeout: ' + label)


def run(scenario='SC11', limit=2):
    import rospy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Bool, String

    rospy.init_node('a7_tracking_runtime', anonymous=True, disable_signals=True)
    statuses, holds = [], []
    reference = reference_for(scenario)
    name = 'a7_tracking_monitor'
    node = subprocess.Popen([sys.executable, '-B', str(ROOT / 'adapters' / 'tracking_monitor.py'),
                             '__name:=' + name,
                             '_reference_topic:=/a7_tracking/reference',
                             '_pose_topic:=/a7_tracking/pose',
                             '_status_topic:=/a7_tracking/status',
                             '_hold_topic:=/a7_tracking/hold',
                             '_rejection_limit:=%d' % limit],
                            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    ref_pub = rospy.Publisher('/a7_tracking/reference', String, queue_size=1, latch=True)
    pose_pub = rospy.Publisher('/a7_tracking/pose', PoseStamped, queue_size=20)
    subs = [rospy.Subscriber('/a7_tracking/status', String,
                             lambda m: statuses.append(json.loads(m.data)), queue_size=50),
            rospy.Subscriber('/a7_tracking/hold', Bool,
                             lambda m: holds.append(bool(m.data)), queue_size=50)]
    t0 = time.monotonic()
    result = {'scenario': scenario, 'reference_points': len(reference),
              'limit': limit, 'scope': 'SC11.tracking_monitor_ROS;no_gazebo;no_real_robot'}
    try:
        wait_for(lambda: ref_pub.get_num_connections() > 0 and pose_pub.get_num_connections() > 0,
                 20, 'monitor subscriptions')
        ref_pub.publish(String(data=json.dumps(reference)))
        wait_for(lambda: any(s.get('reason') == 'reference_ready' for s in statuses),
                 10, 'reference ready')
        clean_count = min(20, len(reference))
        for point in reference[::max(1, len(reference) // clean_count)][:clean_count]:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.orientation.w = 1.
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point
            pose_pub.publish(pose)
            time.sleep(.02)
        wait_for(lambda: len(statuses) > clean_count, 5, 'clean statuses')
        clean_stop = any(s.get('stop') for s in statuses) or any(holds)
        mid = reference[len(reference) // 2]
        injected = (mid[0] + .5, mid[1], mid[2])
        for _ in range(limit):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.orientation.w = 1.
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = injected
            pose_pub.publish(pose)
            time.sleep(.05)
        stopped = wait_for(lambda: next((s for s in statuses if s.get('stop')), None),
                           10, 'tracking stop')
        result.update({
            'clean_samples': clean_count, 'clean_had_stop': clean_stop,
            'deviated_pose': list(injected), 'stop_reason': stopped.get('reason'),
            'stop_tracker': stopped.get('tracker'), 'stop_consecutive': stopped.get('consecutive'),
            'hold_true': any(holds), 'statuses': statuses, 'holds': holds,
            'elapsed_wall_s': round(time.monotonic() - t0, 3)})
        result['ok'] = (not clean_stop and stopped.get('reason') == 'E_TRACK_DEVIATION'
                        and stopped.get('tracker') == 'stop' and any(holds))
    except Exception as error:
        result.update(ok=False, error=str(error), statuses=statuses, holds=holds)
    finally:
        node.terminate()
        try:
            node.wait(timeout=5)
        except subprocess.TimeoutExpired:
            node.kill()
            node.wait(timeout=5)
        for sub in subs:
            sub.unregister()
        ref_pub.unregister()
        pose_pub.unregister()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', default='')
    parser.add_argument('--limit', type=int, default=2)
    args = parser.parse_args()
    result = run(limit=args.limit)
    encoded = json.dumps(result, ensure_ascii=False, indent=1) + '\n'
    if args.out:
        Path(args.out).write_text(encoded)
    print(json.dumps({k: result.get(k) for k in ('ok', 'stop_reason', 'stop_tracker',
                                                'clean_had_stop', 'hold_true', 'reference_points')},
                     ensure_ascii=False))
    return 0 if result.get('ok') else 6


if __name__ == '__main__':
    raise SystemExit(main())
