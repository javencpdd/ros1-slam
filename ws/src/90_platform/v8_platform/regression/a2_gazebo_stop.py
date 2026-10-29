#!/usr/bin/env python3
"""A2 Gazebo candidate-source crash / supervisor hold → physical stop probe.

Prerequisites: an isolated master, Gazebo lite3_planar and safe_cmd_vel_mux.
This script owns only its candidate source process and hold input. It observes
Gazebo model_states pose/twist, never /sim/odom_gt.twist (the latter is command
echo). Detection tolerances below define measurements, not frozen #151 budgets.
"""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time

BASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BASE / 'orchestrator'))
import orchestrator as orch  # noqa: E402
from process_runner import ProcessRunner  # noqa: E402
from a5_gazebo_runtime import sensed_box  # noqa: E402


def owned_source(topic, speed):
    import rospy
    from geometry_msgs.msg import Twist
    rospy.init_node('a2_owned_candidate', anonymous=True)
    pub = rospy.Publisher(topic, Twist, queue_size=1)
    deadline = time.monotonic() + 15
    while pub.get_num_connections() == 0 and time.monotonic() < deadline:
        time.sleep(0.02)
    if pub.get_num_connections() == 0:
        raise RuntimeError('candidate has no mux subscriber')
    print('READY', flush=True)
    msg = Twist()
    msg.linear.x = speed
    while not rospy.is_shutdown():
        pub.publish(msg)
        time.sleep(0.05)  # wall bounded even if /clock pauses


def wait_for(predicate, seconds, label):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.005)
    raise RuntimeError('observation timeout: ' + label)


def probe(args):
    import rosgraph
    import rospy
    from gazebo_msgs.msg import ModelStates
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs import point_cloud2 as pc2
    from std_msgs.msg import Bool

    rospy.init_node('a2_gazebo_stop', anonymous=True, disable_signals=True)
    guard = threading.Lock()
    physics = deque(maxlen=50000)
    commands = deque(maxlen=50000)
    perception = {'enabled': False, 'trigger': None}
    hold_active = False

    def model_cb(msg):
        if args.model not in msg.name:
            return
        i = msg.name.index(args.model)
        pos, twist = msg.pose[i].position, msg.twist[i]
        with guard:
            physics.append({'wall': time.monotonic(), 'sim': rospy.Time.now().to_sec(),
                            'p': [pos.x, pos.y, pos.z],
                            'linear': [twist.linear.x, twist.linear.y, twist.linear.z],
                            'angular': [twist.angular.x, twist.angular.y, twist.angular.z]})

    def cmd_cb(msg):
        with guard:
            commands.append({'wall': time.monotonic(), 'sim': rospy.Time.now().to_sec(),
                             'v': [msg.linear.x, msg.linear.y, msg.angular.z]})

    def cloud_cb(msg):
        nonlocal hold_active
        if not perception['enabled'] or perception['trigger'] is not None:
            return
        points = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        observed, _roi = sensed_box(points)
        if observed is None:
            return
        with guard:
            perception['trigger'] = {'wall': time.monotonic(),
                                     'sim': rospy.Time.now().to_sec(), 'obstacle': observed}
        hold.publish(Bool(data=True))
        hold_active = True

    def samples(queue):
        with guard:
            return list(queue)

    rospy.Subscriber('/gazebo/model_states', ModelStates, model_cb, queue_size=100)
    rospy.Subscriber('/rslidar_points', PointCloud2, cloud_cb, queue_size=1)
    rospy.Subscriber(args.command_topic, Twist, cmd_cb, queue_size=100)
    hold = rospy.Publisher(args.hold_topic, Bool, queue_size=1, latch=True)
    runner = None
    cases = []
    box_name = None
    raw = []
    try:
        wait_for(lambda: len(samples(physics)) > 2, 30, 'Gazebo physical samples')
        wait_for(lambda: hold.get_num_connections() > 0, 15, 'hold subscriber')
        publishers = dict(rosgraph.Master('/a2_gazebo_stop').getSystemState()[0])
        cmd_writers = sorted(publishers.get(args.command_topic, []))
        if cmd_writers != [args.mux_node]:
            raise RuntimeError('command ownership mismatch: %r' % cmd_writers)
        existing_sources = publishers.get(args.source_topic, [])
        if existing_sources:
            raise RuntimeError('candidate source already owned: %r' % existing_sources)
        hold.publish(Bool(data=False))
        for case in ('source_crash', 'supervisor_hold', 'perception_hold'):
            if case == 'perception_hold':
                from gazebo_msgs.srv import SpawnModel
                from geometry_msgs.msg import Pose
                box_name = 'a2_perception_box_%d' % os.getpid()
                xml = ('<sdf version="1.6"><model name="%s"><static>true</static><link name="body">'
                       '<collision name="collision"><geometry><box><size>.4 .4 .6</size></box></geometry></collision>'
                       '<visual name="visual"><geometry><box><size>.4 .4 .6</size></box></geometry></visual>'
                       '</link></model></sdf>') % box_name
                pose = Pose()
                pose.position.x, pose.position.z, pose.orientation.w = 1., .3, 1.
                response = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)(
                    box_name, xml, '', pose, 'world')
                if not response.success:
                    raise RuntimeError(response.status_message)
                perception.update(enabled=False, trigger=None)
            # Start a new named source for each segment; readiness is connection-based.
            spec = orch.ComponentSpec('a2_owned_candidate', ['controller'], 'process_group_signal')
            runner = ProcessRunner(spec, [sys.executable, '-B', str(Path(__file__).resolve()),
                                         '--source', '--source-topic', args.source_topic,
                                         '--speed', str(args.speed)], ready_marker='READY')
            runner.start()
            orch.wait_ready([runner], timeout=20)
            begin_wall = time.monotonic()
            begin = samples(physics)[-1]

            def moving():
                later = [p for p in samples(physics) if p['wall'] >= begin_wall]
                if len(later) < 3:
                    return None
                p = later[-1]
                moved = math.hypot(p['p'][0] - begin['p'][0], p['p'][1] - begin['p'][1])
                speed = math.hypot(*p['linear'][:2])
                return p if moved >= 0.02 and speed >= 0.03 else None

            before = wait_for(moving, args.observation_timeout, 'physical movement: ' + case)
            if case == 'perception_hold':
                perception['enabled'] = True
                trigger = wait_for(lambda: perception['trigger'], args.observation_timeout,
                                   'lidar perception: ' + case)
                trigger_wall, trigger_sim = trigger['wall'], trigger['sim']
            else:
                trigger_wall = time.monotonic()
                trigger_sim = rospy.Time.now().to_sec()
                if case == 'source_crash':
                    os.kill(runner.proc.pid, signal.SIGKILL)
                    wait_for(lambda: not runner.is_alive(), 3, 'candidate leader death')
                else:
                    hold.publish(Bool(data=True))
                    hold_active = True

            def zero_after_trigger():
                return next((c for c in samples(commands)
                             if c['wall'] >= trigger_wall and sum(abs(v) for v in c['v']) <= 1e-3), None)

            zero = wait_for(zero_after_trigger, args.observation_timeout, 'fresh mux zero: ' + case)

            def stopped():
                ps = [p for p in samples(physics) if p['wall'] >= zero['wall']]
                if len(ps) < 3:
                    return None
                latest = ps[-1]
                stable = [p for p in ps if latest['sim'] - p['sim'] <= 0.3]
                if len(stable) < 3 or latest['sim'] - stable[0]['sim'] < 0.2:
                    return None
                if any(math.hypot(*p['linear'][:2]) > 0.01 or abs(p['angular'][2]) > 0.01
                       for p in stable):
                    return None
                excursion = max(math.hypot(p['p'][0] - stable[0]['p'][0],
                                           p['p'][1] - stable[0]['p'][1]) for p in stable)
                return stable[0] if excursion <= 0.005 else None

            stopped_at = wait_for(stopped, args.observation_timeout, 'physical stable stop: ' + case)
            after = samples(physics)
            segment = [p for p in after if trigger_wall <= p['wall'] <= stopped_at['wall']]
            coast = math.hypot(stopped_at['p'][0] - before['p'][0],
                               stopped_at['p'][1] - before['p'][1])
            cases.append({'case': case, 'verdict': 'PASS', 'source_pid': runner.proc.pid,
                          'trigger_wall': trigger_wall, 'trigger_sim': trigger_sim,
                          'mux_zero_wall_s': zero['wall'] - trigger_wall,
                          'mux_zero_sim_s': zero['sim'] - trigger_sim,
                          'physical_stop_wall_s': stopped_at['wall'] - trigger_wall,
                          'physical_stop_sim_s': stopped_at['sim'] - trigger_sim,
                          'coast_displacement_xy_m': coast,
                          'speed_before_m_s': math.hypot(*before['linear'][:2]),
                          'speed_after_m_s': math.hypot(*stopped_at['linear'][:2]),
                          'samples_after_trigger': len(segment),
                          'source_alive_at_stop': runner.is_alive()})
            raw.append({'case': case, 'physics': [p for p in after if p['wall'] >= begin_wall],
                        'commands': [c for c in samples(commands) if c['wall'] >= begin_wall]})
            runner.reap()
            runner = None
            if box_name is not None:
                from gazebo_msgs.srv import DeleteModel
                rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)(box_name)
                box_name = None
                perception.update(enabled=False, trigger=None)
            if case == 'supervisor_hold':
                # A dead publisher can leave a still-fresh nonzero candidate in the
                # mux cache. Keep hold until that final message has really expired.
                killed_sim = rospy.Time.now().to_sec()
                wait_for(lambda: rospy.Time.now().to_sec() - killed_sim > args.source_timeout_sim + 0.1,
                         args.observation_timeout, 'candidate cache expiry before hold release')
            hold.publish(Bool(data=False))
            hold_active = False
        return {'scope': 'A2.Gazebo_mux_candidate_crash_hold_perception;no_151_freeze',
                'verdict': 'PASS', 'command_writers': cmd_writers,
                'measurement_timebase': {'wall': 'time.monotonic receipt/trigger',
                                         'sim': 'current /clock at callback receipt; ModelStates has no header'},
                'detector': {'moving_xy_m': 0.02, 'moving_speed_m_s': 0.03,
                             'stopped_speed_m_s': 0.01, 'stable_span_sim_s': 0.2,
                             'stable_excursion_xy_m': 0.005, 'budget_frozen': False},
                'cases': cases, 'raw_segments': raw}
    except Exception as exc:
        return {'scope': 'A2.Gazebo_mux_candidate_crash_hold_perception;no_151_freeze',
                'verdict': 'FAIL', 'error': str(exc), 'cases': cases,
                'raw_segments': raw + [{'case': 'failure_snapshot',
                                        'physics': samples(physics), 'commands': samples(commands)}]}
    finally:
        if runner is not None:
            runner.reap()
        # Only release hold after our source is dead; no stale candidate can resume.
        if hold_active:
            cleanup_sim = rospy.Time.now().to_sec()
            wait_for(lambda: rospy.Time.now().to_sec() - cleanup_sim > args.source_timeout_sim + 0.1,
                     args.observation_timeout, 'candidate cache expiry during cleanup')
        hold.publish(Bool(data=False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out')
    parser.add_argument('--source', action='store_true')
    parser.add_argument('--source-topic', default='/follow/cmd_vel')
    parser.add_argument('--command-topic', default='/cmd_vel')
    parser.add_argument('--hold-topic', default='/lite3/supervisor/hold')
    parser.add_argument('--mux-node', default='/mux')
    parser.add_argument('--model', default='lite3_planar')
    parser.add_argument('--speed', type=float, default=0.2)
    parser.add_argument('--observation-timeout', type=float, default=35)
    parser.add_argument('--source-timeout-sim', type=float, default=0.5)
    args = parser.parse_args()
    if args.source:
        owned_source(args.source_topic, args.speed)
        return 0
    try:
        result = probe(args)
    except Exception as exc:
        result = {'verdict': 'FAIL', 'error': str(exc),
                  'scope': 'A2.Gazebo_mux_candidate_crash_hold_perception;no_151_freeze'}
    encoded = json.dumps(result, ensure_ascii=False, indent=2) + '\n'
    if args.out:
        Path(args.out).write_text(encoded)
    # Raw samples live in the artifact; stdout stays small for orchestration logs.
    print(json.dumps({k: v for k, v in result.items() if k != 'raw_segments'}, ensure_ascii=False))
    return 0 if result['verdict'] == 'PASS' else 6


if __name__ == '__main__':
    sys.exit(main())
