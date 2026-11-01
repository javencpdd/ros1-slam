#!/usr/bin/env python3
"""ROS wrapper for the A6 RecoverySupervisor (relocalize/candidate/lease wiring)."""
import argparse
import json
import math
import threading
import time
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from localization_state import LocalizationState
from recovery_supervisor import RecoverySupervisor, TRACKING, RELOCALIZING, RECOVERING, FAILED


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--namespace', default='/a6/recovery')
    parser.add_argument('--odom-topic', default='/Odometry')
    parser.add_argument('--health-topic', default='')
    parser.add_argument('--candidate-topic', default='')
    parser.add_argument('--lease-in-topic', default='')
    parser.add_argument('--initial-lease', type=int, default=0)
    parser.add_argument('--backend-name', default='faster_lio')
    parser.add_argument('--relocalize-service', default='/relocalize')
    parser.add_argument('--relocalize-service-type', choices=('empty', 'trigger'),
                        default='empty')
    parser.add_argument('--score-min', type=float, default=0.5)
    parser.add_argument('--deadline-s', type=float, default=5.0)
    parser.add_argument('--timeout-s', type=float, default=0.25)
    parser.add_argument('--bounds', default='-100,100,-100,100')
    args = parser.parse_args(argv)
    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String, UInt64
    from std_srvs.srv import Empty, Trigger

    rospy.init_node('a6_recovery', anonymous=True, disable_signals=True)
    prefix = args.namespace.rstrip('/')
    nums = [float(v) for v in args.bounds.split(',')]
    if len(nums) != 4:
        raise SystemExit('--bounds needs min_x,max_x,min_y,max_y')
    state = LocalizationState(timeout_s=args.timeout_s)
    lock = threading.RLock()
    callbacks = []
    pending = {'lease': None}
    last_candidate_yaw = [None]
    relocalize_calls = [0]

    def relocalize():
        relocalize_calls[0] += 1
        def call():
            try:
                rospy.wait_for_service(args.relocalize_service, timeout=2.0)
                service = Trigger if args.relocalize_service_type == 'trigger' else Empty
                rospy.ServiceProxy(args.relocalize_service, service)()
            except Exception as error:
                rospy.logwarn('relocalize service: %r', error)
        threading.Thread(target=call, daemon=True).start()

    def switch_backend(lease, pose):
        pending['lease'] = lease
        backend.publish(String(data=json.dumps({'backend': args.backend_name, 'lease': lease,
                                                'pose': list(pose),
                                                'yaw': last_candidate_yaw[0]})))

    supervisor = RecoverySupervisor(
        state, relocalize, switch_backend, score_min=args.score_min,
        deadline_s=args.deadline_s,
        bounds=((nums[0], nums[1]), (nums[2], nums[3])))
    status = rospy.Publisher(prefix + '/status', String, queue_size=20)
    allowed = rospy.Publisher(prefix + '/command_allowed', Bool, queue_size=20)
    lease_pub = rospy.Publisher(prefix + '/lease', UInt64, queue_size=20)
    backend = rospy.Publisher(prefix + '/backend', String, queue_size=20)
    last_lease = [None]

    def odometry(message):
        point = message.pose.pose.position
        covariance = message.pose.covariance
        valid = all(math.isfinite(covariance[i]) and covariance[i] >= 0 for i in (0, 7, 14))
        with lock:
            state.observe(message.header.stamp.to_sec(), (point.x, point.y, point.z),
                          time.monotonic(), valid=valid)

    def candidate(message):
        try:
            payload = json.loads(message.data)
            pose = tuple(float(v) for v in payload['pose'][:3])
            score = float(payload['score'])
            yaw = payload.get('yaw')
            yaw = None if yaw is None else float(yaw)
        except (ValueError, KeyError, TypeError, IndexError):
            return
        if yaw is not None and not math.isfinite(yaw):
            return
        with lock:
            last_candidate_yaw[0] = yaw
            accepted = supervisor.offer_candidate(pose, score, time.monotonic())
        if accepted:
            rospy.loginfo('candidate accepted pose=%s score=%.3f', pose, score)
        else:
            rospy.logwarn('candidate rejected pose=%s score=%.3f phase=%s requested=%s last_lease=%s',
                          pose, score, supervisor.phase, supervisor.requested,
                          state.last_lease)

    def lease_in(message):
        with lock:
            state.activate(int(message.data), time.monotonic())

    def health(message):
        with lock:
            state.backend_health(bool(message.data))

    callbacks.append(rospy.Subscriber(args.odom_topic, Odometry, odometry, queue_size=20))
    if args.candidate_topic:
        callbacks.append(rospy.Subscriber(args.candidate_topic, String, candidate, queue_size=10))
    if args.lease_in_topic:
        callbacks.append(rospy.Subscriber(args.lease_in_topic, UInt64, lease_in, queue_size=10))
    if args.health_topic:
        state.backend_health(False)
        callbacks.append(rospy.Subscriber(args.health_topic, Bool, health, queue_size=10))
    try:
        while not rospy.is_shutdown():
            now = time.monotonic()
            with lock:
                if args.initial_lease > 0 and state.state == 'HEALTHY' \
                        and state.active_lease is None and state.last_lease < args.initial_lease:
                    state.activate(args.initial_lease, now)
                ok = supervisor.tick(now)
                active = state.active_lease
                if active is not None and active != last_lease[0]:
                    last_lease[0] = active
                    lease_pub.publish(UInt64(data=active))
                report = supervisor.report()
                report['command_allowed'] = bool(ok)
                report['relocalize_calls'] = relocalize_calls[0]
                status.publish(String(data=json.dumps(report)))
                allowed.publish(Bool(data=bool(ok)))
            time.sleep(.05)
    finally:
        for subscription in callbacks:
            subscription.unregister()
        allowed.publish(Bool(data=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
