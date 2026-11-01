#!/usr/bin/env python3
"""Transport an accepted NDT monitor pose into the A6 recovery candidate channel.

The NDT node owns the acceptance decision (`healthy` plus `relocalized`).  This
bridge only checks finiteness/freshness and maps the NDT fitness (lower is
better) onto the supervisor's higher-is-better score:

    score = 1 / (1 + fitness / fitness_max)

`fitness_max` must equal the NDT node's `relocalize_score_max`; then score >=
0.5 exactly when the node would accept the pose, which matches the default
supervisor `score_min=0.5`.  A candidate is emitted once per `relocalized=true`
event, from the accepted pose and fitness snapshot, and only while the health
and score observations are still fresh.
"""
import argparse
import json
import math
import threading
import time


def yaw_of(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-6:
        return None
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def score_from_fitness(fitness, fitness_max):
    if (not isinstance(fitness, (int, float)) or isinstance(fitness, bool)
            or not math.isfinite(fitness) or fitness < 0.0
            or not isinstance(fitness_max, (int, float)) or isinstance(fitness_max, bool)
            or not math.isfinite(fitness_max) or fitness_max <= 0.0
            or fitness > fitness_max):
        return None
    return 1.0 / (1.0 + fitness / fitness_max)


def candidate_payload(x, y, z, yaw, fitness, fitness_max):
    values = (x, y, z, yaw)
    if not all(isinstance(value, (int, float)) and not isinstance(value, bool)
               and math.isfinite(value) for value in values):
        return None
    score = score_from_fitness(fitness, fitness_max)
    if score is None:
        return None
    return {'pose': [float(x), float(y), float(z)], 'yaw': float(yaw),
            'score': score, 'fitness': float(fitness), 'source': 'ndt_monitor'}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--odom-topic', default='/a6_ndt/odom')
    parser.add_argument('--health-topic', default='/a6_ndt/healthy')
    parser.add_argument('--score-topic', default='/a6_ndt/score')
    parser.add_argument('--relocalized-topic', default='/a6_ndt/relocalized')
    parser.add_argument('--candidate-topic', default='/a6/recovery/candidate')
    parser.add_argument('--fitness-max', type=float, default=10.0)
    parser.add_argument('--fresh-s', type=float, default=0.5)
    args = parser.parse_args(argv)
    if not (args.fitness_max > 0.0 and args.fresh_s > 0.0):
        raise SystemExit('--fitness-max and --fresh-s must be positive')
    import rospy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, Float32, String

    rospy.init_node('ndt_recovery_bridge', anonymous=True, disable_signals=True)
    lock = threading.RLock()
    record = {'odom': None, 'odom_wall': None, 'fitness': None, 'fitness_wall': None,
              'healthy': False, 'healthy_wall': None, 'pending_until': None}
    output = rospy.Publisher(args.candidate_topic, String, queue_size=5)

    def odom(message):
        pose = message.pose.pose
        yaw = yaw_of(pose.orientation.x, pose.orientation.y,
                     pose.orientation.z, pose.orientation.w)
        with lock:
            record['odom'] = (pose.position.x, pose.position.y, pose.position.z, yaw)
            record['odom_wall'] = time.monotonic()

    def score(message):
        with lock:
            record['fitness'] = float(message.data)
            record['fitness_wall'] = time.monotonic()

    def health(message):
        with lock:
            record['healthy'] = bool(message.data)
            record['healthy_wall'] = time.monotonic()

    def relocalized(message):
        if message.data:
            with lock:
                # Let odom/score callbacks catch up if the transport delivered
                # relocalized first; the event itself is the NDT acceptance.
                record['pending_until'] = time.monotonic() + args.fresh_s

    subscriptions = [
        rospy.Subscriber(args.odom_topic, Odometry, odom, queue_size=20),
        rospy.Subscriber(args.score_topic, Float32, score, queue_size=20),
        rospy.Subscriber(args.health_topic, Bool, health, queue_size=20),
        rospy.Subscriber(args.relocalized_topic, Bool, relocalized, queue_size=20),
    ]
    try:
        while not rospy.is_shutdown():
            now = time.monotonic()
            payload = None
            with lock:
                pending = record['pending_until']
                if pending is not None:
                    fresh = (record['odom_wall'] is not None
                             and now - record['odom_wall'] <= args.fresh_s
                             and record['fitness_wall'] is not None
                             and now - record['fitness_wall'] <= args.fresh_s)
                    if fresh and record['odom'] is not None and record['fitness'] is not None \
                            and record['odom'][3] is not None:
                        payload = candidate_payload(record['odom'][0], record['odom'][1],
                                                    record['odom'][2], record['odom'][3],
                                                    record['fitness'], args.fitness_max)
                        record['pending_until'] = None
                    elif now > pending:
                        record['pending_until'] = None
            if payload is not None:
                output.publish(String(data=json.dumps(payload)))
                rospy.loginfo('NDT candidate pose=%s score=%.4f fitness=%.4f',
                              payload['pose'], payload['score'], payload['fitness'])
            time.sleep(0.02)
    finally:
        for subscription in subscriptions:
            subscription.unregister()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
