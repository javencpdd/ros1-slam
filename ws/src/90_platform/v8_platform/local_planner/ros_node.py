"""ROS1 adapter for the native space-time planner; explicit map-frame 3D IO.

Separate wall-clock controller and planning loops ensure a slow/failed planner
cannot keep an old nonzero command alive. This does not connect to real hardware
or silently convert a 3D velocity into the existing planar cmd_vel interface.
"""
import json
import math
import threading
import time

import local_native as native


def freshness(now, pose_received, obstacles_received, localization_received,
              localization_ok, max_age_s):
    if not localization_ok or now - localization_received >= max_age_s:
        return 'E_LOCAL_LOCALIZATION'
    if now - pose_received >= max_age_s:
        return 'E_LOCAL_POSE_STALE'
    if now - obstacles_received >= max_age_s:
        return 'E_LOCAL_OBSTACLES_STALE'
    return None


def feedback_at(pose, observed_at, now, commands):
    """Predict to controller time from timestamped feedback and issued commands.

    This changes the controller estimate only. Received/raw poses are never
    moved or snapped. Piecewise integration avoids braking against an old pose
    while the previous nonzero command is still moving the experimental carrier.
    """
    if observed_at <= 0:
        return tuple(pose)
    if observed_at > now:
        raise ValueError('pose timestamp is in the future')
    position, cursor, velocity = tuple(pose), observed_at, (0., 0., 0.)
    for stamp, issued in commands:
        if stamp <= observed_at:
            velocity = tuple(issued)
        elif stamp <= now:
            position = tuple(position[i] + (stamp - cursor) * velocity[i] for i in range(3))
            cursor, velocity = stamp, tuple(issued)
        else:
            break
    return tuple(position[i] + (now - cursor) * velocity[i] for i in range(3))


def feedback_uncertainty(observed_at, now, commands, latency_bound_s):
    """Distance bound from issued-vs-applied timing and one pending command."""
    velocity, variation = (0., 0., 0.), 0.
    for stamp, issued in commands:
        if stamp > now:
            break
        if stamp > observed_at:
            variation += math.sqrt(sum((issued[i] - velocity[i]) ** 2 for i in range(3)))
        velocity = tuple(issued)
    return latency_bound_s * (variation + math.sqrt(sum(v * v for v in velocity)))


def main():
    import rospy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from std_msgs.msg import Bool, String

    rospy.init_node('v8_native_local_planner')
    config = native.Config(**rospy.get_param('~experimental_config', {}))
    config.validate()
    max_age = float(rospy.get_param('~input_max_age_s', 1.0))
    if max_age <= 0:
        raise ValueError('input_max_age_s must be positive')
    map_path = rospy.get_param('~map_json')
    profile_path = rospy.get_param('~robot_profile_json', '')
    profile = None
    if profile_path:
        with open(profile_path, encoding='utf-8') as stream:
            profile = json.load(stream)
    # Missing localization or obstacle heartbeat is unsafe, including at startup.
    shared = {'pose': None, 'goal': None, 'pose_at': -1e100, 'obstacles': [],
              'obstacles_at': -1e100, 'localization_at': -1e100, 'localization_ok': False,
              'result': None, 'result_at': -1e100, 'map': None, 'generation': 0,
              'result_generation': -1, 'last_error': 'E_LOCAL_NOT_READY', 'obstacle_key': None,
              'pose_ros_stamp': 0.}
    lock = threading.Lock()
    command_pub = rospy.Publisher(rospy.get_param('~command_topic', '/v8/local/cmd_vel_3d'),
                                  TwistStamped, queue_size=1)
    path_pub = rospy.Publisher(rospy.get_param('~trajectory_topic', '/v8/local/trajectory'),
                               String, queue_size=1, latch=True)
    status_pub = rospy.Publisher(rospy.get_param('~status_topic', '/v8/local/status'),
                                 String, queue_size=1, latch=True)

    def on_pose(msg):
        with lock:
            if msg.header.frame_id != 'map':
                shared['last_error'] = 'E_LOCAL_FRAME'
                shared['pose_at'] = -1e100
                return
            p = msg.pose.position
            shared.update(pose=(p.x, p.y, p.z), pose_at=time.monotonic(),
                          pose_ros_stamp=msg.header.stamp.to_sec())

    def on_goal(msg):
        with lock:
            if msg.header.frame_id != 'map':
                shared.update(goal=None, last_error='E_LOCAL_FRAME')
                return
            p = msg.pose.position
            shared.update(goal=(p.x, p.y, p.z), generation=shared['generation'] + 1, result=None)

    def on_obstacles(msg):
        try:
            payload = json.loads(msg.data)
            values = payload['obstacles'] if isinstance(payload, dict) else payload
            native._obstacles(values)
            # Repeated heartbeat with identical prediction does not discard valid motion.
            key = json.dumps(payload, sort_keys=True, separators=(',', ':'))
            with lock:
                if key != shared['obstacle_key']:
                    shared.update(generation=shared['generation'] + 1, result=None)
                shared.update(obstacles=values, obstacles_at=time.monotonic(), obstacle_key=key)
        except (ValueError, TypeError, KeyError, native.LocalError):
            with lock:
                shared.update(obstacles_at=-1e100, result=None, last_error='E_LOCAL_OBSTACLES_INVALID')

    def on_localization(msg):
        with lock:
            shared.update(localization_ok=msg.data, localization_at=time.monotonic())
            if not msg.data:
                shared.update(result=None, generation=shared['generation'] + 1)

    rospy.Subscriber(rospy.get_param('~pose_topic', '/v8/local/pose'), PoseStamped, on_pose, queue_size=1)
    rospy.Subscriber(rospy.get_param('~goal_topic', '/v8/local/goal'), PoseStamped, on_goal, queue_size=1)
    rospy.Subscriber(rospy.get_param('~obstacles_topic', '/v8/local/obstacles'), String, on_obstacles, queue_size=1)
    rospy.Subscriber(rospy.get_param('~localization_topic', '/v8/local/localization_ok'),
                     Bool, on_localization, queue_size=1)

    def control_loop():
        command_history = []
        previous_control_stamp = None
        while not rospy.is_shutdown():
            wall_now = time.monotonic()
            with lock:
                snapshot = dict(shared)
            reason = freshness(wall_now, snapshot['pose_at'], snapshot['obstacles_at'],
                               snapshot['localization_at'], snapshot['localization_ok'], max_age)
            if reason is None and snapshot['result'] is None:
                reason = snapshot['last_error']
            if reason is None and wall_now - snapshot['result_at'] >= config.validity_s:
                reason = 'E_LOCAL_WALL_EXPIRED'
            if reason is None and snapshot['generation'] != snapshot['result_generation']:
                reason = 'E_LOCAL_WORLD_UPDATED'
            stamp = rospy.Time.now()
            if not reason and snapshot['map'] is not None:
                try:
                    estimated = feedback_at(snapshot['pose'], snapshot['pose_ros_stamp'], stamp.to_sec(), command_history)
                    base_period = (.05 if previous_control_stamp is None else
                                   max(.05, min(.1, stamp.to_sec() - previous_control_stamp)))
                    uncertainty = feedback_uncertainty(snapshot['pose_ros_stamp'], stamp.to_sec(),
                                                       command_history, config.command_latency_bound_s)
                    out = native.track_segment(snapshot['map'], snapshot['result'], estimated,
                                               stamp.to_sec(), snapshot['obstacles'], config=config,
                                               control_period_s=base_period,
                                               feedback_pose=snapshot['pose'], uncertainty_m=uncertainty)
                except ValueError:
                    out = {'velocity': [0., 0., 0.], 'stop': True, 'reason': 'E_LOCAL_CLOCK'}
            else:
                out = {'velocity': [0., 0., 0.], 'stop': True, 'reason': reason or snapshot['last_error']}
            message = TwistStamped()
            message.header.frame_id, message.header.stamp = 'map', stamp
            message.twist.linear.x, message.twist.linear.y, message.twist.linear.z = out['velocity']
            command_pub.publish(message)
            command_history.append((stamp.to_sec(), tuple(out['velocity'])))
            command_history[:] = command_history[-200:]
            previous_control_stamp = stamp.to_sec()
            status_pub.publish(String(data=json.dumps(dict(out, clock='ros+wall_watchdog',
                                                           capability_domain='experimental_kinematic_carrier'
                                                           if profile is None else 'validated_robot_profile'))))
            if out.get('complete') or out['reason'].startswith('E_LOCAL_'):
                with lock:
                    if shared['result'] is snapshot['result']:
                        shared.update(result=None, last_error=out['reason'])
            # Intentionally wall time: paused /clock cannot suspend the zero-command watchdog.
            time.sleep(.05)

    thread = threading.Thread(target=control_loop, daemon=True)
    thread.start()
    try:
        while not rospy.is_shutdown():
            started = time.monotonic()
            # Simulation fault injection: the independent control/watchdog loop
            # keeps running while planner refresh is deliberately suspended.
            if rospy.get_param('~pause_planning', False):
                time.sleep(.05)
                continue
            with lock:
                snapshot = dict(shared)
            try:
                with open(map_path, encoding='utf-8') as stream:
                    value = json.load(stream)
                with lock:
                    shared['map'] = value
                # Replanning from a held contact pose while its command is
                # still being executed changes direction mid-phase and loses
                # the state/time relation used by collision prediction.
                if snapshot['result'] is not None:
                    time.sleep(max(.01, .2 - (time.monotonic() - started)))
                    continue
                reason = freshness(started, snapshot['pose_at'], snapshot['obstacles_at'],
                                   snapshot['localization_at'], snapshot['localization_ok'], max_age)
                stamp = rospy.Time.now().to_sec()
                if reason is None and snapshot['pose'] is not None and snapshot['goal'] is not None and stamp > 0:
                    result = native.plan(value, snapshot['pose'], snapshot['goal'], snapshot['obstacles'],
                                         now=stamp, config=config, robot_profile=profile)
                    with lock:
                        if snapshot['generation'] == shared['generation']:
                            shared.update(result=result, result_at=started,
                                          result_generation=snapshot['generation'], last_error='ok')
                    path_pub.publish(String(data=json.dumps(result)))
                else:
                    with lock:
                        shared.update(result=None, last_error=reason or 'E_LOCAL_NOT_READY')
            except (native.LocalError, OSError, ValueError, KeyError) as error:
                with lock:
                    shared.update(result=None, last_error=getattr(error, 'code', 'E_LOCAL_INPUT'))
                rospy.logwarn_throttle(2., 'local planner refused: %s', error)
            time.sleep(max(.01, .2 - (time.monotonic() - started)))
    finally:
        command_pub.publish(TwistStamped())
        thread.join(timeout=1.)


if __name__ == '__main__':
    main()
