#!/usr/bin/env python3
"""ROS1 topic adapter for the real support planner: map/start/goal in, Path out."""
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import support_planner  # noqa: E402
import ros_node as legacy  # noqa: E402


def plan_request(map_value, start, goal, budget=20000, min_clearance_m=None,
                 robot_radius_m=None, robot_height_m=None):
    return support_planner.plan(map_value, tuple(start), tuple(goal), budget=budget,
                                min_clearance_m=min_clearance_m,
                                robot_radius_m=robot_radius_m,
                                robot_height_m=robot_height_m)


def map_from_message(text):
    value = json.loads(text)
    if isinstance(value, dict) and 'map' in value:
        value = value['map']
    if not isinstance(value, dict):
        raise ValueError('map message must be an object')
    return value


def main():
    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from std_msgs.msg import String

    rospy.init_node('v8_support_planner_topic')
    frame = rospy.get_param('~frame', 'map')
    budget = int(rospy.get_param('~budget', 20000))
    policy = {key: rospy.get_param('~' + key, None)
              for key in ('min_clearance_m', 'robot_radius_m', 'robot_height_m')}
    path_pub = rospy.Publisher(rospy.get_param('~path_topic', '/global_path'),
                               Path, queue_size=1, latch=True)
    status_pub = rospy.Publisher(rospy.get_param('~status_topic', '/global_path/status'),
                                 String, queue_size=10, latch=True)
    state = {'map': None, 'map_version': None, 'start': None, 'goal': None}

    def publish(value):
        status_pub.publish(String(data=json.dumps(value, ensure_ascii=False)))

    def replan():
        if state['map'] is None or state['start'] is None or state['goal'] is None:
            publish({'ok': False, 'reason': 'E_PLAN_NOT_READY',
                     'map_version': state['map_version']})
            return
        try:
            result = plan_request(state['map'], state['start'], state['goal'],
                                  budget=budget, **policy)
        except Exception as error:
            path_pub.publish(Path())
            publish(dict(legacy.refusal(error), map_version=state['map_version']))
            return
        points = legacy.to_path_points(result)
        message = Path()
        message.header.frame_id, message.header.stamp = frame, rospy.Time.now()
        for x, y, z, _surface in points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
            pose.pose.orientation.w = 1.
            message.poses.append(pose)
        path_pub.publish(message)
        publish({'ok': True, 'reason': result.get('reason'),
                 'points': len(points), 'map_version': state['map_version'],
                 'expansions': result.get('expansions'),
                 'goal': list(state['goal'])})

    def on_map(message):
        try:
            value = map_from_message(message.data)
        except ValueError:
            publish({'ok': False, 'reason': 'E_PLAN_MAP_JSON'})
            return
        state['map'], state['map_version'] = value, value.get('map_version')
        replan()

    def on_pose(key):
        def callback(message):
            if message.header.frame_id != frame:
                publish({'ok': False, 'reason': 'E_PLAN_FRAME'})
                return
            p = message.pose.position
            state[key] = (p.x, p.y, p.z)
            replan()
        return callback

    rospy.Subscriber(rospy.get_param('~map_topic', '/v8_support_map'), String,
                     on_map, queue_size=1)
    rospy.Subscriber(rospy.get_param('~start_topic', '/v8/global/start'), PoseStamped,
                     on_pose('start'), queue_size=1)
    rospy.Subscriber(rospy.get_param('~goal_topic', '/v8/global/goal'), PoseStamped,
                     on_pose('goal'), queue_size=1)
    keep_alive_s = float(rospy.get_param('~keep_alive_s', 0.0))
    publish({'ok': True, 'waiting': True})
    deadline = time.monotonic() + keep_alive_s
    while not rospy.is_shutdown():
        if keep_alive_s > 0 and time.monotonic() >= deadline:
            break
        time.sleep(.05)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
