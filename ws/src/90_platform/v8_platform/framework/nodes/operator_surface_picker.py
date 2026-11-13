#!/usr/bin/env python3
"""3D operator goal picker: click a support surface marker -> PoseStamped goal.

RViz 'Publish Point' provides a 3D PointStamped. The node resolves the clicked
x/y/z to exactly one support surface; ambiguous 2D clicks return the candidate
list instead of guessing. The selected surface id is published in status.
"""
import os as _os
import sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
import json

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import String

import surface_resolver


def main():
    rospy.init_node('operator_surface_picker')
    map_topic = rospy.get_param('~map_topic', '/v8_support_map')
    click_topic = rospy.get_param('~click_topic', '/clicked_point')
    goal_topic = rospy.get_param('~goal_topic', '/v8/3d/global/goal')
    status_topic = rospy.get_param('~status_topic', '/v8/3d/surface_pick/status')
    hint = rospy.get_param('~surface_hint', None)
    frame = rospy.get_param('~frame', 'map')
    state = {'map': None}
    goal_pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=1, latch=True)
    status_pub = rospy.Publisher(status_topic, String, queue_size=10, latch=True)

    def publish_status(payload):
        status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def on_map(message):
        try:
            state['map'] = json.loads(message.data)
        except ValueError:
            publish_status({'ok': False, 'reason': 'E_PICK_MAP_JSON'})

    def on_click(message):
        if state['map'] is None:
            publish_status({'ok': False, 'reason': 'E_PICK_MAP_NOT_READY'})
            return
        if message.header.frame_id not in ('', frame):
            publish_status({'ok': False, 'reason': 'E_PICK_FRAME',
                            'frame': message.header.frame_id})
            return
        p = message.point
        try:
            picked = surface_resolver.pick_goal(state['map'], (p.x, p.y, p.z), hint)
        except surface_resolver.SurfaceResolutionError as error:
            publish_status({'ok': False, 'reason': error.code, 'detail': error.detail,
                            'candidates': error.candidates, 'click': [p.x, p.y, p.z]})
            return
        goal = PoseStamped()
        goal.header.frame_id, goal.header.stamp = frame, rospy.Time.now()
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = (
            picked['x'], picked['y'], picked['z'])
        goal.pose.orientation.w = 1.
        goal_pub.publish(goal)
        publish_status({'ok': True, 'goal': [picked['x'], picked['y'], picked['z']],
                        'surface_id': picked['surface_id'],
                        'z_reference': picked['z']})

    rospy.Subscriber(map_topic, String, on_map, queue_size=1)
    rospy.Subscriber(click_topic, PointStamped, on_click, queue_size=2)
    publish_status({'ok': True, 'ready': True, 'hint': hint})
    print('READY', flush=True)
    rospy.spin()


if __name__ == '__main__':
    main()
