#!/usr/bin/env python3
"""Bridge RViz 2D Nav Goal (/move_base_simple/goal) to the MBF action server.

MBF (mbf_costmap_nav) exposes /move_base_flex/move_base, not the legacy
move_base_simple topic. This node lets RViz's 2D Nav Goal drive the MBF stack.
"""
import argparse

import actionlib
import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal-topic', default='/move_base_simple/goal')
    parser.add_argument('--action', default='/move_base_flex/move_base')
    parser.add_argument('--cancel-topic', default='/move_base/cancel')
    parser.add_argument('--server-wait-s', type=float, default=1.0)
    args = parser.parse_args(rospy.myargv(argv=argv)[1:])
    if args.server_wait_s <= 0:
        raise SystemExit('server-wait-s must be positive')

    rospy.init_node('mbf_goal_bridge', anonymous=True)
    client = actionlib.SimpleActionClient(args.action, MoveBaseAction)
    state = {'goals': 0, 'cancels': 0}

    def on_goal(message):
        if message.header.frame_id not in ('', 'map'):
            rospy.logwarn('mbf goal bridge ignored frame=%s', message.header.frame_id)
            return
        if not message.header.frame_id:
            message.header.frame_id = 'map'
        if not client.wait_for_server(rospy.Duration(args.server_wait_s)):
            rospy.logwarn('MBF action server unavailable: %s', args.action)
            return
        goal = MoveBaseGoal()
        goal.target_pose = message
        client.send_goal(goal)
        state['goals'] += 1
        rospy.loginfo('MBF goal %d sent: %.2f, %.2f', state['goals'],
                      message.pose.position.x, message.pose.position.y)

    def on_cancel(_message):
        client.cancel_all_goals()
        state['cancels'] += 1
        rospy.loginfo('MBF goal cancel %d', state['cancels'])

    rospy.Subscriber(args.goal_topic, PoseStamped, on_goal, queue_size=5)
    rospy.Subscriber(args.cancel_topic, GoalID, on_cancel, queue_size=5)
    rospy.loginfo('mbf goal bridge %s -> %s', args.goal_topic, args.action)
    rospy.spin()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
