#!/usr/bin/env python3
"""Publish the last pose of a 3D global Path as the local planner goal."""
import json

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String


def last_pose(message):
    if not message.poses:
        return None
    pose = message.poses[-1]
    goal = PoseStamped()
    goal.header = pose.header
    goal.pose = pose.pose
    return goal


def main():
    rospy.init_node('path_goal3d', anonymous=False, disable_signals=True)
    path_topic = rospy.get_param('~path_topic', '/v8/3d/global/path')
    goal_topic = rospy.get_param('~goal_topic', '/v8/local/goal')
    status_topic = rospy.get_param('~status_topic', '/v8/3d/path_goal/status')
    goal_pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=10)
    status_pub = rospy.Publisher(status_topic, String, queue_size=10)

    def on_path(message):
        goal = last_pose(message)
        if goal is None:
            status_pub.publish(String(data=json.dumps({'ok': False, 'reason': 'empty_path'})))
            return
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = goal.header.frame_id or 'map'
        goal_pub.publish(goal)
        status_pub.publish(String(data=json.dumps({
            'ok': True, 'poses': len(message.poses),
            'goal': [goal.pose.position.x, goal.pose.position.y, goal.pose.position.z],
            'frame': goal.header.frame_id})))

    rospy.Subscriber(path_topic, Path, on_path, queue_size=10)
    rospy.spin()


if __name__ == '__main__':
    main()
