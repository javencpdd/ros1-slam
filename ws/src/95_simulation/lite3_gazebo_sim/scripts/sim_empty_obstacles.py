#!/usr/bin/env python3
"""Publish a fresh, empty obstacle prediction for the sim local planner.

`local_planner/ros_node.py` refuses to command motion while obstacles are stale
(`E_LOCAL_OBSTACLES_STALE`) — it needs a periodic heartbeat, not a one-shot.
The Gazebo stack has no perception producer for `/v8/local/obstacles`, so this
node publishes an explicit "no obstacles" prediction at a fixed rate.

It is a sim-side fixture, not a perception substitute: the payload is
configurable so a scenario can inject real boxes later (`~payload`), and it
writes the same contract topic a real detector would.
"""
import json

import rospy
from std_msgs.msg import String


def main():
    rospy.init_node('sim_empty_obstacles')
    topic = rospy.get_param('~topic', '/v8/local/obstacles')
    rate = float(rospy.get_param('~rate', 2.0))
    payload_text = rospy.get_param('~payload', '')
    if payload_text:
        payload = json.loads(payload_text)
    else:
        payload = {'obstacles': []}
    data = json.dumps(payload, separators=(',', ':'))
    pub = rospy.Publisher(topic, String, queue_size=1, latch=True)
    rospy.Timer(rospy.Duration(1.0 / rate), lambda _event: pub.publish(String(data=data)))
    rospy.loginfo('[sim_empty_obstacles] %s @ %.1f Hz payload=%s', topic, rate, data)
    rospy.spin()


if __name__ == '__main__':
    main()
