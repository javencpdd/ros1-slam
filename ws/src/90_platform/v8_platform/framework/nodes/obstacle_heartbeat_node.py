#!/usr/bin/env python3
"""Empty (or configured) dynamic-obstacle heartbeat for the 3D local planner.

The local planner is fail-closed: it refuses to move until it has seen an obstacle
message. Static obstacles live in the support map; this node only supplies the
dynamic layer, which is empty until a real perception source is wired.
"""
import os as _os
import sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
import json

import rospy
from std_msgs.msg import String


def main():
    rospy.init_node('v8_obstacle_heartbeat')
    topic = rospy.get_param('~obstacle_topic', '/v8/local/obstacles')
    rate_hz = float(rospy.get_param('~rate_hz', 1.0))
    payload = rospy.get_param('~payload', [])
    publisher = rospy.Publisher(topic, String, queue_size=1)
    body = json.dumps(payload, separators=(',', ':'))
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        publisher.publish(String(data=body))
        rate.sleep()


if __name__ == '__main__':
    main()
