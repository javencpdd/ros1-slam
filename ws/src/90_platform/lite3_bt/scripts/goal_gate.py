#!/usr/bin/env python3
"""ROS-level 3D goal gate for the console BT demo.

The BT node publishes /v8/bt/goal_request and a lease heartbeat on
/v8/bt/lease.  This node is the only intended writer of
/v8/3d/global/goal and forwards a request only while the lease is fresh.
A stale or missing lease drops the request instead of leaving an old goal
live.  This is a demo gate; formal/real navigation stays on the work order.
"""
from __future__ import annotations

import time

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class GoalGate:
    def __init__(self):
        self.request_topic = rospy.get_param("~request_topic", "/v8/bt/goal_request")
        self.lease_topic = rospy.get_param("~lease_topic", "/v8/bt/lease")
        self.output_topic = rospy.get_param("~output_topic", "/v8/3d/global/goal")
        self.lease_timeout = float(rospy.get_param("~lease_timeout", 1.0))
        self.require_lease = bool(rospy.get_param("~require_lease", True))
        self.lease = False
        self.lease_at = 0.0
        self.last_drop_log = 0.0
        self.output = rospy.Publisher(self.output_topic, PoseStamped, queue_size=1)
        rospy.Subscriber(self.request_topic, PoseStamped, self.on_request, queue_size=1)
        rospy.Subscriber(self.lease_topic, Bool, self.on_lease, queue_size=1)
        rospy.loginfo("[goal-gate] %s -> %s (lease=%s, timeout=%.1fs)",
                      self.request_topic, self.output_topic,
                      self.lease_topic, self.lease_timeout)

    def on_lease(self, msg):
        self.lease = bool(msg.data)
        self.lease_at = time.monotonic()

    def on_request(self, msg):
        fresh = (time.monotonic() - self.lease_at) <= self.lease_timeout
        if self.require_lease and not (self.lease and fresh):
            now = time.monotonic()
            if now - self.last_drop_log > 2.0:
                rospy.logwarn("[goal-gate] request dropped: lease=%s fresh=%s",
                              self.lease, fresh)
                self.last_drop_log = now
            return
        self.output.publish(msg)
        rospy.loginfo("[goal-gate] forwarded 3D goal (%.2f, %.2f, %.2f)",
                      msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def spin(self):
        rospy.spin()


def main():
    rospy.init_node("v8_goal_gate")
    GoalGate().spin()


if __name__ == "__main__":
    main()
