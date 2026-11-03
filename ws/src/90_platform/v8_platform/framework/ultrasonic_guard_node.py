#!/usr/bin/env python3
"""Front ultrasonic speed cap -> /v8/safety/speed_cap (Float32).

The bridge publishes both ranges on /us_publisher/ultrasound
(Float64MultiArray, index 0 = front). This node turns the front range into a
speed cap the safety mux applies; it never publishes a velocity itself.
"""
import json

import rospy
from std_msgs.msg import Float32, Float64MultiArray, String

from ultrasonic_guard import SpeedCapDecision


class UltrasonicGuardNode(object):
    def __init__(self):
        self.topic = rospy.get_param('~us_topic', '/us_publisher/ultrasound')
        self.front_index = int(rospy.get_param('~front_index', 0))
        self.rate_hz = float(rospy.get_param('~rate_hz', 10.0))
        self.stale_s = float(rospy.get_param('~stale_s', 0.5))
        self.enable = bool(rospy.get_param('~enable', True))
        self.cap_topic = rospy.get_param('~cap_topic', '/v8/safety/speed_cap')
        self.status_topic = rospy.get_param('~status_topic', '/v8/safety/ultrasonic_status')
        self.decision = SpeedCapDecision(
            cap_distance_m=float(rospy.get_param('~cap_distance_m', 0.8)),
            clear_distance_m=float(rospy.get_param('~clear_distance_m', 1.2)),
            cap_speed_mps=float(rospy.get_param('~cap_speed_mps', 0.1)),
            stop_distance_m=float(rospy.get_param('~stop_distance_m', 0.3)),
            stop_speed_mps=float(rospy.get_param('~stop_speed_mps', 0.0)),
            clear_speed_mps=float(rospy.get_param('~clear_speed_mps', 0.5)),
            fail_cap=bool(rospy.get_param('~fail_cap', False)))

        self.front = None
        self.stamp = None
        self.cap_pub = rospy.Publisher(self.cap_topic, Float32, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1)
        rospy.Subscriber(self.topic, Float64MultiArray, self.on_ranges, queue_size=1)

    def on_ranges(self, msg):
        if len(msg.data) > self.front_index:
            self.front = float(msg.data[self.front_index])
            self.stamp = rospy.get_time()

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            now = rospy.get_time()
            fresh = self.stamp is not None and (now - self.stamp) <= self.stale_s
            cap = (self.decision.update(self.front, fresh) if self.enable
                   else self.decision.clear_speed_mps)
            self.cap_pub.publish(Float32(data=cap))
            self.status_pub.publish(String(data=json.dumps({
                'enabled': self.enable, 'front_m': self.front,
                'fresh': fresh, 'cap_mps': cap, 'reason': self.decision.reason})))
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('v8_ultrasonic_guard')
    UltrasonicGuardNode().spin()
