#!/usr/bin/env python3
"""sim_speed_cap.py — 仿真链的限速输入（mux speed_cap_required 的最小占位源）。

mux 的 speed_cap_required=true 且无 /v8/safety/speed_cap 时按 blind 0.1 m/s
限速；仿真链没有超声/限速源，用本节点显式发 0.5（mux max_linear 也是 0.5）。
真机不启动本节点（真机限速由超声 guard 提供）。
"""
import rospy
from std_msgs.msg import Float32

rospy.init_node('sim_speed_cap')
topic = rospy.get_param('~topic', '/v8/safety/speed_cap')
value = float(rospy.get_param('~value', 0.5))
rate = rospy.Rate(float(rospy.get_param('~rate', 2.0)))
pub = rospy.Publisher(topic, Float32, queue_size=1)
while not rospy.is_shutdown():
    pub.publish(Float32(data=value))
    rate.sleep()
