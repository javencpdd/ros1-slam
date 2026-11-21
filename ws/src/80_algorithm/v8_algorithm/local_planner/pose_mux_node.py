#!/usr/bin/env python3
"""ROS pose multiplexer: publish the active localization pose, reject jumps."""
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import pose_mux  # noqa: E402


def main():
    import rospy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String

    rospy.init_node('v8_pose_mux')
    max_gap_m = float(rospy.get_param('~max_gap_m', 0.1))
    max_age_s = float(rospy.get_param('~max_age_s', 0.5))
    rate_hz = float(rospy.get_param('~rate_hz', 20.0))
    mux = pose_mux.PoseMux(max_gap_m=max_gap_m, max_age_s=max_age_s)
    output_topic = rospy.get_param('~output_topic', '/v8/local/pose')
    status_topic = rospy.get_param('~status_topic', '/v8/local/pose_mux/status')
    publisher = rospy.Publisher(output_topic, PoseStamped, queue_size=1, latch=True)
    status_pub = rospy.Publisher(status_topic, String, queue_size=10, latch=True)
    last_status = {'value': None}

    def status(value):
        text = json.dumps(value, ensure_ascii=False)
        if text != last_status['value']:
            last_status['value'] = text
            status_pub.publish(String(data=text))

    def on_pose(source):
        def callback(message):
            if message.header.frame_id != 'map':
                status({'ok': False, 'reason': 'E_POSE_MUX_FRAME', 'source': source})
                return
            p = message.pose.position
            mux.update(source, (p.x, p.y, p.z), time.monotonic())
        return callback

    def on_select(message):
        result = mux.select(message.data.strip().lower(), time.monotonic())
        status(dict(result, action='select'))

    rospy.Subscriber(rospy.get_param('~pose_a_topic', '/v8/local/pose_in_a'),
                     PoseStamped, on_pose('a'), queue_size=5)
    rospy.Subscriber(rospy.get_param('~pose_b_topic', '/v8/local/pose_in_b'),
                     PoseStamped, on_pose('b'), queue_size=5)
    rospy.Subscriber(rospy.get_param('~select_topic', '/v8/local/pose_select'),
                     String, on_select, queue_size=5)
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        result = mux.output(time.monotonic())
        if result['ok']:
            x, y, z = result['pose']
            message = PoseStamped()
            message.header.frame_id, message.header.stamp = 'map', rospy.Time.now()
            message.pose.position.x, message.pose.position.y, message.pose.position.z = x, y, z
            message.pose.orientation.w = 1.
            publisher.publish(message)
            status({'ok': True, 'source': result['source'], 'pose': list(result['pose'])})
        else:
            status(result)
        rate.sleep()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
