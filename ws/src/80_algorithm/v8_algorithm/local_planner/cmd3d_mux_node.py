#!/usr/bin/env python3
"""3D command arbiter: one map-frame TwistStamped advice -> sole /cmd_vel owner.

The local planner publishes a map-frame 3D velocity.  This node validates frame,
freshness and hold, then publishes a Twist whose linear xyz is still map-frame
velocity for the 3D kinematic carrier.  It is the only /cmd_vel writer.
"""
import json
import math
import threading
import time


def arbitrate(velocity, frame, received, hold, now, timeout_s=.4):
    """Return (linear_xyz_wz, reason); pure helper for tests."""
    zero = (0., 0., 0., 0.)
    if hold:
        return zero, 'hold'
    if received is None or now - received > timeout_s:
        return zero, 'E_CMD3D_STALE'
    if frame != 'map':
        return zero, 'E_CMD3D_FRAME'
    if velocity is None or len(velocity) != 3 or not all(
            isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
            for v in velocity):
        return zero, 'E_CMD3D_INVALID'
    return (float(velocity[0]), float(velocity[1]), float(velocity[2]), 0.), 'track'


def main():
    import rospy
    from geometry_msgs.msg import Twist, TwistStamped
    from std_msgs.msg import Bool, String

    rospy.init_node('cmd3d_mux', anonymous=False, disable_signals=True)
    cmd_topic = rospy.get_param('~cmd_topic', '/v8/local/cmd_vel_3d')
    hold_topic = rospy.get_param('~hold_topic', '/lite3/supervisor/hold')
    output_topic = rospy.get_param('~output_topic', '/cmd_vel')
    status_topic = rospy.get_param('~status_topic', '/v8/3d/mux/status')
    timeout_s = float(rospy.get_param('~timeout_s', .4))
    lock = threading.Lock()
    state = {'velocity': None, 'frame': None, 'received': None, 'hold': False}
    output = rospy.Publisher(output_topic, Twist, queue_size=20)
    status = rospy.Publisher(status_topic, String, queue_size=20)

    def on_cmd(message):
        with lock:
            state['velocity'] = [message.twist.linear.x, message.twist.linear.y,
                                 message.twist.linear.z]
            state['frame'] = message.header.frame_id
            state['received'] = time.monotonic()

    def on_hold(message):
        with lock:
            state['hold'] = bool(message.data)

    rospy.Subscriber(cmd_topic, TwistStamped, on_cmd, queue_size=10)
    rospy.Subscriber(hold_topic, Bool, on_hold, queue_size=10)
    try:
        while not rospy.is_shutdown():
            with lock:
                snapshot = dict(state)
            values, reason = arbitrate(snapshot['velocity'], snapshot['frame'],
                                       snapshot['received'], snapshot['hold'],
                                       time.monotonic(), timeout_s)
            value = Twist()
            value.linear.x, value.linear.y, value.linear.z, value.angular.z = values
            output.publish(value)
            age = None if snapshot['received'] is None else time.monotonic() - snapshot['received']
            status.publish(String(data=json.dumps({
                'reason': reason, 'velocity': list(values), 'input_frame': snapshot['frame'],
                'age_s': age, 'hold': snapshot['hold']})))
            time.sleep(.02)
    finally:
        output.publish(Twist())


if __name__ == '__main__':
    main()
