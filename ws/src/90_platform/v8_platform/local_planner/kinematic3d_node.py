#!/usr/bin/env python3
"""3D kinematic carrier: integrate map-frame /cmd_vel into a pose for A5 tests.

No physics/contact.  It is a deterministic test carrier that lets the 3D
global/local graph close through Assembly; it is not a robot or Gazebo model.
"""
import json
import math
import threading
import time


def integrate(position, yaw, velocity, dt):
    """Return the next (x, y, z, yaw) for map-frame velocity and yaw rate."""
    x, y, z = position
    vx, vy, vz, wz = velocity
    return (x + vx * dt, y + vy * dt, z + vz * dt, yaw + wz * dt)


def main():
    import rospy
    from geometry_msgs.msg import PoseStamped, Twist
    from std_msgs.msg import Bool, String

    rospy.init_node('kinematic3d_carrier', anonymous=False, disable_signals=True)
    start = [float(rospy.get_param('~start_x', 3.)),
             float(rospy.get_param('~start_y', 2.)),
             float(rospy.get_param('~start_z', .6))]
    yaw = float(rospy.get_param('~yaw', 0.))
    rate = float(rospy.get_param('~rate_hz', 50.))
    timeout_s = float(rospy.get_param('~timeout_s', .4))
    cmd_topic = rospy.get_param('~cmd_topic', '/cmd_vel')
    pose_topic = rospy.get_param('~pose_topic', '/v8/local/pose')
    localization_topic = rospy.get_param('~localization_topic', '/v8/local/localization_ok')
    status_topic = rospy.get_param('~status_topic', '/v8/3d/carrier/status')
    lock = threading.Lock()
    received = {'at': None, 'velocity': (0., 0., 0., 0.)}
    pose_pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=20)
    localization_pub = rospy.Publisher(localization_topic, Bool, queue_size=20)
    status_pub = rospy.Publisher(status_topic, String, queue_size=20)

    def on_cmd(message):
        with lock:
            received['velocity'] = (message.linear.x, message.linear.y,
                                    message.linear.z, message.angular.z)
            received['at'] = time.monotonic()

    rospy.Subscriber(cmd_topic, Twist, on_cmd, queue_size=20)
    position, last = start[:], time.monotonic()
    try:
        while not rospy.is_shutdown():
            now = time.monotonic()
            dt = min(.05, max(0., now - last))
            last = now
            with lock:
                at = received['at']
                velocity = received['velocity']
            stale = at is None or now - at > timeout_s
            applied = (0., 0., 0., 0.) if stale else velocity
            if not all(math.isfinite(v) for v in applied):
                applied = (0., 0., 0., 0.)
            position = list(integrate(position, yaw, applied, dt)[:3])
            yaw += applied[3] * dt
            message = PoseStamped()
            message.header.frame_id = 'map'
            message.header.stamp = rospy.Time.now()
            message.pose.position.x, message.pose.position.y, message.pose.position.z = position
            message.pose.orientation.z = math.sin(yaw / 2.)
            message.pose.orientation.w = math.cos(yaw / 2.)
            pose_pub.publish(message)
            localization_pub.publish(Bool(data=True))
            status_pub.publish(String(data=json.dumps({
                'xyz': position, 'yaw': yaw, 'velocity': list(applied),
                'stale': stale, 'mode': 'kinematic3d'})))
            time.sleep(1. / rate)
    finally:
        pose_pub.unpublish()


if __name__ == '__main__':
    main()
