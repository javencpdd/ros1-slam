#!/usr/bin/env python3
"""ROS map-velocity kinematic carrier for the #151 negative control.

No dynamics, no braking: map velocity is integrated at 50 Hz.  When the local
node refuses/latches, the next command is zero and the pose stops immediately.
This is the same-batch control that separates the braking segment from link
latency; it is not a robot or a validated carrier.
"""
import argparse
import json
import time
from pathlib import Path

import a5_v8x1_bridge as a5


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True)
    parser.add_argument('--prefix', required=True)
    parser.add_argument('--start-map-y', type=float, default=-1.9)
    parser.add_argument('--x-offset', type=float, default=0.)
    parser.add_argument('--y-offset', type=float, default=-2.7)
    parser.add_argument('--radius', type=float, default=.15)
    parser.add_argument('--cmd-timeout', type=float, default=.5)
    args = parser.parse_args()
    value = json.loads(Path(args.map).read_text())
    import rospy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from std_msgs.msg import Bool, String
    rospy.init_node('a5_kinematic_stop_bridge', anonymous=True, disable_signals=True)
    shared = {'velocity': (0., 0., 0.), 'at': -1e9}
    lock = __import__('threading').Lock()

    def on_cmd(message):
        with lock:
            shared['velocity'] = (message.twist.linear.x, message.twist.linear.y,
                                  message.twist.linear.z)
            shared['at'] = time.monotonic()

    rospy.Subscriber(args.prefix + '/cmd', TwistStamped, on_cmd, queue_size=10)
    pose_pub = rospy.Publisher(args.prefix + '/pose', PoseStamped, queue_size=20)
    body_pub = rospy.Publisher(args.prefix + '/body', PoseStamped, queue_size=50)
    applied_pub = rospy.Publisher(args.prefix + '/applied', String, queue_size=20)
    loc_pub = rospy.Publisher(args.prefix + '/localization', Bool, queue_size=1)
    map_x, map_y, sim = 0., args.start_map_y, 0.
    next_tick = time.monotonic()
    tick = 0
    while not rospy.is_shutdown():
        now = time.monotonic()
        with lock:
            velocity = shared['velocity'] if now - shared['at'] <= args.cmd_timeout else (0., 0., 0.)
        dt = .02
        map_x += velocity[0] * dt
        map_y += velocity[1] * dt
        sim += dt
        support = a5.support_z(value, map_x, map_y, args.radius)
        if support is None:
            support = 0.
        world_x, world_y = a5.map_to_world(map_x, map_y, args.x_offset, args.y_offset)
        world_vx, world_vy = a5.map_velocity_to_world(velocity[0], velocity[1])
        stamp = rospy.Time.now()
        pose = PoseStamped()
        pose.header.frame_id, pose.header.stamp = 'map', stamp
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map_x, map_y, support
        pose.pose.orientation.w = 1.
        pose_pub.publish(pose)
        body = PoseStamped()
        body.header.frame_id, body.header.stamp = 'world', stamp
        body.pose.position.x, body.pose.position.y, body.pose.position.z = world_x, world_y, support + .35
        body.pose.orientation.w = 1.
        body_pub.publish(body)
        if tick % 5 == 0:
            applied_pub.publish(String(data=json.dumps({
                'map_velocity': list(velocity), 'body_command': list(velocity),
                'support_z': support, 'world_xyz': [world_x, world_y, support + .35],
                'base_vxy': [world_vx, world_vy], 'map_xyz': [map_x, map_y, support],
                'sim_time': sim, 'kinematic_control': True,
                'scenario': 'a5_kinematic_stop_negative_control'})))
        loc_pub.publish(Bool(data=True))
        tick += 1
        next_tick += .02
        delay = next_tick - time.monotonic()
        if delay > 0:
            time.sleep(delay)


if __name__ == '__main__':
    main()
