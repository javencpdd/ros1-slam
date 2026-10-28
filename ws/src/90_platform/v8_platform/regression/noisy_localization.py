#!/usr/bin/env python3
"""Publish the seeded estimated-localization track as a dynamic map->odom.

Starts only in the estimated-localization experiment; the Gazebo launch must be
run with map_to_odom:=false so this is the single publisher of that edge.
Not LIO/NDT, not hardware; evaluation still uses /sim/odom_gt truth.
"""
import argparse
import math
import time

import rospy
import tf
from estimated_track import offset


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--rate-hz', type=float, default=50.0)
    parser.add_argument('--lookahead-s', type=float, default=0.06)
    parser.add_argument('--start-timeout-s', type=float, default=60.0)
    args, _ = parser.parse_known_args(argv)
    if args.rate_hz <= 0 or args.start_timeout_s <= 0 or args.lookahead_s <= 0:
        raise SystemExit('rate, lookahead and start timeout must be positive')
    rospy.init_node('a1_estimated_localization', anonymous=True)
    if not rospy.get_param('use_sim_time', False):
        raise SystemExit('estimated-localization track requires use_sim_time')
    deadline = time.monotonic() + args.start_timeout_s
    while rospy.Time.now().to_sec() <= 0 and not rospy.is_shutdown():
        if time.monotonic() > deadline:
            raise SystemExit('no nonzero /clock before start timeout')
        time.sleep(0.05)
    broadcaster = tf.TransformBroadcaster()
    rate = rospy.Rate(args.rate_hz)
    started = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        # Stamp one lookahead period ahead: a moving map->odom that only covers
        # the past makes every controller lookup at "now" extrapolate forward.
        stamp = now + rospy.Duration.from_sec(args.lookahead_s)
        x, y, yaw = offset(max(0.0, now.to_sec() - started), args.seed)
        broadcaster.sendTransform(
            (x, y, 0.0),
            tf.transformations.quaternion_from_euler(0.0, 0.0, yaw),
            stamp, 'odom', 'map')
        rate.sleep()


if __name__ == '__main__':
    main()
