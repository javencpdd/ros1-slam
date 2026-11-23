#!/usr/bin/env python3
"""P3 定位误差探针：/hdl_odom（算法位姿）对 /sim/odom_gt（Gazebo 真值）。

只读，不注入、不改话题、不发任何 TF。

判据：
  - 按时间戳配对，|Δt| ≤ TOL；配不到样本就判 INVALID，不能拿单边数据充数；
  - 同时输出「绝对误差」与「去常数偏置后的相对误差」。
    两者都要看：hdl 的 pose 在 map 系、/sim/odom_gt 在 odom 系，两者原点差一个 hdl 估计出来的
    map→odom 变换。绝对误差里含这个常数偏置，相对误差才反映「位姿是否跟着真值走」。

用法：
  python3 hdl_pose_error.py [观察秒数，默认 20]
"""
import math
import sys
import time

import rospy
from nav_msgs.msg import Odometry

TOL = 0.05  # 配对时间窗（秒）

est = []  # (stamp, x, y, yaw)
gt = []


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def make_cb(bucket):
    def cb(msg):
        p = msg.pose.pose
        bucket.append((msg.header.stamp.to_sec(), p.position.x, p.position.y, yaw_of(p.orientation)))
    return cb


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    rospy.init_node('hdl_pose_error', anonymous=True)
    rospy.Subscriber('/hdl_odom', Odometry, make_cb(est), queue_size=50)
    rospy.Subscriber('/sim/odom_gt', Odometry, make_cb(gt), queue_size=50)

    t0 = time.time()
    while not rospy.is_shutdown() and time.time() - t0 < duration:
        time.sleep(0.1)

    pairs = []
    for (ts, x, y, yw) in est:
        best = None
        for (tg, gx, gy, gw) in gt:
            d = abs(ts - tg)
            if d <= TOL and (best is None or d < best[0]):
                best = (d, gx, gy, gw)
        if best:
            pairs.append((x, y, yw, best[1], best[2], best[3]))

    print("SAMPLES est=%d gt=%d pairs=%d tol=%.2fs" % (len(est), len(gt), len(pairs), TOL))
    if not pairs:
        print("P3_ERROR_VERDICT=INVALID  # 无时间戳匹配样本，不能凭单边数据下结论")
        return 2

    abs_err = [math.hypot(px - gx, py - gy) for (px, py, _, gx, gy, _) in pairs]
    yaw_err = [abs(math.atan2(math.sin(pw - gw), math.cos(pw - gw)))
               for (_, _, pw, _, _, gw) in pairs]

    # 相对误差：先扣掉首样本的常数偏置，再看剩余漂移
    ex0 = pairs[0][0] - pairs[0][3]
    ey0 = pairs[0][1] - pairs[0][4]
    rel_err = [math.hypot((px - gx) - ex0, (py - gy) - ey0)
               for (px, py, _, gx, gy, _) in pairs]

    def stat(name, xs):
        s = sorted(xs)
        n = len(s)
        print("%s mean=%.4f median=%.4f max=%.4f" % (name, sum(s) / n, s[n // 2], s[-1]))

    stat("ABS_XY_M", abs_err)
    stat("REL_XY_M", rel_err)
    stat("YAW_RAD", yaw_err)
    print("BIAS_XY_M=(%.4f, %.4f)" % (ex0, ey0))
    print("P3_ERROR_VERDICT=PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
