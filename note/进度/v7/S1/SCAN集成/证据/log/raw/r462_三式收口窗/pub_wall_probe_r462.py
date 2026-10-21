#!/usr/bin/env python3
# pub_wall_probe_r462.py —— ★C7-B 的靶★：给第二枚 StaticLayer（elevation_layer）喂一条★致命墙★
#   ★靶功率下限 ≥1200 致命格（r459q 新立）★ ⇒ 带厚 = 1.10 m（11 列）× 全图纵向 120 行 ＝★理论 1320 格★；R459 那发是 0.60m＝720 格、★低于下限★
#   内容＝与 /map 同尺寸同分辨率同原点同坐标系；其余全自由 0；带内＝致命 100（该层 use_maximum: true ⇒ 只升不降）
#   ★自报行逐字（unknown=／lethal=／free=／size=／origin=／band=）＝靶生效的第一证★
import signal
import sys

import rospy
import tf
from nav_msgs.msg import OccupancyGrid

BAND = (0.25, 1.35)
LATCH_S = float(sys.argv[1]) if len(sys.argv) > 1 else 170.0


def bye(sig, frm):
    rospy.signal_shutdown("SIGINT from window")


def main():
    signal.signal(signal.SIGINT, bye)
    rospy.init_node("wall_probe_r462")
    pub = rospy.Publisher("/elev_map", OccupancyGrid, queue_size=1, latch=True)
    try:
        base = rospy.wait_for_message("/map", OccupancyGrid, timeout=60)
    except rospy.ROSException:
        print("SELFREPORT FAIL: /map 60s 未到 ⇒ C 态 INVALID")
        sys.exit(3)
    listener = tf.TransformListener()
    rx = ry = None
    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and rx is None and (rospy.Time.now() - t0).to_sec() < 20:
        try:
            tr, rot = listener.lookupTransform("map", "base_link", rospy.Time(0))
            rx, ry = tr[0], tr[1]     # ★lookupTransform 返回 (trans, rot)、trans 本身就是三元表（R458 的 TypeError 就在这）★
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.sleep(0.2)
    if rx is None:
        print("SELFREPORT FAIL: TF map→base_link 20s 不可查 ⇒ C 态 INVALID")
        sys.exit(3)
    m = OccupancyGrid()
    m.header.frame_id = base.header.frame_id
    m.header.stamp = rospy.Time.now()
    m.info = base.info
    w, h, res = int(base.info.width), int(base.info.height), float(base.info.resolution)
    ox, oy = float(base.info.origin.position.x), float(base.info.origin.position.y)
    data = [0] * (w * h)
    lethal = 0
    for r in range(h):
        for c in range(w):
            wx = ox + (c + 0.5) * res
            if rx + BAND[0] <= wx <= rx + BAND[1]:
                data[r * w + c] = 100
                lethal += 1
    m.data = data
    pub.publish(m)
    print("SELFREPORT topic=/elev_map size=%dx%d res=%.3f frame=%s origin=(%.2f,%.2f) robot=(%.2f,%.2f) band=(%.2f,%.2f) free=%d lethal=%d unknown=0 total=%d 靶功率下限=1200 ⇒ %s" % (
        w, h, res, m.header.frame_id, ox, oy, rx, ry, rx + BAND[0], rx + BAND[1], w * h - lethal, lethal, w * h,
        "达标" if lethal >= 1200 else "★不达标★"))
    sys.stdout.flush()
    t1 = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - t1).to_sec() < LATCH_S:
        m.header.stamp = rospy.Time.now()
        pub.publish(m)
        rospy.sleep(0.5)
    print("SELFREPORT 退出（latch 保活到点或被 SIGINT）")
    sys.stdout.flush()


main()
