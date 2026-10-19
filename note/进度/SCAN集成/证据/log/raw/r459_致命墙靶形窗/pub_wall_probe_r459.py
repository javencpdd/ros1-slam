#!/usr/bin/env python3
# pub_wall_probe_r459.py —— ★强靶形★：给第二枚层喂一张★横贯全图的致命墙★（R458 的 C 态＝起点半径 0.40m 圆盘 ⇒ 13 枚零变化；本件把靶做强到★几何上必然切断左右连通★，用来分掉「靶太弱」与「结构上看不见」两种解释）
# 形＝沿用 R442／R445／R446 的探针发布器姿势＝★发布前自报行逐字打★（unknown／lethal／free／size／origin／robot）
# 内容＝与 /map 同尺寸同分辨率同原点同坐标系；★全自由 0★；★只在雷达载体当前 base_link 位姿半径 0.40m 的圆内涂致命 100★
#        ⇒ 该层以 use_maximum: true 叠加 ⇒ 全局代价图里"起点所在格"变致命 ⇒ 规划行为应改变（改没改由对照件报数、本件不判）
# ★不改动任何在册件★；★话题名 /elev_map 与副本 yaml 里 elevation_layer/map_topic 逐字一致（两侧同式互校）★
import math
import signal
import sys
import time

import rospy
import tf
from nav_msgs.msg import OccupancyGrid

BAND = (0.25, 0.85)   # 相对当前 base_link 的 x 偏移带（米）＝一条厚 0.6m、纵向横贯全图的墙
LATCH_S = float(sys.argv[1]) if len(sys.argv) > 1 else 170.0


def bye(sig, frm):
    rospy.signal_shutdown("SIGINT from window")


def main():
    signal.signal(signal.SIGINT, bye)
    rospy.init_node("wall_probe_r459")
    pub = rospy.Publisher("/elev_map", OccupancyGrid, queue_size=1, latch=True)
    rospy.loginfo("探针起动｜等 /map（≤60s）")
    try:
        base = rospy.wait_for_message("/map", OccupancyGrid, timeout=60)
    except rospy.ROSException:
        print("SELFREPORT FAIL: /map 60s 未到 ⇒ 本窗 C 态判 INVALID")
        sys.exit(3)
    listener = tf.TransformListener()
    rx = ry = None
    t0 = time.time()
    while not rospy.is_shutdown() and rx is None and time.time() - t0 < 20:
        try:
            tr, _ = listener.lookupTransform("map", "base_link", rospy.Time(0))
            rx, ry = tr[0], tr[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.sleep(0.2)
    if rx is None:
        print("SELFREPORT FAIL: TF map→base_link 20s 不可查 ⇒ 本窗 C 态判 INVALID")
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
            wy = oy + (r + 0.5) * res
            if rx + BAND[0] <= wx <= rx + BAND[1]:
                data[r * w + c] = 100
                lethal += 1
    m.data = data
    pub.publish(m)
    print("SELFREPORT topic=/elev_map size=%dx%d res=%.3f frame=%s origin=(%.2f,%.2f) robot=(%.2f,%.2f) band=(%.2f,%.2f) free=%d lethal=%d unknown=0 total=%d" % (
        w, h, res, m.header.frame_id, ox, oy, rx, ry, rx + BAND[0], rx + BAND[1], w * h - lethal, lethal, w * h))
    sys.stdout.flush()
    t1 = time.time()
    while not rospy.is_shutdown() and time.time() - t1 < LATCH_S:
        m.header.stamp = rospy.Time.now()
        pub.publish(m)
        rospy.sleep(0.5)
    print("SELFREPORT 退出（latch 保活 %.0fs 到点或被 SIGINT）" % LATCH_S)
    sys.stdout.flush()


main()
