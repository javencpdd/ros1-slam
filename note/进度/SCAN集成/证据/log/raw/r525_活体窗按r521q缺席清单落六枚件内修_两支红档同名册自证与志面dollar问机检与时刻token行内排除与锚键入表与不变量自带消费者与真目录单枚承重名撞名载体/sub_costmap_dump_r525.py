#!/usr/bin/env python3
# sub_costmap_dump_r525.py —— ★内容栏取样器★：抓一帧 global costmap 全格＋几何落 CSV
#   自报行★按 R465q 的钉★：凡发布侧计数★必带谓词号★＝P90（v>=90≡{99,100}）‖ P100（v==100），另报 distinct 值域与 1..98 格数
# 用法：sub_costmap_dump_r525.py <输出csv> [超时秒]
import sys

import rospy
from nav_msgs.msg import OccupancyGrid

OUT = sys.argv[1]
TMO = float(sys.argv[2]) if len(sys.argv) > 2 else 80.0
box = {}


def cb(m):
    if "m" not in box:
        box["m"] = m


def main():
    rospy.init_node("sub_costmap_dump_r525", anonymous=True)
    rospy.Subscriber("/move_base_flex/global_costmap/costmap", OccupancyGrid, cb, queue_size=1)
    t0 = __import__("time").time()
    while "m" not in box and __import__("time").time() - t0 < TMO:
        __import__("time").sleep(0.2)
    if "m" not in box:
        print("DUMPSAMPLE FAIL: %ss 内未拿到 /move_base_flex/global_costmap/costmap" % TMO)
        sys.exit(3)
    m = box["m"]
    w, h, res = int(m.info.width), int(m.info.height), float(m.info.resolution)
    ox, oy = float(m.info.origin.position.x), float(m.info.origin.position.y)
    with open(OUT, "w", encoding="utf-8") as f:
        f.write("META,%d,%d,%.4f,%.4f,%.4f,%s\n" % (w, h, res, ox, oy, m.header.frame_id))
        f.write(",".join(str(int(v)) for v in m.data) + "\n")
    vals = [int(v) for v in m.data]
    distinct = sorted(set(vals))
    p90 = sum(1 for v in vals if v >= 90)
    p100 = sum(1 for v in vals if v == 100)
    mid = sum(1 for v in vals if 1 <= v <= 98)
    unk = sum(1 for v in vals if v < 0)
    print("DUMPSAMPLE 件＝%s ‖ 几何 W=%d H=%d res=%.3f origin=(%.2f,%.2f) frame=%s ‖ distinct=%s ‖ P90(>=90≡{99,100})=%d ‖ P100(==100)=%d ‖ 1..98=%d ‖ 负值=%d ‖ 全格=%d" % (
        OUT.split("/")[-1], w, h, res, ox, oy, m.header.frame_id, distinct, p90, p100, mid, unk, len(vals)))
    sys.stdout.flush()


main()
