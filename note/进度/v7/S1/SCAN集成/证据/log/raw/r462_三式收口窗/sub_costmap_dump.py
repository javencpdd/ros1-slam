#!/usr/bin/env python3
# sub_costmap_dump.py —— ★C7-B 的内容栏取样器★：抓一帧 global costmap 全格、连几何一起落 CSV
# 用法：sub_costmap_dump.py <输出csv> [超时秒]；★只订不写别的、不改动任何在册件★
# 首行＝`META,W,H,RES,OX,OY,FRAME`；次行＝★按行主序的格值（-1..100）逗号串★ ⇒ 对照件按★世界坐标对齐★做逐格差异
import sys

import rospy
from nav_msgs.msg import OccupancyGrid

out = sys.argv[1]
tmo = float(sys.argv[2]) if len(sys.argv) > 2 else 90.0
box = {"m": None}


def cb(m):
    if box["m"] is None:
        box["m"] = m


rospy.init_node("cm_dump_r462")
sub = rospy.Subscriber("/move_base_flex/global_costmap/costmap", OccupancyGrid, cb, queue_size=1)
t0 = rospy.Time.now()
while box["m"] is None and (rospy.Time.now() - t0).to_sec() < tmo and not rospy.is_shutdown():
    rospy.sleep(0.2)
if box["m"] is None:
    print("DUMPSAMPLE FAIL: %ss 内未拿到 /move_base_flex/global_costmap/costmap" % tmo)
    sys.exit(3)
m = box["m"]
w, h = int(m.info.width), int(m.info.height)
res = float(m.info.resolution)
ox = float(m.info.origin.position.x)
oy = float(m.info.origin.position.y)
with open(out, "w", encoding="utf-8") as f:
    f.write("META,%d,%d,%.4f,%.4f,%.4f,%s\n" % (w, h, res, ox, oy, m.header.frame_id))
    f.write(",".join(str(int(v)) for v in m.data) + "\n")
lethal = sum(1 for v in m.data if int(v) >= 90)
free = sum(1 for v in m.data if int(v) == 0)
print("DUMPSAMPLE ok %s W=%d H=%d res=%.3f origin=(%.2f,%.2f) frame=%s lethal=%d free=%d unknown=%d total=%d" % (
    out.split("/")[-1], w, h, res, ox, oy, m.header.frame_id, lethal, free, len(m.data) - lethal - free, len(m.data)))
