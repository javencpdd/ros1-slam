#!/usr/bin/env python3
# 03_sub_elev_count.py —— ★缺② 的正解：给 lethal= 找一个真独立来源★＝★订阅侧回读 /elev_map 后对收到的消息逐格数 100★
#   ★与发布器内的 painted／lethal_重数不共用同一枚 data（不同进程、走 ROS 序列化往返）★
import collections
import sys
import rospy
from nav_msgs.msg import OccupancyGrid
GOT = []
def cb(m):
    GOT.append(m)
rospy.init_node("elev_count_r525")
pub = rospy.Subscriber("/elev_map", OccupancyGrid, cb, queue_size=5)
t0 = rospy.Time.now()
while not rospy.is_shutdown() and len(GOT) < 1 and (rospy.Time.now() - t0).to_sec() < 35:
    rospy.sleep(0.2)
if not GOT:
    print("SECSUB FAIL: /elev_map 35s 内一条未收到 ⇒ ★独立来源未合、C 态自记缺项★")
    sys.exit(7)
m = GOT[0]
cnt = collections.Counter(m.data)
w, h = int(m.info.width), int(m.info.height)
res = float(m.info.resolution)
ox = float(m.info.origin.position.x); oy = float(m.info.origin.position.y)
# 独立来源二之二：在订阅到的那张图上按世界坐标重算靶区 T 的 100 枚数
sc = int(round((ox - (-50.0)) / res)); sr = int(round((oy - (-50.0)) / res))
inT = 0
for c in range(476, 535):
    for r in range(523, 544):
        cc, rr = c - sc, r - sr
        if 0 <= cc < w and 0 <= rr < h and m.data[rr * w + cc] == 100:
            inT += 1
print("SECSUB src=/elev_map size=%dx%d res=%.3f origin=(%.2f,%.2f) ‖ 消息内 v==100 全图＝%d ‖ v==99 全图＝%d ‖ 0 全图＝%d ‖ ★订阅侧 T 内 v==100＝%d（n_T 期望 1239）★ ‖ 收到消息数＝%d ‖ 订阅进程 pid＝%d" % (
    w, h, res, ox, oy, cnt[100], cnt[99], cnt[0], inT, len(GOT), os.getpid() if False else __import__("os").getpid()))
