#!/usr/bin/env python3
# 只读探针发布器（本窗专用、★不进 scripts/ 目录⇒不入任何面、★不改任何在册件）
# 目的＝造一张★含未知格★的栅格：背景=−1（未知），中间一块=0（自由），一块=100（致命）
# 这样 costmap 出来后可以★直接比★"未知格落到的 cost" 与 "自由格落到的 cost" 是不是同一个值
import sys, time
import rospy
from nav_msgs.msg import OccupancyGrid

N = 40
def build(mode):
    d = [-1] * (N * N)                     # 背景＝未知
    for r in range(10, 13):
        for c in range(10, 13):
            d[r * N + c] = 0                # 自由块
        for c in range(20, 23):
            d[r * N + c] = 100 if mode == 'lethal' else 0   # 对照块（默认致命）
    return d

def main():
    tag = sys.argv[1] if len(sys.argv) > 1 else 'u'
    rospy.init_node('x_up_' + tag)
    pub = rospy.Publisher('/umap_' + tag, OccupancyGrid, queue_size=2, latch=True)
    m = OccupancyGrid()
    m.header.frame_id = 'map'
    m.info.resolution = 0.1
    m.info.width = N; m.info.height = N
    m.info.origin.position.x = -2.0; m.info.origin.position.y = -2.0
    m.data = build('lethal')
    print('X_UPUB built: unknown=%d free=%d lethal=%d' % (m.data.count(-1), m.data.count(0), m.data.count(100))); sys.stdout.flush()
    t0 = time.time()
    while not rospy.is_shutdown() and time.time() - t0 < 40:
        m.header.stamp = rospy.Time.now()
        pub.publish(m); time.sleep(0.5)

main()
