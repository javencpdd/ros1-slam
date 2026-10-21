#!/usr/bin/env python3
# 只读探针发布器（本窗专用、★不进 scripts/ ⇒ 不入任何面）；用法：pub_grid_probe.py <ns> <mode>
# mode=3state ＝ 背景 −1 ＋ 自由块 0 ＋ 致命块 100；mode=allunknown ＝ ★整张全 −1（未知）★
import sys, time
import rospy
from nav_msgs.msg import OccupancyGrid
N = 40
def build(mode):
    d = [-1] * (N * N)
    if mode == '3state':
        for r in range(10, 13):
            for c in range(10, 13): d[r * N + c] = 0
            for c in range(20, 23): d[r * N + c] = 100
    return d
def main():
    ns, mode = sys.argv[1], sys.argv[2]
    rospy.init_node('x_gp_' + ns + '_' + mode)
    pub = rospy.Publisher('/gmap_' + ns, OccupancyGrid, queue_size=2, latch=True)
    m = OccupancyGrid(); m.header.frame_id = 'map'
    m.info.resolution = 0.1; m.info.width = N; m.info.height = N
    m.info.origin.position.x = -2.0; m.info.origin.position.y = -2.0
    m.data = build(mode)
    print('X_GPUB ns=%s mode=%s 自报落位＝背景未知 rows0-39×cols0-39(除两块) 自由块=rows10-12×cols10-12 致命块=rows10-12×cols20-22 ‖ 枚数 unknown=%d free=%d lethal=%d 总=%d'
          % (ns, mode, m.data.count(-1), m.data.count(0), m.data.count(100), len(m.data)))
    sys.stdout.flush()
    t0 = time.time()
    while not rospy.is_shutdown() and time.time() - t0 < 40:
        m.header.stamp = rospy.Time.now(); pub.publish(m); time.sleep(0.5)
main()
