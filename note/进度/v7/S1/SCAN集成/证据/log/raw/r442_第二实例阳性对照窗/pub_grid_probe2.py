#!/usr/bin/env python3
# R442 专用只读探针发布器（★新建在本留底目录内 ⇒ 不进 scripts/、不入任何面★）
# 用法：pub_grid_probe2.py <ns> <mode>   topic＝/gmap_<ns>
# mode＝3state（背景 −1 ＋ 自由块 0 ＋ 致命块 100）｜allunknown（整张全 −1）｜alllethal（整张全 100＝★阳性对照★）
import sys, time
import rospy
from nav_msgs.msg import OccupancyGrid
N = 40
def build(mode):
    if mode == 'alllethal':
        return [100] * (N * N)
    d = [-1] * (N * N)
    if mode == '3state':
        for r in range(10, 13):
            for c in range(10, 13):
                d[r * N + c] = 0
            for c in range(20, 23):
                d[r * N + c] = 100
    return d
def main():
    ns, mode = sys.argv[1], sys.argv[2]
    rospy.init_node('y_gp_' + ns + '_' + mode)
    pub = rospy.Publisher('/gmap_' + ns, OccupancyGrid, queue_size=2, latch=True)
    m = OccupancyGrid(); m.header.frame_id = 'map'
    m.info.resolution = 0.1; m.info.width = N; m.info.height = N
    m.info.origin.position.x = -2.0; m.info.origin.position.y = -2.0
    m.data = build(mode)
    print('Y_GPUB ns=%s mode=%s 自报落位＝(3state: 自由块 rows10-12×cols10-12／致命块 rows10-12×cols20-22；allunknown: 整张 −1；alllethal: 整张 100) ‖ 枚数 unknown=%d free=%d lethal=%d 总=%d ‖ origin=(-2.0,-2.0) res=0.1 w=h=40'
          % (ns, mode, m.data.count(-1), m.data.count(0), m.data.count(100), len(m.data)))
    sys.stdout.flush()
    t0 = time.time(); n = 0
    while not rospy.is_shutdown() and time.time() - t0 < 60:
        m.header.stamp = rospy.Time.now(); pub.publish(m); n += 1
        if n == 1:
            print('Y_GPUB ns=%s 首帧已发 Δt=%.3fs（latch=True ⇒ 后到的订阅者也应能拿到这一帧）' % (ns, time.time() - t0)); sys.stdout.flush()
        time.sleep(0.5)
    print('Y_GPUB ns=%s 退出前发布次数=%d 存活=%.2fs' % (ns, n, time.time() - t0)); sys.stdout.flush()
main()
