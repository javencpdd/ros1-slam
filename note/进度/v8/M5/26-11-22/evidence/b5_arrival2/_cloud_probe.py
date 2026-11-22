"""云链路探针：为什么 FAR 的图没建起来。

背景：`is_cloud_init_` 只在 **`surround_obs_cloud_` 非空**时才置真
（`far_planner.cpp:669`，在 `TerrainCallBack` 里），而 `Loop()` 的入口守卫是
`PreconditionCheck() = is_cloud_init_ && is_odom_init_`。所以「地形云里有障碍格」是整条
FAR 链路的前置。本探针把这条链拆成四段数一遍：

  1. `/rslidar_points` 有没有来（仿真 → 适配器）
  2. `/terrain_cloud` 有没有来（适配器 → FAR）
  3. 来的地形里 `intensity ≥ 0.2` 的点有几个（＝会被判成障碍格的点）
  4. `/odom_world` 有没有来（适配器 → FAR）

判读口径：2 为 0 ⇒ 卡在适配器；2 有而 3 为 0 ⇒ 卡在地形内容（平地无竖直障碍）；
3 有而 FAR 仍不建图 ⇒ 卡在 FAR 内部，再去查 `far_debug` 的图读数。
"""
import sys
import time

import rospy
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


def main():
    window = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
    state = {'cloud': 0, 'terrain': 0, 'odom': 0, 'obs': None, 'free': None, 'total': None}

    def on_cloud(_msg):
        state['cloud'] += 1

    def on_odom(_msg):
        state['odom'] += 1

    def on_terrain(msg):
        state['terrain'] += 1
        try:
            values = [p[3] for p in pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'),
                                                    skip_nans=True)]
        except Exception as error:                       # 字段缺失之类，如实报
            state['obs'] = 'ERR:%s' % str(error)[:60]
            return
        state['total'] = len(values)
        state['obs'] = sum(1 for v in values if v >= 0.2)
        state['free'] = sum(1 for v in values if v < 0.2)

    rospy.init_node('b5_cloud_probe', disable_signals=True)
    rospy.Subscriber('/rslidar_points', PointCloud2, on_cloud, queue_size=2)
    rospy.Subscriber('/terrain_cloud', PointCloud2, on_terrain, queue_size=2)
    rospy.Subscriber('/odom_world', Odometry, on_odom, queue_size=2)

    end = time.monotonic() + window
    while time.monotonic() < end:
        time.sleep(0.1)
    print('CLOUD_PROBE window=%.1f cloud=%d terrain=%d odom=%d total=%s obs=%s free=%s'
          % (window, state['cloud'], state['terrain'], state['odom'],
             state['total'], state['obs'], state['free']), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
