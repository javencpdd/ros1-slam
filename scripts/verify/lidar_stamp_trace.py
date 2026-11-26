#!/usr/bin/env python3
"""雷达时间戳链路追踪（2026-11-26，M2 专项）：只读，不改感知判据。

背景：M2 组合下 `/v8/3d/perception/status` 报 `state=duplicate_frame, note="no new cloud frame this tick"`，
导致 `/v8/local/obstacles` 一帧不发、local 判 `E_LOCAL_OBSTACLES_STALE` 停车。
要判断问题在**数据源**（插件/适配层不给新戳）还是**感知判据**（去重过严），先逐级量戳：

  · `/clock`            仿真钟（rosgraph_msgs/Clock）
  · `/rslidar_points`   sim_adapter 输出（header.stamp 应逐帧前进）
  · `/livox/lidar`      插件原始 CustomMsg（可选，需 livox_laser_simulation 消息包）

判据（机器判定）：
  * 帧数 > 20 且 `unique_stamp / frames < 0.5` ⇒ **戳不前进**（数据源问题）
  * 戳逐帧单调且 Δ ≈ 0.1 s ⇒ 数据源正常，问题在感知侧
  * 戳晚于 /clock 或早于 /clock 很多 ⇒ 时基不一致

用法：
  lidar_stamp_trace.py --seconds 12
"""
import argparse

import rospy
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=12.0)
    ap.add_argument('--trace-livox', action='store_true',
                    help='同时追踪插件原始 /livox/lidar（需 livox_laser_simulation 消息类型）')
    args = ap.parse_args()

    rospy.init_node('lidar_stamp_trace', anonymous=True, disable_signals=True)
    st = {'cloud': [], 'clock': [], 'raw': []}

    def on_cloud(msg):
        st['cloud'].append(msg.header.stamp.to_sec())

    def on_clock(msg):
        st['clock'].append(msg.clock.to_sec())

    rospy.Subscriber('/rslidar_points', PointCloud2, on_cloud, queue_size=50)
    rospy.Subscriber('/clock', Clock, on_clock, queue_size=200)

    if args.trace_livox:
        try:
            from livox_laser_simulation.msg import CustomMsg

            def on_raw(msg):
                st['raw'].append(msg.header.stamp.to_sec())
            rospy.Subscriber('/livox/lidar', CustomMsg, on_raw, queue_size=50)
        except Exception as error:
            print('[warn] 无法订阅 /livox/lidar：%s' % str(error)[:120])

    rospy.sleep(max(1.0, args.seconds))

    def summarize(name, values):
        if not values:
            print('%-16s 无数据' % name)
            return None
        uniq = len(set(values))
        deltas = [b - a for a, b in zip(values, values[1:])]
        avg = sum(deltas) / len(deltas) if deltas else 0.0
        nonzero = [d for d in deltas if d > 1e-9]
        print('%-16s frames=%d unique_stamp=%d (%.0f%%)  平均Δ=%.4fs  非零Δ=%d/%d'
              % (name, len(values), uniq, 100.0 * uniq / len(values), avg,
                 len(nonzero), len(deltas)))
        return {'frames': len(values), 'unique': uniq, 'avg_delta': avg}

    cloud = summarize('/rslidar_points', st['cloud'])
    summarize('/clock', st['clock'])
    if st['raw']:
        summarize('/livox/lidar', st['raw'])

    verdict = 'UNKNOWN'
    if cloud:
        ratio = cloud['unique'] / max(1, cloud['frames'])
        if cloud['frames'] > 20 and ratio < 0.5:
            verdict = 'SRC_STAMP_STUCK'      # 数据源不给新戳
        elif 0.05 <= cloud['avg_delta'] <= 0.2:
            verdict = 'SRC_OK'               # 10 Hz 且逐帧前进 ⇒ 问题在感知侧
        else:
            verdict = 'SRC_STAMP_ODD'
    print('VERDICT %s' % verdict)


if __name__ == '__main__':
    main()
