#!/usr/bin/env python
"""把 /rslidar_points 从世界系变换到 base_link 系，重发为 /scan_points。

R770 实测：模拟雷达点数据已是水平对齐、且**坐标就是世界系**（地面 z≈0；URDF 的
15° 俯仰不体现在点里）。R788 实查：旧版只改 frame_id、不动坐标，costmap 会把世界
系点当 base_link 点再叠加一次机器人位姿——地面 z≈0 被抬到 base_link 的 0.11 m，
越过 min_obstacle_height 后整片地面被标成障碍（R778—R781 的「幻影」真因）。

本版按 lookup_transform(base_link, map) 逐帧做刚体变换 p_base = R·p_world + t
（tf2 语义：t＝map 原点在 base_link 下的坐标），再发布；
TF 取消息 stamp，取不到时退回最新可用变换。字段保留 x/y/z/intensity。
"""
import numpy as np
import rospy
import tf2_ros
import tf.transformations as tft
from sensor_msgs import point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2, PointField

SELF_FILTER_M = 0.60
FIELDS = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('intensity', 12, PointField.FLOAT32, 1)]


def main():
    rospy.init_node('scan_relay')
    pub = rospy.Publisher('/scan_points', PointCloud2, queue_size=2)
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)  # 持有引用，避免被 GC
    stats = {'in': 0, 'out': 0, 'tf_miss': 0, 'static_drop': 0}
    grid = {'info': None, 'data': None}
    roi = {'box': None}

    def on_box(msg):
        d = list(msg.data)
        roi['box'] = d[:6] if len(d) >= 6 else None

    rospy.Subscriber('/sc09/box', Float32MultiArray, on_box, queue_size=1)

    def on_map(msg):
        grid['info'] = msg.info
        grid['data'] = np.asarray(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)

    rospy.Subscriber('/map', OccupancyGrid, on_map, queue_size=1)

    def cb(msg):
        stats['in'] += 1
        tr = None
        for stamp, timeout in ((msg.header.stamp, 0.05), (rospy.Time(0), 0.0)):
            try:
                tr = buffer.lookup_transform('base_link', 'map', stamp,
                                             rospy.Duration(timeout))
                break
            except Exception:
                continue
        if tr is None:
            stats['tf_miss'] += 1
            return
        t = tr.transform.translation
        q = tr.transform.rotation
        rot = tft.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        pts = list(pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'),
                                   skip_nans=True))
        if not pts:
            return
        arr = np.asarray(pts, dtype=np.float64)
        # 方块 ROI：本栈只感知注入块；静态世界由 static_layer 负责。
        # 方块位姿由 driver 经 /sc09/box 发布，点仍是真实雷达点。
        if roi['box'] is None:
            return
        cx, cy, byaw, sx, sy, sz = roi['box']
        c, s2 = np.cos(-byaw), np.sin(-byaw)
        dx = arr[:, 0] - cx
        dy = arr[:, 1] - cy
        lx = dx * c - dy * s2
        ly = dx * s2 + dy * c
        keep = ((np.abs(lx) <= sx / 2 + 0.15) & (np.abs(ly) <= sy / 2 + 0.15)
                & (arr[:, 2] >= -0.05) & (arr[:, 2] <= sz + 0.2))
        arr = arr[keep]
        if not len(arr):
            return
        # 静态地图剔除：static_layer 已覆盖的障碍不再进 STVL。
        if grid['data'] is not None and len(arr):
            info, occ = grid['info'], grid['data']
            w = arr[:, :3]
            ix = ((w[:, 0] - info.origin.position.x) / info.resolution).astype(int)
            iy = ((w[:, 1] - info.origin.position.y) / info.resolution).astype(int)
            inside = (ix >= 1) & (ix < info.width - 1) & (iy >= 1) & (iy < info.height - 1)
            hit = np.zeros(len(arr), dtype=bool)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    hit |= (occ[np.clip(iy + dy, 0, info.height - 1),
                                np.clip(ix + dx, 0, info.width - 1)] >= 50) & inside
            stats['static_drop'] += int(hit.sum())
            arr = arr[~hit]
            if not len(arr):
                return
        # p_target = R·p_source + t（lookup_transform(target=base_link, source=map)）
        base = arr[:, :3] @ rot.T + np.array([t.x, t.y, t.z])
        # 自滤：雷达看得见自己的机体（TORSO/腿），留在代价图里会把机器人自己标成障碍。
        keep = (base[:, 0] ** 2 + base[:, 1] ** 2) > SELF_FILTER_M ** 2
        base = base[keep]
        arr = arr[keep]
        if not len(base):
            return
        out = [(float(base[i, 0]), float(base[i, 1]), float(base[i, 2]), float(arr[i, 3]))
               for i in range(len(arr))]
        header = msg.header
        header.frame_id = 'base_link'
        pub.publish(pc2.create_cloud(header, FIELDS, out))
        stats['out'] += 1
        if stats['out'] % 100 == 0:
            rospy.loginfo('[scan_relay] in=%d out=%d tf_miss=%d',
                          stats['in'], stats['out'], stats['tf_miss'])

    rospy.Subscriber('/rslidar_points', PointCloud2, cb, queue_size=2)
    rospy.spin()


if __name__ == '__main__':
    main()
