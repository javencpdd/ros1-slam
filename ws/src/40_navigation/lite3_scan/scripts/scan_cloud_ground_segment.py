#!/usr/bin/env python3
"""scan_cloud_ground_segment.py — 壳侧地面平面分割（上游 SCAN 零改动）。

SCAN 的 2.5D 栅格没有地面语义：坡面/地面点会被当占据，A* 起点不可用
（2026-11-12 实测：坡上 a star error 9224—11410 次；平地地面在规划平面下方
0.28m 侥幸不触发，坡上只剩 0.075m 就落进膨胀带）。

本节点用 RANSAC 拟合机体系点云中的主导平面（地面/坡面），把该平面上的点与
平面下方的点去掉，再发 `~dst_topic`；障碍（箱体上部、墙）与机体圆柱外的点保留。
比固定 `min_z` 阈值更通用：平地、坡面、真机室内外同一套，不依赖坡度/机体俯仰。

只做点云预处理；不写上游话题，不替代感知，不改变 SCAN 算法。
"""
import numpy as np


def segment_ground(points, distance=0.08, iterations=80, self_radius=0.35, seed=0):
    """RANSAC 地面分割；返回 (保留点, 诊断信息)。

    points: Nx3 机体系点云。地面＝RANSAC 主导平面 ±distance 的内点
    （法向统一朝 +z；多面取内点最多者），另去平面下方点与机体圆柱内点。
    """
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    info = {"input": int(len(points)), "self_removed": 0, "plane": None,
            "inliers": 0, "kept": 0}
    if len(points) < 10:
        info["kept"] = int(len(points))
        return points, info
    radius = np.hypot(points[:, 0], points[:, 1])
    self_mask = radius < self_radius
    info["self_removed"] = int(self_mask.sum())
    pts = points[~self_mask]
    if len(pts) < 10:
        info["kept"] = int(len(pts))
        return pts, info

    rng = np.random.default_rng(seed)
    triples = pts[rng.integers(0, len(pts), size=(max(1, int(iterations)), 3))]
    best_count, best = -1, None
    for triple in triples:
        normal = np.cross(triple[1] - triple[0], triple[2] - triple[0])
        norm = float(np.linalg.norm(normal))
        if norm < 1e-9:
            continue
        normal = normal / norm
        offset = -float(normal.dot(triple[0]))
        if normal[2] < 0.0:
            normal, offset = -normal, -offset
        signed = pts.dot(normal) + offset
        count = int((np.abs(signed) < distance).sum())
        if count > best_count:
            best_count, best = count, (normal, offset)
    if best is None or best_count < max(10, int(0.2 * len(pts))):
        info["kept"] = int(len(pts))
        return pts, info

    normal, offset = best
    signed = pts.dot(normal) + offset
    ground = (np.abs(signed) < distance) | (signed < -distance)
    keep = ~ground
    info.update({"plane": [round(float(v), 4) for v in normal],
                 "offset": round(offset, 4), "inliers": best_count,
                 "kept": int(keep.sum())})
    return pts[keep], info


def main():
    import rospy
    from sensor_msgs import point_cloud2
    from sensor_msgs.msg import PointCloud2

    rospy.init_node('scan_cloud_ground_segment')
    src = rospy.get_param('~src_topic', '/LIO/clouds_lidar')
    dst = rospy.get_param('~dst_topic', '/LIO/clouds_lidar_ground_filtered')
    distance = float(rospy.get_param('~distance', 0.08))
    iterations = int(rospy.get_param('~iterations', 80))
    self_radius = float(rospy.get_param('~self_radius', 0.35))
    pub = rospy.Publisher(dst, PointCloud2, queue_size=2)
    rospy.loginfo('ground segment up: %s -> %s (distance=%.2f iterations=%d self_r=%.2f)',
                  src, dst, distance, iterations, self_radius)

    def callback(message):
        points = np.array([[p[0], p[1], p[2]] for p in point_cloud2.read_points(
            message, field_names=('x', 'y', 'z'), skip_nans=True)], dtype=float)
        kept, info = segment_ground(points, distance=distance, iterations=iterations,
                                    self_radius=self_radius)
        pub.publish(point_cloud2.create_cloud_xyz32(message.header, kept.tolist()))
        rospy.loginfo_throttle(5.0, 'ground segment: in=%d self=%d plane=%s inliers=%d kept=%d',
                               info['input'], info['self_removed'], info['plane'],
                               info['inliers'], info['kept'])

    rospy.Subscriber(src, PointCloud2, callback, queue_size=2, buff_size=2 ** 24)
    rospy.spin()


if __name__ == '__main__':
    main()
