"""flat2d 纯几何件（无 ROS 依赖，可宿主侧测试）。"""


def box_dist(px, py, cx, cy, sx, sy, yaw):
    import math
    c, s = math.cos(-yaw), math.sin(-yaw)
    dx, dy = px - cx, py - cy
    lx, ly = dx * c - dy * s, dx * s + dy * c
    return max(abs(lx) - sx / 2, abs(ly) - sy / 2, 0.0)


def min_clearance(points, boxes):
    if not points:
        return None
    return min(box_dist(p[0], p[1], *b) for p in points for b in boxes)


def sdf_box(name, size, pose_xy_yaw, z):
    sx, sy, sz = size
    return """<?xml version="1.0"?>
<sdf version="1.6"><model name="%s"><static>true</static>
<pose>%.3f %.3f %.3f 0 0 %.4f</pose>
<link name="link"><collision name="col"><geometry><box><size>%.2f %.2f %.2f</size></box></geometry></collision>
<visual name="vis"><geometry><box><size>%.2f %.2f %.2f</size></box></geometry>
<material><ambient>0.9 0.1 0.1 1</ambient></material></visual></link></model></sdf>""" % (
        name, pose_xy_yaw[0], pose_xy_yaw[1], z, pose_xy_yaw[2],
        sx, sy, sz, sx, sy, sz)
