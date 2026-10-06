#!/usr/bin/env python3
# map_snapshot_pub.py — 采图伴侣：订阅 /cloud_registered 内存累积，每 30s 发布 LIO 视角全图快照（latched）
# 累积含 faster_lio 的历史修正（区别于 Foxglove 端 Decay 显示），0.1m 体素在线去重防内存膨胀
import rospy
import numpy as np
from collections import deque
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

VOXEL = 0.10          # 在线去重体素（m）
SNAP_PERIOD = 30.0    # 快照周期（s）
CAP = 600_000         # 快照点数上限（超限提体素至 0.15 重剪）

rospy.init_node("map_snapshot_pub")
pub = rospy.Publisher("/map_snapshot", PointCloud2, queue_size=1, latch=True)
voxels = {}           # 体素键 -> 点(np 4f32)
order = deque()       # 插入顺序（FIFO 防膨胀）

def fields4():
    return [PointField(n, i, PointField.FLOAT32, 1) for i, n in enumerate(["x","y","z","intensity"])]

def make_msg(pts):
    m = PointCloud2()
    m.header = Header(); m.header.frame_id = "camera_init"; m.header.stamp = rospy.Time.now()
    m.height, m.width = 1, len(pts)
    m.fields = fields4()
    m.point_step = 16
    m.is_dense = True
    m.data = np.ascontiguousarray(pts[:, :4]).tobytes()
    m.row_step = m.point_step * len(pts)
    return m

def prune():
    """超限时提高体素粒度整体重剪（0.10→0.15→0.20）"""
    global voxels, order, VOXEL
    while len(voxels) > CAP and VOXEL < 0.20:
        VOXEL += 0.05
        pts = np.array([v for v in voxels.values()])
        keys = set(map(tuple, np.floor(pts[:, :3] / VOXEL).astype(np.int64)))
        nv, no = {}, deque()
        for k in keys:
            pass
        # 简化：按新粒度重建
        nv = {}
        for v in pts:
            key = tuple(np.floor(v[:3] / VOXEL).astype(np.int64))
            if key not in nv:
                nv[key] = v
                no.append(key)
        voxels, order = nv, no
    rospy.loginfo(f"[snapshot] 体素粒度调至 {VOXEL}m，当前 {len(voxels)} 点")

def cb(msg):
    step = msg.point_step
    if step < 12:
        return
    n = msg.width * msg.height
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if len(raw) < n * step:
        return
    arr = raw[: n * step].reshape(n, step)
    x = arr[:, 0:4].copy().view(np.float32).ravel()
    y = arr[:, 4:8].copy().view(np.float32).ravel()
    z = arr[:, 8:12].copy().view(np.float32).ravel()
    it = arr[:, 12:16].copy().view(np.float32).ravel() if step >= 16 else np.zeros(n, np.float32)
    fin = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    for j in np.where(fin)[0]:
        key = (int(np.floor(x[j] / VOXEL)), int(np.floor(y[j] / VOXEL)), int(np.floor(z[j] / VOXEL)))
        if key not in voxels:
            voxels[key] = np.array([x[j], y[j], z[j], it[j]], np.float32)
            order.append(key)
            if len(voxels) > CAP:
                prune()

def snap_timer(_):
    if not voxels:
        rospy.loginfo_throttle(60, "[snapshot] 暂无点（等 /cloud_registered）")
        return
    pts = np.array([voxels[k] for k in order])
    pub.publish(make_msg(pts))
    rospy.loginfo(f"[snapshot] {len(pts)} 点 @ {VOXEL}m 体素")

rospy.Subscriber("/cloud_registered", PointCloud2, cb, queue_size=2)
rospy.Timer(rospy.Duration(SNAP_PERIOD), snap_timer)
rospy.Timer(rospy.Duration(SNAP_PERIOD), snap_timer) if False else None
rospy.spin()
