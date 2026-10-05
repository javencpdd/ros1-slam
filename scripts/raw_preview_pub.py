#!/usr/bin/env python3
# 原始 master 的 1/10 抽点预览发布器（latched：订阅一次即得，不反复拉流）
import rospy, numpy as np, re
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
rospy.init_node("raw_preview_pub")
pub = rospy.Publisher("/cloud_pcd_raw", PointCloud2, queue_size=1, latch=True)
raw = open("/home/ros1-slam/data/maps/lite3_raw_master.pcd","rb").read()
n = int(re.search(rb"POINTS (\d+)", raw).group(1))
idx = raw.find(b"DATA binary\n") + len(b"DATA binary\n")
arr = np.frombuffer(raw[idx:idx+n*32], dtype=np.uint8).reshape(n,32)
sel = arr[::10]                       # 1/10 抽点
N = len(sel)
msg = PointCloud2()
msg.header = Header(); msg.header.frame_id = "map"; msg.header.stamp = rospy.Time.now()
msg.height, msg.width = 1, N
msg.fields = [PointField(n_, i, PointField.FLOAT32, 1) for i, n_ in enumerate(["x","y","z","intensity"])]
msg.point_step, msg.is_dense = 16, True
msg.data = np.ascontiguousarray(sel[:, :16]).tobytes()   # 保留 x,y,z,intensity 16 字节
msg.row_step = msg.point_step * N
pub.publish(msg)
rospy.loginfo(f"published {N} points (raw 1/10 preview, latched)")
rospy.spin()
