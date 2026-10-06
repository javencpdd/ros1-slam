#!/usr/bin/env python3
# pcd_latch_pub.py — PCD → latched PointCloud2（4字段 x/y/z/intensity，NaN 滤除，可选抽点）
import sys, re, rospy, numpy as np
from sensor_msgs.msg import PointCloud2, PointField
path, topic, dec = sys.argv[1], sys.argv[2], int(sys.argv[3])
rospy.init_node("pcd_latch_pub", anonymous=True)
raw = open(path, "rb").read()
n = int(re.search(rb"POINTS (\d+)", raw).group(1))
idx = raw.find(b"DATA binary\n") + len(b"DATA binary\n")
arr = np.frombuffer(raw[idx:idx+n*32], dtype=np.uint8).reshape(n, 32)
x = arr[:,0:4].copy().view("<f4").ravel(); y = arr[:,4:8].copy().view("<f4").ravel(); z = arr[:,8:12].copy().view("<f4").ravel()
fin = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
sel = arr[fin]
if dec > 1:
    sel = sel[::dec]
msg = PointCloud2()
msg.header.frame_id = "camera_init"; msg.header.stamp = rospy.Time.now()
msg.height, msg.width = 1, len(sel)
msg.fields = [PointField(nm, i, PointField.FLOAT32, 1) for i, nm in enumerate(["x","y","z","intensity"])]
msg.point_step, msg.is_dense = 16, True
msg.data = np.ascontiguousarray(sel[:, :16]).tobytes()
msg.row_step = msg.point_step * len(sel)
pub = rospy.Publisher(topic, PointCloud2, queue_size=1, latch=True)
for _ in range(5):
    pub.publish(msg); rospy.sleep(0.2)
rospy.loginfo(f"[latch] {path.split('/')[-1]} → {topic}: {len(sel)} 点（NaN 滤后，1/{dec} 抽点）已 latched")
rospy.spin()
