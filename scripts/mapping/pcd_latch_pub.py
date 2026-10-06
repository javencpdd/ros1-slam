#!/usr/bin/env python3
# pcd_latch_pub.py v3 — v2 基础上 NaN 滤除扩展到 intensity 列（v2 只滤 x/y/z，intensity 的 NaN 漏网致 Foxglove 892 告警）
import sys, re, rospy, numpy as np
from sensor_msgs.msg import PointCloud2, PointField
path, topic, dec = sys.argv[1], sys.argv[2], int(sys.argv[3])
rospy.init_node("pcd_latch_pub", anonymous=True)
raw = open(path, "rb").read()
hdr = raw[:raw.find(b"DATA binary\n")+len(b"DATA binary\n")].decode()
n  = int(re.search(r"POINTS (\d+)", hdr).group(1))
step = int(re.search(r"POINT_STEP (\d+)", hdr).group(1))
fields_line = re.search(r"FIELDS (.+)", hdr).group(1).split()
idx = raw.find(b"DATA binary\n")+len(b"DATA binary\n")
arr = np.frombuffer(raw[idx:], dtype=np.uint8).reshape(n, step)
x = arr[:,0:4].copy().view("<f4").ravel(); y = arr[:,4:8].copy().view("<f4").ravel(); z = arr[:,8:12].copy().view("<f4").ravel()
it = arr[:,12:16].copy().view("<f4").ravel() if step >= 16 and fields_line[3] == "intensity" else np.zeros(n, np.float32)
fin = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & np.isfinite(it)   # ★ v3：intensity 纳入 NaN 滤除
sel = arr[fin]
if dec > 1:
    sel = sel[::dec]
N = len(sel)
out = np.zeros((N, 4), dtype=np.float32)
out[:,0] = sel[:,0:4].copy().view("<f4").ravel()
out[:,1] = sel[:,4:8].copy().view("<f4").ravel()
out[:,2] = sel[:,8:12].copy().view("<f4").ravel()
out[:,3] = sel[:,12:16].copy().view("<f4").ravel() if step >= 16 and fields_line[3] == "intensity" else 0.0
msg = PointCloud2()
msg.header.frame_id = "camera_init"; msg.header.stamp = rospy.Time.now()
msg.height, msg.width = 1, N
msg.fields = [PointField(nm, i, PointField.FLOAT32, 1) for i, nm in enumerate(["x","y","z","intensity"])]
msg.point_step, msg.is_dense = 16, True
msg.data = np.ascontiguousarray(out).tobytes()
msg.row_step = msg.point_step * N
pub = rospy.Publisher(topic, PointCloud2, queue_size=1, latch=True)
for _ in range(5):
    pub.publish(msg); rospy.sleep(0.2)
rospy.loginfo(f"[latch] {path.split('/')[-1]} → {topic}: {N} 点（NaN 全字段滤除，1/{dec} 抽点）已 latched")
rospy.spin()
