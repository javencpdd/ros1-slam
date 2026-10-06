#!/usr/bin/env python3
# pcd_latch_pub.py v4 — 兼容 PCL 无 POINT_STEP 头 + 自适应抽点（大图自动降到 ≤200 万点）
import sys, re, rospy, numpy as np
from sensor_msgs.msg import PointCloud2, PointField
path, topic, dec_arg = sys.argv[1], sys.argv[2], sys.argv[3]
rospy.init_node("pcd_latch_pub", anonymous=True)
raw = open(path, "rb").read()
hdr = raw[:raw.find(b"DATA binary\n")+len(b"DATA binary\n")].decode(errors="replace")
n = int(re.search(r"POINTS (\d+)", hdr).group(1))
m = re.search(r"POINT_STEP (\d+)", hdr)
if m:
    step = int(m.group(1))
else:
    sizes = [int(s) for s in re.search(r"SIZE (.+)", hdr).group(1).split()]
    counts = [int(c) for c in re.search(r"COUNT (.+)", hdr).group(1).split()] if re.search(r"COUNT (.+)", hdr) else [1]*len(sizes)
    step = sum(s*c for s, c in zip(sizes, counts))
fields_line = re.search(r"FIELDS (.+)", hdr).group(1).split()
idx = raw.find(b"DATA binary\n")+len(b"DATA binary\n")
arr = np.frombuffer(raw[idx:], dtype=np.uint8).reshape(n, step)
x = arr[:,0:4].copy().view("<f4").ravel(); y = arr[:,4:8].copy().view("<f4").ravel(); z = arr[:,8:12].copy().view("<f4").ravel()
fin = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
sel = arr[fin]
N0 = len(sel)
dec = int(dec_arg) if int(dec_arg) > 0 else max(1, int(np.ceil(N0 / 2_000_000)))   # 自适应：目标 ≤200 万点
sel = sel[::dec]
N = len(sel)
out = np.zeros((N, 4), dtype=np.float32)
out[:,0] = sel[:,0:4].copy().view("<f4").ravel()
out[:,1] = sel[:,4:8].copy().view("<f4").ravel()
out[:,2] = sel[:,8:12].copy().view("<f4").ravel()
out[:,3] = sel[:,12:16].copy().view("<f4").ravel() if (step >= 16 and fields_line[3] == "intensity") else 0.0
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
rospy.loginfo(f"[latch] {path.split('/')[-1]}: {N0}→{N} 点（自适应 1/{dec}，NaN 全滤）已 latched")
rospy.spin()
