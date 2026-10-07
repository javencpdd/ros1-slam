#!/usr/bin/env python3
"""p1_occ_sample.py —— 采样 /grid_map/occupancy 序列，输出 JSON（点数/包围盒/frame_id/时间戳）。
A7 判据①「grid_map 30s 内收敛且占据图与 world 一致」的可测化取证件。零写入，只订阅。"""
import json
import sys
import time
from collections import deque

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2


def axis(msg, name):
    """按步长切片取单分量，避开 np.dtype(fields=...) 在 pip numpy 上的构造异常。"""
    for f in msg.fields:
        if f.name != name or f.datatype != 7:  # 7 = FLOAT32
            continue
        stride = msg.point_step // 4
        start = f.offset // 4
        n = msg.width * msg.height
        need = (n - 1) * stride + start + 1          # 末点该分量之后不再取元素
        if n <= 0 or start >= stride or need > len(msg.data) // 4:
            return None
        try:
            v = np.frombuffer(msg.data, dtype=np.float32,
                              count=need).reshape(-1)[start:start + n * stride:stride]
        except (ValueError, IndexError):
            return None
        return v[np.isfinite(v)]
    return None


def to_axes(msg):
    out = {}
    for ax in ("x", "y", "z"):
        try:
            v = axis(msg, ax)
        except Exception:
            v = None
        if v is not None and len(v):
            out[ax] = (float(v.min()), float(v.max()), float(np.median(v)), int(len(v)))
    return out


samples = []


def cb(msg):
    axes = to_axes(msg)
    n = msg.width * msg.height
    rec = {"n": int(n), "frame_id": msg.header.frame_id,
           "stamp": float(msg.header.stamp.secs) + msg.header.stamp.nsecs / 1e9,
           "wall": time.time()}
    for ax, (lo, hi, med, cnt) in axes.items():
        rec[ax] = [round(lo, 3), round(hi, 3), round(med, 3)]
    samples.append(rec)


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else "/grid_map/occupancy"
    window = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
    ref_topic = sys.argv[3] if len(sys.argv) > 3 else "/cloud_registered"

    rospy.init_node("p1_occ_sample", anonymous=True)
    ref = {}

    def refcb(msg):
        for ax, (lo, hi, _med, _cnt) in to_axes(msg).items():
            ref[ax] = [lo, hi]

    rospy.Subscriber(ref_topic, PointCloud2, refcb, queue_size=1, buff_size=2 ** 26)
    rospy.Subscriber(topic, PointCloud2, cb, queue_size=5, buff_size=2 ** 26)

    t0 = time.time()
    while time.time() - t0 < window and not rospy.is_shutdown():
        time.sleep(0.5)

    out = {"topic": topic, "window_s": window, "frames": len(samples),
           "counts": [s["n"] for s in samples],
           "first": samples[0] if samples else None,
           "last": samples[-1] if samples else None,
           "ref_bbox": {k: [round(v[0], 3), round(v[1], 3)] for k, v in ref.items()},
           "ref_topic": ref_topic}
    if samples and ref:
        inside = 0
        tot = 0
        for s in samples[-3:]:
            if not ("x" in s and "y" in s and "x" in ref and "y" in ref):
                continue
            tot += 1
            if (s["x"][0] >= ref["x"][0] - 0.5 and s["x"][1] <= ref["x"][1] + 0.5
                    and s["y"][0] >= ref["y"][0] - 0.5 and s["y"][1] <= ref["y"][1] + 0.5):
                inside += 1
        out["bbox_check_samples"] = tot
        out["bbox_within_ref"] = (inside == tot) if tot else None
        out["bbox_inside_count"] = inside
    out["ref_has_xy"] = ("x" in ref and "y" in ref)
    print(json.dumps(out, ensure_ascii=False))


if __name__ == "__main__":
    main()
