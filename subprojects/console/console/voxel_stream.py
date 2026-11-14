#!/usr/bin/env python3
"""Server-side voxel stream for the Lite3 console.

Runs inside the ROS container.  Subscribes to /cloud_registered (preferred) and
/rslidar_points, voxel-downsamples the newest cloud, and atomically writes:

  data/console/voxels/latest.bin   compact binary (16 bytes/point)
  data/console/voxels/meta.json    count/leaf/source/frame/bbox/written

The host agent serves these two files at /api/voxels/{latest,meta}; the browser
never parses a full PointCloud2 in this mode.

Binary layout (little endian):
  4s magic 'V8VX'; B version; B flags; H point_step;
  I count; f leaf; d stamp; 6f bbox; H frame_len; frame bytes;
  count * (3f xyz + 4B rgba)
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import struct
import sys
import time

import numpy as np

MAGIC = b'V8VX'
VERSION = 1
HEADER = struct.Struct('<4sBBHIfd6fH')
POINT_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                        ('r', 'u1'), ('g', 'u1'), ('b', 'u1'), ('a', 'u1')])
PALETTE = [(0.0, (30, 60, 140)), (0.3, (30, 170, 190)), (0.55, (80, 200, 90)),
           (0.78, (230, 200, 60)), (1.0, (230, 80, 60))]


def height_colour(z, zmin, zmax):
    t = 0.0 if zmax <= zmin else (float(z) - zmin) / (zmax - zmin)
    t = max(0.0, min(1.0, t))
    for index in range(len(PALETTE) - 1):
        t0, c0 = PALETTE[index]
        t1, c1 = PALETTE[index + 1]
        if t <= t1:
            f = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
            return tuple(int(round(c0[i] + (c1[i] - c0[i]) * f)) for i in range(3))
    return PALETTE[-1][1]


def voxel_downsample(xyz, colours=None, leaf=0.10, max_points=60000):
    xyz = np.asarray(xyz, dtype=np.float32)
    if xyz.size == 0:
        empty = np.zeros((0, 3), dtype=np.float32)
        return empty, (np.zeros((0, 4), dtype=np.uint8) if colours is None else colours[:0])
    finite = np.isfinite(xyz).all(axis=1)
    xyz = xyz[finite]
    if colours is not None:
        colours = np.asarray(colours)[finite]
    if not len(xyz):
        return xyz.reshape(0, 3), colours
    keys = np.floor(xyz / float(leaf)).astype(np.int64)
    _, index = np.unique(keys, axis=0, return_index=True)
    index = np.sort(index)
    xyz = xyz[index]
    if colours is not None:
        colours = colours[index]
    if max_points and len(xyz) > max_points:
        step = int(math.ceil(len(xyz) / float(max_points)))
        xyz = xyz[::step]
        if colours is not None:
            colours = colours[::step]
    return xyz, colours


def _field_array(buffer, offset, count, dtype):
    return buffer[:, offset:offset + np.dtype(dtype).itemsize].copy().view(dtype).ravel()


def cloud_to_arrays(msg):
    fields = {field.name: field for field in msg.fields}
    for name in ('x', 'y', 'z'):
        if name not in fields:
            raise ValueError('PointCloud2 missing field %s' % name)
    count = int(msg.width) * int(msg.height or 1)
    step = int(msg.point_step)
    if count <= 0 or step <= 0:
        return np.zeros((0, 3), dtype=np.float32), None
    raw = np.frombuffer(msg.data, dtype=np.uint8, count=count * step).reshape(count, step)
    xyz = np.column_stack([
        _field_array(raw, fields['x'].offset, count, '<f4'),
        _field_array(raw, fields['y'].offset, count, '<f4'),
        _field_array(raw, fields['z'].offset, count, '<f4'),
    ]).astype(np.float32)
    colours = None
    rgb_field = fields.get('rgb') or fields.get('rgba')
    if rgb_field is not None and rgb_field.offset + 4 <= step:
        packed = _field_array(raw, rgb_field.offset, count, '<u4')
        colours = np.empty((count, 4), dtype=np.uint8)
        colours[:, 0] = (packed >> 16) & 0xFF
        colours[:, 1] = (packed >> 8) & 0xFF
        colours[:, 2] = packed & 0xFF
        colours[:, 3] = 255
    return xyz, colours


def encode_snapshot(xyz, colours, leaf, frame, stamp):
    xyz = np.asarray(xyz, dtype=np.float32)
    colours = np.asarray(colours, dtype=np.uint8)
    count = len(xyz)
    if xyz.size:
        bbox = [float(xyz[:, 0].min()), float(xyz[:, 1].min()), float(xyz[:, 2].min()),
                float(xyz[:, 0].max()), float(xyz[:, 1].max()), float(xyz[:, 2].max())]
    else:
        bbox = [0.0] * 6
    frame_bytes = frame.encode('utf-8')[:65535]
    header = HEADER.pack(MAGIC, VERSION, 0, POINT_DTYPE.itemsize, count,
                         float(leaf), float(stamp), *bbox, len(frame_bytes))
    points = np.empty(count, dtype=POINT_DTYPE)
    points['x'] = xyz[:, 0]
    points['y'] = xyz[:, 1]
    points['z'] = xyz[:, 2]
    points['r'] = colours[:, 0]
    points['g'] = colours[:, 1]
    points['b'] = colours[:, 2]
    points['a'] = colours[:, 3] if colours.shape[1] > 3 else 255
    return header + frame_bytes + points.tobytes(), bbox


def parse_snapshot(payload):
    if len(payload) < HEADER.size:
        raise ValueError('short snapshot')
    magic, version, flags, step, count, leaf, stamp, *rest = HEADER.unpack_from(payload)
    if magic != MAGIC or version != VERSION:
        raise ValueError('bad snapshot magic/version')
    bbox = rest[:6]
    frame_len = rest[6]
    offset = HEADER.size
    frame = payload[offset:offset + frame_len].decode('utf-8')
    offset += frame_len
    expected = count * step
    if len(payload) < offset + expected:
        raise ValueError('snapshot payload truncated')
    points = np.frombuffer(payload, dtype=POINT_DTYPE, count=count, offset=offset)
    return {
        'count': count, 'leaf': leaf, 'stamp': stamp, 'frame': frame,
        'bbox': list(bbox), 'points': points,
    }


class VoxelStreamer:
    def __init__(self, topics, leaf, out_dir, max_points=60000):
        self.topics = topics
        self.leaf = leaf
        self.out = Path(out_dir)
        self.max_points = max_points
        self.latest = {}
        self.last_key = None
        self.stats = {'writes': 0, 'ticks': 0, 'errors': 0, 'last_log': 0.0}

    def on_cloud(self, msg, topic):
        self.latest[topic] = (msg, time.time())

    def choose(self):
        now = time.time()
        registered = self.latest.get('/cloud_registered')
        if registered and now - registered[1] < 1.5:
            return '/cloud_registered', registered[0]
        raw = self.latest.get('/rslidar_points')
        if raw and now - raw[1] < 1.5:
            return '/rslidar_points', raw[0]
        newest = max(self.latest.items(), key=lambda item: item[1][1], default=None)
        if newest and now - newest[1][1] < 3.0:
            return newest[0], newest[1][0]
        return None, None

    def process(self, topic, msg):
        stamp = msg.header.stamp.to_sec() if hasattr(msg.header.stamp, 'to_sec') else 0.0
        key = (topic, float(stamp), int(msg.width), int(msg.height or 1))
        if key == self.last_key:
            return False
        xyz, colours = cloud_to_arrays(msg)
        raw_count = len(xyz)
        xyz, colours = voxel_downsample(xyz, colours, self.leaf, self.max_points)
        if colours is None or len(colours) != len(xyz):
            if len(xyz):
                zmin, zmax = float(xyz[:, 2].min()), float(xyz[:, 2].max())
                colours = np.empty((len(xyz), 4), dtype=np.uint8)
                for index, point in enumerate(xyz):
                    colours[index, :3] = height_colour(point[2], zmin, zmax)
                    colours[index, 3] = 255
            else:
                colours = np.zeros((0, 4), dtype=np.uint8)
        frame = msg.header.frame_id or 'map'
        written = time.time()
        payload, bbox = encode_snapshot(xyz, colours, self.leaf, frame, written)
        self.out.mkdir(parents=True, exist_ok=True)
        for folder in (self.out.parent, self.out):
            try:
                os.chmod(folder, 0o777)
            except OSError:
                pass
        binary_tmp = self.out / 'latest.bin.tmp'
        meta_tmp = self.out / 'meta.json.tmp'
        binary_tmp.write_bytes(payload)
        os.chmod(binary_tmp, 0o666)
        meta = {
            'ok': True,
            'schema': 'v8-voxels/v1',
            'stamp': written,
            'written': written,
            'source_stamp': float(stamp),
            'source': topic,
            'frame': frame,
            'count': int(len(xyz)),
            'raw_count': int(raw_count),
            'leaf': float(self.leaf),
            'max_points': int(self.max_points),
            'bbox': bbox,
            'bytes': len(payload),
        }
        meta_tmp.write_text(json.dumps(meta, ensure_ascii=False, separators=(',', ':')),
                            encoding='utf-8')
        os.chmod(meta_tmp, 0o666)
        os.replace(binary_tmp, self.out / 'latest.bin')
        os.replace(meta_tmp, self.out / 'meta.json')
        self.last_key = key
        self.stats['writes'] += 1
        now = time.time()
        if now - self.stats['last_log'] > 5.0:
            self.stats['last_log'] = now
            print('[voxel] %s %d -> %d pts leaf=%.2f frame=%s' % (
                topic, raw_count, len(xyz), self.leaf, frame), flush=True)
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--leaf', type=float, default=0.10)
    parser.add_argument('--rate', type=float, default=2.0)
    parser.add_argument('--max-points', type=int, default=60000)
    parser.add_argument('--out', default=os.environ.get('V8_CONSOLE_DATA', '/opt/ros1-slam/data/console') + '/voxels')
    parser.add_argument('--topics', nargs='*',
                        default=['/cloud_registered', '/rslidar_points'])
    args = parser.parse_args()
    import rospy
    from sensor_msgs.msg import PointCloud2

    streamer = VoxelStreamer(args.topics, args.leaf, args.out, args.max_points)
    rospy.init_node('v8_console_voxel_stream', anonymous=False)
    for topic in args.topics:
        rospy.Subscriber(topic, PointCloud2,
                         lambda msg, name=topic: streamer.on_cloud(msg, name),
                         queue_size=1, buff_size=2 ** 24)
        print('[voxel] subscribe %s' % topic, flush=True)
    rate = rospy.Rate(max(0.1, args.rate))
    print('[voxel] out=%s leaf=%.3f rate=%.1f Hz' % (args.out, args.leaf, args.rate),
          flush=True)
    while not rospy.is_shutdown():
        streamer.stats['ticks'] += 1
        topic, msg = streamer.choose()
        if topic is not None:
            try:
                streamer.process(topic, msg)
            except (ValueError, KeyError, IndexError, OSError) as error:
                streamer.stats['errors'] += 1
                if streamer.stats['errors'] <= 5 or streamer.stats['errors'] % 50 == 0:
                    print('[voxel] error: %s' % error, file=sys.stderr, flush=True)
        rate.sleep()


if __name__ == '__main__':
    main()
