#!/usr/bin/env python3
"""Unit tests for the server-side voxel stream (no ROS master required)."""
import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import unittest
from types import SimpleNamespace

import numpy as np

HERE = Path(__file__).resolve().parent
SCRIPT = HERE / 'voxel_stream.py'
SPEC = importlib.util.spec_from_file_location('voxel_stream', SCRIPT)
VOXEL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VOXEL)


def point_cloud(points):
    data = bytearray()
    for x, y, z, rgb in points:
        data.extend(struct.pack('<fffI', x, y, z, rgb))
    fields = [SimpleNamespace(name='x', offset=0), SimpleNamespace(name='y', offset=4),
              SimpleNamespace(name='z', offset=8), SimpleNamespace(name='rgb', offset=12)]
    stamp = SimpleNamespace(to_sec=lambda: 12.5)
    return SimpleNamespace(header=SimpleNamespace(stamp=stamp, frame_id='map'),
                           width=len(points), height=1, point_step=16,
                           fields=fields, data=bytes(data))


class VoxelTest(unittest.TestCase):
    def test_downsample_and_nan(self):
        xyz = np.array([[0.01, 0.01, 0.01], [0.02, 0.02, 0.02],
                        [0.31, 0.01, 0.01], [1.0, np.nan, 0.0]], dtype=np.float32)
        colours = np.array([[1, 2, 3, 255], [4, 5, 6, 255],
                            [7, 8, 9, 255], [10, 11, 12, 255]], dtype=np.uint8)
        out_xyz, out_colours = VOXEL.voxel_downsample(xyz, colours, leaf=0.1)
        self.assertEqual(len(out_xyz), 2)
        self.assertEqual(out_colours.shape, (2, 4))
        self.assertTrue(all(np.isfinite(out_xyz.ravel())))

    def test_max_points_cap(self):
        xyz = np.column_stack([np.arange(1000) * 0.11, np.zeros(1000), np.zeros(1000)])
        out_xyz, _ = VOXEL.voxel_downsample(xyz, leaf=0.1, max_points=100)
        self.assertLessEqual(len(out_xyz), 100)

    def test_encode_parse_roundtrip(self):
        xyz = np.array([[1.5, -2.0, 0.25], [3.0, 4.0, 1.0]], dtype=np.float32)
        colours = np.array([[1, 2, 3, 255], [4, 5, 6, 255]], dtype=np.uint8)
        payload, bbox = VOXEL.encode_snapshot(xyz, colours, 0.1, 'map', 123.5)
        self.assertEqual(VOXEL.HEADER.size, 50)
        self.assertEqual(len(payload), 50 + 3 + 2 * 16)
        parsed = VOXEL.parse_snapshot(payload)
        self.assertEqual(parsed['count'], 2)
        self.assertEqual(parsed['frame'], 'map')
        self.assertAlmostEqual(parsed['stamp'], 123.5)
        self.assertAlmostEqual(parsed['points'][0]['x'], 1.5)
        self.assertEqual(parsed['points'][0]['r'], 1)
        self.assertEqual(bbox[0], 1.5)

    def test_cloud_to_arrays_rgb(self):
        msg = point_cloud([(1.0, 2.0, 3.0, (10 << 16) | (20 << 8) | 30)])
        xyz, colours = VOXEL.cloud_to_arrays(msg)
        self.assertEqual(xyz.shape, (1, 3))
        self.assertAlmostEqual(float(xyz[0, 2]), 3.0)
        self.assertEqual(colours[0].tolist(), [10, 20, 30, 255])

    def test_streamer_writes_atomic_snapshot(self):
        with tempfile.TemporaryDirectory() as tmp:
            streamer = VOXEL.VoxelStreamer(['/rslidar_points'], 0.1, tmp)
            msg = point_cloud([(0.0, 0.0, 0.0, 0), (1.0, 0.0, 0.0, 0)])
            self.assertTrue(streamer.process('/rslidar_points', msg))
            self.assertFalse(streamer.process('/rslidar_points', msg))
            meta = json.loads((Path(tmp) / 'meta.json').read_text(encoding='utf-8'))
            self.assertEqual(meta['count'], 2)
            self.assertEqual(meta['source'], '/rslidar_points')
            self.assertGreater(meta['bytes'], 50)
            parsed = VOXEL.parse_snapshot((Path(tmp) / 'latest.bin').read_bytes())
            self.assertEqual(parsed['count'], 2)


if __name__ == '__main__':
    unittest.main()
