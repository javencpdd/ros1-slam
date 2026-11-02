import json
import math
import struct
import tempfile
import unittest
from pathlib import Path

import numpy as np

import pcd_stats


def ascii_pcd(path, rows):
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
        'WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n'
        % (len(rows), len(rows)))
    body = ''.join('%f %f %f\n' % tuple(row) for row in rows)
    Path(path).write_text(header + body, encoding='utf-8')


def binary_pcd(path, rows):
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
        'WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n'
        % (len(rows), len(rows)))
    body = b''.join(struct.pack('<3f', *row) for row in rows)
    Path(path).write_bytes(header.encode('ascii') + body)


class PcdStatsTests(unittest.TestCase):
    def test_ascii_and_binary_read(self):
        rows = [(0., 0., 0.), (1., 2., 0.5)]
        with tempfile.TemporaryDirectory() as tmp:
            ascii_path = Path(tmp) / 'a.pcd'
            binary_path = Path(tmp) / 'b.pcd'
            ascii_pcd(ascii_path, rows)
            binary_pcd(binary_path, rows)
            for path in (ascii_path, binary_path):
                points = pcd_stats.read_pcd(path)
                self.assertEqual(points.shape, (2, 3))
                self.assertAlmostEqual(points[1, 2], 0.5)
                result = pcd_stats.stats(points, cell_m=0.5, z_band_m=0.1)
                self.assertEqual(result['points'], 2)
                self.assertEqual(result['xy_cells_0_5m'], 2)
                self.assertEqual(result['z_bands_0_1m'], 2)

    def test_clearance_and_transform(self):
        path_value = {
            'waypoints': [
                {'x': 0., 'y': 0., 'z': 0.05},
                {'x': 1., 'y': 0., 'z': 0.05},
            ]
        }
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'path.json'
            path.write_text(json.dumps(path_value), encoding='utf-8')
            clear = np.array([[0.5, 0.5, 0.5], [2., 2., 0.5]])
            result = pcd_stats.clearance(
                clear, path, compare={'origin_offset': [0., 0., 0.],
                                      'origin_yaw_rad': 0.},
                radius_m=.3, sample_m=.1)
            self.assertTrue(result['ok'])
            self.assertAlmostEqual(result['min_clearance_m'], .5, places=6)
            close = np.vstack([clear, [0.1, 0.1, 0.5]])
            result = pcd_stats.clearance(
                close, path, compare={'origin_offset': [0., 0., 0.],
                                      'origin_yaw_rad': 0.},
                radius_m=.3, sample_m=.1)
            self.assertFalse(result['ok'])
            self.assertAlmostEqual(result['min_clearance_m'], .1, places=6)


if __name__ == '__main__':
    unittest.main()
