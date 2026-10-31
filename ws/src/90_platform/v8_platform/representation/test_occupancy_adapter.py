"""2D PGM -> OccupancyGrid data tests (no ROS)."""
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import occupancy_adapter as adapter  # noqa: E402


def make_map(width=4, height=3):
    return {'kind': 'occupancy_grid', 'frame': 'map', 'resolution': .5,
            'origin': [-1., -2., 0.], 'extent': {'width': width, 'height': height}}


class OccupancyAdapterTests(unittest.TestCase):
    def test_p5_conversion_and_row_order(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'map.pgm'
            path.write_bytes(b'P5\n# comment\n4 3\n255\n' + bytes([
                0, 254, 205, 32,
                10, 200, 255, 0,
                1, 2, 3, 4]))
            grid = adapter.occupancy_grid(make_map(), path)
            self.assertEqual((grid['width'], grid['height']), (4, 3))
            self.assertEqual(grid['frame_id'], 'map')
            self.assertEqual(grid['origin'], [-1., -2., 0.])
            self.assertEqual(len(grid['data']), 12)
            # PGM row 3 (last) becomes first ROS row.
            self.assertEqual(grid['data'][:4], [100, 100, 100, 100])
            self.assertEqual(grid['data'][4:8], [100, -1, 0, 100])
            self.assertEqual(grid['data'][8:12], [100, 0, -1, 100])

    def test_p2_and_bad_kind_or_extent(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'map.pgm'
            path.write_text('P2\n4 3\n255\n' + ' '.join(['254'] * 12) + '\n')
            grid = adapter.occupancy_grid(make_map(), path)
            self.assertEqual(grid['data'], [0] * 12)
            with self.assertRaises(ValueError):
                adapter.occupancy_grid(dict(make_map(), kind='support_map'), path)
            bad = make_map(); bad['extent'] = {'width': 5, 'height': 3}
            with self.assertRaises(ValueError):
                adapter.occupancy_grid(bad, path)

    def test_header_and_body_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'map.pgm'
            path.write_bytes(b'P5\n4 3\n255\n' + b'\x00' * 3)
            with self.assertRaises(ValueError):
                adapter.read_pgm(path)
            path.write_bytes(b'P1\n4 3\n255\n' + b'\x00' * 12)
            with self.assertRaises(ValueError):
                adapter.read_pgm(path)


if __name__ == '__main__':
    unittest.main()
