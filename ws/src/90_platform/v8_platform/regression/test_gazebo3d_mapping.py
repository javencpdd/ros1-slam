"""Sensor-map transform and support-map pipeline checks (no ROS)."""
import json
import math
import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import gazebo3d_mapping as mapping


class MappingTests(unittest.TestCase):
    def test_transform_uses_mount_pitch_and_base_yaw(self):
        point = np.array([[1.0, 0.0, 0.0]])
        world = mapping.transform_points(point, (2.0, 3.0, 0.1), mapping.yaw_quaternion(math.pi / 2))
        expected = np.array([[2.0, 3.0 + math.cos(math.radians(15.0)),
                              0.1 + 0.34 - math.sin(math.radians(15.0))]])
        np.testing.assert_allclose(world, expected, atol=1e-9)

    def test_consolidation_merges_coplanar_fragments_only(self):
        surfaces = [
            {'surface_id': 'a', 'z_reference': 0.200, 'footprint': {'min_x': -1., 'min_y': 0., 'max_x': 0., 'max_y': 1.}},
            {'surface_id': 'b', 'z_reference': 0.205, 'footprint': {'min_x': 0., 'min_y': 0., 'max_x': 1., 'max_y': 1.}},
            {'surface_id': 'c', 'z_reference': 0.500, 'footprint': {'min_x': -1., 'min_y': 0., 'max_x': 1., 'max_y': 1.}},
        ]
        merged = mapping.consolidate_surfaces(surfaces, z_merge=0.05, xy_gap=0.02)
        self.assertEqual(len(merged), 2)
        self.assertEqual([item['surface_id'] for item in merged], ['m0', 'm1'])
        self.assertAlmostEqual(merged[0]['z_reference'], 0.2025, places=6)
        self.assertEqual(merged[0]['footprint']['min_x'], -1.)
        self.assertEqual(merged[0]['footprint']['max_x'], 1.)

    def test_synthetic_stairs_build_connected_map(self):
        points = []
        for level, (low, high) in zip((0.0, 0.2, 0.4), ((-3., -2.), (-2., -1.), (-1., 0.))):
            for x in np.arange(-1., 1.01, 0.1):
                for y in np.arange(low, high + 0.01, 0.1):
                    points.append((x, y, level))
        value, info = mapping.build_map_from_points(
            np.array(points), resolution=0.1, map_id='synthetic', map_version=1,
            source={'uri': 'test://synthetic', 'sha256': '0' * 64},
            z_tol=0.05, min_cells=1, min_area=0.05, min_side=0.05,
            xy_gap=0.05, max_z_step=0.25)
        self.assertEqual([round(s['z_reference'], 3) for s in value['surfaces']], [0.0, 0.2, 0.4])
        self.assertEqual(len(value['surfaces']), 3)
        self.assertTrue(all(surface['edges'] for surface in value['surfaces']))
        self.assertEqual(info['supported_surfaces'], 3)
        self.assertEqual(info['map_version'], 1)

    def test_score_matches_truth_levels(self):
        points = []
        for level, (low, high) in zip((0.0, 0.2), ((-2., -1.), (-1., 0.))):
            for x in np.arange(-0.5, 0.51, 0.1):
                for y in np.arange(low, high + 0.01, 0.1):
                    points.append((x, y, level))
        value, _ = mapping.build_map_from_points(
            np.array(points), resolution=0.1, map_id='score', map_version=1,
            source={'uri': 'test://score', 'sha256': '0' * 64},
            z_tol=0.05, min_cells=1, min_area=0.05, min_side=0.05)
        truth = {'surfaces': [
            {'surface_id': 'ground', 'z_reference': 0.0,
             'footprint': {'min_x': -1., 'min_y': -2., 'max_x': 1., 'max_y': -1.},
             'edges': [{'to': 'step1'}]},
            {'surface_id': 'step1', 'z_reference': 0.2,
             'footprint': {'min_x': -1., 'min_y': -1., 'max_x': 1., 'max_y': 0.},
             'edges': [{'to': 'ground'}]},
        ]}
        score = mapping.score_against_truth(value, truth)
        self.assertEqual(score['matched_truth'], 2)
        self.assertEqual(score['missing_truth'], [])
        self.assertLess(score['level_max_error_m'], 1e-6)

    def test_z_align_absorb_cells_and_full_cloud_obstacles(self):
        """a+d+D：地面对齐、近地碎片并入、格集、全点云墙盒与稀疏列剔除。"""
        floor, patch, plate = [], [], []
        for x in np.arange(-2., 2.01, 0.1):
            for y in np.arange(-2., 2.01, 0.1):
                floor.append((x, y, -0.05))
        for x in np.arange(0.5, 1.51, 0.1):
            for y in np.arange(0.5, 1.51, 0.1):
                patch.append((x, y, 0.02))
        for x in np.arange(2.05, 3.01, 0.1):
            for y in np.arange(-0.5, 0.51, 0.1):
                plate.append((x, y, 0.15))
        wall = [(3.0, y, z) for y in np.arange(-1.0, 1.01, 0.05)
                for z in np.arange(0.1, 1.01, 0.05)]
        sparse = [(-1.8, -1.8, 0.30), (-1.8, -1.8, 0.55), (-1.8, -1.8, 0.80),
                  (-1.8, -1.8, 1.05), (-1.8, -1.8, 1.30), (-1.8, -1.8, 1.55)]
        horizontal = np.array(floor + patch + plate, dtype=float)
        full = np.array(floor + patch + plate + wall + sparse, dtype=float)
        value, info = mapping.build_map_from_points(
            horizontal, resolution=0.1, map_id='cells', map_version=4,
            source={'uri': 'test://cells', 'sha256': '0' * 64},
            z_tol=0.05, min_cells=1, min_area=0.05, min_side=0.05,
            xy_gap=0.05, max_z_step=0.25,
            obstacle_mode='full', full_points=full,
            obstacle_band=(0.02, 1.20), z_align=True, absorb_near_floor_m=0.08)
        levels = [round(surface['z_reference'], 3) for surface in value['surfaces']]
        self.assertIn(0.0, levels)
        self.assertIn(0.20, levels)
        self.assertNotIn(0.07, levels)          # 近地碎片已并入地面
        self.assertEqual(info['z_shift'], 0.05)
        self.assertEqual(info['absorbed_surfaces'], ['m1'])
        self.assertTrue(all(surface['cells']['runs'] for surface in value['surfaces']))
        wall_boxes = [box for box in value['obstacles']
                      if box['min'][0] >= 2.8 and box['max'][2] >= 0.9]
        self.assertTrue(wall_boxes, 'full-cloud wall missing')
        self.assertFalse([box for box in value['obstacles']
                          if box['min'][0] <= -1.6 <= box['max'][0]
                          and box['min'][1] <= -1.6 <= box['max'][1]],
                         'sparse column extruded into an obstacle box')

    def test_refined_surface_obstacles_follow_cells(self):
        """B：被拒面按真实格拆矩形，空角不再被障碍盒覆盖。"""
        rejected = [{'surface_id': 's0', 'z_reference': 0.4,
                     'footprint': {'min_x': 0., 'min_y': 0., 'max_x': 2., 'max_y': 2.},
                     'cells': [[0, 0], [1, 0], [0, 1], [1, 1], [0, 2], [1, 2]]}]
        boxes = mapping.refined_surface_obstacles(rejected, 1.0, -0.2, 0.15)
        self.assertEqual(len(boxes), 1)          # 3x2 实格合成一个矩形
        self.assertEqual(boxes[0]['max'][0], 2.0)
        self.assertEqual(boxes[0]['max'][1], 3.0)  # 不含 y=3..4 的空角
        self.assertEqual(boxes[0]['min'][2], -0.2)

    def test_horizontal_filter_rejects_vertical_wall(self):
        points = []
        for x in np.arange(-0.5, 0.51, 0.03):
            for y in np.arange(-0.5, 0.51, 0.03):
                points.append((x, y, 0.0))
        for y in np.arange(-0.5, 0.51, 0.03):
            for z in np.arange(0.05, 0.6, 0.03):
                points.append((0.7, y, z))
        kept, info = mapping.filter_horizontal(np.array(points), threshold=0.8)
        self.assertGreater(len(kept), 100)
        self.assertLess(kept[:, 0].max(), 0.6)
        self.assertEqual(int(np.sum(np.abs(kept[:, 0] - 0.7) < 1e-9)), 0)


if __name__ == '__main__':
    unittest.main()
