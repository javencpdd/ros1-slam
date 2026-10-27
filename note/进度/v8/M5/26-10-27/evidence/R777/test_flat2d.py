"""flat2d 定标集件测试：几何、预检、世界模型一致性。宿主可跑，无 ROS 依赖。"""
import json
import pathlib
import sys
import unittest
import xml.etree.ElementTree as ET

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import geometry as geo  # noqa: E402
import validate_goals as vg  # noqa: E402


class GeometryTests(unittest.TestCase):
    def test_box_dist_inside_is_zero(self):
        self.assertEqual(geo.box_dist(0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0), 0.0)

    def test_box_dist_axis_aligned(self):
        self.assertAlmostEqual(geo.box_dist(1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0), 0.5)
        self.assertAlmostEqual(geo.box_dist(0.0, 2.0, 0.0, 0.0, 1.0, 1.0, 0.0), 1.5)

    def test_box_dist_rotated_matches_axis_aligned(self):
        # 45° 旋转后点 (0,0.75) 的盒面距离 = 0.75/√2 - 0.5（Chebyshev 口径）
        d = geo.box_dist(0.0, 0.75, 0.0, 0.0, 1.0, 1.0, 3.141592653589793 / 4)
        self.assertAlmostEqual(d, 0.75 / 2 ** 0.5 - 0.5, places=6)

    def test_min_clearance_empty(self):
        self.assertIsNone(geo.min_clearance([], [(0, 0, 1, 1, 0)]))

    def test_min_clearance_picks_nearest_box(self):
        boxes = [(0.0, 0.0, 1.0, 1.0, 0.0), (3.0, 0.0, 1.0, 1.0, 0.0)]
        self.assertAlmostEqual(geo.min_clearance([(2.0, 0.0)], boxes), 0.5)

    def test_sdf_box_is_valid_xml_with_name_and_size(self):
        sdf = geo.sdf_box('sc09_block', (0.8, 0.8, 0.8), (4.0, 0.0, 0.0), 0.4)
        root = ET.fromstring(sdf)
        model = root.find('model')
        self.assertEqual(model.get('name'), 'sc09_block')
        self.assertEqual(model.find('static').text, 'true')
        size = model.find('link/collision/geometry/box/size').text
        self.assertEqual(size, '0.80 0.80 0.80')


class WorldModelTests(unittest.TestCase):
    def setUp(self):
        self.model = json.loads((HERE / 'world_model.json').read_text())
        self.world = vg.World(self.model)

    def test_world_boxes_are_eight(self):
        self.assertEqual(len(self.world.boxes), 8)

    def test_start_is_free(self):
        self.assertGreater(self.world.clear(*self.model['start']), 1.0)

    def test_sc01_goal_passes_precheck(self):
        report = vg.check_goals(self.model, {'scenarios': [
            {'id': 'SC01', 'goals': [[3.0, 0.0]]}]})
        self.assertTrue(report['all_ok'])

    def test_v1_sc02_g1_fails_geometric_precheck(self):
        # v1 实测净空 0.148 的失败目标，几何预检也判不达标（0.395 瓶颈 < 0.55）
        report = vg.check_goals(self.model, {'scenarios': [
            {'id': 'SC02', 'goals': [[5.5, -1.8]]}]})
        self.assertFalse(report['all_ok'])
        self.assertLess(report['rows'][0]['path_bottleneck_m'], vg.PATH_BOTTLENECK_MIN_M)

    def test_precheck_cannot_predict_planner_stuck(self):
        # v1 的 (4.0,-1.8) 几何全过却卡死：预检只是必要条件，不是充分条件
        report = vg.check_goals(self.model, {'scenarios': [
            {'id': 'SC02', 'goals': [[4.0, -1.8]]}]})
        self.assertTrue(report['all_ok'])


if __name__ == '__main__':
    unittest.main()
