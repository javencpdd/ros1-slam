"""flat2d 定标集件测试：几何、预检、世界模型一致性。宿主可跑，无 ROS 依赖。"""
import json
import pathlib
import sys
import unittest
import xml.etree.ElementTree as ET

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import cp  # noqa: E402
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


class PreRegisterInjectTests(unittest.TestCase):
    def test_pre_register_writes_inject_per_group(self):
        import subprocess
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            td = pathlib.Path(td)
            setp = td / 'set.json'
            setp.write_text(json.dumps({
                'set_id': 't', 'world': 'w', 'map': 'm', 'combo': 'c',
                'start': [0.0, 0.0, 0.0], 'repetitions': 1, 'config_lock': {},
                'scenarios': [{
                    'id': 'SC09', 'name': 'n', 'expected_outcome': 'reach',
                    'goals': [[1.0, 0.0], [2.0, 0.0]],
                    'injects': [
                        {'at_progress_m': 0.5, 'model_name': 'b0',
                         'size': [0.8, 0.8, 0.8], 'pose_xy_yaw': [1.0, 0.0, 0.0]},
                        {'at_progress_m': 0.5, 'model_name': 'b1',
                         'size': [0.8, 0.8, 0.8], 'pose_xy_yaw': [2.0, 0.0, 0.0]},
                    ]}]}))
            out = td / 'runs'
            subprocess.run([sys.executable, str(HERE / 'pre_register.py'),
                            str(setp), str(out)], check=True, capture_output=True)
            m0 = json.loads((out / 't_SC09_g0_r1.json').read_text())
            m1 = json.loads((out / 't_SC09_g1_r1.json').read_text())
            self.assertEqual(m0['inject']['model_name'], 'b0')
            self.assertEqual(m1['inject']['pose_xy_yaw'], [2.0, 0.0, 0.0])

    def test_sc09_set_has_four_groups_and_three_reps(self):
        s = json.loads((HERE / 'scenarios_v3_sc09.json').read_text())
        sc = s['scenarios'][0]
        self.assertEqual(len(sc['goals']), 4)
        self.assertEqual(len(sc['injects']), 4)
        self.assertEqual(s['repetitions'], 3)


class ClopperPearsonTests(unittest.TestCase):
    def test_all_success_24(self):
        self.assertAlmostEqual(cp.cp_lower(24, 24), 0.05 ** (1 / 24), places=4)

    def test_all_success_36(self):
        self.assertAlmostEqual(cp.cp_lower(36, 36), 0.9202, places=3)

    def test_partial_is_lower(self):
        self.assertLess(cp.cp_lower(20, 24), cp.cp_lower(24, 24))
        self.assertGreater(cp.cp_lower(20, 24), 0.6)

    def test_zero_success_is_zero(self):
        self.assertEqual(cp.cp_lower(0, 24), 0.0)


if __name__ == '__main__':
    unittest.main()
