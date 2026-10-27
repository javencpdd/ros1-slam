"""V8-09 运行面适配层测试：不 import rospy，只测纯函数。"""
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ros_node
import support_planner
import test_support_planner as fixture


class RosNodeTests(unittest.TestCase):
    def setUp(self):
        self.map = fixture.floors()
        handle = tempfile.NamedTemporaryFile('w', suffix='.json', delete=False)
        json.dump(self.map, handle)
        handle.close()
        self.path = handle.name

    def tearDown(self):
        os.unlink(self.path)

    def test_request_shape_and_z_required(self):
        request = ros_node.to_request(self.path, [0, 0, 0], [2, 0, 0.6], 500, None)
        self.assertEqual(request['budget'], 500)
        with self.assertRaises(ValueError):
            ros_node.to_request(self.path, [0, 0, 0], [2, 0, None], 500, None)
        with self.assertRaises(ValueError):
            ros_node.to_request(self.path, [0, 0], [2, 0, 0], 500, None)
        with self.assertRaises(ValueError):
            ros_node.to_request(self.path, [0, 0, 0], [2, 0, 0], 0, None)

    def test_route_points_match_plan(self):
        request = ros_node.to_request(self.path, [0, 0, 0], [2, 0, 0.6], 2000, None)
        result = ros_node.plan_route(request)
        points = ros_node.to_path_points(result)
        self.assertEqual(len(points), len(result['path']))
        first = result['path'][0]
        expect = first['surface_id'] if isinstance(first, dict) else first[3]
        self.assertEqual(points[0][3], expect)
        self.assertTrue(all(isinstance(p[2], float) for p in points))

    def test_unknown_hint_refused(self):
        request = ros_node.to_request(self.path, [0, 0, 0], [2, 0, 0.6], 2000, 'attic')
        with self.assertRaises(support_planner.PlannerError) as caught:
            ros_node.plan_route(request)
        self.assertEqual(caught.exception.code, 'E_PLAN_GOAL_INVALID')

    def test_refusal_is_reportable(self):
        try:
            support_planner.plan(self.map, (0, 0, 0), (99, 99, 0.6), budget=50)
        except support_planner.PlannerError as error:
            info = ros_node.refusal(error)
        self.assertFalse(info['ok'])
        self.assertTrue(info['reason'].startswith('E_PLAN_'))
        self.assertTrue(info['detail'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
