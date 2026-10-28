"""A4 运行面 cell 测试（宿主无 ROS）：判分口径、运行面一致性对照、表结构。"""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'scenarios'))
sys.path.insert(0, str(HERE.parent / 'planner'))
sys.path.insert(0, str(HERE.parent / 'calibration'))
import a4_runtime as a4  # noqa: E402
import scenarios as scn  # noqa: E402
import support_planner as pln  # noqa: E402


def design_route(case):
    planned = pln.plan(case['map'], case['start'], case['goal'], budget=20000)
    return [(x, y, z) for x, y, z, _sid in planned['path']]


class A4RuntimeTests(unittest.TestCase):
    def test_planning_ids_cover_sc01_to_sc10(self):
        self.assertEqual(a4.planning_ids(),
                         ['SC01', 'SC02', 'SC03', 'SC04', 'SC05',
                          'SC06', 'SC07', 'SC08', 'SC09', 'SC10'])

    def test_judge_positive_and_negative(self):
        positive = scn.scenario('SC01')
        verdict = a4.judge(positive, design_route(positive))
        self.assertEqual(verdict['outcome'], 'pass')
        no_route = scn.scenario('SC07')
        verdict = a4.judge(no_route, None, 'E_PLAN_NO_ROUTE')
        self.assertEqual(verdict['outcome'], 'expected_failure')
        verdict = a4.judge(no_route, None, 'E_PLAN_BUDGET_EXCEEDED')
        self.assertEqual(verdict['outcome'], 'unexpected_failure')
        low_roof = scn.scenario('SC09')
        verdict = a4.judge(low_roof, design_route(low_roof))
        self.assertEqual(verdict['outcome'], 'expected_failure')
        self.assertIn('E_TRAJ_CLEARANCE', verdict['reason'])

    def test_same_points(self):
        route = [(0.0, 0.0, 0.0), (1.0, 2.0, 0.0)]
        self.assertTrue(a4.same_points(route, list(route)))
        self.assertFalse(a4.same_points(route, route[:1]))
        self.assertFalse(a4.same_points(route, [(0.0, 0.0, 0.0), (1.0, 2.0, 0.001)]))
        self.assertFalse(a4.same_points(None, route))

    def test_run_with_design_backend_is_green_and_consistent(self):
        """注入「设计层即运行时」的调用：全 cell 按预期、且运行面路径与参考解逐点一致。"""
        def fake_call(map_value, start, goal, budget, name):
            try:
                planned = pln.plan(map_value, start, goal, budget=budget)
            except pln.PlannerError as error:
                return None, error.code, {'ok': False, 'reason': error.code}
            route = [(x, y, z) for x, y, z, _sid in planned['path']]
            return route, None, {'ok': True, 'expansions': planned['expansions']}

        cells = a4.run(call_node_fn=fake_call)
        s = a4.summary(cells)
        self.assertEqual(s['cells'], 20)
        self.assertEqual(s['unexpected_failures'], 0)
        self.assertEqual(s['passed'], 7 * 2)          # SC01—06、SC08 双轨
        self.assertEqual(s['expected_failures'], 3 * 2)  # SC07 无路、SC09 净空、SC10 预算
        self.assertEqual(s['same_as_design'], 20)


if __name__ == '__main__':
    unittest.main()
