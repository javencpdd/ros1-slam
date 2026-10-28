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
        def fake_call(map_value, start, goal, budget, name, **kwargs):
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

    def test_run_cancel_expects_cancelled_code(self):
        """A4 取消可区分：预取消必须返回 E_PLAN_CANCELLED；返回路径或其他拒绝都不合格。"""
        def cancelled(map_value, start, goal, budget, name, **kwargs):
            self.assertTrue(kwargs.get('cancel_first'))
            return None, 'E_PLAN_CANCELLED', {'ok': False, 'reason': 'E_PLAN_CANCELLED'}

        cells = a4.run_cancel(call_node_fn=cancelled)
        self.assertEqual(len(cells), 2)
        self.assertTrue(all(c['outcome'] == 'pass' for c in cells))

        def route_returned(map_value, start, goal, budget, name, **kwargs):
            return [(0.0, 0.0, 0.0)], None, {'ok': True}

        cells = a4.run_cancel(call_node_fn=route_returned)
        self.assertTrue(all(c['outcome'] == 'unexpected_failure' for c in cells))

        def wrong_refusal(map_value, start, goal, budget, name, **kwargs):
            return None, 'E_PLAN_NO_ROUTE', {'ok': False, 'reason': 'E_PLAN_NO_ROUTE'}

        cells = a4.run_cancel(call_node_fn=wrong_refusal)
        self.assertTrue(all(c['outcome'] == 'unexpected_failure' for c in cells))


    def test_run_version_tracks_versions_and_effect(self):
        """A3 版本追踪：版本号递增、v2 更新须真生效（路径变化）、v3 恢复与 v1 同形。"""
        good = [('v1', True, 'ok', 1, 16), ('v2', True, 'ok', 2, 9),
                ('v3', True, 'ok', 3, 16)]
        cells = a4.run_version(call_versions_fn=lambda: list(good))
        self.assertEqual([c['outcome'] for c in cells], ['pass', 'pass', 'pass'])
        no_effect = [('v1', True, 'ok', 1, 16), ('v2', True, 'ok', 2, 16),
                     ('v3', True, 'ok', 3, 16)]
        cells = a4.run_version(call_versions_fn=lambda: list(no_effect))
        self.assertEqual(cells[1]['outcome'], 'unexpected_failure')
        self.assertEqual(cells[1]['reason'], 'v2_no_effect')
        stale = [('v1', True, 'ok', 1, 16), ('v2', True, 'ok', 1, 9),
                 ('v3', True, 'ok', 3, 16)]
        cells = a4.run_version(call_versions_fn=lambda: list(stale))
        self.assertEqual(cells[1]['outcome'], 'unexpected_failure')
        not_restored = [('v1', True, 'ok', 1, 16), ('v2', True, 'ok', 2, 9),
                        ('v3', True, 'ok', 3, 11)]
        cells = a4.run_version(call_versions_fn=lambda: list(not_restored))
        self.assertEqual(cells[2]['outcome'], 'unexpected_failure')
        self.assertEqual(cells[2]['reason'], 'v3_not_restored')


if __name__ == '__main__':
    unittest.main()
