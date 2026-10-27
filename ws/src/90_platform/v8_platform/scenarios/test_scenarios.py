"""V8-11 首切片测试：三场景全链（构造→任务配对→规划→评价）、真值判定面、
A7 结构面（评价器不依赖被测链内部）、槽位守卫。"""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
sys.path.insert(0, str(HERE.parent / 'planner'))
import scenarios as scn  # noqa: E402
import support_planner as pln  # noqa: E402
import validate as contract  # noqa: E402


class ScenarioTests(unittest.TestCase):
    def _full_chain(self, name):
        case = scn.scenario(name)
        contract.validate('map', case['map'])
        contract.validate('task', case['task'])
        contract.check_pair(case['task'], case['map'])
        route = pln.plan(case['map'], case['start'], case['goal'])
        trajectory = [(x, y, z) for x, y, z, _sid in route['path']]
        return case, scn.evaluate(case, trajectory)

    def test_sc01_flat(self):
        case, verdict = self._full_chain('SC01')
        self.assertTrue(verdict['ok'], verdict)
        self.assertEqual(case['truth']['must_surfaces'], ['ground'])

    def test_sc02_ramp(self):
        case, verdict = self._full_chain('SC02')
        self.assertTrue(verdict['ok'], verdict)
        self.assertEqual(case['truth']['must_surfaces'], ['ground', 'mezz'])

    def test_sc03_transition_return(self):
        case, verdict = self._full_chain('SC03')
        self.assertTrue(verdict['ok'], verdict)

    def test_evaluator_rejects_hover(self):
        """评价器负控：悬空轨迹 FAIL 且不修复（错误路径不被载体自动修正掩盖）。"""
        case = scn.scenario('SC02')
        hover = [(-4.0, -4.0, 0.0), (-2.0, -4.0, 0.55), (3.0, 2.0, 0.6)]
        verdict = scn.evaluate(case, hover)
        self.assertFalse(verdict['ok'])
        self.assertIn('checker:E_TRAJ_UNSUPPORTED', verdict['failures'])
        self.assertTrue(verdict['check_violations'])  # 违例如实带出，非静默

    def test_evaluator_rejects_wrong_goal(self):
        case = scn.scenario('SC01')
        route = pln.plan(case['map'], case['start'], (0.0, 0.0, 0.0))
        short = [(x, y, z) for x, y, z, _sid in route['path']]
        verdict = scn.evaluate(case, short)
        self.assertFalse(verdict['ok'])
        self.assertIn('goal_not_reached', verdict['failures'])

    def test_all_constructors_and_kinds(self):
        """R661 后全 14 场景有构造器；kind 五类分布正确。"""
        kinds = {}
        for sid, spec in scn.SCENARIOS.items():
            case = scn.scenario(sid)  # 构造不抛即通过
            kinds[case['kind']] = kinds.get(case['kind'], 0) + 1
        self.assertEqual(len(scn.SCENARIOS), 14)
        self.assertEqual(kinds, {'planning': 10, 'tracking': 1,
                                 'orchestration': 1, 'versioning': 1, 'matrix': 1})

    def test_a7_no_leakage_surface(self):
        """A7 结构面：评价器源不 import 被测规划链（数据不回流的结构保证）。"""
        source = Path(scn.__file__).read_text()
        self.assertNotIn('import support_planner', source.split('def evaluate')[1])
        # 场景构造可以起真值路线（truth 不泄漏给被测链——evaluate 只收输出）


if __name__ == '__main__':
    unittest.main(verbosity=2)
