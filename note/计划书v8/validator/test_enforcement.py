"""V8-10 次切片测试：运动学三上限＋铁则拒判（declared+prohibited）＋
失效撤销（on_rejection→V8-06 停止链联动、持续被拒强制停）。"""
import copy
import json
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
sys.path.insert(0, str(HERE.parent / 'representation'))
sys.path.insert(0, str(HERE.parent / 'planner'))
sys.path.insert(0, str(HERE.parent / 'orchestrator'))
import enforcement as enf  # noqa: E402
import orchestrator as orch  # noqa: E402
import support_map as rep  # noqa: E402
import support_planner as pln  # noqa: E402
import trajectory_checker as chk  # noqa: E402

ROBOT = json.loads((HERE.parent / 'contracts' / 'fixtures' / 'robot_lite3_design.json').read_text())
SRC = {'uri': 'design://x', 'sha256': 'a' * 64}


def tested_robot():
    """把三处能力改为 tested（带 report）——判定可用面。"""
    robot = copy.deepcopy(ROBOT)
    for name in ('max_slope_deg', 'max_step_height_m', 'min_clearance_m'):
        robot['capability'][name].update(level='tested', report='reports/kin.json')
    robot['brake'].update(level='tested', report='reports/brake.json')
    return robot


def floors():
    return rep.build_support_map(
        'enf_map', 0.5, [-5.0, -5.0, 0.0],
        [{'surface_id': 'ground', 'z_reference': 0.0, 'data_uri': 'g',
          'footprint': {'min_x': -5, 'min_y': -5, 'max_x': 5, 'max_y': 5},
          'edges': [{'to': 'mezz', 'kind': 'slope', 'max_slope_deg': 12.0,
                     'bidirectional': True}]},
         {'surface_id': 'mezz', 'z_reference': 0.6, 'data_uri': 'm',
          'footprint': {'min_x': 2, 'min_y': 1, 'max_x': 4, 'max_y': 3},
          'edges': [{'to': 'ground', 'kind': 'slope', 'max_slope_deg': 12.0,
                     'bidirectional': True}]}],
        None, SRC)


class KinematicsTests(unittest.TestCase):
    def test_within_limits_pass(self):
        traj = [(-2.0, 0.0, 0.0, 0.0), (-1.5, 0.0, 0.0, 2.0), (-1.0, 0.0, 0.0, 4.0)]
        self.assertEqual(enf.check_kinematics(traj, tested_robot()), [])

    def test_step_violation(self):
        """robot max_step=0.08：0.6 m 直上跨面步（即使地图边合法）违机器人能力。"""
        traj = [(1.5, 0.0, 0.0, 0.0), (2.0, 0.0, 0.6, 4.0)]
        codes = {v[0] for v in enf.check_kinematics(traj, tested_robot())}
        self.assertIn('E_TRAJ_STEP', codes)  # 0.6 > 0.08（坡度项同违）

    def test_slope_violation(self):
        traj = [(0.0, 0.0, 0.0, 0.0), (1.0, 0.0, 0.1, 2.0)]  # 坡角≈5.7°>15? 否——构造更陡
        traj = [(0.0, 0.0, 0.0, 0.0), (0.2, 0.0, 0.1, 2.0)]  # 26.6°>15°
        codes = {v[0] for v in enf.check_kinematics(traj, tested_robot())}
        self.assertIn('E_TRAJ_SLOPE', codes)

    def test_decel_violation(self):
        """减速 1.5→0.1 m/s 于 0.4 s＝减速度 3.5 > brake 1.0。"""
        traj = [(0.0, 0.0, 0.0, 0.0), (0.6, 0.0, 0.0, 0.4), (0.64, 0.0, 0.0, 0.8)]
        codes = {v[0] for v in enf.check_kinematics(traj, tested_robot())}
        self.assertIn('E_TRAJ_DECEL', codes)

    def test_iron_rule_declared_prohibited_refuses(self):
        """铁则：declared+prohibited 能力值不得用于判定——拒绝判定而非假绿。"""
        with self.assertRaises(enf.EnforcementError) as caught:
            enf.check_kinematics([(0, 0, 0, 0), (1, 0, 0, 1)], ROBOT)
        self.assertEqual(caught.exception.code, 'E_TRAJ_CAPABILITY_UNVERIFIED')

    def test_guards(self):
        with self.assertRaises(enf.EnforcementError) as caught:
            enf.check_kinematics([(0, 0, 0, 0)], tested_robot())
        self.assertEqual(caught.exception.code, 'E_ENF_TRAJ_TOO_SHORT')
        with self.assertRaises(enf.EnforcementError) as caught:
            enf.check_kinematics([(0, 0, 0), (1, 0, 0, 1)], tested_robot())
        self.assertEqual(caught.exception.code, 'E_ENF_NEED_TIMESTAMPS')
        bad = tested_robot()
        bad['robot_id'] = '!!'
        with self.assertRaises(enf.EnforcementError) as caught:
            enf.check_kinematics([(0, 0, 0, 0), (1, 0, 0, 1)], bad)
        self.assertEqual(caught.exception.code, 'E_ENF_ROBOT_INVALID')


class RevocationTests(unittest.TestCase):
    def test_on_rejection_maps_to_stop_chain(self):
        verdict = chk.check(floors(), [(0.29, 0.0, 0.598)])  # 悬空否决
        action = enf.on_rejection(verdict)
        self.assertEqual(action['action'], 'stop_chain')
        self.assertEqual(action['task_outcome'], 'FAILED')
        self.assertIn('E_TRAJ_UNSUPPORTED', action['reason'])
        with self.assertRaises(enf.EnforcementError):
            enf.on_rejection({'ok': True, 'violations': []})

    def test_rejection_tracker_forces_stop(self):
        """A5：连续被拒达限→强制停，不无限重规划；通过即复位。"""
        tracker = enf.RejectionTracker(limit=3)
        rejected = {'ok': False, 'violations': [('E_TRAJ_UNSUPPORTED', 0, 'x')]}
        self.assertEqual(tracker.record(rejected), 'replan')
        self.assertEqual(tracker.record(rejected), 'replan')
        self.assertEqual(tracker.record(rejected), 'stop')  # 第 3 次
        self.assertEqual(tracker.record({'ok': True, 'violations': []}), 'accept')
        self.assertEqual(tracker.record(rejected), 'replan')  # 复位后重新计

    def test_full_loop_rejection_stops_orchestrator(self):
        """编排闭环：检查器否决→on_rejection→V8-06 停止链→任务 FAILED 且退净。"""
        value = floors()
        hover = [(-4.0, -4.0, 0.0), (-3.0, -4.0, 0.55), (-2.0, -4.0, 0.0)]
        verdict = chk.check(value, hover)
        self.assertFalse(verdict['ok'])
        action = enf.on_rejection(verdict)
        self.assertEqual(action['action'], 'stop_chain')
        specs = [orch.ComponentSpec('scan', ['local_planner']),
                 orch.ComponentSpec('mux', ['safety_arbiter'], stop_method='process_group_signal')]
        engine = orch.Orchestrator()
        task = engine.submit('patrol', specs)
        runners = {s.id: orch.SimulatedRunner(s) for s in specs}
        engine.start(task, runners)
        engine.cancel(task, runners)  # 停止链联动（V8-06 接口）
        self.assertEqual(task.state, orch.TaskState.CANCELLED)
        self.assertTrue(all(not r.is_alive() for r in runners.values()))
        # 对照：合法规划器轨迹通过检查器且 Tracker 接受
        route = pln.plan(value, (-4.0, -4.0, 0.0), (3.0, 2.0, 0.6))
        ok_verdict = chk.check(value, [(x, y, z) for x, y, z, _ in route['path']])
        self.assertEqual(enf.RejectionTracker().record(ok_verdict), 'accept')


if __name__ == '__main__':
    unittest.main(verbosity=2)
