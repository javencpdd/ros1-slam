"""V8-10 第三切片测试：三向偏差核验、时戳配准、越限→撤销全环、参考耗尽、守卫。"""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
sys.path.insert(0, str(HERE.parent / 'representation'))
sys.path.insert(0, str(HERE.parent / 'planner'))
import enforcement as enf  # noqa: E402
import support_map as rep  # noqa: E402
import support_planner as pln  # noqa: E402
import tracking as trk  # noqa: E402

SRC = {'uri': 'design://x', 'sha256': 'a' * 64}


def route_reference():
    value = rep.build_support_map(
        'trk_map', 0.5, [-5.0, -5.0, 0.0],
        [{'surface_id': 'ground', 'z_reference': 0.0, 'data_uri': 'g',
          'footprint': {'min_x': -5, 'min_y': -5, 'max_x': 5, 'max_y': 5},
          'edges': [{'to': 'mezz', 'kind': 'slope', 'max_slope_deg': 12.0,
                     'bidirectional': True}]},
         {'surface_id': 'mezz', 'z_reference': 0.6, 'data_uri': 'm',
          'footprint': {'min_x': 2, 'min_y': 1, 'max_x': 4, 'max_y': 3},
          'edges': [{'to': 'ground', 'kind': 'slope', 'max_slope_deg': 12.0,
                     'bidirectional': True}]}],
        None, SRC)
    route = pln.plan(value, (-4.0, -4.0, 0.0), (3.0, 2.0, 0.6))
    return [(x, y, z) for x, y, z, _sid in route['path']]


class TrackingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = route_reference()

    def test_on_route_passes(self):
        mid = self.reference[len(self.reference) // 2]
        self.assertEqual(trk.check_tracking(self.reference, mid), [])

    def test_lateral_deviation(self):
        mid = self.reference[len(self.reference) // 2]
        # 垂直于全局前进向（路径主向近似 (1,1)/√2）偏 0.3 m（>0.15）
        actual = (mid[0] + 0.3 * 0.7071, mid[1] - 0.3 * 0.7071, mid[2])
        violations = trk.check_tracking(self.reference, actual, tol_lateral=0.15)
        self.assertTrue(violations)
        self.assertEqual(violations[0][0], 'E_TRACK_DEVIATION')
        self.assertIn('横向', violations[0][2])

    def test_vertical_deviation(self):
        mid = self.reference[len(self.reference) // 2]
        violations = trk.check_tracking(self.reference, (mid[0], mid[1], mid[2] + 0.2))
        self.assertTrue(any('垂向' in v[2] for v in violations))

    def test_timestamp_matching(self):
        ref = [(x, y, z, t) for t, (x, y, z) in enumerate(self.reference)]
        mid = self.reference[len(ref) // 2]
        violations = trk.check_tracking(ref, mid, stamp=len(ref) // 2)
        self.assertEqual(violations, [])

    def test_deviation_triggers_revocation_loop(self):
        """跟踪闭环：偏差越限→on_rejection→Tracker 达限停。"""
        mid = self.reference[len(self.reference) // 2]
        off = (mid[0] + 0.5, mid[1], mid[2])
        violations = trk.check_tracking(self.reference, off)
        verdict = {'ok': not violations, 'violations': violations}
        action = enf.on_rejection(verdict)
        self.assertEqual(action['action'], 'stop_chain')
        tracker = enf.RejectionTracker(limit=2)
        self.assertEqual(tracker.record(verdict), 'replan')
        self.assertEqual(tracker.record(verdict), 'stop')

    def test_overrun(self):
        self.assertEqual(trk.overrun(self.reference, len(self.reference)),
                         [('E_TRACK_OVERRUN', len(self.reference) - 1, '参考已耗尽仍收到位姿')])
        self.assertEqual(trk.overrun(self.reference, 0), [])

    def test_guard(self):
        with self.assertRaises(trk.TrackingError) as caught:
            trk.check_tracking([], (0, 0, 0))
        self.assertEqual(caught.exception.code, 'E_TRACK_EMPTY_REFERENCE')


if __name__ == '__main__':
    unittest.main(verbosity=2)
