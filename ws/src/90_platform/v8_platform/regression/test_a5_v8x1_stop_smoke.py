"""Pure tests for the ROS/MuJoCo full-chain stop smoke; no ROS or physics."""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'planner'))
sys.path.insert(0, str(HERE.parent / 'local_planner'))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import a5_v8x1_stop_smoke as smoke  # noqa: E402


def samples(points):
    return [{'sim_time': t, 'world_xyz': [x, y, .3], 'base_vxy': [vx, 0.]}
            for t, x, y, vx in points]


class StopMeasureTests(unittest.TestCase):
    def test_stationary_window_completes(self):
        trace = samples([(0., 0., 0., .3), (.2, .1, 0., .3), (.7, .1, 0., 0.),
                         (1.2, .1, 0., 0.), (1.7, .1, 0., 0.), (2.2, .1, 0., 0.)])
        result = smoke.measure(trace, .2, hold_s=.5, speed_limit=.2, xy_limit=.05)
        self.assertTrue(result['completed'])
        self.assertEqual(result['status'], 'PASS')
        self.assertAlmostEqual(result['stop_time_s'], 1.0)
        self.assertAlmostEqual(result['peak_forward_m'], 0.)
        self.assertAlmostEqual(result['path_m'], 0.)

    def test_forward_progress_is_measured(self):
        trace = samples([(0., 0., 0., .3), (.2, .1, 0., .3), (.4, .2, 0., .1),
                         (.7, .22, 0., 0.), (1.2, .22, 0., 0.), (1.7, .22, 0., 0.)])
        result = smoke.measure(trace, .2, hold_s=.5, speed_limit=.2, xy_limit=.05)
        self.assertTrue(result['completed'])
        self.assertAlmostEqual(result['peak_forward_m'], .12, places=6)
        self.assertAlmostEqual(result['path_m'], .12, places=6)

    def test_moving_or_missing_trigger_is_not_completed(self):
        moving = samples([(0., 0., 0., .3), (.5, .2, 0., .3),
                          (1., .4, 0., .3), (1.5, .6, 0., .3), (2., .8, 0., .3)])
        self.assertFalse(smoke.measure(moving, .2)['completed'])
        self.assertEqual(smoke.measure(moving, None)['status'], 'INVALID')
        self.assertEqual(smoke.measure(moving, 9.)['reason'], 'trigger_not_observed')

    def test_map_and_obstacle_contracts(self):
        value = smoke.map_value(1.)
        self.assertEqual(value['map_id'], 'mj_flat_stop_ros')
        self.assertEqual(value['surfaces'][0]['footprint']['min_y'], -3.)
        obstacle = smoke.obstacle()
        self.assertEqual(obstacle['min'], [-.4, .8, 0.])
        self.assertEqual(obstacle['max'], [.4, 1.2, .5])


if __name__ == '__main__':
    unittest.main()
