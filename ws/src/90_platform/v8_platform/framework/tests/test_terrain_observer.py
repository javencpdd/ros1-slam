"""TerrainObserver 影子通道单测（纯 Python，无 ROS）。

覆盖：GridMap 列主序还原口径、F10 包络同量判别、空集拒判、
墙高注入有效性、撤值反控、影子对照只报不拦。
"""
import sys as _sys
from pathlib import Path as _Path

_BASE = _Path(__file__).resolve().parent.parent
for _entry in ('', 'nodes', 'tools', 'smokes'):
    _sys.path.insert(0, str(_BASE / _entry) if _entry else str(_BASE))
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(_BASE))
import terrain_observer as obs  # noqa: E402

N_ROWS, N_COLS, RES = 9, 20, 0.05
PATH = [(4, c) for c in range(N_COLS)]


def flat_matrix(height=0.0):
    return [[height for _ in range(N_COLS)] for _ in range(N_ROWS)]


class TerrainObserverTests(unittest.TestCase):
    def test_column_major_roundtrip(self):
        flat = [float(c * N_ROWS + r) for c in range(N_COLS)
                for r in range(N_ROWS)]
        m = obs.gridmap_to_matrix(flat, N_ROWS, N_COLS)
        for r in range(N_ROWS):
            for c in range(N_COLS):
                self.assertEqual(m[r][c], float(c * N_ROWS + r))
        with self.assertRaises(ValueError):
            obs.gridmap_to_matrix(flat[:-1], N_ROWS, N_COLS)

    def test_flat_ground_is_none(self):
        r = obs.observe(flat_matrix(), PATH, resolution=RES)
        self.assertEqual(r['discontinuity_type'], 'none')
        self.assertFalse(r['ahead'])
        self.assertEqual(r['valid_cells'], 20)
        # clearance 是横向连续有效格数，不是纵向行数；路径起点 col=0
        # 向左越界即停、向右最多 half_width=5 格 => 6 格。
        self.assertAlmostEqual(r['clearance_m'], 6 * RES, places=4)

    def test_clearance_responds_to_half_width(self):
        wide = obs.observe(flat_matrix(), PATH, resolution=RES, half_width=8)
        base = obs.observe(flat_matrix(), PATH, resolution=RES, half_width=5)
        self.assertAlmostEqual(wide['clearance_m'], 9 * RES, places=4)
        self.assertGreater(wide['clearance_m'], base['clearance_m'])

    def test_empty_set_refused(self):
        nan = float('nan')
        r = obs.observe([[nan] * N_COLS for _ in range(N_ROWS)],
                        PATH, resolution=RES)
        self.assertEqual(r['discontinuity_type'], 'insufficient_data')
        self.assertFalse(r['ahead'])
        self.assertIsNone(r['clearance_m'])
        self.assertIn('valid_cells_below_min', r['reason'])

    def test_f10_step_vs_slope_same_envelope(self):
        # 两者总升高同为 0.10 m，一个竖直阶面、一个连续梯度
        step = [[(0.10 if c >= 10 else 0.0) for c in range(N_COLS)]
                for _ in range(N_ROWS)]
        slope = [[0.10 * c / (N_COLS - 1) for c in range(N_COLS)]
                 for _ in range(N_ROWS)]
        rs = obs.observe(step, PATH, resolution=RES)
        rp = obs.observe(slope, PATH, resolution=RES)
        self.assertEqual(rs['discontinuity_type'], 'vertical_step')
        self.assertTrue(rs['ahead'])
        self.assertAlmostEqual(rs['step_m'], 0.10, places=4)
        self.assertEqual(rp['discontinuity_type'], 'continuous_slope')
        self.assertFalse(rp['ahead'])
        self.assertIsNotNone(rp['slope_deg'])
        self.assertAlmostEqual(math.degrees(
            math.atan2(0.10, 19 * RES)), rp['slope_deg'], places=2)

    def test_injection_wall_becomes_blocked(self):
        wall = [[(0.45 if c >= 10 else 0.0) for c in range(N_COLS)]
                for _ in range(N_ROWS)]
        r = obs.observe(wall, PATH, resolution=RES, max_step_m=0.15)
        self.assertEqual(r['discontinuity_type'], 'blocked')
        self.assertIn('step_exceeds_max_step_m', r['reason'])

    def test_reverse_control(self):
        step = [[(0.10 if c >= 10 else 0.0) for c in range(N_COLS)]
                for _ in range(N_ROWS)]
        self.assertEqual(obs.observe(step, PATH, resolution=RES)
                         ['discontinuity_type'], 'vertical_step')
        for row in step:
            for c in range(10, N_COLS):
                row[c] = 0.45
        self.assertEqual(obs.observe(step, PATH, resolution=RES)
                         ['discontinuity_type'], 'blocked')
        for row in step:
            for c in range(10, N_COLS):
                row[c] = 0.10
        self.assertEqual(obs.observe(step, PATH, resolution=RES)
                         ['discontinuity_type'], 'vertical_step')

    def test_shadow_compare_never_writes_gait(self):
        declared = {'ahead': True, 'step_m': 0.10, 'distance_m': 0.5}
        step = [[(0.10 if c >= 10 else 0.0) for c in range(N_COLS)]
                for _ in range(N_ROWS)]
        slope = [[0.10 * c / (N_COLS - 1) for c in range(N_COLS)]
                 for _ in range(N_ROWS)]
        ok = obs.shadow_compare(declared, obs.observe(step, PATH, resolution=RES))
        bad = obs.shadow_compare(declared, obs.observe(slope, PATH, resolution=RES))
        self.assertTrue(ok['consistent'])
        self.assertIn('declared_step_but_terrain_is_slope', bad['disagreements'])
        self.assertIn('never writes', bad['note'])

    def test_world_to_cell_center(self):
        cell = obs.world_to_cell(0.0, 0.0, 0.0, 0.0, N_ROWS, N_COLS, RES)
        self.assertIsNotNone(cell)
        self.assertIsNone(obs.world_to_cell(100.0, 0.0, 0.0, 0.0,
                                            N_ROWS, N_COLS, RES))


if __name__ == '__main__':
    unittest.main(verbosity=2)
