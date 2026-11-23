"""perception → local 的障碍框契约：三轴必须严格 lo < hi。

2026-11-24 实测：薄墙在 3 位小数舍入后某一轴厚度为 0，local 的 `_obstacles()`
整帧抛 `E_LOCAL_INPUT`，表现为 `E_LOCAL_OBSTACLES_STALE` + 永久零速。
"""
import sys
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[6]
FRAMEWORK = REPO / 'ws/src/90_platform/v8_platform/framework'
sys.path.insert(0, str(FRAMEWORK / 'nodes'))

import dynamic_obstacles_node as node   # noqa: E402


class BoxContractTest(unittest.TestCase):

    def test_degenerate_axis_is_inflated(self):
        lo, hi = node.box_minmax([1.34, -0.724, 0.172], [1.34, -0.089, 0.632])
        self.assertLess(lo[0], hi[0])
        for a, b in zip(lo, hi):
            self.assertLess(a, b)

    def test_center_is_preserved(self):
        lo, hi = node.box_minmax([2.0, 0.0, 0.0], [2.0, 1.0, 1.0])
        self.assertAlmostEqual((lo[0] + hi[0]) / 2.0, 2.0, places=3)

    def test_healthy_box_untouched(self):
        lo, hi = node.box_minmax([0.0, 0.0, 0.0], [1.0, 2.0, 3.0])
        self.assertEqual((lo, hi), ([0.0, 0.0, 0.0], [1.0, 2.0, 3.0]))

    def test_local_accepts_result(self):
        sys.path.insert(0, str(REPO / 'ws/src/80_algorithm/v8_algorithm/local_planner'))
        import local_native
        lo, hi = node.box_minmax([1.34, -0.724, 0.172], [1.34, -0.089, 0.632])
        out = local_native._obstacles([{'min': lo, 'max': hi, 'velocity': [0, 0, 0],
                                        'stamp': 1.0, 'active_from': 0.0,
                                        'active_until': 2.0}])
        self.assertEqual(len(out), 1)


if __name__ == '__main__':
    unittest.main()
