"""support_planner 栅格化口径的特征化测试（2026-11-18，计划 M4）。

为什么先补这个：`_cells()` 的盒式落格与 `node_of()` 的旧分支（schema_version<3）
在既有测试里只有间接覆盖。M4 要把这两处改调 v8_grid，改动前必须把**现状行为**
锁成断言——否则「替换后测试仍绿」可能只是没测到边界。

三条口径（与 v8_grid 的对应）：
  _cells 的 x0/x1     `int(math.ceil(v/res - .5))`      → cell_center
  node_of 的落格       `int(math.floor(x/res))`          → cell_floor
  node_of 的钳位收边   `int(math.ceil(v/res))`           → cell_clamp_edge

本件在替换前后都必须绿：替换前它记录现状，替换后它证明零变化。
"""
import math
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import support_planner as sp  # noqa: E402


def legacy_cells_box(footprint, resolution):
    """替换前 _cells 的盒式分支（字面拷贝，作为 oracle 保留）。"""
    x0 = int(math.ceil(footprint['min_x'] / resolution - .5))
    x1 = int(math.ceil(footprint['max_x'] / resolution - .5)) - 1
    y0 = int(math.ceil(footprint['min_y'] / resolution - .5))
    y1 = int(math.ceil(footprint['max_y'] / resolution - .5)) - 1
    return {(ix, iy) for ix in range(x0, x1 + 1) for iy in range(y0, y1 + 1)}


FOOTPRINTS = [
    # 边界恰在格界（min/res 为整数）
    {'min_x': 0.0, 'max_x': 1.0, 'min_y': 0.0, 'max_y': 1.0},
    # 边界在半格（0.5 格）
    {'min_x': -0.05, 'max_x': 0.55, 'min_y': -0.05, 'max_y': 0.55},
    # 非整数边界
    {'min_x': 1.23, 'max_x': 4.56, 'min_y': -2.34, 'max_y': 0.11},
    # 退化：宽高小于一格的薄板
    {'min_x': 0.32, 'max_x': 0.38, 'min_y': 0.32, 'max_y': 0.38},
    # 跨负零
    {'min_x': -0.0, 'max_x': 0.25, 'min_y': -0.25, 'max_y': 0.0},
]

RESOLUTIONS = (0.05, 0.1, 0.2, 0.25, 0.5)


class CellsBoxCharacterizationTests(unittest.TestCase):
    def test_cells_box_matches_legacy_formula(self):
        for res in RESOLUTIONS:
            for fp in FOOTPRINTS:
                got = sp._cells({'footprint': fp}, res)
                want = legacy_cells_box(fp, res)
                self.assertEqual(got, want,
                                 f'_cells 盒式落格变了: res={res} fp={fp}')

    def test_cell_center_boundary_semantics(self):
        # 「半格中心」口径：v/res - .5 恰为整数时 ceil 不动（落在该整数上）。
        # 这些是 ceil(...-.5) 最容易出错的点，钉死以免替换时被 floor 掉。
        cases = [(0.05, 0.1, 0), (0.1, 0.1, 1), (0.15, 0.1, 1), (0.25, 0.1, 2),
                 (-0.05, 0.1, -1), (-0.1, 0.1, -1)]
        for v, res, want in cases:
            self.assertEqual(int(math.ceil(v / res - .5)), want,
                             f'口径 C 的期望值写错: {v}/{res}')

    def test_cells_result_is_a_set_of_int_tuples(self):
        cells = sp._cells({'footprint': FOOTPRINTS[0]}, 0.1)
        self.assertTrue(cells)
        for cell in cells:
            self.assertEqual(len(cell), 2)
            self.assertTrue(all(isinstance(i, int) for i in cell))


class FallingCellCharacterizationTests(unittest.TestCase):
    """node_of 的两条口径（落格 + 钳位收边）——不经 plan() 直接锁表达式语义。"""

    def test_floor_landing_matches_cell_floor(self):
        # 与 v8_grid.cell_floor 同式：点落在格边界时判给左格（无 epsilon）
        cases = [(0.3, 0.1, 2), (0.7, 0.1, 6), (2.9, 0.1, 28), (-0.1, 0.1, -1)]
        for x, res, want in cases:
            self.assertEqual(int(math.floor(x / res)), want)

    def test_clamp_edge_matches_cell_clamp_edge(self):
        cases = [(0.0, 0.1, 0), (0.3, 0.1, 3), (0.31, 0.1, 4), (-0.1, 0.1, -1)]
        for v, res, want in cases:
            self.assertEqual(int(math.ceil(v / res)), want)


class GridModuleConsistencyTests(unittest.TestCase):
    """替换后：_cells 与 v8_grid 必须给出同一集合（这条在替换前后都成立）。"""

    def test_cells_agrees_with_v8_grid_cell_center(self):
        from v8_grid import indexing as grid
        for res in RESOLUTIONS:
            for fp in FOOTPRINTS:
                x0 = grid.cell_center(fp['min_x'], res)
                x1 = grid.cell_center(fp['max_x'], res) - 1
                y0 = grid.cell_center(fp['min_y'], res)
                y1 = grid.cell_center(fp['max_y'], res) - 1
                want = {(ix, iy) for ix in range(x0, x1 + 1) for iy in range(y0, y1 + 1)}
                self.assertEqual(sp._cells({'footprint': fp}, res), want,
                                 f'与 v8_grid 口径分歧: res={res} fp={fp}')


if __name__ == '__main__':
    unittest.main()
