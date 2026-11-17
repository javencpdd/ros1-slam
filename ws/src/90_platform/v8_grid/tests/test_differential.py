"""差分测试：v8_grid 的七个口径 vs 历史表达式 oracle，逐值锁死。

★ 这是全案（计划 M1–M5）的核心门禁 ★
oracle 是各历史调用点表达式的**字面拷贝**（注明出处），永久保留——替换后
它们就是历史契约的化身；任何「顺手优化」新实现导致输出变化的提交都会在此变红。

对抗值集覆盖：
  - res ∈ {0.05, 0.1, 0.2, 0.25, 0.5}（0.05 是二进制不可精确表示的典型，0.25 精确）
  - k ∈ [-50, 50] 的 k*res 与 math.nextafter(k*res, ±inf) 双侧邻值
  - 命名用例：x=0.3, res=0.1（用户实证的 2 vs 3 分歧点）
  - 负零、|x| ≥ 1e7（大数边缘）
  - 固定种子 fuzz 1e5 点
断言逐值逐类型（int vs np.int64 各按原站点返回类型）。
"""
import math
import random
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from v8_grid import indexing as grid  # noqa: E402


# ───────────────────────── oracle（历史表达式字面拷贝，永久保留）─────────────────────────

def oracle_cell_floor(x, res):          # 冻结自 representation/elevation.py:38
    return int(x // res)


def oracle_cell_snap(x, res):           # 冻结自 representation/support_map.py:157
    return math.floor(x / res + 1e-9)


def oracle_cell_clamp_edge(v, res):     # 冻结自 planner/support_planner.py:269-272
    return int(math.ceil(v / res))


def oracle_cell_center(v, res):         # 冻结自 planner/support_planner.py:40-43
    return int(math.ceil(v / res - .5))


def oracle_image_axis(v, center, res, half):   # 冻结自 framework/terrain_observer.py:48
    return int(math.floor((v - center) / res + half))


def oracle_cells_floor_min(arr, min_v, res):   # 冻结自 representation/open_data_import.py:102
    return np.floor((arr - min_v) / res)


def oracle_cells_floor_origin(arr, x0, res):   # 冻结自 publisher:141-142
    return np.floor((arr - x0) / res)


# ───────────────────────────── 对抗值集 ─────────────────────────────

RESOLUTIONS = (0.05, 0.1, 0.2, 0.25, 0.5)


def adversarial_scalars():
    """标量对抗集：k*res 与其 nextafter 双侧邻值 + 边界命名用例。"""
    values = []
    for res in RESOLUTIONS:
        for k in range(-50, 51):
            base = k * res
            values.append(base)
            values.append(math.nextafter(base, -math.inf))
            values.append(math.nextafter(base, math.inf))
    values += [0.3, 0.7, 2.9, -0.1, 0.0, -0.0, 1e7, -1e7, 123456.7]
    return values


class DifferentialScalarTests(unittest.TestCase):
    """逐值逐类型断言（int 站点断 int，np 站点断 np.float）。"""

    def test_cell_floor_matches_elevation_oracle(self):
        for res in RESOLUTIONS:
            for x in adversarial_scalars():
                self.assertEqual(
                    grid.cell_floor(x, res), oracle_cell_floor(x, res),
                    f'cell_floor({x!r}, {res}) 分歧')

    def test_cell_snap_matches_support_map_oracle(self):
        for res in RESOLUTIONS:
            for x in adversarial_scalars():
                self.assertEqual(
                    grid.cell_snap(x, res), oracle_cell_snap(x, res),
                    f'cell_snap({x!r}, {res}) 分歧')

    def test_cell_clamp_edge_matches_planner_oracle(self):
        for res in RESOLUTIONS:
            for v in adversarial_scalars():
                self.assertEqual(
                    grid.cell_clamp_edge(v, res), oracle_cell_clamp_edge(v, res),
                    f'cell_clamp_edge({v!r}, {res}) 分歧')

    def test_cell_center_matches_planner_cells_oracle(self):
        for res in RESOLUTIONS:
            for v in adversarial_scalars():
                self.assertEqual(
                    grid.cell_center(v, res), oracle_cell_center(v, res),
                    f'cell_center({v!r}, {res}) 分歧')

    def test_image_axis_matches_terrain_observer_oracle(self):
        rng = random.Random(20261118)
        for res in RESOLUTIONS:
            for half in (5.0, 10.0, 40.0):
                for center in (-2.5, 0.0, 3.7):
                    for _ in range(2000):
                        v = rng.uniform(-10.0, 10.0)
                        self.assertEqual(
                            grid.image_axis(v, center, res, half),
                            oracle_image_axis(v, center, res, half),
                            f'image_axis({v!r},{center},{res},{half}) 分歧')


class DifferentialVectorTests(unittest.TestCase):
    def test_cells_floor_min_matches_import_oracle(self):
        rng = np.random.default_rng(20261118)
        for res in RESOLUTIONS:
            for min_v in (-1.0, 0.0, 2.5):
                arr = rng.uniform(min_v - 1, min_v + 12, 4096)
                edge = np.array([min_v, min_v + res, min_v + 2.3 * res])
                for a in (arr, edge):
                    np.testing.assert_array_equal(
                        grid.cells_floor_min(a, min_v, res),
                        oracle_cells_floor_min(a, min_v, res),
                        err_msg=f'cells_floor_min 分歧 res={res} min_v={min_v}')

    def test_cells_floor_origin_matches_publisher_oracle(self):
        rng = np.random.default_rng(20261119)
        for res in RESOLUTIONS:
            x0 = math.floor(1.37 / res) * res     # 调用侧已对齐的窗口原点
            arr = rng.uniform(x0 - 2, x0 + 8, 4096)
            np.testing.assert_array_equal(
                grid.cells_floor_origin(arr, x0, res),
                oracle_cells_floor_origin(arr, x0, res),
                err_msg=f'cells_floor_origin 分歧 res={res}')


class NamedCaseTests(unittest.TestCase):
    """用户实证的分歧点，作为命名用例钉死（行为说明，不是缺陷报告）。"""

    def test_the_03_disagreement_is_real_and_documented(self):
        # x=0.3, res=0.1：floor 口径得 2、snap 口径得 3——两种口径**当前都合法**，
        # Phase B 裁决交换对之前不许任何一侧悄悄变化。
        self.assertEqual(grid.cell_floor(0.3, 0.1), 2)
        self.assertEqual(grid.cell_snap(0.3, 0.1), 3)
        self.assertEqual(oracle_cell_floor(0.3, 0.1), 2)
        self.assertEqual(oracle_cell_snap(0.3, 0.1), 3)

    def test_fuzz_one_hundred_k_points(self):
        rng = random.Random(20261118)
        for _ in range(100_000):
            res = rng.choice(RESOLUTIONS)
            x = rng.uniform(-100, 100)
            self.assertEqual(grid.cell_floor(x, res), oracle_cell_floor(x, res))
            self.assertEqual(grid.cell_snap(x, res), oracle_cell_snap(x, res))


class ReturnShapeTests(unittest.TestCase):
    """返回类型按原站点断言：标量口径返 int，向量化口径返 ndarray。"""

    def test_scalar_kinds(self):
        for fn in (grid.cell_floor, grid.cell_snap,
                   grid.cell_clamp_edge, grid.cell_center, grid.image_axis):
            result = fn(0.3, 0.1) if fn is not grid.image_axis \
                else fn(0.3, 0.0, 0.1, 5.0)
            self.assertIsInstance(result, int, f'{fn.__name__} 应返 int，得 {type(result)}')

    def test_vector_kinds(self):
        arr = np.array([0.1, 0.2, 0.3])
        for result in (grid.cells_floor_min(arr, 0.0, 0.1),
                       grid.cells_floor_origin(arr, 0.0, 0.1)):
            self.assertIsInstance(result, np.ndarray)


if __name__ == '__main__':
    unittest.main()
