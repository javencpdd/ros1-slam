"""V8-08 首切片测试：A3 三判据设计层证据＋extent 定界＋契约 fail-closed。"""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import support_map as rep  # noqa: E402
import validate as contract  # noqa: E402


def surface(sid, z, fp, edges=(), traversability='supported'):
    return {'surface_id': sid, 'z_reference': z,
            'footprint': fp, 'data_uri': 'fixtures/%s.json' % sid,
            'edges': list(edges), 'traversability': traversability}


TWO_FLOORS = [
    surface('ground', 0.0, {'min_x': -5, 'min_y': -5, 'max_x': 5, 'max_y': 5},
            edges=[{'to': 'mezz', 'kind': 'slope', 'max_slope_deg': 12.0, 'bidirectional': True}]),
    surface('mezz', 0.6, {'min_x': 2, 'min_y': 1, 'max_x': 4, 'max_y': 3},
            edges=[{'to': 'ground', 'kind': 'slope', 'max_slope_deg': 12.0, 'bidirectional': True}]),
]
SOURCE = {'uri': 'design://two_floors', 'sha256': 'a' * 64}


class ColumnTests(unittest.TestCase):
    def test_same_xy_keeps_all_layers(self):
        """A3-1：同 XY 双层不丢层——构造上无『取最高』入口，查询返回全部。"""
        col = rep.HeightColumn(0, 0).add(0.0, 'ground').add(0.6, 'mezz').add(3.2, 'roof')
        self.assertEqual(col.heights(), [0.0, 0.6, 3.2])
        self.assertEqual(col.surface_at(0.6), 'mezz')
        self.assertFalse(hasattr(col, 'top') or hasattr(col, 'highest'))  # 无折叠入口

    def test_z_conflict_rejected(self):
        with self.assertRaises(rep.RepresentationError) as caught:
            rep.HeightColumn(0, 0).add(0.0, 'ground').add(0.0, 'other')
        self.assertEqual(caught.exception.code, 'E_REP_Z_CONFLICT')

    def test_clearance(self):
        col = rep.HeightColumn(0, 0).add(0.0, 'ground').add(2.0, 'roof')
        self.assertAlmostEqual(col.clearance(0.0), 2.0)
        self.assertIsNone(col.clearance(2.0))  # 开顶不冒充无限净空


class EvidenceTests(unittest.TestCase):
    def test_unknown_never_free_by_default(self):
        """A3-2：缺证＝unknown，没有默认 free 路径。"""
        grid = rep.EvidenceGrid()
        self.assertEqual(grid.state(0, 0, 0.0), 'unknown')
        grid.add_free_evidence(0, 0, 0.0)
        self.assertEqual(grid.state(0, 0, 0.0), 'free')
        self.assertEqual(grid.state(1, 0, 0.0), 'unknown')


class EdgeTests(unittest.TestCase):
    def test_suggest_edges_advisory_only(self):
        """A3-3：几何邻近只出咨询候选，不自动成边。"""
        near = rep.suggest_edges(TWO_FLOORS, max_xy_gap=0.1, max_z_step=0.5)
        self.assertTrue(all(c['advisory_only'] for c in near))
        far = rep.suggest_edges([surface('a', 0, {'min_x': 0, 'min_y': 0, 'max_x': 1, 'max_y': 1}),
                                 surface('b', 0, {'min_x': 9, 'min_y': 9, 'max_x': 10, 'max_y': 10})],
                                max_xy_gap=0.5, max_z_step=0.5)
        self.assertEqual(far, [])


class BuilderTests(unittest.TestCase):
    def test_build_valid_and_extent_defined(self):
        """R631 挂账闭合：support_map 的 extent＝足迹并集外包框格数。"""
        value = rep.build_support_map('two_floors', 0.1, [-5.0, -5.0, 0.0],
                                      TWO_FLOORS, None, SOURCE)
        contract.validate('map', value)  # 契约层复验
        self.assertEqual(value['extent'], {'width': 100, 'height': 100})
        # 多高度列反查：重叠 XY 处两层俱在（A3-1 全链证据）
        columns = rep.columns_of(value)
        overlap = columns[(3, 2)]
        self.assertEqual(overlap.heights(), [0.0, 0.6])

    def test_disconnected_rejected_per_component_split(self):
        """不连通支撑域按契约拒（E_MAP_DISCONNECTED）——登记的语义裁定＝按连通分量拆 map。"""
        apart = [surface('a', 0, {'min_x': 0, 'min_y': 0, 'max_x': 1, 'max_y': 1}),
                 surface('b', 0.6, {'min_x': 9, 'min_y': 9, 'max_x': 10, 'max_y': 10})]
        with self.assertRaises(rep.RepresentationError) as caught:
            rep.build_support_map('apart', 0.1, [0.0, 0.0, 0.0], apart, None, SOURCE)
        self.assertEqual(caught.exception.code, 'E_REP_CONTRACT')
        self.assertIn('E_MAP_DISCONNECTED', caught.exception.detail)

    def test_no_surfaces_rejected(self):
        with self.assertRaises(rep.RepresentationError) as caught:
            rep.build_support_map('empty', 0.1, [0, 0, 0], [], None, SOURCE)
        self.assertEqual(caught.exception.code, 'E_REP_NO_SURFACES')

    def test_extent_scales_with_footprints(self):
        wide = [surface('g', 0, {'min_x': 0, 'min_y': 0, 'max_x': 3.05, 'max_y': 1.0}),
                surface('m', 0.6, {'min_x': 0, 'min_y': 0, 'max_x': 3.05, 'max_y': 1.0},
                        edges=[{'to': 'g', 'kind': 'slope', 'max_slope_deg': 12.0, 'bidirectional': True}]),
                surface('g2', 0, {'min_x': 0, 'min_y': 0, 'max_x': 3.05, 'max_y': 1.0},
                        edges=[{'to': 'm', 'kind': 'slope', 'max_slope_deg': 12.0, 'bidirectional': True}])]
        # 修正：g2 与 g 同高同足迹会触发 E_MAP_ML_SAME_Z —— 构造合法的单连通三层样例
        del wide[2]
        wide[0]['edges'] = [{'to': 'm', 'kind': 'slope', 'max_slope_deg': 12.0, 'bidirectional': True}]
        value = rep.build_support_map('wide', 0.5, [0, 0, 0], wide, None, SOURCE)
        self.assertEqual(value['extent'], {'width': 7, 'height': 2})  # ceil(3.05/0.5)=7


if __name__ == '__main__':
    unittest.main(verbosity=2)
