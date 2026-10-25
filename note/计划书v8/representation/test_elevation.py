"""V8-08 第三切片测试：点云→双层列→面提取全链（A3-1 数据级证据）、噪声拒绝、
z 聚类边界、咨询边不自动成图。"""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import elevation as elev  # noqa: E402
import support_map as rep  # noqa: E402

RES = 0.5


def two_layer_cloud():
    """地面 5×5 格（z=0）＋夹层 3×3 格（z=0.6，与地面部分重叠）＋两粒噪声。"""
    points = []
    for i in range(10):          # x 0..4.5（10 列格×0.5）
        for j in range(10):
            points.append((i * RES, j * RES, 0.0))
    for i in range(3, 9):        # x 1.5..4.0
        for j in range(4, 10):   # y 2.0..4.5
            points.append((i * RES, j * RES, 0.6))
    points += [(9.9, 9.9, 0.3), (9.8, 9.8, 0.3)]  # 噪声：单格小簇
    return points


class ElevationTests(unittest.TestCase):
    def test_two_layers_kept_per_cell(self):
        """A3-1 全链证据：点云进、同格双层列出（重叠区 heights=[0.0, 0.6]）。"""
        result = elev.mapping_pipeline(two_layer_cloud(), RES)
        # (3,4) 格（x=1.5,y=2.0）在两层点云的交叠区
        self.assertEqual(result['columns']['(3, 4)'], [(0.0, 1), (0.6, 1)])
        columns = elev.to_height_columns(result)
        self.assertEqual(columns[(3, 4)].heights(), [0.0, 0.6])

    def test_surfaces_extracted_and_noise_dropped(self):
        result = elev.mapping_pipeline(two_layer_cloud(), RES)
        surfaces = {s['surface_id']: s for s in result['surfaces']}
        self.assertEqual(len(surfaces), 2)  # 地面＋夹层
        zs = sorted(s['z_reference'] for s in surfaces.values())
        self.assertEqual(zs, [0.0, 0.6])
        # 噪声单格簇被拒并如实报数（两粒同格→一小簇）
        self.assertTrue(all(z == 0.3 for z, _ in result['dropped_clusters']))
        self.assertEqual(sum(n for _, n in result['dropped_clusters']), 1)  # 单格
        # 被拒簇的层仍在列上（数据不丢，只是不成面）
        columns = elev.to_height_columns(result)
        self.assertIn(0.3, columns[(19, 19)].heights())
        self.assertEqual(columns[(19, 19)].surface_at(0.3), 'unsurfaced')

    def test_z_cluster_tolerance(self):
        points = [(0.1, 0.1, 0.0), (0.2, 0.2, 0.02), (0.3, 0.3, 0.6)]
        columns = elev.extract_columns(points, RES, z_tol=0.05)
        self.assertEqual(columns[(0, 0)], [(0.01, 2), (0.6, 1)])  # 0.0/0.02 合层，0.6 分层

    def test_min_cells_threshold(self):
        """2×2 格（4 点）恰好达标成面；3 点不成面。"""
        points = [(i * RES, j * RES, 0.0) for i in range(2) for j in range(2)]
        surfaces, dropped = elev.extract_surfaces(elev.extract_columns(points, RES), RES, min_cells=4)
        self.assertEqual(len(surfaces), 1)
        points = points[:3]
        surfaces, dropped = elev.extract_surfaces(elev.extract_columns(points, RES), RES, min_cells=4)
        self.assertEqual(surfaces, [])
        self.assertEqual(dropped, [(0.0, 3)])  # 3 点各占一格，连成一簇被拒（dropped 报簇格数）

    def test_advisory_edges_not_auto_applied(self):
        """edges 只出咨询候选；成图必须用确认后的边（断桥不误连的全链落点）。"""
        result = elev.mapping_pipeline(two_layer_cloud(), RES, advisory_gap=0.0, advisory_step=0.7)
        self.assertTrue(all(e['advisory_only'] for e in result['advisory_edges']))
        # 咨询候选转确认边后成图合法（z 差 0.6、足印重叠→slope 边）
        confirmed = [{'to': e['to'], 'kind': 'slope', 'max_slope_deg': 12.0, 'bidirectional': True}
                     for e in result['advisory_edges']]
        surfaces = [dict(s, edges=confirmed if i == 0 else [])
                    for i, s in enumerate(result['surfaces'])]
        value = rep.build_support_map('from_cloud', RES, [0, 0, 0], surfaces, None,
                                      {'uri': 'elevation://pipeline', 'sha256': 'c' * 64})
        self.assertEqual(value['kind'], 'support_map')
        # 不带确认边直接成图＝DISCONNECTED 拒（管线绝不静默自动连边）
        with self.assertRaises(rep.RepresentationError) as caught:
            rep.build_support_map('no_edges', RES, [0, 0, 0],
                                  [dict(s, edges=[]) for s in result['surfaces']], None,
                                  {'uri': 'x', 'sha256': 'd' * 64})
        self.assertIn('E_MAP_DISCONNECTED', caught.exception.detail)


if __name__ == '__main__':
    unittest.main(verbosity=2)
