#!/usr/bin/env python3
"""支撑面关联契约：关联容差与几何容差分离，且关联结果可判读、可拒绝。

2026-11-27 裁决：新增 support_assoc_tol_m 只用于"把估计位姿关联到支撑面"，
support_tol_m 继续服务几何/接触/碰撞/净空判据；关联后的 z 不得覆盖原始位姿。
这里用最小合成地图把四种情形钉住：默认兼容、无匹配、多层歧义、残差如实上报。
"""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import local_native as ln  # noqa: E402


def _map(extra_surface=None):
    res = 0.1
    # 不写 cells ⇒ 走足迹盒判定（cells=None 分支），合成地图不必编码 runs
    surfaces = [{'surface_id': 'ground', 'traversability': 'supported',
                 'z_reference': 0.0, 'thickness_m': 0.1,
                 'footprint': {'min_x': 0.0, 'min_y': 0.0, 'max_x': 4.0, 'max_y': 4.0}}]
    if extra_surface is not None:
        surfaces.append(extra_surface)
    return {'map_id': 'm', 'map_version': 3, 'resolution': res,
            'origin': [0.0, 0.0], 'extent': [4.0, 4.0], 'surfaces': surfaces}


class AssociateTest(unittest.TestCase):
    def test_default_tol_keeps_history(self):
        cfg = ln.Config()
        self.assertAlmostEqual(cfg.support_assoc_tol_m, 1e-6)
        self.assertAlmostEqual(cfg.support_tol_m, 1e-6)

    def test_exact_pose_associates(self):
        r = ln.associate(_map(), (1.0, 1.0, 0.0), ln.Config())
        self.assertTrue(r['valid'])
        self.assertEqual(r['surface_id'], 'ground')
        self.assertAlmostEqual(r['residual_z'], 0.0, places=9)
        self.assertFalse(r['ambiguous'])
        self.assertEqual(r['map_version'], 3)

    def test_pose_off_plane_is_rejected_by_default(self):
        r = ln.associate(_map(), (1.0, 1.0, -0.0637), ln.Config())
        self.assertFalse(r['valid'], '默认容差 1e-6 下 6.4cm 残差必须判无匹配')
        self.assertTrue(r['candidates'] or not r['valid'])

    def test_assoc_tol_can_be_widened_without_touching_geometry_tol(self):
        cfg = ln.Config(support_assoc_tol_m=0.08)
        r = ln.associate(_map(), (1.0, 1.0, -0.0637), cfg)
        self.assertTrue(r['valid'])
        self.assertAlmostEqual(r['residual_z'], -0.0637, places=6)
        # 几何容差必须**没被**带着放宽
        self.assertAlmostEqual(cfg.support_tol_m, 1e-6)

    def test_multi_floor_is_flagged_ambiguous(self):
        upper = {'surface_id': 'floor2', 'traversability': 'supported',
                 'z_reference': 0.05, 'thickness_m': 0.1,
                 'footprint': {'min_x': 0.0, 'min_y': 0.0, 'max_x': 4.0, 'max_y': 4.0}}
        cfg = ln.Config(support_assoc_tol_m=0.08)
        r = ln.associate(_map(upper), (1.0, 1.0, 0.02), cfg)
        # 两层都在容差内 ⇒ 必须标 ambiguous，供上层拒绝运动
        self.assertTrue(r['ambiguous'], '同 XY 两层都在容差内必须报歧义')

    def test_outside_cells_is_invalid(self):
        r = ln.associate(_map(), (9.0, 9.0, 0.0), ln.Config())
        self.assertFalse(r['valid'])

    def test_association_does_not_mutate_input_point(self):
        p = (1.0, 1.0, -0.0637)
        ln.associate(_map(), p, ln.Config(support_assoc_tol_m=0.08))
        self.assertEqual(p[2], -0.0637, '关联不得改写原始位姿（z 不能被覆盖）')


if __name__ == '__main__':
    unittest.main()
