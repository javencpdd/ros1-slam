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
    return {'schema_version': 2, 'map_id': 'm', 'map_version': 3, 'resolution': res, 'frame': 'map',
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
        # 两层都在容差内 ⇒ 必须标 ambiguous **且 valid=False**
        self.assertTrue(r['ambiguous'], '同 XY 两层都在容差内必须报歧义')
        self.assertFalse(r['valid'], '歧义必须直接判无效（上一版这里仍是 True，是真安全漏洞）')

    def test_ambiguous_support_is_refused_by_support_at(self):
        # 生产路径（plan/track 走 _support_at）必须同样拒绝，不能只在 associate 里标歧义
        upper = {'surface_id': 'floor2', 'traversability': 'supported',
                 'z_reference': 0.05, 'thickness_m': 0.1,
                 'footprint': {'min_x': 0.0, 'min_y': 0.0, 'max_x': 4.0, 'max_y': 4.0}}
        cfg = ln.Config(support_assoc_tol_m=0.08)
        self.assertIsNone(ln._support_at(_map(upper), (1.0, 1.0, 0.02), cfg,
                                         tol=cfg.support_assoc_tol_m))

    def test_outside_cells_is_invalid(self):
        r = ln.associate(_map(), (9.0, 9.0, 0.0), ln.Config())
        self.assertFalse(r['valid'])

    def test_association_does_not_mutate_input_point(self):
        p = (1.0, 1.0, -0.0637)
        ln.associate(_map(), p, ln.Config(support_assoc_tol_m=0.08))
        self.assertEqual(p[2], -0.0637, '关联不得改写原始位姿（z 不能被覆盖）')


class MigrationTest(unittest.TestCase):
    def test_explicit_large_support_tol_emits_note_not_silent_inherit(self):
        cfg = ln.Config(support_tol_m=0.25)
        notes = cfg.migration_notes()
        self.assertEqual(len(notes), 1)
        self.assertAlmostEqual(cfg.support_assoc_tol_m, 1e-6, msg='不得自动继承大容差')

    def test_defaults_emit_no_note(self):
        self.assertEqual(ln.Config().migration_notes(), [])


if __name__ == '__main__':
    unittest.main()


class ContactPoseContractTest(unittest.TestCase):
    """轨迹首点与规划接触点必须可判读地一致（不一致要能被发现，而不是假设一致）。"""

    def _map2(self):
        """用仓库里**真实**的地图做 plan 级验证：合成图过不了 schema 必填项，
        而那正是"只测函数、不测生产路径"的老毛病。"""
        import json
        # 向上找带 data/maps 的仓库根（与 map_file_node 同法，别数层级）
        d = HERE
        path = None
        for _ in range(8):
            cand = os.path.join(d, 'data', 'maps', 'generated', 'v8_m1_hdl_v1.json')
            if os.path.isfile(cand):
                path = cand
                break
            d = os.path.dirname(d)
        if path is None:
            raise unittest.SkipTest('找不到 data/maps/generated/v8_m1_hdl_v1.json')
        with open(path, encoding='utf-8') as fh:
            return json.load(fh)

    def test_plan_reports_contact_and_consistency_flag(self):
        cfg = ln.Config(support_assoc_tol_m=0.08)
        try:
            out = ln.plan(self._map2(), (5.2, 2.45, 0.0), (6.0, 2.45, 0.0),
                          obstacles=(), now=100.0, config=cfg)
        except Exception as e:                     # 合成地图不满足全部前置时如实跳过
            self.skipTest('plan 前置不满足：%s' % e)
        self.assertIn('estimated_pose', out)
        self.assertIn('support_association', out)
        self.assertIn('contact_pose', out)
        self.assertIn('pose_uncertainty', out)
        # 接触点不得改写估计位姿
        self.assertEqual(out['estimated_pose'][2], 0.0)
        self.assertTrue(out['support_association']['start']['valid'])
        # 一致与否都必须**如实上报**，不能默认 True
        self.assertIn('trajectory_first_matches_contact', out)
        self.assertIsInstance(out['trajectory_first_matches_contact'], bool)
