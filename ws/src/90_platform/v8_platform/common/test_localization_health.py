#!/usr/bin/env python3
"""localization_health 的自测：正向 + 逐条反控 + 降级口的两条边界。

反控要点：三条判据（收敛／质量／跳变）任意一条被写成"没数据就算过"，对应的
`*_unknown` 用例就会失败；降级口的两条用例则钉住"只放行未知、不放行已判坏"。
"""
import unittest

import localization_health as lh


def cfg(**kw):
    return lh.HealthConfig(**kw)


GOOD = dict(pose_fresh=True, dt_s=0.1, delta_m=0.02, delta_rad=0.01,
            quality=0.9, quality_age_s=0.1, converged=True, converged_age_s=0.1)


class HappyPathTest(unittest.TestCase):
    def test_all_good_passes(self):
        r = lh.evaluate(**GOOD)
        self.assertTrue(r['ok'], r)
        self.assertFalse(r['degraded'], r)
        self.assertEqual([], r['reasons'])
        self.assertIs(True, r['checks']['converged'])

    def test_speed_within_boundary_passes(self):
        r = lh.evaluate(**dict(GOOD, dt_s=0.1, delta_m=0.14))   # 1.4 m/s <= 1.5
        self.assertTrue(r['ok'], r)


class ControlTest(unittest.TestCase):
    def test_stale_pose_fails(self):
        r = lh.evaluate(**dict(GOOD, pose_fresh=False))
        self.assertFalse(r['ok'])
        self.assertIn('E_LOC_NOT_FRESH', r['reasons'])

    def test_jump_by_speed_fails(self):
        r = lh.evaluate(**dict(GOOD, dt_s=0.1, delta_m=0.40))   # 4 m/s
        self.assertFalse(r['ok'])
        self.assertIn('E_LOC_JUMP', r['reasons'])

    def test_jump_by_single_step_fails(self):
        r = lh.evaluate(**dict(GOOD, dt_s=2.0, delta_m=0.60))   # 速度不高但单帧过大
        self.assertFalse(r['ok'])
        self.assertIn('E_LOC_JUMP', r['reasons'])

    def test_jump_by_yaw_fails(self):
        r = lh.evaluate(**dict(GOOD, delta_rad=1.2))
        self.assertFalse(r['ok'])
        self.assertIn('E_LOC_JUMP', r['reasons'])

    def test_converged_unknown_fails(self):
        """没有收敛话题 ⇒ 未知 ⇒ 必须不健康（真机当前就是这种状态）。"""
        r = lh.evaluate(**dict(GOOD, converged=None, converged_age_s=None))
        self.assertFalse(r['ok'])
        self.assertIn('E_LOC_CONVERGED_UNKNOWN', r['reasons'])

    def test_converged_stale_fails(self):
        r = lh.evaluate(**dict(GOOD, converged_age_s=5.0))
        self.assertFalse(r['ok'])
        self.assertIn('E_LOC_CONVERGED_UNKNOWN', r['reasons'])

    def test_not_converged_fails(self):
        r = lh.evaluate(**dict(GOOD, converged=False))
        self.assertFalse(r['ok'])
        self.assertIn('E_LOC_NOT_CONVERGED', r['reasons'])

    def test_quality_unknown_fails(self):
        r = lh.evaluate(**dict(GOOD, quality=None, quality_age_s=None))
        self.assertFalse(r['ok'])
        self.assertIn('E_LOC_QUALITY_UNKNOWN', r['reasons'])

    def test_quality_stale_fails(self):
        r = lh.evaluate(**dict(GOOD, quality_age_s=9.0))
        self.assertFalse(r['ok'])
        self.assertIn('E_LOC_QUALITY_UNKNOWN', r['reasons'])

    def test_quality_low_fails(self):
        r = lh.evaluate(config=cfg(quality_min=0.3), **dict(GOOD, quality=0.05))
        self.assertFalse(r['ok'])
        self.assertIn('E_LOC_QUALITY_LOW', r['reasons'])


class DegradedTest(unittest.TestCase):
    def test_allow_unverified_releases_unknown(self):
        r = lh.evaluate(config=cfg(allow_unverified=True),
                        **dict(GOOD, converged=None, converged_age_s=None,
                               quality=None, quality_age_s=None))
        self.assertTrue(r['ok'], r)
        self.assertTrue(r['degraded'], '放行时必须标 degraded')

    def test_allow_unverified_never_releases_known_bad(self):
        """降级口不得放行已判定的坏：跳变仍必须不健康。"""
        r = lh.evaluate(config=cfg(allow_unverified=True),
                        **dict(GOOD, delta_m=0.9, converged=None, converged_age_s=None))
        self.assertFalse(r['ok'], '降级口放行了跳变')
        self.assertIn('E_LOC_JUMP', r['reasons'])

    def test_require_flags_off_is_explicit(self):
        r = lh.evaluate(config=cfg(require_converged=False, require_quality=False),
                        **dict(GOOD, converged=None, converged_age_s=None,
                               quality=None, quality_age_s=None))
        self.assertTrue(r['ok'])
        self.assertEqual('not_required', r['checks']['converged'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
