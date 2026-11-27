"""速度相关安全距离（dynamic_margin_extra）的判据测试。

三连结构（本仓惯例）：
  1) 主测     —— 速度越高，额外余量越大；且撞得上/撞不上随余量翻转；
  2) 注入有效性 —— 把 reaction_s / brake_decel_mps2 置零后主测必须失效（证明测的就是它）；
  3) 反控     —— fail-closed：拿不到速度或速度非法时按 max_speed 取，不得降回固定值。
"""
import sys
import unittest
from dataclasses import replace
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
import local_native as native  # noqa: E402


def config(**overrides):
    base = native.Config(radius_m=0.15, clearance_m=0.10, max_speed_mps=0.6,
                         reaction_s=0.10, brake_decel_mps2=1.0,
                         dynamic_margin_cap_m=0.30)
    return replace(base, **overrides) if overrides else base


class DynamicMarginTests(unittest.TestCase):

    # ---------- 1) 主测 ----------
    def test_margin_grows_with_speed(self):
        cfg = config()
        slow = native.dynamic_margin_extra(cfg, 0.1)
        fast = native.dynamic_margin_extra(cfg, 0.6)
        self.assertGreater(fast, slow)
        # 手算：0.6*0.10 + 0.36/2 = 0.06 + 0.18 = 0.24
        self.assertAlmostEqual(fast, 0.24, places=6)

    def test_margin_changes_hit_verdict(self):
        """同一几何，余量变大后必须从「不碰」翻成「碰」。

        盒子 y∈[0,0.2]，base 膨胀 0.25 -> 上界 0.45；再叠 0.24 -> 上界 0.69。
        取 y=0.55：恰好落在两档之间，只有加上速度余量才判碰撞。
        """
        cfg = config()
        lo, hi = (1.0, 0.0, 0.0), (1.2, 0.2, 1.0)
        obstacle = (lo, hi, (0.0, 0.0, 0.0), 0.0, 0.0, 10.0)
        a, b = (0.0, 0.55, 0.0, 0.0), (2.0, 0.55, 0.0, 4.0)
        self.assertFalse(native._swept_hit(a, b, obstacle, cfg, margin_extra=0.0))
        self.assertTrue(native._swept_hit(a, b, obstacle, cfg,
                                         margin_extra=native.dynamic_margin_extra(cfg, 0.6)))

    # ---------- 2) 注入有效性 ----------
    def test_weaker_brake_increases_tail_and_flips_hit(self):
        # 同一速度/几何，只改制动减速度：弱制动必须给出更长尾段，且能把“不碰”翻成“碰”。
        normal = config(dynamic_margin_cap_m=1.0)
        weak = config(brake_decel_mps2=0.25, dynamic_margin_cap_m=1.0)
        speed = 0.6
        normal_extra = native.dynamic_margin_extra(normal, speed)
        weak_extra = native.dynamic_margin_extra(weak, speed)
        self.assertGreater(weak_extra, normal_extra)
        lo, hi = (1.0, 0.0, 0.0), (1.2, 0.2, 1.0)
        obstacle = (lo, hi, (0.0, 0.0, 0.0), 0.0, 0.0, 10.0)
        a, b = (0.0, 0.8, 0.0, 0.0), (2.0, 0.8, 0.0, 4.0)
        self.assertFalse(native._swept_hit(a, b, obstacle, normal, margin_extra=normal_extra))
        self.assertTrue(native._swept_hit(a, b, obstacle, weak, margin_extra=weak_extra))

    def test_flat_when_terms_zeroed(self):
        """把反应与制动项清零后，余量必须与速度无关 —— 否则主测测的是别的东西。"""
        cfg = config(reaction_s=1e-9, brake_decel_mps2=1e9)
        low = native.dynamic_margin_extra(cfg, 0.1)
        high = native.dynamic_margin_extra(cfg, 0.6)
        self.assertAlmostEqual(low, 0.0, places=6)
        self.assertAlmostEqual(high, 0.0, places=6)

    # ---------- 3) 反控（fail-closed） ----------
    def test_unknown_speed_falls_back_to_max(self):
        cfg = config()
        expected = native.dynamic_margin_extra(cfg, cfg.max_speed_mps)
        for bad in (None, float('nan'), float('inf'), -0.5):
            self.assertAlmostEqual(native.dynamic_margin_extra(cfg, bad), expected,
                                   places=6, msg='speed=%r 必须退到 max_speed' % (bad,))

    def test_cap_bounds_extra(self):
        cfg = config(dynamic_margin_cap_m=0.05)
        self.assertAlmostEqual(native.dynamic_margin_extra(cfg, 0.6), 0.05, places=6)

    def test_never_reduces_base_clearance(self):
        """余量恒非负 —— 既有 radius+clearance 只能被加，不能被减。"""
        for speed in (0.0, 0.1, 0.6, 5.0):
            self.assertGreaterEqual(native.dynamic_margin_extra(config(), speed), 0.0)

    def test_segment_speed(self):
        self.assertAlmostEqual(native._segment_speed((0, 0, 0, 0.0), (1, 0, 0, 2.0)), 0.5)
        self.assertIsNone(native._segment_speed((0, 0, 0, 1.0), (1, 0, 0, 1.0)))


if __name__ == '__main__':
    unittest.main()
