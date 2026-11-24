#!/usr/bin/env python3
"""real_gate 的自测：准入检查 + 运动门控，各带反控。

反控要点：
  准入 —— 每一种"未标定/占位"都必须让它不通过；且开发口只能放行外参这一类，
          不得把"阈值占位"一起放掉（那会让质量判据形同虚设）。
  门控 —— 异常发生时必须真的产出"取消"动作（不是只记日志）；恢复时不得产出任何
          目标类动作（结构上杜绝自动重启旧任务）。
"""
import unittest

import real_gate as rg

GOOD_ATT = {'xyz': [0.10, 0.0, 0.30], 'measured_at': '2026-11-24', 'measured_by': 'G8-作业单-07'}


class ExtrinsicGateTest(unittest.TestCase):
    def test_all_good_passes(self):
        r = rg.check_extrinsic(GOOD_ATT, [0.10, 0.0, 0.30], 0.35)
        self.assertTrue(r['ok'], r)
        self.assertFalse(r['degraded'])

    def test_missing_attestation_fails(self):
        r = rg.check_extrinsic(None, [0.10, 0.0, 0.30], 0.35)
        self.assertFalse(r['ok'])
        self.assertIn('E_EXTRINSIC_UNATTESTED', r['reasons'])

    def test_mismatch_fails(self):
        r = rg.check_extrinsic(GOOD_ATT, [0.10, 0.0, 0.90], 0.35)
        self.assertFalse(r['ok'])
        self.assertIn('E_EXTRINSIC_MISMATCH', r['reasons'])

    def test_incomplete_attestation_fails(self):
        att = dict(GOOD_ATT)
        del att['measured_by']
        r = rg.check_extrinsic(att, [0.10, 0.0, 0.30], 0.35)
        self.assertFalse(r['ok'])
        self.assertIn('E_EXTRINSIC_ATTESTATION_INCOMPLETE', r['reasons'])

    def test_quality_placeholder_fails(self):
        r = rg.check_extrinsic(GOOD_ATT, [0.10, 0.0, 0.30], 0.0)
        self.assertFalse(r['ok'])
        self.assertIn('E_QUALITY_MIN_PLACEHOLDER', r['reasons'])

    def test_dev_switch_releases_extrinsic_only(self):
        r = rg.check_extrinsic(None, None, 0.35, allow_uncalibrated=True)
        self.assertTrue(r['ok'], r)
        self.assertTrue(r['degraded'], '放行未标定必须标 degraded')

    def test_dev_switch_never_releases_placeholder(self):
        """反控：开发口不得把阈值占位一起放掉。"""
        r = rg.check_extrinsic(None, None, 0.0, allow_uncalibrated=True)
        self.assertFalse(r['ok'], '允许未标定把阈值占位也带过去了')
        self.assertIn('E_QUALITY_MIN_PLACEHOLDER', r['reasons'])

    def test_bad_numbers_fail_closed(self):
        r = rg.check_extrinsic({'xyz': ['a', 'b', 'c']}, [0.1, 0.0, 0.3], 0.35)
        self.assertFalse(r['ok'])

    def test_non_motion_entry_warns_only(self):
        """不运动的入口（建图/定位干跑）只告警：ok=True 但必须 degraded 且保留理由。"""
        r = rg.check_extrinsic(None, None, 0.0, require_calibrated=False)
        self.assertTrue(r['ok'], '干跑入口被阻断，标定数据就采不到了')
        self.assertTrue(r['degraded'])
        self.assertTrue(r['reasons'], '只告警时不该把理由吞掉')

    def test_motion_entry_blocks_by_default(self):
        """反控：默认 require_calibrated=True，未标定必须阻断。"""
        r = rg.check_extrinsic(None, None, 0.35)
        self.assertFalse(r['ok'])


class MotionGateTest(unittest.TestCase):
    def test_healthy_is_silent(self):
        g = rg.MotionGate(republish_s=2.0)
        self.assertEqual([], g.update(True, 0.0))
        self.assertFalse(g.latched)

    def test_unhealthy_emits_cancel_immediately(self):
        g = rg.MotionGate(republish_s=2.0)
        acts = g.update(False, 10.0, detail='E_LOC_JUMP')
        kinds = [a['type'] for a in acts]
        self.assertIn('cancel', kinds, '定位异常没有产出取消动作（只是打了日志）')
        cancel = [a for a in acts if a['type'] == 'cancel'][0]
        self.assertRegex(cancel['payload'], r'^\d+:mission:loc_health')
        self.assertIn('E_LOC_JUMP', cancel['payload'])

    def test_no_goal_actions_ever(self):
        """结构反控：本闸永远不发目标，恢复后旧任务不可能被它重启。"""
        g = rg.MotionGate(republish_s=1.0)
        seen = []
        for t, ok in ((0.0, True), (1.0, False), (2.0, False), (3.5, False),
                      (4.0, True), (6.0, True)):
            seen += g.update(ok, t)
        self.assertTrue(seen)
        self.assertTrue(all(a['type'] in ('cancel', 'note') for a in seen),
                        '出现了非取消/日志类动作：%s' % seen)
        self.assertEqual(1, len([a for a in seen if a['type'] == 'note'
                                 and '不自动重启' in a['text']]),
                         '恢复时没有明确声明不自动重启')

    def test_republish_throttles_but_keeps_asserting(self):
        g = rg.MotionGate(republish_s=2.0)
        self.assertEqual(1, len([a for a in g.update(False, 0.0) if a['type'] == 'cancel']))
        self.assertEqual([], g.update(False, 1.0), '窗口内重复发取消')
        self.assertEqual(1, len([a for a in g.update(False, 2.5) if a['type'] == 'cancel']),
                         '过了窗口没有重申取消')

    def test_recovery_clears_latch_without_resuming(self):
        g = rg.MotionGate(republish_s=2.0)
        g.update(False, 0.0)
        self.assertTrue(g.latched)
        acts = g.update(True, 1.0)
        self.assertFalse(g.latched)
        self.assertEqual(['note'], [a['type'] for a in acts])

    def test_second_episode_increments_seq(self):
        g = rg.MotionGate(republish_s=10.0)
        first = [a for a in g.update(False, 0.0) if a['type'] == 'cancel'][0]['payload']
        g.update(True, 1.0)
        second = [a for a in g.update(False, 2.0) if a['type'] == 'cancel'][0]['payload']
        self.assertNotEqual(first.split(':')[0], second.split(':')[0],
                            '两次异常用了同一个 seq，取消代次无法区分')

    def test_negative_republish_rejected(self):
        with self.assertRaises(rg.GateError):
            rg.MotionGate(republish_s=-1)


if __name__ == '__main__':
    unittest.main(verbosity=2)
