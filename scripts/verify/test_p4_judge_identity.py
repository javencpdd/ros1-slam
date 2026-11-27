#!/usr/bin/env python3
"""p4 导航判据的同任务身份负控。"""
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import p4_judge  # noqa: E402


GOAL = {'accepted': True, 'seq_after': 1, 'goal': [3.9, 2.45, 0.0]}
PROBE = {'bt_goal_seq': 1, 'goal': [3.9, 2.45]}
ARR = {'bt_goal_seq': 1, 'goal': [3.9, 2.45]}


class TaskIdentityTest(unittest.TestCase):
    def test_same_task_passes(self):
        self.assertTrue(p4_judge.task_identity(GOAL, PROBE, ARR)[0])

    def test_missing_acceptance_fails(self):
        self.assertFalse(p4_judge.task_identity(None, PROBE, ARR)[0])
        self.assertFalse(p4_judge.task_identity(dict(GOAL, accepted=False), PROBE, ARR)[0])

    def test_probe_seq_mismatch_fails(self):
        self.assertFalse(p4_judge.task_identity(GOAL, dict(PROBE, bt_goal_seq=2), ARR)[0])

    def test_arrival_seq_mismatch_fails(self):
        self.assertFalse(p4_judge.task_identity(GOAL, PROBE, dict(ARR, bt_goal_seq=2))[0])

    def test_probe_goal_mismatch_fails(self):
        self.assertFalse(p4_judge.task_identity(GOAL, dict(PROBE, goal=[1.0, 2.45]), ARR)[0])

    def test_arrival_goal_mismatch_fails(self):
        self.assertFalse(p4_judge.task_identity(GOAL, PROBE, dict(ARR, goal=[3.9, 1.0]))[0])

    def test_missing_identity_field_fails(self):
        self.assertFalse(p4_judge.task_identity(GOAL, {}, ARR)[0])
        self.assertFalse(p4_judge.task_identity(GOAL, PROBE, {})[0])


class JudgeNavIdentityTest(unittest.TestCase):
    def _write(self, root, name, data):
        with open(os.path.join(root, name), 'w') as fh:
            json.dump(data, fh)

    def _root(self, probe_seq=1, arrival_seq=1):
        root = tempfile.mkdtemp()
        self._write(root, 'nav_goal_once.json', GOAL)
        self._write(root, 'nav_arrival.json', {
            'goal': [3.9, 2.45], 'gt_to_goal_m': 0.10, 'est_to_goal_m': 0.20,
            'bt_goal_seq': arrival_seq})
        self._write(root, 'nav_probe.json', {
            'goal': [3.9, 2.45], 'speed_frames': 10, 'bt_arrived': True,
            'max_cmd_gap_s': 0.1, 'est_travel_m': 1.2, 'bt_goal_seq': probe_seq})
        return root

    def test_all_checks_pass_with_same_task(self):
        verdict, detail, extra = p4_judge.judge_nav(self._root())
        self.assertEqual(verdict, 'PASS', detail + ' ' + extra)

    def test_good_endpoint_but_mixed_task_is_fail(self):
        verdict, detail, _ = p4_judge.judge_nav(self._root(probe_seq=2))
        self.assertEqual(verdict, 'FAIL')
        self.assertIn('身份', detail)

    def test_good_endpoint_but_missing_identity_is_fail(self):
        root = self._root()
        data = json.load(open(os.path.join(root, 'nav_probe.json')))
        data.pop('bt_goal_seq')
        self._write(root, 'nav_probe.json', data)
        verdict, detail, _ = p4_judge.judge_nav(root)
        self.assertEqual(verdict, 'FAIL')
        self.assertIn('身份', detail)


if __name__ == '__main__':
    unittest.main()
