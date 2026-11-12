#!/usr/bin/env python3
"""bt_stage_marker 纯函数单测（不需要 ROS）。"""
import importlib.util
import unittest
from pathlib import Path

SPEC = importlib.util.spec_from_file_location(
    'bt_stage_marker',
    Path(__file__).resolve().parents[1] / 'scripts' / 'bt_stage_marker.py')
MOD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MOD)


class StageOfTest(unittest.TestCase):
    def test_follow_path(self):
        stage, detail = MOD.stage_of(
            '{"nodes":[{"name":"mission","status":"RUNNING"},'
            '{"name":"follow_path","status":"RUNNING"}],'
            '"current_seq":2,"goal_seq":2,"path_state":"ready","event":"path_ready"}')
        self.assertEqual(stage, 'FOLLOW_PATH')
        self.assertIn('seq 2/2', detail)
        self.assertIn('path_ready', detail)

    def test_idle_waits_for_goal(self):
        stage, detail = MOD.stage_of(
            '{"nodes":[{"name":"wait_goal","status":"RUNNING"}],'
            '"current_seq":0,"goal_seq":0,"event":"arrival_reset"}')
        self.assertEqual(stage, 'WAIT_GOAL')
        self.assertIn('seq 0/0', detail)

    def test_outcome_preferred(self):
        stage, detail = MOD.stage_of(
            '{"nodes":[{"name":"wait_goal","status":"RUNNING"}],'
            '"current_seq":1,"goal_seq":1,"outcome":"reached","event":"stop"}')
        self.assertIn('reached', detail)
        self.assertNotIn('stop', detail)

    def test_dwell(self):
        stage, _ = MOD.stage_of(
            '{"nodes":[{"name":"dwell_at_goal","status":"RUNNING"}],'
            '"current_seq":1,"goal_seq":1}')
        self.assertEqual(stage, 'DWELL_AT_GOAL')

    def test_bad_input(self):
        self.assertEqual(MOD.stage_of('not-json'), ('', ''))
        self.assertEqual(MOD.stage_of('[]'), ('', ''))
        self.assertEqual(MOD.stage_of(''), ('', ''))
        self.assertEqual(MOD.stage_of('{"nodes":[{"name":7}]}')[0], 'IDLE')


if __name__ == '__main__':
    unittest.main()
