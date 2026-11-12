#!/usr/bin/env python3
"""real_preflight 的纯解析单测（不需要 ROS）。"""
import importlib.util
import unittest
from pathlib import Path

SPEC = importlib.util.spec_from_file_location(
    'real_preflight', Path(__file__).resolve().parent / 'real_preflight.py')
MOD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MOD)


class ParseStatusTest(unittest.TestCase):
    def test_valid_status(self):
        data = MOD.parse_bt_status(
            '{"tree":"NavigateWithPlanner","event":"stop","outcome":"reached",'
            '"queue_depth":0,"planner_mode":"signal","arrival_signal":true}')
        self.assertEqual(data['planner_mode'], 'signal')
        self.assertEqual(data['outcome'], 'reached')
        self.assertIs(data['arrival_signal'], True)

    def test_invalid_json(self):
        self.assertIsNone(MOD.parse_bt_status('not-json'))

    def test_non_dict_json(self):
        self.assertIsNone(MOD.parse_bt_status('[1,2,3]'))
        self.assertIsNone(MOD.parse_bt_status('"text"'))

    def test_empty(self):
        self.assertIsNone(MOD.parse_bt_status(''))


if __name__ == '__main__':
    unittest.main()
