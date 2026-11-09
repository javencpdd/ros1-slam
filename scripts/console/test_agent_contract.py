#!/usr/bin/env python3
"""Whitelist contract for the console agent; no Docker or ROS required."""
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
STACKS = ROOT / 'scripts/console/stacks.json'


class StackWhitelistTest(unittest.TestCase):
    def setUp(self):
        self.data = json.loads(STACKS.read_text(encoding='utf-8'))
        self.stacks = {item['name']: item for item in self.data['stacks']}

    def test_required_demo_stacks(self):
        self.assertIn('sim_m1', self.stacks)
        self.assertIn('mapping_m1', self.stacks)
        self.assertIn('stairstudy', self.stacks)
        for stack in self.stacks.values():
            self.assertEqual(stack.get('category'), 'demo')
            self.assertTrue(stack.get('cmd'))
            self.assertTrue(stack.get('container'))
            self.assertNotIn(';', stack['cmd'])
            self.assertNotIn('&&', stack['cmd'])

    def test_dependency_and_master(self):
        self.assertEqual(self.stacks['mapping_m1'].get('requires'), ['sim_m1'])
        for stack in self.stacks.values():
            self.assertTrue(str(stack.get('master', '')).startswith('http://'))

    def test_no_real_robot_or_formal_names(self):
        joined = ' '.join(item['cmd'] for item in self.stacks.values()).lower()
        for word in ('real:=true', 'enable_motion:=true', 'evt', 'formal', '192.168.1.120'):
            self.assertNotIn(word, joined)


if __name__ == '__main__':
    unittest.main()
