#!/usr/bin/env python3
"""Whitelist contract for the console agent; no Docker or ROS required."""
import importlib.util
import ipaddress
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
STACKS = ROOT / 'scripts/console/stacks.json'
SPEC = importlib.util.spec_from_file_location('console_agent', ROOT / 'scripts/console/agent.py')
AGENT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AGENT)


class StackWhitelistTest(unittest.TestCase):
    def setUp(self):
        self.data = json.loads(STACKS.read_text(encoding='utf-8'))
        self.stacks = {item['name']: item for item in self.data['stacks']}

    def test_required_demo_stacks(self):
        self.assertIn('sim_m1', self.stacks)
        self.assertIn('mapping_m1', self.stacks)
        self.assertIn('stairstudy', self.stacks)
        self.assertIn('sim_nist', self.stacks)
        self.assertIn('safety_mux', self.stacks)
        self.assertIn('bt_console', self.stacks)
        self.assertIn('voxel_stream', self.stacks)
        for stack in self.stacks.values():
            self.assertEqual(stack.get('category'), 'demo')
            self.assertTrue(stack.get('cmd'))
            self.assertTrue(stack.get('container'))
            self.assertNotIn(';', stack['cmd'])
            self.assertNotIn('&&', stack['cmd'])

    def test_world_choices_exist(self):
        package = ROOT / 'ws/src/95_simulation/lite3_gazebo_sim'
        sim = self.stacks['sim_m1']
        self.assertEqual(len(sim['world_choices']), 4)
        self.assertIn('{world}', sim['cmd_template'])
        for choice in sim['world_choices']:
            path = package / (choice.get('path') or choice['file'])
            self.assertTrue(path.exists(), choice['file'])
        study = self.stacks['stairstudy']
        self.assertEqual(len(study['world_choices']), 9)
        self.assertIn('{world}', study['cmd_template'])
        for choice in study['world_choices']:
            self.assertTrue((package / 'stairstudy/worlds' / choice['file']).exists(), choice['file'])

    def test_build_stack_command_whitelist(self):
        stack = self.stacks['sim_m1']
        cmd, world, error = AGENT.build_stack_command(stack, 'demo_two_floor.world')
        self.assertIsNone(error)
        self.assertEqual(world, 'demo_two_floor.world')
        self.assertIn('worlds/demo_two_floor.world', cmd)
        cmd, world, error = AGENT.build_stack_command(stack, 'open.world')
        self.assertIsNone(error)
        self.assertEqual(world, 'open.world')
        self.assertIn('scenarios/two_floor_nav/worlds/open.world', cmd)
        cmd, world, error = AGENT.build_stack_command(stack, 'not_listed.world')
        self.assertIsNotNone(error)
        self.assertIsNone(cmd)
        cmd, world, error = AGENT.build_stack_command(stack, None)
        self.assertIsNone(error)
        self.assertEqual(cmd, stack['cmd'])

    def test_dependency_and_master(self):
        self.assertEqual(self.stacks['mapping_m1'].get('requires'), ['sim_m1'])
        self.assertEqual(self.stacks['voxel_stream'].get('requires'), ['sim_m1'])
        for stack in self.stacks.values():
            self.assertTrue(str(stack.get('master', '')).startswith('http://'))

    def test_no_real_robot_or_formal_names(self):
        joined = ' '.join(item['cmd'] for item in self.stacks.values()).lower()
        for word in ('real:=true', 'enable_motion:=true', 'evt', 'formal', '192.168.1.120'):
            self.assertNotIn(word, joined)


class ClientIdentityTest(unittest.TestCase):
    def setUp(self):
        data = json.loads(STACKS.read_text(encoding='utf-8'))
        self.stacks = {item['name']: item for item in data['stacks']}

    def test_local_sources(self):
        networks = [ipaddress.ip_network('172.16.0.0/12')]
        self.assertTrue(AGENT.classify_client('127.0.0.1', {'10.63.23.135'}, networks)[0])
        self.assertTrue(AGENT.classify_client('::1', {'10.63.23.135'}, networks)[0])
        self.assertTrue(AGENT.classify_client('::ffff:127.0.0.1', {'10.63.23.135'}, networks)[0])
        self.assertTrue(AGENT.classify_client('10.63.23.135', {'10.63.23.135'}, networks)[0])
        self.assertTrue(AGENT.classify_client('172.18.0.2', {'10.63.23.135'}, networks)[0])

    def test_lan_is_remote(self):
        networks = [ipaddress.ip_network('172.16.0.0/12')]
        local, reason = AGENT.classify_client('10.63.23.50', {'10.63.23.135'}, networks)
        self.assertFalse(local)
        self.assertEqual(reason, 'lan')

    def test_pid_match_declared(self):
        for stack in self.stacks.values():
            self.assertTrue(stack.get('pid_match'), stack['name'])

    def test_dependent_stop_order(self):
        order = AGENT.dependent_order('sim_m1')
        self.assertEqual(order[-1], 'sim_m1')
        self.assertEqual(set(order[:-1]),
                         {'mapping_m1', 'mapping_save', 'safety_mux',
                          'bt_console', 'voxel_stream'})

    def test_dependency_cycle_rejected(self):
        stacks = {'a': {'name': 'a', 'requires': ['b']},
                  'b': {'name': 'b', 'requires': ['a']}}
        with self.assertRaisesRegex(ValueError, 'dependency cycle'):
            AGENT.dependent_order('a', stacks)

    def test_local_only_stack_flag_default_false(self):
        for stack in self.stacks.values():
            self.assertFalse(stack.get('local_only'), stack['name'])


if __name__ == '__main__':
    unittest.main()
