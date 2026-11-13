#!/usr/bin/env python3
"""Consistency tests for the canonical world registry and derived console data."""
import importlib.util
import json
from pathlib import Path
import unittest

REPO = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'world_registry', REPO / 'scripts/ops/world_registry.py')
REG = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REG)

WORLDS_JSON = REPO / 'scripts/console/ui/assets/data/worlds/worlds.json'
MAPS_JSON = REPO / 'scripts/console/ui/assets/data/maps.json'
STACKS_JSON = REPO / 'scripts/console/stacks.json'

GENERATED = WORLDS_JSON.is_file() and MAPS_JSON.is_file()
SKIP = 'generated console data missing; run scripts/ops/build_world_scenes.py and build_maps_index.py'


@unittest.skipUnless(GENERATED, SKIP)
class WorldRegistryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.registry = REG.load_registry()
        cls.worlds = REG.by_id(cls.registry)
        cls.maps = {item['id']: item for item in
                    json.loads(MAPS_JSON.read_text(encoding='utf-8'))['maps']}
        cls.world_index = {item['id']: item for item in
                           json.loads(WORLDS_JSON.read_text(encoding='utf-8'))['worlds']}
        data = json.loads(STACKS_JSON.read_text(encoding='utf-8'))
        cls.stacks = {item['name']: item for item in data['stacks']}

    def test_registry_self_check(self):
        self.assertEqual(REG.check_registry(self.registry), [])
        self.assertEqual(len(self.worlds), 17)
        self.assertEqual(REG.map_bindings(self.registry)['two_floor_nav']['world_id'], 'open')

    def test_world_index_matches_registry(self):
        self.assertEqual(set(self.world_index), set(self.worlds))
        for wid, entry in self.worlds.items():
            index = self.world_index[wid]
            self.assertEqual(index['source'], entry['path'])
            self.assertEqual(index['sha256'], entry['sha256'])
            self.assertEqual(index['label'], entry['label'])
            scene = json.loads((REPO / 'scripts/console/ui' / index['file']).read_text(encoding='utf-8'))
            self.assertEqual(scene['world_id'], wid)
            self.assertEqual(scene['sha256'], entry['sha256'])

    def test_map_world_bindings(self):
        bindings = REG.map_bindings(self.registry)
        for map_id, item in self.maps.items():
            if item.get('world_id') is not None:
                self.assertIn(map_id, bindings)
                self.assertEqual(item['world_id'], bindings[map_id]['world_id'])
                self.assertEqual(item['world'], bindings[map_id]['label'])
        for map_id, binding in bindings.items():
            self.assertIn(map_id, self.maps)
            self.assertEqual(self.maps[map_id]['world_id'], binding['world_id'])
        self.assertEqual(self.maps['two_floor_nav']['world_id'], 'open')
        self.assertEqual(self.maps['mapped_m1']['world_id'], 'm1_world')
        self.assertEqual(self.maps['mapped_nist']['world_id'], 'nist_stairs_120')
        self.assertEqual(self.maps['nist_stairs']['world_id'], 'nist_stairs_120_s020x170')
        self.assertNotEqual(self.maps['two_floor_nav']['world_id'], 'demo_two_floor')

    def test_stack_world_bindings(self):
        bindings = REG.stack_bindings(self.registry)
        for stack_name, stack in self.stacks.items():
            for choice in stack.get('world_choices') or []:
                key = (stack_name, choice['file'])
                self.assertIn(key, bindings, key)
                world = self.worlds[bindings[key]]
                expected = choice.get('path') or ('stairstudy/worlds/' + choice['file'])
                self.assertTrue(world['path'].endswith(expected))
            if stack.get('world_choices'):
                continue
            if not stack.get('world_stack'):
                continue
            if stack_name == 'sim_nist':
                self.assertIn(('sim_nist', 'nist_stairs_120.world'), bindings)
                self.assertEqual(stack['world'], 'nist_stairs_120.world')
            elif stack.get('world') not in (None, '—'):
                self.assertIn((stack_name, stack['world']), bindings, stack_name)
        sim = self.stacks['sim_m1']
        self.assertEqual({choice['file'] for choice in sim['world_choices']},
                         {'m1_world.world', 'm1_world_v2.world',
                          'demo_two_floor.world', 'open.world'})
        study = self.stacks['stairstudy']
        self.assertEqual(len(study['world_choices']), 9)
        for choice in study['world_choices']:
            self.assertEqual(choice['file'], self.worlds[
                bindings[('stairstudy', choice['file'])]]['path'].split('/')[-1])


if __name__ == '__main__':
    unittest.main()
