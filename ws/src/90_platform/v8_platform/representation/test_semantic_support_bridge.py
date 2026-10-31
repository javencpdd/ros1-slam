"""Semantic support-surface convention -> support_map (no ROS)."""
import copy
import json
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import semantic_support_bridge as bridge  # noqa: E402

FIXTURE = HERE.parent / 'framework' / 'fixtures' / 'maps' / 'sem_support.json'


def fixture():
    return json.loads(FIXTURE.read_text())


class SemanticBridgeTests(unittest.TestCase):
    def test_boxes_become_contract_checked_surfaces(self):
        value = bridge.convert(fixture())
        self.assertEqual((value['map_id'], value['map_version']), ('sem_support', 1))
        surfaces = {item['surface_id']: item for item in value['surfaces']}
        self.assertEqual(sorted(surfaces), ['ground', 'mezz'])
        self.assertEqual(surfaces['mezz']['z_reference'], .6)
        self.assertAlmostEqual(surfaces['mezz']['thickness_m'], .05)
        self.assertEqual(surfaces['mezz']['footprint'],
                         {'min_x': 2., 'min_y': 1., 'max_x': 4., 'max_y': 3.})
        self.assertEqual(surfaces['mezz']['data_uri'], 'semantic://sem_support/mezz')
        self.assertEqual(surfaces['ground']['edges'][0]['to'], 'mezz')
        self.assertEqual(len(value['source']['sha256']), 64)
        self.assertEqual(value['origin'][:2], [-5., -5.])

    def test_version_and_resolution_follow_input(self):
        value = fixture()
        value['map_version'] = 7
        converted = bridge.convert(value, resolution=.25)
        self.assertEqual(converted['map_version'], 7)
        self.assertEqual(converted['resolution'], .25)

    def test_non_support_features_are_ignored(self):
        value = fixture()
        value['features'] = [value['features'][2]]
        with self.assertRaises(bridge.SemanticBridgeError) as caught:
            bridge.convert(value)
        self.assertEqual(caught.exception.code, 'E_SEMANTIC_BRIDGE_EMPTY')

    def test_bad_geometry_id_and_resolution_are_rejected(self):
        value = fixture()
        polygon = copy.deepcopy(value)
        polygon['features'][1]['geometry'] = {
            'type': 'polygon', 'points': [[2., 1., .6], [4., 1., .6], [4., 3., .6]]}
        with self.assertRaises(bridge.SemanticBridgeError) as caught:
            bridge.convert(polygon)
        self.assertEqual(caught.exception.code, 'E_SEMANTIC_BRIDGE_GEOMETRY')
        upper = copy.deepcopy(value)
        upper['features'][1]['id'] = 'Mezz'
        with self.assertRaises(bridge.SemanticBridgeError) as caught:
            bridge.convert(upper)
        self.assertEqual(caught.exception.code, 'E_SEMANTIC_BRIDGE_ID')
        with self.assertRaises(bridge.SemanticBridgeError) as caught:
            bridge.convert(value, resolution=0)
        self.assertEqual(caught.exception.code, 'E_SEMANTIC_BRIDGE_RESOLUTION')

    def test_disconnected_surfaces_are_refused_by_contract(self):
        value = fixture()
        for feature in value['features']:
            if 'edges' in (feature.get('attributes') or {}):
                feature['attributes']['edges'] = []
        with self.assertRaises(bridge.SemanticBridgeError) as caught:
            bridge.convert(value)
        self.assertEqual(caught.exception.code, 'E_REP_CONTRACT')


if __name__ == '__main__':
    unittest.main()
