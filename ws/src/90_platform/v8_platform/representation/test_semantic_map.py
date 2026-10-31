"""semantic_map/v1 validation and catalog tests (no ROS)."""
import json
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import semantic_map as smap  # noqa: E402


def make_map(map_id='sem_a', version=1, features=None):
    return {'schema': 'semantic_map/v1', 'map_id': map_id, 'map_version': version,
            'frame': 'map', 'features': list(features or [])}


def feature(feature_id, kind):
    if kind == 'point':
        geometry = {'type': 'point', 'point': [1., 2., 0.]}
    elif kind == 'line':
        geometry = {'type': 'line', 'points': [[0., 0., 0.], [1., 0., 0.]]}
    elif kind == 'polygon':
        geometry = {'type': 'polygon',
                    'points': [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.]]}
    else:
        geometry = {'type': 'box', 'min': [-1., -1., 0.], 'max': [1., 1., 1.]}
    return {'id': feature_id, 'kind': kind, 'geometry': geometry,
            'attributes': {'rotatable': False}}


class SemanticMapTests(unittest.TestCase):
    def test_all_geometry_kinds_validate(self):
        value = make_map(features=[feature('p1', 'point'), feature('l1', 'line'),
                                   feature('g1', 'polygon'), feature('b1', 'box')])
        self.assertIs(smap.validate(value), value)
        self.assertEqual(smap.summary(value),
                         {'map_id': 'sem_a', 'map_version': 1, 'frame': 'map',
                          'features': 4,
                          'kinds': {'box': 1, 'line': 1, 'point': 1, 'polygon': 1}})

    def test_schema_version_and_map_id_are_checked(self):
        for mutate, code in [
                (lambda v: v.update(schema='map/v1'), 'E_SEMANTIC_SCHEMA'),
                (lambda v: v.update(map_id='Sem-A'), 'E_SEMANTIC_MAP_ID'),
                (lambda v: v.update(map_version=0), 'E_SEMANTIC_VERSION'),
                (lambda v: v.update(map_version=True), 'E_SEMANTIC_VERSION'),
                (lambda v: v.pop('frame'), 'E_SEMANTIC_FRAME'),
                (lambda v: v.update(features={}), 'E_SEMANTIC_FEATURES')]:
            value = make_map()
            mutate(value)
            with self.assertRaises(smap.SemanticMapError) as caught:
                smap.validate(value)
            self.assertEqual(caught.exception.code, code)

    def test_duplicate_feature_ids_and_bad_features(self):
        value = make_map(features=[feature('p1', 'point'), feature('p1', 'line')])
        with self.assertRaises(smap.SemanticMapError) as caught:
            smap.validate(value)
        self.assertEqual(caught.exception.code, 'E_SEMANTIC_ID')
        for index, bad, code in [(0, lambda f: f.pop('kind'), 'E_SEMANTIC_KIND'),
                                 (0, lambda f: f.update(attributes=[]),
                                  'E_SEMANTIC_ATTRIBUTES'),
                                 (0, lambda f: f['geometry'].update(type='mesh'),
                                  'E_SEMANTIC_GEOMETRY'),
                                 (0, lambda f: f['geometry'].update(point=[0., 0.]),
                                  'E_SEMANTIC_GEOMETRY'),
                                 (1, lambda f: f['geometry'].update(
                                     points=[[0., 0., 0.]]), 'E_SEMANTIC_GEOMETRY')]:
            value = make_map(features=[feature('p1', 'point'), feature('l1', 'line')])
            bad(value['features'][index])
            with self.assertRaises(smap.SemanticMapError) as caught:
                smap.validate(value)
            self.assertEqual(caught.exception.code, code)

    def test_box_bounds_and_non_finite_points(self):
        value = make_map(features=[feature('b1', 'box')])
        value['features'][0]['geometry']['max'] = [-2., 1., 1.]
        with self.assertRaises(smap.SemanticMapError):
            smap.validate(value)
        value = make_map(features=[feature('p1', 'point')])
        value['features'][0]['geometry']['point'] = [float('nan'), 0., 0.]
        with self.assertRaises(smap.SemanticMapError):
            smap.validate(value)

    def test_load_and_catalog(self):
        with tempfile.TemporaryDirectory() as directory:
            paths = []
            for map_id, version in [('sem_a', 1), ('sem_b', 3)]:
                path = Path(directory) / (map_id + '.json')
                path.write_text(json.dumps(make_map(map_id, version,
                                                    [feature('p1', 'point')])))
                paths.append(path)
            catalog = smap.SemanticCatalog().load_files(paths)
            self.assertEqual(catalog.ids(), ['sem_a', 'sem_b'])
            self.assertEqual(catalog.get('sem_b')['map_version'], 3)
            self.assertEqual(catalog.status()['sem_a']['features'], 1)
            with self.assertRaises(smap.SemanticMapError) as caught:
                catalog.add(make_map('sem_a'))
            self.assertEqual(caught.exception.code, 'E_SEMANTIC_DUPLICATE')
            with self.assertRaises(smap.SemanticMapError) as caught:
                catalog.get('sem_c')
            self.assertEqual(caught.exception.code, 'E_SEMANTIC_MAP_ID')
            broken = Path(directory) / 'broken.json'
            broken.write_text('{')
            with self.assertRaises(smap.SemanticMapError) as caught:
                smap.load(broken)
            self.assertEqual(caught.exception.code, 'E_SEMANTIC_JSON')


if __name__ == '__main__':
    unittest.main()
