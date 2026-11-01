"""A/B/C/E pure-layer tests: app config, registry, surface resolution, markers."""
import json
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
sys.path.insert(0, str(HERE.parent / 'planner'))
import app_config
import map_registry
import surface_resolver
import support_map_viz
import support_planner


def two_layer():
    return {
        'map_id': 'two', 'map_version': 1, 'resolution': .1, 'frame': 'map',
        'surfaces': [
            {'surface_id': 'ground', 'z_reference': 0., 'traversability': 'supported',
             'footprint': {'min_x': -1., 'min_y': -1., 'max_x': 1., 'max_y': 1.}, 'edges': []},
            {'surface_id': 'mezz', 'z_reference': 1., 'traversability': 'supported',
             'footprint': {'min_x': -.5, 'min_y': -.5, 'max_x': .5, 'max_y': .5}, 'edges': []},
        ],
    }


class SurfaceTests(unittest.TestCase):
    def test_single_layer_infers_z(self):
        map_value = {'surfaces': [two_layer()['surfaces'][0]]}
        surface, z = surface_resolver.resolve_surface(map_value, 0., 0.)
        self.assertEqual((surface['surface_id'], z), ('ground', 0.))

    def test_multilayer_requires_hint_or_z(self):
        with self.assertRaises(surface_resolver.SurfaceResolutionError) as caught:
            surface_resolver.resolve_surface(two_layer(), 0., 0.)
        self.assertEqual(caught.exception.code, 'E_SURFACE_AMBIGUOUS')
        self.assertEqual(len(caught.exception.candidates), 2)

    def test_hint_selects_layer(self):
        surface, z = surface_resolver.resolve_surface(two_layer(), 0., 0., hint='mezz')
        self.assertEqual(surface['surface_id'], 'mezz')
        self.assertEqual(z, 1.)
        with self.assertRaises(surface_resolver.SurfaceResolutionError):
            surface_resolver.resolve_surface(two_layer(), 0., 0., z=0., hint='mezz')

    def test_task_fills_start_and_goal_z(self):
        task = {
            'start': {'xyz': [0.9, 0.9, None], 'yaw': 0},
            'goal': {'xyz': [0., 0., None], 'yaw': 0, 'surface_hint': 'mezz',
                     'surface_disambiguation': 'explicit_hint'},
        }
        resolved, meta = surface_resolver.resolve_task(task, two_layer())
        self.assertEqual(resolved['start']['xyz'][2], 0.)
        self.assertEqual(resolved['goal']['xyz'][2], 1.)
        self.assertEqual(meta['goal_surface'], 'mezz')

    def test_pick_goal_returns_surface_z(self):
        picked = surface_resolver.pick_goal(two_layer(), (0., 0., 1.), hint='mezz')
        self.assertEqual((picked['z'], picked['surface_id']), (1., 'mezz'))


class PlannerHintTests(unittest.TestCase):
    def test_hint_beats_nearest_z(self):
        value = json.loads((HERE.parent / 'contracts' / 'fixtures'
                            / 'map_ml_two_surfaces.json').read_text())
        with_hint = support_planner.plan(value, (0., 0., 0.), (3., 2., .2),
                                         budget=20000, goal_surface_id='mezz')
        without_hint = support_planner.plan(value, (0., 0., 0.), (3., 2., .2),
                                            budget=20000)
        self.assertEqual(with_hint['path'][-1][3], 'mezz')
        self.assertEqual(without_hint['path'][-1][3], 'ground')


class RegistryTests(unittest.TestCase):
    def test_registry_resolves_generated_map(self):
        registry = map_registry.load_registry()
        entry = map_registry.resolve_entry(registry, 'nist_sensor', 2)
        value, path, sha = map_registry.load_map(entry)
        self.assertEqual(value['map_id'], 'nist_sensor')
        self.assertEqual(value['map_version'], 2)
        self.assertEqual(len(value['surfaces']), 6)
        self.assertTrue(path.is_file())

    def test_missing_version_is_explicit(self):
        registry = map_registry.load_registry()
        with self.assertRaises(map_registry.RegistryError):
            map_registry.resolve_entry(registry, 'nist_sensor', 99)


class AppTests(unittest.TestCase):
    def test_app_resolves_task_and_graph(self):
        app_path = HERE / 'fixtures' / 'apps' / 'nist_sensor_3d.yaml'
        resolved = app_config.load_app(app_path)
        self.assertEqual(resolved['map_entry']['map_id'], 'nist_sensor')
        self.assertEqual(resolved['task_resolved']['start']['xyz'][2], -1e-06)
        self.assertEqual(resolved['task_resolved']['goal']['xyz'][2], 0.888142)
        self.assertTrue(resolved['graph_path'].is_file())

    def test_lio_app_selects_loc_graph(self):
        app_path = HERE / 'fixtures' / 'apps' / 'nist_sensor_3d_lio.yaml'
        resolved = app_config.load_app(app_path)
        self.assertEqual(resolved['app']['localization']['source'], 'faster_lio')
        self.assertEqual(resolved['graph_path'].name, 'graph_3d_gazebo_loc.yaml')


class SensorMapLayerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _, cls.value, _, _ = map_registry.get_map('nist_sensor', 2)

    def test_overlap_keeps_both_layers(self):
        candidates = surface_resolver.candidates_at(self.value, 1.0, -1.05)
        self.assertEqual([item['surface_id'] for item in candidates], ['m1', 'm2'])
        self.assertAlmostEqual(candidates[0]['z_reference'], .198474)
        self.assertAlmostEqual(candidates[1]['z_reference'], .371516)

    def test_ambiguity_is_explicit_without_hint_or_z(self):
        with self.assertRaises(surface_resolver.SurfaceResolutionError) as caught:
            surface_resolver.resolve_surface(self.value, 1.0, -1.05)
        self.assertEqual(caught.exception.code, 'E_SURFACE_AMBIGUOUS')
        self.assertEqual([item['surface_id'] for item in caught.exception.candidates],
                         ['m1', 'm2'])

    def test_hint_and_z_select_upper_surface(self):
        hinted, z_hint = surface_resolver.resolve_surface(
            self.value, 1.0, -1.05, hint='m2')
        self.assertEqual(hinted['surface_id'], 'm2')
        self.assertAlmostEqual(z_hint, .371516)
        by_z, z_value = surface_resolver.resolve_surface(
            self.value, 1.0, -1.05, z=.3715)
        self.assertEqual(by_z['surface_id'], 'm2')
        self.assertAlmostEqual(z_value, .371516)

    def test_task_resolution_fills_generated_surface_z(self):
        task = {'map': {'map_id': 'nist_sensor', 'map_version': 2},
                'start': {'xyz': [0., -3.06, None], 'yaw': 0.},
                'goal': {'xyz': [1.0, -1.05, None], 'yaw': 0.,
                         'surface_hint': 'm2', 'surface_disambiguation': 'explicit_hint'}}
        resolved, meta = surface_resolver.resolve_task(task, self.value)
        self.assertEqual(meta['start_surface'], 'm0')
        self.assertEqual(meta['goal_surface'], 'm2')
        self.assertAlmostEqual(meta['goal_surface_z'], .371516)
        self.assertAlmostEqual(resolved['goal']['xyz'][2], .371516)
        self.assertEqual(meta['goal_candidates'], [
            {'surface_id': 'm1', 'z_reference': .198474},
            {'surface_id': 'm2', 'z_reference': .371516}])

    def test_planner_selects_hinted_upper_surface(self):
        result = support_planner.plan(
            self.value, (0., -3.06, 0.), (1.0, -1.05, .371516),
            start_surface_id='m0', goal_surface_id='m2')
        self.assertTrue(result['ok'])
        self.assertEqual(result['path'][0][3], 'm0')
        self.assertEqual(result['path'][-1][3], 'm2')
        self.assertAlmostEqual(result['path'][-1][2], .371516)
        self.assertIn({'from': 'm1', 'to': 'm2'}, result['transitions'])
        lower = support_planner.plan(
            self.value, (0., -3.06, 0.), (1.0, -1.05, .198474),
            start_surface_id='m0')
        self.assertEqual(lower['path'][-1][3], 'm1')
        self.assertAlmostEqual(lower['path'][-1][2], .198474)


class VizTests(unittest.TestCase):
    def test_surface_and_obstacle_specs(self):
        value = two_layer()
        value['obstacles'] = [{'obstacle_id': 'box', 'min': [0., 0., 0.], 'max': [.2, .2, .5]}]
        specs = support_map_viz.surface_specs(value)
        self.assertEqual(len(specs), 2)
        self.assertEqual(specs[1]['label'], 'mezz z=1.000')
        self.assertEqual(len(support_map_viz.obstacle_specs(value)), 1)

    def test_trajectory_points(self):
        payload = {'trajectory': [[0, 0, 0, 1], [0, 0, .2, 2], {'x': 0, 'y': 1, 'z': .2}]}
        self.assertEqual(support_map_viz.trajectory_points(payload),
                         [(0., 0., 0.), (0., 0., .2), (0., 1., .2)])


if __name__ == '__main__':
    unittest.main()
