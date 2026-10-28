"""Regression tests for obstacle contracts, swept edges and endpoint connectors."""
import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
for folder in (HERE, HERE.parent / 'contracts', HERE.parent / 'representation',
               HERE.parent / 'regression'):
    sys.path.insert(0, str(folder))
import validate as contract
import support_map
import support_planner as planner
import ros_node
from global_obstacles import box, derived_case, evaluate_route, run


def mapping(obstacles=(), resolution=.1, footprint=None):
    value = json.loads((HERE.parent / 'contracts/fixtures/map_single_floor.json').read_text())
    value.update(schema_version=2, resolution=resolution, obstacles=copy.deepcopy(list(obstacles)))
    value['surfaces'][0].update(surface_id='ground', z_reference=0,
                              footprint=footprint or {'min_x': -2, 'max_x': 2,
                                                      'min_y': -1, 'max_y': 1}, edges=[])
    return value


def plan(value, start=(-1.5, 0, 0), goal=(1.5, 0, 0), **kwargs):
    return planner.plan(value, start, goal, robot_radius_m=.10, robot_height_m=.30, **kwargs)


class ObstacleTests(unittest.TestCase):
    def assert_reason(self, expected, function):
        with self.assertRaises((planner.PlannerError, contract.ContractError)) as caught:
            function()
        self.assertEqual(caught.exception.code, expected)

    def test_v2_requires_opt_in_and_old_schema_stays_frozen(self):
        value = mapping()
        contract.validate('map', value, allow_obstacles=True)
        self.assert_reason('E_VERSION', lambda: contract.validate('map', value))
        value['schema_version'] = 1
        self.assert_reason('E_SCHEMA_UNKNOWN_FIELD', lambda: contract.validate('map', value))

    def test_obstacle_shape_bounds_finite_and_duplicate_rejected(self):
        item = box('box', (0, 0, 0), (.2, .2, .2))
        for mutate, expected in (
                (lambda value: value.pop('obstacles'), 'E_SCHEMA_REQUIRED'),
                (lambda value: value['obstacles'][0]['min'].pop(), 'E_SCHEMA_MINITEMS'),
                (lambda value: value['obstacles'][0]['min'].__setitem__(0, math.nan), 'E_SCHEMA_NONFINITE'),
                (lambda value: value['obstacles'][0]['min'].__setitem__(0, True), 'E_SCHEMA_TYPE'),
                (lambda value: value['obstacles'][0]['max'].__setitem__(0, 0), 'E_MAP_OBSTACLE_BOUNDS'),
                (lambda value: value['obstacles'][0].update(frame='other'), 'E_SCHEMA_UNKNOWN_FIELD'),
                (lambda value: value['obstacles'].append(copy.deepcopy(item)), 'E_MAP_OBSTACLE_DUP')):
            value = mapping([item])
            mutate(value)
            with self.subTest(expected=expected):
                self.assert_reason(expected, lambda: contract.validate('map', value, allow_obstacles=True))

    def test_builder_preserves_obstacles_without_aliasing(self):
        data = mapping([box('box', (0, 0, 0), (.2, .2, .2))])
        value = support_map.build_support_map('test', .1, [0, 0, 0], data['surfaces'], None,
                                              data['source'], obstacles=data['obstacles'])
        self.assertEqual(value['schema_version'], 2)
        self.assertFalse(value['multiple_surfaces_per_xy'])
        data['obstacles'][0]['max'][0] = 99
        self.assertEqual(value['obstacles'][0]['max'][0], .2)

    def test_body_envelope_must_be_explicit(self):
        value = mapping([box('box', (0, -.2, 0), (.2, .2, .3))])
        for kwargs in ({}, {'robot_radius_m': .1}, {'robot_height_m': .3},
                       {'robot_radius_m': True, 'robot_height_m': .3},
                       {'robot_radius_m': .1, 'robot_height_m': 0}):
            with self.subTest(kwargs=kwargs):
                self.assert_reason('E_PLAN_INPUT', lambda: planner.plan(value, (-1, 0, 0), (1, 0, 0), **kwargs))

    def test_start_and_goal_body_collision_refused(self):
        value = mapping([box('box', (-.1, -.1, .1), (.1, .1, .3))])
        for start, goal in (((0, 0, 0), (1, 0, 0)), ((-1, 0, 0), (0, 0, 0))):
            self.assert_reason('E_PLAN_OBSTACLE', lambda: plan(value, start, goal))

    def test_thin_wall_between_cell_centers_blocks_edges(self):
        value = mapping([box('thin', (-.001, -1.1, 0), (.001, 1.1, .4))], resolution=.5)
        self.assert_reason('E_PLAN_NO_ROUTE', lambda: plan(value))

    def test_same_xy_obstacle_above_and_below_layer_are_not_flattened(self):
        for item in (box('overhead', (-.1, -2, .31), (.1, 2, .5)),
                     box('buried', (-.1, -2, -.2), (.1, 2, 0))):
            value = mapping([item])
            route = plan(value)
            self.assertTrue(evaluate_route(value, route, .1, .3)['ok'])

    def test_diagonal_corner_cannot_cut_box(self):
        value = mapping([box('corner', (-.12, -.12, 0), (.12, .12, .4))], resolution=.5)
        route = plan(value, (-1, -1, 0), (1, .75, 0))
        self.assertTrue(evaluate_route(value, route, .1, .3)['ok'])
        unsafe = {'path': [(-1, -1, 0, 'ground'), (1, 1, 0, 'ground')]}
        self.assertFalse(evaluate_route(value, unsafe, .1, .3)['ok'])

    def test_transition_segment_cannot_tunnel_through_floating_box(self):
        value = mapping([box('between_layers', (-3, -2, .4), (3, 2, .5))])
        upper = copy.deepcopy(value['surfaces'][0])
        upper.update(surface_id='upper', z_reference=.6,
                     edges=[{'to': 'ground', 'kind': 'step', 'max_step_height_m': .7,
                             'bidirectional': True}])
        value['surfaces'][0]['edges'] = [{'to': 'upper', 'kind': 'step',
                                         'max_step_height_m': .7, 'bidirectional': True}]
        value.update(surfaces=[value['surfaces'][0], upper], multiple_surfaces_per_xy=True)
        # Both endpoint envelopes are collision free; only the vertical edge hits.
        self.assert_reason('E_PLAN_NO_ROUTE', lambda: plan(value, (-1, 0, 0), (1, 0, .6)))

    def test_endpoint_connector_cannot_snap_across_wall(self):
        value = mapping([box('thin', (.09, -1.1, 0), (.11, 1.1, .4))], resolution=.5)
        self.assert_reason('E_PLAN_NO_ROUTE', lambda: planner.plan(
            value, (.05, .05, 0), (1, 0, 0), robot_radius_m=0, robot_height_m=.3))

    def test_unaligned_footprint_never_returns_outside_grid_center(self):
        value = mapping(footprint={'min_x': 0, 'max_x': .12, 'min_y': 0, 'max_y': .2})
        route = plan(value, (.02, .05, 0), (.11, .05, 0))
        self.assertTrue(all(0 <= x <= .12 and 0 <= y <= .2 for x, y, _, _ in route['path']))

    def test_invalid_zero_or_false_resolution_does_not_default(self):
        for resolution in (0, False, math.nan, math.inf, -.1):
            with self.subTest(resolution=resolution):
                self.assert_reason('E_PLAN_INPUT', lambda: plan(mapping(), resolution=resolution))

    def test_serialization_does_not_round_safe_route_into_box(self):
        resolution = .123456789
        y = .5 * resolution
        value = mapping([box('edge', (0, .02, 0), (.25, y - .01 - 2e-7, .3))],
                        resolution=resolution,
                        footprint={'min_x': 0, 'max_x': .4, 'min_y': 0, 'max_y': .2})
        route = planner.plan(value, (.5 * resolution, y, 0), (1.5 * resolution, y, 0),
                             robot_radius_m=.01, robot_height_m=.2)
        serialized = json.loads(json.dumps(route))
        self.assertTrue(evaluate_route(value, serialized, .01, .2)['ok'])

    def test_ros_request_preserves_boxes_and_body_policy(self):
        value = mapping([box('wall', (-.001, -2, 0), (.001, 2, .4))])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'map.json'
            path.write_text(json.dumps(value))
            request = ros_node.to_request(str(path), [-1, 0, 0], [1, 0, 0], 1000, None,
                                          robot_radius_m=.1, robot_height_m=.3)
            self.assert_reason('E_PLAN_NO_ROUTE', lambda: ros_node.plan_route(request))

    def test_derived_scenarios_and_unaware_output_negative_controls(self):
        result = run()
        self.assertTrue(result['ok'], result)
        self.assertEqual((result['passed'], result['total']), (9, 9))

    def test_independent_checker_rejects_forged_same_height_surface_id(self):
        value, start, goal = derived_case('SC05', 'clear')
        route = plan(value, start, goal)
        point = route['path'][-1]
        route['path'][-1] = (*point[:3], 'ground_a')
        self.assertFalse(evaluate_route(value, route, .1, .3, start, goal)['ok'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
