"""Directed controls for the proved simulation-only old-contact egress."""
import json
import sys
import threading
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
PLATFORM = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(PLATFORM / 'representation'))
import contact_egress
import local_native as native
import ros_node as node
from test_pose_command_adapter import Data, Publisher, Stamp, Stamped, Twist, pose


def scene_map():
    path = PLATFORM.parents[1] / '95_simulation/lite3_gazebo_sim/scenarios/two_floor_nav/maps/open.json'
    return json.loads(path.read_text())


def config():
    return native.Config(resolution=.4, tick_s=1.2, horizon_s=8.4, validity_s=20.,
                         radius_m=.3, height_m=.5, clearance_m=.05, goal_tolerance_m=.2)


def descending_result(value):
    return dict(ok=True, reason='horizon',
                trajectory=[(-3.199999, 1.2, 1.6, 10.),
                            (-3.199999, 1.2, 1.44, 11.),
                            (-2.8, 1.2, 1.44, 12.)],
                surfaces=['step10', 'step09', 'step09'],
                map_id=value['map_id'], map_version=value['map_version'],
                created_at=10., valid_until=18.)


def pose_at(x, y, z, stamp):
    message = pose(stamp=stamp)
    message.pose.position.x, message.pose.position.y, message.pose.position.z = x, y, z
    return message


class ContactEgressGeometryTests(unittest.TestCase):
    def setUp(self):
        self.value = scene_map()
        self.config = config()
        self.result = descending_result(self.value)

    def geometry(self, map_value=None, result=None, cfg=None):
        return contact_egress.ContactEgress.create(map_value or self.value,
                                                   result or self.result,
                                                   cfg or self.config)

    def test_descending_step_geometry_binds_identity_and_outward_direction(self):
        geometry = self.geometry()
        self.assertIsNotNone(geometry)
        self.assertEqual((geometry.source, geometry.landed, geometry.index), ('step10', 'step09', 1))
        self.assertEqual((geometry.axis, geometry.side), (0, 1))
        self.assertAlmostEqual(geometry.plane, -3.2)
        self.assertTrue(geometry.outward(self.result['trajectory'][1], self.result['trajectory'][2], 0.))
        self.assertFalse(geometry.outward(self.result['trajectory'][1], (-3.5, 1.2, 1.44), 0.))
        self.assertTrue(geometry.valid_for(self.value, self.result, self.config))
        changed = dict(self.result, created_at=10.01)
        self.assertFalse(geometry.valid_for(self.value, changed, self.config))

    def test_non_descending_wrong_next_or_missing_step_contract_is_rejected(self):
        ascending = dict(self.result, surfaces=['step09', 'step10', 'step10'],
                         trajectory=[(-3.199999, 1.2, 1.44, 10.),
                                     (-3.199999, 1.2, 1.6, 11.),
                                     (-3.6, 1.2, 1.6, 12.)])
        self.assertIsNone(self.geometry(result=ascending))
        wrong_next = dict(self.result, surfaces=['step10', 'step09', 'step08'])
        self.assertIsNone(self.geometry(result=wrong_next))
        too_high_edge = json.loads(json.dumps(self.value))
        surface = next(s for s in too_high_edge['surfaces'] if s['surface_id'] == 'step09')
        surface['edges'][0]['max_step_height_m'] = .1
        self.assertIsNone(self.geometry(map_value=too_high_edge))

    def test_open_permission_allows_only_outward_old_contact_until_full_clear(self):
        geometry = self.geometry()
        raw = (-3.19, 1.2, 1.44)
        predicted = (-3.18, 1.2, 1.44)
        previous = (.1, 0., 0.)
        window = geometry.window(raw, predicted, previous, .005, native._support_contains_segment)
        self.assertIsNotNone(window)
        self.assertTrue(window.allow_old)
        old = next(s for s in self.value['surfaces'] if s['surface_id'] == 'step10')
        collision_config = native.replace(self.config, radius_m=self.config.radius_m + .005)
        self.assertTrue(window.permits(self.value, old, raw, predicted, collision_config))
        boundary_window = geometry.window((-3.199999, 1.2, 1.44), (-3.19, 1.2, 1.44),
                                          previous, .005, native._support_contains_segment)
        self.assertIsNotNone(boundary_window)
        self.assertTrue(boundary_window.allow_old)

        clear_raw = (-2.7, 1.2, 1.44)
        clear_predicted = (-2.69, 1.2, 1.44)
        clear_window = geometry.window(clear_raw, clear_predicted, (.01, 0., 0.), .005,
                                       native._support_contains_segment)
        self.assertIsNotNone(clear_window)
        self.assertFalse(clear_window.allow_old)
        self.assertFalse(clear_window.permits(self.value, old, clear_raw, clear_predicted,
                                              collision_config))
        geometry.close_permission()
        self.assertIsNone(geometry.window(raw, predicted, previous, .005,
                                          native._support_contains_segment))
        self.assertIsNotNone(geometry.window(clear_raw, clear_predicted, (.01, 0., 0.), .005,
                                             native._support_contains_segment))

    def test_tracking_uses_one_checked_window_and_never_reuses_retired_context(self):
        value, result, cfg = self.value, self.result, self.config
        raw = predicted = (-3.19, 1.2, 1.44)
        previous = (.05, 0., 0.)
        landed_leg = dict(result,
                          trajectory=[result['trajectory'][1], result['trajectory'][2],
                                      result['trajectory'][2]],
                          surfaces=['step09', 'step09'])
        ordinary = native.track_segment(value, landed_leg, predicted, 11.2, [], cfg,
                                        feedback_pose=raw, uncertainty_m=.005,
                                        previous_velocity=previous)
        self.assertEqual(ordinary['reason'], 'E_LOCAL_COLLISION')
        self.assertEqual(ordinary['collision'], 'static:step10')
        geometry = self.geometry()
        permitted = native.track_segment(value, result, predicted, 11.2, [], cfg,
                                         feedback_pose=raw, uncertainty_m=.005,
                                         previous_velocity=previous, contact_egress=geometry)
        self.assertEqual(permitted['reason'], 'track', permitted)
        self.assertGreater(permitted['velocity'][0], 0.)
        geometry.retire()
        retired = native.track_segment(value, result, predicted, 11.2, [], cfg,
                                       feedback_pose=raw, uncertainty_m=.005,
                                       previous_velocity=previous, contact_egress=geometry)
        self.assertEqual(retired['reason'], 'E_LOCAL_INPUT')


class ContactEgressControllerTests(unittest.TestCase):
    def fixture(self, enabled=True):
        self.clock = {'ros': 100., 'wall': 10.}
        self.map_value = scene_map()
        self.cfg = config()
        self.result = descending_result(self.map_value)
        self.result['trajectory'] = [(-3.199999, 1.2, 1.6, 99.8),
                                     (-3.199999, 1.2, 1.44, 100.8),
                                     (-2.8, 1.2, 1.44, 101.8)]
        self.result['created_at'], self.result['valid_until'] = 99.8, 108.
        self.shared = dict(pose=None, goal=(-2.8, 1.2, 1.44), pose_at=-1e100,
                           pose_ros_stamp=0., obstacles=[], obstacles_at=10.,
                           localization_at=10., localization_ok=True, result=self.result,
                           result_at=9.9, map=self.map_value, generation=0, result_generation=0,
                           last_error='ok', stop_latched=False, stop_reason=None,
                           plan_ms=None, plan_ms_max=0., map_version=self.map_value['map_version'])
        self.lock = threading.Lock()
        self.input = node.LocalPoseInput(self.shared, 1., False)
        self.assertIsNone(self.input.accept(pose_at(-3.3, 1.2, 1.6, 100.), 100., 10.))
        self.command, self.twist, self.status = Publisher(), None, Publisher()
        self.control = node.LocalControl(self.shared, self.input, self.lock, self.cfg, 1., 7.5,
                                         self.command, self.twist, self.status,
                                         (Stamped, Twist, Data), sim_contact_egress=enabled)

    def move(self, x, y, z, ros_time, wall_time):
        self.assertIsNone(self.input.accept(pose_at(x, y, z, ros_time), ros_time, wall_time))
        self.clock.update(ros=ros_time, wall=wall_time)

    def test_sim_only_witness_completion_activates_immediate_landed_leg(self):
        self.fixture(True)
        first = self.control.tick(lambda: self.clock['ros'], lambda: self.clock['wall'], Stamp)
        self.assertEqual(first['reason'], 'track', first)
        self.assertIsNotNone(self.control._transfer_witness)
        self.move(-3.199999, 1.2, 1.44, 100.1, 10.1)
        self.control.history[:] = []
        self.control.previous_command = (0., 0., 0.)
        held = self.control.tick(lambda: self.clock['ros'], lambda: self.clock['wall'], Stamp)
        self.assertEqual(held['reason'], 'hold', held)
        self.assertFalse(held.get('complete', False))
        self.assertIsNotNone(self.control._egress)
        status = json.loads(self.status.messages[-1].data)
        self.assertEqual(status['contact_egress']['source'], 'step10')
        third = self.control.tick(lambda: self.clock['ros'], lambda: self.clock['wall'], Stamp)
        self.assertEqual(third['reason'], 'track', third)
        self.assertGreater(third['velocity'][0], 0.)

    def test_default_disabled_never_activates_contact_egress(self):
        self.fixture(False)
        first = self.control.tick(lambda: self.clock['ros'], lambda: self.clock['wall'], Stamp)
        self.assertEqual(first['reason'], 'track')
        self.move(-3.199999, 1.2, 1.44, 100.1, 10.1)
        self.control.history[:] = []
        self.control.previous_command = (0., 0., 0.)
        second = self.control.tick(lambda: self.clock['ros'], lambda: self.clock['wall'], Stamp)
        self.assertIsNone(self.control._egress)
        self.assertNotEqual(second['reason'], 'hold')

    def test_egress_cannot_survive_map_identity_change(self):
        self.fixture(True)
        self.control.tick(lambda: self.clock['ros'], lambda: self.clock['wall'], Stamp)
        self.move(-3.199999, 1.2, 1.44, 100.1, 10.1)
        self.control.history[:] = []
        self.control.previous_command = (0., 0., 0.)
        self.control.tick(lambda: self.clock['ros'], lambda: self.clock['wall'], Stamp)
        self.assertIsNotNone(self.control._egress)
        self.shared['map'] = dict(self.map_value, map_version=self.map_value['map_version'] + 1)
        out = self.control.tick(lambda: self.clock['ros'], lambda: self.clock['wall'], Stamp)
        self.assertIsNone(self.control._egress)
        self.assertEqual(out['reason'], 'E_LOCAL_MAP_STALE')


if __name__ == '__main__':
    unittest.main()
