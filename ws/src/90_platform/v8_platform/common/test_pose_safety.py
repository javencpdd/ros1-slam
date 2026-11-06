"""Numerical and clock contracts; no ROS or process is needed."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.pose_safety import (PoseError, PoseFreshness, finite_number, finite_vector,
                                map_to_body_planar, normalized_quaternion, validated_yaw)


class GeometryTests(unittest.TestCase):
    def test_yaw_rotation_sign_and_inverse(self):
        for yaw in (0., math.pi / 2., -math.pi / 2., .37):
            q = (0., 0., math.sin(yaw / 2.), math.cos(yaw / 2.))
            vx, vy, vz = map_to_body_planar((1., 0., 0.), q)
            self.assertAlmostEqual(vx, math.cos(yaw))
            self.assertAlmostEqual(vy, -math.sin(yaw))
            self.assertEqual(vz, 0.)
            self.assertAlmostEqual(vx * vx + vy * vy, 1.)
            self.assertAlmostEqual(math.cos(yaw) * vx - math.sin(yaw) * vy, 1.)

    def test_sign_and_scale_of_quaternion_represent_same_rotation(self):
        q = (0., 0., math.sin(.3), math.cos(.3))
        self.assertAlmostEqual(validated_yaw(q), .6)
        for scale in (-1., 3., 1e200):
            self.assertAlmostEqual(validated_yaw(tuple(scale * v for v in q)), .6)
        self.assertAlmostEqual(sum(v*v for v in normalized_quaternion(q)), 1.)

    def test_zero_nonfinite_boolean_and_wrong_shapes_rejected(self):
        for q in ((0.,0.,0.,0.), (0.,0.,math.nan,1.), (0.,0.,math.inf,1.),
                  (False,0.,0.,1.), (0.,0.,1.), None):
            with self.subTest(q=q), self.assertRaises(PoseError):
                validated_yaw(q)
        for velocity in ((True,0.,0.), (0.,math.inf,0.), None, (0.,0.)):
            with self.assertRaises(PoseError):
                map_to_body_planar(velocity, (0.,0.,0.,1.))
        with self.assertRaises(PoseError):
            finite_vector((0., math.nan, 0.), 3)

    def test_finite_input_rotation_overflow_is_rejected(self):
        q=(0.,0.,math.sin(math.pi/8.),math.cos(math.pi/8.))
        with self.assertRaisesRegex(PoseError, 'VELOCITY'):
            map_to_body_planar((1.7e308,1.7e308,0.),q)

    def test_unrepresentable_integer_and_conversion_overflow_are_input_errors(self):
        class OverflowFloat(int):
            def __float__(self):raise OverflowError('owned numeric conversion')
        for value in (10**1000,-10**1000,OverflowFloat(1)):
            self.assertFalse(finite_number(value))
            with self.assertRaises(PoseError):finite_vector((value,0.,0.),3)
            with self.assertRaises(PoseError):normalized_quaternion((value,0.,0.,1.))
            self.assertFalse(PoseFreshness(.5).observe(value,100.,10.)[0]['ok'])
        self.assertTrue(finite_number(10**100))

    def test_vertical_arithmetic_boundary_is_explicit(self):
        q=(0.,0.,0.,1.)
        for vz in (0., 1e-6, -1e-6):
            self.assertEqual(map_to_body_planar((1., 0., vz), q), (1.,0.,0.))
        for vz in (1.000001e-6, -1.000001e-6, .1):
            with self.assertRaisesRegex(PoseError, 'VERTICAL'):
                map_to_body_planar((1.,0.,vz),q)


class FreshnessTests(unittest.TestCase):
    def test_fresh_advancing_source_is_healthy(self):
        guard=PoseFreshness(.5)
        for stamp,wall in ((10.,1.),(10.1,1.1),(10.2,1.2)):
            state, advanced=guard.observe(stamp,stamp+.01,wall)
            self.assertTrue(state['ok']); self.assertTrue(advanced)

    def test_duplicate_source_does_not_renew_monotonic_lease(self):
        guard=PoseFreshness(.5); guard.observe(10.,10.,1.)
        state,advanced=guard.observe(10.,10.,1.49)
        self.assertTrue(state['ok']); self.assertFalse(advanced)
        state,advanced=guard.observe(10.,10.,1.5)
        self.assertFalse(state['ok']); self.assertEqual(state['reason'],'E_POSE_NO_PROGRESS')
        self.assertEqual(guard.progress_at,1.)
        self.assertFalse(guard.observe(10.,10.,1.51)[0]['ok'])
        self.assertTrue(guard.observe(10.1,10.1,1.52)[0]['ok'])

    def test_source_age_boundary_is_independent_of_delivery(self):
        guard=PoseFreshness(.5)
        state,advanced=guard.observe(10.,10.5,1.)
        self.assertFalse(state['ok']);self.assertFalse(advanced)
        self.assertEqual(state['reason'],'E_POSE_STALE')

    def test_zero_future_nonfinite_sources_are_invalid(self):
        for source in (0.,-1.,10.01, math.nan, math.inf,True):
            guard=PoseFreshness(.5)
            self.assertFalse(guard.observe(source,10.,1.)[0]['ok'])

    def test_stamp_regression_cannot_recover_by_duplicate(self):
        guard=PoseFreshness(.5);guard.observe(10.,10.,1.)
        self.assertEqual(guard.observe(9.9,10.,1.1)[0]['reason'],'E_POSE_STAMP_BACKWARD')
        self.assertFalse(guard.observe(10.,10.,1.2)[0]['ok'])
        self.assertTrue(guard.observe(10.1,10.1,1.3)[0]['ok'])

    def test_ros_rollback_invalidates_one_cycle_and_changes_epoch(self):
        guard=PoseFreshness(.5);guard.observe(10.,10.,1.)
        status,advanced=guard.observe(5.,5.,1.1)
        self.assertFalse(status['ok']);self.assertFalse(advanced)
        self.assertEqual(status['reason'],'E_POSE_CLOCK_BACKWARD')
        self.assertEqual(guard.epoch,1);self.assertIsNone(guard.source_stamp)
        self.assertTrue(guard.observe(5.1,5.1,1.2)[0]['ok'])

    def test_rejected_future_sample_cannot_heal_only_when_clock_catches_up(self):
        guard=PoseFreshness(.5);guard.observe(100.,100.,10.)
        self.assertFalse(guard.observe(101.,100.1,10.1)[0]['ok'])
        status,advanced=guard.observe(101.,101.1,11.1)
        self.assertFalse(status['ok']);self.assertFalse(advanced)
        self.assertEqual(guard.progress_at,10.)
        self.assertEqual(guard.observed_stamp,101.)
        self.assertTrue(guard.observe(101.2,101.2,11.2)[0]['ok'])

    def test_new_epoch_can_advance_through_previous_numeric_stamp(self):
        guard=PoseFreshness(.5);guard.observe(100.,100.,10.)
        self.assertFalse(guard.check(20.,10.1)['ok'])
        for stamp,wall in ((20.1,10.2),(99.9,10.3),(100.,10.4),(100.1,10.5)):
            self.assertTrue(guard.observe(stamp,stamp,wall)[0]['ok'])

    def test_pre_reset_cache_cannot_heal_on_first_observation_after_clock_catchup(self):
        guard=PoseFreshness(.5);guard.observe(100.,100.,10.)
        self.assertFalse(guard.check(20.,10.1)['ok'])
        self.assertFalse(guard.check(10.,10.2)['ok'])
        self.assertEqual(guard.observe(100.,100.,90.)[0]['reason'],'E_POSE_EPOCH_REPLAY')
        self.assertFalse(guard.observe(100.,100.1,90.1)[0]['ok'])
        self.assertTrue(guard.observe(100.2,100.2,90.2)[0]['ok'])

    def test_new_epoch_continuous_progress_clears_only_transient_cache_quarantine(self):
        guard=PoseFreshness(.5);guard.observe(100.,100.,10.)
        self.assertFalse(guard.check(20.,10.1)['ok'])
        for tick in range(201,1002):
            stamp=tick/10.
            with self.subTest(stamp=stamp):
                self.assertTrue(guard.observe(stamp,stamp,10.2+(tick-201)/10.)[0]['ok'])
        self.assertFalse(guard.epoch_cached_stamps)

    def test_previous_accepted_and_rejected_cache_both_quarantined_after_rollback(self):
        for cached in (100.,101.):
            guard=PoseFreshness(.5);guard.observe(100.,100.,10.)
            guard.observe(101.,100.1,10.1)
            guard.check(20.,10.2)
            self.assertFalse(guard.observe(cached,cached,90.)[0]['ok'])
            self.assertTrue(guard.observe(cached+.1,cached+.1,90.1)[0]['ok'])

    def test_rejected_geometry_does_not_release_epoch_cache_or_renew_progress(self):
        guard=PoseFreshness(.5);guard.observe(100.,100.,10.)
        guard.check(20.,10.1)
        status,advanced=guard.observe(20.1,20.1,10.2,rejection='E_POSE_FRAME')
        self.assertFalse(status['ok']);self.assertFalse(advanced)
        self.assertIsNone(guard.source_stamp);self.assertIsNone(guard.progress_at)
        self.assertFalse(guard.observe(100.,100.,90.)[0]['ok'])
        self.assertTrue(guard.observe(100.1,100.1,90.1)[0]['ok'])

    def test_invalid_frame_fault_requires_source_progress(self):
        guard=PoseFreshness(.5);guard.observe(10.,10.,1.)
        guard.invalidate('E_POSE_FRAME')
        self.assertFalse(guard.observe(10.,10.,1.1)[0]['ok'])
        self.assertTrue(guard.observe(10.1,10.1,1.2)[0]['ok'])

    def test_invalid_clock_and_monotonic_backwards(self):
        for value in (0.,-1.,math.nan,math.inf,True):
            self.assertFalse(PoseFreshness(.5).observe(1.,value,2.)[0]['ok'])
        guard=PoseFreshness(.5);guard.observe(10.,10.,2.)
        self.assertEqual(guard.check(10.1,1.9)['reason'],'E_POSE_WALL_BACKWARD')
        for value in (0.,-1.,math.nan,math.inf,True):
            with self.assertRaises(ValueError):PoseFreshness(value)


if __name__=='__main__':unittest.main()
