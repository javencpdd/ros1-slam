"""Countercontrols for actual command timing and committed-segment execution."""
import copy
import json
import math
import os
import subprocess
import sys
import unittest
from dataclasses import asdict, replace
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import local_native as native
import evaluate
import runtime_probe
from test_local_native import flat, layers, BOX
from test_local_native import ros_node


def record(value, points, config=None, execution=None):
    result = {'map_id': value['map_id'], 'map_version': value['map_version'],
              'trajectory': points, 'config': asdict(config or native.Config())}
    if execution is not None:
        result['execution_trace'] = execution
    return result


class ExecutionTests(unittest.TestCase):
    def test_timestamped_feedback_predicts_all_issued_command_changes(self):
        predicted = ros_node.feedback_at((.1, 0., .025), 1., 1.1,
                                        [(.9, (.5, 0., .125)), (1.05, (0., 0., 0.))])
        self.assertAlmostEqual(predicted[0], .125)
        self.assertAlmostEqual(predicted[2], .03125)
        with self.assertRaises(ValueError):
            ros_node.feedback_at((0., 0., 0.), 2., 1., [])

    def test_terminal_roundoff_keeps_raw_position_and_other_roof_red(self):
        value = layers()
        raw = (.2, 0., .15 - 2.5e-7)
        cfg = replace(native.Config(), tick_s=1.2, validity_s=1.5)
        result = native.plan(value, raw, (.6, 0., .15), config=cfg)
        self.assertEqual(result['trajectory'][0][:3], raw)
        evidence = record(value, [(*raw, 0.), (.6, 0., .15, 1.2)], cfg)
        self.assertTrue(evaluate.evaluate(value, evidence, [])['ok'])

    def test_delayed_feedback_terminal_controller_settles_before_original_ttl(self):
        value = layers()
        cfg = replace(native.Config(), tick_s=1.2, validity_s=1.5)
        result = native.plan(value, (-.4, 0., 0.), (.6, 0., .15), config=cfg)
        actual, velocity = (-.4, 0., 0.), (0., 0., 0.)
        feedback, feedback_stamp = actual, 0.
        history, pending = [], []
        execution, contacts = [(*actual, 0.)], [(*actual, 0.)]
        next_pose, next_control = 0., 0.
        done = False
        # Explicit 1 ms command transport and 10.5 ms pose sampling reproduce
        # the ROS loop's delayed terminal feedback without loosening 1e-6.
        for tick in range(15000):
            now = tick * .0001
            while pending and pending[0][0] <= now:
                _stamp, velocity = pending.pop(0)
            actual = tuple(actual[i] + velocity[i] * .0001 for i in range(3))
            if tick:
                sample = (*actual, now)
                execution.append(sample)
                if any(abs(actual[2] - s['z_reference']) <= 1e-6 for s in value['surfaces']):
                    contacts.append(sample)
            if now + 1e-10 >= next_pose:
                feedback, feedback_stamp = actual, now
                next_pose += .0105
            if now + 1e-10 >= next_control:
                estimate = ros_node.feedback_at(feedback, feedback_stamp, now, history)
                uncertainty = ros_node.feedback_uncertainty(feedback_stamp, now, history, cfg.command_latency_bound_s)
                out = native.track_segment(value, result, estimate, now, [], cfg, control_period_s=.0506,
                                           feedback_pose=feedback, uncertainty_m=uncertainty)
                history.append((now, tuple(out['velocity'])))
                pending.append((now + .001, tuple(out['velocity'])))
                next_control += .0506
                if out.get('complete'):
                    done = True
            if done and pending == [] and not any(abs(v) > 0 for v in velocity):
                break
        self.assertTrue(done, (actual, now))
        self.assertLess(now, result['valid_until'])
        self.assertLessEqual(native._distance(actual, result['trajectory'][1]), 1e-6)
        verdict = evaluate.evaluate(value, record(value, contacts, cfg, execution), [])
        self.assertTrue(verdict['ok'], (verdict['failures'], max(p[2] for p in execution),
                                       verdict['static_collisions'][:2]))
    def test_slightly_off_target_is_docked_not_completed(self):
        # General completion check: a pose 9.66e-6 past the deck target is not
        # certified; the controller commands toward the target. (This is not the
        # R841 step regression: the plan's first segment is 0.618 m, so the
        # zero-length terminal branch never runs here.)
        value = layers()
        cfg = replace(native.Config(), tick_s=1.2, validity_s=4.)
        result = native.plan(value, (-.4, 0., 0.), (.6, 0., .15), config=cfg)
        target = result['trajectory'][1]
        after = target[3] + .01
        self.assertLess(after, result['valid_until'])
        pose = (target[0] + 3.9e-5, 0., target[2] + 9.66e-6)
        out = native.track_segment(value, result, pose, after, [], config=cfg,
                                   feedback_pose=pose, uncertainty_m=0.)
        self.assertFalse(out.get('complete', False))
        self.assertLess(out['velocity'][2], 0.)
        settle = native.track_segment(value, result, target[:3], after, [], config=cfg)
        self.assertTrue(settle['complete'])
        self.assertEqual(settle['velocity'], [0., 0., 0.])

    def test_in_flight_uncertainty_blocks_contact_certification(self):
        # An in-flight command can still move the carrier off the support after
        # the pose sample; completion must wait until gap + uncertainty <= 1e-6.
        # Removing the uncertainty term from track_segment makes this red.
        value = layers()
        cfg = replace(native.Config(), tick_s=1.2, validity_s=4.)
        result = native.plan(value, (-.4, 0., 0.), (.6, 0., .15), config=cfg)
        target = result['trajectory'][1]
        when = result['created_at'] + .1
        blocked = native.track_segment(value, result, target[:3], when, [], config=cfg,
                                       feedback_pose=target[:3], uncertainty_m=5e-6)
        self.assertFalse(blocked.get('complete', False))
        settled = native.track_segment(value, result, target[:3], when, [], config=cfg,
                                       feedback_pose=target[:3], uncertainty_m=0.)
        self.assertTrue(settled['complete'])

    def test_receipt_integrator_keeps_hold_and_all_command_changes(self):
        position, stamp, velocity, samples = runtime_probe.integrate_received(
            (0., 0., 0.), 0., (0., 0., 0.), [(.4, (1., 0., 0.)), (.5, (0., 1., 0.))], .6)
        self.assertAlmostEqual(position[0], .1)
        self.assertAlmostEqual(position[1], .1)
        self.assertEqual(samples[0], (0., 0., 0., .4))
        self.assertEqual([p[3] for p in samples], [.4, .5, .6])
        with self.assertRaises(ValueError):
            runtime_probe.integrate_received(position, stamp, velocity, [(.59, (1., 0., 0.))], .7)

    def test_original_dynamic_failure_requires_actual_hold_trace(self):
        # Fixed numerical counterexample derived from the preserved failed ROS
        # artifact: omitting the wait starts lateral movement too early relative
        # to a box moving down at the ORIGINAL -0.4 m/s.
        value = flat()
        moving = dict(BOX, min=[.6, .8, -.05], max=[1., 1.2, .8], velocity=[0., -.4, 0.], stamp=0.)
        a, b = (.2, .565, 0., 2.304), (.4, .4422, 0., 2.927)
        compressed = evaluate.evaluate(value, record(value, [a, b]), [moving])
        self.assertFalse(compressed['ok'])
        self.assertIn('dynamic_collision', compressed['failures'])
        actual = [a, (.2, .565, 0., 2.427), b]
        complete = evaluate.evaluate(value, record(value, [a, b], execution=actual), [moving])
        self.assertTrue(complete['ok'], complete)
        # Feeding the actually unsafe path remains red; receipt logging is not
        # permission to suppress an independently observed collision.
        unsafe = evaluate.evaluate(value, record(value, [a, b], execution=[a, b]), [moving])
        self.assertFalse(unsafe['ok'])

    def test_delayed_execution_rechecks_moving_box_at_execution_time(self):
        value = flat()
        config = replace(native.Config(), resolution=.4, max_speed_mps=.8, validity_s=2.)
        moving = {'min': [.6, 1.5, -.05], 'max': [1., 1.7, .8], 'velocity': [0., -1., 0.]}
        result = native.plan(value, (0., 0., 0.), (.4, 0., 0.), [moving], config=config)
        self.assertEqual(result['trajectory'][1][:3], (.4, 0., 0.))
        ontime = native.track_segment(value, result, (0., 0., 0.), 0., [moving], config)
        delayed = native.track_segment(value, result, (0., 0., 0.), .7, [moving], config)
        self.assertFalse(ontime['stop'])
        self.assertTrue(delayed['stop'])
        self.assertEqual(delayed['reason'], 'E_LOCAL_COLLISION')

    def test_controller_keeps_committed_target_until_actual_feedback_arrives(self):
        value = flat()
        config = replace(native.Config(), validity_s=2.)
        result = native.plan(value, (0., 0., 0.), (.4, .4, 0.), config=config)
        first = result['trajectory'][1]
        midway = tuple(first[i] / 2 for i in range(3))
        out = native.track_segment(value, result, midway, .6, [], config)
        self.assertEqual(out['segment_target'], list(first[:3]))
        done = native.track_segment(value, result, first[:3], .6, [], config)
        self.assertTrue(done['complete'])
        self.assertEqual(done['velocity'], [0., 0., 0.])

    def test_execution_trace_cannot_hide_unsupported_body_samples(self):
        value = flat()
        a, b = (0., 0., 0., 0.), (.2, 0., 0., .5)
        bad = [a, (.1, 0., .6, .25), b]
        verdict = evaluate.evaluate(value, record(value, [a, b], execution=bad), [])
        self.assertIn('execution_unsupported', verdict['failures'])
        mismatch = evaluate.evaluate(value, record(value, [a, b], execution=[a, (.3, 0., 0., .5)]), [])
        self.assertIn('execution_contact_mismatch', mismatch['failures'])

    def test_execution_transfer_requires_declared_edge_and_actual_line(self):
        value = layers()
        config = replace(native.Config(), tick_s=1.2, validity_s=1.5)
        a, b = (-.4, 0., 0., 0.), (.2, 0., .15, 1.2)
        actual = [a, (-.1, 0., .075, .6), b]
        good = evaluate.evaluate(value, record(value, [a, b], config, actual), [])
        self.assertTrue(good['ok'], good)
        bad = [a, (-.1, .3, .075, .6), b]
        verdict = evaluate.evaluate(value, record(value, [a, b], config, bad), [])
        self.assertIn('execution_unsupported', verdict['failures'])

    def test_continuous_feedback_replans_only_after_segment_completion(self):
        for name, value, start, goal, obstacles, config in (
            ('dynamic', flat(), (0., 0., 0.), (1.2, 0., 0.),
             [dict(BOX, min=[.6, .8, -.05], max=[1., 1.2, .8], velocity=[0., -.4, 0.])], native.Config()),
            ('step', layers(), (-.4, 0., 0.), (.6, 0., .15), [],
             replace(native.Config(), tick_s=1.2, validity_s=1.5))):
            actual, command, active = start, (0., 0., 0.), None
            execution, contacts = [(*start, 0.)], [(*start, 0.)]
            for tick in range(2500):
                stamp = tick * .01
                if tick % 5 == 0:
                    if active is None:
                        active = native.plan(value, actual, goal, obstacles, now=stamp, config=config)
                    output = native.track_segment(value, active, actual, stamp, obstacles, config)
                    command = output['velocity']
                    if output['complete'] or output['reason'].startswith('E_LOCAL_'):
                        active = None
                actual = tuple(actual[i] + .01 * command[i] for i in range(3))
                point = (*actual, stamp + .01)
                execution.append(point)
                if any(abs(actual[2] - s['z_reference']) <= 1e-6 for s in value['surfaces']):
                    contacts.append(point)
                if native._distance(actual, goal) <= .08 and abs(actual[2] - goal[2]) <= 1e-6:
                    break
            self.assertLessEqual(native._distance(actual, goal), .08, (name, actual))
            cfg = replace(config, horizon_s=execution[-1][3] + 1.)
            verdict = evaluate.evaluate(value, record(value, contacts, cfg, execution), obstacles)
            self.assertTrue(verdict['ok'], (name, verdict))

    def test_ros_node_inherits_probe_group_and_is_reaped(self):
        with open(os.devnull, 'wb') as stream:
            process = runtime_probe.start_node([sys.executable, '-c', 'import time; time.sleep(30)'], stream)
            try:
                self.assertEqual(os.getpgid(process.pid), os.getpgrp())
            finally:
                process.terminate()
                process.wait(timeout=5.)
            self.assertIsNotNone(process.returncode)


if __name__ == '__main__':
    unittest.main(verbosity=2)
