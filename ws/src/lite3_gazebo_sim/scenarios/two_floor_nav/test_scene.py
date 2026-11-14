"""Owned offline tests. Synthetic observations test checks, never prove Gazebo."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import scene
import validation
import support_planner
import local_native
sys.path.insert(0, str(scene.PLATFORM / 'framework'))
from app_trace import Trace, prepare_logs

spec = importlib.util.spec_from_file_location('scene_cli', scene.ROOT / 'scripts/sim/two_floor_nav.py')
cli = importlib.util.module_from_spec(spec); spec.loader.exec_module(cli)


def synthetic(case='upstairs'):
    """Dense command-consistent synthetic control, clearly not engine evidence."""
    m = scene.scene_map(case == 'blocked'); task = scene.task(case)
    start, goal = (task[k]['xyz'] for k in ('start', 'goal'))
    trace = Trace({'backend': 'gazebo', 'execution_mode': 'simulation', 'model_name': 'app_carrier',
                   'map_sha256': __import__('hashlib').sha256(scene.json_bytes(m)).hexdigest()})
    trace.record('global_status', {'ok': case != 'blocked', 'reason': 'E_PLAN_NO_ROUTE' if case == 'blocked' else 'arrived'}, 0, 0)
    if case == 'blocked':
        steps = [(i * .02, start, [0, 0, 0]) for i in range(301)]
    else:
        route = support_planner.plan(m, start, goal, robot_radius_m=.35, robot_height_m=.5, min_clearance_m=.55)
        path = [list(v[:3]) for v in route['path']]
        trace.record('global_path', {'points': path, 'frame': 'map', 'stamp': 0}, 0, 0)
        points = [start] + path + [goal]
        steps, at = [], 0.
        for a, b in zip(points, points[1:]):
            count = max(1, math.ceil(math.dist(a, b) / .005))
            velocity = [(b[i] - a[i]) / (count * .02) for i in range(3)]
            for j in range(count):
                p = [a[i] + (b[i] - a[i]) * j / count for i in range(3)]
                steps.append((at, p, velocity)); at += .02
        for i in range(20):
            steps.append((at, goal, [0, 0, 0])); at += .02
        trace.record('local_trajectory', {'trajectory': [list(p) + [i] for i, p in enumerate(path)]}, 0, 0)
        trace.record('local_status', {'ok': True, 'reason': 'arrived'}, at, at)
    for at, p, velocity in steps:
        trace.record('pose', {'xyz': list(p), 'frame': 'map', 'stamp': at}, at, at)
        trace.record('gazebo_model', {'present': True, 'model': 'app_carrier', 'xyz': [p[0], p[1], p[2] + .3]}, at, at)
        trace.record('command', {'linear': list(velocity), 'angular': [0, 0, 0]}, at, at)
    return trace.snapshot(), {'ok': case != 'blocked', 'checks': {'assembly_ready': True}}


class SceneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.good, cls.good_result = synthetic()
        cls.blocked, cls.blocked_result = synthetic('blocked')

    def test_committed_artifacts_and_private_registry(self):
        self.assertEqual(scene.check_artifacts(), [])
        import map_registry
        with tempfile.TemporaryDirectory(prefix='two-floor-no-site-registry-') as t:
            with mock.patch.object(map_registry, 'DEFAULT_PATH', Path(t) / 'missing-registry.json'):
                for case in scene.CASES:
                    result = cli.plan_case(case)
                    self.assertTrue(result['ok'])
                    if case == 'blocked': self.assertEqual(result['expected_refusal'], 'E_PLAN_NO_ROUTE')

    def test_output_geometry_matches_xml_collision_and_visual(self):
        for blocked in (False, True):
            tree = ET.fromstring(scene.world_xml(blocked))
            self.assertFalse(tree.findall('.//include'))
            links = {v.get('name'): v for v in tree.findall('.//link')}
            for value in scene.geometry(blocked)['boxes']:
                link = links[value['name']]
                center = list(map(float, link.findtext('pose').split()[:3]))
                for tag in ('collision', 'visual'):
                    size = list(map(float, link.findtext(tag + '/geometry/box/size').split()))
                    self.assertEqual([round(center[i] - size[i] / 2, 6) for i in range(3)], value['min'])
                    self.assertEqual([round(center[i] + size[i] / 2, 6) for i in range(3)], value['max'])

    def test_support_cells_are_real_tops_and_exclude_covered_ground(self):
        m = scene.scene_map(); boxes = {b['name']: b for b in scene.geometry()['boxes']}
        for support in m['surfaces']:
            physical = boxes[support['surface_id']]
            self.assertEqual(support['z_reference'], physical['max'][2])
            for i, j in scene.support_map.surface_cells(support):
                x, y = (i + .5) * scene.RESOLUTION, (j + .5) * scene.RESOLUTION
                self.assertLessEqual(physical['min'][0], x); self.assertLess(x, physical['max'][0])
                self.assertLessEqual(physical['min'][1], y); self.assertLess(y, physical['max'][1])
                if support['surface_id'] == 'ground_floor':
                    self.assertFalse(-4 <= x < 4 and 0 <= y < 2)
        self.assertEqual(m['schema_version'], 3)
        self.assertTrue(m['multiple_surfaces_per_xy'])

    def test_global_same_xy_different_floors_uses_every_step(self):
        m = scene.scene_map()
        route = support_planner.plan(m, [-6, 1, 0], [-6, 1, 1.6], robot_radius_m=.35, robot_height_m=.5)
        self.assertEqual({v[2] for v in route['path']}, {round(i * .16, 6) for i in range(11)})
        self.assertGreater(len(route['transitions']), 10)

    def test_local_replanning_up_down_and_around_box(self):
        m = scene.scene_map()
        cfg = local_native.Config(resolution=.4, tick_s=1.2, horizon_s=8.4, validity_s=20,
                                  max_speed_mps=.6, max_step_m=.2, radius_m=.3, height_m=.5,
                                  clearance_m=.05, goal_tolerance_m=.2)
        for case in ('upstairs', 'downstairs', 'upper_detour'):
            task = scene.task(case); pose = task['start']['xyz']; goal = task['goal']['xyz']
            for n in range(70):
                result = local_native.plan(m, pose, goal, config=cfg, now=n * cfg.tick_s)
                self.assertTrue(result['checker']['ok'])
                if result['reason'] == 'arrived':
                    pose = result['trajectory'][-1][:3]; break
                pose = result['trajectory'][1][:3]
            else: self.fail('offline repeated planning did not reach ' + case)
            self.assertLessEqual(math.dist(pose, goal), .2 + 1e-9)

    def test_trace_positive_controls_are_only_synthetic(self):
        result = validation.validate_trace(self.good, 'upstairs', self.good_result)
        self.assertTrue(result['ok']); self.assertEqual(result['four_foot_physics'], 'NOT_TESTED')
        self.assertTrue(validation.validate_trace(self.blocked, 'blocked', self.blocked_result)['ok'])

    def test_wrong_six_axis_command_and_dense_wrong_direction_are_rejected(self):
        bad = copy.deepcopy(self.blocked)
        for event in bad['events']:
            if event['kind'] == 'command': event['payload']['angular'][2] = 1.
        with self.assertRaisesRegex(validation.ValidationError, 'nonzero commands'):
            validation.validate_trace(bad, 'blocked', self.blocked_result)
        bad = copy.deepcopy(self.good)
        for event in bad['events']:
            if event['kind'] == 'command': event['payload']['linear'] = [.001, 0, 0]
        with self.assertRaisesRegex(validation.ValidationError, 'integral'):
            validation.validate_trace(bad, 'upstairs', self.good_result)

    def test_missing_samples_final_pose_only_nan_and_wrong_goal_rejected(self):
        edits = [lambda v: v.update(events=[]), lambda v: v['events'][0].update(ros_time=float('nan')),
                 lambda v: v.update(overflow=True), lambda v: v.update(errors=[{'error': 'lost input'}])]
        for edit in edits:
            bad = copy.deepcopy(self.good); edit(bad)
            with self.assertRaises(validation.ValidationError): validation.validate_trace(bad, 'upstairs', self.good_result)
        bad = copy.deepcopy(self.good)
        bad['events'] = [v for v in bad['events'] if v['kind'] != 'pose'] + [next(v for v in bad['events'] if v['kind'] == 'pose')]
        with self.assertRaisesRegex(validation.ValidationError, 'insufficient'):
            validation.validate_trace(bad, 'upstairs', self.good_result)
        with self.assertRaises(validation.ValidationError):
            validation.validate_trace(self.good, 'downstairs', self.good_result)

    def test_gazebo_whole_run_coverage_and_numeric_source_clock(self):
        bad = copy.deepcopy(self.good)
        count = 0
        selected = []
        for event in bad['events']:
            if event['kind'] == 'gazebo_model':
                count += 1
                if count > 20: continue
            selected.append(event)
        bad['events'] = selected
        with self.assertRaisesRegex(validation.ValidationError, 'cover the complete'):
            validation.validate_trace(bad, 'upstairs', self.good_result)
        bad = copy.deepcopy(self.good)
        for event in bad['events']:
            if event['kind'] == 'pose': event['payload']['stamp'] = 'not-a-clock'
        with self.assertRaisesRegex(validation.ValidationError, 'source stamp'):
            validation.validate_trace(bad, 'upstairs', self.good_result)

    def test_preflight_refusal_never_calls_engine(self):
        from nodes import app_runner
        with tempfile.TemporaryDirectory(prefix='two-floor-refusal-owned-') as t:
            output = Path(t) / 'result.json'
            with mock.patch.object(cli, 'plan_case', return_value={}), \
                 mock.patch.object(cli, 'validate_window', return_value={'id': 'owned'}), \
                 mock.patch.object(cli, 'ports_idle'), \
                 mock.patch.object(cli.shutil, 'which', return_value='/owned/tool'), \
                 mock.patch.object(cli.subprocess, 'run', return_value=subprocess.CompletedProcess([], 5, 'owned refusal')) as command, \
                 mock.patch.object(app_runner, 'execute') as engine:
                with self.assertRaisesRegex(ValueError, 'preflight refused'):
                    cli.execute_case('upstairs', output, window_id='owned')
                engine.assert_not_called()
                self.assertEqual(command.call_args_list[0][0][0], ['bash', str(scene.ROOT / 'scripts/verify/v8_window_preflight.sh')])
            self.assertFalse(output.exists())
            self.assertTrue(output.with_suffix('.preflight.log').is_file())

    def test_busy_port_rejected(self):
        import socket
        with socket.socket() as listener:
            listener.bind(('127.0.0.1', 0)); listener.listen(1)
            port = listener.getsockname()[1]
            with self.assertRaises(ValueError): cli.ports_idle(port)

    def test_complete_segment_collision_catches_safe_endpoints(self):
        geo = scene.geometry()
        with self.assertRaisesRegex(validation.ValidationError, 'intersects upper_detour_box'):
            validation.collision_free([[-4.5, -3, 1.6], [-7, -3, 1.6]], geo)

    def test_capture_overflow_invalid_clock_and_fresh_log_allocation(self):
        trace = Trace({}, limit=1); trace.record('x', {}, 0, 0); trace.record('x', {}, 1, 1)
        self.assertFalse(trace.snapshot()['capture_ok'])
        trace = Trace({}); trace.record('x', {}, 'invalid')
        self.assertFalse(trace.snapshot()['capture_ok'])
        with tempfile.TemporaryDirectory(prefix='two-floor-owned-') as t:
            path = Path(t) / 'run.trace.json'
            paths, home, logs = prepare_logs(path)
            self.assertEqual(len(paths), 9)
            self.assertTrue(Path(home).is_dir() and Path(logs).is_dir())
            with self.assertRaises(FileExistsError): prepare_logs(path)

    def test_receipt_exact_identity_case_kind_and_paths(self):
        with tempfile.TemporaryDirectory(prefix='two-floor-receipt-owned-') as t:
            root = Path(t); (root / 'scripts/verify').mkdir(parents=True); (root / 'evidence').mkdir()
            receipt = root / 'scripts/verify/_window_receipt.tsv'
            good = 'R-test\tevidence\tsim\ttwo_floor_nav case=upstairs\n'
            receipt.write_text(good)
            self.assertEqual(cli.validate_window(root, 'R-test', 'upstairs')['id'], 'R-test')
            for text in (good + good, good.replace('\tsim\t', '\treal\t'), good.replace('case=upstairs', 'case=downstairs'),
                         good.replace('\tevidence\t', '\t../escape\t'), good.replace('\tevidence\t', '\tmissing\t')):
                receipt.write_text(text)
                with self.assertRaises(ValueError): cli.validate_window(root, 'R-test', 'upstairs')
            receipt.write_text(good)
            with self.assertRaises(ValueError): cli.validate_window(root, None, 'upstairs')

    def test_default_cli_does_not_start_runtime_or_write_files(self):
        with mock.patch.object(cli.subprocess, 'run', side_effect=AssertionError('unexpected subprocess')):
            self.assertTrue(cli.plan_case('upstairs')['ok'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
