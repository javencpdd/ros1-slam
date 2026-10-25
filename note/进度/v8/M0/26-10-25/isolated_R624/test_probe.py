import contextlib
import copy
import io
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import probe


class ProbeTests(unittest.TestCase):
    def setUp(self):
        self.args = probe.create_args('unit-test')
        self.capture = json.loads(Path(__file__).with_name('preflight.json').read_text())

    def test_network_and_privileges(self):
        self.assertEqual(self.args[self.args.index('--network') + 1], 'none')
        self.assertIn('--read-only', self.args)
        self.assertEqual(self.args[self.args.index('--cap-drop') + 1], 'ALL')
        self.assertEqual(self.args[self.args.index('--security-opt') + 1], 'no-new-privileges')
        for flag in ('--privileged', '--device', '--pid', '--ipc', '-p', '--publish', '-v', '--volume'):
            self.assertNotIn(flag, self.args)

    def test_mounts_are_source_only_and_readonly(self):
        mounts = [self.args[i + 1] for i, arg in enumerate(self.args) if arg == '--mount']
        self.assertEqual(len(mounts), 2)
        for mount in mounts:
            self.assertTrue(mount.startswith('type=bind,src=' + str(probe.ROOT)))
            self.assertTrue(mount.endswith(',readonly'))
        self.assertIn('dst=/opt/ros1-slam/ws/src,readonly', mounts[0])
        self.assertIn('dst=/opt/ros1-slam/scripts,readonly', mounts[1])

    def test_fixed_image_and_private_identity(self):
        self.assertIn(probe.IMAGE, self.args)
        self.assertEqual(self.args[self.args.index('--name') + 1], 'ros1-slam-r624-unit-test')
        self.assertIn(probe.LABEL + '=unit-test', self.args)
        self.assertEqual(self.args[self.args.index('--user') + 1], '1000:1000')

    def test_resource_limits(self):
        for flag, value in (('--cpus', '4'), ('--memory', '8g'), ('--memory-swap', '8g'),
                            ('--pids-limit', '512')):
            self.assertEqual(self.args[self.args.index(flag) + 1], value)

    def test_no_build_or_simulation_in_payload(self):
        payload = self.args[-1]
        for forbidden in ('catkin_make', 'cmake ', 'roslaunch ', 'roscore', 'gzserver', 'FORCE=',
                          'ISO_TARGET=', 'VPF_LOAD_MAX=', 'VPF_NUMPY_PREFIX='):
            self.assertNotIn(forbidden, payload)
        result = subprocess.run(['bash', '-n', '-c', payload], capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(payload.endswith('exit "$rc"'))

    def test_real_capture_is_rejection_not_acceptance(self):
        self.assertNotIn('error', self.capture)
        self.assertEqual(self.capture['preflight']['rc'], 5)
        self.assertEqual(self.capture['exit_state']['ExitCode'], 5)
        self.assertFalse(self.capture['exit_state']['Running'])
        text = self.capture['preflight']['stdout']
        self.assertIn('PREFLIGHT VERDICT rc=5', text)
        self.assertNotIn('PREFLIGHT VERDICT rc=0', text)
        self.assertIn('R624_ISO_DIAGNOSTIC_RC=1', text)
        self.assertIn('elevation_merged_layer.cpp', text)
        self.assertIn('elevation_merged_layer.h', text)

    def test_real_capture_retirement_and_protection(self):
        self.assertEqual(self.capture['before'], self.capture['after'])
        self.assertEqual(self.capture['input_before'], self.capture['input_after'])
        self.assertGreater(len(self.capture['input_before']), 0)
        self.assertEqual(self.capture['remaining'], [])
        self.assertEqual(self.capture['remove']['rc'], 0)
        self.assertEqual(self.capture['remove']['argv'], ['docker', 'rm', self.capture['container_id']])
        self.assertNotIn('stop', self.capture)

    def test_real_capture_mount_configuration(self):
        conf = self.capture['isolation']['HostConfig']
        self.assertEqual(conf['NetworkMode'], 'none')
        self.assertTrue(conf['ReadonlyRootfs'])
        self.assertFalse(conf['Privileged'])
        self.assertFalse(conf['PortBindings'])
        self.assertFalse(conf['Devices'])
        self.assertEqual(conf['NanoCpus'], 4000000000)
        self.assertEqual(conf['Memory'], 8 * 1024 ** 3)
        self.assertEqual(conf['MemorySwap'], conf['Memory'])
        self.assertEqual(conf['PidsLimit'], 512)
        self.assertEqual(len(self.capture['isolation']['Mounts']), 2)
        for mount in self.capture['isolation']['Mounts']:
            self.assertFalse(mount['RW'])
            self.assertEqual(mount['Type'], 'bind')

    def test_acceptance_rejects_partial_or_failed_start(self):
        good = copy.deepcopy(self.capture)
        good['preflight']['rc'] = 0
        good['exit_state']['ExitCode'] = 0
        good['errors'] = []
        self.assertTrue(probe.accepted(good))
        mutations = [('preflight', 'rc', 125), ('exit_state', 'Status', 'created'),
                     ('exit_state', 'Running', True), ('exit_state', 'Error', 'start failed'),
                     ('exit_state', 'OOMKilled', True), ('exit_state', 'StartedAt', ''),
                     ('exit_state', 'FinishedAt', ''), ('remove', 'rc', 1)]
        for section, key, value in mutations:
            with self.subTest(section=section, key=key):
                bad = copy.deepcopy(good)
                bad[section][key] = value
                self.assertFalse(probe.accepted(bad))
        for key, value in [('errors', ['failed']), ('remaining', ['cid']),
                           ('inputs_unchanged', False), ('existing_state_unchanged', False)]:
            bad = copy.deepcopy(good)
            bad[key] = value
            self.assertFalse(probe.accepted(bad))
        self.assertFalse(probe.accepted({}))

    def simulate(self, fault=None):
        calls = []
        token = 'mocktoken'
        info = copy.deepcopy(self.capture['isolation'])
        info.update({'Name': '/ros1-slam-r624-' + token,
                     'Config': {'Labels': {probe.LABEL: token}},
                     'State': copy.deepcopy(self.capture['exit_state'])})
        info['State']['ExitCode'] = 0
        if fault == 'foreign':
            info['Config']['Labels'][probe.LABEL] = 'other-owner'

        def fake_checked(args):
            calls.append(args)
            if args[0] == 'setfacl':
                return ''
            if args[:2] == ['docker', 'create']:
                if fault == 'create_timeout':
                    raise subprocess.TimeoutExpired(args, 120)
                return info['Id'] + '\n'
            if args[:2] == ['docker', 'inspect']:
                return json.dumps([info])
            if args[:2] == ['docker', 'ps']:
                return ''
            raise AssertionError(args)

        def fake_run(args, timeout=120):
            calls.append(args)
            if fault == 'cleanup_timeout' and args[:2] == ['docker', 'rm']:
                raise subprocess.TimeoutExpired(args, 120)
            return {'argv': args, 'rc': 0, 'stdout': '', 'stderr': ''}

        snapshots = [{}, RuntimeError('post snapshot unavailable')] if fault == 'resample' else [{}, {}]
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder) / 'probe.json'
            with patch.object(probe, 'checked', side_effect=fake_checked), \
                    patch.object(probe, 'run', side_effect=fake_run), \
                    patch.object(probe, 'state', side_effect=snapshots), \
                    patch.object(probe, 'fingerprint', return_value={'file': 'sha'}), \
                    patch.object(probe.uuid, 'uuid4') as ident, contextlib.redirect_stdout(io.StringIO()):
                ident.return_value.hex = token
                rc = probe.main(out)
            return rc, json.loads(out.read_text()), calls

    def test_mock_clean_exit_accepts(self):
        rc, data, calls = self.simulate()
        self.assertEqual(rc, 0)
        self.assertTrue(data['accepted'])
        self.assertTrue(any(c[:2] == ['docker', 'rm'] for c in calls))

    def test_create_timeout_recovers_owned_id_and_records_failure(self):
        rc, data, calls = self.simulate('create_timeout')
        self.assertEqual(rc, 5)
        self.assertTrue(data['errors'])
        self.assertIn(['docker', 'inspect', 'ros1-slam-r624-mocktoken'], calls)
        self.assertIn(['docker', 'rm', self.capture['container_id']], calls)
        self.assertFalse(data['accepted'])

    def test_cleanup_timeout_still_saves_evidence(self):
        rc, data, _ = self.simulate('cleanup_timeout')
        self.assertEqual(rc, 5)
        self.assertTrue(any(s.startswith('cleanup:') for s in data['errors']))
        self.assertIn('after', data)
        self.assertIn('finished_at', data)

    def test_resample_error_still_saves_evidence(self):
        rc, data, _ = self.simulate('resample')
        self.assertEqual(rc, 5)
        self.assertFalse(data['existing_state_unchanged'])
        self.assertTrue(any(s.startswith('after:') for s in data['errors']))

    def test_foreign_identity_never_removed(self):
        rc, data, calls = self.simulate('foreign')
        self.assertEqual(rc, 5)
        self.assertTrue(any('ownership mismatch' in s for s in data['errors']))
        self.assertFalse(any(c[:2] in (['docker', 'rm'], ['docker', 'stop']) for c in calls))


if __name__ == '__main__':
    unittest.main()
