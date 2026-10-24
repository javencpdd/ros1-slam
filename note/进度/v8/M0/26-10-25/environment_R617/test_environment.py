#!/usr/bin/env python3
import csv
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tarfile
import unittest
from unittest.mock import patch

sys.dont_write_bytecode = True
import capture_environment as capture

HERE = Path(__file__).resolve().parent


class CaptureTests(unittest.TestCase):
    def test_package_fields_and_status(self):
        rows = capture.parse_packages(
            b'Package: z\nVersion: 1:2-3\nArchitecture: amd64\n'
            b'Status: install ok installed\nDescription: text\n Source: not-a-field\n\n'
            b'Package: a\nVersion: 4\nArchitecture: all\n'
            b'Status: deinstall ok config-files\nSource: real-source (4)\n')
        self.assertEqual([row['Package'] for row in rows], ['a', 'z'])
        self.assertEqual(rows[1]['Version'], '1:2-3')
        self.assertEqual(rows[1]['Source'], '')
        self.assertEqual(rows[0]['Source'], 'real-source (4)')
        self.assertEqual(sum(row['Status'] == 'install ok installed' for row in rows), 1)
        self.assertEqual(capture.parse_packages(b''), [])

    def test_static_versions_do_not_execute(self):
        self.assertEqual(capture.static_versions(
            b"version = '1.2.3'\nfull_version = '1.2.3+build'\nx = fail()\n"),
            {'version': '1.2.3', 'full_version': '1.2.3+build'})
        with self.assertRaises(SyntaxError):
            capture.static_versions(b'version = (')

    def test_single_file_archive(self):
        for count in (0, 1, 2):
            stream = io.BytesIO()
            with tarfile.open(fileobj=stream, mode='w') as archive:
                for index in range(count):
                    member = tarfile.TarInfo(str(index))
                    member.size = 3
                    archive.addfile(member, io.BytesIO(b'abc'))
            result = subprocess.CompletedProcess([], 0, stream.getvalue(), b'')
            with self.subTest(count=count), patch.object(capture.subprocess, 'run', return_value=result):
                if count != 1:
                    with self.assertRaises(ValueError):
                        capture.copy_file('specimen', '/file')
                else:
                    record, content = capture.copy_file('specimen', '/file')
                    self.assertEqual(content, b'abc')
                    self.assertEqual(record['bytes'], 3)
                    self.assertEqual(record['sha256'], hashlib.sha256(content).hexdigest())

    def test_copy_error_not_success(self):
        result = subprocess.CompletedProcess([], 1, b'', b'mount failure')
        with patch.object(capture.subprocess, 'run', return_value=result):
            record, content = capture.copy_file('specimen', '/file')
        self.assertIsNone(content)
        self.assertEqual(record, {'path': '/file', 'rc': 1, 'error': 'mount failure'})

    def test_existing_snapshot_refused_before_docker(self):
        with patch.object(sys, 'argv', ['capture', '--root', str(HERE), '--output', str(HERE)]), \
                patch.object(capture, 'inspect') as inspect, \
                patch('sys.stderr', new_callable=io.StringIO) as errors:
            with self.assertRaises(SystemExit) as result:
                capture.main()
        self.assertEqual(result.exception.code, 2)
        self.assertIn('Refusing to overwrite', errors.getvalue())
        inspect.assert_not_called()

    def test_saved_snapshot_integrity(self):
        snapshot = json.loads((HERE / 'environment-lock.json').read_text())
        baseline = snapshot['image_baseline']
        raw = (HERE / baseline['packages_file']).read_bytes()
        rows = list(csv.DictReader(io.StringIO(raw.decode()), delimiter='\t'))
        self.assertEqual(hashlib.sha256(raw).hexdigest(), baseline['packages_sha256'])
        self.assertEqual(len(rows), baseline['package_records'])
        self.assertEqual(len({(row['Package'], row['Architecture']) for row in rows}), len(rows))
        self.assertEqual(sum(row['Status'] == 'install ok installed' for row in rows),
                         baseline['installed_packages'])
        self.assertEqual(snapshot['status'], 'partial_observed_baseline')
        self.assertTrue(snapshot['host']['cpu'])
        self.assertFalse(snapshot['docker']['existing_container']['State']['Running'])
        self.assertEqual(snapshot['docker']['existing_container']['Image'], baseline['image']['Id'])
        self.assertTrue(snapshot['docker']['existing_container_metadata_unchanged'])
        self.assertTrue(baseline['specimen_never_started'])
        self.assertTrue(baseline['specimen_removed'])
        self.assertIn('clean_rebuild', snapshot['unverified'])
        self.assertIn('named_volume_build_and_devel', snapshot['unverified'])
        installed = {row['Package'] for row in rows if row['Status'] == 'install ok installed'}
        expected = {'gazebo11'} | {'ros-noetic-' + suffix for suffix in (
            'catkin', 'roscpp', 'move-base-flex', 'mbf-costmap-nav', 'mbf-costmap-core',
            'dwa-local-planner', 'teb-local-planner', 'global-planner', 'navfn',
            'base-local-planner', 'costmap-2d', 'grid-map-core', 'grid-map-ros', 'grid-map-msgs')}
        self.assertTrue(expected <= installed, expected - installed)


if __name__ == '__main__':
    unittest.main(verbosity=2)
