import copy
import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import xml.etree.ElementTree as ET

import capture

ROOT = Path(__file__).resolve().parents[6]
DATA = Path(__file__).with_name('inventory-final.json')
COPY_TARGETS = {
    '00_drivers/lslidar_driver': 4, '00_drivers/lslidar_msgs': 5,
    '00_drivers/message_transformer': 6, '10_slam/faster_lio': 7,
    '10_slam/pcd2grid': 8, '20_localization/hdl_localization': 9,
    '20_localization/hdl_global_localization': 10, '20_localization/ndt_omp': 11,
    '20_localization/fast_gicp': 12, '30_perception/deeprobotics_local_height_map': 14,
    '30_perception/libraries': 15, '50_task/pipeline_tracking': 16,
    '95_simulation/mockamap': 25, '95_simulation/map_generator': 26,
    '95_simulation/local_sensing': 27, '95_simulation/lite3_description': 28,
    '95_simulation/Mid360_simulation_plugin': 43, '30_3rd/traj_utils': 51,
    '30_3rd/plan_env': 52, '30_3rd/path_searching': 53,
    '30_3rd/bspline_opt': 54, '30_3rd/plan_manage': 55,
}
BINARY_NAMES = {
    'ros-noetic-base-local-planner', 'ros-noetic-costmap-2d', 'ros-noetic-dwa-local-planner',
    'ros-noetic-global-planner', 'ros-noetic-map-server', 'ros-noetic-mbf-costmap-nav',
    'ros-noetic-move-base', 'ros-noetic-move-base-flex', 'ros-noetic-navfn',
    'ros-noetic-sob-layer', 'ros-noetic-spatio-temporal-voxel-layer', 'ros-noetic-teb-local-planner',
}


def mappings_valid(packages):
    if sum(bool(p['copy_record_lines']) for p in packages) != 28:
        return False
    for row in packages:
        expected = sorted(number for target, number in COPY_TARGETS.items()
                          if Path(row['path']).is_relative_to(Path('ws/src') / target))
        if row['copy_record_lines'] != expected:
            return False
    return True


def binaries_valid(rows):
    return len(rows) == len(BINARY_NAMES) and {r['Package'] for r in rows} == BINARY_NAMES


def verify_hashes(root, hashes):
    return bool(hashes) and all(hashlib.sha256((root / name).read_bytes()).hexdigest() == digest
                                for name, digest in hashes.items())


class InventoryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data = json.loads(DATA.read_text())

    def test_current_metadata_coverage(self):
        paths = sorted(str(p.relative_to(ROOT)) for p in (ROOT / 'ws/src').rglob('package.xml'))
        self.assertTrue(paths)
        self.assertEqual(paths, sorted(p['metadata'] for p in self.data['packages']))
        self.assertEqual(len(paths), len(set(p['path'] for p in self.data['packages'])))

    def test_exact_metadata_and_build_entry(self):
        for p in self.data['packages']:
            for key, value in capture.metadata(ROOT / p['metadata']).items():
                self.assertEqual(value, p[key])
            self.assertIn(p['build_entry'], self.data['source_sha256'])
            self.assertEqual(p['main_repo_tree'], capture.git(ROOT, 'rev-parse', 'HEAD:' + p['path']))

    def test_source_hashes(self):
        self.assertTrue(verify_hashes(ROOT, self.data['source_sha256']))

    def test_empty_and_tampered_hash_rejected(self):
        self.assertFalse(verify_hashes(ROOT, {}))
        self.assertFalse(verify_hashes(ROOT, {'docker/Dockerfile': '0' * 64}))

    def test_xml_comments_are_not_declarations(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'package.xml'
            path.write_text('<package><name>x</name><version>1</version><!--<license>TODO</license>-->'
                            '<license>BSD</license><license>MIT</license></package>')
            self.assertEqual(capture.metadata(path)['licenses_declared'], ['BSD', 'MIT'])
            self.assertEqual(capture.metadata(path)['urls_declared'], [])

    def test_invalid_xml_is_not_skipped(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'package.xml'
            path.write_text('<package>')
            with self.assertRaises(ET.ParseError):
                capture.metadata(path)

    def test_parent_git_is_not_upstream(self):
        with patch.object(capture, 'git', side_effect=AssertionError('must not query parent git')):
            self.assertEqual(capture.own_git(ROOT / 'ws/src/30_3rd/plan_env')['status'], 'no_own_git')

    def test_mismatched_git_root_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            (path / '.git').mkdir()
            with patch.object(capture, 'git', return_value=str(path.parent)):
                self.assertEqual(capture.own_git(path)['status'], 'toplevel_mismatch')

    def test_historical_binaries_are_exact_rows(self):
        path = ROOT / 'note/进度/v8/M0/26-10-25/environment_R617/image-packages.tsv'
        lines = path.read_text().splitlines()
        rows = [dict(zip(lines[0].split('\t'), line.split('\t'))) for line in lines[1:]]
        selected = self.data['historical_binary_baseline']['selected_packages']
        self.assertTrue(binaries_valid(selected))
        for row in selected:
            self.assertIn(row, rows)
            self.assertEqual(row['Status'], 'install ok installed')

    def test_repeated_binary_row_rejected(self):
        rows = self.data['historical_binary_baseline']['selected_packages']
        self.assertFalse(binaries_valid([rows[0]] * 12))
        self.assertFalse(binaries_valid([]))
        self.assertFalse(binaries_valid(rows[:-1]))

    def test_copy_records_match_parent_not_identity(self):
        lines = (ROOT / 'note/03_外部文件来源清单.md').read_text().splitlines()
        for target, number in COPY_TARGETS.items():
            self.assertIn('/home/ros1-slam/ws/src/' + target, lines[number - 1])
        self.assertTrue(mappings_valid(self.data['packages']))
        for row in self.data['packages']:
            self.assertEqual(row['upstream_revision_binding'], 'not_established_by_this_capture')

    def test_empty_and_wrong_copy_mappings_rejected(self):
        rows = copy.deepcopy(self.data['packages'])
        for row in rows:
            row['copy_record_lines'] = []
        self.assertFalse(mappings_valid(rows))
        rows = copy.deepcopy(self.data['packages'])
        for row in rows:
            if row['copy_record_lines']:
                row['copy_record_lines'] = [4]
        self.assertFalse(mappings_valid(rows))
        rows = copy.deepcopy(self.data['packages'])
        for row in rows:
            row['copy_record_lines'] = [4]
        self.assertFalse(mappings_valid(rows))

    def test_todo_and_conflicting_declarations_preserved(self):
        by_name = {p['name']: p for p in self.data['packages']}
        self.assertEqual(sum('TODO' in p['licenses_declared'] for p in by_name.values()), 9)
        self.assertEqual(by_name['faster_lio']['licenses_declared'], ['BSD'])
        license_text = (ROOT / 'ws/src/10_slam/faster_lio/LICENSE').read_text()
        self.assertIn('Version 2, June 1991', license_text)
        self.assertIn('Proprietary and confidential',
                      (ROOT / 'ws/src/50_task/pipeline_tracking/scripts/TaskPoint.py').read_text())

    def test_end_timestamp_follows_final_git_read(self):
        events = []
        original_git = capture.git
        original_datetime = capture.datetime

        class Clock:
            @staticmethod
            def now(zone):
                events.append('clock')
                return original_datetime.now(zone)

        def observed_git(*args):
            value = original_git(*args)
            events.append('git')
            return value

        with patch.object(capture, 'datetime', Clock), patch.object(capture, 'git', observed_git):
            data = capture.collect(ROOT)
        self.assertEqual(events[0], 'clock')
        self.assertEqual(events[-2:], ['git', 'clock'])
        self.assertEqual(events.count('clock'), 2)
        self.assertLess(data['started_at_utc'], data['completed_at_utc'])

    def test_capture_stable_except_timestamps(self):
        now = capture.collect(ROOT)
        saved = dict(self.data)
        for key in ('started_at_utc', 'completed_at_utc'):
            now.pop(key)
            saved.pop(key)
        self.assertEqual(now, saved)


if __name__ == '__main__':
    unittest.main(verbosity=2)
