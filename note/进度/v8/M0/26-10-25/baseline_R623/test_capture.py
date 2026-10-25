import copy
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tarfile
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('capture', HERE / 'capture.py')
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


def archive(entries):
    stream = io.BytesIO()
    with tarfile.open(fileobj=stream, mode='w') as tar:
        for name, content, target in entries:
            member = tarfile.TarInfo(name)
            if target is not None:
                member.type, member.linkname = tarfile.SYMTYPE, target
                tar.addfile(member)
            else:
                member.size = len(content)
                tar.addfile(member, io.BytesIO(content))
    stream.seek(0)
    return stream


def isolated(row):
    return (row['state']['Status'] == 'created' and row['state']['Pid'] == 0
            and not row['state']['Running']
            and row['host_config']['ReadonlyRootfs']
            and row['host_config']['NetworkMode'] == 'none'
            and all(not m['RW'] for m in row['mounts'])
            and all(m.get('VolumeOptions', {}).get('NoCopy')
                    for m in (row['host_config'].get('Mounts') or [])))


class CaptureTests(unittest.TestCase):
    def test_archive_exact_hash_and_path(self):
        rows = capture.read_archive(archive([('src/a.cpp', b'abc', None)]), 'ws/src', 'src')
        self.assertEqual(rows, [{'path': 'ws/src/a.cpp', 'size': 3, 'kind': 'file',
                               'sha256': hashlib.sha256(b'abc').hexdigest()}])

    def test_single_file_prefix(self):
        rows = capture.read_archive(archive([('lib.a', b'xyz', None)]), 'lib.a', 'lib.a')
        self.assertEqual(rows[0]['path'], 'lib.a')

    def test_symlink_not_dereferenced(self):
        rows = capture.read_archive(archive([('src/CMakeLists.txt', b'', '/absent/file')]), 'ws/src', 'src')
        self.assertEqual(rows[0]['kind'], 'symlink')
        self.assertEqual(rows[0]['target'], '/absent/file')
        self.assertNotIn('sha256', rows[0])

    def test_traversal_and_absolute_rejected(self):
        for name in ['src/../etc/file', '/etc/file']:
            with self.assertRaises(ValueError):
                capture.read_archive(archive([(name, b'x', None)]), 'ws/src', 'src')

    def test_wrong_or_mixed_root_rejected(self):
        for entries in [[('wrong/a.cpp', b'x', None)],
                        [('src/a.cpp', b'x', None), ('wrong/b.cpp', b'y', None)]]:
            with self.assertRaises(ValueError):
                capture.read_archive(archive(entries), 'ws/src', 'src')

    def test_invalid_tar_rejected(self):
        with self.assertRaises(tarfile.TarError):
            capture.read_archive(io.BytesIO(b'not a tar'), 'ws/src', 'src')

    def test_comparison_four_disjoint_sets(self):
        left = [{'path': 'same', 'kind': 'file', 'sha256': 'a' * 64},
                {'path': 'different', 'kind': 'file', 'sha256': 'b' * 64},
                {'path': 'local', 'kind': 'file', 'sha256': 'c' * 64}]
        right = [{'path': 'same', 'kind': 'file', 'sha256': 'a' * 64},
                 {'path': 'different', 'kind': 'file', 'sha256': 'd' * 64},
                 {'path': 'image', 'kind': 'file', 'sha256': 'e' * 64}]
        self.assertEqual(capture.compare(left, right), {'equal': ['same'],
            'different': ['different'], 'only_left': ['local'], 'only_right': ['image']})
        with self.assertRaises(ValueError):
            capture.compare(left + [left[0]], right)

    def test_incomplete_or_empty_inventory_rejected(self):
        for row in [{'path': 'a', 'kind': 'file'},
                    {'path': 'a', 'kind': 'file', 'sha256': ''},
                    {'path': 'a', 'kind': 'file', 'sha256': 'z' * 64},
                    {'path': 'a', 'kind': 'symlink'},
                    {'path': 'a', 'kind': 'unknown'},
                    {'kind': 'file', 'sha256': 'a' * 64}]:
            with self.assertRaises(ValueError):
                capture.compare([row], [row])
        with self.assertRaises(ValueError):
            capture.compare([], [])

    def test_source_missing_roots_and_broken_links(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'ws/src').mkdir(parents=True)
            (root / 'ws/src/CMakeLists.txt').symlink_to('/no-such-R623-path')
            rows = capture.source_inventory(root)
            self.assertEqual(sum(r['kind'] == 'absent' for r in rows), 4)
            link = next(r for r in rows if r['kind'] == 'symlink')
            self.assertFalse(link['target_exists_on_host'])

    def test_observed_container_isolation_and_cleanup(self):
        data = json.loads((HERE / 'observations-final.json').read_text())
        self.assertEqual(len(data['temporary_containers']), 2)
        for row in data['temporary_containers']:
            self.assertTrue(isolated(row))
            bad = copy.deepcopy(row)
            bad['host_config']['ReadonlyRootfs'] = False
            self.assertFalse(isolated(bad))
        self.assertEqual(len(data['cleanup']), 2)
        self.assertTrue(all(row['rc'] == 0 for row in data['cleanup']))
        self.assertTrue(data['existing_metadata_unchanged'])
        self.assertEqual(data['existing_before'], data['existing_after'])
        self.assertTrue(data['local_inputs_unchanged'])
        self.assertLess(data['started_at_utc'], data['completed_at_utc'])

    def test_source_coverage_and_hashes(self):
        data = json.loads((HERE / 'observations-final.json').read_text())
        rows = data['local_inputs']
        paths = [r['path'] for r in rows]
        expected = set()
        for name in capture.COPY_INPUTS:
            path = capture.ROOT / name
            if path.is_dir():
                expected.update(str(p.relative_to(capture.ROOT)) for p in path.rglob('*')
                                if p.is_file() or p.is_symlink())
            else:
                expected.add(name)
        self.assertEqual(len(paths), len(set(paths)))
        self.assertEqual(set(paths), expected)
        for row in rows:
            if row['kind'] == 'file':
                self.assertRegex(row['sha256'], r'^[0-9a-f]{64}$')
                with (capture.ROOT / row['path']).open('rb') as f:
                    self.assertEqual(capture.digest(f), row['sha256'])

    def test_comparison_accounting(self):
        data = json.loads((HERE / 'observations-final.json').read_text())
        for name, left, right in [
                ('source_comparison', [r for r in data['local_inputs'] if r['path'].startswith('ws/src/')], data['image_sources']['rows']),
                ('artifact_comparison', data['image_artifacts']['rows'], data['volume_artifacts']['rows'])]:
            self.assertEqual(data[name], capture.compare(left, right))
            c = data[name]
            self.assertEqual(len(left), len(c['equal']) + len(c['different']) + len(c['only_left']))
            self.assertEqual(len(right), len(c['equal']) + len(c['different']) + len(c['only_right']))
        for name in ['image_sources', 'image_artifacts', 'volume_artifacts']:
            self.assertEqual(data[name]['rc'], 0)
            self.assertIsNone(data[name]['parse_error'])
            self.assertTrue(data[name]['rows'])

    def test_repeated_content_capture_stable(self):
        old = json.loads((HERE / 'observations.json').read_text())
        new = json.loads((HERE / 'observations-final.json').read_text())
        for key in ['local_inputs', 'source_comparison', 'artifact_comparison', 'existing_before', 'existing_after']:
            self.assertEqual(old[key], new[key])
        for key in ['image_sources', 'image_artifacts', 'volume_artifacts', 'image_sdk']:
            self.assertEqual(old[key]['rows'], new[key]['rows'])
        for key in ['image_cache', 'volume_cache']:
            self.assertEqual(old[key]['sha256'], new[key]['sha256'])

    def test_copy_roots_and_ignore_coverage(self):
        data = json.loads((HERE / 'supplement.json').read_text())
        instructions = data['docker_copy_instructions']
        self.assertEqual([r['line'] for r in instructions], [253, 319, 361, 397, 411, 412])
        self.assertEqual([r['text'].split()[1] for r in instructions if r['scope'] == 'host'], capture.COPY_INPUTS)
        packages = data['package_ignore_inventory']
        self.assertEqual(len(packages), 41)
        expected = {str(p.relative_to(capture.ROOT)) for p in (capture.ROOT / 'ws/src').rglob('package.xml')}
        self.assertEqual({r['path'] for r in packages}, expected)
        ignored = [r for r in packages if r['ignore_markers']]
        self.assertEqual(len(ignored), 1)
        self.assertIn('/deeprobotics_local_height_map/', ignored[0]['path'])
        self.assertEqual(data['temporary_containers_remaining']['rc'], 0)
        self.assertEqual(data['temporary_containers_remaining']['stdout'], '')

    def test_height_map_missing_inputs_exact(self):
        data = json.loads((HERE / 'supplement.json').read_text())['height_map']
        rows = data['declared_sources']
        self.assertEqual([r['line'] for r in rows], [80,81,82,83,84,85,86,87,95,102,109,123,130,142,143,144])
        self.assertEqual(sum(r['class'] == 'production' for r in rows), 13)
        self.assertEqual(sum(r['class'] == 'test' for r in rows), 3)
        self.assertFalse(any(r['is_file'] for r in rows))
        for row in rows:
            self.assertFalse((capture.ROOT / row['path']).is_file())
        self.assertTrue(data['catkin_ignore'])
        self.assertEqual(data['catkin_ignore_size'], 0)
        self.assertEqual(data['source_files'], [])


if __name__ == '__main__':
    unittest.main(verbosity=2)
