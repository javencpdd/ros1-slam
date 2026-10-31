"""Portable map bundle export/import (state + maps + sha256 manifest)."""
import json
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import map_bundle  # noqa: E402
import map_service  # noqa: E402

FIXTURE = HERE.parent / 'contracts' / 'fixtures' / 'map_single_floor.json'


def make_state(root, map_id, map_path):
    value = json.loads(FIXTURE.read_text())
    value['map_id'] = map_id
    value['surfaces'][0]['data_uri'] = str(map_path)
    service = map_service.MapService(map_id)
    service.publish(value)
    bumped = json.loads(json.dumps(value))
    bumped['source'] = dict(bumped.get('source', {}))
    bumped['source']['uri'] = 'test://' + map_id + '/v2'
    service.update(bumped)
    store = map_service.MapStore(root)
    store.save(service)
    return service


class MapBundleTests(unittest.TestCase):
    def test_export_import_relocates_state_and_map_files(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            old_maps = base / 'old_maps'
            old_maps.mkdir()
            map_file = old_maps / 'floor.json'
            map_file.write_text('{"points": 3}\n')
            state = base / 'state_a'
            make_state(state, 'map_a', map_file)
            make_state(state, 'map_b', map_file)
            bundle = base / 'bundle'
            exported = map_bundle.export_bundle(state, bundle, [map_file])
            self.assertEqual([item['map_id'] for item in exported['maps']],
                             ['map_a', 'map_b'])
            self.assertEqual(exported['files'], 3)
            new_state, new_maps = base / 'state_b', base / 'new_maps'
            imported = map_bundle.import_bundle(bundle, new_state, new_maps,
                                                original_root=old_maps)
            self.assertEqual([item['map_id'] for item in imported['maps']],
                             ['map_a', 'map_b'])
            self.assertEqual((new_maps / 'floor.json').read_text(), '{"points": 3}\n')
            for map_id in ('map_a', 'map_b'):
                service = map_service.MapService.load(new_state / (map_id + '.json'))
                self.assertEqual(service.current['map_version'], 3)
                self.assertEqual(service.current['surfaces'][0]['data_uri'],
                                 str(new_maps / 'floor.json'))

    def test_tampered_file_is_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            map_file = base / 'floor.json'
            map_file.write_text('{}\n')
            state = base / 'state'
            make_state(state, 'map_a', map_file)
            bundle = base / 'bundle'
            map_bundle.export_bundle(state, bundle, [map_file])
            target = bundle / 'state' / 'map_a.json'
            target.write_text(target.read_text() + ' ')
            with self.assertRaises(map_bundle.BundleError) as caught:
                map_bundle.import_bundle(bundle, base / 'new_state')
            self.assertEqual(caught.exception.code, 'E_BUNDLE_HASH')

    def test_path_traversal_and_missing_manifest_are_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            bundle = base / 'bundle'
            bundle.mkdir()
            manifest = {'schema': 'map_bundle/v1', 'maps': [],
                        'files': [{'path': '../escape.json', 'sha256': 'a' * 64,
                                   'bytes': 0}]}
            (bundle / 'bundle.json').write_text(json.dumps(manifest))
            with self.assertRaises(map_bundle.BundleError) as caught:
                map_bundle.verify_bundle(bundle)
            self.assertEqual(caught.exception.code, 'E_BUNDLE_PATH')
            with self.assertRaises(map_bundle.BundleError) as caught:
                map_bundle.verify_bundle(base / 'missing')
            self.assertEqual(caught.exception.code, 'E_BUNDLE_MANIFEST')

    def test_cli_export_import_and_error_rc(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            map_file = base / 'floor.json'
            map_file.write_text('{}\n')
            state = base / 'state'
            make_state(state, 'map_a', map_file)
            bundle = base / 'bundle'
            self.assertEqual(map_bundle.main(['export', str(state), str(bundle),
                                              '--map-file', str(map_file)]), 0)
            self.assertEqual(map_bundle.main(['import', str(bundle),
                                              str(base / 'state2')]), 0)
            self.assertEqual(map_bundle.main(['import', str(base / 'nope'),
                                              str(base / 'state3')]), 5)


if __name__ == '__main__':
    unittest.main()
