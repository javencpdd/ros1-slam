import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import open_data_import as imp


def tiny_pcd(path):
    rows = []
    for i in range(40):
        rows.append('%f %f %f 0' % (0.1 * (i % 10), 0.1 * (i // 10), 0.2 + .01 * i))
    path.write_text('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\n'
                    'SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
                    'WIDTH 40\nHEIGHT 1\nPOINTS 40\nDATA ascii\n' + '\n'.join(rows) + '\n')


def tiny_world(root):
    model = root / 'models' / 'tiny_model'
    model.mkdir(parents=True)
    (model / 'model.config').write_text('<model><name>tiny</name></model>')
    (model / 'model.sdf').write_text('<sdf version="1.6"><model name="tiny"/></sdf>')
    world = root / 'tiny.world'
    world.write_text('<?xml version="1.0"?><sdf version="1.6"><world name="tiny">'
                     '<include><uri>model://tiny_model</uri></include></world></sdf>')
    return world, model


class OpenImportTests(unittest.TestCase):
    def test_map_import_creates_hashed_occupancy_and_verifies(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            pcd = root / 'source.pcd'
            tiny_pcd(pcd)
            dest = root / 'imported'
            record = imp.import_map(pcd, dest, 'tiny', z_min=0., z_max=1.)
            self.assertEqual(record['points'], 40)
            self.assertTrue((dest / 'map.pgm').exists())
            self.assertTrue((dest / 'map.yaml').exists())
            self.assertGreater(record['occupied_cells'], 0)
            self.assertEqual(len(imp.verify(dest)), 1)
            (dest / 'map.pgm').write_bytes(b'bad')
            with self.assertRaises(imp.ImportError_):
                imp.verify(dest)

    def test_world_import_copies_referenced_model(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            world, _model = tiny_world(root)
            dest = root / 'world_import'
            record = imp.import_world(world, dest, 'tiny_world', [root / 'models'])
            self.assertEqual(record['models'][0]['name'], 'tiny_model')
            self.assertTrue((dest / 'models/tiny_model/model.sdf').exists())
            self.assertTrue(imp.verify(dest))

            missing = root / 'missing.world'
            missing.write_text('<?xml version="1.0"?><sdf version="1.6"><world name="x">'
                               '<include><uri>model://absent</uri></include></world></sdf>')
            with self.assertRaises(imp.ImportError_):
                imp.import_world(missing, root / 'bad_world', 'bad')

    def test_dataset_import_and_verify_are_recursive(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            pcd = root / 'source.pcd'
            tiny_pcd(pcd)
            world, _model = tiny_world(root)
            descriptor = root / 'dataset.json'
            descriptor.write_text(json.dumps({
                'source': 'unit://fixture', 'license': 'test',
                'map': str(pcd), 'z_min': 0., 'z_max': 1.,
                'world': str(world), 'model_paths': [str(root / 'models')]}))
            dest = root / 'dataset_import'
            record = imp.import_dataset(descriptor, dest, 'tiny_dataset')
            self.assertEqual(sorted(a['role'] for a in record['artifacts']), ['map', 'world'])
            verified = imp.verify(dest)
            self.assertEqual(sorted(v['kind'] for v in verified), ['dataset', 'map', 'world'])

    def test_cli_rejects_bad_pcd(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            pcd = root / 'bad.pcd'
            pcd.write_text('not a pcd')
            code = imp.main(['map', '--pcd', str(pcd), '--id', 'x', '--out', str(root / 'x')])
            self.assertEqual(code, 5)


if __name__ == '__main__':
    unittest.main()
