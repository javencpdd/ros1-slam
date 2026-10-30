"""Multi-map node core: map_id routing and restart persistence without ROS."""
import json
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import multi_map_node as node  # noqa: E402
import test_map_delta as td  # noqa: E402
import test_map_service as tm  # noqa: E402


def value(map_id):
    item = tm.two_floors()
    item['map_id'] = map_id
    return item


class MultiMapNodeCoreTests(unittest.TestCase):
    def test_two_maps_route_and_restore(self):
        with tempfile.TemporaryDirectory() as directory:
            first = node.MultiMapCore(directory, [value('map_a'), value('map_b')])
            result = first.apply_text(json.dumps(td.delta(
                1, [{'op': 'upsert_surface', 'surface': td.changed_surface()}],
                delta_id='a1', map_id='map_a')))
            self.assertTrue(result['ok'])
            self.assertEqual(first.snapshot('map_a')['map_version'], 2)
            self.assertEqual(first.snapshot('map_b')['map_version'], 1)
            second = node.MultiMapCore(directory, [value('map_a'), value('map_b')])
            self.assertEqual(second.snapshot('map_a')['map_version'], 2)
            self.assertEqual(second.snapshot('map_b')['map_version'], 1)
            self.assertEqual(second.status()['maps']['map_a']['applied_deltas'], 1)

    def test_bad_delta_does_not_change_maps(self):
        with tempfile.TemporaryDirectory() as directory:
            core = node.MultiMapCore(directory, [value('map_a')])
            for text in ('{not json',
                         json.dumps(td.delta(
                             1, [{'op': 'upsert_surface', 'surface': td.changed_surface()}],
                             delta_id='x', map_id='missing'))):
                result = core.apply_text(text)
                self.assertFalse(result['ok'])
                self.assertEqual(core.snapshot('map_a')['map_version'], 1)
            self.assertTrue(core.status()['errors'])


if __name__ == '__main__':
    unittest.main()
