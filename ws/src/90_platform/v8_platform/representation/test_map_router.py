"""Multi-map router: map_id dispatch, independent versions and persistence."""
import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import map_router  # noqa: E402
import map_service as svc  # noqa: E402
import test_map_delta as td  # noqa: E402
import test_map_service as tm  # noqa: E402

FLAT = Path(__file__).resolve().parent.parent / 'contracts/fixtures/map_2d_flat.json'


def map_value(map_id):
    value = tm.two_floors()
    value['map_id'] = map_id
    return value


class MapRouterTests(unittest.TestCase):
    def test_delta_routes_by_map_id_and_versions_stay_independent(self):
        with tempfile.TemporaryDirectory() as directory:
            router = map_router.MapRouter(directory, [map_value('map_a'), map_value('map_b')])
            version = router.apply_text(json.dumps(td.delta(
                1, [{'op': 'upsert_surface', 'surface': td.changed_surface()}],
                delta_id='a1', map_id='map_a')))
            self.assertEqual(version, 2)
            self.assertEqual(router.snapshot('map_a')['map_version'], 2)
            self.assertEqual(router.snapshot('map_b')['map_version'], 1)
            self.assertEqual(router.status()['map_a']['applied_deltas'], 1)
            self.assertEqual(router.status()['map_b']['applied_deltas'], 0)
            self.assertEqual(router.apply_text(json.dumps(td.delta(
                1, [{'op': 'upsert_surface', 'surface': td.changed_surface()}],
                delta_id='a1', map_id='map_a'))), 2)
            with self.assertRaises(svc.MapServiceError) as caught:
                router.apply_text(json.dumps(td.delta(
                    1, [{'op': 'upsert_surface', 'surface': td.changed_surface()}],
                    delta_id='x', map_id='missing')))
            self.assertEqual(caught.exception.code, 'E_MAPSERVICE_MAP_ID')

    def test_reload_restores_each_map_and_rejects_duplicate(self):
        with tempfile.TemporaryDirectory() as directory:
            first = map_router.MapRouter(directory, [map_value('map_a'), map_value('map_b')])
            first.apply_text(json.dumps(td.delta(
                1, [{'op': 'upsert_surface', 'surface': td.changed_surface()}],
                delta_id='a1', map_id='map_a')))
            second = map_router.MapRouter(directory, [map_value('map_a'), map_value('map_b')])
            self.assertEqual(second.snapshot('map_a')['map_version'], 2)
            self.assertEqual(second.snapshot('map_b')['map_version'], 1)
            self.assertEqual(second.status()['map_a']['applied_deltas'], 1)
            self.assertEqual(second.ids(), ['map_a', 'map_b'])
            with self.assertRaises(svc.MapServiceError) as caught:
                second.add(map_value('map_a'))
            self.assertEqual(caught.exception.code, 'E_MAPSERVICE_DUPLICATE')

    def test_occupancy_grid_is_read_only_selectable_and_reloadable(self):
        flat = json.loads(FLAT.read_text())
        with tempfile.TemporaryDirectory() as directory:
            router = map_router.MapRouter(directory, [map_value('map_a'), flat])
            self.assertEqual(router.select('occupancy_grid'), [flat['map_id']])
            self.assertEqual(router.select('support_map'), ['map_a'])
            self.assertEqual(router.snapshot(flat['map_id'])['kind'], 'occupancy_grid')
            self.assertEqual(router.snapshot(flat['map_id'])['evidence']['state'], 'not_applicable')
            with self.assertRaises(svc.MapServiceError) as caught:
                router.apply(td.delta(1, [{'op': 'upsert_surface',
                                            'surface': td.changed_surface()}],
                                      delta_id='flat1', map_id=flat['map_id']))
            self.assertEqual(caught.exception.code, 'E_MAPSERVICE_UPDATE_SCHEMA')
            self.assertEqual(router.apply_text(json.dumps(td.delta(
                1, [{'op': 'upsert_surface', 'surface': td.changed_surface()}],
                delta_id='a1', map_id='map_a'))), 2)
            reloaded = map_router.MapRouter(directory, [map_value('map_a'), flat])
            self.assertEqual(reloaded.select('occupancy_grid'), [flat['map_id']])
            self.assertEqual(reloaded.snapshot(flat['map_id'])['kind'], 'occupancy_grid')
            self.assertEqual(reloaded.snapshot('map_a')['map_version'], 2)

    def test_bad_json_and_foreign_map_id_are_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            router = map_router.MapRouter(directory, [map_value('map_a')])
            with self.assertRaises(svc.MapServiceError) as caught:
                router.apply_text('{not json')
            self.assertEqual(caught.exception.code, 'E_DELTA_JSON')
            wrong = td.delta(1, [{'op': 'upsert_surface', 'surface': td.changed_surface()}],
                             delta_id='x', map_id='map_b')
            with self.assertRaises(svc.MapServiceError) as caught:
                router.apply(wrong)
            self.assertEqual(caught.exception.code, 'E_MAPSERVICE_MAP_ID')
            self.assertEqual(router.snapshot('map_a')['map_version'], 1)


if __name__ == '__main__':
    unittest.main()
