"""Map cache/merge core tests (no ROS)."""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import map_cache  # noqa: E402
import map_service as svc  # noqa: E402
import test_map_service as tm  # noqa: E402


def surface(surface_id, z):
    return {'surface_id': surface_id, 'z_reference': z, 'thickness_m': 0.05,
            'representation': 'elevation_grid', 'data_uri': surface_id + '.json',
            'traversability': 'supported', 'clearance_min_m': 0.4,
            'footprint': {'min_x': 0, 'min_y': 0, 'max_x': 1, 'max_y': 1},
            'edges': []}


class MapCacheTests(unittest.TestCase):
    def test_latest_operation_per_surface_wins(self):
        cache = map_cache.MapCache('svc_map', 1)
        cache.upsert(surface('mezz', 0.7)).upsert(surface('mezz', 0.8))
        self.assertEqual(len(cache.pending()), 1)
        self.assertEqual(cache.pending()[0]['surface']['z_reference'], 0.8)
        cache.remove('mezz')
        self.assertEqual(cache.pending()[0]['op'], 'remove_surface')
        cache.upsert(surface('mezz', 0.9))
        self.assertEqual(cache.pending()[0]['op'], 'upsert_surface')
        self.assertEqual(cache.pending()[0]['surface']['z_reference'], 0.9)

    def test_delta_validation_and_ack(self):
        cache = map_cache.MapCache('svc_map', 1)
        with self.assertRaises(map_cache.MapCacheError):
            cache.make_delta('d0')
        cache.upsert(surface('mezz', 0.8))
        delta = cache.make_delta('d1')
        self.assertEqual(delta['base_version'], 1)
        self.assertEqual(delta['map_id'], 'svc_map')
        self.assertEqual(len(delta['source']['sha256']), 64)
        self.assertEqual(cache.pending(), [delta['ops'][0]])
        with self.assertRaises(map_cache.MapCacheError) as caught:
            cache.ack(3)
        self.assertEqual(caught.exception.code, 'E_CACHE_ACK')
        self.assertEqual(cache.ack(2), 2)
        self.assertEqual(cache.pending(), [])
        self.assertEqual(cache.base_version, 2)

    def test_merge_later_delta_wins_and_rejects_mismatch(self):
        a = map_cache.MapCache('svc_map', 1).upsert(surface('mezz', 0.7)) \
            .make_delta('a')
        b = map_cache.MapCache('svc_map', 1).upsert(surface('mezz', 0.9)) \
            .upsert(surface('wall', 0.2)).make_delta('b')
        merged = map_cache.merge_deltas([a, b])
        self.assertEqual(merged['base_version'], 1)
        by_id = {op.get('surface', op).get('surface_id'): op for op in merged['ops']}
        self.assertEqual(by_id['mezz']['surface']['z_reference'], 0.9)
        self.assertIn('wall', by_id)
        bad = dict(a, map_id='other')
        with self.assertRaises(map_cache.MapCacheError) as caught:
            map_cache.merge_deltas([a, bad])
        self.assertEqual(caught.exception.code, 'E_MERGE_MAP_ID')
        bad = dict(a, base_version=2)
        with self.assertRaises(map_cache.MapCacheError) as caught:
            map_cache.merge_deltas([a, bad])
        self.assertEqual(caught.exception.code, 'E_MERGE_BASE')

    def test_cache_delta_applies_to_service_and_advances_base(self):
        service = svc.MapService('svc_map')
        self.assertEqual(service.publish(tm.two_floors()), 1)
        cache = map_cache.MapCache('svc_map', service.current['map_version'])
        changed = tm.changed_floors(tm.two_floors())
        mezz = next(item for item in changed['surfaces']
                    if item['surface_id'] == 'mezz')
        cache.upsert(mezz)
        version = service.propose_delta(cache.make_delta('c1'))
        self.assertEqual(version, 2)
        cache.ack(version)
        self.assertEqual(cache.base_version, 2)
        second = map_cache.MapCache('svc_map', cache.base_version)
        wall = surface('wall', 0.2)
        wall['edges'] = [{'to': 'ground', 'kind': 'slope',
                          'max_slope_deg': 12.0, 'bidirectional': True}]
        ground = surface('ground', 0.0)
        ground['edges'] = [{'to': 'wall', 'kind': 'slope',
                            'max_slope_deg': 12.0, 'bidirectional': True}]
        second.remove('mezz').upsert(wall).upsert(ground)
        version = service.propose_delta(second.make_delta('c2'))
        self.assertEqual(version, 3)
        self.assertNotIn('mezz', {item['surface_id']
                                  for item in service.current['surfaces']})


if __name__ == '__main__':
    unittest.main()
