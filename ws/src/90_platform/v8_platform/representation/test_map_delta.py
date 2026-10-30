"""MapDelta: base-version, idempotent replay, atomic apply, notifications."""
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import map_service as svc  # noqa: E402
import test_map_service as tm  # noqa: E402


def delta(base, ops, delta_id='d1', map_id='svc_map'):
    return {'schema_version': 1, 'map_id': map_id, 'base_version': base,
            'delta_id': delta_id,
            'source': {'uri': 'local://cache', 'sha256': 'b' * 64},
            'ops': ops}


def changed_surface():
    return tm.changed_floors(tm.two_floors())['surfaces'][1]


class MapDeltaTests(unittest.TestCase):
    def service(self):
        service = svc.MapService('svc_map')
        service.publish(tm.two_floors())
        return service

    def test_upsert_bumps_version_and_notifies(self):
        service = self.service()
        seen = []
        service.subscribe(seen.append)
        version = service.propose_delta(delta(1, [
            {'op': 'upsert_surface', 'surface': changed_surface()}]))
        self.assertEqual(version, 2)
        self.assertEqual(seen, [2])
        mezz = next(s for s in service.current['surfaces'] if s['surface_id'] == 'mezz')
        self.assertEqual(mezz['z_reference'], 0.8)
        self.assertEqual(len(service.current['source']['sha256']), 64)

    def test_replay_is_idempotent(self):
        service = self.service()
        seen = []
        service.subscribe(seen.append)
        first = delta(1, [{'op': 'upsert_surface', 'surface': changed_surface()}])
        self.assertEqual(service.propose_delta(first), 2)
        self.assertEqual(service.propose_delta(first), 2)
        self.assertEqual(seen, [2])

    def test_replay_with_different_content_is_rejected(self):
        service = self.service()
        service.propose_delta(delta(1, [
            {'op': 'upsert_surface', 'surface': changed_surface()}]))
        with self.assertRaises(svc.MapServiceError) as caught:
            service.propose_delta(delta(2, [
                {'op': 'remove_surface', 'surface_id': 'mezz'}]))
        self.assertEqual(caught.exception.code, 'E_DELTA_REPLAY_CONFLICT')
        self.assertEqual(service.current['map_version'], 2)

    def test_replay_idempotency_survives_snapshot_restore(self):
        service = self.service()
        first = delta(1, [{'op': 'upsert_surface', 'surface': changed_surface()}])
        self.assertEqual(service.propose_delta(first), 2)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'delta.state.json'
            service.save(path)
            restored = svc.MapService.load(path)
            self.assertEqual(restored.propose_delta(first), 2)  # replay across restart
            self.assertEqual(len(restored.applied_deltas), 1)
            second = delta(2, [{'op': 'remove_surface', 'surface_id': 'mezz'}],
                           delta_id='d2')
            # removing mezz while ground references it is contract-rejected;
            # keep the version and prove the restored base is usable.
            with self.assertRaises(svc.MapServiceError) as caught:
                restored.propose_delta(second)
            self.assertEqual(caught.exception.code, 'E_MAPSERVICE_CONTRACT')
            self.assertEqual(restored.current['map_version'], 2)

    def test_stale_base_and_unknown_remove_are_rejected(self):
        service = self.service()
        with self.assertRaises(svc.MapServiceError) as caught:
            service.propose_delta(delta(99, [
                {'op': 'remove_surface', 'surface_id': 'mezz'}]))
        self.assertEqual(caught.exception.code, 'E_MAPSERVICE_STALE')
        with self.assertRaises(svc.MapServiceError) as caught:
            service.propose_delta(delta(1, [
                {'op': 'remove_surface', 'surface_id': 'missing'}]))
        self.assertEqual(caught.exception.code, 'E_DELTA_OP')
        self.assertEqual(service.current['map_version'], 1)

    def test_invalid_result_is_atomic(self):
        service = self.service()
        bad = changed_surface()
        bad['edges'] = [{'to': 'ghost', 'kind': 'slope', 'max_slope_deg': 12.0,
                         'bidirectional': True}]
        with self.assertRaises(svc.MapServiceError) as caught:
            service.propose_delta(delta(1, [{'op': 'upsert_surface', 'surface': bad}]))
        self.assertEqual(caught.exception.code, 'E_MAPSERVICE_CONTRACT')
        self.assertEqual(service.current['map_version'], 1)

    def test_remove_referenced_surface_is_rejected(self):
        service = self.service()
        with self.assertRaises(svc.MapServiceError) as caught:
            service.propose_delta(delta(1, [
                {'op': 'remove_surface', 'surface_id': 'ground'}]))
        self.assertEqual(caught.exception.code, 'E_MAPSERVICE_CONTRACT')
        self.assertEqual(service.current['map_version'], 1)

    def test_noop_delta_is_rejected(self):
        service = self.service()
        with self.assertRaises(svc.MapServiceError) as caught:
            service.propose_delta(delta(1, [
                {'op': 'upsert_surface', 'surface': tm.two_floors()['surfaces'][1]}]))
        self.assertEqual(caught.exception.code, 'E_DELTA_NO_CHANGE')
        self.assertEqual(service.current['map_version'], 1)


if __name__ == '__main__':
    unittest.main()
