"""Runtime map provider core: apply map_delta/v1, snapshot/status."""
import json
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import delta_map_node as node  # noqa: E402
import test_map_delta as td  # noqa: E402
import test_map_service as tm  # noqa: E402


def core(state_path=None):
    return node.DeltaNodeCore(tm.two_floors(), state_path=state_path)


def valid_delta(base=1, delta_id='d1'):
    return td.delta(base, [{'op': 'upsert_surface', 'surface': td.changed_surface()}],
                    delta_id=delta_id)


class DeltaMapNodeTests(unittest.TestCase):
    def test_apply_publishes_new_version_and_replay_is_idempotent(self):
        value = core()
        first = value.apply_text(json.dumps(valid_delta()))
        self.assertTrue(first['ok'])
        self.assertEqual(first['map_version'], 2)
        snapshot = value.snapshot()
        self.assertEqual(snapshot['map_version'], 2)
        self.assertEqual(snapshot['map']['surfaces'][1]['z_reference'], .8)
        again = value.apply_text(json.dumps(valid_delta()))
        self.assertEqual(again['map_version'], 2)
        self.assertEqual(value.status()['applied_deltas'], 1)

    def test_stale_bad_json_and_map_id_are_reported_without_change(self):
        value = core()
        for text, reason in ((json.dumps(valid_delta(base=99)), 'E_MAPSERVICE_STALE'),
                             ('{not json', 'E_DELTA_JSON'),
                             (json.dumps(td.delta(1, [{'op': 'upsert_surface', 'surface':
                                          td.changed_surface()}], map_id='other')),
                              'E_MAPSERVICE_MAP_ID')):
            result = value.apply_text(text)
            self.assertFalse(result['ok'])
            self.assertEqual(result['reason'], reason)
            self.assertEqual(value.snapshot()['map_version'], 1)
        self.assertEqual(len(value.status()['errors']), 3)

    def test_state_path_restores_version_and_replay_idempotency(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'delta.state.json'
            first = core(path)
            self.assertFalse(first.restored)
            self.assertTrue(first.apply_text(json.dumps(valid_delta()))['persisted'])
            second = core(path)
            self.assertTrue(second.restored)
            self.assertEqual(second.snapshot()['map_version'], 2)
            self.assertEqual(second.status()['applied_deltas'], 1)
            replay = second.apply_text(json.dumps(valid_delta()))
            self.assertEqual(replay['map_version'], 2)
            self.assertEqual(second.status()['applied_deltas'], 1)
            third = core(path)
            self.assertEqual(third.snapshot()['map_version'], 2)
            self.assertEqual(third.status()['applied_deltas'], 1)

    def test_status_reports_history_and_evidence(self):
        value = core()
        value.apply_text(json.dumps(valid_delta()))
        status = value.status()
        self.assertEqual([row[0] for row in status['history']], [1, 2])
        self.assertEqual(status['evidence'], {'state': 'no_evidence_layer'})


if __name__ == '__main__':
    unittest.main()
