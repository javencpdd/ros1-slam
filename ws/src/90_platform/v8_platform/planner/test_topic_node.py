"""Pure adapter tests: support planner request/path/refusal."""
import json
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import topic_node  # noqa: E402
import support_planner  # noqa: E402

FIXTURE = HERE.parent / 'contracts' / 'fixtures' / 'map_ml_two_surfaces.json'


class TopicNodeTests(unittest.TestCase):
    def test_map_envelope_and_plan(self):
        value = json.loads(FIXTURE.read_text())
        envelope = json.dumps({'ok': True, 'map': value})
        self.assertEqual(topic_node.map_from_message(envelope), value)
        result = topic_node.plan_request(value, (3., 2., .6), (3.5, 2., .6))
        points = topic_node.legacy.to_path_points(result)
        self.assertGreaterEqual(len(points), 2)
        self.assertLess(((points[-1][0] - 3.5) ** 2 + (points[-1][1] - 2.) ** 2
                         + (points[-1][2] - .6) ** 2) ** .5, .1)

    def test_raw_map_and_bad_messages(self):
        value = json.loads(FIXTURE.read_text())
        self.assertEqual(topic_node.map_from_message(json.dumps(value)), value)
        for text in ('[]', '"x"', '{"map": []}', '3'):
            with self.assertRaises(ValueError):
                topic_node.map_from_message(text)

    def test_cancel_predicate_is_honoured(self):
        value = json.loads(FIXTURE.read_text())
        calls = []

        def cancel():
            calls.append(1)
            return len(calls) >= 2
        with self.assertRaises(support_planner.PlannerError) as caught:
            support_planner.plan(value, (3., 2., .6), (3.8, 2., .6), cancel=cancel)
        self.assertEqual(caught.exception.code, 'E_PLAN_CANCELLED')

    def test_refusal_keeps_reason_code(self):
        try:
            support_planner.plan({}, (0., 0., 0.), (1., 0., 0.))
        except support_planner.PlannerError as error:
            refused = topic_node.legacy.refusal(error)
        else:
            self.fail('expected PlannerError')
        self.assertFalse(refused['ok'])
        self.assertEqual(refused['reason'], 'E_PLAN_MAP_INVALID')


if __name__ == '__main__':
    unittest.main()
