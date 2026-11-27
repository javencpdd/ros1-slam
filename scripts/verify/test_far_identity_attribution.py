#!/usr/bin/env python3
"""FAR 航点身份归因纯逻辑负控：逐条原因、request_id、task_n 计数。"""
import json
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import far_identity_runtime_ctl as ctl  # noqa: E402


class Msg(object):
    def __init__(self, data):
        self.data = data


BASE = {'rejected': 10, 'stale_gen': 1, 'bad': 1, 'unnamed': 1}


def row(rejected, stale=1, bad=1, unnamed=1, rid='r1'):
    return {'rejected': rejected, 'stale_gen': stale, 'bad': bad,
            'unnamed': unnamed, 'request_id': rid}


class AttributionTest(unittest.TestCase):
    def test_two_distinct_reasons_pass(self):
        rows = [dict(BASE, step='producer'), row(11, stale=2),
                row(12, stale=3), row(13, stale=1, bad=2, unnamed=2)]
        self.assertTrue(ctl._attributed(rows, BASE))

    def test_total_only_is_not_attribution(self):
        rows = [dict(BASE, step='producer'), row(11), row(12), row(13)]
        self.assertFalse(ctl._attributed(rows, BASE))

    def test_missing_request_id_fails(self):
        rows = [dict(BASE, step='producer'), row(11, stale=2),
                row(12, stale=3), row(13, stale=3, bad=2, rid=None)]
        self.assertFalse(ctl._attributed(rows, BASE))

    def test_missing_rejected_fails(self):
        rows = [dict(BASE, step='producer'), dict(row(11, stale=2), rejected=None),
                row(12, stale=3), row(13, stale=3, bad=2)]
        self.assertFalse(ctl._attributed(rows, BASE))

    def test_rejected_delta_is_incremental_and_guards_wrap(self):
        self.assertEqual(ctl._rej_delta({'rejected': 13}, 10), 3)
        self.assertEqual(ctl._rej_delta({'rejected': 9}, 10), 0)
        self.assertEqual(ctl._rej_delta({}, 10), 0)


class TaskCounterTest(unittest.TestCase):
    def test_task_frames_count_even_when_json_malformed(self):
        rec = ctl.Rec()
        rec.on_task(Msg('not-json'))
        rec.on_task(Msg(json.dumps({'path_revision': 7, 'waypoint_index': 2,
                                    'poses': [[[1.0, 2.0]], [[3.0, 4.0]]]})))
        self.assertEqual(rec.task_n, 2)
        self.assertEqual(len(rec.task_hist), 1)
        self.assertEqual(rec.task_hist[0], {'rev': 7, 'idx': 2, 'wp': [[3.0, 4.0]]})

    def test_cmd_counter_ignores_zero_frames(self):
        rec = ctl.Rec()

        class T(object):
            def __init__(self, x, z):
                self.linear = type('L', (), {'x': x})
                self.angular = type('A', (), {'z': z})

        rec.on_cmd(T(0.0, 0.0))
        rec.on_cmd(T(0.01, 0.0))
        self.assertEqual(rec.cmd_n, 1)


if __name__ == '__main__':
    unittest.main()
