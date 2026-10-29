#!/usr/bin/env python3
"""Process isolation: planning must not starve the parent heartbeat."""
import json
import threading
import time
import unittest
from pathlib import Path

import local_native as native
from ros_node import PlannerWorker

HERE = Path(__file__).resolve().parent


def flat():
    value = json.loads((HERE.parent / 'contracts/fixtures/map_single_floor.json').read_text())
    value['surfaces'][0]['footprint'].update(min_x=-2.0, max_x=3.0, min_y=-2.0, max_y=2.0)
    return value


def heavy_plan(value):
    # 2.5 m forces a long horizon search (hundreds of ms) on this fixture.
    return native.plan(value, (0., 0., 0.), (2.5, 0., 0.))


class Heartbeat:
    def __init__(self):
        self.stamps = []
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while not self.stop.is_set():
            self.stamps.append(time.monotonic())
            time.sleep(.005)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1.)
        return max((b - a for a, b in zip(self.stamps, self.stamps[1:])), default=0.)


class IsolationTest(unittest.TestCase):
    def test_worker_returns_plan(self):
        worker = PlannerWorker(native.Config(max_expansions=2000), None)
        try:
            self.assertTrue(worker.submit(flat(), (0., 0., 0.), (1., 0., 0.), [], 10., 0))
            deadline = time.monotonic() + 5.
            payload = None
            while time.monotonic() < deadline and payload is None:
                payload = worker.poll()
                time.sleep(.01)
            self.assertIsNotNone(payload)
            self.assertTrue(payload['ok'])
            self.assertEqual(payload['result']['reason'], 'arrived')
        finally:
            worker.close()

    def test_worker_keeps_parent_heartbeat_responsive(self):
        value = flat()
        beat = Heartbeat()
        started = time.monotonic()
        try:
            for _ in range(4):
                heavy_plan(value)
            elapsed = time.monotonic() - started
            in_thread_gap = beat.close()
        finally:
            if not beat.stop.is_set():
                beat.close()

        worker = PlannerWorker(native.Config(max_expansions=20000), None)
        beat = Heartbeat()
        try:
            for _ in range(4):
                self.assertTrue(worker.submit(value, (0., 0., 0.), (2.5, 0., 0.), [], 0., 0))
                deadline = time.monotonic() + 10.
                while time.monotonic() < deadline:
                    payload = worker.poll()
                    if payload is not None:
                        break
                    time.sleep(.005)
                self.assertIsNotNone(payload)
                self.assertTrue(payload['ok'])
            worker_gap = beat.close()
        finally:
            if not beat.stop.is_set():
                beat.close()
            worker.close()

        # Python yields the GIL every few ms, so both paths keep the parent
        # heartbeat alive; this is a negative control for the GIL hypothesis.
        print('heavy_plan_s=%.3f in_thread_gap=%.4f worker_gap=%.4f'
              % (elapsed, in_thread_gap, worker_gap))
        self.assertGreater(elapsed, .2)
        self.assertLess(max(in_thread_gap, worker_gap), .10)


if __name__ == '__main__':
    unittest.main()
