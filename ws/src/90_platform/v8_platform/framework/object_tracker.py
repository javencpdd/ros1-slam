"""Multi-object constant-velocity tracking for dynamic obstacle boxes (pure Python).

One 4-state Kalman filter per track (px, py, vx, vy). Association uses gated
nearest predicted position; unmatched tracks coast for a few frames and are then
dropped, unmatched detections start new tracks. Boxes keep their detected size
and are shifted to the filtered centre, so the planner sees a smooth obstacle.
"""
import math

import numpy as np


class Track(object):
    def __init__(self, track_id, box, dt=0.1, sigma_a=0.8, sigma_pos=0.08,
                 max_speed_mps=1.5):
        self.id = track_id
        self.dt = dt
        self.sigma_a = sigma_a
        self.sigma_pos = sigma_pos
        self.max_speed = max_speed_mps
        self.missed = 0
        self.age = 0
        self.center = self._center(box)
        self.state = np.array([self.center[0], self.center[1], 0.0, 0.0])
        self.cov = np.diag([sigma_pos ** 2, sigma_pos ** 2,
                            max_speed_mps ** 2, max_speed_mps ** 2])
        self.size = (box['max'][0] - box['min'][0],
                     box['max'][1] - box['min'][1],
                     box['min'][2], box['max'][2])

    @staticmethod
    def _center(box):
        return ((box['min'][0] + box['max'][0]) / 2.0,
                (box['min'][1] + box['max'][1]) / 2.0)

    def _transition(self, dt):
        return np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)

    def _process_noise(self, dt):
        q = self.sigma_a ** 2
        dt2, dt3, dt4 = dt * dt, dt ** 3, dt ** 4
        return q * np.array([[dt4 / 4, 0, dt3 / 2, 0],
                             [0, dt4 / 4, 0, dt3 / 2],
                             [dt3 / 2, 0, dt2, 0],
                             [0, dt3 / 2, 0, dt2]])

    def predict(self, dt):
        if dt > 0:
            f = self._transition(dt)
            self.state = f.dot(self.state)
            self.cov = f.dot(self.cov).dot(f.T) + self._process_noise(dt)
            speed = math.hypot(self.state[2], self.state[3])
            if speed > self.max_speed and speed > 0:
                self.state[2] *= self.max_speed / speed
                self.state[3] *= self.max_speed / speed
        self.missed += 1
        return self.state

    def update(self, box, dt=0.0):
        """Correct with a matched detection (predict is done once by the caller)."""
        center = self._center(box)
        z = np.array([center[0], center[1]])
        h = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
        r = np.eye(2) * self.sigma_pos ** 2
        innovation = z - h.dot(self.state)
        s = h.dot(self.cov).dot(h.T) + r
        gain = self.cov.dot(h.T).dot(np.linalg.inv(s))
        self.state = self.state + gain.dot(innovation)
        self.cov = (np.eye(4) - gain.dot(h)).dot(self.cov)
        self.missed = 0
        self.age += 1
        self.size = (box['max'][0] - box['min'][0],
                     box['max'][1] - box['min'][1],
                     box['min'][2], box['max'][2])

    def predicted_center(self):
        return (float(self.state[0]), float(self.state[1]))

    def box(self):
        cx, cy = self.predicted_center()
        sx, sy, zmin, zmax = self.size
        return {'min': [cx - sx / 2.0, cy - sy / 2.0, zmin],
                'max': [cx + sx / 2.0, cy + sy / 2.0, zmax]}

    def velocity(self):
        return (float(self.state[2]), float(self.state[3]), 0.0)


class MultiObjectTracker(object):
    def __init__(self, gate_m=0.8, max_miss=3, sigma_a=0.8, sigma_pos=0.08,
                 max_speed_mps=1.5, max_tracks=32):
        self.gate_m = gate_m
        self.max_miss = max_miss
        self.sigma_a = sigma_a
        self.sigma_pos = sigma_pos
        self.max_speed = max_speed_mps
        self.max_tracks = max_tracks
        self.tracks = []
        self.next_id = 1

    def _spawn(self, box, dt):
        track = Track(self.next_id, box, dt, self.sigma_a, self.sigma_pos, self.max_speed)
        self.next_id += 1
        self.tracks.append(track)

    def update(self, boxes, dt):
        """boxes: list of {'min','max'}; returns live tracks (list)."""
        if dt > 0:
            for track in self.tracks:
                track.predict(dt)
        pairs = []
        for ti, track in enumerate(self.tracks):
            px, py = track.predicted_center()
            for di, box in enumerate(boxes):
                cx = (box['min'][0] + box['max'][0]) / 2.0
                cy = (box['min'][1] + box['max'][1]) / 2.0
                distance = math.hypot(cx - px, cy - py)
                if distance <= self.gate_m:
                    pairs.append((distance, ti, di))
        pairs.sort()
        used_tracks, used_boxes = set(), set()
        for distance, ti, di in pairs:
            if ti in used_tracks or di in used_boxes:
                continue
            used_tracks.add(ti)
            used_boxes.add(di)
            self.tracks[ti].update(boxes[di], 0.0)
        for di, box in enumerate(boxes):
            if di not in used_boxes and len(self.tracks) < self.max_tracks:
                self._spawn(box, dt)
        self.tracks = [t for t in self.tracks if t.missed <= self.max_miss]
        return list(self.tracks)
