"""A6 recovery supervisor: LOST/RELOCALIZING -> candidate -> new lease -> TRACKING.

The localization quality/lease rules stay in LocalizationState.  This class
only sequences recovery; relocalization and backend switching are injected so
the state machine can be tested without ROS or a real localizer.
"""
import math

TRACKING = 'TRACKING'
RELOCALIZING = 'RELOCALIZING'
RECOVERING = 'RECOVERING'
FAILED = 'FAILED'


class RecoverySupervisor:
    def __init__(self, state, relocalize, switch_backend,
                 score_min=0.0, deadline_s=5.0, bounds=None):
        if score_min < 0 or deadline_s <= 0:
            raise ValueError('score_min >= 0 and deadline_s > 0 required')
        self.state = state
        self.relocalize = relocalize
        self.switch_backend = switch_backend
        self.score_min = score_min
        self.deadline_s = deadline_s
        self.bounds = bounds
        self.phase = TRACKING if state.active_lease is not None else RELOCALIZING
        self.requested = False
        self.deadline = None
        self.pending_lease = None
        self.rejected = 0
        self.recoveries = 0
        self.last_error = None

    def _enter_relocalizing(self, now, reason):
        self.phase = RELOCALIZING
        self.requested = False
        self.pending_lease = None
        self.last_error = reason
        self.deadline = now + self.deadline_s

    def tick(self, now):
        self.state.tick(now)
        if self.phase == RECOVERING and self.pending_lease is not None:
            if self.state.state == 'HEALTHY' and self.state.activate(self.pending_lease, now):
                self.pending_lease = None
                self.phase = TRACKING
                self.recoveries += 1
                return True
            if self.deadline is not None and now > self.deadline:
                self.phase = FAILED
            return False
        healthy = self.state.state == 'HEALTHY' and self.state.active_lease is not None
        if healthy and self.phase == RELOCALIZING:
            self.phase = TRACKING
            return True
        if not healthy:
            if self.phase not in (RELOCALIZING, FAILED):
                self._enter_relocalizing(now, self.state.reason)
            if self.phase == RELOCALIZING:
                if not self.requested:
                    self.state.relocalize()
                    self.relocalize()
                    self.requested = True
                if self.deadline is not None and now > self.deadline:
                    self.phase = FAILED
            return False
        return self.phase == TRACKING

    def offer_candidate(self, pose, score, now):
        if self.phase != RELOCALIZING:
            return False
        if (len(pose) != 3 or not all(isinstance(v, (int, float))
                                      and not isinstance(v, bool) and math.isfinite(v)
                                      for v in pose)
                or not isinstance(score, (int, float)) or not math.isfinite(score)
                or score < self.score_min):
            self.rejected += 1
            return False
        if self.bounds is not None:
            (min_x, max_x), (min_y, max_y) = self.bounds
            if not (min_x <= pose[0] <= max_x and min_y <= pose[1] <= max_y):
                self.rejected += 1
                return False
        lease = self.state.last_lease + 1
        self.pending_lease = lease
        self.phase = RECOVERING
        self.switch_backend(lease, tuple(pose))
        return True

    def command_allowed(self, now):
        return self.tick(now)

    def report(self):
        return {'phase': self.phase, 'requested': self.requested,
                'pending_lease': self.pending_lease, 'rejected': self.rejected,
                'recoveries': self.recoveries, 'last_error': self.last_error,
                'deadline': self.deadline, 'localization': self.state.report()}
