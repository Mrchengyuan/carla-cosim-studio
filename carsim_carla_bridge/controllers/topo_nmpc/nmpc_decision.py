"""Feasible escape after recovery gates; confirmed ordinary lane changes."""
import math


class PlanSelector:
    def __init__(self, confirm_time=0.25):
        self.confirm_time = confirm_time
        self.pending = None
        self.pending_since = None
        self.infeasible_frames = 0
        self.last_t = None
        self.context = None
        self.reason = ""

    def _clear(self):
        self.pending = None
        self.pending_since = None

    def choose(self, keys, scores, violation, lane_idx, target_idx, t,
               dwell_until=-1.0, abort_until=-1.0, recovery_ready=True):
        context = (lane_idx, target_idx)
        if context != self.context or (self.last_t is not None and t < self.last_t):
            self._clear()
            self.infeasible_frames = 0
        self.context = context
        self.last_t = t
        safe = [i for i in range(len(keys))
                if math.isfinite(float(scores[i])) and math.isfinite(float(violation[i]))
                and violation[i] <= 0.0]
        admission_blocked = not recovery_ready or t < max(dwell_until, abort_until)
        stay = [i for i, key in enumerate(keys) if key[1] == lane_idx]
        if not stay:
            raise ValueError("A current-lane or yield hypothesis is required")
        safe_stay = [i for i in safe if i in stay]

        def cheapest(indices):
            return min(indices, key=lambda i: float(scores[i]))

        def fallback():
            return min(stay, key=lambda i: (
                float(violation[i]) if math.isfinite(float(violation[i])) else math.inf,
                float(scores[i]) if math.isfinite(float(scores[i])) else math.inf))

        if not safe:
            self._clear()
            self.infeasible_frames += 1
            self.reason = "no feasible plan: brake in current lane"
            return fallback(), True
        self.infeasible_frames = 0

        if target_idx != lane_idx:
            target = [i for i in safe if keys[i] == ("lane", target_idx)]
            self._clear()
            if target:
                self.reason = "continue feasible lane change"
                return cheapest(target), False
            if safe_stay:
                self.reason = "unsafe target: abort immediately"
                return cheapest(safe_stay), False
            self.reason = "unsafe target and no safe return: emergency brake"
            return fallback(), True

        # Every selected side corridor must be feasible in the current frame.
        # If staying has no feasible plan, waiting can destroy a still-feasible
        # escape. Recovery and manoeuvre gates remain mandatory before entry.
        # With a feasible stay option, confirm sustained cost advantage first.
        best = cheapest(safe)
        keep = cheapest(safe_stay) if safe_stay else fallback()
        brake_while_waiting = not bool(safe_stay)
        if keys[best][1] == lane_idx:
            self._clear()
            self.reason = "current lane preferred"
            return best, False
        if admission_blocked:
            self._clear()
            self.reason = "recovering vehicle pose or waiting for manoeuvre cooldown"
            return keep, brake_while_waiting

        if not safe_stay:
            # This branch is deliberately AFTER the gates, and AFTER the
            # active-target branch: an unsafe active change must first abort
            # into the current lane, never reverse directly to the other side.
            self._clear()
            self.reason = "current corridor unsafe: enter currently feasible escape"
            return best, False
        candidate = keys[best]
        if self.pending != candidate:
            self.pending = candidate
            self.pending_since = t
        if t - self.pending_since + 1e-9 < self.confirm_time:
            self.reason = "confirming feasible lane-change advantage"
            return keep, brake_while_waiting
        self._clear()
        self.reason = "confirmed feasible lane change"
        return best, False
