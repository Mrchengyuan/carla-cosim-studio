"""Deterministic safety and anti-chatter checks for the NMPC plan selector.

Run directly: python tests/test_nmpc_decision.py
These tests exercise observable plan choices; no CARLA, solver, or learned net.
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'controllers', 'topo_nmpc')))
from nmpc_decision import PlanSelector
from controller import ABORT_COOLDOWN


KEYS = [('lane', 0), ('lane', 1), ('lane', -1), ('yield', 0)]
STAY, LEFT, RIGHT, YIELD = range(4)


class PlanSelectorTests(unittest.TestCase):
    def setUp(self):
        self.selector = PlanSelector(confirm_time=0.25)

    def choose(self, t, scores=(10.0, 0.0, 20.0, 30.0), violation=(0.0, 0.0, 0.0, 0.0),
               lane_idx=0, target_idx=0, dwell_until=-1, abort_until=-1, recovery_ready=True):
        return self.selector.choose(KEYS, scores, violation, lane_idx, target_idx, t,
                                    dwell_until=dwell_until, abort_until=abort_until, recovery_ready=recovery_ready)

    def test_transient_advantage_does_not_start_lane_change(self):
        for t in (0.0, 0.05, 0.10, 0.20):
            self.assertEqual(self.choose(t)[0], STAY)
        self.assertEqual(self.choose(0.24, scores=(0.0, 10.0, 20.0, 30.0))[0], STAY)
        self.assertEqual(self.choose(0.30)[0], STAY)
        self.assertEqual(self.choose(0.50)[0], STAY)
        self.assertEqual(self.choose(0.60)[0], LEFT)

    def test_sustained_same_target_is_confirmed(self):
        self.assertEqual(self.choose(2.0)[0], STAY)
        self.assertEqual(self.choose(2.20)[0], STAY)
        self.assertEqual(self.choose(2.30)[0], LEFT)

    def test_changing_best_side_restarts_confirmation(self):
        self.assertEqual(self.choose(0.0)[0], STAY)
        self.assertEqual(self.choose(0.20)[0], STAY)
        right_best = (10.0, 20.0, 0.0, 30.0)
        self.assertEqual(self.choose(0.25, scores=right_best)[0], STAY)
        self.assertEqual(self.choose(0.40, scores=right_best)[0], STAY)
        self.assertEqual(self.choose(0.55, scores=right_best)[0], RIGHT)

    def test_feasible_active_target_is_kept_despite_cost_jump(self):
        idx, emergency = self.choose(4.0, scores=(-1e9, 1e6, -1e10, -1e8), target_idx=1)
        self.assertEqual(idx, LEFT)
        self.assertFalse(emergency)

    def test_unsafe_active_target_is_abandoned_immediately(self):
        idx, _ = self.choose(4.0, scores=(30.0, -1e12, 0.0, 10.0),
                             violation=(0.0, 0.01, 0.0, 0.0), target_idx=1)
        self.assertIn(idx, (STAY, YIELD))

    def test_unsafe_active_target_does_not_reverse_into_opposite_lane(self):
        # An ongoing left manoeuvre cannot be converted directly into a right
        # manoeuvre. With no safe in-lane fallback, brake in the current lane.
        args = dict(scores=(100.0, -1e12, -1e12, 50.0),
                    violation=(3.0, 1.0, 0.0, 2.0), target_idx=1)
        idx, _ = self.choose(4.0, **args)
        self.assertIn(idx, (STAY, YIELD))
        idx, emergency = self.choose(4.05, **args)
        self.assertIn(idx, (STAY, YIELD))
        self.assertTrue(emergency)

    def test_nonfinite_active_target_is_abandoned(self):
        idx, _ = self.choose(4.0, scores=(30.0, math.inf, 0.0, 10.0), target_idx=1)
        self.assertIn(idx, (STAY, YIELD))

    def test_feasibility_dominates_arbitrarily_good_cost(self):
        for t in (0.0, 0.10, 0.30, 0.60):
            idx, emergency = self.choose(t, scores=(10.0, -1e12, 20.0, 30.0),
                                         violation=(0.0, 0.01, 0.0, 0.0))
            self.assertEqual(idx, STAY)
            self.assertFalse(emergency)

    def test_nonfinite_candidates_are_not_eligible(self):
        for scores, violation in (
            ((10.0, -math.inf, math.nan, 30.0), (0.0, 0.0, 0.0, 0.0)),
            ((10.0, -1e12, -1e12, 30.0), (0.0, math.nan, math.inf, 0.0)),
        ):
            with self.subTest(scores=scores, violation=violation):
                self.selector = PlanSelector(confirm_time=0.25)
                self.assertEqual(self.choose(0.0, scores=scores, violation=violation)[0], STAY)
                self.assertEqual(self.choose(0.30, scores=scores, violation=violation)[0], STAY)

    def test_all_infeasible_stays_in_lane_and_brakes_from_first_frame(self):
        # Neighbour lanes have smaller violations and much better values, but
        # neither is a safe escape. Choose the less unsafe in-lane plan.
        args = dict(scores=(-100.0, -1e12, -1e12, 50.0), violation=(3.0, 0.1, 0.1, 2.0))
        idx, emergency = self.choose(0.0, **args)
        self.assertEqual(idx, YIELD)
        self.assertTrue(emergency)
        idx, emergency = self.choose(0.05, **args)
        self.assertEqual(idx, YIELD)
        self.assertTrue(emergency)

    def test_emergency_clears_on_feasible_frame_and_returns_immediately(self):
        bad = dict(violation=(3.0, 2.0, 2.0, 1.0))
        _, emergency = self.choose(0.0, **bad)
        self.assertTrue(emergency)
        idx, emergency = self.choose(0.05, scores=(0.0, 10.0, 20.0, 30.0))
        self.assertEqual(idx, STAY)
        self.assertFalse(emergency)
        # A fresh infeasible frame requires braking without a second-frame delay.
        _, emergency = self.choose(0.10, **bad)
        self.assertTrue(emergency)

    def test_feasible_escape_cannot_bypass_any_gate(self):
        for gate in ('recovery_ready', 'abort_until', 'dwell_until'):
            with self.subTest(gate=gate):
                self.selector = PlanSelector(confirm_time=0.25)
                args = dict(scores=(-1e12, 100.0, -1e12, -1e12),
                            violation=(2.0, 0.0, 1.0, 1.0))
                blocked = {gate: False if gate == 'recovery_ready' else 1.0}
                for t in (0.0, 0.30, 0.90):
                    idx, emergency = self.choose(t, **args, **blocked)
                    self.assertEqual(idx, YIELD)
                    self.assertTrue(emergency)
                # Once every gate opens, do not destroy a currently feasible
                # escape by requiring another cost-confirmation interval.
                idx, emergency = self.choose(1.0, **args)
                self.assertEqual(idx, LEFT)
                self.assertFalse(emergency)

    def test_only_feasible_neighbor_is_admitted_immediately(self):
        idx, emergency = self.choose(0.0, violation=(2.0, 0.0, 1.0, 1.0))
        self.assertEqual(idx, LEFT)
        self.assertFalse(emergency)

    def test_normal_lane_change_requires_fresh_confirmation_after_recovery(self):
        for t, ready in ((0.0, True), (0.20, True), (0.24, False), (1.0, False),
                         (1.05, True), (1.25, True)):
            idx, emergency = self.choose(t, recovery_ready=ready)
            self.assertEqual(idx, STAY)
            self.assertFalse(emergency)
        idx, emergency = self.choose(1.35, recovery_ready=True)
        self.assertEqual(idx, LEFT)
        self.assertFalse(emergency)

    def test_feasible_active_target_survives_new_manoeuvre_recovery_gates(self):
        idx, emergency = self.choose(4.0, target_idx=1, recovery_ready=False,
                                     dwell_until=100.0, abort_until=100.0,
                                     scores=(-1e9, 1e6, -1e10, -1e8))
        self.assertEqual(idx, LEFT)
        self.assertFalse(emergency)

    def test_feasible_yield_prevents_emergency_side_switch(self):
        idx, emergency = self.choose(0.0, scores=(10.0, -1e6, 20.0, 30.0),
                                     violation=(1.0, 0.0, 0.0, 0.0), abort_until=100.0)
        self.assertEqual(idx, YIELD)
        self.assertFalse(emergency)

    def test_cooldown_blocks_normal_change_then_requires_fresh_confirmation(self):
        for limit in ('dwell_until', 'abort_until'):
            with self.subTest(limit=limit):
                self.selector = PlanSelector(confirm_time=0.25)
                self.assertEqual(self.choose(0.0)[0], STAY)
                self.assertEqual(self.choose(0.20)[0], STAY)
                kw = {limit: 1.0}
                self.assertEqual(self.choose(0.30, **kw)[0], STAY)
                self.assertEqual(self.choose(0.90, **kw)[0], STAY)
                self.assertEqual(self.choose(1.0, **kw)[0], STAY)
                self.assertEqual(self.choose(1.20, **kw)[0], STAY)
                self.assertEqual(self.choose(1.30, **kw)[0], LEFT)

    def test_infeasible_candidate_resets_confirmation(self):
        self.assertEqual(self.choose(0.0)[0], STAY)
        self.assertEqual(self.choose(0.20)[0], STAY)
        self.assertEqual(self.choose(0.24, violation=(0.0, 0.01, 0.0, 0.0))[0], STAY)
        self.assertEqual(self.choose(0.30)[0], STAY)
        self.assertEqual(self.choose(0.50)[0], STAY)
        self.assertEqual(self.choose(0.60)[0], LEFT)
















    def test_ordinary_confirmation_resets_when_candidate_disappears(self):
        self.assertEqual(self.choose(0.0)[0], STAY)
        self.assertEqual(self.choose(0.20)[0], STAY)
        keys = [KEYS[STAY], KEYS[RIGHT], KEYS[YIELD]]
        idx, _ = self.selector.choose(keys, (0.0, 20.0, 30.0), (0.0, 0.0, 0.0), 0, 0, 0.24)
        self.assertEqual(keys[idx], KEYS[STAY])
        self.assertEqual(self.choose(0.30)[0], STAY)
        self.assertEqual(self.choose(0.50)[0], STAY)
        self.assertEqual(self.choose(0.60)[0], LEFT)

    def test_ordinary_confirmation_resets_on_time_reversal(self):
        self.assertEqual(self.choose(2.0)[0], STAY)
        self.assertEqual(self.choose(2.20)[0], STAY)
        self.assertEqual(self.choose(1.0)[0], STAY)
        self.assertEqual(self.choose(1.20)[0], STAY)
        self.assertEqual(self.choose(1.30)[0], LEFT)

    def test_ordinary_confirmation_resets_on_lane_context_change(self):
        # LEFT retains the same absolute key 1 while the current lane changes.
        self.assertEqual(self.choose(0.0)[0], STAY)
        self.assertEqual(self.choose(0.20)[0], STAY)
        keys = [('lane', -1), ('lane', 1), ('yield', -1)]
        for t in (0.30, 0.50):
            idx, _ = self.selector.choose(keys, (10.0, 0.0, 30.0), (0.0, 0.0, 0.0), -1, -1, t)
            self.assertEqual(keys[idx], ('lane', -1))
        idx, _ = self.selector.choose(keys, (10.0, 0.0, 30.0), (0.0, 0.0, 0.0), -1, -1, 0.60)
        self.assertEqual(keys[idx], ('lane', 1))

    def test_ordinary_confirmation_resets_after_active_target_abort(self):
        self.assertEqual(self.choose(0.0)[0], STAY)
        self.assertEqual(self.choose(0.20)[0], STAY)
        idx, _ = self.choose(0.25, target_idx=-1, violation=(0.0, 0.0, 1.0, 0.0))
        self.assertIn(idx, (STAY, YIELD))
        self.assertEqual(self.choose(0.30)[0], STAY)
        self.assertEqual(self.choose(0.50)[0], STAY)
        self.assertEqual(self.choose(0.60)[0], LEFT)

    def test_ordinary_confirmation_tracks_key_when_candidate_array_reorders(self):
        self.assertEqual(self.choose(0.0)[0], STAY)
        self.assertEqual(self.choose(0.20)[0], STAY)
        keys = [KEYS[YIELD], KEYS[RIGHT], KEYS[STAY], KEYS[LEFT]]
        idx, emergency = self.selector.choose(keys, (30.0, 20.0, 10.0, 0.0),
                                               (0.0, 0.0, 0.0, 0.0), 0, 0, 0.30)
        self.assertEqual(keys[idx], KEYS[LEFT])
        self.assertFalse(emergency)

    def test_escape_uses_only_current_finite_feasible_candidates(self):
        # A previously safe side is rejected as soon as its current result is
        # unavailable, non-finite, or violates any near-horizon constraint.
        for bad_score, bad_violation in ((-1e12, 0.01), (math.nan, 0.0),
                                         (-math.inf, 0.0), (-1e12, math.nan)):
            with self.subTest(score=bad_score, violation=bad_violation):
                self.selector = PlanSelector(confirm_time=0.25)
                self.choose(0.0, scores=(0.0, 10.0, 20.0, 30.0))
                idx, emergency = self.choose(0.30, scores=(-1e12, bad_score, 100.0, -1e12),
                                             violation=(2.0, bad_violation, 0.0, 1.0))
                self.assertEqual(idx, RIGHT)
                self.assertFalse(emergency)
        keys = [KEYS[STAY], KEYS[RIGHT], KEYS[YIELD]]
        idx, emergency = self.selector.choose(keys, (0.0, 100.0, 0.0), (2.0, 0.0, 1.0), 0, 0, 1.0)
        self.assertEqual(keys[idx], KEYS[RIGHT])
        self.assertFalse(emergency)

    def test_worst_start_abort_retry_sequence_obeys_two_second_log_window(self):
        # Use the production controller's cooldown, not a duplicated constant.
        # Drive the selector as control() does: update target on every returned
        # plan and set abort_until at every cancellation. Adversarially make an
        # admitted side unsafe on the very next 50 ms control frame.
        self.assertGreaterEqual(ABORT_COOLDOWN, 2.0)
        for offset in (0.0, 0.01, 0.025, 0.049, 0.05, 0.099, 0.15, 0.35):
            with self.subTest(offset=offset):
                self.selector = PlanSelector(confirm_time=0.25)
                target, abort_until = 0, -1.0
                events = []
                cancel_times = []
                for frame in range(241):
                    t = offset + frame * 0.05
                    violation = (0.0, 1.0, 0.0, 0.0) if target != 0 else (2.0, 0.0, 1.0, 1.0)
                    idx, _ = self.choose(t, target_idx=target, violation=violation,
                                         abort_until=abort_until)
                    chosen = KEYS[idx][1]
                    if chosen != target:
                        if chosen == 0:
                            abort_until = t + ABORT_COOLDOWN
                            cancel_times.append(t)
                        elif cancel_times:
                            self.assertGreaterEqual(t + 1e-9, cancel_times[-1] + ABORT_COOLDOWN)
                        events.append(t)
                    target = chosen
                self.assertGreaterEqual(len(events), 8, 'the adversarial retry cycle was not exercised')
                for timestamps in (events, [float('%.1f' % t) for t in events]):
                    burst = max(sum(0 <= u - x < 2.0 for u in timestamps) for x in timestamps)
                    self.assertLessEqual(burst, 2, 'too many decisions in a 2 s window: %r' % timestamps)


if __name__ == '__main__':
    unittest.main(verbosity=2)
