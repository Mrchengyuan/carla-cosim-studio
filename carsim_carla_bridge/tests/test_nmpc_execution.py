"""One-frame real-controller checks for solver failure and executed-input safety.

Run with one numerical-library thread per process:
    OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
        python tests/test_nmpc_execution.py

The real AL-iLQR solve runs before fault injection. No CARLA server, long
simulation, synthetic solver replacement, or production file changes are used.
"""
import contextlib
import importlib.util
import io
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CTL_DIR = os.path.abspath(os.path.join(HERE, '..', 'controllers', 'topo_nmpc'))
sys.path.insert(0, HERE)
sys.path.insert(0, CTL_DIR)
from topo_nmpc_sim import Actor, Road, Sim

spec = importlib.util.spec_from_file_location('nmpc_execution_controller', os.path.join(CTL_DIR, 'controller.py'))
C = importlib.util.module_from_spec(spec)
spec.loader.exec_module(C)
C.DRAW = False
C.PRINT_EVERY = 1e9


class ExecutionSafetyTests(unittest.TestCase):
    def make_controller(self, v0=20.0):
        sim = Sim(Road([(1500.0, 0.0)]), v0=v0)
        ctl = C.Controller()
        with contextlib.redirect_stdout(io.StringIO()):
            ctl.reset()
        return ctl, sim

    def control(self, ctl, sim, t=0.0):
        with contextlib.redirect_stdout(io.StringIO()), np.errstate(all='ignore'):
            return ctl.control(sim.plant.exports(), t, 0.05, sim.scene())

    @staticmethod
    def invalid_warm_keys(ctl):
        return [key for key, entry in ctl.warm.items()
                if any(not np.isfinite(np.asarray(value)).all() for value in entry.values())]

    def inject_nan_after_real_solve(self, ctl, candidate=None):
        original = ctl.solver.solve
        injected_keys = []

        def solve(*args, **kwargs):
            sol = original(*args, **kwargs)
            indices = range(len(sol['U'])) if candidate is None else [candidate]
            for h in indices:
                # The controller's first hypothesis is its current lane.
                if h == 0:
                    injected_keys.append(('lane', ctl.lane_idx))
                sol['U'][h] = np.nan
                sol['X'][h] = np.nan
            return sol

        ctl.solver.solve = solve
        return original, injected_keys

    def test_all_invalid_candidates_return_finite_braking_and_clear_bad_warm(self):
        ctl, sim = self.make_controller()
        self.inject_nan_after_real_solve(ctl)
        out = self.control(ctl, sim)
        with self.subTest(check='finite executed output'):
            self.assertTrue(np.isfinite(out).all(), 'non-finite actuator command: %r' % (out,))
        with self.subTest(check='immediate braking'):
            self.assertLess(out[0], 0.0, 'all solver plans failed but acceleration was not braking: %r' % (out,))
        with self.subTest(check='invalid warm starts discarded'):
            self.assertEqual(self.invalid_warm_keys(ctl), [], 'NaN solver state was retained for the next frame')

    def test_one_invalid_candidate_is_never_selected_or_kept_warm(self):
        ctl, sim = self.make_controller()
        _, injected_keys = self.inject_nan_after_real_solve(ctl, candidate=0)
        out = self.control(ctl, sim)
        bad = injected_keys[0]
        with self.subTest(check='invalid plan cannot win'):
            self.assertNotEqual(ctl.choice, bad)
        with self.subTest(check='remaining candidates give finite output'):
            self.assertTrue(np.isfinite(out).all(), 'non-finite actuator command: %r' % (out,))
        with self.subTest(check='invalid candidate has no warm entry'):
            self.assertNotIn(bad, list(ctl.warm), 'faulted candidate retained a warm start')
        with self.subTest(check='all retained warm starts are finite'):
            self.assertEqual(self.invalid_warm_keys(ctl), [])

    def test_next_frame_recovers_after_all_candidate_failure(self):
        ctl, sim = self.make_controller()
        original, _ = self.inject_nan_after_real_solve(ctl)
        self.control(ctl, sim)
        ctl.solver.solve = original
        # Keep the real finite plant state: a corrupt actuator command must not
        # be fed into the plant merely to manufacture a second failure.
        out = self.control(ctl, sim, t=0.05)
        self.assertTrue(np.isfinite(out).all(), 'normal solver cannot recover on the next frame: %r' % (out,))
        self.assertEqual(self.invalid_warm_keys(ctl), [])
        self.assertTrue(np.isfinite([ctl.ax_cmd, ctl.dc, ctl.ax_est]).all())

    def test_valid_warm_recovers_failed_or_violating_next_optimization(self):
        for failure in ('nan', 'finite_violation'):
            with self.subTest(failure=failure):
                ctl, sim = self.make_controller()
                first = self.control(ctl, sim)
                self.assertTrue(np.isfinite(first).all())
                self.assertIn(('lane', 0), ctl.warm)
                sim.plant.step(first[0], first[1], 0.05)
                sim.t = 0.05
                sim.ego_frenet()
                if failure == 'nan':
                    self.inject_nan_after_real_solve(ctl)
                else:
                    original_solve = ctl.solver.solve

                    def solve(*args, **kwargs):
                        sol = original_solve(*args, **kwargs)
                        # Finite but grossly violates input/steering constraints
                        # throughout the near horizon; first-input clipping alone
                        # cannot make this a usable optimized trajectory.
                        sol['U'][:, :, 0] = C.AX_MAX + 30.0
                        sol['U'][:, :, 1] = C.DELTA_RATE_MAX + 1.0
                        return sol

                    ctl.solver.solve = solve
                out = self.control(ctl, sim, t=0.05)
                self.assertGreater(ctl.debug.get('可行热启动回退', 0), 0,
                                   'safe warm trajectory was lost when optimization failed')
                self.assertTrue(np.isfinite(out).all())
                self.assertEqual(ctl.debug['紧急制动'], 0.0,
                                 'validated feasible warm plan should avoid an unnecessary emergency stop')
                self.assertEqual(ctl.choice, ('lane', 0))
                self.assertEqual(self.invalid_warm_keys(ctl), [])

    def test_changed_scene_rejects_previously_safe_warm_trajectory(self):
        ctl, sim = self.make_controller()
        first = self.control(ctl, sim)
        self.assertTrue(np.isfinite(first).all())
        self.assertIn(('lane', 0), ctl.warm)
        sim.plant.step(first[0], first[1], 0.05)
        sim.t = 0.05
        sim.ego_frenet()
        # A newly visible obstacle spans the available neighbouring corridors.
        # The old cruise/side-step controls cannot safely continue; old safety
        # evidence from the obstacle-free frame must not authorize reuse.
        sim.actors.append(Actor(sim.s_ego + 15.0, lane=0, v=0.0,
                                kind='static', length=4.8, width=20.0))
        self.assertEqual(len(sim.scene()['objects']), 1)
        self.inject_nan_after_real_solve(ctl)
        out = self.control(ctl, sim, t=0.05)
        self.assertEqual(ctl.debug.get('可行热启动回退'), 0.0,
                         'a stale safe trajectory was reused despite the new blocking obstacle')
        self.assertTrue(np.isfinite(out).all())
        self.assertLessEqual(out[0], -C.RSS_BRAKE)
        self.assertEqual(ctl.debug['紧急制动'], 1.0)
        self.assertEqual(ctl.target_idx, ctl.lane_idx)
        self.assertEqual(self.invalid_warm_keys(ctl), [])

    def test_low_speed_input_normalization_is_revalidated_with_matching_rollout(self):
        v0 = min(0.1, C.STANDSTILL_STEER_MS / 2.0)
        ctl, sim = self.make_controller(v0=v0)
        original_solve = ctl.solver.solve
        original_cost = ctl.solver.total_cost
        original_rollout = ctl.solver.rollout
        injected = False
        captured = {}

        def solve(*args, **kwargs):
            nonlocal injected
            sol = original_solve(*args, **kwargs)
            sol['U'][:, 0, 0] = C.AX_MIN - 30.0
            sol['U'][:, 0, 1] = C.DELTA_RATE_MAX + 1.0
            injected = True
            return sol

        def total_cost(X, U, *args, **kwargs):
            result = original_cost(X, U, *args, **kwargs)
            if injected:
                captured.update(X=X.copy(), U=U.copy(), constraints=result[3].copy())
            return result

        ctl.solver.solve = solve
        ctl.solver.total_cost = total_cost
        out = self.control(ctl, sim)
        self.assertIn('X', captured, 'executed-input plan was not checked after the solve')
        first = captured['U'][:, 0]
        self.assertTrue(np.allclose(first[:, 1], 0.0), 'stopped steering rate was still used in validation')
        expected_ax = max(C.AX_MIN, -C.HOLD_GAIN * v0)
        self.assertTrue(np.allclose(first[:, 0], expected_ax), 'validation missed clipped/tapered braking input')
        H = captured['U'].shape[0]
        expected_X = original_rollout(np.broadcast_to(ctl.x0, (H, C.NX)), captured['U'])
        self.assertTrue(np.allclose(captured['X'], expected_X, rtol=1e-10, atol=1e-10),
                        'constraint check used a trajectory from different actuator inputs')
        self.assertLessEqual(float(np.max(captured['constraints'][:, 0][:, [0, 1, 4, 5]])), 1e-9)
        self.assertTrue(np.isfinite(out).all())
        self.assertAlmostEqual(out[1], 0.0, places=12)


if __name__ == '__main__':
    unittest.main(verbosity=2)
