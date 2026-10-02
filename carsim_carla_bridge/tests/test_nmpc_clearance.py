"""Regression checks for the real NMPC rectangle-clearance constraints.

Run directly with OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
    python tests/test_nmpc_clearance.py

These are constraint evaluations, not simulations. They keep the existing
ellipse and following constraints active so a rejection must be attributable
to the additional physical clearance envelope.
"""
import importlib.util
import math
import os
import sys
from types import SimpleNamespace
import unittest

import numpy as np

CTL_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'controllers', 'topo_nmpc'))
sys.path.insert(0, CTL_DIR)
from nmpc_reference import Obstacle

spec = importlib.util.spec_from_file_location('nmpc_clearance_controller', os.path.join(CTL_DIR, 'controller.py'))
C = importlib.util.module_from_spec(spec)
spec.loader.exec_module(C)


class RectangleClearanceTests(unittest.TestCase):
    EGO_LENGTH, EGO_WIDTH = 4.6, 1.85
    OTHER_LENGTH, OTHER_WIDTH = 4.8, 1.9
    SPEED = 15.0

    def make_problem(self, other_d, kind="vehicle"):
        x0 = np.zeros(C.NX)
        x0[C.VX] = self.SPEED
        ctl = SimpleNamespace(veh=C.Vehicle(), ego_len=self.EGO_LENGTH,
                              ego_wid=self.EGO_WIDTH, x0=x0)
        path = SimpleNamespace(kappa=lambda s: np.zeros_like(s))
        # Wide road boundaries isolate obstacle clearance from lane limits.
        hyps = [dict(target=0.0, left=10.0, right=-10.0, vscale=1.0)]
        other_speed = self.SPEED if kind == "vehicle" else 0.0
        other = Obstacle(s=0.0, d=other_d, vs=other_speed, vd=0.0,
                         half_s=self.OTHER_LENGTH / 2, half_d=self.OTHER_WIDTH / 2,
                         kind=kind, id=1, speed=other_speed)
        problem = C._Problem(ctl, path, hyps, np.array([0.0, 100.0]),
                             np.array([self.SPEED, self.SPEED]), [other], None)
        return problem, x0

    def evaluate(self, problem, x, terminal=False):
        state = x[None, :].copy()
        if terminal:
            # Evaluate the same relative geometry at the horizon, accounting
            # for a moving vehicle or a stationary obstacle.
            state[:, C.S] += problem.os[C.N_STEPS, 0] - problem.os[0, 0]
            all_constraints = problem.con_T(state, np.array([0]))[0]
            self.assertEqual(len(all_constraints), problem.nc - 6)
            base = 4  # state constraints before the obstacle ellipses
        else:
            all_constraints = problem.con(state, np.zeros((1, C.NU)),
                                          np.array([0]), np.array([0]))[0]
            self.assertEqual(len(all_constraints), problem.nc)
            base = 6 + 4  # input and state constraints before obstacle ellipses
        self.assertTrue(np.isfinite(all_constraints).all())
        ellipse_end = base + C.N_CIRCLES * C.MAX_OBS
        rectangle_end = ellipse_end + C.MAX_OBS
        ellipses = all_constraints[base:ellipse_end].reshape(C.N_CIRCLES, C.MAX_OBS)[:, 0]
        rectangle = all_constraints[ellipse_end:rectangle_end][0]
        previous_constraints = np.concatenate((all_constraints[:ellipse_end],
                                               all_constraints[rectangle_end:]))
        return all_constraints, ellipses, rectangle, previous_constraints

    def test_half_metre_side_gap_is_rejected_despite_clear_ellipses(self):
        # Actual body gap is 0.5 m, with longitudinally overlapping vehicles.
        centre_spacing = (self.EGO_WIDTH + self.OTHER_WIDTH) / 2 + 0.5
        for side in (-1, 1):
            problem, x = self.make_problem(side * centre_spacing)
            for terminal in (False, True):
                with self.subTest(side=side, terminal=terminal):
                    constraints, ellipses, rectangle, previous = self.evaluate(problem, x, terminal)
                    self.assertLessEqual(float(np.max(ellipses)), 0.0)
                    self.assertLessEqual(float(np.max(previous)), C.VIOL_TOL)
                    self.assertGreater(rectangle, C.VIOL_TOL)
                    self.assertGreater(float(np.max(constraints)), C.VIOL_TOL)

    def test_parallel_adjacent_lane_centres_remain_feasible(self):
        # Standard 3.5 m lanes leave 1.625 m between these straight car bodies.
        for side in (-1, 1):
            problem, x = self.make_problem(side * 3.5)
            for terminal in (False, True):
                with self.subTest(side=side, terminal=terminal):
                    constraints, _, rectangle, _ = self.evaluate(problem, x, terminal)
                    self.assertLess(rectangle, 0.0)
                    self.assertLessEqual(float(np.max(constraints)), C.VIOL_TOL)

    def test_static_mask_preserves_original_collision_protection(self):
        # Static obstacles keep their original ellipses and following gap;
        # disabling the new vehicle envelope must not disable those checks.
        for other_d, overlapping in ((0.0, True), (3.5, False)):
            problem, x = self.make_problem(other_d, kind="static")
            for terminal in (False, True):
                with self.subTest(overlapping=overlapping, terminal=terminal):
                    constraints, ellipses, rectangle, _ = self.evaluate(problem, x, terminal)
                    self.assertLess(rectangle, 0.0)
                    if overlapping:
                        self.assertGreater(float(np.max(ellipses)), C.VIOL_TOL)
                        self.assertGreater(float(np.max(constraints)), C.VIOL_TOL)
                    else:
                        self.assertLessEqual(float(np.max(ellipses)), 0.0)
                        self.assertLessEqual(float(np.max(constraints)), C.VIOL_TOL)

    def test_ego_heading_expands_body_envelope_before_ellipses_touch(self):
        for side in (-1, 1):
            problem, x = self.make_problem(side * 3.5)
            straight, _, _, _ = self.evaluate(problem, x)
            self.assertLessEqual(float(np.max(straight)), C.VIOL_TOL)
            x[C.EPSI] = side * 0.3
            # Independently transform the physical four corners. Their road-
            # aligned box now leaves less than 1 m, even though lane centres
            # are unchanged and the original ellipses remain collision-free.
            corners = np.array([(s, d) for s in (-self.EGO_LENGTH / 2, self.EGO_LENGTH / 2)
                                for d in (-self.EGO_WIDTH / 2, self.EGO_WIDTH / 2)])
            c, s = math.cos(x[C.EPSI]), math.sin(x[C.EPSI])
            corners = corners @ np.array([[c, s], [-s, c]])
            closest_ego_side = float(np.max(side * corners[:, 1]))
            physical_gap = 3.5 - self.OTHER_WIDTH / 2 - closest_ego_side
            self.assertGreater(physical_gap, 0.0)
            self.assertLess(physical_gap, 1.0)
            for terminal in (False, True):
                with self.subTest(side=side, terminal=terminal):
                    constraints, ellipses, rectangle, previous = self.evaluate(problem, x, terminal)
                    self.assertLessEqual(float(np.max(ellipses)), 0.0)
                    self.assertLessEqual(float(np.max(previous)), C.VIOL_TOL)
                    self.assertGreater(rectangle, C.VIOL_TOL)
                    self.assertGreater(float(np.max(constraints)), C.VIOL_TOL)


if __name__ == '__main__':
    unittest.main(verbosity=2)
