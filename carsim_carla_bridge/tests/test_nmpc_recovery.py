"""Measured abort recovery, using the production Controller with no simulation.

Apply patch_stopped_recovery.py before running this test. No CARLA or solver
iterations are started; only the real geometric recovery predicate is called.
"""
import importlib.util
import math
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CTL_DIR = os.path.abspath(os.path.join(HERE, '..', 'controllers', 'topo_nmpc'))
sys.path.insert(0, CTL_DIR)
spec = importlib.util.spec_from_file_location('nmpc_recovery_controller', os.path.join(CTL_DIR, 'controller.py'))
C = importlib.util.module_from_spec(spec)
spec.loader.exec_module(C)


class RecoveryPoseTests(unittest.TestCase):
    def setUp(self):
        # The predicate needs dimensions and measured speed; bypass reset so
        # these tests do not load a value network or construct solver state.
        self.ctl = C.Controller()
        self.ctl.veh = C.Vehicle()
        self.ctl.ego_len, self.ctl.ego_wid = C.EGO_LEN, C.EGO_WID
        self.ctl.vx_now = 0.0
        self.x = np.zeros(C.NX)
        self.width = 3.5

    def test_stopped_offcentre_but_wholly_inside_lane_can_recover(self):
        self.x[C.EY] = 0.4
        self.assertGreater(abs(self.x[C.EY]), C.RECOVER_EY)
        self.assertLess(abs(self.x[C.EY]) + self.ctl.ego_wid / 2 + C.LANE_MARGIN,
                        self.width / 2)
        self.assertTrue(self.ctl._recovery_settled(self.x, self.width))

    def test_same_offcentre_pose_while_moving_must_recentre(self):
        self.x[C.EY] = 0.4
        self.x[C.VX] = 2.0
        self.ctl.vx_now = 2.0
        self.assertFalse(self.ctl._recovery_settled(self.x, self.width))

    def test_stopped_body_crossing_lane_margin_cannot_recover(self):
        self.x[C.EY] = self.width / 2 - self.ctl.ego_wid / 2 - C.LANE_MARGIN + 0.05
        self.assertFalse(self.ctl._recovery_settled(self.x, self.width))

    def test_excess_heading_error_cannot_recover_even_while_stopped(self):
        self.x[C.EPSI] = C.RECOVER_EPSI + math.radians(0.5)
        self.assertFalse(self.ctl._recovery_settled(self.x, self.width))

    def test_measured_lateral_motion_cannot_recover_even_at_zero_longitudinal_speed(self):
        self.x[C.VY] = C.RECOVER_LATERAL_MS + 0.05
        self.assertFalse(self.ctl._recovery_settled(self.x, self.width))


if __name__ == '__main__':
    unittest.main(verbosity=2)
