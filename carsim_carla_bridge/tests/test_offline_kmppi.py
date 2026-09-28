"""The KMPPI port (controllers/kmppi) without CARLA or Chrono: the lane
reference on a straight line and on a circle (position, heading, vy / r
from the original's steady-state bicycle relation), a closed loop on the
original's own 3DOF plant along a straight into a 250 m arc from 0.5 m off
the lane centre, the zero-order hold between KMPPI periods, the refusal of a
frame step that does not divide 0.05 s, and the CarSim service's --chrono
(no .sim checked, like --mock). About half a minute.

    python tests/test_offline_kmppi.py
"""
import importlib.util
import math
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
KDIR = os.path.join(HERE, "..", "controllers", "kmppi")
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, KDIR)

from kmppi_config import KMPPIConfig  # noqa: E402
from lane_reference import LaneReference  # noqa: E402
from prediction_model import BicycleModel, BicyclePlant  # noqa: E402

UNITS = {"angle": "deg", "speed": "km/h", "rate": "deg/s"}


def load_controller():
    spec = importlib.util.spec_from_file_location("kmppi_controller_file", os.path.join(KDIR, "controller.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    c = m.Controller()
    c.reset()
    return c


def road(ds=0.5, straight=100.0, radius=250.0, length=1500.0):
    """A straight, then a left arc; (N,2) points."""
    pts, x, y, h = [], 0.0, 0.0, 0.0
    for i in range(int(length / ds)):
        pts.append((x, y))
        h += (0.0 if i * ds < straight else 1.0 / radius) * ds
        x, y = x + ds * math.cos(h), y + ds * math.sin(h)
    return np.array(pts)


def lane_ahead(P, fx, fy, yaw):
    """center_rel as the scene gives it: from the point nearest the front axle, every 2 m, 50 m, ego frame."""
    i = int(np.argmin(np.hypot(P[:, 0] - fx, P[:, 1] - fy)))
    c, s = math.cos(yaw), math.sin(yaw)
    return [[(px - fx) * c + (py - fy) * s, -(px - fx) * s + (py - fy) * c] for px, py in P[i:i + 101:4]]


class LaneReferenceTests(unittest.TestCase):
    def setUp(self):
        self.cfg = KMPPIConfig(plant="chrono", vehicle_params="chrono_bmw_e90")
        self.ref = LaneReference(self.cfg, self.cfg.resolved_vehicle())

    def test_straight_lane_ahead_of_the_centre_of_gravity(self):
        a = self.cfg.resolved_vehicle().a
        pts = [[2.0 * i, 0.4] for i in range(26)]  # 0.4 m to the left, parallel
        h = self.ref.horizon(pts, (-a, 0.0))
        step = self.cfg.ref_speed * self.cfg.dt
        np.testing.assert_allclose(h[:, 0], -a + step * np.arange(1, self.cfg.T + 1), atol=1e-9)
        np.testing.assert_allclose(h[:, 1:3], [[0.4, 0.0]] * self.cfg.T, atol=1e-9)
        np.testing.assert_allclose(h[:, 3:], [[self.cfg.ref_speed, 0.0, 0.0]] * self.cfg.T, atol=1e-9)

    def test_circle_heading_and_yaw_rate(self):
        R = 100.0
        th = np.arange(0.0, 0.6, 2.0 / R)
        pts = np.stack([R * np.sin(th), R * (1 - np.cos(th))], axis=1)  # left circle from the origin, heading +x
        h = self.ref.horizon(pts.tolist(), (0.0, 0.0))
        s = self.cfg.ref_speed * self.cfg.dt * np.arange(1, self.cfg.T + 1)
        np.testing.assert_allclose(h[:, 2], s / R, atol=0.02)                     # heading along the arc
        np.testing.assert_allclose(np.hypot(h[:, 0], h[:, 1] - R), R, atol=0.05)  # on the circle
        np.testing.assert_allclose(h[5:-5, 5], self.cfg.ref_speed / R, rtol=0.02)   # r = v / R
        vy, r = self.ref.bicycle_reference(np.array([1.0 / R]), np.array([self.cfg.ref_speed]))
        np.testing.assert_allclose(h[5:-5, 4], vy[0], rtol=0.05)

    def test_too_few_points(self):
        with self.assertRaises(ValueError):
            self.ref.horizon([[0.0, 0.0]], (-1.0, 0.0))


class ClosedLoopTests(unittest.TestCase):
    def test_3dof_plant_straight_into_an_arc(self):
        """The original's own 3DOF plant (vehicle3dof_sfun.m) with the BMW's parameters."""
        cfg = KMPPIConfig(plant="3dof", vehicle_params="chrono_bmw_e90")
        veh = cfg.resolved_vehicle()
        plant = BicyclePlant(BicycleModel(veh, 0.05, cfg.ax_max, cfg.delta_max), np.array([0.0, 0.5, 0.0, 20.0, 0.0, 0.0]))
        P = road()
        c = load_controller()
        offs, speeds, t = [], [], 0.0
        for _ in range(int(12.0 / 0.05)):
            X, Y, yaw, vx, vy, r = plant.get_state()
            fx, fy = X + veh.a * math.cos(yaw), Y + veh.a * math.sin(yaw)
            rel = lane_ahead(P, fx, fy, yaw)
            offs.append(-rel[0][1])
            speeds.append(vx)
            exports = {"Vx": vx * 3.6, "Vy": (vy + veh.a * r) * 3.6, "AVz": math.degrees(r)}
            u = c.control(exports, t, 0.05, {"units": UNITS, "lane": {"center_rel": rel, "offset": offs[-1]}})
            self.assertEqual(len(u), 2)
            plant.step(u[0], u[1])
            t += 0.05
        late = np.abs(offs[60:])   # after 3 s: the 0.5 m start offset is gone, into the arc at 5 s
        self.assertLess(float(np.max(late)), 0.1, late.max())
        self.assertLess(abs(float(np.mean(speeds[60:])) - 20.0), 0.1)
        self.assertGreater(plant.get_state()[2], 0.1)  # it did turn into the arc


class TimingTests(unittest.TestCase):
    def scene(self):
        return {"units": UNITS, "lane": {"center_rel": [[2.0 * i, 0.0] for i in range(26)], "offset": 0.0}}

    def test_held_between_kmppi_periods(self):
        c = load_controller()
        e = {"Vx": 60.0, "Vy": 0.0, "AVz": 0.0}  # 16.7 m/s: KMPPI speeds up
        outs = [c.control(e, 0.01 * i, 0.01, self.scene()) for i in range(10)]
        self.assertEqual(c.n_calls, 2)             # frames 0 and 5
        self.assertTrue(all(o == outs[0] for o in outs[:5]) and all(o == outs[5] for o in outs[5:]))
        self.assertNotEqual(outs[0], outs[5])

    def test_a_frame_step_that_does_not_divide_the_period(self):
        c = load_controller()
        with self.assertRaises(ValueError) as cm:
            c.control({"Vx": 72.0, "Vy": 0.0, "AVz": 0.0}, 0.0, 0.02, self.scene())
        self.assertIn("仿真步长 0.020 s 不能整除", str(cm.exception))

    def test_no_lane_keeps_the_last_output(self):
        c = load_controller()
        e = {"Vx": 60.0, "Vy": 0.0, "AVz": 0.0}
        first = c.control(e, 0.0, 0.05, self.scene())
        self.assertEqual(c.control(e, 0.05, 0.05, {"units": UNITS, "lane": None}), first)


class ServiceTests(unittest.TestCase):
    def test_chrono_needs_no_sim(self):
        import carsim_service
        svc = carsim_service.CarSim(False, 20.0)
        self.assertEqual(svc.check({"sim_path": "", "repo_path": "", "mock": False, "export_names": [], "units": {}}, 5.0), {})


if __name__ == "__main__":
    unittest.main(verbosity=2)
