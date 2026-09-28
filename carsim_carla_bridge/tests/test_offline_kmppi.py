"""The KMPPI port (controllers/kmppi) without CARLA or Chrono: the lane
reference on a straight line and on a circle (position, heading, vy / r
from the original's steady-state bicycle relation), a closed loop on the
original's own 3DOF plant along a straight into a 250 m arc from 0.5 m off
the lane centre, the zero-order hold between KMPPI periods, the refusal of a
frame step that does not divide 0.05 s, and the CarSim service's --chrono
(no .sim checked, like --mock). The platform's lines (self.draw): to CARLA's
world from the ego frame, format errors in plain words, the segment limit,
none while collecting; KMPPI's lines (64 candidates coloured by weight, the
reference, the weighted mean, the best one bold and last); finding the
Chrono Python. About half a minute.

    python tests/test_offline_kmppi.py
"""
import importlib.util
import math
import os
import sys
import unittest
import unittest.mock

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

    def test_no_lane_keeps_the_steering_and_the_speed(self):
        """No lane: the front wheel angle held, ax 0 (not speeding up off the road)."""
        c = load_controller()
        e = {"Vx": 60.0, "Vy": 0.0, "AVz": 0.0}
        first = c.control(e, 0.0, 0.05, self.scene())
        self.assertGreater(first[0], 0.0)  # 16.7 m/s: speeding up towards 20
        self.assertEqual(c.control(e, 0.05, 0.05, {"units": UNITS, "lane": None}), [0.0, first[1]])


class DrawTests(unittest.TestCase):
    def test_ego_frame_to_carla_world(self):
        import session
        # Reference point at (100, 50, 2) facing CARLA yaw 90 (+y): forward is +y, left (CarSim y+) is +x... in CARLA
        # (y right): left of a car facing +y is -x... checked point by point:
        segs, cut = session.draw_segments([{"points": [[0, 0], [10, 0], [10, 2]], "color": [1, 2, 3], "width": 0.2}],
                                          (100.0, 50.0, 2.0, 90.0))
        self.assertEqual((len(segs), cut), (2, 0))
        (a, b, w, c), (b2, e, _, _) = segs
        z = 2.0 + session.DRAW_LIFT_M
        for got, want in ((a, (100, 50, z)), (b, (100, 60, z)), (e, (102, 60, z))):
            np.testing.assert_allclose((got.x, got.y, got.z), want, atol=1e-9)
        self.assertEqual((w, c.r, c.g, c.b), (0.2, 1, 2, 3))

    def test_left_is_left(self):
        """CarSim y left -> CARLA y right: facing +x (yaw 0), a point to the left has a smaller CARLA y."""
        import session
        (seg,), _ = session.draw_segments([{"points": [[0, 0], [0, 1]]}], (0.0, 0.0, 0.0, 0.0))
        self.assertAlmostEqual(seg[1].y, -1.0)
        self.assertEqual((seg[2], seg[3].r, seg[3].g, seg[3].b), (0.05, 255, 255, 255))  # defaults

    def test_format_errors_and_the_limit(self):
        import session
        with self.assertRaisesRegex(RuntimeError, "self.draw 应是一组线"):
            session.draw_segments({"points": []}, (0, 0, 0, 0))
        with self.assertRaisesRegex(RuntimeError, "第 2 条线格式不对"):
            session.draw_segments([{"points": [[0, 0], [1, 1]]}, {"pts": []}], (0, 0, 0, 0))
        segs, cut = session.draw_segments([{"points": [[i, 0] for i in range(6001)]}], (0, 0, 0, 0))
        self.assertEqual((len(segs), cut), (session.MAX_DRAW_SEGMENTS, 6000))

    def test_taken_once_and_not_while_collecting(self):
        import session

        class Ses:
            _take_draw = session.CoSimSession._take_draw

        s = Ses()
        s.driver = type("D", (), {"draw": [{"points": [[0, 0], [1, 0]]}]})()
        s.scene = type("S", (), {"ref_pose": (0.0, 0.0, 0.0, 0.0)})()
        s.d = {"collect": {"enabled": False}}
        self.assertEqual(s._take_draw(), [])
        self.assertEqual((len(s._draw_segs), s.driver.draw), (1, None))  # taken: the next frames draw the same
        self.assertEqual(s._take_draw(), [])
        self.assertEqual(len(s._draw_segs), 1)
        s.d = {"collect": {"enabled": True}}
        s.driver.draw = [{"points": [[0, 0], [1, 0]]}]
        warn = s._take_draw()
        self.assertEqual(s._draw_segs, [])
        self.assertIn("采集数据时不画", warn[0])
        s.driver.draw = [{"points": [[0, 0], [1, 0]]}]
        self.assertEqual(s._take_draw(), [])  # said once

    def test_kmppi_lines(self):
        c = load_controller()
        scene = {"units": UNITS, "lane": {"center_rel": [[2.0 * i, 0.0] for i in range(26)], "offset": 0.0}}
        c.control({"Vx": 72.0, "Vy": 0.0, "AVz": 0.0}, 0.0, 0.05, scene)
        lines = c.draw
        self.assertEqual(len(lines), 64 + 64 + 3)
        cand, dots, ref, mean, best = lines[:64], lines[64:128], lines[128], lines[129], lines[130]
        # A dot at each candidate's end, in its colour.
        self.assertTrue(all(len(d["points"]) == 1 for d in dots))
        self.assertEqual([d["points"][0] for d in dots], [l["points"][-1] for l in cand])
        self.assertEqual([d["color"] for d in dots], [l["color"] for l in cand])
        self.assertEqual((ref["color"], mean["color"], best["color"]), ([40, 230, 90], [255, 215, 0], [255, 30, 30]))
        self.assertGreater(best["width"], mean["width"])
        self.assertGreater(mean["width"], cand[0]["width"])
        self.assertEqual(len(best["points"]), 12)  # 34 states, every 3rd
        self.assertEqual(best["points"][0], [-c.vehicle.a, 0.0])  # from the centre of gravity
        self.assertTrue(all(len(l["points"]) == 12 for l in cand))
        # The bold one is the highest weight: the same as the reddest candidate (drawn last among them).
        self.assertEqual(best["points"], cand[-1]["points"])
        self.assertEqual(cand[-1]["color"], [255, 60, 40])
        self.assertEqual(cand[0]["width"], 0.015)

    def test_a_single_point_is_a_dot(self):
        import session
        segs, _ = session.draw_segments([{"points": [[1, 0]], "width": 0.2}, {"points": []}], (0.0, 0.0, 0.0, 0.0))
        self.assertEqual(len(segs), 1)
        a, b, w, c = segs[0]
        self.assertEqual((round(a.x, 9), b, w), (1.0, None, 0.2))

    def test_find_chrono_python(self):
        import chrono_local
        with self.assertRaisesRegex(ValueError, "找不到装了 PyChrono 的 Python"):
            with unittest.mock.patch.dict(os.environ, {"CHRONO_PYTHON": ""}), \
                    unittest.mock.patch("os.path.isfile", return_value=False):
                chrono_local.find_python("")
        self.assertEqual(chrono_local.find_python(sys.executable), sys.executable)


class FrameDtTests(unittest.TestCase):
    """FRAME_DT in an algorithm file: the run's frame step (read with ast, never run)."""

    def setUp(self):
        import tempfile
        self.tmp = tempfile.mkdtemp(prefix="cc_framedt_")
        self.addCleanup(__import__("shutil").rmtree, self.tmp, True)

    def cfg(self, text, dt=0.02, dynamics="cosim"):
        import settings
        path = os.path.join(self.tmp, "algo.py")
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
        d = settings.default_dict()
        d["run"].update(driver="custom", controller={"path": path, "entry": "Controller"})
        d["drive"]["dynamics"] = dynamics
        d["sync"]["frame_dt"] = dt
        return d

    def test_the_file_sets_the_step_and_says_so(self):
        import session
        d = self.cfg("import os\nFRAME_DT = 0.05\nopen('never_run', 'w')\n")
        notes = session.check_run_config(d)
        self.assertEqual(d["sync"]["frame_dt"], 0.05)
        self.assertEqual(notes, ["仿真步长按控制算法 algo.py 里的 FRAME_DT 用 0.05 s（界面上设的是 0.02 s）"])
        self.assertFalse(os.path.exists("never_run"))
        d = self.cfg("FRAME_DT = 0.05\n", dt=0.05)
        self.assertEqual(session.check_run_config(d), [])  # the same: nothing to say

    def test_without_it_the_page_setting(self):
        import session
        for text in ("x = 1\n", "def f():\n    FRAME_DT = 0.05\n", "class C:\n    FRAME_DT = 0.05\n"):
            d = self.cfg(text)
            self.assertEqual((session.check_run_config(d), d["sync"]["frame_dt"]), ([], 0.02), text)
        d = self.cfg("FRAME_DT = 0.05\n", dynamics="carla")  # CARLA dynamics: no algorithm runs
        session.check_run_config(d)
        self.assertEqual(d["sync"]["frame_dt"], 0.02)

    def test_plain_words_for_a_bad_value(self):
        import session
        with self.assertRaisesRegex(ValueError, "第 2 行的 FRAME_DT 要直接写成数字"):
            session.check_run_config(self.cfg("DT = 0.05\nFRAME_DT = DT\n"))
        with self.assertRaisesRegex(ValueError, "FRAME_DT = 0.5 s 超出范围"):
            session.check_run_config(self.cfg("FRAME_DT = 0.5\n"))

    def test_kmppi_asks_for_its_period(self):
        import session
        self.assertEqual(session.algorithm_frame_dt(os.path.join(KDIR, "controller.py")), 0.05)


class CarSimOutputTests(unittest.TestCase):
    """OUTPUT = "carsim": [油门, 制动 MPa, 方向盘转角 deg] from KMPPI's [ax, delta] (the adapter, not the algorithm)."""

    def ctrl(self):
        spec = importlib.util.spec_from_file_location("kmppi_carsim_out", os.path.join(KDIR, "controller.py"))
        m = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(m)
        m.OUTPUT = "carsim"
        c = m.Controller()
        c.reset()
        return m, c

    def test_steering_wheel_and_the_learned_ratio(self):
        m, c = self.ctrl()
        c.action = [0.0, math.radians(1.0)]
        e = {"Vx": 72.0, "Steer_SW": 0.0, "Steer_L1": 0.0, "Steer_R1": 0.0}
        self.assertAlmostEqual(c._output(e, 0.05, 1.0, 1.0)[2], m.STEER_RATIO_INIT)   # 1 deg x 19
        e = {"Vx": 72.0, "Steer_SW": 32.0, "Steer_L1": 2.1, "Steer_R1": 1.9}         # the car's ratio is 16
        for _ in range(400):
            out = c._output(e, 0.05, 1.0, 1.0)
        self.assertAlmostEqual(c.ratio, 16.0, delta=0.05)
        self.assertAlmostEqual(out[2], 16.0, delta=0.05)

    def test_throttle_and_brake_from_ax(self):
        m, c = self.ctrl()
        e = {"Vx": 72.0}
        c.action = [2.0, 0.0]
        thr, brk, _ = c._output(e, 0.05, 1.0, 1.0)
        self.assertTrue(thr > 0 and brk == 0, (thr, brk))
        c.reset()
        c.action = [-4.0, 0.0]
        for _ in range(20):  # a second of hard braking at a constant speed: the target drops away
            thr, brk, _ = c._output(e, 0.05, 1.0, 1.0)
        self.assertEqual(thr, 0.0)
        self.assertGreater(brk, 0.3 * m.BRAKE_MAX)
        self.assertLessEqual(brk, m.BRAKE_MAX)
        self.assertLessEqual(72.0 / 3.6 - c.v_target, m.V_TARGET_SLACK + 1e-9)  # never far below the car

    def test_ax_delta_is_passed_on_untouched(self):
        m, c = self.ctrl()
        m.OUTPUT = "ax_delta"
        c.action = [1.5, -0.02]
        self.assertEqual(c._output({"Vx": 72.0}, 0.05, 1.0, 1.0), [1.5, -0.02])


class ServiceTests(unittest.TestCase):
    def test_chrono_needs_no_sim(self):
        import carsim_service
        svc = carsim_service.CarSim(False, 20.0)
        self.assertEqual(svc.check({"sim_path": "", "repo_path": "", "mock": False, "export_names": [], "units": {}}, 5.0), {})


if __name__ == "__main__":
    unittest.main(verbosity=2)
