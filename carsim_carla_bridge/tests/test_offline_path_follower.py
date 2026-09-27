"""The lane path follower (controllers/path_follower.py) without CARLA: signs,
units, the learned steering ratio, and closed loops with the mock CarSim on
synthetic roads: straight, a 40 m radius left curve, straight; tight turns (an 11 m
junction turn, an 8 m hairpin); a junction whose lane reading jumps to a turning
lane, flickers, or is missing for a moment. Besides the mock, a dynamic bicycle
with tyre slip and a slow steering (DynamicBicycle): pure pursuit with about 1 s
of look-ahead must stay stable there too."""

import contextlib
import importlib.util
import io
import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

import settings as st  # noqa: E402
from mock_carsim import MockCarSimEnv  # noqa: E402


def load_follower():
    spec = importlib.util.spec_from_file_location("path_follower", os.path.join(HERE, "..", "controllers", "path_follower.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


PF = load_follower()
UNITS = {"angle": "deg", "speed": "km/h", "rate": "deg/s", "wheel_spin": "rpm", "jounce": "mm"}


def straight(offset, n=26):
    """Lane centre ahead of a car `offset` m left of it (ego frame, 2 m apart)."""
    return [[2.0 * i, -offset] for i in range(n)]


def left_curve(radius, n=26):
    return [[radius * math.sin(2.0 * i / radius), radius * (1 - math.cos(2.0 * i / radius))] for i in range(n)]


def scene(pts, offset=0.0, units=UNITS, speed=40.0):
    return {"units": units, "ego": {"Speed": speed},
            "lane": {"offset": offset, "heading_err": 0.0, "width": 3.5, "center_rel": pts}}


def fresh():
    c = PF.Controller()
    c.reset()
    return c


class SignTests(unittest.TestCase):
    def test_left_of_centre_steers_right(self):
        self.assertLess(fresh().control({"Vx": 40.0}, 1.0, 0.02, scene(straight(0.5), 0.5))[2], 0.0)

    def test_right_of_centre_steers_left(self):
        self.assertGreater(fresh().control({"Vx": 40.0}, 1.0, 0.02, scene(straight(-0.5), -0.5))[2], 0.0)

    def test_left_curve_steers_left_on_the_centre(self):
        self.assertGreater(fresh().control({"Vx": 40.0}, 1.0, 0.02, scene(left_curve(40.0)))[2], 0.0)

    def test_right_curve_steers_right(self):
        pts = [[x, -y] for x, y in left_curve(40.0)]
        self.assertLess(fresh().control({"Vx": 40.0}, 1.0, 0.02, scene(pts))[2], 0.0)

    def test_on_the_centre_of_a_straight_lane_steers_straight(self):
        self.assertAlmostEqual(fresh().control({"Vx": 40.0}, 1.0, 0.02, scene(straight(0.0)))[2], 0.0, places=6)


class UnitTests(unittest.TestCase):
    def test_si_units_give_the_same_commands(self):
        deg = fresh().control({"Vx": 36.0, "Steer_SW": 0.0}, 1.0, 0.02, scene(left_curve(60.0)))
        si = dict(UNITS, angle="rad", speed="m/s")
        rad = fresh().control({"Vx": 10.0, "Steer_SW": 0.0}, 1.0, 0.02, scene(left_curve(60.0), units=si))
        for a, b in zip(deg, rad):
            self.assertAlmostEqual(a, b, places=9)  # the steering wheel stays in deg (python_carsim_env)

    def test_speed_from_the_scene_without_a_vx_export(self):
        out = fresh().control({}, 1.0, 0.02, scene(straight(0.0), speed=20.0))
        self.assertGreater(out[0], 0.0)  # 20 km/h below the target: throttle


class RatioTests(unittest.TestCase):
    def test_learns_the_carsim_steering_ratio(self):
        c = fresh()
        for _ in range(400):
            c.control({"Vx": 40.0, "Steer_SW": 200.0, "Steer_L1": 10.5, "Steer_R1": 9.5}, 1.0, 0.02, scene(straight(0.0)))
        self.assertAlmostEqual(c.ratio, 20.0, delta=0.2)

    def test_ignores_readings_near_zero_or_of_opposite_sign(self):
        c = fresh()
        for ex in ({"Steer_SW": 3.0, "Steer_L1": 0.1, "Steer_R1": 0.1},     # too small to trust
                   {"Steer_SW": -50.0, "Steer_L1": 3.0, "Steer_R1": 3.0}):  # inconsistent
            c.control(dict(ex, Vx=40.0), 1.0, 0.02, scene(straight(0.0)))
        self.assertEqual(c.ratio, PF.STEER_RATIO_INIT)


class SpeedTests(unittest.TestCase):
    def test_slows_down_before_a_tight_curve(self):
        c = fresh()
        self.assertLess(c._curve_speed(left_curve(20.0), 11.0), PF.TARGET_KMH)
        self.assertAlmostEqual(c._curve_speed(left_curve(20.0), 11.0), math.sqrt(PF.A_LAT_MAX * 20.0) * 3.6, delta=1.0)
        self.assertEqual(c._curve_speed(straight(0.0), 11.0), PF.TARGET_KMH)

    def test_brakes_when_too_fast(self):
        thr, brk, _ = fresh().control({"Vx": 70.0}, 1.0, 0.02, scene(straight(0.0)))
        self.assertEqual(thr, 0.0)
        self.assertGreater(brk, 0.0)


class NoLaneTests(unittest.TestCase):
    def test_off_the_lane_slows_and_centres_the_wheel(self):
        c = fresh()
        c.control({"Vx": 40.0}, 1.0, 0.02, scene(left_curve(40.0)))
        sw0 = c.sw_last
        out = c.control({"Vx": 40.0}, 1.02, 0.02, {"units": UNITS, "ego": {"Speed": 40.0}, "lane": None})
        self.assertTrue(all(math.isfinite(v) for v in out))
        self.assertLess(abs(out[2]), abs(sw0))
        self.assertGreater(out[1], 0.0)  # 40 km/h, target 10: brakes


# ------------------------------------------------------------------ closed loop
class Road:
    """A lane centre line as points 0.25 m apart (x, y, heading), read like the
    platform does (scene.py with CARLA's waypoints): the nearest point on the
    line (continuous), the offset (left +), 50 m ahead every 2 m in the ego frame."""

    STEP = 0.25

    def __init__(self, pts):
        self.pts = pts
        self.hint = 0

    @classmethod
    def build(cls, segs, shift=0.0, x0=-20.0):
        """segs: ("S", length) straight, ("C", radius (left +), degrees) arc; starts at (x0, 0)
        heading +x; shift: the whole line moved left (+) / right (-)."""
        x, y, h, pts = x0, 0.0, 0.0, []
        for seg in segs:
            n = int(seg[1] / cls.STEP) if seg[0] == "S" else int(abs(seg[1]) * math.radians(seg[2]) / cls.STEP)
            for _ in range(n):
                pts.append((x - shift * math.sin(h), y + shift * math.cos(h), h))
                x += cls.STEP * math.cos(h)
                y += cls.STEP * math.sin(h)
                if seg[0] == "C":
                    h += cls.STEP / seg[1]
        pts.append((x - shift * math.sin(h), y + shift * math.cos(h), h))
        return cls(pts)

    @classmethod
    def with_curve(cls):
        """Straight 170 m, a 40 m radius left curve over 90 deg, straight 560 m north;
        0.8 m right of the start, so the car starts 0.8 m left of the centre."""
        road = cls.build([("S", 170), ("C", 40.0, 90), ("S", 560)], shift=-0.8)
        road.r = 40.0
        return road

    @classmethod
    def straight(cls):
        return cls.build([("S", 700)])

    @classmethod
    def right_turn(cls, x0, r=12.0):
        """A junction's right-turn lane leaving the straight road at x0, then 60 m south."""
        return cls.build([("C", -r, 90), ("S", 60)], x0=x0)

    def _at(self, s):
        j = max(0, min(len(self.pts) - 2, int(s)))
        k = s - j
        (ax, ay, ah), (bx, by, bh) = self.pts[j], self.pts[j + 1]
        return ax + k * (bx - ax), ay + k * (by - ay), ah + k * (bh - ah)

    def _project(self, X, Y):
        lo = max(0, self.hint - 200)
        i = min(range(lo, min(len(self.pts), self.hint + 400)),
                key=lambda k: (self.pts[k][0] - X) ** 2 + (self.pts[k][1] - Y) ** 2)
        if i == lo and lo > 0:  # far from the last position: search everything
            i = min(range(len(self.pts)), key=lambda k: (self.pts[k][0] - X) ** 2 + (self.pts[k][1] - Y) ** 2)
        self.hint = i
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(self.pts) - 1:
                (ax, ay, _), (bx, by, _) = self.pts[j], self.pts[j + 1]
                dx, dy = bx - ax, by - ay
                k = max(0.0, min(1.0, ((X - ax) * dx + (Y - ay) * dy) / (dx * dx + dy * dy)))
                d = (ax + k * dx - X) ** 2 + (ay + k * dy - Y) ** 2
                if best is None or d < best[0]:
                    best = (d, j + k)
        return best[1]

    def offset(self, X, Y):
        px, py, h = self._at(self._project(X, Y))
        return (X - px) * -math.sin(h) + (Y - py) * math.cos(h), h

    def lane(self, X, Y, yaw_deg):
        s = self._project(X, Y)
        px, py, h = self._at(s)
        yaw = math.radians(yaw_deg)
        ahead = []
        for k in range(26):
            if s + k * 2.0 / self.STEP > len(self.pts) - 1:
                break
            gx, gy, _ = self._at(s + k * 2.0 / self.STEP)
            dx, dy = gx - X, gy - Y
            ahead.append([dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw)])
        err = math.degrees((yaw - h + math.pi) % (2 * math.pi) - math.pi)
        offset = (X - px) * -math.sin(h) + (Y - py) * math.cos(h)  # left of the centre: +
        return {"offset": offset, "heading_err": err, "width": 3.5, "center_rel": ahead}, h


class DynamicBicycle:
    """A vehicle closer to CarSim than the mock: linear tyres that saturate (sideslip,
    understeer), the road-wheel angle behind the command by a delay and a first-order
    lag (steering system, co-simulation step). Reports the front axle like CarSim.
    tau = 0.25 s + delay = 0.1 s is far more than a CarSim steering system has."""

    m, Iz, a, b, Cf, Cr, RATIO = 1650.0, 2700.0, 1.25, 1.65, 130000.0, 150000.0, 16.0

    def __init__(self, names, tau=0.15, delay=0.05):
        self.names, self.tau, self.n_delay = names, tau, int(round(delay / 0.001))

    def reset(self):
        self.X, self.Y, self.psi = -self.a, 0.0, 0.0  # centre of gravity: the front axle at 0
        self.vx = self.vy = self.r = self.delta = self.sw = 0.0
        self.queue = []
        return self._exports()

    def control_step(self, action, inner):
        throttle, brake, self.sw = action
        dt = 0.001
        for _ in range(inner):
            self.queue.append(math.radians(self.sw / self.RATIO))
            cmd = self.queue.pop(0) if len(self.queue) > self.n_delay else 0.0
            self.delta += (cmd - self.delta) * dt / self.tau
            acc = 3.0 * throttle - 8.0 * brake - 0.02 * self.vx
            self.vx = max(0.0, self.vx + (0.0 if self.vx <= 0.0 and acc < 0.0 else acc) * dt)
            if self.vx < 1.0:  # walking pace: kinematic
                self.r = self.vx * math.tan(self.delta) / (self.a + self.b)
                self.vy = self.r * self.b
            else:
                lim = 0.9 * self.m * 9.81 / 2  # mu 0.9
                ff = max(-lim, min(lim, -self.Cf * ((self.vy + self.a * self.r) / self.vx - self.delta)))
                fr = max(-lim, min(lim, -self.Cr * (self.vy - self.b * self.r) / self.vx))
                self.vy += ((ff * math.cos(self.delta) + fr) / self.m - self.vx * self.r) * dt
                self.r += (self.a * ff * math.cos(self.delta) - self.b * fr) / self.Iz * dt
            c, s = math.cos(self.psi), math.sin(self.psi)
            self.X += (self.vx * c - self.vy * s) * dt
            self.Y += (self.vx * s + self.vy * c) * dt
            self.psi += self.r * dt
        return self._exports(), 0.0, False, {}

    def _exports(self):
        c, s, d = math.cos(self.psi), math.sin(self.psi), math.degrees(self.delta)
        v = {"Xo": self.X + self.a * c, "Yo": self.Y + self.a * s, "Yaw": math.degrees(self.psi), "Vx": self.vx * 3.6,
             "Vy": (self.vy + self.a * self.r) * 3.6, "AVz": math.degrees(self.r), "Steer_SW": self.sw,
             "Steer_L1": d, "Steer_R1": d}
        return tuple(float(v.get(n, 0.0)) for n in self.names)


NAMES = st.default_dict()["carsim"]["export_names"]


def mock():
    return MockCarSimEnv(NAMES, t_step=0.001, t_stop=1e9, units=UNITS)


def drive(read_lane, seconds, pose=True, printed=None, plant=None, target_kmh=None, road=None):
    """Closed loop: read_lane(X, Y, yaw_deg) -> (lane or None, road heading); plant: the
    mock CarSim unless given. pose: hand the algorithm the ego's X / Y / Yaw (the
    platform's default selection). road: also measure the rear axle's offset from it.
    Returns [(t, offset, Vx, heading, X, Y, brake, rear axle offset)]."""
    env = plant or mock()
    obs = env.reset()
    c = fresh()
    saved = PF.TARGET_KMH
    if target_kmh:
        PF.TARGET_KMH = target_kmh
    dt, t, log = 0.02, 0.0, []
    try:
        for _ in range(int(seconds / dt)):
            ex = dict(zip(NAMES, obs))  # Xo / Yo: the front axle, like CarSim (the scene's origin)
            X, Y, yaw = ex["Xo"], ex["Yo"], math.radians(ex["Yaw"])
            lane, h = read_lane(X, Y, ex["Yaw"])
            ego = {"Speed": ex["Vx"]}
            if pose:
                ego.update(X=X, Y=Y, Yaw=ex["Yaw"])
            else:
                ex = {k: v for k, v in ex.items() if k not in ("Xo", "Yo")}
            with contextlib.redirect_stdout(io.StringIO()) as out:
                action = c.control(ex, t, dt, {"units": UNITS, "ego": ego, "lane": lane})
            if printed is not None:
                printed.append(out.getvalue())
            rear = road.offset(X - PF.WHEELBASE * math.cos(yaw), Y - PF.WHEELBASE * math.sin(yaw))[0] if road else None
            log.append((t, lane["offset"] if lane else None, ex["Vx"], h, X, Y, action[1], rear))
            obs, _, done, _ = env.control_step(action, 20)
            t += dt
            assert not done
    finally:
        PF.TARGET_KMH = saved
    return log


class ClosedLoopTests(unittest.TestCase):
    def test_follows_the_road_with_the_mock_carsim(self):
        road = Road.with_curve()
        log = drive(road.lane, 40.0)
        north = road.pts[-1][2]  # the heading of the last straight (about 90 deg)
        # Pure pursuit aims ahead: it turns in a little before a curve (and out after one).
        straight = [abs(o) for tt, o, v, h, x, y, b, r in log if tt > 8.0 and h == 0.0 and x < 120.0]
        settled = [abs(o) for tt, o, v, h, x, y, b, r in log if tt > 8.0]
        curve = [abs(o) for tt, o, v, h, x, y, b, r in log if 0.0 < h < north - 1e-9]
        final = [abs(o) for tt, o, v, h, x, y, b, r in log if h > north - 1e-9 and y > 70.0]
        curve_speed = [v for tt, o, v, h, x, y, b, r in log if 0.3 < h < north - 0.3]
        print("closed loop: straight max %.3f m, anywhere after 8 s max %.3f m, curve max %.3f m, final max %.3f m, "
              "curve speed %.1f-%.1f km/h, end at (%.0f, %.0f)" % (
                  max(straight), max(settled), max(curve), max(final), min(curve_speed), max(curve_speed),
                  log[-1][4], log[-1][5]))
        self.assertLess(max(straight), 0.05)   # the 0.8 m start offset is gone: on the centre
        self.assertLess(max(settled), 0.40)    # 3.5 m lane, 1.9 m car: 0.8 m to the marking
        self.assertLess(max(curve), 0.40)
        self.assertLess(max(final), 0.10)
        self.assertLess(max(curve_speed), math.sqrt(PF.A_LAT_MAX * road.r) * 3.6 + 3.0)  # slowed for the curve
        self.assertGreater(log[-1][5], 150.0)  # well into the last straight

    def check_tight_turns(self, plant, limit):
        """A junction's 11 m right turn and an 8 m hairpin: front and rear axle both near the centre."""
        for name, segs in (("right turn R 11 m", [("S", 60), ("C", -11.0, 90), ("S", 400)]),
                           ("hairpin R 8 m", [("S", 40), ("C", 8.0, 180), ("S", 400)])):
            road = Road.build(segs)
            log = drive(road.lane, 30.0, plant=plant(), road=road)
            front = max(abs(o) for t, o, v, h, x, y, b, r in log if t > 3.0)
            rear = max(abs(r) for t, o, v, h, x, y, b, r in log if t > 3.0)
            print("%s, %s: front axle max %.2f m, rear axle max %.2f m" % (type(plant()).__name__, name, front, rear))
            self.assertLess(max(front, rear), limit, name)
            self.assertLess(abs(log[-1][1]), 0.1, name + ": back on the centre after the turn")

    def test_tight_turns_with_the_mock_carsim(self):
        self.check_tight_turns(mock, 0.6)

    def test_tight_turns_with_tyre_slip_and_steering_lag(self):
        self.check_tight_turns(lambda: DynamicBicycle(NAMES), 0.6)
        self.check_tight_turns(lambda: DynamicBicycle(NAMES, tau=0.25, delay=0.1), 0.6)

    def test_stable_with_much_steering_lag_at_60_kmh(self):
        # 0.8 m off the centre on a straight, 60 km/h, the steering 0.35 s behind: no weaving.
        road = Road.build([("S", 700)], shift=-0.8)
        log = drive(road.lane, 30.0, plant=DynamicBicycle(NAMES, tau=0.25, delay=0.1), target_kmh=60.0)
        late = [abs(o) for t, o, v, h, x, y, b, r in log if t > 12.0]
        print("60 km/h, steering lag 0.35 s: offset after 12 s max %.3f m, speed %.0f km/h" % (max(late), log[-1][2]))
        self.assertGreater(log[-1][2], 57.0)
        self.assertLess(max(late), 0.10)


class JunctionTests(unittest.TestCase):
    """Inside a CARLA junction the nearest lane can be a turning lane that overlaps
    the straight one: the reading jumps to it, or there is none for a moment."""

    X0 = 100.0  # the junction starts here, 12 m long

    def reading(self, jump=None, gap=False):
        main, turn = Road.straight(), Road.right_turn(self.X0)

        def read(X, Y, yaw):
            if self.X0 <= X <= self.X0 + 12.0:
                if gap:
                    return None, 0.0
                if jump is None or jump():
                    return turn.lane(X, Y, yaw)[0], 0.0
            return main.lane(X, Y, yaw)
        return read

    def check_straight_through(self, log):
        through = [abs(y) for t, o, v, h, x, y, b, r in log if self.X0 - 20.0 <= x <= self.X0 + 60.0]
        self.assertGreater(len(through), 50, "never reached the junction")
        self.assertLess(max(through), 0.10)
        self.assertGreater(log[-1][4], self.X0 + 60.0)

    def test_keeps_its_path_when_the_reading_jumps_to_a_turning_lane(self):
        printed = []
        log = drive(self.reading(), 25.0, printed=printed)
        self.check_straight_through(log)
        text = "".join(printed)
        self.assertEqual(text.count("车道读数跳到了另一条车道"), 1, text)
        self.assertEqual(text.count("车道读数和路径又一致了"), 1, text)

    def test_the_reading_flickering_between_two_lanes(self):
        flip = iter(range(10 ** 6))
        printed = []
        self.check_straight_through(drive(self.reading(jump=lambda: next(flip) % 2 == 0), 25.0, printed=printed))
        text = "".join(printed)  # said once, not at every flip
        self.assertEqual(text.count("车道读数跳到了另一条车道"), 1, text)
        self.assertEqual(text.count("车道读数和路径又一致了"), 1, text)

    def test_no_lane_reading_for_a_moment_keeps_speed_and_path(self):
        printed = []
        log = drive(self.reading(gap=True), 25.0, printed=printed)
        self.check_straight_through(log)
        gap = [(v, b) for t, o, v, h, x, y, b, r in log if self.X0 <= x <= self.X0 + 12.0]
        self.assertTrue(gap and min(v for v, b in gap) > 35.0 and max(b for v, b in gap) == 0.0, gap[:3])
        self.assertIn("暂时没有车道信息", "".join(printed))
        self.assertNotIn("低速、方向盘回正", "".join(printed))

    def test_without_the_ego_pose_it_follows_each_reading(self):
        # No X / Y / Yaw for the algorithm: nothing to remember, it takes the turning lane
        # (so the checks above really catch a follower without the memory).
        log = drive(self.reading(), 25.0, pose=False)
        self.assertGreater(max(abs(y) for t, o, v, h, x, y, b, r in log), 1.0)

    def test_the_memory_runs_out_off_the_road(self):
        # Off every lane for good: after the remembered 50 m it slows down and centres the wheel.
        main = Road.straight()
        printed = []
        log = drive(lambda X, Y, yaw: (None, 0.0) if X > 60.0 else main.lane(X, Y, yaw), 30.0, printed=printed)
        self.assertIn("低速、方向盘回正", "".join(printed))
        self.assertLess(log[-1][2], 15.0)


if __name__ == "__main__":
    unittest.main()
