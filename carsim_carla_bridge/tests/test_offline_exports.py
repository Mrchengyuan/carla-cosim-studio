"""Export order / unit checks, mock CarSim units, scene["units"] and the example
algorithms in both unit sets. Needs no CARLA server.

    python tests/test_offline_exports.py
"""

import math
import os
import shutil
import sys
import tempfile
import time
import types
import unittest
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import carla  # noqa: E402

import config as cfg  # noqa: E402
import session  # noqa: E402
import settings as st  # noqa: E402
from backend_server import Backend  # noqa: E402
from bridge import CarSimExports, ExportCheck  # noqa: E402
from mock_carsim import MockCarSimEnv  # noqa: E402
from scene import SceneProvider  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT = dict(cfg.UNITS)
SI = {"angle": "rad", "speed": "m/s", "rate": "rad/s", "wheel_spin": "rad/s", "jounce": "m"}


def swapped(a, b):
    n = list(cfg.EXPORT_NAMES)
    i, j = n.index(a), n.index(b)
    n[i], n[j] = n[j], n[i]
    return n


def drive(page_names=None, page_units=None, sim_units=None, action=(0.6, 0.0, 0.0), until=3.0):
    """The .sim (mock, default order) against what the CarSim page says.
    Returns [(t, warning)] and the check."""
    env = MockCarSimEnv(cfg.EXPORT_NAMES, units=sim_units or DEFAULT)
    chk = ExportCheck(CarSimExports(page_names or cfg.EXPORT_NAMES, page_units or DEFAULT), [0.35] * 4)
    obs = env.reset()
    out = [(0.0, w) for w in chk(obs)]
    while env.t_current < until and not chk.done:
        prev, t_prev = obs, env.t_current
        obs, _, _, _ = env.control_step(list(action), 20)
        if env.t_current >= 1.0:
            out += [(env.t_current, w) for w in chk(obs, prev, env.t_current - t_prev)]
    return out, chk


class ExportCheckTests(unittest.TestCase):
    def test_matching_setup_gives_no_warning(self):
        for units in (DEFAULT, SI):
            for action in ((0.6, 0.0, 0.0), (1.0, 0.0, 540.0), (1.0, 0.0, -540.0)):
                with self.subTest(units=units["speed"], action=action):
                    out, chk = drive(page_units=units, sim_units=units, action=action)
                    self.assertEqual(out, [])
                    self.assertTrue(chk.done, "the motion checks never ran")

    def test_swapped_order_warns(self):
        for (a, b), action, word in ((("Zo", "Vx"), (0.6, 0.0, 0.0), "Zo ="),
                                     (("Vx", "Vy"), (0.6, 0.0, 0.0), "Vx ="),
                                     (("Steer_L1", "Steer_SW"), (0.6, 0.0, 90.0), "Steer_L1 ="),
                                     (("AVy_L1", "AVz"), (0.6, 0.0, 0.0), "AVy_L1"),
                                     (("AVy_R2", "Jnc_L1"), (0.6, 0.0, 0.0), "AVy_R2")):
            with self.subTest(swap=(a, b)):
                out, _ = drive(page_names=swapped(a, b), action=action)
                self.assertTrue(any(word in w for _, w in out), out)
                self.assertTrue(all(t >= 1.0 for t, _ in out), "at rest these look fine; warned at %s" % out)

    def test_unit_mismatch_warns_in_carsim_units(self):
        out, _ = drive(page_units=dict(DEFAULT, speed="m/s"))
        self.assertEqual(len(out), 1, out)
        self.assertIn("Vx =", out[0][1])
        self.assertIn("m/s", out[0][1])
        self.assertNotIn("km/h", out[0][1])
        out, _ = drive(page_units=dict(DEFAULT, wheel_spin="rad/s"))
        self.assertEqual(len(out), 1, out)
        self.assertIn("AVy_L1、AVy_R1、AVy_L2、AVy_R2（rad/s）", out[0][1])
        self.assertIn("km/h", out[0][1])
        out, _ = drive(page_units=dict(DEFAULT, angle="rad"), action=(0.6, 0.0, 90.0))
        self.assertTrue(any("Pitch =" in w and "rad" in w for _, w in out), out)

    def test_each_warning_once(self):
        names = swapped("Zo", "Vx")
        chk = ExportCheck(CarSimExports(names, DEFAULT))
        obs = [0.0] * len(names)
        obs[names.index("Zo")] = 40.0
        self.assertEqual(len(chk(obs)), 1)
        self.assertEqual(chk(obs), [])

    def test_reference_height_and_toe(self):
        names = list(cfg.EXPORT_NAMES)
        obs = [0.0] * len(names)
        obs[names.index("Zo")] = 0.6
        obs[names.index("Steer_L1")], obs[names.index("Steer_R1")] = -0.15, 0.15  # toe-in, no steering
        ex = CarSimExports(names, DEFAULT)
        self.assertEqual(ExportCheck(ex, z0=0.6)(obs), [])
        w = ExportCheck(ex)(obs)
        self.assertEqual(len(w), 1)
        self.assertIn("Zo = 0.6 m", w[0])

    def test_zo_not_checked_without_z0(self):
        names = list(cfg.EXPORT_NAMES)
        obs = [0.0] * len(names)
        obs[names.index("Zo")] = 40.0  # a CarSim road that high: fine in height mode "ground"
        self.assertEqual(ExportCheck(CarSimExports(names, DEFAULT), z0=None)(obs), [])


class MockUnitsTests(unittest.TestCase):
    def run_mock(self, units, frames=60):
        env = MockCarSimEnv(cfg.EXPORT_NAMES, units=units)
        env.reset()
        obs = None
        for _ in range(frames):
            obs, _, _, _ = env.control_step([0.7, 0.0, 120.0], 20)
        return obs

    def test_mock_follows_units(self):
        a, b = self.run_mock(DEFAULT), self.run_mock(SI)
        va = dict(zip(cfg.EXPORT_NAMES, a))
        vb = dict(zip(cfg.EXPORT_NAMES, b))
        self.assertAlmostEqual(vb["Vx"], va["Vx"] / 3.6)
        self.assertGreater(va["Vx"], 1.0)
        for n in ("Yaw", "Roll", "Steer_SW", "Steer_L1", "Steer_R1"):
            self.assertAlmostEqual(vb[n], math.radians(va[n]), msg=n)
        self.assertAlmostEqual(vb["AVz"], math.radians(va["AVz"]))
        self.assertAlmostEqual(vb["AVy_L1"], va["AVy_L1"] * 2 * math.pi / 60.0)
        self.assertAlmostEqual(vb["Jnc_R1"], va["Jnc_R1"] / 1000.0)
        for n in ("Xo", "Yo", "Zo", "Throttle", "GearStat"):
            self.assertEqual(vb[n], va[n], n)
        # Read with the matching page units, both give the same pose / speeds.
        ea, eb = CarSimExports(cfg.EXPORT_NAMES, DEFAULT), CarSimExports(cfg.EXPORT_NAMES, SI)
        for f, n in ((ea.angle, "Yaw"), (ea.speed, "Vx"), (ea.rate, "AVz"), (ea.spin, "AVy_R1"), (ea.jounce, "Jnc_L1")):
            self.assertAlmostEqual(f(a, n), getattr(eb, f.__name__)(b, n), msg=n)

    def test_make_env_passes_the_page_units(self):
        d = st.default_dict()
        d["carsim"]["mock"] = True
        d["carsim"]["units"] = dict(SI)
        env = session.make_env(d)
        env.reset()
        obs, _, _, _ = env.control_step([0.7, 0.0, 0.0], 500)
        vx = dict(zip(cfg.EXPORT_NAMES, obs))["Vx"]
        self.assertAlmostEqual(vx, env.v, msg="Vx in m/s")


class SceneUnitsTests(unittest.TestCase):
    def provider(self, units):
        d = st.default_dict()
        d["carsim"]["units"] = units
        sp = SceneProvider(None, None, d)
        sp.latest = {"t": 0.0, "frame": 1, "ego": {"X": 1.0, "Speed": 2.0}, "objects": [], "collisions": []}
        return sp

    def test_algorithm_always_gets_units(self):
        sp = self.provider(dict(SI))
        self.assertEqual(sp.view()["units"], SI)
        self.assertNotIn("units", sp.record_view())  # not a record column
        sp.view()["units"]["speed"] = "changed"  # the algorithm may keep / edit it
        sp.latest = dict(sp.latest)
        sp._view = None
        self.assertEqual(sp.view()["units"]["speed"], "m/s")
        # A partial (older) units entry: the rest are the defaults.
        self.assertEqual(self.provider({"speed": "m/s"}).view()["units"], dict(DEFAULT, speed="m/s"))


class ExampleControllerTests(unittest.TestCase):
    """The same physical state in km/h and m/s: the same commands."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="cc_test_exports_")

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)
        sys.modules.pop("simple_controller", None)

    def outputs(self, path, units, scene, v_kmh, frames=5):
        d = st.default_dict()
        d["carsim"]["repo_path"] = self.tmp
        d["carsim"]["units"] = dict(units)
        d["run"]["controller"] = {"path": os.path.join(HERE, "..", path), "entry": "Controller"}
        ex = CarSimExports(cfg.EXPORT_NAMES, units)
        k = 1.0 if units["speed"] == "km/h" else 1 / 3.6
        obs = [0.0] * len(cfg.EXPORT_NAMES)
        obs[cfg.EXPORT_NAMES.index("Vx")] = v_kmh * k
        sc = dict(scene, units=dict(units))
        for o in sc.get("objects", []):
            o["rel_vx"] *= k
        ctl = session.load_controller(d, ex, lambda: 3, lambda: sc)
        return [ctl(tuple(obs), 0.1 * i) for i in range(frames)]

    def assert_same(self, path, scene, v_kmh):
        a = self.outputs(path, DEFAULT, {k: [dict(o) for o in v] if k == "objects" else v for k, v in scene.items()}, v_kmh)
        b = self.outputs(path, SI, {k: [dict(o) for o in v] if k == "objects" else v for k, v in scene.items()}, v_kmh)
        for x, y in zip(a, b):
            for p, q in zip(x, y):
                self.assertAlmostEqual(p, q, places=9)
        return a

    def test_example_controller(self):
        out = self.assert_same("controllers/example_controller.py", {"objects": []}, 40.0)
        self.assertEqual(out[0][:2], [0.0, 0.0], "at the target speed: no throttle, no brake")
        self.assert_same("controllers/example_controller.py", {"objects": []}, 10.0)

    def test_scene_controller(self):
        scene = {"objects": [{"rel_x": 20.0, "rel_y": 0.0, "rel_vx": -20.0, "gap": 15.0}],
                 "lane": {"center_rel": [[0.0, 0.0], [10.0, 0.3], [20.0, 0.8]]}}
        out = self.assert_same("controllers/scene_controller.py", scene, 30.0)
        self.assertGreater(out[0][1], 0.0, "closing in on the car ahead: brake")

    def test_simple_path_follower(self):
        seen = []

        class SimplePathFollower:
            def reset(self):
                pass

            def control(self, current_speed, target_speed, lateral_error, dt=0.01):
                seen.append(current_speed)
                return [0.0, 0.0, 0.0]

        sys.modules["simple_controller"] = types.SimpleNamespace(SimplePathFollower=SimplePathFollower)
        names = list(cfg.EXPORT_NAMES) + ["LatErr"]
        with mock.patch.object(cfg, "EXPORT_NAMES", names):
            self.assert_same("controllers/simple_path_follower.py", {"objects": []}, 36.0)
        self.assertTrue(seen and all(abs(v - 36.0) < 1e-9 for v in seen), seen)


class HillEnv(MockCarSimEnv):
    """The .sim's road lies 5 m up (Zo = 5 m on it)."""

    def _exports(self):
        v = list(super()._exports())
        v[self.export_names.index("Zo")] += 5.0
        return tuple(v)


class FakeSync:
    """CarlaVehicleSync without CARLA: the exports as the CarSim page reads them."""

    def __init__(self, world, vehicle, anchor, use_external_api=None, settings=None):
        self.ex = CarSimExports(settings.EXPORT_NAMES, settings.UNITS)
        self.wheel_radius_m = [0.35] * 4
        self.ref_local = [1.4, 0.0, 0.0]
        self.external_api, self.server_api = False, None

    def sync(self, obs, t, dt):
        return SimpleNamespace(velocity=carla.Vector3D(), wheel_steer=[], wheel_rotation=[], wheel_suspension=[])

    def release(self):
        pass


class SessionTests(unittest.TestCase):
    """CoSimSession.start() / step() hand the check's warnings out (start: at
    rest; then once, about 1 s in)."""

    def run_session(self, d, env=None, seconds=2.0):
        world = SimpleNamespace(get_settings=lambda: SimpleNamespace(), apply_settings=lambda s: None, tick=lambda: 1)
        bb = SimpleNamespace(location=SimpleNamespace(z=0.7), extent=SimpleNamespace(z=0.7))
        vehicle = SimpleNamespace(bounding_box=bb, get_transform=lambda: carla.Transform())
        patches = [mock.patch.object(session, "CarlaVehicleSync", FakeSync),
                   mock.patch.object(session, "start_scene", lambda *a, **k: {"collisions": []}),
                   mock.patch.object(session, "scene_step", lambda *a, **k: {"scene": {}, "collisions": []})]
        if env is not None:
            patches.append(mock.patch.object(session, "make_env", lambda d: env))
        for p in patches:
            p.start()
        try:
            ses = session.CoSimSession(world, vehicle, None, d)
            info = ses.start()
            out = []
            while ses.env.t_current < seconds:
                tel = ses.step()
                out += [(tel["t"], w) for w in tel["warnings"]]
            ses.stop()
            return info, out, ses
        finally:
            for p in patches:
                p.stop()

    def config(self, units):
        d = st.default_dict()
        d["carsim"]["mock"] = True
        d["carsim"]["units"] = dict(units)
        d["run"]["driver"] = "demo"
        d["run"]["log_path"] = ""
        return d

    def test_matching_units_no_warning(self):
        for units in (DEFAULT, SI):
            with self.subTest(units=units["speed"]):
                info, out, ses = self.run_session(self.config(units))
                self.assertEqual(info["warnings"], [])
                self.assertEqual(out, [])
                self.assertTrue(ses.export_check.done)

    def test_wrong_speed_unit_warns_once_after_1s(self):
        env = MockCarSimEnv(cfg.EXPORT_NAMES, t_stop=10.0)  # the .sim exports km/h
        info, out, _ = self.run_session(self.config(dict(DEFAULT, speed="m/s")), env)
        self.assertEqual(info["warnings"], [])
        self.assertEqual(len(out), 1, out)
        self.assertGreaterEqual(out[0][0], 1.0)
        self.assertIn("Vx =", out[0][1])

    def test_raised_carsim_road(self):
        d = self.config(DEFAULT)
        info, out, _ = self.run_session(d, HillEnv(cfg.EXPORT_NAMES, t_stop=10.0))
        self.assertEqual(len(info["warnings"]), 1, info["warnings"])
        self.assertIn("Zo = 5 m", info["warnings"][0])
        self.assertIn("贴合 CARLA 路面", info["warnings"][0])
        self.assertEqual(out, [])
        d["sync"]["z_mode"] = "ground"  # the car follows the CARLA road: Zo is not checked
        info, out, _ = self.run_session(d, HillEnv(cfg.EXPORT_NAMES, t_stop=10.0))
        self.assertEqual(info["warnings"], [])
        self.assertEqual(out, [])

    def test_backend_logs_start_warnings(self):
        class World:
            def get_settings(self):
                return SimpleNamespace(synchronous_mode=False, fixed_delta_seconds=None)

            def apply_settings(self, s):
                pass

            def tick(self):
                return 1

            def reset_all_traffic_lights(self):
                pass

        class StubSession:
            scene = None

            def __init__(self, *args):
                pass

            def start(self):
                return {"external_api": False, "server_api": None, "reference_point": [0.0, 0.0, 0.0],
                        "t_step": 0.001, "inner_steps": 20, "frame_dt": 0.02, "t_stop": 0.0, "mock": True,
                        "warnings": ["导出变量可疑：开始时"]}

        b = Backend()
        events = []
        b.emit = events.append
        b.world = World()
        b.cmd_spawn_ego = lambda *args: setattr(b, "ego", SimpleNamespace())
        with mock.patch("backend_server.CoSimSession", StubSession), \
                mock.patch("backend_server.rigmod.spec_of", lambda v: {}):
            info = b.cmd_cosim_start({"carsim": {"mock": True}})
        self.assertEqual(info["warnings"], ["导出变量可疑：开始时"])
        self.assertIn({"event": "log", "level": "warn", "msg": "导出变量可疑：开始时"}, events)
        self.assertEqual(b.cosim_state, "running")

    def test_backend_logs_and_strips_warnings(self):
        b = Backend()
        events = []
        b.emit = events.append
        b._update_spectator = lambda: None
        b.session = SimpleNamespace(step=lambda: {"t": 1.1, "frame": 2, "done": False, "world_frame": 7,
                                                  "collisions": [], "warnings": ["导出变量可疑：测试"]})
        b._cosim_frame(always_emit=True)
        self.assertIn({"event": "log", "level": "warn", "msg": "导出变量可疑：测试"}, events)
        tel = [e for e in events if e.get("event") == "telemetry"]
        self.assertEqual(len(tel), 1)
        self.assertNotIn("warnings", tel[0]["data"])


if __name__ == "__main__":
    unittest.main()
