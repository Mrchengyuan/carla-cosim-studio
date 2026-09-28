"""The example algorithms of docs/控制算法编写指南.md (controllers/examples/):
each file is the same as its code in the guide, and each runs through the
platform's own loader (session.load_controller, as a run does) in a closed
loop with the mock CarSim on the roads of test_offline_path_follower.

The examples give the brake as master-cylinder pressure (MPa, 0~8, like the
user's .sim); the mock takes a 0~1 pedal: divided by 8 on the way in."""

import contextlib
import io
import os
import re
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)

import session  # noqa: E402
import settings as st  # noqa: E402
from bridge import CarSimExports  # noqa: E402
from mock_carsim import MockCarSimEnv  # noqa: E402
from test_offline_path_follower import UNITS, Road  # noqa: E402

EXAMPLES = os.path.join(HERE, "..", "controllers", "examples")
GUIDE = os.path.join(HERE, "..", "..", "docs", "控制算法编写指南.md")
NAMES = st.default_dict()["carsim"]["export_names"]
FILES = ["ex1_cruise.py", "ex2_lane.py", "ex3_follow.py", "ex4_function.py",
         "ex5_my_algo/controller.py", "ex5_my_algo/speed.py", "ex5_my_algo/params.json"]


def read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def drive(rel, entry, road, seconds=40.0, lead=None):
    """Closed loop; lead: {"x0", "v"} a car ahead on the straight road. Returns
    (speeds km/h, |lane offset| after 10 s, gaps m, brakes MPa, printed text)."""
    d = st.load_dict(None, {})
    d["run"]["controller"] = {"path": os.path.join(EXAMPLES, rel), "entry": entry}
    d["sync"]["frame_dt"] = 0.02
    env = MockCarSimEnv(NAMES, t_step=0.001, t_stop=1e9, units=UNITS)
    now = {"obs": env.reset(), "t": 0.0}

    def scene():
        e = dict(zip(NAMES, now["obs"]))
        lane, _ = road.lane(e["Xo"], e["Yo"], e["Yaw"])
        objects = []
        if lead is not None:
            rel_x = lead["x0"] + lead["v"] / 3.6 * now["t"] - e["Xo"]
            objects.append({"id": 1, "type": "vehicle", "rel_x": rel_x, "rel_y": -e["Yo"],
                            "rel_vx": lead["v"] - e["Vx"], "rel_vy": 0.0, "dist": abs(rel_x),
                            "gap": max(0.0, abs(rel_x) - 4.7)})
        return {"units": UNITS, "t": now["t"], "lane": lane, "objects": objects,
                "ego": {"X": e["Xo"], "Y": e["Yo"], "Yaw": e["Yaw"], "Speed": e["Vx"]}}

    out = io.StringIO()
    speeds, offsets, gaps, brakes = [], [], [], []
    with contextlib.redirect_stdout(out):
        ctl = session.load_controller(d, CarSimExports(NAMES, UNITS), lambda: 3, scene)
        for _ in range(int(seconds / 0.02)):
            sc = scene()
            action = ctl(now["obs"], now["t"])
            speeds.append(dict(zip(NAMES, now["obs"]))["Vx"])
            if now["t"] > 10.0:
                offsets.append(abs(sc["lane"]["offset"]))
            if lead is not None:
                gaps.append(sc["objects"][0]["gap"])
            brakes.append(action[1])
            now["obs"], _, _, _ = env.control_step([action[0], action[1] / 8.0, action[2]], 20)
            now["t"] += 0.02
        session.call_finish(ctl, "达到设定的运行时长 %.0f s" % seconds)
    return speeds, offsets, gaps, brakes, out.getvalue()


class GuideTests(unittest.TestCase):
    def test_the_files_are_the_guides_code(self):
        guide = read(GUIDE)
        blocks = {b.strip() for b in re.findall(r"```(?:python|json)\n(.*?)```", guide, re.S)}
        for f in FILES:
            with self.subTest(file=f):
                self.assertIn(read(os.path.join(EXAMPLES, f)).strip(), blocks)

    def test_the_guides_table_names_the_files(self):
        guide = read(GUIDE)
        for f, entry in (("ex1_cruise.py", "Controller"), ("ex4_function.py", "control"),
                         ("ex5_my_algo/controller.py", "Controller")):
            self.assertIn("`controllers/examples/%s` | `%s`" % (f, entry), guide)


class ExampleTests(unittest.TestCase):
    def test_ex1_cruise(self):
        speeds, _, _, _, _ = drive("ex1_cruise.py", "Controller", Road.build([("S", 900)]))
        self.assertAlmostEqual(speeds[-1], 30.0, delta=0.5)

    def test_ex2_lane_through_a_curve(self):
        road = Road.with_curve()  # starts 0.8 m off the centre, a 40 m radius curve
        speeds, offsets, _, _, _ = drive("ex2_lane.py", "Controller", road)
        self.assertLess(max(offsets), 0.3)
        self.assertAlmostEqual(speeds[-1], 30.0, delta=1.0)

    def test_ex3_follows_a_slower_car_and_cruises_without_one(self):
        speeds, offsets, gaps, brakes, _ = drive("ex3_follow.py", "Controller", Road.build([("S", 900)]),
                                                 lead={"x0": 40.0, "v": 20.0})
        want = 5.0 + 1.5 * 20.0 / 3.6
        self.assertAlmostEqual(speeds[-1], 20.0, delta=1.0)
        self.assertAlmostEqual(gaps[-1], want, delta=1.0)
        self.assertGreater(min(gaps), 8.0)
        self.assertLessEqual(max(brakes), 8.0)
        speeds, offsets, _, _, _ = drive("ex3_follow.py", "Controller", Road.with_curve())
        self.assertLess(max(offsets), 0.3)
        self.assertAlmostEqual(speeds[-1], 40.0, delta=1.0)

    def test_ex4_function_entry_and_its_finish(self):
        speeds, _, _, _, printed = drive("ex4_function.py", "control", Road.build([("S", 900)]))
        self.assertAlmostEqual(speeds[-1], 30.0, delta=0.5)
        self.assertIn("运行结束： 达到设定的运行时长 40 s", printed)

    def test_ex5_helper_module_and_params_file(self):
        speeds, _, _, _, printed = drive("ex5_my_algo/controller.py", "Controller", Road.build([("S", 900)]))
        self.assertAlmostEqual(speeds[-1], 30.0, delta=0.5)
        self.assertIn("'target_kmh': 30", printed)


if __name__ == "__main__":
    unittest.main()
