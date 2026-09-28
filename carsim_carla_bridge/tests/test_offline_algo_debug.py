"""The algorithm's own values (self.debug) without CARLA: cleaned to finite
numbers (other values named, a non-dict refused in plain words, at most 32),
taken after every control() call and kept (a class or a module-level debug),
recorded in <log>_debug.csv at the record's samples with the columns of the
first one (later names reported, not written).

    python tests/test_offline_algo_debug.py
"""
import csv
import math
import os
import shutil
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

import numpy as np  # noqa: E402

import session  # noqa: E402
import settings  # noqa: E402
from scene import Recorder  # noqa: E402


class DebugValuesTests(unittest.TestCase):
    def test_numbers_kept_the_rest_named(self):
        vals, bad = session.debug_values({"ESS": np.float32(16.0), "n": 3, "ok": True, "text": "abc",
                                          "nan": float("nan"), "none": None, 5: 2.5})
        self.assertEqual(vals, {"ESS": 16.0, "n": 3.0, "ok": 1.0, "5": 2.5})
        self.assertEqual(sorted(bad), ["none", "text"])  # NaN: a number, just not shown

    def test_not_a_dict(self):
        with self.assertRaisesRegex(RuntimeError, "self.debug 应是"):
            session.debug_values([1, 2])

    def test_at_most_32(self):
        vals, _ = session.debug_values({"v%d" % i: i for i in range(40)})
        self.assertEqual(len(vals), session.MAX_DEBUG)


class LoadedControllerTests(unittest.TestCase):
    """Through session.load_controller, as a run calls the algorithm."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="cc_dbg_")
        self.addCleanup(shutil.rmtree, self.tmp, True)

    def load(self, text, entry):
        path = os.path.join(self.tmp, "algo_dbg.py")
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
        d = settings.default_dict()
        d["run"].update(driver="custom", controller={"path": path, "entry": entry})

        class Ex:
            index = {"Vx": 0}

            @staticmethod
            def raw(obs, n):
                return obs[0]
        return session.load_controller(d, Ex())

    def test_class_values_taken_after_each_call(self):
        c = self.load("class C:\n    def control(self, e, t, dt):\n        self.debug = {'t': t, 'twice': 2 * t}\n"
                      "        return [0, 0, 0]\n", "C")
        self.assertIsNone(c.debug)
        c([1.0], 0.5)
        self.assertEqual(c.debug, {"t": 0.5, "twice": 1.0})
        c([1.0], 0.7)
        self.assertEqual(c.debug["t"], 0.7)

    def test_module_level_debug_and_bad_names(self):
        c = self.load("debug = None\n\ndef control(e, t, dt):\n    global debug\n"
                      "    debug = {'x': 1.5, 'label': 'fast'}\n    return [0, 0, 0]\n", "control")
        c([1.0], 0.0)
        self.assertEqual((c.debug, c.debug_bad), ({"x": 1.5}, {"label"}))


class RecorderTests(unittest.TestCase):
    def test_debug_csv_columns_from_the_first(self):
        tmp = tempfile.mkdtemp(prefix="cc_dbgrec_")
        self.addCleanup(shutil.rmtree, tmp, True)
        paths = Recorder.run_paths(os.path.join(tmp, "log.csv"))
        self.assertEqual(os.path.basename(paths["debug"]), "log_debug.csv")
        r = Recorder(paths, settings.default_dict()["scene"], ["Vx"], n_actions=3)
        scene = {"t": 0.0, "frame": 1, "ego": {}, "objects": [], "lane": None}
        r.write(scene, {"Vx": 1.0}, None, None)          # t0: no values yet, no file
        self.assertFalse(os.path.exists(paths["debug"]))
        r.write(dict(scene, t=0.1, frame=2), {"Vx": 1.0}, [0, 0, 0], {"ESS": 16.0, "cost": 2.5})
        r.write(dict(scene, t=0.2, frame=3), {"Vx": 1.0}, [0, 0, 0], {"ESS": 15.0, "new": 1.0})
        r.close()
        with open(paths["debug"], encoding="utf-8") as f:
            rows = list(csv.reader(f))
        self.assertEqual(rows[0], ["t", "frame", "ESS", "cost"])
        self.assertEqual(rows[1], ["0.1", "2", "16.0", "2.5"])
        self.assertEqual(rows[2][:3], ["0.2", "3", "15.0"])
        self.assertTrue(rows[2][3] in ("", "nan") or math.isnan(float(rows[2][3])))
        self.assertEqual(r.debug_new, {"new"})


if __name__ == "__main__":
    unittest.main(verbosity=2)
