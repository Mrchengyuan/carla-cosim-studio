"""算法参数 without CARLA: an algorithm file's upper-case constants read
with ast, never run (numbers, text, True / False, negative numbers, `A, B =
1, 2`, the comment at the end of the line; lower-case names and expressions
left out; a file that does not parse: the error); the page's changes by
algorithm file (run.params); set on the loaded module before the algorithm
object is made, with the type of the file's value, and said in the output
(names the file no longer has, too); a changed FRAME_DT is the run's step.

    python tests/test_offline_algo_params.py
"""
import contextlib
import io
import os
import shutil
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

import session  # noqa: E402
import settings  # noqa: E402

ALGO = '''"""An algorithm with constants."""
import math
SPEED = 3.0  # m/s，目标车速
GAIN = -2  # 负数
NAME = "fast"
ON = True
KP, KI = 0.08, 0.02  # 车速 PI
lower = 5.0
DERIVED = SPEED * 2  # 表达式：不列
FRAME_DT = 0.05


class Controller:
    def reset(self):
        self.seen = (SPEED, GAIN, NAME, ON, KP)

    def control(self, exports, t, dt):
        print("seen", self.seen)
        return [0, 0, 0]
'''


class ParamsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="cc_params_")
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.path = os.path.join(self.tmp, "algo_params.py")
        with open(self.path, "w", encoding="utf-8") as f:
            f.write(ALGO)

    def test_read_never_run(self):
        with open(os.path.join(self.tmp, "side.py"), "w") as f:
            f.write("open(%r, 'w').write('ran')\nX = 1\n" % os.path.join(self.tmp, "ran.txt"))
        self.assertEqual([p["name"] for p in session.controller_params(os.path.join(self.tmp, "side.py"))["params"]], ["X"])
        self.assertFalse(os.path.exists(os.path.join(self.tmp, "ran.txt")))
        r = session.controller_params(self.path)
        got = {p["name"]: (p["value"], p["type"], p["comment"]) for p in r["params"]}
        self.assertEqual(got, {"SPEED": (3.0, "float", "m/s，目标车速"), "GAIN": (-2, "int", "负数"),
                               "NAME": ("fast", "str", ""), "ON": (True, "bool", ""), "KP": (0.08, "float", "车速 PI"),
                               "KI": (0.02, "float", "车速 PI"), "FRAME_DT": (0.05, "float", "")})
        self.assertEqual(r["error"], "")

    def test_a_broken_file(self):
        with open(os.path.join(self.tmp, "bad.py"), "w") as f:
            f.write("X = (\n")
        r = session.controller_params(os.path.join(self.tmp, "bad.py"))
        self.assertEqual(r["params"], [])
        self.assertTrue(r["error"])

    def cfg(self, over, key=None):
        d = settings.default_dict()
        d["run"].update(driver="custom", controller={"path": self.path, "entry": "Controller"},
                        params={key or self.path: over, "/other/file.py": {"SPEED": 99.0}})
        return d

    def test_by_file(self):
        self.assertEqual(session.params_for(self.cfg({"SPEED": 5.0})), {"SPEED": 5.0})
        rel = os.path.relpath(self.path)
        self.assertEqual(session.params_for(self.cfg({"SPEED": 5.0}, key=rel)), {"SPEED": 5.0})
        d = self.cfg({"SPEED": 5.0})
        d["run"]["params"] = {"/other/file.py": {"SPEED": 99.0}}
        self.assertEqual(session.params_for(d), {})

    def load(self, over):
        d = self.cfg(over)

        class Ex:
            index = {}

            @staticmethod
            def raw(obs, n):
                return 0.0
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            c = session.load_controller(d, Ex())
            c([], 0.0)
        return out.getvalue()

    def test_set_before_the_object_is_made(self):
        text = self.load({"SPEED": 5, "GAIN": 3.0, "NAME": "slow", "ON": 0, "KP": "0.5", "GONE": 1})
        self.assertIn("seen (5.0, 3, 'slow', False, 0.5)", text)  # the file's types: float, int, str, bool, float
        self.assertIn("算法参数（界面上改过的）：", text)
        self.assertIn("SPEED = 5.0（文件里 3.0）", text)
        self.assertIn("这些参数文件里已经没有了（或值不对），没有用上：GONE", text)

    def test_nothing_changed(self):
        text = self.load({})
        self.assertIn("seen (3.0, -2, 'fast', True, 0.08)", text)
        self.assertNotIn("算法参数", text)

    def test_frame_dt_changed_on_the_page(self):
        d = self.cfg({"FRAME_DT": 0.025})
        notes = session.check_run_config(d)
        self.assertEqual(d["sync"]["frame_dt"], 0.025)
        self.assertTrue(any("0.025" in n for n in notes), notes)


if __name__ == "__main__":
    unittest.main(verbosity=2)
