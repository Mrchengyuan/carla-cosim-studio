"""运行对比 without CARLA: runs.list_runs (the record dir's runs newest first,
their algorithm, vehicle, duration, key figures; folders without run.json
left out; a missing dir in plain words) and runs.run_series (the time series
aligned by t: trajectory, speed, lane offset, the outputs named by the kind
of vehicle, self.debug values; thinned to max_points keeping the last
sample; not a run: refused in plain words); both served by the backend's
IO thread.

    python tests/test_offline_runs_compare.py
"""
import json
import os
import shutil
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

import runs  # noqa: E402
from backend_server import IO_CMDS  # noqa: E402


def make_run(root, name, run, n=50, u=3, lane=True, debug=True):
    f = os.path.join(root, name)
    os.makedirs(f)
    with open(os.path.join(f, "run.json"), "w", encoding="utf-8") as fh:
        json.dump(run, fh)
    with open(os.path.join(f, "log.csv"), "w", encoding="utf-8") as fh:
        fh.write("t,frame,ego_X,ego_Y,ego_Speed" + "".join(",u%d" % i for i in range(1, u + 1)) + "\n")
        for i in range(n):
            t = round(i * 0.1, 6)
            fh.write("%s,%d,%s,%s,%s" % (t, i, 2 * t, 0.1 * t, 72.0) + "".join(",%s" % (0.1 * k + t) if i else "," for k in range(u)) + "\n")
    if lane:
        with open(os.path.join(f, "log_lane.csv"), "w", encoding="utf-8") as fh:
            fh.write("t,frame,offset,heading_err\n")
            for i in range(0, n, 2):  # every other sample: others have no lane
                fh.write("%s,%d,%s,0\n" % (round(i * 0.1, 6), i, 0.01 * i))
    if debug:
        with open(os.path.join(f, "log_debug.csv"), "w", encoding="utf-8") as fh:
            fh.write("t,frame,ESS\n")
            for i in range(1, n):
                fh.write("%s,%d,16\n" % (round(i * 0.1, 6), i))
    return f


class RunsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="cc_runs_")
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.root = os.path.join(self.tmp, "runs")
        self.a = make_run(self.root, "20260101_120000_a", {"controller": "/x/algo_a.py", "map": "Town04", "t_start": 0.0,
                                                          "t_end": 4.9, "carsim_mock": True, "end": "finished",
                                                          "kpi": {"lane_offset_rms": 0.05, "collisions": 0}})
        self.b = make_run(self.root, "20260102_080000_b", {"controller": "/x/kmppi.py", "map": "Town04", "t_start": 0.0,
                                                          "t_end": 4.9, "carsim_chrono": True, "end": "stopped"}, u=2, debug=False)
        os.makedirs(os.path.join(self.root, "not_a_run"))

    def test_list_newest_first(self):
        r = runs.list_runs("runs", base=self.tmp)
        self.assertEqual(r["error"], "")
        self.assertEqual([x["name"] for x in r["runs"]], ["20260102_080000_b", "20260101_120000_a"])
        b, a = r["runs"]
        self.assertEqual((a["controller"], a["vehicle"], a["map"], round(a["duration"], 3), a["debug"]),
                         ("algo_a.py", "模拟 CarSim", "Town04", 4.9, True))
        self.assertEqual(a["kpi"]["lane_offset_rms"], 0.05)
        self.assertEqual((b["vehicle"], b["kpi"], b["debug"]), ("Chrono 宝马", {}, False))

    def test_missing_dir(self):
        r = runs.list_runs("nope", base=self.tmp)
        self.assertEqual(r["runs"], [])
        self.assertIn("运行记录目录不存在", r["error"])

    def test_series(self):
        s = runs.run_series(self.a)
        self.assertEqual(len(s["t"]), 50)
        self.assertEqual((s["t"][3], s["x"][3], s["speed"][3]), (0.3, 0.6, 72.0))
        self.assertEqual((s["offset"][2], s["offset"][3]), (0.02, None))  # a sample without a lane
        self.assertEqual(s["u"][0][0], None)  # t0: before the first control() call
        self.assertEqual(s["u_names"], ["油门", "制动", "方向盘转角 (°)"])
        self.assertEqual((s["debug"]["ESS"][0], s["debug"]["ESS"][1]), (None, 16.0))
        c = runs.run_series(self.b)
        self.assertEqual((c["u_names"], c["debug"], c["u"][2][:2]), (["ax (m/s²)", "前轮转角 (rad)"], {}, [None, None]))

    def test_thinned_keeps_the_end(self):
        s = runs.run_series(self.a, max_points=7)
        self.assertLessEqual(len(s["t"]), 9)
        self.assertEqual((s["t"][0], s["t"][-1]), (0.0, 4.9))

    def test_not_a_run(self):
        with self.assertRaisesRegex(ValueError, "不是一次运行的记录"):
            runs.run_series(os.path.join(self.root, "not_a_run"))

    def test_on_the_io_thread(self):
        self.assertTrue({"runs_list", "run_series"} <= IO_CMDS)


if __name__ == "__main__":
    unittest.main(verbosity=2)
