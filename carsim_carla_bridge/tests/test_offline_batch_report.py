"""批量测试 report without CARLA: a row per item from its run.json (pass =
finished by its duration, no collision, never off the lanes; an item that
did not start or has no record: not passed, its reason kept), report.csv
(with a BOM: Excel reads the Chinese headers) and report.md in the batch's
folder (relative: the bridge dir); served by the backend's IO thread.

    python tests/test_offline_batch_report.py
"""
import csv
import json
import os
import shutil
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

import batch_report  # noqa: E402
from backend_server import IO_CMDS  # noqa: E402


def run(root, name, end, kpi):
    f = os.path.join(root, name)
    os.makedirs(f)
    with open(os.path.join(f, "run.json"), "w", encoding="utf-8") as fh:
        json.dump({"end": end, "end_reason": "because", "kpi": kpi}, fh)
    return f


class BatchReportTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="cc_batch_")
        self.addCleanup(shutil.rmtree, self.tmp, True)
        root = os.path.join(self.tmp, "runs", "batch_x")
        self.good = run(root, "a", "finished", {"lane_offset_rms": 0.04, "collisions": 0, "time_off_lane": 0.0, "distance": 800})
        self.hit = run(root, "b", "finished", {"lane_offset_rms": 0.2, "collisions": 2, "time_off_lane": 0.0})
        self.off = run(root, "c", "finished", {"lane_offset_rms": 1.1, "collisions": 0, "time_off_lane": 3.5})
        self.stopped = run(root, "d", "stopped", {"lane_offset_rms": 0.05, "collisions": 0, "time_off_lane": 0.0})

    def test_rows_and_files(self):
        items = [{"label": "a", "record_dir": self.good, "state": "finished"},
                 {"label": "b", "record_dir": self.hit, "state": "finished"},
                 {"label": "c", "record_dir": self.off, "state": "finished"},
                 {"label": "d", "record_dir": self.stopped, "state": "stopped"},
                 {"label": "e | x", "record_dir": "", "state": "未启动", "detail": "那里没有这条车道"}]
        r = batch_report.summarize("runs/batch_x", items, base=self.tmp)
        self.assertEqual([x["passed"] for x in r["rows"]], [True, False, False, False, False])
        self.assertEqual((r["passed"], r["total"], r["dir"]), (1, 5, os.path.join(self.tmp, "runs", "batch_x")))
        self.assertEqual(r["rows"][4]["detail"], "那里没有这条车道")
        self.assertEqual(r["rows"][0]["lane_offset_rms"], 0.04)
        with open(r["csv"], encoding="utf-8-sig") as f:
            rows = list(csv.reader(f))
        self.assertEqual(rows[0][:4], ["测试项", "结果", "结束", "车道偏差均方根 m"])
        self.assertEqual(rows[1][:4], ["a", "通过", "finished", "0.040"])
        self.assertEqual(rows[5][1], "未通过")
        with open(r["csv"], "rb") as f:
            self.assertTrue(f.read(3) == b"\xef\xbb\xbf")
        md = open(r["md"], encoding="utf-8").read()
        self.assertIn("共 5 项，通过 1 项", md)
        self.assertIn("| a | ✅ 通过 | 0.040 |", md)
        self.assertIn("e | x", md)  # the label as given; a "|" in the reason would be replaced
        self.assertEqual(md.count("\n| "), 6)  # header + 5 rows

    def test_on_the_io_thread(self):
        self.assertIn("batch_summary", IO_CMDS)


if __name__ == "__main__":
    unittest.main(verbosity=2)
