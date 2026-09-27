"""Disk-safety checks that need no CARLA server: disk_info on a drive that is
not there, the CARLA recorder state, the run record (rounded numbers, free
space guard) and the removed raw-data paths."""

import csv
import io
import ntpath
import os
import shutil
import sys
import tempfile
import threading
import unittest
from types import SimpleNamespace
from unittest import mock

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from backend_server import Backend  # noqa: E402
import collector as coll  # noqa: E402
import run_cosim  # noqa: E402
import scene as scn  # noqa: E402
import session  # noqa: E402
import settings  # noqa: E402

MISSING = {"path": "Z:\\", "free_gb": None, "total_gb": None, "error": "输出目录所在的盘或网络共享不存在：Z:\\"}


def in_thread(fn, timeout=5.0):
    """fn() in a daemon thread: (still running after timeout = hung, result)."""
    out = {}

    def run():
        out["result"] = fn()

    t = threading.Thread(target=run, daemon=True)
    t.start()
    t.join(timeout)
    return t.is_alive(), out.get("result")


class DiskInfoTests(unittest.TestCase):
    def test_missing_windows_drive_or_share_does_not_hang(self):
        # ntpath.dirname("Z:\\") is "Z:\\": the old parent walk never ended.
        for path in ("Z:\\datasets\\run1", "\\\\nas\\share\\datasets"):
            with self.subTest(path=path), mock.patch.object(coll.os, "path", ntpath):
                hung, r = in_thread(lambda: coll.disk_info(path))
            self.assertFalse(hung, "disk_info hangs on %s" % path)
            self.assertIsNone(r["free_gb"])
            self.assertIn("不存在", r["error"])

    def test_existing_parent(self):
        with tempfile.TemporaryDirectory() as root:
            r = coll.disk_info(os.path.join(root, "not", "yet"))
            self.assertEqual(r["path"], os.path.abspath(root))
            self.assertGreater(r["free_gb"], 0.0)

    def test_unreadable_drive(self):
        with mock.patch.object(coll.shutil, "disk_usage", side_effect=OSError("device not ready")):
            r = coll.disk_info(".")
        self.assertIsNone(r["free_gb"])
        self.assertIn("device not ready", r["error"])


class MissingDriveCollectionTests(unittest.TestCase):
    def config(self, **overrides):
        return {"frame_dt": 0.1, "capture_every": 1, "image_format": "jpg", "out_dir": "Z:\\datasets",
                "max_frames": 5, "max_seconds": 0, "max_gb": 0, "labels": False, **overrides}

    def test_run_start_refused_before_the_ego_is_touched(self):
        backend = Backend()
        backend.world = object()
        spawned = []
        backend.cmd_spawn_ego = lambda *args: spawned.append(args)
        cfg = {"collect": {"enabled": True, "out_dir": "Z:\\datasets", "max_frames": 5},
               "rig": {"sensors": [{"name": "cam", "type": "rgb", "attributes": {}}]}}
        with mock.patch.object(coll, "disk_info", return_value=dict(MISSING)):
            with self.assertRaisesRegex(RuntimeError, "不存在"):
                backend.cmd_cosim_start(cfg)
        self.assertEqual(spawned, [])

    def test_collector_start_refused(self):
        c = coll.DataCollector(None, None, [], self.config())
        with mock.patch.object(coll, "disk_info", return_value=dict(MISSING)):
            with self.assertRaisesRegex(RuntimeError, "不存在"):
                c.start()

    def test_collection_stops_when_the_drive_goes_away(self):
        c = coll.DataCollector(None, None, [], self.config(max_frames=0))
        c.root = "E:\\datasets\\session"
        with mock.patch.object(coll, "disk_info", return_value=dict(MISSING)):
            c._check_limits()
        self.assertTrue(c.done)
        self.assertIn("不可用", c.stop_reason)


class RunRecordTests(unittest.TestCase):
    SEL = {"record": {"ego": ["X", "Yaw", "Speed"], "objects": ["rel_x"], "lane": ["curvature"]}}

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="cc_disk_rec_")
        self.paths = scn.Recorder.run_paths(os.path.join(self.tmp, "log.csv"))

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def scene(self, frame=5):
        return {"t": 1.23456789, "frame": frame,
                "ego": {"X": 1234.56789012345, "Yaw": np.float32(12.345678), "Speed": 0.1 + 0.2},
                "objects": [{"id": 7, "type": "vehicle", "rel_x": -3.141592653589793}],
                "lane": {"curvature": 0.00123456789012, "center_rel": []}}

    def rows(self, key):
        with open(self.paths[key], newline="", encoding="utf-8") as f:
            return list(csv.reader(f))

    def test_numbers_have_8_significant_digits(self):
        rec = scn.Recorder(self.paths, self.SEL, ["Xo", "Flag"])
        rec.write(self.scene(), {"Xo": 1234.56789012345, "Flag": 1})
        rec.close()
        self.assertEqual(self.rows("main"), [["t", "frame", "ego_X", "ego_Yaw", "ego_Speed", "Xo", "Flag"],
                                             ["1.234568", "5", "1234.5679", "12.345678", "0.3", "1234.5679", "1"]])
        self.assertEqual(self.rows("objects")[1], ["1.234568", "5", "7", "vehicle", "-3.1415927"])
        self.assertEqual(self.rows("lane")[1], ["1.234568", "5", "0.0012345679"])

    def test_stops_below_the_free_space_reserve(self):
        rec = scn.Recorder(self.paths, self.SEL, [], min_free_gb=1e12)
        rec.write(self.scene(), {})
        self.assertIn("停止写入", rec.stopped)
        self.assertEqual(rec.files, [])  # closed
        rec.write(self.scene(6), {})  # a no-op now
        self.assertEqual(len(self.rows("main")), 1)  # the header only

    def test_drive_gone(self):
        rec = scn.Recorder(self.paths, self.SEL, [], min_free_gb=10.0)
        with mock.patch.object(scn.shutil, "disk_usage", side_effect=OSError("gone")):
            rec.write(self.scene(), {})
        self.assertIn("不可用", rec.stopped)

    def test_free_space_checked_once_a_second(self):
        rec = scn.Recorder(self.paths, self.SEL, [], min_free_gb=10.0)
        clock = [100.0]
        usage = mock.Mock(return_value=SimpleNamespace(free=1e15, total=2e15))
        with mock.patch.object(scn.shutil, "disk_usage", usage), mock.patch.object(scn.time, "monotonic", lambda: clock[0]):
            for f in range(10):
                rec.write(self.scene(f), {})
            self.assertEqual(usage.call_count, 1)
            clock[0] += 1.5
            rec.write(self.scene(10), {})
            self.assertEqual(usage.call_count, 2)
        rec.close()
        self.assertEqual(rec.stopped, "")
        self.assertEqual(len(self.rows("main")), 12)

    def test_no_check_without_a_reserve(self):
        # The data collector has its own disk guard for its frames.csv.
        rec = scn.Recorder(self.paths, self.SEL, [])
        with mock.patch.object(scn.shutil, "disk_usage", side_effect=AssertionError("checked")):
            rec.write(self.scene(), {})
        rec.close()
        self.assertEqual(len(self.rows("main")), 2)

    def test_scene_step_reports_once_and_run_goes_on(self):
        rec = scn.Recorder(self.paths, self.SEL, [], min_free_gb=1e12)
        sc = self.scene()
        sc["collisions"] = []
        sc["objects"][0].update({k: 0.0 for k in scn.OBJECT_KEYS if k not in sc["objects"][0]}, parked=False, dist=1.0)
        ses = SimpleNamespace(
            d={"collect": {"sample_period": 0.0, "capture_every": 1}, "sync": {"frame_dt": 0.02},
               "scene": {"collision": "log"}},
            scene=SimpleNamespace(update=lambda frame, t, v: sc, record_view=lambda: sc, _ego_box=(0.0, 0.0)),
            recorder=rec, exports=lambda: {}, end_reason="", frame=1, last_action=None)
        tel = session.scene_step(ses, 1, 0.02)
        self.assertIn("停止写入", tel["warning"])
        self.assertIsNone(ses.recorder)
        ses.frame = 2
        self.assertNotIn("warning", session.scene_step(ses, 2, 0.04))

    def test_backend_logs_the_warning_not_the_telemetry(self):
        backend = Backend()
        events = []
        backend.emit = events.append
        backend.cosim_state = "running"
        backend.session = SimpleNamespace(step=lambda: {"t": 0.02, "frame": 1, "world_frame": 9, "done": False,
                                                         "warning": "磁盘只剩 1.0 GB"})
        backend._cosim_frame(always_emit=True)
        self.assertIn({"event": "log", "level": "warn", "msg": "磁盘只剩 1.0 GB"}, events)
        tel = [e["data"] for e in events if e.get("event") == "telemetry"]
        self.assertEqual(len(tel), 1)
        self.assertNotIn("warning", tel[0])

    def test_start_warning_is_logged_not_returned(self):
        class Session:
            scene, done = None, False

            def __init__(self, *a):
                pass

            def start(self):  # start_scene's step 0 found the disk (nearly) full
                return {"external_api": False, "server_api": None, "reference_point": [0, 0, 0], "t_step": 0.001,
                        "inner_steps": 20, "frame_dt": 0.02, "t_stop": 0.0, "mock": True, "warnings": [],
                        "t": 0.0, "collisions": [], "warning": "磁盘只剩 1.0 GB，运行记录停止写入"}

        settings_obj = SimpleNamespace(synchronous_mode=False, fixed_delta_seconds=None)
        backend = Backend()
        events = []
        backend.emit = events.append
        backend.world = SimpleNamespace(get_settings=lambda: settings_obj, apply_settings=lambda s: None,
                                        tick=lambda: 0, reset_all_traffic_lights=lambda: None)
        backend.cmd_spawn_ego = lambda *a: setattr(backend, "ego", SimpleNamespace(
            type_id="vehicle.test", attributes={}, is_alive=True))
        with mock.patch("backend_server.CoSimSession", Session), mock.patch("backend_server.rigmod.spec_of", lambda v: {}):
            info = backend.cmd_cosim_start({"carsim": {"mock": True}, "run": {"driver": "demo"}})
        self.assertIn({"event": "log", "level": "warn", "msg": "磁盘只剩 1.0 GB，运行记录停止写入"}, events)
        self.assertNotIn("warning", info)


class FakeActors(list):
    def filter(self, pattern):
        return FakeActors()


class FakeClient:
    def __init__(self, answer="/data/rec.log"):
        self.answer, self.calls = answer, []

    def start_recorder(self, name, additional_data):
        self.calls.append("start")
        return self.answer

    def stop_recorder(self):
        self.calls.append("stop")

    def set_timeout(self, t):
        pass

    def replay_file(self, name, start, duration, follow_id):
        self.calls.append("replay")
        return "Replaying File: %s" % name


def fake_world():
    s = SimpleNamespace(synchronous_mode=False, fixed_delta_seconds=None, no_rendering_mode=False)
    return SimpleNamespace(get_settings=lambda: s, get_map=lambda: SimpleNamespace(name="Carla/Maps/Town10HD_Opt"),
                           get_weather=lambda: None, get_actors=FakeActors, wait_for_tick=lambda timeout=None: None)


class CarlaRecorderStateTests(unittest.TestCase):
    def backend(self, answer="/data/rec.log"):
        b = Backend()
        b.events = []
        b.emit = b.events.append
        b.world, b.client = fake_world(), FakeClient(answer)
        b._weather_dict = lambda w: {}
        return b

    def test_start_and_stop_reported_by_world_info(self):
        b = self.backend()
        self.assertEqual(b.cmd_world_info()["recording"], "")
        self.assertEqual(b.cmd_start_recorder("rec.log"), "/data/rec.log")
        self.assertEqual(b.cmd_world_info()["recording"], "/data/rec.log")
        b.cmd_stop_recorder()
        self.assertEqual(b.cmd_world_info()["recording"], "")

    def test_file_carla_cannot_create(self):
        b = self.backend(answer="")
        with self.assertRaisesRegex(RuntimeError, "无法创建"):
            b.cmd_start_recorder("/no/such/dir/rec.log")
        self.assertEqual(b.cmd_world_info()["recording"], "")

    def test_teardown_stops_our_recording_only(self):
        b = self.backend()
        b._teardown()
        self.assertNotIn("stop", b.client.calls)  # not ours: leave it alone
        b.cmd_start_recorder("rec.log")
        b._teardown()  # reconnect, shutdown, exit
        self.assertEqual(b.client.calls, ["start", "stop"])
        self.assertIsNone(b.recording)

    def test_recover_stops_a_recording_left_behind(self):
        b = self.backend()
        b._remove_leftovers()
        self.assertEqual(b.client.calls, ["stop"])

    def test_map_change_and_replay_end_the_recording(self):
        b = self.backend()
        b.cmd_start_recorder("rec.log")
        b._switch_world(fake_world)
        self.assertEqual(b.cmd_world_info()["recording"], "")
        self.assertTrue(any(e.get("level") == "warn" and "录制" in e.get("msg", "") for e in b.events))
        b.cmd_start_recorder("rec.log")
        with tempfile.NamedTemporaryFile(suffix=".log") as f:
            b.cmd_replay(f.name)
        self.assertIn("replay", b.client.calls)
        self.assertEqual(b.cmd_world_info()["recording"], "")

    def test_carla_lost_forgets_the_recording(self):
        b = self.backend()
        b.cmd_start_recorder("rec.log")
        b.carla_addr = ("localhost", 2000)
        b._carla_listening = lambda now=False: False
        b._check_carla(now=True)
        self.assertIsNone(b.world)
        self.assertIsNone(b.recording)


class RemovedRawDataPathTests(unittest.TestCase):
    def test_legacy_commands_are_gone(self):
        b = Backend()
        for cmd in ("add_sensor", "list_sensors", "remove_sensor", "view_start", "views_stop"):
            with self.subTest(cmd=cmd), self.assertRaisesRegex(RuntimeError, "未知命令"):
                b.handle({"cmd": cmd, "args": {}})
        self.assertTrue(b.handle({"cmd": "view_stop"}))  # the GUI's

    def test_no_chase_camera_dump(self):
        self.assertNotIn("record_dir", settings.default_dict()["run"])
        self.assertFalse(hasattr(session, "spawn_chase_camera"))
        with mock.patch.object(sys, "argv", ["run_cosim.py", "--mock", "--record", "out/"]), \
                mock.patch("sys.stderr", io.StringIO()), self.assertRaises(SystemExit):
            run_cosim.main()


if __name__ == "__main__":
    unittest.main()
