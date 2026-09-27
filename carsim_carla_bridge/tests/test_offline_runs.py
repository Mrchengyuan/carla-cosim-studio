"""One result folder per run, its key figures and the algorithm's finish(),
without a CARLA server: every run writes <record dir>/<time>_<algorithm>/
(an older config's cosim_log.csv: its folder and file names), with the CSV
files, config.json, a copy of the algorithm file and run.json; a second run
never touches the first one's files; the figures (lane offset, heading
error, time off the lane, collisions, gap ahead, distance, |Ay|) in CarSim
units from the full sampled scenes whatever the record ticks, contacts from
every step; finish(reason) of the instance (a class entry) or the module (a
function entry), once, before the record closes, its errors reported and
never raised, a slow one named by the heartbeat; CARLA dynamics complete
run.json even with CARLA gone; the backend and the command line pass how
the run ended and show the folder and the figures."""

import contextlib
import copy
import csv
import io
import json
import math
import os
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import run_cosim  # noqa: E402
import scene as scn  # noqa: E402
import session  # noqa: E402
import settings  # noqa: E402
from backend_server import Backend  # noqa: E402

CONTROLLER = '''
class Controller:
    def control(self, exports, t, dt):
        return [0.1, 0.0, 0.0]
'''


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.reader(f))


def read_json(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def obj(rel_x, rel_y, gap, width=1.8, model="vehicle.test", oid=5, length=4.5, rel_yaw=0.0):
    """An object of the full scene (every key)."""
    return dict({k: 0.0 for k in scn.OBJECT_KEYS}, id=oid, type="vehicle", parked=False, model=model,
                rel_x=rel_x, rel_y=rel_y, width=width, length=length, rel_yaw=rel_yaw, gap=gap, dist=abs(rel_x))


class ScriptedScene:
    """Stands in for scene.SceneProvider: the scene of run step k is script(k, t)."""
    script = None

    def __init__(self, world, ego, d, anchor=None, ref_local=None, sensor_cfgs=()):
        self.world, self.s, self.sensor_cfgs = world, d["scene"], list(sensor_cfgs)
        self.frame, self._ego_box, self.latest = None, (0.0, 0.0), None
        self.map = SimpleNamespace(name="Carla/Maps/Town04")
        self.step = 0

    def start(self):
        pass

    def stop(self):
        pass

    def update(self, frame, t=0.0, ego_velocity=None):
        self.frame = frame
        self.latest = dict(self.script(self.step, t), t=t, frame=frame)
        self.step += 1
        return self.latest

    def record_view(self):
        """The keys ticked for the record, as SceneProvider selects them."""
        return scn.SceneProvider._select(self, self.s.get("record") or {}, False)


class FakeWorld:
    def __init__(self, frame0=100):
        self.frame = frame0 - 1

    def tick(self):
        self.frame += 1
        return self.frame


def drive_script(k, t):
    """10 m/s along x on a lane, a car ahead closing in, a contact at step 3 (between two samples)."""
    return {"ego": {"X": 10.0 * t, "Y": 0.0, "width": 2.0},
            "lane": {"width": 3.5, "offset": 0.1 if k % 2 else -0.2, "heading_err": 1.0, "center_rel": []},
            "objects": [obj(20.0 - k, 0.3, 15.0 - k)],
            "collisions": [{"id": 9, "type": "walker", "model": "walker.x", "new": True}] if k == 3 else []}


class RunFolderTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="cc_runs_test_")
        self.addCleanup(lambda: __import__("shutil").rmtree(self.tmp, ignore_errors=True))
        self.ctrl = os.path.join(self.tmp, "my_ctrl.py")
        with open(self.ctrl, "w", encoding="utf-8") as f:
            f.write(CONTROLLER)

    def config(self, log_path):
        d = settings.default_dict()
        d["sync"]["frame_dt"] = 0.02
        d["collect"]["sample_period"] = 0.1          # every 5 steps
        d["run"]["log_path"] = log_path
        d["run"]["controller"]["path"] = self.ctrl
        d["scene"]["collision"] = "log"
        d["carsim"]["mock"] = True
        d["rig"]["sensors"] = [{"name": "cam", "type": "rgb", "enabled": False}]  # no preset fitted to a car
        return d

    def run_once(self, d, steps=12, script=drive_script, exports=None):
        """start_scene + scene_step with a scripted scene, then end_run (what stop() does last)."""
        ses = SimpleNamespace(d=d, world=FakeWorld(), vehicle=None, frame=0, recorder=None, scene=None, end_reason="",
                              last_action=None, run_meta={"traffic_seed": 7}, record_dir=None, kpi=None,
                              exports=lambda: dict(exports or {"Xo": 1.0}), export_names=lambda: ["Xo"],
                              n_actions=lambda: 3, sync=SimpleNamespace(external_api=True))
        ScriptedScene.script = staticmethod(script)
        with mock.patch.object(session, "SceneProvider", ScriptedScene):
            session.start_scene(ses, None, [1.4, 0.0, 0.0], 0.0)
            start = read_json(os.path.join(ses.record_dir, "run.json")) if ses.record_dir else None
            for k in range(1, steps + 1):
                ses.frame = k
                ses.last_action = [0.1, 0.0, float(k)]
                session.scene_step(ses, ses.world.tick(), 0.02 * k)
        if ses.recorder is not None:
            ses.recorder.close()
        return ses, start, session.end_run(ses, "finished", "达到设定的运行时长 0 s")

    def test_each_run_a_folder_with_csv_config_controller_and_run_json(self):
        d = self.config(os.path.join(self.tmp, "runs"))
        ses, start, summary = self.run_once(d)
        folder = ses.record_dir
        self.assertEqual(os.path.dirname(folder), os.path.join(self.tmp, "runs"))
        self.assertRegex(os.path.basename(folder), r"^\d{8}_\d{6}_my_ctrl$")
        self.assertEqual(sorted(os.listdir(folder)), ["config.json", "log.csv", "log_lane.csv", "log_objects.csv",
                                                      "my_ctrl.py", "run.json"])
        with open(os.path.join(folder, "my_ctrl.py"), encoding="utf-8") as f:
            self.assertEqual(f.read(), CONTROLLER)
        self.assertEqual(read_json(os.path.join(folder, "config.json")), json.loads(json.dumps(d)))
        rows = read_csv(os.path.join(folder, "log.csv"))
        self.assertEqual([r[0] for r in rows[1:]], ["0.0", "0.1", "0.2"])  # steps 0, 5, 10
        # run.json: written at the start (no end yet), completed at the end.
        self.assertIsNone(start["end"])
        run = read_json(os.path.join(folder, "run.json"))
        self.assertEqual({k: run[k] for k in ("map", "spawn_index", "traffic_seed", "dynamics", "driver", "controller",
                                              "external_api", "carsim_sim", "carsim_mock", "end", "end_reason")},
                         {"map": "Town04", "spawn_index": 0, "traffic_seed": 7, "dynamics": "cosim", "driver": "custom",
                          "controller": self.ctrl, "external_api": True, "carsim_sim": None,
                          "carsim_mock": True, "end": "finished", "end_reason": "达到设定的运行时长 0 s"})
        self.assertEqual((run["t_start"], run["t_end"]), (0.0, 0.24))
        self.assertEqual(run["units"]["angle"], "deg")
        self.assertEqual(run["kpi"], summary["kpi"])
        self.assertEqual(summary["record_dir"], folder)
        self.assertEqual(summary["errors"], [])

    def test_figures_from_the_samples_contacts_from_every_step(self):
        ses, _, summary = self.run_once(self.config(os.path.join(self.tmp, "runs")), exports={"Xo": 1.0, "Ay": -0.25})
        k = summary["kpi"]
        self.assertEqual(k["samples"], 3)                        # steps 0, 5, 10
        self.assertAlmostEqual(k["lane_offset_rms"], (0.2 ** 2 * 2 / 3 + 0.1 ** 2 / 3) ** 0.5, places=5)  # -0.2, 0.1, -0.2
        self.assertEqual((k["lane_offset_max"], k["heading_err_max"], k["time_off_lane"]), (0.2, 1.0, 0.0))
        self.assertEqual((k["collisions"], k["first_collision_t"], k["first_collision_with"]), (1, 0.06, "walker.x"))
        self.assertEqual(k["min_gap_ahead"], 5.0)                # step 10: 15 - 10
        self.assertAlmostEqual(k["distance"], 2.0)               # 10 m/s, samples 0 ... 0.2 s
        self.assertEqual(k["ay_max"], 0.25)
        self.assertIn("碰撞 1 次（第一次 t = 0.06 s，walker.x）", summary["kpi_text"])

    def test_figures_do_not_depend_on_the_record_ticks(self):
        d = self.config(os.path.join(self.tmp, "runs"))
        _, _, full = self.run_once(d)
        d["scene"]["record"] = {"ego": [], "objects": [], "lane": []}  # nothing ticked for the record
        ses, _, bare = self.run_once(d)
        self.assertEqual(read_csv(os.path.join(ses.record_dir, "log.csv"))[0], ["t", "frame", "Xo", "u1", "u2", "u3"])
        self.assertNotIn("log_lane.csv", os.listdir(ses.record_dir))
        self.assertEqual(bare["kpi"], full["kpi"])
        self.assertIsNotNone(full["kpi"]["lane_offset_rms"])

    def test_a_second_run_leaves_the_first_alone(self):
        d = self.config(os.path.join(self.tmp, "runs"))
        with mock.patch.object(session.time, "strftime", lambda fmt, *a: "20260927_101500"):
            first, _, _ = self.run_once(d)
            before = read_csv(os.path.join(first.record_dir, "log.csv"))
            d2 = copy.deepcopy(d)
            d2["scene"]["lane"], d2["scene"]["record"]["lane"] = [], []  # no lane keys this time
            second, _, _ = self.run_once(d2, steps=3)
        self.assertEqual(os.path.basename(first.record_dir), "20260927_101500_my_ctrl")
        self.assertEqual(os.path.basename(second.record_dir), "20260927_101500_my_ctrl_2")  # same second
        self.assertEqual(read_csv(os.path.join(first.record_dir, "log.csv")), before)
        # No _lane.csv of an earlier run next to this run's record.
        self.assertNotIn("log_lane.csv", os.listdir(second.record_dir))
        self.assertIn("log_lane.csv", os.listdir(first.record_dir))

    def test_older_config_file_name(self):
        ses, _, _ = self.run_once(self.config(os.path.join(self.tmp, "old", "cosim_log.csv")), steps=1)
        self.assertEqual(os.path.dirname(ses.record_dir), os.path.join(self.tmp, "old"))
        self.assertEqual(sorted(n for n in os.listdir(ses.record_dir) if n.endswith(".csv")),
                         ["cosim_log.csv", "cosim_log_lane.csv", "cosim_log_objects.csv"])
        self.assertEqual(os.listdir(os.path.join(self.tmp, "old")), [os.path.basename(ses.record_dir)])

    def test_no_record_still_has_figures(self):
        ses, _, summary = self.run_once(self.config(""), steps=5)
        self.assertIsNone(ses.record_dir)
        self.assertEqual(summary["kpi"]["samples"], 2)
        self.assertEqual(os.listdir(self.tmp), ["my_ctrl.py"])

    def test_disk_guard_stops_the_csv_not_the_run(self):
        with mock.patch.object(session, "DISK_RESERVE_GB", 1e12):
            ses, _, summary = self.run_once(self.config(os.path.join(self.tmp, "runs")), steps=5)
        self.assertIsNone(ses.recorder)  # stopped at step 0
        self.assertEqual(len(read_csv(os.path.join(ses.record_dir, "log.csv"))), 1)  # the header only
        self.assertEqual(read_json(os.path.join(ses.record_dir, "run.json"))["end"], "finished")
        self.assertEqual(summary["kpi"]["samples"], 2)

    def test_folder_names(self):
        d = settings.default_dict()
        self.assertEqual(session._run_stem(d), "example_controller")
        d["run"]["driver"] = "demo"
        self.assertEqual(session._run_stem(d), "demo")
        d["drive"].update(dynamics="carla", carla_driver="autopilot")
        self.assertEqual(session._run_stem(d), "carla_autopilot")
        path = os.path.join(self.tmp, "a_file")
        open(path, "w").close()
        with self.assertRaisesRegex(ValueError, "是一个文件"):
            session.run_dir(path, "x")
        self.assertEqual(settings.default_dict()["run"]["log_path"], "runs")


class KpiTests(unittest.TestCase):
    def test_figures(self):
        k = scn.RunKpi(0.1)
        ahead, beside, behind = obj(20.0, 0.5, 15.0), obj(5.0, 3.5, 1.0), obj(-6.0, 0.0, 2.0)
        scenes = [
            ({"t": 0.0, "ego": {"X": 0.0, "Y": 0.0, "width": 2.0}, "lane": {"offset": 0.1, "heading_err": 1.0},
              "objects": [ahead, beside, behind]}, {"Ay": 0.1}),
            ({"t": 0.1, "ego": {"X": 3.0, "Y": 4.0, "width": 2.0}, "lane": {"offset": -0.3, "heading_err": -2.5},
              "objects": [dict(ahead, gap=10.0)]}, {"Ay": -0.4}),
            ({"t": 0.2, "ego": {"X": 6.0, "Y": 8.0, "width": 2.0}, "lane": None,       # off the road
              "objects": [obj(8.0, -1.2, 4.0, width=0.6, model="walker.w")]}, {"Ay": 0.2}),
        ]
        for sc, ex in scenes:
            k.step(sc["t"], [])
            k.sample(sc, ex)
        k.step(0.25, [{"model": "vehicle.x"}])
        k.step(0.3, [{"model": "walker.w"}, {"model": "vehicle.y"}])
        r = k.result()
        self.assertAlmostEqual(r["lane_offset_rms"], 0.05 ** 0.5, places=5)
        self.assertEqual((r["lane_offset_max"], r["heading_err_max"], r["time_off_lane"]), (0.3, 2.5, 0.1))
        self.assertEqual((r["collisions"], r["first_collision_t"], r["first_collision_with"]), (3, 0.25, "vehicle.x"))
        self.assertEqual(r["min_gap_ahead"], 4.0)  # not the car beside (1.0) or behind (2.0)
        self.assertEqual((r["distance"], r["ay_max"], r["samples"]), (10.0, 0.4, 3))
        self.assertEqual((k.t_start, k.t_end), (0.0, 0.3))
        text = scn.kpi_text(r, "deg")
        for part in ("车道偏移 RMS 0.22 m、最大 0.30 m", "航向偏差最大 2.5 deg", "不在车道上 0.1 s",
                     "碰撞 3 次（第一次 t = 0.25 s，vehicle.x）", "前方最小间距 4.00 m", "行驶距离 10.0 m", "最大 |Ay| 0.4"):
            self.assertIn(part, text)

    def test_a_crossing_car_is_ahead(self):
        """A car turned across the lane reaches into the ego's path with its length, not its width."""
        for angle, yaw in (("deg", 90.0), ("rad", math.pi / 2)):
            k = scn.RunKpi(0.1, angle=angle)
            crossing = obj(12.0, 2.5, 9.0, rel_yaw=yaw)  # 2.5 - 4.5 / 2 < 1.0
            along = obj(12.0, 2.5, 8.0, oid=6)            # 2.5 - 1.8 / 2 > 1.0: the next lane
            k.sample({"t": 0.0, "ego": {"X": 0.0, "Y": 0.0, "width": 2.0}, "objects": [crossing, along]}, {})
            self.assertEqual(k.result()["min_gap_ahead"], 9.0, angle)

    def test_nothing_to_measure(self):
        k = scn.RunKpi(0.02, collisions=False)
        k.step(0.0, [{"model": "vehicle.x"}])
        k.sample({"t": 0.0, "ego": {"X": 0.0, "Y": 0.0}, "objects": []}, {})  # no lane keys, no Ay export
        r = k.result()
        self.assertEqual((r["lane_offset_rms"], r["time_off_lane"], r["collisions"], r["min_gap_ahead"]),
                         (None, None, None, None))
        self.assertNotIn("ay_max", r)
        text = scn.kpi_text(r)
        self.assertIn("没有车道数据", text)
        self.assertIn("前方没有目标", text)
        self.assertNotIn("碰撞", text)
        self.assertNotIn("Ay", text)


class FinishTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.calls = []

    def load(self, src, entry="Controller"):
        path = os.path.join(self.tmp.name, "fin_ctrl.py")
        with open(path, "w", encoding="utf-8") as f:
            f.write(src)
        d = settings.default_dict()
        d["run"]["controller"] = {"path": path, "entry": entry}
        ex = SimpleNamespace(index={"Xo": 0}, raw=lambda obs, n: 0.0)
        drv = session.load_controller(d, ex)
        sys.modules["user_controller"].CALLS = self.calls
        return drv

    def test_instance_finish_once_with_the_reason(self):
        drv = self.load("class Controller:\n"
                        "    def control(self, e, t, dt):\n        return [0, 0, 0]\n"
                        "    def finish(self, reason):\n        CALLS.append(('inst', reason))\n"
                        "def finish(reason):\n    CALLS.append(('mod', reason))\n")
        self.assertEqual(session.call_finish(drv, "碰撞：撞到 vehicle.x（id 3）"), [])
        self.assertEqual(session.call_finish(drv, "again"), [])
        self.assertEqual(self.calls, [("inst", "碰撞：撞到 vehicle.x（id 3）")])

    def test_module_finish_for_a_function_entry(self):
        drv = self.load("def control(e, t, dt):\n    return [0, 0, 0]\n"
                        "def finish(reason):\n    CALLS.append(reason)\n", entry="control")
        session.call_finish(drv, "运行被停止")
        self.assertEqual(self.calls, ["运行被停止"])

    def test_a_class_entry_ignores_a_module_finish(self):
        drv = self.load("class Controller:\n"
                        "    def control(self, e, t, dt):\n        return [0, 0, 0]\n"
                        "def finish(x):\n    CALLS.append(x)\n")  # a helper of the module, not the end hook
        self.assertIsNone(drv.finish)
        session.call_finish(drv, "x")
        self.assertEqual(self.calls, [])

    def test_a_slow_finish_is_named_by_the_heartbeat(self):
        drv = self.load("class Controller:\n"
                        "    def control(self, e, t, dt):\n        return [0, 0, 0]\n"
                        "    def finish(self, reason):\n        CALLS.append(BUSY())\n")
        sys.modules["user_controller"].BUSY = lambda: session.control_busy(drv)
        session.call_finish(drv, "x")
        self.assertEqual(self.calls[0][1], "（fin_ctrl.py 第 5 行）")  # where finish() is, for the GUI
        self.assertIsNone(session.control_busy(drv))

    def test_without_finish(self):
        drv = self.load(CONTROLLER)
        self.assertIsNone(drv.finish)
        self.assertEqual(session.call_finish(drv, "x"), [])
        self.assertEqual(session.call_finish(lambda obs, t: [0, 0, 0], "x"), [])  # demo / route drivers

    def test_errors_are_reported_not_raised(self):
        drv = self.load("class Controller:\n"
                        "    def control(self, e, t, dt):\n        return [0, 0, 0]\n"
                        "    def finish(self, reason):\n        raise ValueError('boom')\n")
        self.assertEqual(session.call_finish(drv, "x"),
                         ["控制算法的 finish() 出错：ValueError: boom（fin_ctrl.py 第 5 行）"])
        drv = self.load("import sys\nclass Controller:\n"
                        "    def control(self, e, t, dt):\n        return [0, 0, 0]\n"
                        "    def finish(self, reason):\n        sys.exit(3)\n")
        self.assertIn("SystemExit", session.call_finish(drv, "x")[0])

    def test_stop_calls_finish_before_the_record_closes(self):
        d = settings.default_dict()
        d["sync"]["duration"] = 2.0
        ses = session.CoSimSession(None, None, None, d)
        ses.record_dir = self.tmp.name
        ses.recorder = scn.Recorder(scn.Recorder.run_paths(os.path.join(self.tmp.name, "log.csv")), d["scene"], [])
        ses.kpi = scn.RunKpi(0.02)
        ses.n_frames, ses.frame = 100, 100
        seen = []
        ses.driver = SimpleNamespace(path=os.path.join(self.tmp.name, "c.py"),
                                     finish=lambda reason: seen.append((reason, bool(ses.recorder.files))))
        summary = ses.stop(release_vehicle=False, end="finished")
        self.assertEqual(seen, [("达到设定的运行时长 2 s", True)])  # the reason of a run that reached its duration
        self.assertIsNone(ses.recorder)
        run = read_json(os.path.join(self.tmp.name, "run.json"))
        self.assertEqual((run["end"], run["end_reason"], run["kpi"]), ("finished", "达到设定的运行时长 2 s", None))
        self.assertEqual(summary, {"record_dir": self.tmp.name, "errors": []})

    def test_carla_dynamics_stop_completes_run_json_with_carla_gone(self):
        def gone(*a):
            raise RuntimeError("time-out of 500ms while waiting for the simulator")
        d = settings.default_dict()
        d["drive"]["dynamics"] = "carla"
        ses = session.CarlaDriveSession(SimpleNamespace(apply_settings=gone), None, d, None)
        ses._original_settings = object()
        ses.scene = SimpleNamespace(stop=gone)
        ses.record_dir = self.tmp.name
        ses.recorder = scn.Recorder(scn.Recorder.run_paths(os.path.join(self.tmp.name, "log.csv")), d["scene"], [])
        ses.kpi = scn.RunKpi(0.05)
        ses.kpi.step(0.0, [])
        ses.kpi.sample({"t": 0.0, "ego": {"X": 0.0, "Y": 0.0, "width": 2.0}, "objects": []}, {})
        with contextlib.redirect_stdout(io.StringIO()):
            summary = ses.stop(end="error", reason="CARLA 服务器已退出或连不上（localhost:2000）")
        self.assertIsNone(ses.recorder)
        self.assertIsNone(ses._original_settings)
        run = read_json(os.path.join(self.tmp.name, "run.json"))
        self.assertEqual((run["end"], run["end_reason"], run["dynamics"]),
                         ("error", "CARLA 服务器已退出或连不上（localhost:2000）", "carla"))
        self.assertEqual(run["kpi"]["samples"], 1)
        self.assertEqual((summary["record_dir"], summary["errors"], summary["kpi"]),
                         (self.tmp.name, [], run["kpi"]))
        self.assertIn("行驶距离", summary["kpi_text"])

    def test_end_words(self):
        d = settings.default_dict()
        ses = SimpleNamespace(d=d, end_reason="", n_frames=0, frame=10)
        self.assertEqual(session.end_words(ses, "finished"), "CarSim 到达 .sim 里设定的结束时间")
        self.assertEqual(session.end_words(ses, "stopped"), "运行被停止")
        self.assertEqual(session.end_words(ses, "error"), "运行出错")
        ses.end_reason = "碰撞：撞到 a（id 1）"
        self.assertEqual(session.end_words(ses, "finished"), "碰撞：撞到 a（id 1）")


class BackendTests(unittest.TestCase):
    def backend(self):
        b = Backend()
        self.events = []
        b.emit = self.events.append
        b.world = SimpleNamespace(get_settings=lambda: SimpleNamespace(synchronous_mode=False),
                                  apply_settings=lambda s: None)
        return b

    def test_end_of_run_shows_folder_figures_and_finish_errors(self):
        b = self.backend()
        got = []
        b.session = SimpleNamespace(stop=lambda **kw: got.append(kw) or {
            "record_dir": "/data/runs/20260927_101500_my_ctrl", "kpi_text": "行驶距离 12.0 m",
            "errors": ["控制算法的 finish() 出错：ValueError: boom（c.py 第 5 行）"]})
        b.cosim_state = "running"
        b._stop_cosim_if_running("finished", "达到设定的运行时长 20 s")
        self.assertEqual(got, [{"release_vehicle": True, "end": "finished", "reason": "达到设定的运行时长 20 s"}])
        logs = [(e["level"], e["msg"]) for e in self.events if e.get("event") == "log"]
        self.assertEqual(logs, [("warn", "控制算法的 finish() 出错：ValueError: boom（c.py 第 5 行）"),
                                ("info", "运行记录：/data/runs/20260927_101500_my_ctrl"),
                                ("info", "运行指标：行驶距离 12.0 m")])
        self.assertEqual(self.events[-1], {"event": "cosim_state", "state": "finished", "detail": "达到设定的运行时长 20 s"})

    def test_the_heartbeat_sees_the_session_in_stop(self):
        b = self.backend()
        seen = []
        ses = SimpleNamespace(stop=lambda **kw: seen.append((b.session, b.ending)))
        b.session, b.cosim_state = ses, "running"
        b._stop_cosim_if_running("stopped")
        self.assertEqual(seen, [(None, ses)])  # finish() runs in stop(): named as the algorithm's, not CARLA
        self.assertIsNone(b.ending)

    def test_stop_button(self):
        b = self.backend()
        got = []
        b.session = SimpleNamespace(stop=lambda **kw: got.append(kw))
        b.cosim_state = "running"
        b.cmd_cosim_stop()
        self.assertEqual(got, [{"release_vehicle": True, "end": "stopped", "reason": ""}])

    def test_traffic_seed_for_run_json(self):
        seen = []

        class Session:
            scene, done = None, False

            def __init__(self, *a):
                pass

            def start(self):
                seen.append(self.run_meta)
                return {"external_api": False, "server_api": None, "reference_point": [0, 0, 0], "t_step": 0.001,
                        "inner_steps": 20, "frame_dt": 0.02, "t_stop": 0.0, "mock": True, "warnings": [],
                        "t": 0.0, "collisions": [], "record_dir": "/data/runs/x"}

        settings_obj = SimpleNamespace(synchronous_mode=False, fixed_delta_seconds=None)
        for traffic, want in (({"vehicles": [1], "walkers": [], "controllers": []}, 7),
                              ({"vehicles": [], "walkers": [], "controllers": []}, None)):
            b = self.backend()
            b.world = SimpleNamespace(get_settings=lambda: settings_obj, apply_settings=lambda s: None,
                                      tick=lambda: 0, reset_all_traffic_lights=lambda: None)
            b.cmd_spawn_ego = lambda *a, b=b: setattr(b, "ego", SimpleNamespace(
                type_id="vehicle.test", attributes={}, is_alive=True))
            b.traffic, b.traffic_seed = traffic, 7
            with mock.patch("backend_server.CoSimSession", Session), \
                    mock.patch("backend_server.rigmod.spec_of", lambda v: {}):
                info = b.cmd_cosim_start({"carsim": {"mock": True}, "run": {"driver": "demo"}})
            self.assertEqual(seen[-1], {"traffic_seed": want})
            self.assertEqual(info["record_dir"], "/data/runs/x")  # for the 驾驶模式 page


class CommandLineTests(unittest.TestCase):
    def run_cli(self, step):
        original = SimpleNamespace(synchronous_mode=False, fixed_delta_seconds=None)
        vehicle = mock.Mock()
        world = SimpleNamespace(get_settings=lambda: copy.deepcopy(original), apply_settings=lambda s: None,
                                get_map=lambda: SimpleNamespace(get_spawn_points=lambda: ["anchor"]),
                                get_blueprint_library=lambda: SimpleNamespace(find=lambda name: name),
                                spawn_actor=lambda bp, anchor: vehicle, tick=lambda: None)
        stops = []

        class Session:
            def __init__(self, *args):
                self.done = False

            def start(self):
                return {"external_api": False, "reference_point": [0, 0, 0], "t_step": 0.001, "inner_steps": 20,
                        "frame_dt": 0.02, "t_stop": 0.0, "mock": True, "warnings": []}

            def step(self):
                self.done = True
                return step()

            def stop(self, release_vehicle, **end):
                stops.append(end)
                return {"record_dir": "/data/runs/20260927_101500_demo", "kpi_text": "行驶距离 1.0 m", "errors": []}

        client = SimpleNamespace(set_timeout=lambda timeout: None, get_world=lambda: world)
        out = io.StringIO()
        with mock.patch.object(run_cosim.carla, "Client", return_value=client), \
                mock.patch.object(run_cosim, "CoSimSession", Session), \
                mock.patch.object(sys, "argv", ["run_cosim.py", "--mock", "--duration", "0.02", "--driver", "demo"]), \
                contextlib.redirect_stdout(out):
            try:
                run_cosim.main()
            except RuntimeError:
                pass
        return stops, out.getvalue()

    def test_finished_run(self):
        stops, out = self.run_cli(lambda: {"t": 0.02, "rt_factor": 1.0})
        self.assertEqual(stops, [{"end": "finished", "reason": ""}])  # the session words the reason
        self.assertIn("run record: /data/runs/20260927_101500_demo", out)
        self.assertIn("KPI: 行驶距离 1.0 m", out)

    def test_error_run(self):
        def boom():
            raise RuntimeError("CarSim 报错")
        stops, _ = self.run_cli(boom)
        self.assertEqual(stops, [{"end": "error", "reason": "CarSim 报错"}])


if __name__ == "__main__":
    unittest.main()
