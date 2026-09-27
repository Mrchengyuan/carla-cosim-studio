"""Run record and data collection timeline / content, without a CARLA server:
samples by run step from step 0 (t0 row), the first scene has a frame and
sensor data, a contact at the start is new once, control outputs u1..un,
IMU in CarSim axes in frames/, the map's parked cars in labels/, CarSim's
velocity in ego/ (stock CARLA reads 0 for the teleported car)."""

import csv
import json
import math
import os
import sys
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import carla  # noqa: E402

import backend_server  # noqa: E402
import scene as scn  # noqa: E402
import session  # noqa: E402
import settings  # noqa: E402
from backend_server import Backend  # noqa: E402
from collector import DataCollector  # noqa: E402

CAM = {"name": "cam", "type": "rgb", "x": 1.5, "y": 0.0, "z": 1.6, "roll": 0, "pitch": 0, "yaw": 0,
       "attributes": {"image_size_x": 8, "image_size_y": 4, "fov": 90}, "enabled": True}


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.reader(f))


class FakeSceneProvider:
    """Stands in for scene.SceneProvider: remembers the order of calls."""

    def __init__(self, world, ego, d, anchor=None, ref_local=None, sensor_cfgs=()):
        self.world, self.s, self.sensor_cfgs = world, d["scene"], list(sensor_cfgs)
        self.frame, self._ego_box = None, (0.0, 0.0)
        world.log.append("SceneProvider")

    def start(self):
        self.world.log.append("start")

    def stop(self):
        pass

    def update(self, frame, t=0.0, ego_velocity=None):
        self.world.log.append(("update", frame))
        self.frame = frame
        self.latest = {"t": t, "frame": frame, "ego": {"X": 10.0 * t}, "objects": [], "collisions": []}
        return self.latest

    def record_view(self):
        return self.latest


class FakeWorld:
    def __init__(self, frame0):
        self.log, self.frame = [], frame0 - 1

    def tick(self):
        self.frame += 1
        self.log.append(("tick", self.frame))
        return self.frame


class RunRecordTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def config(self):
        d = settings.default_dict()
        d["sync"]["frame_dt"] = 0.02
        d["collect"]["sample_period"] = 0.1          # every 5 steps
        d["run"]["log_path"] = os.path.join(self.tmp.name, "run.csv")
        d["rig"]["sensors"] = [dict(CAM)]
        d["scene"]["sensors"] = ["cam"]
        d["scene"].update({"record": {"ego": ["X"], "objects": [], "lane": []}, "exports_all": False, "exports": ["Xo"]})
        return d

    def test_samples_by_run_step_from_t0_with_control_outputs(self):
        # CARLA's world frame at the run start is arbitrary (1003 = 3 mod 5):
        # the samples must still be t = 0, 0.1, 0.2 (run steps 0, 5, 10).
        world = FakeWorld(1003)
        ses = SimpleNamespace(d=self.config(), world=world, vehicle=None, frame=0, recorder=None, scene=None,
                              end_reason="", last_action=None, exports=lambda: {"Xo": 1.5, "Vx": 3.0},
                              export_names=lambda: ["Xo", "Vx"], n_actions=lambda: 3)
        with mock.patch.object(session, "SceneProvider", FakeSceneProvider):
            tel0 = session.start_scene(ses, None, [1.4, 0.0, 0.0], 0.0)
        # Sensors first, then the tick: the first scene has a frame (and that frame's data).
        self.assertEqual(world.log, ["SceneProvider", "start", ("tick", 1003), ("update", 1003)])
        self.assertEqual(tel0["collisions"], [])
        for k in range(1, 13):
            ses.last_action = [0.1 * k, 0.0, -30.0]  # what control() returned in step k
            ses.frame = k
            session.scene_step(ses, world.tick(), 0.02 * k)
        ses.recorder.close()
        rows = read_csv(os.path.join(self.tmp.name, "run.csv"))
        self.assertEqual(rows[0], ["t", "frame", "ego_X", "Xo", "u1", "u2", "u3"])
        self.assertEqual([r[0] for r in rows[1:]], ["0.0", "0.1", "0.2"])
        self.assertEqual([r[1] for r in rows[1:]], ["1003", "1008", "1013"])
        self.assertEqual(rows[1][4:], ["", "", ""])     # t0: control() not called yet
        self.assertEqual(rows[2][4:], ["0.5", "0.0", "-30.0"])

    def test_no_control_columns_with_carla_dynamics(self):
        path = os.path.join(self.tmp.name, "r.csv")
        rec = scn.Recorder(scn.Recorder.run_paths(path), self.config()["scene"], [])
        rec.write({"t": 0.0, "frame": 7, "ego": {"X": 1.0}, "objects": []}, {}, None)
        rec.close()
        self.assertEqual(read_csv(path), [["t", "frame", "ego_X"], ["0.0", "7", "1.0"]])


class SceneStartTests(unittest.TestCase):
    """A parked car (map object) overlaps the ego at the start."""

    def provider(self):
        tf = carla.Transform(carla.Location(0.0, 0.0, 0.0), carla.Rotation())
        ego = SimpleNamespace(id=1, get_transform=lambda: tf, get_velocity=lambda: carla.Vector3D(),
                              bounding_box=carla.BoundingBox(carla.Location(0.0, 0.0, 0.7), carla.Vector3D(2.3, 1.0, 0.7)))
        parked = SimpleNamespace(id=99, bounding_box=SimpleNamespace(
            location=carla.Location(3.0, 0.0, 0.7), rotation=carla.Rotation(), extent=carla.Vector3D(2.0, 0.9, 0.7)))

        class Snap(list):  # no other actors
            def find(self, i):
                return ego if i == 1 else None

        world = SimpleNamespace(
            get_map=lambda: SimpleNamespace(get_waypoint=lambda *a, **k: None),
            get_blueprint_library=lambda: None,
            get_snapshot=lambda: Snap(),
            get_environment_objects=lambda label: [parked] if label == carla.CityObjectLabel.Car else [])
        d = settings.default_dict()
        d["scene"]["lane"], d["scene"]["record"]["lane"] = [], []
        return scn.SceneProvider(world, ego, d, tf, [1.4, 0.0, 0.0], [])

    def test_contact_at_start_is_new_once_and_first_scene_has_a_frame(self):
        sp = self.provider()
        sp.start()
        self.assertIsNone(sp.latest)  # no scene before the first tick
        first = sp.update(1003, 0.0)
        self.assertEqual(first["frame"], 1003)
        self.assertEqual([(c["id"], c["new"]) for c in first["collisions"]], [(99, True)])
        self.assertEqual([(c["id"], c["new"]) for c in sp.update(1004, 0.02)["collisions"]], [(99, False)])

    def test_session_logs_a_start_contact_once(self):
        world = FakeWorld(500)
        d = settings.default_dict()
        d["run"]["log_path"] = ""
        d["rig"]["sensors"] = [dict(CAM)]

        class Touching(FakeSceneProvider):
            def update(self, frame, t=0.0, ego_velocity=None):
                sc = super().update(frame, t, ego_velocity)
                sc["collisions"] = [{"id": 99, "type": "vehicle", "model": "map.Car", "new": frame == 500}]
                return sc

        d["scene"]["collision"] = "stop"
        ses = SimpleNamespace(d=d, world=world, vehicle=None, frame=0, recorder=None, scene=None, end_reason="",
                              last_action=None, exports=dict, export_names=list, n_actions=lambda: 0)
        with mock.patch.object(session, "SceneProvider", Touching):
            tel0 = session.start_scene(ses, None, [1.4, 0.0, 0.0], 0.0)
            self.assertEqual([c["id"] for c in tel0["collisions"]], [99])
            self.assertIn("map.Car", ses.end_reason)    # "stop" acts on the start contact
            ses.frame = 1
            self.assertEqual(session.scene_step(ses, world.tick(), 0.02)["collisions"], [])


class CollectorTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def collector(self, sensors, **kw):
        cfg = {"frame_dt": 0.02, "capture_every": 5, "max_frames": 0, "max_gb": 0, "labels": False,
               "units": {"speed": "km/h", "angle": "deg", "rate": "deg/s"}}
        c = DataCollector(None, None, sensors, cfg, **kw)
        root = self.tmp.name
        for sub in ["ego", "frames", "labels"] + [s["name"] for s in sensors]:
            os.makedirs(os.path.join(root, sub), exist_ok=True)
        c.root, c._t0, c._last_emit = root, time.time(), 0.0
        return c

    def test_samples_by_step_with_action_and_imu_in_carsim_frame(self):
        last = {"a": None}
        imu = {"name": "imu", "type": "imu"}
        c = self.collector([imu], action=lambda: last["a"], n_actions=2)
        view = {}
        c.scene = SimpleNamespace(record_view=lambda: view)
        c.exports = lambda: {"Xo": 2.0}
        c.recorder = scn.Recorder({"main": os.path.join(c.root, "frames.csv"), "objects": os.path.join(c.root, "objects.csv"),
                                   "lane": os.path.join(c.root, "lane.csv")},
                                  {"record": {"ego": ["X"], "objects": [], "lane": []}}, ["Xo"], c.n_actions)
        c._ego_state = lambda frame: {"frame": frame}
        c.shared = SimpleNamespace(frame=None, datas=[])
        c.writer = threading.Thread(target=c._write_loop, daemon=True)
        c.writer.start()
        meas = lambda f: SimpleNamespace(frame=f, accelerometer=SimpleNamespace(x=1.0, y=2.0, z=9.8),
                                         gyroscope=SimpleNamespace(x=0.1, y=0.2, z=0.3), compass=1.0)
        for step in range(11):
            frame = 2002 + step          # world frames at an arbitrary offset
            view.clear()
            view.update({"t": 0.02 * step, "frame": frame, "ego": {"X": float(step)}, "objects": []})
            c.shared.frame, c.shared.datas = frame, [meas(frame)]
            c.on_tick(frame, step)
            last["a"] = [0.5, float(step)]  # control() output of the next step
        c.stop()
        names = sorted(os.listdir(os.path.join(c.root, "frames")))
        self.assertEqual(names, ["002002.json", "002007.json", "002012.json"])  # steps 0, 5, 10
        with open(os.path.join(c.root, "frames", "002002.json")) as f:
            r0 = json.load(f)
        with open(os.path.join(c.root, "frames", "002007.json")) as f:
            r5 = json.load(f)
        self.assertIsNone(r0["action"])
        self.assertEqual(r5["action"], [0.5, 4.0])   # held over step 4 -> 5
        got = r5["sensors"]["imu"]
        self.assertEqual(got["type"], "imu")
        self.assertEqual(got["data"]["accel"], [1.0, -2.0, 9.8])  # y left
        for v, want in zip(got["data"]["gyro"], (-math.degrees(0.1), math.degrees(0.2), -math.degrees(0.3))):
            self.assertAlmostEqual(v, want)
        self.assertAlmostEqual(got["data"]["compass"], math.degrees(1.0))
        with open(os.path.join(c.root, "ego", "002007.json")) as f:
            self.assertEqual(json.load(f)["imu"]["accelerometer"], [1.0, 2.0, 9.8])  # CARLA's raw values stay in ego/
        rows = read_csv(os.path.join(c.root, "frames.csv"))
        self.assertEqual(rows[0], ["t", "frame", "ego_X", "Xo", "u1", "u2"])
        self.assertEqual([r[0] for r in rows[1:]], ["0.0", "0.1", "0.2"])
        self.assertEqual(rows[1][4:], ["", ""])
        self.assertEqual(rows[2][4:], ["0.5", "4.0"])

    def test_labels_include_map_parked_cars(self):
        ego_tf = carla.Transform(carla.Location(10.0, 20.0, 0.0), carla.Rotation(yaw=90.0))
        c = DataCollector(SimpleNamespace(get_actors=lambda: []), SimpleNamespace(id=1, get_transform=lambda: ego_tf),
                          [], {"frame_dt": 0.1, "label_radius": 80.0})
        c._map_cars = [(777, "Car", (10.0, 30.0, 0.8), 90.0, (2.2, 0.9, 0.7)),   # 10 m ahead
                       (778, "Truck", (500.0, 0.0, 0.0), 0.0, (5.0, 1.5, 2.0)),  # out of range
                       (779, "Bicycle", (12.0, 25.0, 0.5), 0.0, (0.8, 0.3, 0.6)),  # parked bikes: no rider,
                       (780, "Motorcycle", (8.0, 25.0, 0.6), 0.0, (1.0, 0.4, 0.7))]  # not labelled
        objs = c._labels()["objects"]
        self.assertEqual(len(objs), 1)
        o = objs[0]
        self.assertEqual((o["id"], o["type_id"], o["class"], o["velocity"]), (777, "map.Car", "car", [0.0, 0.0, 0.0]))
        for v, want in zip(o["center_ego"], (10.0, 0.0, 0.8)):
            self.assertAlmostEqual(v, want, places=4)
        self.assertAlmostEqual(o["yaw_ego"], 0.0)
        self.assertEqual(o["extent"], [2.2, 0.9, 0.7])

    def test_map_vehicles(self):
        box = SimpleNamespace(location=carla.Location(1.0, 2.0, 0.5), rotation=carla.Rotation(yaw=30.0),
                              extent=carla.Vector3D(2.0, 1.0, 0.75))
        objs = {carla.CityObjectLabel.Car: [SimpleNamespace(id=5, bounding_box=box)],
                carla.CityObjectLabel.Bus: [SimpleNamespace(id=6, bounding_box=box)]}
        world = SimpleNamespace(get_environment_objects=lambda label: objs.get(label, []))
        got = scn.map_vehicles(world)
        self.assertEqual([(g[0], g[1]) for g in got], [(5, "Car"), (6, "Bus")])
        self.assertEqual(got[0][2:], ((1.0, 2.0, 0.5), 30.0, (2.0, 1.0, 0.75)))

    def test_ego_state_takes_carsim_velocity(self):
        ses = session.CoSimSession(None, None, None, {})
        ses.state = SimpleNamespace(velocity=carla.Vector3D(10.0, -2.0, 0.0), angular_velocity=carla.Vector3D(0.0, 0.0, 5.0))
        ego = SimpleNamespace(get_transform=lambda: carla.Transform(), get_velocity=lambda: carla.Vector3D(),
                              get_angular_velocity=lambda: carla.Vector3D(), get_acceleration=lambda: carla.Vector3D(),
                              get_control=lambda: SimpleNamespace(throttle=0.3, steer=0.0, brake=0.0, gear=1))
        world = SimpleNamespace(get_snapshot=lambda: SimpleNamespace(timestamp=SimpleNamespace(elapsed_seconds=1.0)))
        st = DataCollector(world, ego, [], {"frame_dt": 0.1}, extra_state=ses.ego_motion)._ego_state(7)
        self.assertEqual(st["velocity"], [10.0, -2.0, 0.0])  # what the nuScenes radar compensation reads
        self.assertEqual(st["angular_velocity"], [0.0, 0.0, 5.0])


class BackendWiringTests(unittest.TestCase):
    def test_collection_gets_carsim_velocity_actions_and_the_t0_sample(self):
        made = []

        class FakeCollector:
            def __init__(self, world, ego, sensors, cfg, **kw):
                self.kw, self.ticks, self.sensor_cfgs = kw, [], sensors
                self.done, self.frames, self.bytes, self.stop_reason = False, 0, 0, ""
                made.append(self)

            def estimate(self):
                return {"total_gb": 0.001, "mb_per_s": 1.0}

            def start(self):
                return {"root": "x", "estimate": self.estimate()}

            def on_tick(self, frame, step=None):
                self.ticks.append((frame, step))

        class FakeSession:
            done = False

            def __init__(self, world, ego, anchor, d):
                self.scene = SimpleNamespace(frame=None, sensor_cfgs=[dict(CAM)], ref_local=[1.4, 0.0, 0.0])
                self.last_action = None
                made.append(self)

            def start(self):
                self.scene.frame = 4321   # the tick inside start_scene
                return {"external_api": False, "server_api": None, "reference_point": [1.4, 0, 0], "t_step": 0.001,
                        "inner_steps": 20, "frame_dt": 0.02, "t_stop": 0.0, "mock": True, "warnings": [], "t": 0.0,
                        "collisions": [{"id": 5, "type": "vehicle", "model": "map.Car", "new": True}]}

            def ego_motion(self):
                return {"velocity": [1.0, 0.0, 0.0]}

            def exports(self):
                return {}

            def export_names(self):
                return ["Xo"]

            def n_actions(self):
                return 3

        settings_obj = SimpleNamespace(synchronous_mode=False, fixed_delta_seconds=None)
        world = SimpleNamespace(get_settings=lambda: settings_obj, apply_settings=lambda s: None, tick=lambda: 0,
                                reset_all_traffic_lights=lambda: None)
        backend = Backend()
        events = []
        backend.emit = events.append
        backend.world = world
        ego = SimpleNamespace(type_id="vehicle.test", attributes={}, is_alive=True)
        backend.cmd_spawn_ego = lambda *a: setattr(backend, "ego", ego)
        cfg = {"collect": {"enabled": True}, "rig": {"sensors": [dict(CAM)]},
               "carsim": {"mock": True}, "run": {"driver": "demo"}}  # passes the pre-flight
        with mock.patch.object(backend_server.coll, "DataCollector", FakeCollector), \
                mock.patch.object(backend_server.coll, "disk_info", lambda p: {"path": p, "free_gb": 1e3, "total_gb": 1e3}), \
                mock.patch.object(backend_server.rigmod, "spec_of", lambda v: {}), \
                mock.patch.object(backend_server, "CoSimSession", FakeSession):
            backend.cmd_cosim_start(cfg)
        ses, col = made[-2], made[-1]
        self.assertIsInstance(ses, FakeSession)
        self.assertEqual(col.kw["extra_state"](), {"velocity": [1.0, 0.0, 0.0]})
        self.assertIsNone(col.kw["action"]())
        ses.last_action = [0.2, 0.0, 15.0]
        self.assertEqual(col.kw["action"](), [0.2, 0.0, 15.0])
        self.assertEqual(col.kw["n_actions"], 3)
        self.assertEqual(col.ticks, [(4321, 0)])  # step 0 sampled right after the start
        hits = [e["msg"] for e in events if e.get("event") == "log" and "碰撞" in e["msg"]]
        self.assertEqual(hits, ["碰撞：撞到 map.Car（id 5），t = 0.00 s"])


class StartEndsRunTests(unittest.TestCase):
    """Step 0 inside cosim_start can already end the run: no step past it."""

    def start(self, end_reason="", frames_limit_hit=False):
        steps = []

        class Collector:
            def __init__(self, *a, **k):
                self.sensor_cfgs, self.done, self.frames, self.bytes, self.stop_reason = [], False, 0, 0, ""

            def estimate(self):
                return {"total_gb": 0.001, "mb_per_s": 1.0}

            def start(self):
                return {"root": "x", "estimate": self.estimate()}

            def on_tick(self, frame, step=None):
                self.frames += 1
                if frames_limit_hit:  # collect.max_frames = 1
                    self.done, self.stop_reason = True, "达到帧数上限"

            def stop(self):
                pass

        class Session:
            def __init__(self, world, ego, anchor, d):
                self.scene = SimpleNamespace(frame=7, sensor_cfgs=[], ref_local=[0.0, 0.0, 0.0])
                self.end_reason, self.done, self.last_action = "", False, None

            def start(self):
                self.end_reason = end_reason
                self.done = bool(end_reason)
                return {"external_api": False, "server_api": None, "reference_point": [0, 0, 0], "t_step": 0.001,
                        "inner_steps": 20, "frame_dt": 0.02, "t_stop": 0.0, "mock": True, "warnings": [], "t": 0.0,
                        "collisions": []}

            def step(self):
                steps.append(1)

            def stop(self, release_vehicle=True):
                pass

            def ego_motion(self):
                return {}

            def exports(self):
                return {}

            def export_names(self):
                return []

            def n_actions(self):
                return 3

        settings_obj = SimpleNamespace(synchronous_mode=False, fixed_delta_seconds=None)
        world = SimpleNamespace(get_settings=lambda: settings_obj, apply_settings=lambda s: None, tick=lambda: 0,
                                reset_all_traffic_lights=lambda: None)
        backend = Backend()
        events = []
        backend.emit = events.append
        backend.world = world
        backend.cmd_spawn_ego = lambda *a: setattr(backend, "ego", SimpleNamespace(
            type_id="vehicle.test", attributes={}, is_alive=True))
        cfg = {"collect": {"enabled": True, "max_frames": 1}, "rig": {"sensors": [dict(CAM)]},
               "carsim": {"mock": True}, "run": {"driver": "demo"}}
        with mock.patch.object(backend_server.coll, "DataCollector", Collector), \
                mock.patch.object(backend_server.coll, "disk_info", lambda p: {"path": p, "free_gb": 1e3, "total_gb": 1e3}), \
                mock.patch.object(backend_server.rigmod, "spec_of", lambda v: {}), \
                mock.patch.object(backend_server, "CoSimSession", Session):
            backend.cmd_cosim_start(cfg)
        states = [(e["state"], e["detail"]) for e in events if e.get("event") == "cosim_state"]
        return backend, states, steps

    def test_collision_stop_at_t0(self):
        backend, states, steps = self.start(end_reason="碰撞：撞到 map.Car（id 99）")
        self.assertEqual(states, [("finished", "碰撞：撞到 map.Car（id 99）")])  # never "running"
        self.assertEqual((steps, backend.session, backend.collector), ([], None, None))

    def test_collection_limit_reached_at_t0(self):
        backend, states, steps = self.start(frames_limit_hit=True)
        self.assertEqual(states, [("finished", "数据采集达到帧数上限")])
        self.assertEqual((steps, backend.session), ([], None))

    def test_a_normal_start_runs(self):
        backend, states, steps = self.start()
        self.assertEqual(states, [("running", "")])
        self.assertIsNotNone(backend.session)


if __name__ == "__main__":
    unittest.main()
