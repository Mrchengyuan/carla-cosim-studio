"""Regression checks for collection and startup paths that need no CARLA server."""

import copy
import json
import os
import sys
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from backend_server import Backend  # noqa: E402
from collector import DataCollector  # noqa: E402
import run_cosim  # noqa: E402


class CollectorTests(unittest.TestCase):
    def config(self, **overrides):
        return {"frame_dt": 0.1, "capture_every": 1, "image_format": "jpg",
                "max_frames": 1, "max_seconds": 0, "max_gb": 0, **overrides}

    def test_rejects_names_that_overwrite_or_escape(self):
        sensor = lambda name: {"name": name, "type": "rgb", "attributes": {"image_size_x": 1, "image_size_y": 1}}
        for names, session in ((["cam", "cam"], ""), (["cam", "CAM"], ""),
                               (["../outside"], ""), (["/tmp/outside"], ""),
                               (["calib.json"], ""),
                               (["cam"], "../outside"), (["cam"], "C:\\outside")):
            with self.subTest(names=names, session=session):
                c = DataCollector(None, None, [sensor(n) for n in names], self.config(session=session))
                with self.assertRaises(ValueError):
                    c.estimate()
        DataCollector(None, None, [sensor("前视相机")], self.config(session="场景一")).estimate()

    def test_gb_limit_counts_finished_write_before_next_frame(self):
        with tempfile.TemporaryDirectory() as root:
            os.mkdir(os.path.join(root, "ego"))
            c = DataCollector(None, None, [], self.config(max_frames=0, max_gb=1e-9, labels=False))
            c.root = root
            c._t0 = time.time()
            c._last_emit = 0.0
            c._ego_state = lambda frame: {"frame": frame}
            writing = threading.Event()
            release = threading.Event()

            def delayed_writer():
                writing.set()
                release.wait()
                c._write_loop()

            c.writer = threading.Thread(target=delayed_writer, daemon=True)
            c.writer.start()
            tick = threading.Thread(target=c.on_tick, args=(2,), daemon=True)
            try:
                self.assertTrue(writing.wait(1))
                c.on_tick(1)
                tick.start()
                tick.join(0.05)
                self.assertTrue(tick.is_alive(), "accepted another frame before the queued frame was written")
                release.set()
                tick.join(2)
                self.assertFalse(tick.is_alive())
                self.assertTrue(c.done)
                self.assertEqual(c.frames, 1)
                self.assertGreater(c.bytes, 1)
                self.assertEqual(c.q.unfinished_tasks, 0)
            finally:
                release.set()
                c.stop()

    def test_far_from_gb_limit_does_not_wait_for_writer(self):
        with tempfile.TemporaryDirectory() as root:
            os.mkdir(os.path.join(root, "ego"))
            c = DataCollector(None, None, [], self.config(max_frames=0, max_gb=10, labels=False))
            c.root = root
            c._t0 = time.time()
            c._last_emit = 0.0
            c._ego_state = lambda frame: {"frame": frame}
            release = threading.Event()

            def delayed_writer():
                release.wait()
                c._write_loop()

            c.writer = threading.Thread(target=delayed_writer, daemon=True)
            c.writer.start()
            tick = threading.Thread(target=lambda: (c.on_tick(1), c.on_tick(2)), daemon=True)
            try:
                tick.start()
                tick.join(1)
                self.assertFalse(tick.is_alive(), "waited for the writer far from the limit")
                self.assertEqual(c.frames, 2)
            finally:
                release.set()
                c.stop()


class BackendStartupTests(unittest.TestCase):
    def test_invalid_collection_names_do_not_respawn_ego(self):
        backend = Backend()
        backend.world = object()
        spawned = []
        backend.cmd_spawn_ego = lambda *args: spawned.append(args)
        cfg = {"collect": {"enabled": True}, "rig": {"sensors": [
            {"name": "cam", "type": "rgb", "attributes": {}},
            {"name": "cam", "type": "rgb", "attributes": {}}]}}
        with self.assertRaisesRegex(ValueError, "重复"):
            backend.cmd_cosim_start(cfg)
        self.assertEqual(spawned, [])

    def test_restores_previous_ego_if_reattaching_sensor_fails(self):
        backend = Backend()
        backend.world = object()
        old = SimpleNamespace(type_id="vehicle.test", attributes={"color": "red"},
                              get_transform=lambda: "old transform")
        backend.ego = old
        backend.anchor = "old anchor"
        backend._alive = lambda actor: actor is not None
        backend.sensors = {1: {"spec": {"type": "rgb"}}}
        backend.views.specs = lambda: [{"id": "p0"}]
        backend.cmd_spawn_ego = lambda *args: setattr(backend, "ego", object())

        def fail_sensor(**kwargs):
            raise RuntimeError("sensor failed")

        backend.cmd_add_sensor = fail_sensor
        restored = []

        def restore(*args):
            restored.append(args)
            backend.ego = old

        backend._restore_ego = restore
        with self.assertRaisesRegex(RuntimeError, "sensor failed"):
            backend.cmd_cosim_start()
        self.assertIs(backend.ego, old)
        self.assertEqual(restored[0][0], ("vehicle.test", "old transform", "old anchor"))
        self.assertEqual(restored[0][2], [{"type": "rgb"}])
        self.assertEqual(restored[0][3], [{"id": "p0"}])


class CliTests(unittest.TestCase):
    def test_restores_original_world_settings(self):
        original = SimpleNamespace(synchronous_mode=True, fixed_delta_seconds=0.07, no_rendering_mode=True)
        vehicle = SimpleNamespace(destroy=mock.Mock())

        class World:
            def __init__(self):
                self.settings = copy.deepcopy(original)

            def get_settings(self):
                return copy.deepcopy(self.settings)

            def apply_settings(self, settings):
                self.settings = copy.deepcopy(settings)

            def get_map(self):
                return SimpleNamespace(get_spawn_points=lambda: ["anchor"])

            def get_blueprint_library(self):
                return SimpleNamespace(find=lambda name: name)

            def spawn_actor(self, blueprint, anchor):
                return vehicle

            def tick(self):
                pass

        world = World()

        class Session:
            def __init__(self, *args):
                self.done = False

            def start(self):
                return {"external_api": False, "reference_point": [0, 0, 0],
                        "clock_warning": False, "t_step": 0.001, "inner_steps": 20}

            def step(self):
                self.done = True
                return {"t": 0.02, "rt_factor": 1.0}

            def stop(self, release_vehicle):
                pass

        client = SimpleNamespace(set_timeout=lambda timeout: None, get_world=lambda: world)
        with mock.patch.object(run_cosim.carla, "Client", return_value=client), \
                mock.patch.object(run_cosim, "CoSimSession", Session), \
                mock.patch.object(sys, "argv", ["run_cosim.py", "--mock", "--duration", "0.02"]):
            run_cosim.main()
        self.assertEqual(world.settings.__dict__, original.__dict__)
        vehicle.destroy.assert_called_once()


class RigFrameTests(unittest.TestCase):
    def test_old_config_is_marked_carla_frame(self):
        import json
        import settings
        with tempfile.TemporaryDirectory() as root:
            path = os.path.join(root, "old.json")
            with open(path, "w") as f:
                json.dump({"rig": {"sensors": [{"name": "cam", "type": "rgb", "x": 1.5, "y": 0.2, "z": 1.6,
                                                "roll": 0, "pitch": -10, "yaw": 30}]}}, f)
            self.assertEqual(settings.load_dict(path)["rig"]["frame"], "carla")
            self.assertEqual(settings.load_dict()["rig"]["frame"], "carsim")

    def test_carsim_mount_round_trip(self):
        import rig
        ref = [1.45, 0.0, 0.02]
        carla_mount = {"name": "cam", "type": "rgb", "x": 1.5, "y": 0.3, "z": 1.6, "roll": 2.0, "pitch": -10.0, "yaw": 30.0}
        cs = rig.to_carsim(carla_mount, ref)
        # y right 0.3 -> y left -0.3; yaw 30 (right) -> -30; pitch -10 (down in CARLA) -> +10 (down in CarSim)
        self.assertAlmostEqual(cs["x"], 0.05)
        self.assertAlmostEqual(cs["y"], -0.3)
        self.assertAlmostEqual(cs["yaw"], -30.0)
        self.assertAlmostEqual(cs["pitch"], 10.0)
        tf = rig.mount_transform(cs, ref)
        self.assertAlmostEqual(tf.location.x, 1.5, places=5)
        self.assertAlmostEqual(tf.location.y, 0.3, places=5)
        self.assertAlmostEqual(tf.location.z, 1.6, places=5)
        self.assertAlmostEqual(tf.rotation.yaw, 30.0, places=4)
        self.assertAlmostEqual(tf.rotation.pitch, -10.0, places=4)
        self.assertAlmostEqual(tf.rotation.roll, 2.0, places=4)

    def test_presets_in_carsim_frame(self):
        import rig
        spec = {"length_m": 4.8, "width_m": 2.0, "height_m": 1.5, "front_axle_x_m": 1.5}
        nus = {s["name"]: s for s in rig.build_preset("nuscenes", spec)}
        self.assertGreater(nus["radar_front_left"]["y"], 0.0)            # left = +y
        self.assertGreater(nus["cam_front_left"]["yaw"], 0.0)            # looking left = +yaw
        self.assertAlmostEqual(nus["radar_front"]["x"], 2.4 - 1.5)       # bumper relative to the front axle


class RecordTests(unittest.TestCase):
    def test_event_sensor_does_not_lose_the_frame(self):
        import scene as scn
        with tempfile.TemporaryDirectory() as root:
            for sub in ("ego", "frames", "labels"):
                os.mkdir(os.path.join(root, sub))
            os.mkdir(os.path.join(root, "lane_inv"))
            cfg = {"frame_dt": 0.1, "capture_every": 2, "max_frames": 0, "max_gb": 0, "labels": False}
            c = DataCollector(None, None, [{"name": "lane_inv", "type": "lane_invasion"}], cfg)
            c.root, c._t0, c._last_emit = root, time.time(), 0.0
            c.recorder = scn.Recorder({"main": os.path.join(root, "frames.csv"), "objects": os.path.join(root, "objects.csv"),
                                       "lane": os.path.join(root, "lane.csv")},
                                      {"record": {"ego": ["X"], "objects": [], "lane": []}}, ["Xo"])
            ev = lambda f: SimpleNamespace(frame=f, crossed_lane_markings=[SimpleNamespace(type="Solid")])
            rec = {"t": 0.2, "frame": 2, "ego": {"X": 1.0}, "objects": [], "exports": {"Xo": 1.0}}
            c._ego_state = lambda frame: {"frame": frame}
            c._frame_record = lambda frame: dict(rec, frame=frame)
            c.writer = threading.Thread(target=c._write_loop, daemon=True)
            c.writer.start()
            c.shared = SimpleNamespace(frame=None, datas=[])
            for frame, events in ((1, [ev(1)]), (2, [ev(2)])):
                c.shared.frame, c.shared.datas = frame, [events]
                c.on_tick(frame)
            c.stop()  # waits for the writer, closes the CSV files
            self.assertTrue(os.path.exists(os.path.join(root, "frames", "000002.json")))
            with open(os.path.join(root, "ego", "000002.json")) as f:
                ego = json.load(f)
            self.assertEqual([e["frame"] for e in ego["events"]], [1, 2])  # frame 1's event kept
            with open(os.path.join(root, "frames.csv")) as f:
                self.assertIn("2,1.0,1.0", f.read())

    def test_sample_period_in_seconds(self):
        import settings
        d = settings.default_dict()
        d["sync"]["frame_dt"] = 0.02
        d["collect"]["sample_period"] = 0.1
        self.assertEqual(settings.sample_every(d), 5)
        d["sync"]["frame_dt"] = 0.05  # changing the step keeps the period
        self.assertEqual(settings.sample_every(d), 2)
        d["collect"]["sample_period"] = 0.0
        d["collect"]["capture_every"] = 3
        self.assertEqual(settings.sample_every(d), 3)


class ViewAndExportTests(unittest.TestCase):
    def test_tilted_sensor_points_in_live_bird_view(self):
        import carla
        import numpy as np
        from views import ViewStreamer
        m = carla.Transform(carla.Location(1.0, 0.3, 2.0), carla.Rotation(pitch=30.0, yaw=20.0, roll=10.0))
        p = carla.Location(10.0, 1.0, 0.5)
        want = m.transform(carla.Location(p.x, p.y, p.z))
        lidar = SimpleNamespace(raw_data=np.array([[p.x, p.y, p.z, 0.0]], np.float32).tobytes())
        got = ViewStreamer._points(None, {"kind": "lidar", "mount": m}, lidar)[0]
        self.assertAlmostEqual(got[0], want.x, places=4)
        self.assertAlmostEqual(got[1], want.y, places=4)
        self.assertAlmostEqual(got[2], want.z - m.location.z, places=4)  # height above the sensor
        az, alt, dep = 0.2, 0.1, 12.0
        q = carla.Location(dep * np.cos(alt) * np.cos(az), dep * np.cos(alt) * np.sin(az), dep * np.sin(alt))
        want = m.transform(carla.Location(q.x, q.y, q.z))
        radar = SimpleNamespace(raw_data=np.array([[-3.0, az, alt, dep]], np.float32).tobytes())
        got = ViewStreamer._points(None, {"kind": "radar", "mount": m}, radar)[0]
        self.assertAlmostEqual(got[0], want.x, places=3)
        self.assertAlmostEqual(got[1], want.y, places=3)
        self.assertAlmostEqual(got[2], -3.0, places=5)

    def test_kitti_stereo_skips_frames_without_the_right_image(self):
        import numpy as np
        from PIL import Image
        import dataset as dsmod
        with tempfile.TemporaryDirectory() as root:
            ses_dir = os.path.join(root, "s")
            for sub in ("ego", "labels", "cam_left", "cam_right", "lidar"):
                os.makedirs(os.path.join(ses_dir, sub))
            K = [[320.0, 0.0, 320.0], [0.0, 320.0, 180.0], [0.0, 0.0, 1.0]]
            def ext(y):
                return [[1, 0, 0, 1.5], [0, 1, 0, y], [0, 0, 1, 1.6], [0, 0, 0, 1]]
            attrs = {"image_size_x": 640, "image_size_y": 360, "fov": 90.0}
            calib = {"sensors": {
                "cam_left": {"type": "rgb", "K": K, "extrinsic_sensor_to_ego": ext(0.0), "attributes": attrs},
                "cam_right": {"type": "rgb", "K": K, "extrinsic_sensor_to_ego": ext(0.54), "attributes": attrs},  # CARLA y right
                "lidar": {"type": "lidar", "extrinsic_sensor_to_ego": ext(0.0), "attributes": {}}}}
            with open(os.path.join(ses_dir, "calib.json"), "w") as f:
                json.dump(calib, f)
            with open(os.path.join(ses_dir, "meta.json"), "w") as f:
                json.dump({"map": "Town10HD_Opt", "started": "x", "conventions": "carsim"}, f)
            img = Image.fromarray(np.zeros((360, 640, 3), np.uint8))
            for frame in (1, 2):
                with open(os.path.join(ses_dir, "ego", "%06d.json" % frame), "w") as f:
                    json.dump({"frame": frame}, f)
                img.save(os.path.join(ses_dir, "cam_left", "%06d.jpg" % frame))
                np.zeros((4, 4), np.float32).tofile(os.path.join(ses_dir, "lidar", "%06d.bin" % frame))
            img.save(os.path.join(ses_dir, "cam_right", "000001.jpg"))  # frame 2 lacks the right image
            out = os.path.join(root, "kitti")
            res = dsmod.export_kitti(dsmod.Session(ses_dir), out, camera="cam_left", lidar="lidar", min_lidar_pts=1)
            n2 = len(os.listdir(os.path.join(out, "training", "image_2")))
            n3 = len(os.listdir(os.path.join(out, "training", "image_3")))
            with open(os.path.join(out, "ImageSets", "train.txt")) as f:
                train = f.read().split()
            self.assertEqual((res["frames"], n2, n3, len(train)), (1, 1, 1, 1))
            self.assertEqual(res["stereo_right"], "cam_right")
            self.assertIn("缺图", res.get("warning", ""))


if __name__ == "__main__":
    unittest.main()
