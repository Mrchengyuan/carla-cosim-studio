"""Regression checks for sensor extrinsics that need no CARLA server:
vehicle measurement without a world snapshot, the rig mount a live view gets,
old-config detection and the mount conventions the docs give."""

import glob
import os
import re
import sys
import tempfile
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import carla  # noqa: E402
from backend_server import Backend  # noqa: E402

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")


class _Blueprint:
    def __init__(self, vid):
        self.id = vid
        self.attrs = {}

    def set_attribute(self, k, v):
        self.attrs[k] = v


class _World:
    """A world whose new actors have no snapshot yet (synchronous mode, not
    ticked): get_transform() gives zeros, as LibCarla does."""

    # Wheel centres in the CARLA vehicle frame (m): front left / right, rear left / right.
    WHEELS = [(1.35, -0.8, 0.35), (1.35, 0.8, 0.35), (-1.55, -0.8, 0.35), (-1.55, 0.8, 0.35)]

    def __init__(self):
        self.spawned = []
        self.destroyed = []

    def get_blueprint_library(self):
        return SimpleNamespace(find=_Blueprint, filter=lambda pat: [])

    def get_map(self):
        return SimpleNamespace(get_spawn_points=lambda: [carla.Transform(carla.Location(-64.6, 24.5, 0.6))])

    def get_actors(self):
        return SimpleNamespace(filter=lambda pat: [])

    def try_spawn_actor(self, bp, tf):
        at = tf.location
        wheels = [SimpleNamespace(position=carla.Location((at.x + x) * 100.0, (at.y + y) * 100.0, (at.z + z) * 100.0),
                                  radius=35.0, max_steer_angle=70.0) for x, y, z in self.WHEELS]
        actor = SimpleNamespace(
            id=len(self.spawned) + 1,
            get_physics_control=lambda: SimpleNamespace(wheels=wheels, mass=1800.0),
            set_simulate_physics=lambda on: None,
            get_transform=lambda: carla.Transform(),  # no snapshot of this actor yet
            bounding_box=SimpleNamespace(extent=carla.Vector3D(2.4, 1.0, 0.75), location=carla.Location(0.0, 0.0, 0.75)))
        actor.destroy = lambda: self.destroyed.append(actor.id)
        self.spawned.append((bp.id, tf))
        return actor


class VehicleSpecTests(unittest.TestCase):
    def test_measures_in_the_vehicle_frame_without_a_snapshot(self):
        backend = Backend()
        backend.world = _World()
        ids = ["vehicle.a", "vehicle.b"]
        specs = backend.cmd_vehicle_specs(ids)
        for vid in ids:
            with self.subTest(vid=vid):
                s = specs[vid]
                self.assertAlmostEqual(s["front_axle_x_m"], 1.35, places=3)  # not the world x of the probe
                self.assertAlmostEqual(s["wheelbase_m"], 2.9, places=3)
                self.assertAlmostEqual(s["track_m"], 1.6, places=3)
                self.assertAlmostEqual(s["front_axle_z_m"], 0.0, places=3)
        self.assertEqual(len(backend.world.destroyed), 2)
        self.assertEqual(backend.spec_cache["vehicle.b"]["front_axle_x_m"], specs["vehicle.b"]["front_axle_x_m"])

    def test_preset_built_from_the_measurement_sits_on_the_car(self):
        backend = Backend()
        backend.world = _World()
        cam = backend.cmd_rig_build("front_camera", blueprint="vehicle.a")[0]
        # Windshield camera 0.15 L ahead of the car centre, relative to the front axle.
        self.assertAlmostEqual(cam["x"], 4.8 * 0.15 - 1.35, places=2)
        self.assertAlmostEqual(cam["y"], 0.0, places=3)


class RigMountTests(unittest.TestCase):
    """The mount the GUI sends for a rig preview (App::RigMount)."""

    def test_carsim_mount_is_converted_and_a_plain_mount_is_carla(self):
        from views import _transform
        car = object()
        m = {"x": 0.5, "y": 0.8, "z": 1.5, "pitch": 5.0, "yaw": 90.0, "roll": 0.0}
        tf = _transform(dict(m, frame="carsim", ref=[1.4, 0.0, 0.0]), car)
        # CarSim: 0.8 m to the left, looking left, nose down -> CARLA: y right, yaw right-positive, pitch up-positive.
        self.assertAlmostEqual(tf.location.x, 1.9, places=5)
        self.assertAlmostEqual(tf.location.y, -0.8, places=5)
        self.assertAlmostEqual(tf.location.z, 1.5, places=5)
        self.assertAlmostEqual(tf.rotation.yaw, -90.0, places=4)
        self.assertAlmostEqual(tf.rotation.pitch, -5.0, places=4)
        # An unconverted old rig (CARLA frame, car centre) is sent without "frame" and used as it is.
        tf = _transform(m, car)
        self.assertAlmostEqual(tf.location.x, 0.5, places=5)
        self.assertAlmostEqual(tf.location.y, 0.8, places=5)
        self.assertAlmostEqual(tf.rotation.yaw, 90.0, places=4)

    def test_gui_rig_previews_go_through_rig_mount(self):
        # Every preview of a rig sensor (live-view combo, rig editor, view panes)
        # must say which frame its mount is in.
        src = os.path.join(REPO, "cosim_gui", "src")
        for name, n in (("panels.cpp", 1), ("rig_editor.cpp", 1), ("app.cpp", 1)):
            with open(os.path.join(src, name), encoding="utf-8") as f:
                text = f.read()
            with self.subTest(name=name):
                self.assertGreaterEqual(text.count("RigMount(s)"), n)
                self.assertNotIn('{"frame", "carsim"}', text)


class OldConfigTests(unittest.TestCase):
    def test_rig_without_sensors_is_not_old_format(self):
        import json
        import settings
        with tempfile.TemporaryDirectory() as root:
            for rig in ({"preset": "nuscenes"}, {"preset": "nuscenes", "sensors": []}):
                path = os.path.join(root, "partial.json")
                with open(path, "w") as f:
                    json.dump({"rig": rig}, f)
                with self.subTest(rig=rig):
                    self.assertEqual(settings.load_dict(path)["rig"]["frame"], "carsim")


class DocConventionTests(unittest.TestCase):
    # CarSim's mount angles: yaw + = left (90 = left), pitch + = nose down.
    WRONG = [re.compile(r"(?<![-−\d])90\s*=?\s*朝右"), re.compile(r"负数\s*=\s*向下看"), re.compile(r"俯仰负")]

    def test_docs_and_gui_give_carsim_mount_angles(self):
        files = glob.glob(os.path.join(REPO, "docs", "*.md")) + [os.path.join(REPO, "README.md")]
        files += glob.glob(os.path.join(REPO, "cosim_gui", "src", "*.cpp"))
        bad = []
        for path in files:
            with open(path, encoding="utf-8") as f:
                for no, line in enumerate(f, 1):
                    if any(p.search(line) for p in self.WRONG):
                        bad.append("%s:%d: %s" % (os.path.relpath(path, REPO), no, line.strip()[:80]))
        self.assertEqual(bad, [])


if __name__ == "__main__":
    unittest.main()
