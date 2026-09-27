"""What the GUI displays and the documented sensor signs, without a CARLA server:

* the GUI's pose (gui_view "ego") is CarSim's global frame and units, with
  Pitch / Roll like CarSim's exports, and they stay out of what the algorithm
  gets and what is recorded;
* a run on stock CARLA with an IMU (for the algorithm or the dataset) warns
  once at the start that the gyro reads 0; not on the modified CARLA;
* radar / IMU conversions have the signs docs/场景与数据接口.md 4.6 states.

    python tests/test_offline_display.py
"""

import math
import os
import struct
import sys
import unittest
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import carla  # noqa: E402

import config as cfg  # noqa: E402
import scene as scn  # noqa: E402
import session  # noqa: E402
import settings as st  # noqa: E402
from bridge import CarSimExports  # noqa: E402
from coords import AnchorFrame, euler_from_rot  # noqa: E402


class PoseTests(unittest.TestCase):
    """A car placed the way the bridge places it (AnchorFrame.pose_to_world)
    reads back as the CarSim pose it came from."""

    def provider(self, anchor, carsim_pose, units=None):
        X, Y, Z, yaw, pitch, roll = carsim_pose
        a = anchor.rotation
        af = AnchorFrame((anchor.location.x, anchor.location.y, anchor.location.z), a.yaw, a.pitch, a.roll)
        pos, R = af.pose_to_world((X, Y, Z), yaw, pitch, roll)
        cy, cp, cr = euler_from_rot(R)
        tf = carla.Transform(carla.Location(*map(float, pos)), carla.Rotation(pitch=cp, yaw=cy, roll=cr))
        ego = SimpleNamespace(id=1, get_transform=lambda: tf, get_velocity=lambda: carla.Vector3D(),
                              bounding_box=carla.BoundingBox(carla.Location(1.4, 0.0, 0.7), carla.Vector3D(2.3, 1.0, 0.7)))

        class Snap(list):  # no other actors
            def find(self, i):
                return ego if i == 1 else None

        world = SimpleNamespace(get_map=lambda: SimpleNamespace(get_waypoint=lambda *a, **k: None),
                                get_blueprint_library=lambda: None, get_snapshot=lambda: Snap(),
                                get_environment_objects=lambda label: [])
        d = st.default_dict()
        d["scene"]["lane"], d["scene"]["record"]["lane"] = [], []
        if units:
            d["carsim"]["units"] = dict(d["carsim"]["units"], **units)
        sp = scn.SceneProvider(world, ego, d, anchor, [0.0, 0.0, 0.0], [])
        sp.start()
        return sp

    def test_gui_pose_is_carsim_global(self):
        anchor = carla.Transform(carla.Location(120.0, -40.0, 0.3), carla.Rotation(yaw=-150.0))
        pose = (12.0, 5.0, 0.2, 25.0, 3.0, -2.0)  # X, Y, Z m; Yaw, Pitch (nose down), Roll (right side down) deg
        sp = self.provider(anchor, pose)
        eg = scn.gui_view(sp.update(1000, 0.0))["ego"]
        for k, want in zip(("X", "Y", "Z", "Yaw", "Pitch", "Roll"), pose):
            self.assertAlmostEqual(eg[k], want, delta=0.02, msg=k)

    def test_angles_in_the_page_units(self):
        anchor = carla.Transform(carla.Location(0.0, 0.0, 0.0), carla.Rotation(yaw=90.0))
        sp = self.provider(anchor, (0.0, 0.0, 0.0, 10.0, 2.0, 1.5), units={"angle": "rad"})
        eg = sp.update(1000, 0.0)["ego"]
        for k, deg in (("Yaw", 10.0), ("Pitch", 2.0), ("Roll", 1.5)):
            self.assertAlmostEqual(eg[k], math.radians(deg), delta=1e-4, msg=k)

    def test_pitch_roll_only_for_the_gui(self):
        sp = self.provider(carla.Transform(), (0.0, 0.0, 0.0, 0.0, 1.0, 1.0))
        sp.s["ego"] = list(scn.EGO_KEYS)
        sp.s["record"]["ego"] = list(scn.EGO_KEYS)
        sp.update(1000, 0.0)
        for v in (sp.view(), sp.record_view()):
            self.assertEqual(sorted(v["ego"]), sorted(scn.EGO_KEYS))
        self.assertNotIn("Pitch", scn.EGO_KEYS)
        self.assertNotIn("Roll", scn.EGO_KEYS)


class FakeSync:
    """CarlaVehicleSync without CARLA; external_api set per test."""
    external_api = False

    def __init__(self, world, vehicle, anchor, use_external_api=None, settings=None):
        self.ex = CarSimExports(settings.EXPORT_NAMES, settings.UNITS)
        self.wheel_radius_m = [0.35] * 4
        self.ref_local = [1.4, 0.0, 0.0]
        self.server_api = None

    def sync(self, obs, t, dt):
        return SimpleNamespace(velocity=carla.Vector3D(), wheel_steer=[], wheel_rotation=[], wheel_suspension=[])

    def release(self):
        pass


class ImuWarningTests(unittest.TestCase):
    def start(self, sensor_types, external_api):
        world = SimpleNamespace(get_settings=lambda: SimpleNamespace(), apply_settings=lambda s: None, tick=lambda: 1)
        bb = SimpleNamespace(location=SimpleNamespace(z=0.7), extent=SimpleNamespace(z=0.7))
        vehicle = SimpleNamespace(bounding_box=bb, get_transform=lambda: carla.Transform())

        def start_scene(ses, *a, **k):  # the scene's sensors: for the algorithm and / or the collector
            ses.scene = SimpleNamespace(sensor_cfgs=[{"name": t, "type": t} for t in sensor_types], stop=lambda: None)
            return {"collisions": []}

        sync = type("Sync", (FakeSync,), {"external_api": external_api})
        d = st.default_dict()
        d["carsim"]["mock"] = True
        d["run"]["driver"] = "demo"
        d["run"]["log_path"] = ""
        with mock.patch.object(session, "CarlaVehicleSync", sync), \
                mock.patch.object(session, "start_scene", start_scene):
            ses = session.CoSimSession(world, vehicle, None, d)
            try:
                return ses.start()["warnings"]
            finally:
                ses.stop()

    def test_stock_carla_with_imu_warns_once(self):
        warns = self.start(["rgb", "imu"], external_api=False)
        self.assertEqual(warns.count(session.IMU_STOCK_WARNING), 1, warns)
        self.assertIn("gyro", session.IMU_STOCK_WARNING)
        self.assertIn("AVz", session.IMU_STOCK_WARNING)

    def test_no_warning_without_imu_or_on_modified_carla(self):
        for types, ext in ((["rgb", "gnss"], False), (["imu"], True), ([], False)):
            with self.subTest(types=types, external_api=ext):
                self.assertNotIn(session.IMU_STOCK_WARNING, self.start(types, ext))


class SensorSignTests(unittest.TestCase):
    """The signs the spec (4.6) and meta.json state."""

    def test_radar(self):
        # CARLA: velocity m/s (range rate), azimuth rad (+ right), altitude rad (+ up), depth m.
        raw = struct.pack("4f", -5.0, math.radians(10.0), math.radians(4.0), 20.0)
        dist, az, el, vel = scn.radar_iso(raw, 3.6, 1.0)[0]
        self.assertAlmostEqual(dist, 20.0, places=4)
        self.assertAlmostEqual(az, -10.0, places=3)   # right of the sensor: azimuth < 0 (+ = left)
        self.assertAlmostEqual(el, 4.0, places=3)     # above: elevation > 0 (+ = up)
        self.assertAlmostEqual(vel, -18.0, places=3)  # closing in: < 0, km/h

    def test_imu(self):
        v = lambda x, y, z: SimpleNamespace(x=x, y=y, z=z)
        d = SimpleNamespace(accelerometer=v(0.5, 1.0, 9.81), gyroscope=v(0.0, 0.0, 0.0), compass=math.radians(90.0))
        out = scn.imu_gnss("imu", d, scn.Units(cfg.UNITS))
        self.assertEqual(out["accel"], [0.5, -1.0, 9.81])   # m/s^2 with gravity, y left
        self.assertEqual(out["gyro"], [0.0, 0.0, 0.0])      # what stock CARLA gives
        self.assertAlmostEqual(out["compass"], 90.0)        # heading from north, clockwise, deg
        rad = scn.imu_gnss("imu", d, scn.Units({"angle": "rad", "rate": "rad/s"}))
        self.assertAlmostEqual(rad["compass"], math.radians(90.0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
