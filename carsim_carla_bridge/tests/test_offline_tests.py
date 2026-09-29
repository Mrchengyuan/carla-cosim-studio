"""Checks for the test and launcher scripts that need no CARLA server:
test_modified_carla.py fails the process when a check fails (run against a
fake carla module), start_studio.sh reuses only a CARLA it recognises on the
port, and the stop patterns match the servers' command lines through symlinks.
No real CARLA, tmux session or GUI is started or stopped: tmux, pkill, pgrep,
zenity and ss are stubbed.

    python tests/test_offline_tests.py
"""

import contextlib
import importlib
import io
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import types
import unittest
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
SCRIPTS = os.path.join(ROOT, "scripts")
sys.path.insert(0, HERE)


def fake_carla(gyro_sign=1.0, accel_sign=1.0, imu=True, bones=True, susp_sign=1.0):
    """A modified CARLA doing what test_modified_carla.py checks. The IMU follows
    InertialMeasurementUnit.cpp: gyroscope = the external angular velocity in
    rad/s, accelerometer = 2nd difference of the sensor location + gravity,
    both turned into the sensor's frame."""
    m = types.ModuleType("carla")

    class Vector3D:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x, self.y, self.z = float(x), float(y), float(z)

    class Location(Vector3D):
        def distance(self, o):
            return math.dist((self.x, self.y, self.z), (o.x, o.y, o.z))

    class Rotation:
        def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
            self.pitch, self.yaw, self.roll = pitch, yaw, roll

    class Transform:
        def __init__(self, location=None, rotation=None):
            self.location, self.rotation = location or Location(), rotation or Rotation()

    class Vehicle:
        id = 7

        def __init__(self, tf):
            self.tf, self.vel, self.ang, self.state = tf, Vector3D(), Vector3D(), None

        def enable_external_dynamics(self):
            pass

        def apply_external_state(self, tf, vel, ang, wheel_steer=(), wheel_rotation=(), wheel_suspension=(),
                                 throttle=0.0, steer=0.0, brake=0.0, gear=0):
            self.tf, self.vel, self.ang = tf, vel, ang
            self.state = SimpleNamespace(steer=list(wheel_steer), susp=list(wheel_suspension),
                                         control=SimpleNamespace(throttle=throttle, steer=steer, brake=brake, gear=gear))

        def restore_physx_physics(self):
            pass

        def get_transform(self):
            return self.tf

        def get_velocity(self):
            return self.vel

        def get_angular_velocity(self):
            return self.ang

        def get_control(self):
            return self.state.control

        def get_wheel_steer_angle(self, wheel):
            return self.state.steer[wheel]

        def get_bone_names(self):
            return ["Root"] + (["Wheel_Front_Left", "Wheel_Front_Right", "Wheel_Rear_Left", "Wheel_Rear_Right"] if bones else [])

        def get_bone_relative_transforms(self):
            return [Transform()] + [Transform(Location(0, 0, 0.3 + susp_sign * s)) for s in self.state.susp][:len(self.get_bone_names()) - 1]

        def destroy(self):
            pass

    class Imu:
        def __init__(self, parent):
            self.parent, self.cb = parent, None
            self.prev, self.prev_dt = [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0)], 1e30

        def listen(self, cb):
            self.cb = cb

        def stop(self):
            self.cb = None

        def destroy(self):
            pass

        def sense(self, frame, dt):
            loc = self.parent.tf.location
            y0, (y2, y1) = (loc.x, loc.y, loc.z), self.prev
            h1, h2 = dt, self.prev_dt
            a = [-2.0 * (b1 / (h1 * h2) - b2 / (h2 * (h1 + h2)) - b0 / (h1 * (h1 + h2))) for b0, b1, b2 in zip(y0, y1, y2)]
            self.prev, self.prev_dt = [y1, y0], dt
            a[2] += 9.81
            yaw = math.radians(self.parent.tf.rotation.yaw)  # UnrotateVector by the (yaw only) sensor rotation
            ax = math.cos(yaw) * a[0] + math.sin(yaw) * a[1]
            ay = -math.sin(yaw) * a[0] + math.cos(yaw) * a[1]
            if self.cb and imu:
                self.cb(SimpleNamespace(frame=frame, gyroscope=Vector3D(0, 0, gyro_sign * math.radians(self.parent.ang.z)),
                                        accelerometer=Vector3D(ax, accel_sign * ay, a[2])))

    class World:
        def __init__(self):
            self.frame, self.actors = 1000, []
            self.settings = SimpleNamespace(synchronous_mode=False, fixed_delta_seconds=None)

        def get_map(self):
            return SimpleNamespace(name="Town10HD_Opt", get_spawn_points=lambda: [
                Transform(Location(-40.0 + 17 * i, 25.0 - 11 * i, 0.6), Rotation(yaw=-100.0 + 37 * i)) for i in range(6)])

        def get_settings(self):
            return SimpleNamespace(**vars(self.settings))

        def apply_settings(self, s):
            self.settings = s

        def get_blueprint_library(self):
            return SimpleNamespace(find=lambda name: name)

        def spawn_actor(self, bp, tf, attach_to=None):
            a = Imu(attach_to) if bp.startswith("sensor.") else Vehicle(tf)
            self.actors.append(a)
            return a

        def tick(self):
            self.frame += 1
            for a in self.actors:
                if isinstance(a, Imu):
                    a.sense(self.frame, self.settings.fixed_delta_seconds or 0.05)
            return self.frame

        def get_snapshot(self):
            vehicle = next(a for a in self.actors if isinstance(a, Vehicle))
            return SimpleNamespace(find=lambda actor_id: vehicle)

    world = World()

    class Client:
        def __init__(self, host, port):
            pass

        def set_timeout(self, seconds):
            pass

        def get_world(self):
            return world

        def get_server_version(self):
            return "0.9.16-fake"

    m.Vector3D, m.Location, m.Rotation, m.Transform = Vector3D, Location, Rotation, Transform
    m.Vehicle, m.Client = Vehicle, Client
    m.VehicleWheelLocation = SimpleNamespace(FL_Wheel=0, FR_Wheel=1, BL_Wheel=2, BR_Wheel=3)
    return m


class ModifiedCarlaCheckTests(unittest.TestCase):
    def run_checks(self, **faults):
        saved = sys.modules.get("carla")
        sys.modules["carla"] = fake_carla(**faults)
        sys.modules.pop("test_modified_carla", None)
        out = io.StringIO()
        try:
            test_modified_carla = importlib.import_module("test_modified_carla")
            with contextlib.redirect_stdout(out), self.assertRaises(SystemExit) as cm:
                test_modified_carla.main()
        finally:
            sys.modules.pop("test_modified_carla", None)
            if saved is None:
                sys.modules.pop("carla", None)
            else:
                sys.modules["carla"] = saved
        return cm.exception.code, out.getvalue()

    def test_healthy_server_passes_with_exit_0(self):
        code, out = self.run_checks()
        self.assertEqual(code, 0, out)
        self.assertNotIn("FAIL", out)
        self.assertIn("PASS IMU accelerometer sees the centripetal acceleration", out)
        self.assertIn("ALL MODIFIED-CARLA TESTS PASSED", out)

    def test_each_fault_fails_the_process(self):
        for faults, failed in (({"gyro_sign": -1.0}, "IMU gyroscope"),
                               ({"accel_sign": -1.0}, "IMU accelerometer"),
                               ({"imu": False}, "IMU gyroscope"),
                               ({"bones": False}, "suspension"),
                               ({"susp_sign": -1.0}, "suspension")):
            with self.subTest(faults=faults):
                code, out = self.run_checks(**faults)
                self.assertEqual(code, 1, out)
                self.assertIn("FAIL " + failed, out)
                self.assertIn("SOME TESTS FAILED", out)


def read(path):
    with open(path) as f:
        return f.read()


def stub(path, body):
    with open(path, "w") as f:
        f.write("#!/bin/bash\n" + body + "\n")
    os.chmod(path, 0o755)


class LauncherTests(unittest.TestCase):
    """start_studio.sh with stubs: `ss` reports FAKE_OWNER listening on the port
    (no users field when FAKE_OWNER is "-": another user's process), the GUI and
    the backend Python only record how they were called."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="cc_tests_")
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.bin = os.path.join(self.tmp, "bin")
        os.mkdir(self.bin)
        rec = os.path.join(self.tmp, "calls")
        self.rec = rec
        stub(os.path.join(self.bin, "ss"),
             '[ -n "$FAKE_OWNER" ] || exit 0\n'
             'users=""; [ "$FAKE_OWNER" = "-" ] || users="users:((\\"$FAKE_OWNER\\",pid=4242,fd=12))"\n'
             'echo "LISTEN 0      128          0.0.0.0:$FAKE_PORT       0.0.0.0:*    $users"')
        for name, rc in (("tmux", 1), ("pkill", 1), ("pgrep", 1), ("zenity", 0)):
            stub(os.path.join(self.bin, name), 'echo "%s $*" >> "%s"\nexit %d' % (name, rec, rc))
        self.gui = os.path.join(self.tmp, "gui")
        stub(self.gui, 'echo "gui $*" >> "%s"' % rec)
        self.python = os.path.join(self.tmp, "python")
        stub(self.python, 'echo "python $*" >> "%s"\nexit ${FAKE_PY_RC:-1}' % rec)

    def start(self, mode, owner, py_rc=1):
        port = "3000" if mode == "mod" else "2000"
        env = dict(os.environ, PATH=self.bin + os.pathsep + os.environ["PATH"], XDG_CACHE_HOME=self.tmp,
                   STUDIO_BIN=self.gui, COSIM_PYTHON=self.python, CARLA_PORT="2000", CARLA_MOD_PORT="3000",
                   CARLA_MOD_ROOT=os.path.join(self.tmp, "no_carla_mod"),
                   FAKE_OWNER=owner, FAKE_PORT=port, FAKE_PY_RC=str(py_rc))
        env.pop("COSIM_ROOT", None)
        p = subprocess.run(["bash", os.path.join(SCRIPTS, "start_studio.sh")] + ([mode] if mode else []),
                           env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=60)
        calls = []
        if os.path.exists(self.rec):
            calls = read(self.rec).splitlines()
            os.remove(self.rec)
        return p.returncode, calls

    def test_carla_known_by_name_without_connecting(self):
        for mode, owner, port in (("", "CarlaUE4-Linux-", "2000"), ("mod", "UE4Editor", "3000"), ("mod", "CarlaUE4-Linux-", "3000")):
            with self.subTest(owner=owner):
                rc, calls = self.start(mode, owner)
                self.assertEqual(rc, 0, calls)
                self.assertIn("gui --python %s --backend-dir %s --carla-port %s --auto-connect"
                              % (self.python, os.path.join(ROOT, "carsim_carla_bridge"), port), calls)
                self.assertFalse([c for c in calls if c.startswith(("python", "tmux", "pkill", "zenity"))], calls)

    def test_other_program_on_port_is_not_used_as_carla(self):
        rc, calls = self.start("mod", "node")
        self.assertEqual(rc, 1, calls)
        self.assertFalse([c for c in calls if c.startswith(("gui", "tmux", "pkill"))], calls)
        # Asked once, not probed in a loop.
        self.assertEqual([c for c in calls if c.startswith("python")], [calls[0]], calls)
        self.assertIn("3000", calls[0])
        err = [c for c in calls if c.startswith("zenity")]
        self.assertTrue(err and "端口 3000 被其他程序（node）占用" in err[0] and "CARLA_MOD_PORT" in err[0], calls)
        log = read(os.path.join(self.tmp, "carla_cosim_studio", "launch_mod.log"))
        self.assertIn("port 3000 is used by another program (node), not by CARLA", log)

    def test_hidden_owner_is_asked_its_version_once(self):
        rc, calls = self.start("", "-", py_rc=0)
        self.assertEqual(rc, 0, calls)
        self.assertEqual(len([c for c in calls if c.startswith("python")]), 1, calls)
        self.assertTrue(any(c.startswith("gui ") for c in calls), calls)
        rc, calls = self.start("", "-", py_rc=1)
        self.assertEqual(rc, 1, calls)
        self.assertFalse([c for c in calls if c.startswith(("gui", "tmux", "pkill"))], calls)


class StopPatternTests(unittest.TestCase):
    """The stock / mod stop patterns match the command lines the servers really
    get when CARLA_ROOT / CARLA_SRC / CARLA_MOD_ROOT go through a symlink, for
    the modified CARLA's editor build and its packaged build. The servers are
    stubs that print their command line; pkill only records its pattern."""

    # CARLA 0.9.16's own CarlaUE4.sh (starts the binary by its readlink -f path).
    CARLAUE4_SH = ('#!/bin/sh\n'
                   'UE4_TRUE_SCRIPT_NAME=$(echo \\"$0\\" | xargs readlink -f)\n'
                   'UE4_PROJECT_ROOT=$(dirname "$UE4_TRUE_SCRIPT_NAME")\n'
                   'chmod +x "$UE4_PROJECT_ROOT/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"\n'
                   '"$UE4_PROJECT_ROOT/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping" CarlaUE4 "$@" \n')

    def test_patterns_match_through_symlinks(self):
        self.check_patterns(packaged=False)

    def test_patterns_match_packaged_modified_carla(self):
        self.check_patterns(packaged=True)

    def check_patterns(self, packaged):
        tmp = os.path.realpath(tempfile.mkdtemp(prefix="cc_tests_"))
        self.addCleanup(shutil.rmtree, tmp, True)
        real, link = os.path.join(tmp, "disk"), os.path.join(tmp, "home")
        os.makedirs(os.path.join(real, "carla", "CarlaUE4", "Binaries", "Linux"))
        os.makedirs(os.path.join(real, "mod", "CarlaUE4", "Binaries", "Linux"))
        os.makedirs(os.path.join(real, "src", "Unreal", "CarlaUE4"))
        os.makedirs(os.path.join(tmp, "ue4", "Engine", "Binaries", "Linux"))
        os.symlink(real, link)
        cmdlines = os.path.join(tmp, "cmdlines")
        for d in ("carla", "mod") if packaged else ("carla",):
            with open(os.path.join(real, d, "CarlaUE4.sh"), "w") as f:
                f.write(self.CARLAUE4_SH)
            os.chmod(os.path.join(real, d, "CarlaUE4.sh"), 0o755)
        for exe in (os.path.join(real, "carla", "CarlaUE4", "Binaries", "Linux", "CarlaUE4-Linux-Shipping"),
                    os.path.join(real, "mod", "CarlaUE4", "Binaries", "Linux", "CarlaUE4-Linux-Shipping"),
                    os.path.join(tmp, "ue4", "Engine", "Binaries", "Linux", "UE4Editor")):
            stub(exe, 'echo "$0 $*" >> "%s"' % cmdlines)
        bin_ = os.path.join(tmp, "bin")
        os.mkdir(bin_)
        pats = os.path.join(tmp, "patterns")
        stub(os.path.join(bin_, "pkill"), 'printf "%%s\\n" "${@: -1}" >> "%s"' % pats)
        stub(os.path.join(bin_, "pgrep"), "exit 1")
        stub(os.path.join(bin_, "tmux"), "exit 0")
        env = dict(os.environ, PATH=bin_ + os.pathsep + os.environ["PATH"], XDG_CACHE_HOME=tmp,
                   CARLA_ROOT=os.path.join(link, "carla"), CARLA_SRC=os.path.join(link, "src"),
                   # Without a CarlaUE4.sh there the editor build is started.
                   CARLA_MOD_ROOT=os.path.join(link, "mod"),
                   UE4_ROOT=os.path.join(tmp, "ue4"), CARLA_PORT="2999", CARLA_MOD_PORT="3999")
        env.pop("COSIM_ROOT", None)
        for script in ("carla_server.sh", "carla_mod_server.sh"):
            subprocess.run(["bash", os.path.join(SCRIPTS, script)], env=env, check=True, timeout=30)
        subprocess.run(["bash", "-c", 'source "%s"; stop_carla_server stock t; stop_carla_server mod t'
                        % os.path.join(SCRIPTS, "carla_stop_lib.sh")], env=env, check=True, timeout=30)
        stock_cmd, mod_cmd = read(cmdlines).splitlines()
        stock_pat, mod_pat = read(pats).splitlines()  # -TERM only: the stub pgrep finds nothing left
        self.assertTrue(stock_cmd.startswith(real), stock_cmd)
        self.assertIn(real, mod_cmd)
        self.assertEqual("CarlaUE4-Linux-Shipping" in mod_cmd, packaged, mod_cmd)
        self.assertTrue(re.search(stock_pat, stock_cmd), (stock_pat, stock_cmd))
        self.assertTrue(re.search(mod_pat, mod_cmd), (mod_pat, mod_cmd))
        # Still only this installation's servers, each of its own kind.
        self.assertFalse(re.search(stock_pat, mod_cmd) or re.search(mod_pat, stock_cmd))
        self.assertFalse(re.search(stock_pat, stock_cmd.replace(real, os.path.join(tmp, "other"))))


if __name__ == "__main__":
    unittest.main()
