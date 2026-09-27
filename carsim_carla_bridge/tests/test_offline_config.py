"""The config the GUI saves, settings.py and the command line agree; no CARLA
server needed. The command-line side runs here in Python; the GUI side
(cosim_gui/src/config_file.cpp: loading over the defaults, repairing types,
what a save writes, the file write) runs as cosim_gui/tests/config_file_test.cpp,
built here with the system C++ compiler (skipped without one)."""

import copy
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stderr
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
GUI = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "cosim_gui")

from backend_server import Backend  # noqa: E402
import run_cosim  # noqa: E402
import settings as st  # noqa: E402


class FakeWorld:
    def __init__(self):
        self.settings = SimpleNamespace(synchronous_mode=False, fixed_delta_seconds=None, no_rendering_mode=False)

    def get_settings(self):
        return copy.deepcopy(self.settings)

    def apply_settings(self, settings):
        self.settings = copy.deepcopy(settings)

    def get_map(self):
        return SimpleNamespace(get_spawn_points=lambda: ["anchor"] * 4)

    def get_blueprint_library(self):
        return SimpleNamespace(find=lambda name: name)

    def spawn_actor(self, blueprint, anchor):
        return SimpleNamespace(destroy=lambda: None)

    def tick(self):
        pass


def gui_saved(tmp, **drive):
    """A config as SaveConfig writes it (written by hand here; config_file_test.cpp
    checks the GUI's own): the backend's defaults, the GUI's CARLA
    connection, drive.cosim_driver copied into run.driver."""
    d = Backend().cmd_default_config()
    d["carla"].update(host="carla-box", port=3000, spawn_index=2)
    d["drive"].update(cosim_driver="demo", **drive)
    d["run"]["driver"] = d["drive"]["cosim_driver"]
    d["carsim"]["mock"] = True
    d["sync"]["duration"] = 0.02
    d["run"]["log_path"] = ""
    path = os.path.join(tmp, "cosim_config.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(d, f, ensure_ascii=False, indent=2)
    return path


class ConfigFileTests(unittest.TestCase):
    def test_no_dead_carla_fields(self):
        # carla.map / carla.weather were never read by anything.
        keys = {"host", "port", "vehicle", "spawn_index"}
        self.assertEqual(set(st.default_dict()["carla"]), keys)
        self.assertEqual(set(Backend().cmd_default_config()["carla"]), keys)

    def run_cli(self, argv):
        """run_cosim.main() with CARLA and the session mocked: (Client args, the session's config)."""
        seen = {}

        class Session:
            def __init__(self, world, vehicle, anchor, d):
                seen["d"] = d
                self.done = False

            def start(self):
                return {"external_api": False, "reference_point": [0, 0, 0],
                        "clock_warning": False, "t_step": 0.001, "inner_steps": 20}

            def step(self):
                self.done = True
                return {"t": 0.02, "rt_factor": 1.0}

            def stop(self, release_vehicle):
                pass

        world = FakeWorld()
        client = mock.Mock(return_value=SimpleNamespace(set_timeout=lambda t: None, get_world=lambda: world))
        with mock.patch.object(run_cosim.carla, "Client", client), \
                mock.patch.object(run_cosim, "CoSimSession", Session), \
                mock.patch.object(sys, "argv", ["run_cosim.py"] + argv), \
                redirect_stderr(io.StringIO()), mock.patch("sys.stdout", io.StringIO()):
            run_cosim.main()
        return client.call_args[0], seen["d"]

    def test_gui_saved_config_runs_its_driver_on_its_server(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = gui_saved(tmp)
            server, d = self.run_cli(["--config", path])
            self.assertEqual(server, ("carla-box", 3000))
            self.assertEqual((d["run"]["driver"], d["carla"]["spawn_index"]), ("demo", 2))
            # The command-line flags still win over the file.
            server, d = self.run_cli(["--config", path, "--port", "2000", "--driver", "custom"])
            self.assertEqual(server, ("carla-box", 2000))
            self.assertEqual(d["run"]["driver"], "custom")

    def test_carla_physics_config_is_refused_before_connecting(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = gui_saved(tmp, dynamics="carla", carla_driver="autopilot")
            err = io.StringIO()
            client = mock.Mock()
            with mock.patch.object(run_cosim.carla, "Client", client), \
                    mock.patch.object(sys, "argv", ["run_cosim.py", "--config", path]), \
                    redirect_stderr(err):
                with self.assertRaises(SystemExit) as cm:
                    run_cosim.main()
            self.assertEqual(cm.exception.code, 2)
            self.assertIn("drive.dynamics", err.getvalue())
            self.assertIn("CarSim", err.getvalue())
            client.assert_not_called()


class GuiConfigFileTests(unittest.TestCase):
    def test_gui_config_file(self):
        # The GUI's loading, repair and saving code with the backend's real
        # default config (cosim_gui/tests/config_file_test.cpp).
        cxx = shutil.which("c++") or shutil.which("g++") or shutil.which("clang++")
        if not cxx:
            self.skipTest("no C++ compiler")
        with tempfile.TemporaryDirectory() as tmp:
            defaults = os.path.join(tmp, "default_config.json")
            with open(defaults, "w", encoding="utf-8") as f:
                json.dump(Backend().cmd_default_config(), f, ensure_ascii=False)
            exe = os.path.join(tmp, "config_file_test")
            build = subprocess.run([cxx, "-std=c++17", "-Wall", "-I", os.path.join(GUI, "src"),
                                    "-I", os.path.join(GUI, "third_party", "json"),
                                    os.path.join(GUI, "tests", "config_file_test.cpp"),
                                    os.path.join(GUI, "src", "config_file.cpp"), "-o", exe],
                                   capture_output=True, text=True)
            self.assertEqual(build.returncode, 0, build.stderr[-3000:])
            files = os.path.join(tmp, "files")
            os.mkdir(files)
            r = subprocess.run([exe, defaults, files], capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
            self.assertIn(" 0 failed", r.stdout)


if __name__ == "__main__":
    unittest.main()
