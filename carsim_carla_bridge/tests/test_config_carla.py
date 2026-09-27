"""A config saved the way the GUI saves it runs the same from the command
line, against a running CARLA server:

    python tests/test_config_carla.py [--port 2000]

- run_cosim.py --config <file> connects to the file's CARLA host / port and
  drives with the file's driver: the user's control algorithm (mock CarSim),
  then leaves the world as it was;
- a file with drive.dynamics = "carla" is refused before anything is spawned.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
from test_backend import check  # noqa: E402
import settings as st  # noqa: E402

CONTROLLER = """
class Controller:
    def control(self, exports, t, dt):
        with open(%r, "a") as f:
            f.write("%%.3f\\n" %% t)
        return [0.3, 0.0, 0.0]
"""


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    import carla
    cl = carla.Client("localhost", carla_port)
    cl.set_timeout(60)
    w = cl.get_world()
    tmp = tempfile.mkdtemp(prefix="cc_cfg_")
    try:
        marker = os.path.join(tmp, "control_calls.txt")
        ctrl = os.path.join(tmp, "my_controller.py")
        with open(ctrl, "w", encoding="utf-8") as f:
            f.write(CONTROLLER % marker)
        # What the GUI's SaveConfig writes (by hand here; the GUI's own code is
        # checked by cosim_gui/tests/config_file_test.cpp): the defaults, the
        # GUI's CARLA connection, drive.cosim_driver and run.driver in step.
        d = st.default_dict()
        d["carla"].update(host="localhost", port=carla_port, spawn_index=5)
        d["drive"]["cosim_driver"] = d["run"]["driver"] = "custom"
        d["run"]["controller"] = {"path": ctrl, "entry": "Controller"}
        d["run"]["log_path"] = ""
        d["carsim"]["mock"] = True
        d["sync"].update(frame_dt=0.05, duration=0.5)
        path = os.path.join(tmp, "cosim_config.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(d, f, ensure_ascii=False, indent=2)

        def run():
            return subprocess.run([sys.executable, "run_cosim.py", "--config", path], cwd=os.path.join(HERE, ".."),
                                  capture_output=True, text=True, timeout=300)

        n0, sync0 = len(w.get_actors().filter("vehicle.*")), w.get_settings().synchronous_mode
        r = run()
        n1, sync1 = len(w.get_actors().filter("vehicle.*")), w.get_settings().synchronous_mode
        calls = open(marker).read().split() if os.path.exists(marker) else []
        check("GUI-saved config runs from the command line on the file's CARLA server",
              r.returncode == 0 and "ran " in r.stdout, (r.returncode, r.stdout[-200:], r.stderr[-300:]))
        check("... driven by the file's driver: the user's control algorithm", len(calls) >= 5, len(calls))
        check("... and the world is left as it was", n1 == n0 and sync1 == sync0, (n0, n1, sync0, sync1))

        d["drive"]["dynamics"] = "carla"
        with open(path, "w", encoding="utf-8") as f:
            json.dump(d, f, ensure_ascii=False, indent=2)
        r = run()
        n2 = len(w.get_actors().filter("vehicle.*"))
        check("a CARLA-physics config is refused with a clear message, nothing spawned",
              r.returncode == 2 and "drive.dynamics" in r.stderr and n2 == n0, (r.returncode, r.stderr[-200:], n0, n2))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print("ALL CONFIG TESTS PASSED")


if __name__ == "__main__":
    main()
