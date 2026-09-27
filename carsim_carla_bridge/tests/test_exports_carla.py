"""Export checks, units and the examples on a real CARLA server (mock CarSim).

1. Through the backend (mock CarSim, default and SI units on the CarSim page):
   no "导出变量可疑" warning, the algorithm gets scene["units"] = the page's
   units, and example_controller.py reaches the same speed in both unit sets.
2. In-process: a .sim whose export order differs from the page (Vx / Vy
   swapped): exactly one warning, about 1 s in, and the run goes on.

    python tests/test_exports_carla.py [--port 2000]
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
from test_backend import Conn, check  # noqa: E402

import carla  # noqa: E402

import config as cfg  # noqa: E402
import session  # noqa: E402
import settings as st  # noqa: E402
from mock_carsim import MockCarSimEnv  # noqa: E402

PORT = 57191
SI = {"angle": "rad", "speed": "m/s", "rate": "rad/s", "wheel_spin": "rad/s", "jounce": "m"}

# The example algorithm, also writing down the units it was handed.
UNITS_LOG = '''
import json, os, sys
sys.path.insert(0, os.environ["EXPORTS_TEST_CTRL"])
from example_controller import Controller as Example
class Controller(Example):
    def control(self, exports, t, dt, scene):
        with open(os.environ["EXPORTS_TEST_LOG"], "a") as f:
            f.write(json.dumps(scene["units"]) + "\\n")
        return super().control(exports, t, dt, scene)
'''


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    tmp = tempfile.mkdtemp(prefix="cc_test_exports_")
    log = os.path.join(tmp, "units.jsonl")
    os.environ["EXPORTS_TEST_LOG"] = log
    os.environ["EXPORTS_TEST_CTRL"] = os.path.join(HERE, "..", "controllers")
    with open(os.path.join(tmp, "units_ctrl.py"), "w") as f:
        f.write(UNITS_LOG)
    proc = subprocess.Popen([sys.executable, os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT)],
                            cwd=os.path.join(HERE, ".."), env=dict(os.environ))
    client = carla.Client("localhost", carla_port)
    client.set_timeout(60.0)
    try:
        # ---- 1: through the backend ------------------------------------------
        c = Conn(PORT)
        c.call("connect", host="localhost", port=carla_port, timeout=60)
        base = c.call("default_config")
        base["carsim"]["mock"] = True
        base["run"]["log_path"] = ""
        base["run"]["driver"] = "custom"
        base["run"]["controller"] = {"path": os.path.join(tmp, "units_ctrl.py"), "entry": "Controller"}
        base["sync"]["duration"] = 8.0
        base["carla"]["spawn_index"] = 3
        c.call("spawn_ego", blueprint="vehicle.tesla.model3", spawn_index=3)
        top = {}
        for name, units in (("default", dict(cfg.UNITS)), ("SI", SI)):
            r = json.loads(json.dumps(base))
            r["carsim"]["units"] = units
            if os.path.exists(log):
                os.remove(log)
            c.events.clear()
            c.call("cosim_start", config=r, timeout=120)
            stt = c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in ("finished", "error", "stopped"), 120)
            warns = [e["msg"] for e in c.events if e.get("event") == "log" and "导出变量可疑" in e.get("msg", "")]
            tel = [e["data"] for e in c.events if e.get("event") == "telemetry"]
            got = [json.loads(line) for line in open(log)]
            top[name] = max(t["speed_kmh"] for t in tel)
            check("%s units: run finished, no export warning" % name, stt["state"] == "finished" and not warns,
                  "%s %s" % (stt.get("detail"), warns))
            check("%s units: scene['units'] = the page's" % name, got and all(u == units for u in got), got[:1])
        check("example_controller.py: same speed in km/h and m/s", abs(top["default"] - top["SI"]) < 0.05 * top["default"]
              and top["default"] > 20.0, top)
        c.call("destroy_ego")
        proc.terminate()  # nobody else on the world while this process ticks it
        proc.wait(10)

        # ---- 2: in-process, the .sim's order differs from the page -----------
        w = client.get_world()
        original = w.get_settings()
        anchor = w.get_map().get_spawn_points()[3]
        vehicle = w.spawn_actor(w.get_blueprint_library().find("vehicle.tesla.model3"), anchor)
        try:
            s = w.get_settings()
            s.synchronous_mode, s.fixed_delta_seconds = True, 0.02
            w.apply_settings(s)
            for _ in range(20):
                w.tick()
            d = st.load_dict(override={"carsim": {"mock": True}, "run": {"driver": "demo", "log_path": ""}})
            names = list(cfg.EXPORT_NAMES)
            i, j = names.index("Vx"), names.index("Vy")
            names[i], names[j] = names[j], names[i]
            d["carsim"]["export_names"] = names
            with mock.patch.object(session, "make_env", lambda d: MockCarSimEnv(cfg.EXPORT_NAMES, t_stop=1e9)):
                ses = session.CoSimSession(w, vehicle, anchor, d)
                try:
                    info = ses.start()
                    out = []
                    while ses.env.t_current < 3.0:
                        tel = ses.step()
                        out += [(tel["t"], m) for m in tel["warnings"]]
                finally:
                    ses.stop(release_vehicle=False)
            check("swapped Vx / Vy: quiet at rest", info["warnings"] == [], info["warnings"])
            check("swapped Vx / Vy: one warning about 1 s in", len(out) == 1 and out[0][0] >= 1.0 and "Vx =" in out[0][1], out)
        finally:
            vehicle.destroy()
            w.apply_settings(original)
        print("ALL EXPORT TESTS PASSED")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
        if proc.poll() is None:
            proc.terminate()
            proc.wait(10)


if __name__ == "__main__":
    main()
