"""The user's control algorithm going wrong, end to end through the backend:
a start that fails says why (cosim_state detail for the GUI banner), a
KeyError on a scene key that is not ticked points to the 场景信息 page,
sys.exit() in control(), an error in a helper file names both lines, a file
named like a loaded module (config.py) is refused, a helper in a
sub-package is reloaded on the next run, and a slow control() shows up in
the "busy" heartbeat as control() at its line, not as a hung CARLA.

    python tests/test_algoerr_carla.py [--port 2000]
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
from test_backend import Conn, check  # noqa: E402
from test_robustness import expect_error, run_until  # noqa: E402

PORT = 57188


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    tmp = tempfile.mkdtemp(prefix="cc_algoerr_")
    proc = subprocess.Popen([sys.executable, os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT)],
                            cwd=os.path.join(HERE, ".."))
    try:
        time.sleep(2)
        c = Conn(PORT)
        c.call("connect", host="localhost", port=carla_port)
        base = c.call("default_config")
        base["carsim"]["mock"] = True
        base["run"]["log_path"] = ""
        base["carla"]["spawn_index"] = 3
        base["sync"]["duration"] = 0.5
        c.call("spawn_ego", blueprint="vehicle.tesla.model3", spawn_index=3)

        def write(rel, body):
            p = os.path.join(tmp, rel)
            os.makedirs(os.path.dirname(p), exist_ok=True)
            with open(p, "w", encoding="utf-8") as f:
                f.write(body)
            return p

        def ctrl(rel, body, entry="control"):
            cfg = json.loads(json.dumps(base))
            cfg["run"]["controller"] = {"path": write(rel, body), "entry": entry}
            return cfg

        def run_error(cfg):
            c.events.clear()
            c.call("cosim_start", config=cfg)
            st = run_until(c, ("error", "finished", "stopped"))
            return st["state"], st.get("detail", "")

        # ---- a start that fails: the reply and the state say why ---------------
        c.events.clear()
        ok, msg = expect_error(c, "cosim_start", "NameError", config=ctrl("bad/c.py", "x = undefined_name\n"))
        states = [e for e in c.events if e.get("event") == "cosim_state"]
        check("failed start: the error and the cosim_state detail give the reason",
              ok and states and states[-1]["state"] == "error" and "NameError" in states[-1]["detail"]
              and "c.py 第 1 行" in states[-1]["detail"], (msg, states[-1:]))
        check("backend answers after the failed start", c.call("ping") == "pong")

        ok, msg = expect_error(c, "cosim_start", "control（函数）",
                               config=ctrl("fn/c.py", "def control(e, t, dt):\n    return [0.0, 0.0, 0.0]\n", "Controller"))
        check("entry not found: lists what the file has", ok, msg)

        write("clash/config.py", "KP = 0.1\n")
        ok, msg = expect_error(c, "cosim_start", "改个名字",
                               config=ctrl("clash/ctrl.py", "import config\n\ndef control(e, t, dt):\n    return [config.KP, 0, 0]\n"))
        check("own config.py next to the algorithm: asked to rename it", ok, msg)

        # ---- errors during the run --------------------------------------------
        cfg = ctrl("sc/c.py", "def control(e, t, dt, scene):\n    return [0.0 * scene['ego']['Speed'], 0.0, 0.0]\n")
        cfg["scene"]["ego"] = ["X"]
        state, detail = run_error(cfg)
        check("unticked scene key: points to the 场景信息 page", state == "error" and "场景里没有 'Speed'" in detail, detail)

        state, detail = run_error(ctrl("ex/exit3.py", "import sys\n\ndef control(e, t, dt):\n    sys.exit(3)\n"))
        check("sys.exit in control(): says so, with the line",
              state == "error" and "sys.exit(3)（exit3.py 第 4 行）" in detail, detail)

        write("h/mpc.py", "def solve(x):\n    return 1.0 / x\n")
        state, detail = run_error(ctrl("h/my_ctrl.py", "import mpc\n\n\ndef control(e, t, dt):\n    return [mpc.solve(0.0), 0, 0]\n"))
        check("error in a helper: its line and the calling line",
              state == "error" and "（mpc.py 第 2 行，由 my_ctrl.py 第 5 行调用）" in detail, detail)

        # ---- a helper in a sub-package is reloaded on the next run -------------
        write("pkg/mpc/__init__.py", "")
        seen = []
        for thr in (0.11, 0.2222):
            write("pkg/mpc/gain.py", "THROTTLE = %r\n" % thr)
            state, detail = run_error(ctrl("pkg/c.py", "from mpc import gain\n\ndef control(e, t, dt):\n"
                                                       "    return [gain.THROTTLE, 0.0, 0.0]\n"))
            tel = [e["data"] for e in c.events if e.get("event") == "telemetry"]
            seen.append((state, round(tel[-1]["action"][0], 4) if tel else None))
        check("helper module in a sub-folder: the edit applies on the next run",
              seen == [("finished", 0.11), ("finished", 0.2222)], seen)

        # ---- a slow control(): the heartbeat names it, not CARLA ---------------
        cfg = ctrl("slow/slow.py", "import time\n\n\ndef control(e, t, dt):\n    time.sleep(4.0)\n    return [0.0, 0.0, 0.0]\n")
        cfg["sync"]["duration"] = 0.1
        c.events.clear()
        c.call("cosim_start", config=cfg)
        busy = c.wait_event(lambda e: e.get("event") == "busy" and e.get("task") == "control", 15)
        check("slow control(): busy heartbeat says control() at its line",
              busy.get("where") == "（slow.py 第 5 行）" and busy["seconds"] >= 1.5, busy)
        st = run_until(c, ("error", "finished", "stopped"), 30)
        check("slow control(): the run still ends normally", st["state"] == "finished", st)
        c.call("destroy_ego")
    finally:
        proc.terminate()
        proc.wait()
        shutil.rmtree(tmp, ignore_errors=True)
    print("ALL ALGOERR TESTS PASSED")


if __name__ == "__main__":
    main()
