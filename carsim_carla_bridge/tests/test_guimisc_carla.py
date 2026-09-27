"""GUI fixes through the backend on a CARLA server: the GUI and the backend
agree on the protocol ("hello"); CARLA's autopilot on the ego is reported off
after a run (the run respawns the ego and parks it) but kept by a start refused
before the ego was touched, so the GUI's checkbox follows it; a stack dump is
answered during a run without waiting for the worker and lands in the log;
"shutdown" (停止后端, closing the GUI) removes the ego from CARLA and ends the
backend. Mock CarSim, no data collection.

    python tests/test_guimisc_carla.py [--port 2000]
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

import backend_server  # noqa: E402

PORT = 57186


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    tmp = tempfile.mkdtemp(prefix="cc_guimisc_")
    log_path = os.path.join(tmp, "backend.log")
    log = open(log_path, "w")
    proc = subprocess.Popen([sys.executable, "-u", os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT)],
                            cwd=os.path.join(HERE, ".."), stdout=log, stderr=subprocess.STDOUT)
    try:
        c = Conn(PORT)
        check("hello: the protocol this tree's GUI was built for",
              c.call("hello") == {"protocol": backend_server.PROTOCOL})
        c.call("connect", host="localhost", port=carla_port)
        base = c.call("default_config")
        base["carsim"]["mock"] = True
        base["run"]["log_path"] = ""
        base["carla"]["spawn_index"] = 3
        base["sync"]["duration"] = 3.0
        ego = c.call("spawn_ego", blueprint="vehicle.tesla.model3", spawn_index=3)
        check("a new ego is not on autopilot", c.call("world_info")["ego_autopilot"] is False)
        c.call("ego_autopilot", enabled=True)
        check("autopilot on is reported", c.call("world_info")["ego_autopilot"] is True)

        # Refused before the ego is touched: it keeps driving on autopilot.
        bad = json.loads(json.dumps(base))
        bad["run"]["controller"] = {"path": os.path.join(tmp, "missing.py"), "entry": "control"}
        ok, msg = expect_error(c, "cosim_start", "控制算法文件不存在", config=bad)
        wi = c.call("world_info")
        check("refused start: the same ego, still on autopilot",
              ok and wi["ego_id"] == ego["id"] and wi["ego_autopilot"] is True, (msg, wi["ego_id"], wi["ego_autopilot"]))

        # A run respawns the ego and parks it at the end: autopilot off.
        c.events.clear()
        c.call("cosim_start", config=base)
        t0 = time.time()
        check("dump_stacks during a run is answered at once",
              c.call("dump_stacks", timeout=5) is True and time.time() - t0 < 2.0, "%.2f s" % (time.time() - t0))
        st = run_until(c, ("finished", "error", "stopped"))
        check("run finished", st["state"] == "finished", (st["state"], st.get("detail")))
        wi = c.call("world_info")
        check("after the run the parked ego is not on autopilot", wi["ego_id"] > 0 and wi["ego_autopilot"] is False,
              (wi["ego_id"], wi["ego_autopilot"]))
        log.flush()
        with open(log_path, encoding="utf-8", errors="replace") as f:
            text = f.read()
        check("the stacks are in the backend's log", "most recent call first" in text)

        # 停止后端 / closing the GUI: the backend removes the ego and exits.
        ego_id = wi["ego_id"]
        c.call("shutdown")
        t0 = time.time()
        try:
            proc.wait(10)
        except subprocess.TimeoutExpired:
            pass
        check("shutdown ends the backend", proc.poll() is not None, "%.1f s" % (time.time() - t0))
        import carla
        cl = carla.Client("localhost", carla_port)
        cl.set_timeout(30)
        left = [a.id for a in cl.get_world().get_actors().filter("vehicle.*") if a.id == ego_id]
        check("shutdown removed the ego from CARLA", not left, left)
        print("ALL GUIMISC CARLA TESTS PASSED")
    finally:
        if proc.poll() is None:
            proc.terminate()
            proc.wait(10)
        log.close()
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    main()
