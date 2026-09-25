"""A short mock co-sim with every vehicle model as the ego (bicycles, 6-wheel
truck and bus included). The CARLA server must survive each one: the modified
CARLA is a development build and stops on any out-of-bounds access.

    python tests/test_all_vehicles.py [--port 3000]
"""
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
from test_backend import Conn, check  # noqa: E402

PORT = 57185


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    import carla
    cl = carla.Client("localhost", carla_port)
    cl.set_timeout(60)
    ids = sorted(b.id for b in cl.get_world().get_blueprint_library().filter("vehicle.*"))
    proc = subprocess.Popen([sys.executable, os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT)],
                            cwd=os.path.join(HERE, ".."))
    try:
        time.sleep(2)
        c = Conn(PORT)
        c.call("connect", host="localhost", port=carla_port)
        base = c.call("default_config")
        base["carsim"]["mock"] = True
        base["run"]["log_path"] = ""
        base["sync"]["duration"] = 2.0
        base["carla"]["spawn_index"] = 3
        bad = []
        for vid in ids:
            cfg = json.loads(json.dumps(base))
            cfg["carla"]["vehicle"] = vid
            c.events.clear()
            try:
                c.call("cosim_start", config=cfg, timeout=120)
                st = c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in ("finished", "error", "stopped"), 120)
                n_tel = sum(1 for e in c.events if e.get("event") == "telemetry")
                if st["state"] != "finished" or n_tel < 10:
                    bad.append((vid, st["state"], st.get("detail", "")[:60]))
            except Exception as e:  # noqa: BLE001  report and go on with the next model
                bad.append((vid, "exception", str(e)[:60]))
            try:
                cl.get_server_version()
            except RuntimeError:
                check("server survives co-sim with every vehicle model", False, "died at %s" % vid)
        c.call("destroy_ego")
        check("co-sim runs with every vehicle model (%d)" % len(ids), not bad, bad[:5])
    finally:
        proc.terminate()
        proc.wait()
    print("ALL VEHICLE TESTS PASSED")


if __name__ == "__main__":
    main()
