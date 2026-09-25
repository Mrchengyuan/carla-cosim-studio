"""CARLA goes away during a run (closed, crashed) and comes back: the backend
must survive, say so at once, and connect to the new server. Stops and
restarts the CARLA server itself (scripts/carla_server.sh, or
carla_mod_server.sh with --mod).

    python tests/test_carla_restart.py [--mod]
"""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from test_backend import Conn, check  # noqa: E402

PORT = 57194
ROOT = os.path.join(HERE, "..", "..")


def server(action, mod):
    kind, script, port = ("mod", "carla_mod_server.sh", 3000) if mod else ("stock", "carla_server.sh", 2000)
    session = "carla_mod" if mod else "carla_server"
    if action == "stop":
        cmd = "source scripts/carla_stop_lib.sh; stop_carla_server %s 'test_carla_restart'" % kind
    else:
        cmd = ("tmux new -d -s %s 'bash scripts/%s'; for i in $(seq 900); do ss -ltn | grep -q ':%d ' && break; sleep 1; done; sleep 8"
               % (session, script, port))
    subprocess.check_call(["bash", "-c", "cd '%s' && source scripts/env.sh && %s" % (ROOT, cmd)])
    return port


def main():
    mod = "--mod" in sys.argv
    carla_port = 3000 if mod else 2000
    proc = subprocess.Popen([sys.executable, os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT)],
                            cwd=os.path.join(HERE, ".."))
    try:
        time.sleep(2)
        c = Conn(PORT)
        c.call("connect", host="localhost", port=carla_port, timeout=60)
        c.call("spawn_ego", blueprint="vehicle.tesla.model3", spawn_index=3)
        c.call("views_set", views=[{"id": "p0", "kind": "rgb", "mode": "chase", "width": 160, "height": 90, "fps": 5}])
        import carla
        if hasattr(carla.Vehicle, "apply_external_state"):
            # The traffic manager is running. Only the patched carla package keeps
            # its thread from ending the process when CARLA goes away.
            c.call("spawn_traffic", vehicles=8, walkers=4, seed=1)
        else:
            print("original carla package: without traffic (its traffic manager ends the process when CARLA goes away)")
        cfg = c.call("default_config")
        cfg["carsim"]["mock"] = True
        cfg["run"]["log_path"] = ""
        cfg["carla"]["spawn_index"] = 3
        c.call("cosim_start", config=cfg)
        time.sleep(2)
        c.events.clear()
        server("stop", mod)
        t0 = time.time()
        lost = c.wait_event(lambda e: e.get("event") == "carla_lost", 90)
        st = c.wait_event(lambda e: e.get("event") == "cosim_state", 30)
        check("CARLA gone during a run: the backend says so and ends the run", bool(lost) and st["state"] == "error",
              "%.0f s: %s" % (time.time() - t0, lost.get("reason")))
        t = time.time()
        try:
            c.call("world_info", timeout=10)
            answer = "answered"
        except RuntimeError as e:
            answer = str(e)
        check("afterwards requests fail at once with a clear reason", "未连接 CARLA" in answer and time.time() - t < 2,
              "%.1f s: %s" % (time.time() - t, answer[-40:]))
        server("start", mod)
        check("the backend survived CARLA going away and coming back", proc.poll() is None and c.call("ping") == "pong")
        info = c.call("connect", host="localhost", port=carla_port, timeout=60)
        c.call("spawn_ego", blueprint="vehicle.tesla.model3", spawn_index=3)
        n = c.call("spawn_traffic", vehicles=5, walkers=0, seed=2)["vehicles"]
        c.events.clear()
        short = dict(cfg, sync=dict(cfg["sync"], duration=2.0))
        c.call("cosim_start", config=short)
        st = c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in ("finished", "error"), 90)
        check("connect to the new server: ego, traffic and a run work", bool(info["map"]) and n == 5 and st["state"] == "finished",
              (info["map"], n, st["state"], st.get("detail", "")[:60]))
        c.call("shutdown")
    finally:
        time.sleep(2)
        proc.terminate()
        proc.wait()
    print("ALL CARLA RESTART TESTS PASSED")


if __name__ == "__main__":
    main()
