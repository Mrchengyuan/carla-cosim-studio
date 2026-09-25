"""The backend under abuse: bad user code, bad input, reconnects, failed
starts. After every case the backend must still answer and the world must
hold exactly what it should.

    python tests/test_robustness.py [--port 2000]
"""
import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
from test_backend import Conn, check  # noqa: E402

PORT = 57193


def expect_error(c, cmd, text, **args):
    try:
        c.call(cmd, **args)
    except RuntimeError as e:
        return text in str(e), str(e)[-90:]
    return False, "no error"


def heroes(w):
    return [a for a in w.get_actors().filter("vehicle.*") if a.attributes.get("role_name") == "hero"]


def run_until(c, states, timeout=60):
    return c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in states, timeout)


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    import carla
    cl = carla.Client("localhost", carla_port)
    cl.set_timeout(60)  # the modified CARLA (editor build) answers slowly right after start
    w = cl.get_world()
    tmp = tempfile.mkdtemp(prefix="cc_rob_")
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
        c.call("spawn_ego", blueprint="vehicle.tesla.model3", spawn_index=3)

        def ctrl(name, body):
            p = os.path.join(tmp, name)
            with open(p, "w", encoding="utf-8") as f:
                f.write(body)
            cfg = json.loads(json.dumps(base))
            cfg["run"]["controller"] = {"path": p, "entry": "Controller"}
            return cfg

        # ---- user code that calls sys.exit at import (argparse at module level)
        cfg = ctrl("exit_on_import.py", "import sys\nsys.exit(2)\nclass Controller:\n    pass\n")
        ok, msg = expect_error(c, "cosim_start", "sys.exit", config=cfg)
        check("controller calling sys.exit at import: clear error", ok, msg)
        check("backend still answers after that", c.call("ping") == "pong")

        # ---- user code that raises SystemExit / returns NaN during the run ------
        for name, body, text in (
                ("exit_in_control.py", "class Controller:\n    def control(self, e, t, dt):\n        raise SystemExit(3)\n", ""),
                ("nan_out.py", "class Controller:\n    def control(self, e, t, dt):\n        return [float('nan'), 0.0, 0.0]\n", "无效数值")):
            c.events.clear()
            c.call("cosim_start", config=ctrl(name, body))
            st = run_until(c, ("error", "finished", "stopped"))
            check("run with %s ends in error state" % name, st["state"] == "error" and text in st.get("detail", ""),
                  (st["state"], st.get("detail", "")[:60]))
            check("backend answers after %s" % name, c.call("ping") == "pong" and c.call("world_info")["cosim_state"] == "error")

        # ---- malformed request line --------------------------------------------
        c.s.sendall(b"{this is not json\n")
        c.events.clear()
        check("malformed request does not end the backend", c.call("ping") == "pong")
        c.call("ping")
        check("malformed request is reported", any(e.get("event") == "log" and "无法解析" in e.get("msg", "") for e in c.events))

        # ---- stop with nothing running still reports a state --------------------
        c.events.clear()
        c.call("cosim_stop")
        c.call("ping")
        check("cosim_stop always reports the state", any(e.get("event") == "cosim_state" for e in c.events))

        # ---- bad requests never lose the current ego --------------------------
        ego0 = c.call("world_info")["ego_id"]
        ok, msg = expect_error(c, "spawn_ego", "没有车型", blueprint="vehicle.no.such_car", spawn_index=3)
        check("unknown vehicle: clear error", ok, msg)
        check("unknown vehicle: ego kept", c.call("world_info")["ego_id"] == ego0 and len(heroes(w)) == 1)
        c.call("add_sensor", type="imu")
        c.call("views_set", views=[{"id": "p0", "kind": "rgb", "mode": "chase", "width": 160, "height": 90, "fps": 5}])
        bad = json.loads(json.dumps(base))
        bad["carla"]["vehicle"] = "vehicle.no.such_car"
        ok, msg = expect_error(c, "cosim_start", "没有车型", config=bad)
        kinds = [s["type"] for s in c.call("list_sensors")]
        check("run with unknown vehicle: error, ego / sensors kept", ok and c.call("world_info")["ego_id"] == ego0 and kinds == ["imu"],
              (msg, kinds))
        ok, msg = expect_error(c, "views_set", "不能作为视图", views=[{"id": "p1", "kind": "imu"}])
        check("imu as a view is refused before touching the views", ok, msg)
        c.events.clear()
        c.call("views_set", views=[{"id": "p0", "kind": "rgb", "mode": "chase", "width": 160, "height": 90, "fps": 5}])
        check("views still work after the refusal", c.wait_event(lambda e: e.get("event") == "frame", 20)["w"] == 160)
        c.events.clear()
        ok, msg = expect_error(c, "views_set", "", views=[{"id": "p0", "kind": "rgb", "mount": {"x": "not a number"}}])
        c.call("ping")
        va = [e["ids"] for e in c.events if e.get("event") == "views_active"]
        check("a view that cannot be built: error, and the GUI is told no views are left",
              ok and va == [[]] and not [a for a in w.get_actors().filter("sensor.camera.*") if a.parent and a.parent.id == ego0], va)
        c.call("views_set", views=[{"id": "p0", "kind": "rgb", "mode": "chase", "width": 160, "height": 90, "fps": 5}])

        # ---- CARLA interface "auto" works whichever client / server pair it is --
        guess = c.call("world_info")["external_api_server"]  # from the versions at connect
        info = c.call("cosim_start", config=json.loads(json.dumps(base)))
        c.call("cosim_stop")
        if hasattr(carla.Vehicle, "apply_external_state"):  # patched client
            check("auto interface: runs, uses the API only if the server has it; the guess at connect agrees",
                  info["server_api"] in (True, False) and info["external_api"] == info["server_api"]
                  and guess in (None, info["server_api"])
                  and c.call("world_info")["external_api_server"] == info["server_api"], (guess, info["server_api"]))
            if info["server_api"] is False:
                forced = json.loads(json.dumps(base))
                forced["sync"]["use_external_api"] = True
                ok, msg = expect_error(c, "cosim_start", "不是改版", config=forced)
                check("forced patched interface on an original server: clear error", ok, msg)
        else:
            check("auto interface with the original client: compatibility mode",
                  info["external_api"] is False and c.call("world_info")["external_api_server"] is None)

        # ---- measuring every vehicle, trucks with 6 wheels included ------------
        # (the read order matters: the other one crashes development builds of CARLA)
        specs = c.call("vehicle_specs", timeout=600)
        n_bp = len(w.get_blueprint_library().filter("vehicle.*"))
        hgv = specs.get("vehicle.carlamotors.european_hgv", {})
        check("vehicle_specs measures every vehicle, multi-wheel trucks included",
              len(specs) == n_bp and len(hgv.get("wheel_radius_m", [])) == 6 and c.call("ping") == "pong",
              "%d / %d vehicles, european_hgv wheels %d" % (len(specs), n_bp, len(hgv.get("wheel_radius_m", []))))

        # ---- reconnect cleans up what this backend put into the world ----------
        n_traffic = c.call("spawn_traffic", vehicles=5, walkers=0, seed=3)["vehicles"]
        n_before = len(w.get_actors().filter("vehicle.*"))
        c.events.clear()
        c.call("connect", host="localhost", port=carla_port)
        time.sleep(1.0)
        n_after = len(w.get_actors().filter("vehicle.*"))
        c.call("ping")
        va = [e["ids"] for e in c.events if e.get("event") == "views_active"]
        check("reconnect removes our ego, traffic and views", heroes(w) == [] and n_after == n_before - n_traffic - 1 and va and va[-1] == [],
              "vehicles %d -> %d, heroes %d, views %s" % (n_before, n_after, len(heroes(w)), va[-1:]))

        # ---- NaN in outgoing messages ------------------------------------------
        import backend_server as bs
        safe = bs._json_safe({"a": float("nan"), "b": [1.0, float("inf")], "c": {"d": float("-inf")}})
        check("non-finite numbers are sent as null", json.dumps(safe, allow_nan=False) == '{"a": null, "b": [1.0, null], "c": {"d": null}}')
        c.call("destroy_ego")
    finally:
        proc.terminate()
        proc.wait()
        shutil.rmtree(tmp, ignore_errors=True)
    print("ALL ROBUSTNESS TESTS PASSED")


if __name__ == "__main__":
    main()
