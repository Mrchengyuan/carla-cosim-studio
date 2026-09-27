"""Connection and run state on a real CARLA: the trash button on the 场景对象
page, pedestrians spawned while paused, run state after a reconnect,
vehicle_specs with an unknown model, a failed map change, file commands while
control() is slow, a second GUI, a normal exit with idle tick, Ctrl+C during a
run. Collects no data; the world is left as it was found.

    python tests/test_state_carla.py [--port 2000]
"""
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
from test_backend import Conn, check  # noqa: E402
from test_robustness import expect_error, heroes, run_until  # noqa: E402

PORT = 57173
VIEW = [{"id": "p0", "kind": "rgb", "mode": "chase", "width": 160, "height": 90, "fps": 5}]


def start_backend():
    return subprocess.Popen([sys.executable, os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT)],
                            cwd=os.path.join(HERE, ".."))


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    import carla
    cl = carla.Client("localhost", carla_port)
    cl.set_timeout(60)
    w = cl.get_world()
    settings0 = w.get_settings()
    map0 = w.get_map().name
    tmp = tempfile.mkdtemp(prefix="cc_state_")
    proc = start_backend()
    try:
        time.sleep(2)
        c = Conn(PORT)
        c.call("connect", host="localhost", port=carla_port)
        base = c.call("default_config")
        base["carsim"]["mock"] = True
        base["run"]["log_path"] = ""
        base["carla"]["spawn_index"] = 3

        def ctrl(name, body):
            p = os.path.join(tmp, name)
            with open(p, "w", encoding="utf-8") as f:
                f.write(body)
            cfg = json.loads(json.dumps(base))
            cfg["run"]["controller"] = {"path": p, "entry": "Controller"}
            return cfg

        c.call("spawn_ego", blueprint="vehicle.tesla.model3", spawn_index=3)
        ego = c.call("world_info")["ego_id"]

        # ---- the trash button refuses the ego's live-view camera ---------------
        c.call("views_set", views=VIEW)
        w.wait_for_tick(10)
        cams = [a.id for a in w.get_actors().filter("sensor.camera.*") if a.parent and a.parent.id == ego]
        ok, msg = expect_error(c, "destroy_actor", "实时画面", id=cams[0])
        c.events.clear()
        frame = c.wait_event(lambda e: e.get("event") == "frame", 20)
        check("trash on the ego's live-view camera: refused, the view keeps streaming", ok and bool(frame), msg)

        # ---- deleting a pedestrian takes its AI controller; counts follow ------
        n0 = c.call("spawn_traffic", vehicles=3, walkers=3, seed=4)
        w.wait_for_tick(10)
        walker = w.get_actors().filter("walker.pedestrian.*")[0]
        ctrls = [a.id for a in w.get_actors().filter("controller.ai.walker") if a.parent and a.parent.id == walker.id]
        car = [a for a in w.get_actors().filter("vehicle.*") if a.attributes.get("role_name") == "autopilot"][0]
        r_walker = c.call("destroy_actor", id=walker.id)  # the traffic left, for the GUI's count
        r_car = c.call("destroy_actor", id=car.id)
        w.wait_for_tick(10)
        left = {a.id for a in w.get_actors()}
        n1 = c.call("spawn_traffic", vehicles=0, walkers=0, seed=4)
        check("deleting a pedestrian takes its controller; the traffic counts drop by what was deleted",
              walker.id not in left and ctrls and not set(ctrls) & left
              and n1 == {"vehicles": n0["vehicles"] - 1, "walkers": n0["walkers"] - 1}
              and r_walker == {"vehicles": n0["vehicles"], "walkers": n0["walkers"] - 1} and r_car == n1,
              (n0, n1, r_walker, r_car, ctrls))
        c.call("clear_traffic")

        # ---- pedestrians spawned while a run is paused walk once it stops ------
        c.call("cosim_start", config=json.loads(json.dumps(base)))
        c.call("cosim_pause")
        m = c.call("spawn_traffic", vehicles=0, walkers=4, seed=6)["walkers"]
        c.call("cosim_stop")
        w.wait_for_tick(10)
        p0 = {a.id: a.get_location() for a in w.get_actors().filter("walker.pedestrian.*")}
        time.sleep(4.0)
        w.wait_for_tick(10)
        moved = [a.id for a in w.get_actors().filter("walker.pedestrian.*")
                 if a.id in p0 and a.get_location().distance(p0[a.id]) > 0.3]
        check("pedestrians spawned while paused walk after the run stops", m > 0 and len(moved) > 0,
              "%d spawned, %d moved" % (m, len(moved)))
        c.call("clear_traffic")

        # ---- a new connection does not carry an old run's state ----------------
        c.events.clear()
        c.call("cosim_start", config=ctrl("raises.py", "class Controller:\n    def control(self, e, t, dt):\n        raise ValueError('boom')\n"))
        st = run_until(c, ("error", "finished", "stopped"))
        before = c.call("world_info")["cosim_state"]
        info = c.call("connect", host="localhost", port=carla_port)
        check("after a run error, a reconnect starts with no run state",
              st["state"] == "error" and before == "error" and info["cosim_state"] == "stopped"
              and c.call("world_info")["cosim_state"] == "stopped", (st["state"], before, info["cosim_state"]))

        # ---- vehicle_specs: an unknown model does not stop the others ---------
        c.events.clear()
        specs = c.call("vehicle_specs", ids=["vehicle.no.such_car", "vehicle.tesla.model3"])
        c.call("ping")
        warned = [e["msg"] for e in c.events if e.get("event") == "log" and "vehicle.no.such_car" in e.get("msg", "")]
        check("vehicle_specs: an unknown model is reported, the others are measured",
              list(specs) == ["vehicle.tesla.model3"] and 2.8 < specs["vehicle.tesla.model3"]["wheelbase_m"] < 3.1
              and len(warned) == 1, (list(specs), warned))

        # ---- a failed map change leaves no untracked ego ----------------------
        c.call("spawn_ego", blueprint="vehicle.tesla.model3", spawn_index=3)
        ok, msg = expect_error(c, "load_map", "", name="NoSuchTown")
        w.wait_for_tick(10)
        wi = c.call("world_info")
        check("failed map change: an error, no ego left behind, the backend goes on",
              ok and heroes(w) == [] and wi["ego_id"] == 0 and w.get_map().name == map0 and c.call("ping") == "pong", msg)
        c.call("spawn_ego", blueprint="vehicle.tesla.model3", spawn_index=3)
        check("the spawn point is free again after the failed map change", c.call("world_info")["ego_id"] != 0)

        # ---- file commands answer while control() is slow ---------------------
        slow = ctrl("slow.py", "import time\nclass Controller:\n    def control(self, e, t, dt):\n"
                               "        time.sleep(2.0)\n        return [0.0, 0.0, 0.0]\n")
        c.call("cosim_start", config=slow, timeout=60)
        time.sleep(0.5)
        t0 = time.time()
        c.call("disk_info", path=tmp, timeout=10)
        dt = time.time() - t0
        c.call("cosim_stop", timeout=60)
        check("disk_info while control() takes 2 s a frame: answered at once (own thread)", dt < 1.0, "%.2f s" % dt)

        # ---- a second GUI is told the backend is taken -------------------------
        s2 = socket.create_connection(("127.0.0.1", PORT), timeout=10)
        f2 = s2.makefile("rb")
        msg = json.loads(f2.readline())
        eof = f2.readline() == b""
        f2.close()
        s2.close()
        check("a second GUI is told the backend is taken and let go; the first goes on",
              msg.get("rejected") is True and "另一个界面窗口" in msg.get("msg", "") and eof and c.call("ping") == "pong", msg)

        # ---- a normal exit with sync + idle tick leaves CARLA asynchronous -----
        c.call("world_settings", synchronous=True, frame_dt=0.05, idle_tick=True)
        time.sleep(1.0)
        c.call("shutdown")
        proc.wait(20)
        s = w.get_settings()
        check("a normal exit with idle tick: the world is asynchronous again, nothing of ours left",
              not s.synchronous_mode and heroes(w) == [], (s.synchronous_mode, len(heroes(w))))
        proc = start_backend()
        time.sleep(2)
        c = Conn(PORT)
        c.events.clear()
        c.call("connect", host="localhost", port=carla_port)
        c.call("ping")
        check("the next backend does not report an unclean exit",
              not any("没有正常退出" in e.get("msg", "") for e in c.events))

        # ---- Ctrl+C (SIGINT) during a run: bounded clean exit ------------------
        c.call("spawn_ego", blueprint="vehicle.tesla.model3", spawn_index=3)
        c.call("spawn_traffic", vehicles=3, walkers=2, seed=9)
        c.call("cosim_start", config=json.loads(json.dumps(base)))
        time.sleep(1.5)
        proc.send_signal(signal.SIGINT)
        try:
            rc = proc.wait(20)
        except subprocess.TimeoutExpired:
            rc = None
        w.wait_for_tick(10)
        ours = [a for a in w.get_actors() if a.type_id.startswith(("vehicle.", "walker.", "controller.ai.walker", "sensor."))
                and (a.attributes.get("role_name") in ("hero", "autopilot") or not a.type_id.startswith("vehicle."))]
        check("Ctrl+C during a run: the backend exits within the limit, the world is clean and asynchronous",
              rc == 0 and not ours and not w.get_settings().synchronous_mode, (rc, [a.type_id for a in ours][:5]))
    finally:
        if proc.poll() is None:
            proc.terminate()
            proc.wait()
        try:
            w.apply_settings(settings0)
        except RuntimeError:
            pass
        shutil.rmtree(tmp, ignore_errors=True)
    print("ALL STATE TESTS PASSED")


if __name__ == "__main__":
    main()
