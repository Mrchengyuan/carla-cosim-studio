"""Tests for the scene handed to the control algorithm (scene.py).

A parked car is placed on a straight road ahead of the ego; mock CarSim
drives towards it. Checks positions, relative speed, lane, collision
handling, rig sensors shared with a tiny collection (5 frames, temp dir
deleted afterwards) and the example algorithm stopping behind the car.

    python tests/test_scene.py [--port 2000]
"""
import glob
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
from test_backend import Conn, check  # noqa: E402

import carla  # noqa: E402

PORT = 57199

# Records what the algorithm receives; drives straight with a fixed throttle.
RECORDER = '''
import json, os
class Controller:
    def reset(self):
        self.out = open(os.environ["SCENE_TEST_LOG"], "w")
    def control(self, exports, t, dt, scene):
        car = [o for o in scene["objects"] if o["id"] == int(os.environ["SCENE_TEST_CAR"])]
        rec = {"t": t, "v": exports["Vx"] / 3.6, "car": car[0] if car else None, "lane": scene.get("lane"),
               "n": len(scene["objects"]), "types": sorted({o["type"] for o in scene["objects"]}),
               "sensors": {n: [s["type"], list(getattr(s["data"], "shape", []))] for n, s in scene.get("sensors", {}).items()}}
        self.out.write(json.dumps(rec) + "\\n")
        self.out.flush()
        return [0.35, 0.0, 0.0]
'''

OLD_STYLE = '''
def control(exports, t, dt):
    return [0.2, 0.0, 0.0]
'''


def run_until_state(c, states, timeout=120):
    return c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in states, timeout)


def straight_spawn(m):
    """A spawn point with 50 m of straight road ahead (no junction)."""
    pts = m.get_spawn_points()
    for i, p in enumerate(pts):
        wp = cur = m.get_waypoint(p.location)
        ok = True
        for _ in range(25):
            n = cur.next(2.0)
            if len(n) != 1 or n[0].is_junction or \
                    abs((n[0].transform.rotation.yaw - wp.transform.rotation.yaw + 180) % 360 - 180) > 2:
                ok = False
                break
            cur = n[0]
        if ok:
            return i, wp
    raise RuntimeError("no straight road in this map")


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    tmp = tempfile.mkdtemp(prefix="cosim_test_scene_")
    os.environ["SCENE_TEST_LOG"] = os.path.join(tmp, "scene.jsonl")
    proc = subprocess.Popen([sys.executable, os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT)],
                            cwd=os.path.join(HERE, ".."), env=dict(os.environ))
    client = carla.Client("localhost", carla_port)
    client.set_timeout(30.0)
    car = None
    try:
        c = Conn(PORT)
        c.call("connect", host="localhost", port=carla_port, timeout=60)
        w = client.get_world()
        sp, wp = straight_spawn(w.get_map())
        tf = wp.next(25.0)[0].transform
        tf.location.z += 0.3
        bp = w.get_blueprint_library().find("vehicle.audi.a2")
        car = w.spawn_actor(bp, tf)

        def put_back_car():
            # A run that hit it pushed it (the modified CARLA's ego is a physics
            # body): a fresh parked car for the next run.
            nonlocal car
            c.call("destroy_ego")  # parked right behind the car after a contact
            car.destroy()
            car = None
            for _ in range(20):
                w.wait_for_tick(5.0)
                car = w.try_spawn_actor(bp, tf)
                if car is not None:
                    break
            with open(os.path.join(tmp, "recorder.py"), "w") as f:
                f.write(RECORDER.replace('int(os.environ["SCENE_TEST_CAR"])', str(car.id)))

        with open(os.path.join(tmp, "recorder.py"), "w") as f:
            f.write(RECORDER.replace('int(os.environ["SCENE_TEST_CAR"])', str(car.id)))
        with open(os.path.join(tmp, "old_style.py"), "w") as f:
            f.write(OLD_STYLE)

        cfg = c.call("default_config")
        check("scene settings in the config", cfg["scene"]["objects"] and cfg["scene"]["collision"] == "log")
        cfg["carsim"]["mock"] = True
        cfg["run"]["log_path"] = ""
        cfg["carla"]["spawn_index"] = sp
        cfg["run"]["driver"] = "custom"
        cfg["run"]["controller"] = {"path": os.path.join(tmp, "recorder.py"), "entry": "Controller"}

        # ---- 1: drive into the parked car, collision = stop -------------------
        r = json.loads(json.dumps(cfg))
        r["sync"]["duration"] = 15.0
        r["scene"]["collision"] = "stop"
        c.events.clear()
        c.call("cosim_start", config=r, timeout=120)
        st = run_until_state(c, ("finished", "error", "stopped"), 150)
        recs = [json.loads(l) for l in open(os.environ["SCENE_TEST_LOG"])]
        first = recs[0]["car"]
        check("parked car straight ahead", first is not None and 23.0 < first["x"] < 27.0 and abs(first["y"]) < 0.5
              and abs(first["yaw"]) < 3, first and "x=%.2f y=%.2f yaw=%.1f" % (first["x"], first["y"], first["yaw"]))
        check("its size", first and 3.5 < first["length"] < 4.0 and 1.6 < first["width"] < 2.0,
              first and "%.2f x %.2f m" % (first["length"], first["width"]))
        mid = next(x for x in recs if x["v"] > 4.0)
        check("relative speed = -ego speed (CarSim's)", mid["car"] and abs(mid["car"]["vx"] + mid["v"]) < 0.3,
              mid["car"] and "vx=%.2f, ego %.2f m/s" % (mid["car"]["vx"], mid["v"]))
        seen = [x for x in recs if x["car"]]
        closing = all(b["car"]["x"] <= a["car"]["x"] + 1e-6 for a, b in zip(seen, seen[1:]))
        check("distance shrinks every frame", closing and len(seen) == len(recs), "%d frames" % len(recs))
        lane = recs[len(recs) // 2]["lane"]
        check("lane ahead", lane and 2.5 < lane["width"] < 4.5 and abs(lane["offset"]) < 0.5 and len(lane["center"]) > 20
              and abs(lane["center"][-1][1]) < 1.0, lane and "width %.1f, offset %.2f, %d points" % (
                  lane["width"], lane["offset"], len(lane["center"])))
        check("map objects present", "static" in recs[0]["types"] and recs[0]["n"] > 10, recs[0]["types"])
        check("collision stops the run", st["state"] == "finished" and "碰撞" in st.get("detail", "")
              and "audi" in st.get("detail", ""), st.get("detail"))
        last = recs[-1]["car"]
        gap = last["x"] - last["length"] / 2 - 4.7 / 2 if last else None
        check("stopped at contact, not before", last and gap < 0.6, "gap %.2f m" % gap if last else "")
        tel = [e["data"] for e in c.events if e.get("event") == "telemetry"]
        check("scene in the GUI telemetry", "scene" in tel[-1] and tel[-1]["scene"]["n_objects"] > 0
              and len(tel[-1]["scene"]["objects"]) <= 60 and len(json.dumps(tel[-1]["scene"])) < 30000,
              "%d bytes" % len(json.dumps(tel[-1]["scene"])))
        warns = [e["msg"] for e in c.events if e.get("event") == "log" and e.get("level") == "warn"]
        check("collision in the output", any("碰撞" in m for m in warns), warns[:2])

        # ---- 2: collision = log: the run goes on -------------------------------
        put_back_car()
        r["scene"]["collision"] = "log"
        r["sync"]["duration"] = 9.0
        c.events.clear()
        c.call("cosim_start", config=r, timeout=120)
        st = run_until_state(c, ("finished", "error", "stopped"), 150)
        warns = [e["msg"] for e in c.events if e.get("event") == "log" and e.get("level") == "warn" and "碰撞" in e["msg"]]
        recs = [json.loads(l) for l in open(os.environ["SCENE_TEST_LOG"])]
        check("collision = log keeps running", st["state"] == "finished" and "时长" in st.get("detail", "") and warns,
              "%s; %d warnings; car at start %s, at end %s" % (st.get("detail"), len(warns),
                                                              recs[0]["car"] and round(recs[0]["car"]["x"], 1),
                                                              recs[-1]["car"] and round(recs[-1]["car"]["x"], 1)))

        # ---- 3: rig sensors in the scene, shared with a tiny collection ---------
        r["sync"]["duration"] = 0.0
        r["scene"].update({"sensors": True, "collision": "off"})
        r["rig"]["sensors"] = [
            {"name": "cam", "type": "rgb", "x": 1.5, "y": 0, "z": 1.6, "roll": 0, "pitch": 0, "yaw": 0,
             "attributes": {"image_size_x": 320, "image_size_y": 180, "fov": 90}, "enabled": True},
            {"name": "lidar", "type": "lidar", "x": 0, "y": 0, "z": 1.9, "roll": 0, "pitch": 0, "yaw": 0,
             "attributes": {"channels": 16, "range": 50, "points_per_second": 50000}, "enabled": True}]
        r["collect"].update({"enabled": True, "out_dir": tmp, "session": "s", "max_frames": 5, "max_gb": 0.1})
        before = len([a for a in w.get_actors() if a.type_id.startswith("sensor.")])
        c.events.clear()
        info = c.call("cosim_start", config=r, timeout=120)
        c.wait_event(lambda e: e.get("event") == "telemetry" and e["data"]["frame"] >= 2, 60)
        during = len([a for a in w.get_actors() if a.type_id.startswith("sensor.")])
        st = run_until_state(c, ("finished", "error", "stopped"), 120)
        recs = [json.loads(l) for l in open(os.environ["SCENE_TEST_LOG"])]
        s = recs[-1]["sensors"]
        check("sensor data in the scene", s.get("cam") == ["rgb", [180, 320, 3]] and s.get("lidar", [0, [0]])[1][-1] == 4, s)
        check("one set of sensors for scene + collection", during - before == 2, "%d new sensor actors" % (during - before))
        root = info["collect"]["root"]
        counts = {n: len(glob.glob(os.path.join(root, n, "*"))) for n in ("cam", "lidar", "ego")}
        check("collection from the shared sensors", st["state"] == "finished" and all(v == 5 for v in counts.values()), counts)
        after = len([a for a in w.get_actors() if a.type_id.startswith("sensor.")])
        check("sensors removed after the run", after == before, "%d -> %d" % (before, after))

        # ---- 4: a 3-argument algorithm still works ------------------------------
        r = json.loads(json.dumps(cfg))
        r["sync"]["duration"] = 1.0
        r["run"]["controller"] = {"path": os.path.join(tmp, "old_style.py"), "entry": "control"}
        c.events.clear()
        c.call("cosim_start", config=r, timeout=120)
        st = run_until_state(c, ("finished", "error", "stopped"), 60)
        check("control(exports, t, dt) unchanged", st["state"] == "finished", st.get("detail"))

        # ---- 5: the example algorithm stops behind the parked car ---------------
        put_back_car()
        r = json.loads(json.dumps(cfg))
        r["sync"]["duration"] = 20.0
        r["scene"]["collision"] = "stop"
        r["run"]["controller"] = {"path": "controllers/scene_controller.py", "entry": "Controller"}
        c.events.clear()
        c.call("cosim_start", config=r, timeout=120)
        st = run_until_state(c, ("finished", "error", "stopped"), 150)
        tel = [e["data"] for e in c.events if e.get("event") == "telemetry"]
        lead = [o for o in tel[-1]["scene"]["objects"] if o["id"] == car.id]
        check("scene_controller.py follows and stops", st["state"] == "finished" and "时长" in st.get("detail", "")
              and tel[-1]["speed_kmh"] < 1.0 and lead and 7.0 < lead[0]["x"] < 14.0
              and max(t["speed_kmh"] for t in tel) > 8.0,
              "%s; end %.1f km/h, car %.1f m ahead" % (st.get("detail"), tel[-1]["speed_kmh"], lead[0]["x"] if lead else -1))
        c.call("destroy_ego")
        print("ALL SCENE TESTS PASSED")
    finally:
        if car is not None:
            try:
                car.destroy()
            except RuntimeError:
                pass
        shutil.rmtree(tmp, ignore_errors=True)
        proc.terminate()
        proc.wait(10)


if __name__ == "__main__":
    main()
