"""Tests for the scene handed to the control algorithm and recorded (scene.py,
docs/场景与数据接口.md).

A car is parked on a straight road ahead of the ego; mock CarSim drives
towards it. Checks CarSim frames and units (against the mock's Xo / Yo /
Yaw / Vx), the selection of keys, collision handling, the run records at the
sampling period, a tiny collection (5 frames, temp dir deleted afterwards)
with frames/ in step with the sensor files, and the example algorithm.

    python tests/test_scene.py [--port 2000]
"""
import csv
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
ALL_OBJECT_KEYS = ["id", "type", "parked", "model", "length", "width", "height", "X", "Y", "Z", "Yaw",
                   "Vx_global", "Vy_global", "Speed", "rel_x", "rel_y", "rel_yaw", "rel_vx", "rel_vy", "dist", "gap"]
ALL_LANE_KEYS = ["width", "offset", "heading_err", "curvature", "center_rel", "center_global", "center_curvature",
                 "left_marking", "right_marking", "left_lane", "right_lane", "speed_limit", "in_junction",
                 "junction_dist", "light_state", "light_dist"]

# Records what the algorithm receives; drives straight with a fixed throttle.
RECORDER = '''
import json, os
class Controller:
    def reset(self):
        self.out = open(os.environ["SCENE_TEST_LOG"], "w")
    def control(self, exports, t, dt, scene):
        car = [o for o in scene["objects"] if o["id"] == CAR_ID]
        rec = {"t": t, "ex": {k: exports[k] for k in ("Xo", "Yo", "Yaw", "Vx")}, "scene_t": scene["t"],
               "ego": scene["ego"], "car": car[0] if car else None, "lane": scene.get("lane"), "keys": sorted(scene),
               "obj_keys": sorted(scene["objects"][0]) if scene["objects"] else [],
               "types": sorted({o["type"] for o in scene["objects"]}),
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
    for i, p in enumerate(m.get_spawn_points()):
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


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    tmp = tempfile.mkdtemp(prefix="cosim_test_scene_")
    log = os.path.join(tmp, "scene.jsonl")
    os.environ["SCENE_TEST_LOG"] = log
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

        def put_car():
            # A fresh parked car for every run: a hit pushes it away (the
            # modified CARLA's ego is a physics body).
            nonlocal car
            c.call("destroy_ego")  # it may stand right behind the car after a contact
            if car is not None:
                car.destroy()
            car = None
            for _ in range(20):
                w.wait_for_tick(5.0)
                car = w.try_spawn_actor(bp, tf)
                if car is not None:
                    break
            with open(os.path.join(tmp, "recorder.py"), "w") as f:
                f.write(RECORDER.replace("CAR_ID", str(car.id)))

        def records():
            return [json.loads(line) for line in open(log)]

        with open(os.path.join(tmp, "old_style.py"), "w") as f:
            f.write(OLD_STYLE)
        cfg = c.call("default_config")
        check("scene selection in the config", cfg["scene"]["collision"] == "log" and "rel_x" in cfg["scene"]["objects"]
              and set(cfg["scene"]["object_types"]) == {"vehicle", "walker", "parked"})
        cfg["carsim"]["mock"] = True
        cfg["run"]["log_path"] = ""
        cfg["carla"]["spawn_index"] = sp
        cfg["run"]["driver"] = "custom"
        cfg["run"]["controller"] = {"path": os.path.join(tmp, "recorder.py"), "entry": "Controller"}
        cfg["scene"]["objects"] = ALL_OBJECT_KEYS
        cfg["scene"]["lane"] = ALL_LANE_KEYS

        # ---- 1: drive into the parked car, collision = stop -------------------
        put_car()
        r = json.loads(json.dumps(cfg))
        r["sync"]["duration"] = 15.0
        r["scene"]["collision"] = "stop"
        c.events.clear()
        c.call("cosim_start", config=r, timeout=120)
        st = run_until_state(c, ("finished", "error", "stopped"), 150)
        recs = records()
        first = recs[0]["car"]
        check("car straight ahead, ego frame (x forward, y left)",
              first is not None and 23.0 < first["rel_x"] < 27.0 and abs(first["rel_y"]) < 0.5 and abs(first["rel_yaw"]) < 3,
              first and "rel_x=%.2f rel_y=%.2f rel_yaw=%.1f" % (first["rel_x"], first["rel_y"], first["rel_yaw"]))
        check("car in the CarSim global frame", first and abs(first["X"] - first["rel_x"]) < 0.5 and abs(first["Y"]) < 0.5,
              first and "X=%.2f Y=%.2f" % (first["X"], first["Y"]))
        check("its size", first and 3.5 < first["length"] < 4.0 and 1.6 < first["width"] < 2.0 and not first["parked"]
              and first["type"] == "vehicle", first and "%.2f x %.2f m, %s" % (first["length"], first["width"], first["model"]))
        mid = next(x for x in recs if x["ex"]["Vx"] > 15.0)
        e = mid["ego"]
        check("ego = CarSim's Xo / Yo / Yaw (reference point)",
              abs(e["X"] - mid["ex"]["Xo"]) < 0.1 and abs(e["Y"] - mid["ex"]["Yo"]) < 0.1 and abs(e["Yaw"] - mid["ex"]["Yaw"]) < 0.5,
              "X %.2f / Xo %.2f, Y %.2f / Yo %.2f, Yaw %.2f / %.2f" % (e["X"], mid["ex"]["Xo"], e["Y"], mid["ex"]["Yo"],
                                                                      e["Yaw"], mid["ex"]["Yaw"]))
        check("speeds in km/h like CarSim", abs(e["Speed"] - mid["ex"]["Vx"]) < 0.5 and abs(mid["car"]["rel_vx"] + mid["ex"]["Vx"]) < 0.5,
              "ego %.2f, Vx %.2f, car rel_vx %.2f km/h" % (e["Speed"], mid["ex"]["Vx"], mid["car"]["rel_vx"]))
        check("scene time = CarSim time", all(abs(x["scene_t"] - x["t"]) < 1e-6 for x in recs[1:]), recs[1]["scene_t"])
        seen = [x for x in recs if x["car"]]
        closing = all(b["car"]["rel_x"] <= a["car"]["rel_x"] + 1e-6 for a, b in zip(seen, seen[1:]))
        check("distance shrinks every frame", closing and len(seen) == len(recs), "%d frames" % len(recs))
        lane = recs[len(recs) // 2]["lane"]
        check("lane ahead", lane and 2.5 < lane["width"] < 4.5 and abs(lane["offset"]) < 0.5 and len(lane["center_rel"]) > 20
              and abs(lane["center_rel"][-1][1]) < 1.0 and abs(lane["curvature"]) < 0.01 and lane["left_lane"] in ("same", "opposite", "none")
              and isinstance(lane["right_marking"], str),
              lane and "width %.1f, offset %.2f, curvature %.4f, left %s / %s, right %s / %s" % (
                  lane["width"], lane["offset"], lane["curvature"], lane["left_lane"], lane["left_marking"],
                  lane["right_lane"], lane["right_marking"]))
        check("obstacles are vehicles and walkers only", set(recs[0]["types"]) <= {"vehicle", "walker"}, recs[0]["types"])
        check("collision stops the run", st["state"] == "finished" and "碰撞" in st.get("detail", "")
              and "audi" in st.get("detail", ""), st.get("detail"))
        last = recs[-1]["car"]
        check("stopped at contact, not before", last and last["gap"] < 0.3, "gap %.2f m" % last["gap"] if last else "")
        tel = [e["data"] for e in c.events if e.get("event") == "telemetry"]
        check("scene in the GUI telemetry", "scene" in tel[-1] and tel[-1]["scene"]["n_objects"] > 0
              and len(json.dumps(tel[-1]["scene"])) < 40000, "%d bytes" % len(json.dumps(tel[-1]["scene"])))
        warns = [e["msg"] for e in c.events if e.get("event") == "log" and e.get("level") == "warn"]
        check("collision in the output", any("碰撞" in m for m in warns), warns[:2])

        # ---- 2: selection, collision = log, run records every 5th frame -------
        put_car()
        r = json.loads(json.dumps(cfg))
        r["sync"]["duration"] = 9.0
        r["scene"].update({"collision": "log", "objects": ["dist"], "lane": [], "ego": ["X", "Speed"],
                           "exports_all": False, "exports": ["Xo", "Vx"]})
        r["collect"]["capture_every"] = 5
        r["run"]["log_path"] = os.path.join(tmp, "run", "mylog.csv")
        c.events.clear()
        c.call("cosim_start", config=r, timeout=120)
        st = run_until_state(c, ("finished", "error", "stopped"), 150)
        warns = [e["msg"] for e in c.events if e.get("event") == "log" and e.get("level") == "warn" and "碰撞" in e["msg"]]
        check("collision = log keeps running", st["state"] == "finished" and "时长" in st.get("detail", "") and warns,
              "%s; %d warnings" % (st.get("detail"), len(warns)))
        recs = records()
        check("only the selected keys", recs[5]["obj_keys"] == ["dist", "id", "type"] and "lane" not in recs[5]["keys"]
              and sorted(recs[5]["ego"]) == ["Speed", "X"], "%s, scene keys %s" % (recs[5]["obj_keys"], recs[5]["keys"]))
        main = read_csv(os.path.join(tmp, "run", "mylog.csv"))
        objs = read_csv(os.path.join(tmp, "run", "mylog_objects.csv"))
        check("run record columns = selection", list(main[0]) == ["t", "frame", "ego_X", "ego_Speed", "Xo", "Vx"]
              and list(objs[0]) == ["t", "frame", "id", "type", "dist"]
              and not os.path.exists(os.path.join(tmp, "run", "mylog_lane.csv")), list(main[0]))
        frames = [int(x["frame"]) for x in main]
        check("run record every 5th frame", all(f % 5 == 0 for f in frames) and 80 <= len(main) <= 95
              and all(b - a == 5 for a, b in zip(frames, frames[1:])), "%d rows" % len(main))
        same_t = {x["frame"]: x["t"] for x in main}
        check("objects rows share the samples", objs and all(o["frame"] in same_t and o["t"] == same_t[o["frame"]] for o in objs),
              "%d object rows" % len(objs))

        # ---- 3: collection: sensors shared, frames/ in step with the sensor files
        put_car()
        r = json.loads(json.dumps(cfg))
        r["sync"]["duration"] = 0.0
        r["scene"].update({"sensors": ["cam"], "collision": "off"})
        r["collect"]["capture_every"] = 2
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
        recs = records()
        s = recs[-1]["sensors"]
        check("only the selected sensor for the algorithm", s == {"cam": ["rgb", [180, 320, 3]]}, s)
        check("one set of sensors for algorithm + collection", during - before == 2, "%d new sensor actors" % (during - before))
        root = info["collect"]["root"]
        stems = {n: sorted(os.path.splitext(os.path.basename(p))[0] for p in glob.glob(os.path.join(root, n, "*")))
                 for n in ("cam", "lidar", "frames")}
        check("frames/ in step with the sensor files", st["state"] == "finished" and len(stems["frames"]) == 5
              and stems["cam"] == stems["lidar"] == stems["frames"] and all(int(x) % 2 == 0 for x in stems["frames"]), stems["frames"])
        fr = json.load(open(os.path.join(root, "frames", stems["frames"][-1] + ".json")))
        check("frame record: CarSim exports + scene", "exports" in fr and "Xo" in fr["exports"] and "ego" in fr
              and "objects" in fr and "lane" in fr and abs(fr["ego"]["X"] - fr["exports"]["Xo"]) < 0.1,
              sorted(fr))
        fcsv, ocsv = read_csv(os.path.join(root, "frames.csv")), read_csv(os.path.join(root, "objects.csv"))
        check("frames.csv / objects.csv", [("%06d" % int(x["frame"])) for x in fcsv] == stems["frames"]
              and ocsv and {o["frame"] for o in ocsv} <= {x["frame"] for x in fcsv}, "%d / %d rows" % (len(fcsv), len(ocsv)))
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

        # ---- 5: the example algorithm (default selection) stops behind the car --
        put_car()
        r = json.loads(json.dumps(cfg))
        r["scene"] = c.call("default_config")["scene"]
        r["scene"]["collision"] = "stop"
        r["sync"]["duration"] = 20.0
        r["run"]["controller"] = {"path": "controllers/scene_controller.py", "entry": "Controller"}
        c.events.clear()
        c.call("cosim_start", config=r, timeout=120)
        st = run_until_state(c, ("finished", "error", "stopped"), 150)
        tel = [e["data"] for e in c.events if e.get("event") == "telemetry"]
        lead = [o for o in tel[-1]["scene"]["objects"] if o["id"] == car.id]
        check("scene_controller.py follows and stops", st["state"] == "finished" and "时长" in st.get("detail", "")
              and tel[-1]["speed_kmh"] < 1.0 and lead and 4.0 < lead[0]["gap"] < 9.0
              and max(t["speed_kmh"] for t in tel) > 8.0,
              "%s; end %.1f km/h, gap %.1f m" % (st.get("detail"), tel[-1]["speed_kmh"], lead[0]["gap"] if lead else -1))
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
