"""Run record and data collection timeline / content against a CARLA server
(tests/test_offline_recording.py has the same checks without one).

Mock CarSim drives with a known control output. Checks: the first control()
call gets an integer frame and the camera's image; the run record starts at
t = 0 and samples every N steps from there, with the same times in a second
run; u1..u3 are the control outputs held over the step before (empty at
t = 0); a tiny collection (5 frames, temp dir deleted afterwards) starts at
t = 0 and has the action and the IMU in CarSim axes in frames/, CarSim's
speed in ego/ (also on stock CARLA) and the map's parked cars in labels/.

    python tests/test_recording_carla.py [--port 2000]
"""
import csv
import glob
import json
import math
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

PORT = 57176

# Logs what the algorithm receives; throttle 0.5, steering wheel 10 deg per s of CarSim time.
CONTROLLER = '''
import json, os
class Controller:
    def reset(self):
        self.out = open(os.environ["REC_TEST_LOG"], "a")
    def control(self, exports, t, dt, scene):
        cam = scene.get("sensors", {}).get("cam")
        shape = list(cam["data"].shape) if cam and cam["data"] is not None else None
        self.out.write(json.dumps({"t": t, "frame": scene["frame"], "cam": shape}) + "\\n")
        self.out.flush()
        return [0.5, 0.0, round(10.0 * t, 6)]
'''

CAM = {"name": "cam", "type": "rgb", "x": 0.5, "y": 0, "z": 1.6, "roll": 0, "pitch": 0, "yaw": 0,
       "attributes": {"image_size_x": 64, "image_size_y": 36, "fov": 90}, "enabled": True}
IMU = {"name": "imu", "type": "imu", "x": 0, "y": 0, "z": 0.8, "roll": 0, "pitch": 0, "yaw": 0,
       "attributes": {}, "enabled": True}


def run_until_state(c, states, timeout=120):
    return c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in states, timeout)


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def spawn_near_parked(w):
    """A spawn point with one of the map's parked cars 8-40 m away (None if the map has none)."""
    cars = []
    for lab in ("Car", "Truck", "Bus"):
        cars += [o.bounding_box.location for o in w.get_environment_objects(getattr(carla.CityObjectLabel, lab))]
    for i, p in enumerate(w.get_map().get_spawn_points()):
        d = [c.distance(p.location) for c in cars]
        if d and 8.0 < min(d) < 40.0:
            return i
    return None


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    tmp = tempfile.mkdtemp(prefix="cc_rec_test_")
    log = os.path.join(tmp, "control.jsonl")
    os.environ["REC_TEST_LOG"] = log
    proc = subprocess.Popen([sys.executable, os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT)],
                            cwd=os.path.join(HERE, ".."), env=dict(os.environ))
    try:
        c = Conn(PORT)
        c.call("connect", host="localhost", port=carla_port, timeout=60)
        w = carla.Client("localhost", carla_port).get_world()
        sp = spawn_near_parked(w)
        with open(os.path.join(tmp, "ctrl.py"), "w") as f:
            f.write(CONTROLLER)
        cfg = c.call("default_config")
        cfg["carsim"]["mock"] = True
        cfg["carla"]["spawn_index"] = sp or 0
        cfg["run"]["driver"] = "custom"
        cfg["run"]["controller"] = {"path": os.path.join(tmp, "ctrl.py"), "entry": "Controller"}
        cfg["sync"]["frame_dt"] = 0.02
        cfg["collect"]["sample_period"] = 0.1   # every 5 steps
        cfg["scene"].update({"sensors": ["cam"], "collision": "off"})
        cfg["scene"]["record"]["ego"] = ["X", "Speed"]
        cfg["rig"]["sensors"] = [dict(CAM)]

        # ---- 1 + 2: two runs of the same scenario, run record -----------------
        runs = []
        for k in (1, 2):
            if os.path.exists(log):
                os.remove(log)
            r = json.loads(json.dumps(cfg))
            r["sync"]["duration"] = 2.0
            r["run"]["log_path"] = os.path.join(tmp, "run%d.csv" % k)
            c.events.clear()
            c.call("cosim_start", config=r, timeout=120)
            st = run_until_state(c, ("finished", "error", "stopped"), 120)
            check("run %d finished" % k, st["state"] == "finished", st.get("detail"))
            calls = [json.loads(line) for line in open(log)]
            runs.append((read_csv(r["run"]["log_path"]), calls))
        main1, calls = runs[0]
        check("first control(): integer frame and the camera image",
              isinstance(calls[0]["frame"], int) and calls[0]["cam"] == [36, 64, 3], calls[0])
        check("run record starts at t = 0, every 5 steps",
              [float(x["t"]) for x in main1[:3]] == [0.0, 0.1, 0.2] and len(main1) == 21
              and all(abs(float(x["t"]) - i * 0.1) < 1e-6 for i, x in enumerate(main1)), "%d rows (2 s: steps 0, 5 ... 100)" % len(main1))
        check("same sample times in a second run", [x["t"] for x in runs[1][0]] == [x["t"] for x in main1]
              and runs[1][0][0]["frame"] != main1[0]["frame"],
              "world frames %s / %s" % (main1[0]["frame"], runs[1][0][0]["frame"]))
        check("control columns u1..u3", list(main1[0])[-3:] == ["u1", "u2", "u3"]
              and [main1[0][u] for u in ("u1", "u2", "u3")] == ["", "", ""], list(main1[0]))
        held = all(abs(float(x["u1"]) - 0.5) < 1e-9 and abs(float(x["u3"]) - 10.0 * (float(x["t"]) - 0.02)) < 1e-3
                   for x in main1[1:])
        check("u = the control output held over the step before", held, main1[1])

        # ---- 3: tiny collection (5 frames), deleted with the temp dir --------------
        r = json.loads(json.dumps(cfg))
        r["sync"]["duration"] = 0.0
        r["run"]["log_path"] = ""
        r["rig"]["sensors"] = [dict(CAM), dict(IMU)]
        r["collect"].update({"enabled": True, "out_dir": tmp, "session": "rec", "max_frames": 5, "max_gb": 0.1,
                             "sample_period": 0.2})
        c.events.clear()
        info = c.call("cosim_start", config=r, timeout=120)
        st = run_until_state(c, ("finished", "error", "stopped"), 120)
        root = info["collect"]["root"]
        stems = sorted(os.path.splitext(os.path.basename(p))[0] for p in glob.glob(os.path.join(root, "frames", "*.json")))
        check("collection finished with 5 frames", st["state"] == "finished" and len(stems) == 5
              and all(int(b) - int(a) == 10 for a, b in zip(stems, stems[1:])), stems)
        fr = [json.load(open(os.path.join(root, "frames", s + ".json"))) for s in stems]
        eg = [json.load(open(os.path.join(root, "ego", s + ".json"))) for s in stems]
        check("first sample at t = 0, no action yet", fr[0]["t"] == 0.0 and fr[0]["action"] is None
              and fr[1]["action"][:2] == [0.5, 0.0], (fr[0]["t"], fr[0].get("action"), fr[1].get("action")))
        imu, raw = fr[-1]["sensors"]["imu"]["data"], eg[-1]["imu"]
        check("IMU in frames/: CarSim axes (y left) and deg/s",
              abs(imu["accel"][1] + raw["accelerometer"][1]) < 1e-6
              and abs(imu["gyro"][2] + math.degrees(raw["gyroscope"][2])) < 1e-4, (imu, raw))
        v = eg[-1]["velocity"]
        speed = math.hypot(v[0], v[1])
        check("ego/ velocity = CarSim's (stock CARLA reads 0)",
              fr[-1]["ego"]["Speed"] > 1.0 and abs(speed * 3.6 - fr[-1]["ego"]["Speed"]) < 0.2,
              "%.2f km/h in ego/, %.2f km/h in frames/" % (speed * 3.6, fr[-1]["ego"]["Speed"]))
        fcsv = read_csv(os.path.join(root, "frames.csv"))
        check("frames.csv: t0 row, u columns", fcsv[0]["t"] == "0.0" and fcsv[0]["u1"] == "" and fcsv[1]["u1"] == "0.5",
              fcsv[:2])
        if sp is None:
            print("INFO no parked cars in this map: labels/ check skipped")
        else:
            labs = json.load(open(os.path.join(root, "labels", stems[0] + ".json")))["objects"]
            check("labels/ include the map's parked cars", any(o["type_id"].startswith("map.") for o in labs),
                  sorted({o["type_id"] for o in labs})[:8])
            check("labels/: no riderless parked bikes", not any(o["type_id"] in ("map.Bicycle", "map.Motorcycle")
                                                                for o in labs), sorted({o["type_id"] for o in labs})[:8])
        c.call("destroy_ego")
        print("ALL RECORDING TESTS PASSED")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
        proc.terminate()
        proc.wait(10)


if __name__ == "__main__":
    main()
