"""Tests for rigs, driving modes and data collection (tiny captures only).

Data collection writes at most 3 frames into a temp dir that is deleted at
the end, so the test never uses meaningful disk space.

    python tests/test_features.py [--port 2000]
"""
import glob
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from test_backend import Conn, check  # noqa: E402

PORT = 57198


def run_until_state(c, states, timeout=120):
    return c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in states, timeout)


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    proc = subprocess.Popen([sys.executable, os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT)],
                            cwd=os.path.join(HERE, ".."))
    tmp = tempfile.mkdtemp(prefix="cosim_test_ds_")
    try:
        c = Conn(PORT)
        c.call("connect", host="localhost", port=carla_port)

        # ---- rigs ------------------------------------------------------------
        presets = c.call("rig_presets")
        check("rig_presets", len(presets) == 5, [p["id"] for p in presets])
        nus = c.call("rig_build", preset="nuscenes", blueprint="vehicle.tesla.model3")
        kinds = sorted({s["type"] for s in nus})
        check("nuscenes rig", len(nus) == 12 and kinds == ["lidar", "radar", "rgb"], "%d sensors" % len(nus))
        roof = max(s["z"] for s in nus if s["type"] == "rgb")
        check("rig fitted to vehicle height", 1.4 < roof < 1.7, "camera z = %.2f m" % roof)
        est = c.call("rig_estimate", sensors=nus, collect={"max_frames": 100}, frame_dt=0.1)
        check("rig_estimate", 15 < est["mb_per_s"] < 60, "%.1f MB/s, %.0f GB/h, disk free %.0f GB" % (
            est["mb_per_s"], est["gb_per_hour"], est["disk"]["free_gb"]))

        cfg = c.call("default_config")
        cfg["carla"]["spawn_index"] = 3
        cfg["sync"]["frame_dt"] = 0.05
        cfg["run"]["log_path"] = ""

        # ---- disk guard: absurd request must be refused before anything spawns
        big = json.loads(json.dumps(cfg))
        big["collect"].update({"enabled": True, "out_dir": tmp, "max_frames": 0, "max_seconds": 0, "max_gb": 0})
        try:
            c.call("cosim_start", config=big)
            check("refuses collection without a limit", False)
        except RuntimeError as e:
            check("refuses collection without a limit", "停止条件" in str(e))
        big["collect"].update({"max_gb": 100000})
        try:
            c.call("cosim_start", config=big)
            check("refuses collection larger than free disk", False)
        except RuntimeError as e:
            check("refuses collection larger than free disk", "磁盘" in str(e), str(e)[:40])

        # ---- CARLA physics + route follower -----------------------------------
        r = json.loads(json.dumps(cfg))
        r["drive"].update({"dynamics": "carla", "carla_driver": "route", "target_speed_kmh": 30})
        r["sync"]["duration"] = 12.0
        c.call("cosim_start", config=r)
        tel = c.wait_event(lambda e: e.get("event") == "telemetry" and e["data"]["t"] > 11.0, 180)["data"]
        run_until_state(c, ("finished", "error"))
        check("CARLA route follower drives", 18 < tel["speed_kmh"] < 40, "%.1f km/h at t=%.1f" % (tel["speed_kmh"], tel["t"]))

        # ---- CarSim (mock) + route follower: must stay on the road -------------
        r = json.loads(json.dumps(cfg))
        r["carsim"]["mock"] = True
        r["drive"].update({"dynamics": "cosim", "target_speed_kmh": 30})
        r["run"]["driver"] = "route"
        r["sync"]["duration"] = 10.0
        r["sync"]["frame_dt"] = 0.02
        c.events.clear()
        c.call("cosim_start", config=r)
        tels = []
        while True:
            e = c.wait_event(lambda e: e.get("event") in ("telemetry", "cosim_state"), 180)
            c.events.remove(e)
            if e["event"] == "cosim_state" and e["state"] in ("finished", "error"):
                break
            if e["event"] == "telemetry":
                tels.append(e["data"])
        moved = math.dist(tels[0]["location"][:2], tels[-1]["location"][:2])
        steer_used = max(abs(t["wheel_steer"][0]) for t in tels)
        check("CarSim route follower drives", moved > 25 and tels[-1]["speed_kmh"] > 15,
              "moved %.0f m, %.1f km/h, max wheel steer %.1f°" % (moved, tels[-1]["speed_kmh"], steer_used))

        # ---- manual keyboard control in CARLA physics --------------------------
        r = json.loads(json.dumps(cfg))
        r["drive"].update({"dynamics": "carla", "carla_driver": "manual"})
        r["sync"]["duration"] = 0.0
        c.call("cosim_start", config=r)
        for _ in range(40):
            c.call("manual_control", throttle=0.8, brake=0.0, steer=0.0)
            time.sleep(0.05)
        tel = c.wait_event(lambda e: e.get("event") == "telemetry" and e["data"]["speed_kmh"] > 3.0, 60)["data"]
        check("manual control accelerates", tel["speed_kmh"] > 3.0, "%.1f km/h" % tel["speed_kmh"])
        c.call("cosim_stop")

        # ---- CARLA autopilot (traffic manager) --------------------------------
        r = json.loads(json.dumps(cfg))
        # Ignore lights so the check does not depend on the light cycle.
        r["drive"].update({"dynamics": "carla", "carla_driver": "autopilot", "tm_ignore_lights": True})
        r["sync"]["duration"] = 8.0
        c.events.clear()
        c.call("cosim_start", config=r)
        tel = c.wait_event(lambda e: e.get("event") == "telemetry" and e["data"]["t"] > 7.5, 120)["data"]
        run_until_state(c, ("finished", "error"))
        check("autopilot drives", tel["speed_kmh"] > 5.0, "%.1f km/h" % tel["speed_kmh"])

        # ---- tiny collection: 3 frames, every sensor type ---------------------
        sensors = c.call("rig_build", preset="perception_gt", blueprint="vehicle.tesla.model3")
        for s in sensors:
            s["attributes"].update({"image_size_x": 640, "image_size_y": 360})
        sensors += [s for s in nus if s["name"] in ("lidar_top", "radar_front")]
        sensors.append({"name": "imu", "type": "imu", "x": 0, "y": 0, "z": 0.8, "roll": 0, "pitch": 0, "yaw": 0,
                        "attributes": {}, "enabled": True})
        r = json.loads(json.dumps(cfg))
        r["drive"].update({"dynamics": "carla", "carla_driver": "route", "target_speed_kmh": 20})
        r["sync"].update({"frame_dt": 0.1, "duration": 0.0})
        r["rig"]["sensors"] = sensors
        r["collect"].update({"enabled": True, "out_dir": tmp, "session": "t", "max_frames": 3, "max_gb": 0.2})
        c.events.clear()
        info = c.call("cosim_start", config=r)
        run_until_state(c, ("finished", "error"), 180)
        root = info["collect"]["root"]
        counts = {s["name"]: len(glob.glob(os.path.join(root, s["name"], "*"))) for s in sensors if s["type"] != "imu"}
        check("3 frames per sensor", all(n == 3 for n in counts.values()), counts)
        calib = json.load(open(os.path.join(root, "calib.json")))
        k = calib["sensors"]["cam_rgb"]["K"]
        check("calib intrinsics", abs(k[0][0] - 320.0) < 1e-3 and abs(k[0][2] - 320.0) < 1e-3, "fx=%.1f cx=%.1f" % (k[0][0], k[0][2]))
        ego = json.load(open(sorted(glob.glob(os.path.join(root, "ego", "*.json")))[-1]))
        check("ego state + imu", "imu" in ego and "pose" in ego, sorted(ego)[:6])
        labels = json.load(open(sorted(glob.glob(os.path.join(root, "labels", "*.json")))[-1]))
        check("labels file", "objects" in labels, "%d objects" % len(labels["objects"]))
        pts = os.path.getsize(sorted(glob.glob(os.path.join(root, "lidar_top", "*.bin")))[-1])
        check("lidar full sweep", pts > 200000, "%.0f KB/frame" % (pts / 1024))
        sizes = {n: sum(os.path.getsize(p) for p in glob.glob(os.path.join(root, n, "*"))) / 3 / 1024
                 for n in counts}
        print("measured KB/frame:", {k: round(v) for k, v in sizes.items()})
        c.call("destroy_ego")
        print("ALL FEATURE TESTS PASSED")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
        proc.terminate()
        proc.wait(10)


if __name__ == "__main__":
    main()
