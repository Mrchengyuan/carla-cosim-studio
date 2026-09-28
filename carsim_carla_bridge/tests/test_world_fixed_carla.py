"""固定世界 (config world.fixed) with a CARLA server: a run on another map
first loads the config's map, sets its weather and rebuilds its traffic
(cleared, then the same number of vehicles and walkers from the same seed),
says so and records the world block in run.json; the same config run twice
gives the traffic the same positions at the end (CARLA in synchronous mode,
the traffic manager seeded); with fixed off nothing about the world changes.

    python tests/test_world_fixed_carla.py [--port 2000]
"""
import csv
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
import carla  # noqa: E402

from test_backend import Conn  # noqa: E402

PORT = 57151
FAILS = []
WEATHER = {"cloudiness": 80.0, "precipitation": 30.0, "precipitation_deposits": 40.0, "wind_intensity": 20.0,
           "sun_azimuth_angle": 120.0, "sun_altitude_angle": 25.0, "fog_density": 10.0, "wetness": 50.0}


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + (("  " + str(detail)) if detail != "" else ""), flush=True)
    if not cond:
        FAILS.append(name)


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    tmp = tempfile.mkdtemp(prefix="cc_world_test_")
    proc = subprocess.Popen([sys.executable, os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT)],
                            cwd=os.path.join(HERE, ".."))
    client = carla.Client("localhost", carla_port)
    client.set_timeout(60)
    try:
        c = Conn(PORT)
        c.call("connect", host="localhost", port=carla_port, timeout=60)
        if c.call("world_info")["map"] != "Town10HD_Opt":
            c.call("load_map", name="Town10HD_Opt", timeout=400)
        cfg = c.call("default_config")
        check("world.fixed is off by default", cfg["world"]["fixed"] is False, cfg["world"])
        cfg["carla"]["spawn_index"] = 5
        cfg["scene"]["record"]["objects"] = ["id", "type", "model", "X", "Y", "dist"]
        cfg["carsim"].update(mock=True)
        cfg["run"].update(driver="demo", log_path=os.path.join(tmp, "runs"))
        cfg["sync"].update(frame_dt=0.05, duration=8.0)
        cfg["collect"]["sample_period"] = 0.5
        cfg["scene"]["record"]["objects"] = ["id", "type", "model", "X", "Y", "dist"]
        cfg["world"] = {"fixed": True, "map": "Town04_Opt", "weather": WEATHER,
                        "traffic": {"vehicles": 10, "walkers": 5, "seed": 7}}

        def run():
            c.events.clear()
            info = c.call("cosim_start", config=cfg, timeout=900)
            st = c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in ("finished", "error", "stopped"), 900)
            logs = [e["msg"] for e in c.events if e.get("event") == "log"]
            return info, st, logs

        info, st, logs = run()
        wi = c.call("world_info")
        check("the run ended normally", st["state"] == "finished", st)
        check("on another map: the config's map loaded first", wi["map"] == "Town04_Opt" and
              any(m.startswith("按配置加载地图 Town04_Opt") for m in logs), (wi["map"], [m for m in logs if "地图" in m][:2]))
        w = wi["weather"]
        check("the config's weather", all(abs(w[k] - v) < 0.01 for k, v in WEATHER.items()),
              {k: w.get(k) for k in WEATHER})
        vs = [a for a in client.get_world().get_actors().filter("vehicle.*") if a.attributes.get("role_name") == "autopilot"]
        ws = client.get_world().get_actors().filter("walker.pedestrian.*")
        check("the config's traffic: 10 vehicles, 5 walkers", len(vs) == 10 and len(ws) == 5, (len(vs), len(ws)))
        check("the output says what was rebuilt",
              any(m.startswith("按配置重建世界：地图 Town04_Opt，天气，交通流 10 辆车、5 个行人（种子 7）") for m in logs),
              [m for m in logs if "重建" in m])
        rj = json.load(open(os.path.join(info["record_dir"], "run.json"), encoding="utf-8"))
        check("run.json records the world", (rj.get("world") or {}).get("traffic", {}).get("seed") == 7 and rj["traffic_seed"] == 7,
              (rj.get("world"), rj.get("traffic_seed")))

        def end_positions(folder):
            with open(os.path.join(folder, "log_objects.csv"), newline="", encoding="utf-8") as f:
                rows = list(csv.DictReader(f))
            last = max(float(r["t"]) for r in rows)
            return sorted((round(float(r["X"]), 2), round(float(r["Y"]), 2)) for r in rows
                          if float(r["t"]) == last and r["type"] in ("vehicle", "walker"))

        first = end_positions(info["record_dir"])
        info2, st2, _ = run()
        second = end_positions(info2["record_dir"])
        diff = max((math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(first, second)), default=None)
        check("the same config twice: the traffic and walkers near the car end in the same places (< 5 cm)",
              first and len(first) == len(second) and diff is not None and diff < 0.05, (len(first), len(second), diff))

        # Off: the world stays as it is.
        cfg["world"]["fixed"] = False
        cfg["world"]["map"] = "Town10HD_Opt"
        before = len(client.get_world().get_actors().filter("vehicle.*"))
        info3, st3, logs3 = run()
        check("fixed off: no map change, no traffic rebuilt", c.call("world_info")["map"] == "Town04_Opt" and
              not any("按配置" in m for m in logs3), [m for m in logs3 if "按配置" in m])
        c.call("clear_traffic")
        c.call("destroy_ego")
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            proc.kill()
        shutil.rmtree(tmp, ignore_errors=True)
    print("FAILED: %s" % FAILS if FAILS else "ALL WORLD TESTS PASSED")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
