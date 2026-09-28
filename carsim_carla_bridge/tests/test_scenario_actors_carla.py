"""测试场景 moving actors (动态目标, scenario.py Movers) on the Town04 highway
with a CARLA server, the mock CarSim (starting at 11 m/s) driven by
controllers/path_follower.py at 40 km/h (it follows its lane and does not
avoid anything).

Checks, from the run records (log_objects.csv with the objects' model, speed
and position): a slow car ahead keeps its speed (the algorithm sees it: the
velocity reaches the scene though stock CARLA reads 0 for placed actors) and
its lane; a car ahead brakes to a stop once the ego is within its trigger
distance (and the lane follower hits it); a car in the next lane moves into
the ego's lane once triggered; a pedestrian crosses the lane at walking
speed once triggered; the same scenario twice gives the same actor
positions; a cut-in from a lane that is not there refuses the run before
anything is placed; the next run removes them. The actors' lanes are the
map's lane ids where they stand, sampled from CARLA while the run goes (on
a curved road the ego frame's rel_y is not the lane).

    python tests/test_scenario_actors_carla.py [--port 2000]
"""
import csv
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
sys.path.insert(0, os.path.join(HERE, ".."))
import carla  # noqa: E402

import scenario  # noqa: E402
from test_backend import Conn  # noqa: E402

PORT = 57149
FOLLOWER = os.path.abspath(os.path.join(HERE, "..", "controllers", "path_follower.py"))
CONTROLLER = '''import importlib.util
spec = importlib.util.spec_from_file_location("path_follower", %r)
PF = importlib.util.module_from_spec(spec)
spec.loader.exec_module(PF)
PF.WHEELBASE, PF.STEER_RATIO_INIT, PF.BRAKE_MAX = 2.9, 16.0, 1.0
Controller = PF.Controller
''' % FOLLOWER
FAILS = []


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + (("  " + str(detail)) if detail != "" else ""), flush=True)
    if not cond:
        FAILS.append(name)


def rows(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    tmp = tempfile.mkdtemp(prefix="cc_actors_test_")
    ctrl = os.path.join(tmp, "follower_mock.py")
    with open(ctrl, "w", encoding="utf-8") as f:
        f.write(CONTROLLER)
    proc = subprocess.Popen([sys.executable, os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT)],
                            cwd=os.path.join(HERE, ".."))
    client = carla.Client("localhost", carla_port)
    client.set_timeout(60)
    try:
        c = Conn(PORT)
        c.call("connect", host="localhost", port=carla_port, timeout=60)
        if "Town04" not in c.call("world_info")["map"]:
            c.call("load_map", name="Town04_Opt", timeout=400)
        cmap = client.get_world().get_map()
        start = c.call("world_info")["scenario_start"]
        cfg = c.call("default_config")
        cfg["carla"]["spawn_index"] = start["index"]
        cfg["carsim"].update(mock=True, mock_init_speed=11.0)
        cfg["run"].update(driver="custom", controller={"path": ctrl, "entry": "Controller"}, log_path=os.path.join(tmp, "runs"))
        cfg["sync"]["frame_dt"] = 0.05
        cfg["collect"]["sample_period"] = 0.1
        cfg["scene"]["collision"] = "log"
        cfg["scene"]["record"]["objects"] = ["id", "type", "model", "X", "Y", "Speed", "rel_x", "rel_y", "dist", "gap"]

        def run(actors, duration):
            """The run, and the lane (CARLA lane id on the map) each moving actor is in, sampled while it runs:
            on a curved road the ego frame's rel_y is not the lane."""
            cfg["scenario"] = {"enabled": True, "closures": [], "actors": actors}
            cfg["sync"]["duration"] = duration
            c.events.clear()
            info = c.call("cosim_start", config=cfg, timeout=300)
            lanes.clear()
            st = None
            end = time.time() + 1200
            while st is None and time.time() < end:
                try:
                    st = c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in ("finished", "error", "stopped"), 0.5)
                except Exception:
                    st = None
                for a in scenario.leftovers(client.get_world()):
                    if a.type_id.startswith("vehicle."):
                        lanes.append(cmap.get_waypoint(a.get_location()).lane_id)
            folder = info["record_dir"]
            run_json = json.load(open(os.path.join(folder, "run.json"), encoding="utf-8"))
            objs = [r for r in rows(os.path.join(folder, "log_objects.csv")) if r["model"] == scenario.CAR_MODEL
                    or r["model"] == scenario.WALKER_MODEL]
            logs = [e["msg"] for e in c.events if e.get("event") == "log"]
            return st, run_json, objs, logs

        lanes = []
        ego_lane = cmap.get_waypoint(cmap.get_spawn_points()[start["index"]].location).lane_id
        left_lane = cmap.get_waypoint(cmap.get_spawn_points()[start["index"]].location).get_left_lane().lane_id

        # ---- 1: a slow car ahead: its speed held, seen by the algorithm, in its lane --------------
        st, rj, objs, logs = run([dict(type="slow_car", distance_m=60, lane=0, speed_kmh=30, trigger_m=0, param=0)], 20.0)
        sp = [float(r["Speed"]) for r in objs]
        check("slow car: placed and said so", any(m.startswith("测试场景动态目标：第 1 个前车慢行，出生点前方 60 m") for m in logs),
              [m for m in logs if "动态目标" in m][:1])
        check("slow car: the algorithm's scene has it at 30 km/h (not 0)", len(sp) > 50 and all(abs(v - 30.0) < 0.5 for v in sp),
              (len(sp), min(sp) if sp else None, max(sp) if sp else None))
        check("slow car: run.json lists it", rj["scenario"]["actors"][0]["type"] == "slow_car" and rj["scenario"]["actors"][0]["placed"],
              rj["scenario"])
        check("slow car: in the ego's lane all the way (the map's lane id)", len(lanes) >= 3 and set(lanes) == {ego_lane},
              (len(lanes), sorted(set(lanes)), ego_lane))

        # ---- 2: a car that brakes once the ego is near: to a stop; the lane follower runs into it ----
        st, rj, objs, logs = run([dict(type="lead_brake", distance_m=60, lane=0, speed_kmh=25, trigger_m=25, param=4)], 30.0)
        sp = [float(r["Speed"]) for r in objs]
        k = rj["kpi"] or {}
        gaps = [float(r["dist"]) for r in objs]
        check("lead brake: 25 km/h until the ego closes in, then down to 0",
              sp and abs(sp[0] - 25.0) < 0.5 and min(sp) < 0.1, (sp[:1], min(sp) if sp else None))
        i0 = next((i for i, v in enumerate(sp) if v < 24.5), None)
        check("... it starts braking when the ego is within 25 m (+ car lengths)", i0 is not None and gaps[i0] < 25.0 + 6.0,
              (i0, gaps[i0] if i0 is not None else None))
        check("... the lane follower does not stop: a collision with it",
              (k.get("collisions") or 0) > 0 and str(k.get("first_collision_with")).startswith("vehicle."), k.get("first_collision_with"))

        # ---- 3: a cut-in from the left lane into the ego's ------------------------------------------
        cut = [dict(type="cut_in", distance_m=70, lane=-1, speed_kmh=30, trigger_m=35, param=3)]
        st, rj, objs, logs = run(cut, 25.0)
        check("cut-in: starts in the lane to the left and ends in the ego's lane (the map's lane ids)",
              len(lanes) > 5 and lanes[0] == left_lane and lanes[-1] == ego_lane, (lanes[:1], lanes[-1:], left_lane, ego_lane))
        first_xy = [(r["t"], round(float(r["X"]), 3), round(float(r["Y"]), 3)) for r in objs]
        # ---- the same scenario again: the same positions --------------------------------------------
        st2, rj2, objs2, _ = run(cut, 25.0)
        again = [(r["t"], round(float(r["X"]), 3), round(float(r["Y"]), 3)) for r in objs2]
        check("cut-in twice: the same actor positions at the same times", first_xy == again and len(first_xy) > 50,
              (len(first_xy), len(again)))

        # ---- 4: a pedestrian crossing from the right at 5 km/h --------------------------------------
        st, rj, objs, logs = run([dict(type="pedestrian", distance_m=100, lane=1, speed_kmh=5, trigger_m=40, param=0)], 20.0)
        ry = [float(r["rel_y"]) for r in objs if float(r["rel_x"]) > 0]
        sp = [float(r["Speed"]) for r in objs]
        check("pedestrian: waits at the right, crosses to the left once the ego is near",
              ry and ry[0] < -1.5 and max(ry) > 1.0, (ry[:1], max(ry) if ry else None))
        check("... at 5 km/h while crossing (0 while waiting)", sp and min(sp) < 0.1 and abs(max(sp) - 5.0) < 0.3,
              (min(sp) if sp else None, max(sp) if sp else None))

        # ---- a lane that is not there: refused before anything is placed ---------------------------------
        cfg["scenario"] = {"enabled": True, "closures": [], "actors": [dict(type="cut_in", distance_m=50, lane=-2, speed_kmh=30,
                                                                             trigger_m=30, param=3)]}
        try:
            c.call("cosim_start", config=cfg, timeout=120)
            err = ""
        except Exception as e:
            err = str(e)
        check("cut-in from a lane that is not there: refused in plain words", "那里没有左侧第 2 条车道" in err, err)
        cfg["scenario"] = {"enabled": False, "closures": [], "actors": []}
        cfg["sync"]["duration"] = 1.0
        c.events.clear()
        c.call("cosim_start", config=cfg, timeout=300)
        c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in ("finished", "error", "stopped"), 300)
        check("the next run without them: none left", not scenario.leftovers(client.get_world()),
              len(scenario.leftovers(client.get_world())))
        c.call("destroy_ego")
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            proc.kill()
        shutil.rmtree(tmp, ignore_errors=True)
    print("FAILED: %s" % FAILS if FAILS else "ALL SCENARIO ACTOR TESTS PASSED")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
