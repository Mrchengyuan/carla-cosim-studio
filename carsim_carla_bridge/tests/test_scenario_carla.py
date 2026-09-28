"""测试场景 (scenario.py) on the Town04 highway with a CARLA server: lane
closures placed from the highway start (four lanes one way, the car in the
second from the left: spawn point 41 in the 0.9.16 release, 39 in a build
from source; scenario.find_start, given by world_info), the mock CarSim
driven by controllers/path_follower.py (lane following, no avoidance: it
drives into a closure of its own lane).

Checks: world_info's scenario_start; every preset of the GUI places its
props, each where planned on the lane it closes; run.json records them; the
algorithm gets them as scene objects of type "static" and the records keep
them; driving into the closure is a collision with a static prop; the props
stay until the next run starts, which removes them and any another backend
left (a run without the scenario leaves none); a lane that is not there
refuses the run before anything is placed; a backend killed with its props
in the world: the next one (connect with recover, as the GUI's "重启后端")
removes them.

    python tests/test_scenario_carla.py [--port 2000]
"""
import csv
import json
import os
import shutil
import signal
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

PORT = 57144
FOLLOWER = os.path.abspath(os.path.join(HERE, "..", "controllers", "path_follower.py"))
# path_follower with the mock CarSim's vehicle, counting the static objects it is handed.
CONTROLLER = '''import importlib.util
spec = importlib.util.spec_from_file_location("path_follower", %r)
PF = importlib.util.module_from_spec(spec)
spec.loader.exec_module(PF)
PF.WHEELBASE, PF.STEER_RATIO_INIT, PF.BRAKE_MAX = 2.9, 16.0, 1.0


class Controller(PF.Controller):
    def reset(self):
        super().reset()
        self.seen, self.nearest = 0, None

    def control(self, exports, t, dt, scene):
        st = [o for o in scene["objects"] if o["type"] == "static"]
        self.seen = max(self.seen, len(st))
        if st:
            g = min(o["gap"] for o in st)
            self.nearest = g if self.nearest is None else min(self.nearest, g)
        return super().control(exports, t, dt, scene)

    def finish(self, reason):
        print("STATIC_SEEN %%d NEAREST %%s" %% (self.seen, self.nearest))
        super().finish(reason)
''' % FOLLOWER
# The GUI's presets (panels.cpp DrawPanelTestScene).
PRESETS = {
    "封闭本车道（锥桶）": [dict(distance_m=250, lane=0, taper_m=40, length_m=100, kind="cones")],
    "封闭左侧车道（护栏）": [dict(distance_m=250, lane=-1, taper_m=30, length_m=120, kind="barrier")],
    "连续两处封道": [dict(distance_m=250, lane=0, taper_m=40, length_m=80, kind="cones"),
                   dict(distance_m=550, lane=-1, taper_m=40, length_m=80, kind="cones")],
    "只剩一条车道": [dict(distance_m=300, lane=k, taper_m=40, length_m=100, kind="cones") for k in (0, 1, 2)],
}
FAILS = []


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + (("  " + str(detail)) if detail != "" else ""), flush=True)
    if not cond:
        FAILS.append(name)


def backend(port):
    return subprocess.Popen([sys.executable, os.path.join(HERE, "..", "backend_server.py"), "--port", str(port)],
                            cwd=os.path.join(HERE, ".."), env=dict(os.environ))


def props_in(world):
    return scenario.leftovers(world)


def run(c, cfg, timeout=600):
    c.events.clear()
    info = c.call("cosim_start", config=cfg, timeout=300)
    st = c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in ("finished", "error", "stopped"), timeout)
    return info, st


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    tmp = tempfile.mkdtemp(prefix="cc_scenario_test_")
    ctrl = os.path.join(tmp, "follower_mock.py")
    with open(ctrl, "w", encoding="utf-8") as f:
        f.write(CONTROLLER)
    proc = backend(PORT)
    client = carla.Client("localhost", carla_port)
    client.set_timeout(60)
    try:
        c = Conn(PORT)
        c.call("connect", host="localhost", port=carla_port, timeout=60)
        if "Town04" not in c.call("world_info")["map"]:
            c.call("load_map", name="Town04_Opt", timeout=400)
        world = client.get_world()
        cmap = world.get_map()
        start = c.call("world_info")["scenario_start"]
        print("INFO map %s, start %s" % (cmap.name, start))
        SPAWN = start["index"]
        wp = cmap.get_waypoint(cmap.get_spawn_points()[SPAWN].location)
        check("the highway start: four lanes one way, the car in the second from the left, > 800 m to a junction",
              (start["lanes"], start["from_left"], scenario._lanes_around(wp)) == (4, 2, [1, 2])
              and start["free_m"] > 800 and wp.road_id == 45, (start, wp.road_id, wp.lane_id))
        cfg = c.call("default_config")
        cfg["carsim"]["mock"] = True
        cfg["carla"]["spawn_index"] = SPAWN
        cfg["run"]["driver"] = "custom"
        cfg["run"]["controller"] = {"path": ctrl, "entry": "Controller"}
        cfg["run"]["log_path"] = os.path.join(tmp, "runs")
        cfg["collect"]["sample_period"] = 0.1
        cfg["scene"]["collision"] = "log"
        check("static objects are handed to the algorithm by default", "static" in cfg["scene"]["object_types"],
              cfg["scene"]["object_types"])

        # ---- every preset: placed, on its lanes, in run.json -----------------
        spawn = cmap.get_spawn_points()[SPAWN]
        # Props another backend left (killed, never reconnected with recover): a run starts without them.
        foreign = scenario.spawn(client, world, scenario.layout(cmap, spawn, [dict(
            distance_m=700, lane=1, taper_m=10, length_m=10, kind="barrier")])[0])
        for name, closures in PRESETS.items():
            cfg["scenario"] = {"enabled": True, "closures": closures}
            cfg["sync"]["duration"] = 1.0
            info, st = run(c, cfg)
            folder = info.get("record_dir") or ""
            meta = json.load(open(os.path.join(folder, "run.json"), encoding="utf-8"))["scenario"]
            world = client.get_world()
            placed = props_in(world)
            items, summary = scenario.layout(cmap, spawn, closures)
            models = {a.type_id for a in placed}
            want_model = {"cones": "static.prop.constructioncone", "barrier": "static.prop.streetbarrier"}
            check("%s: the run ended normally" % name, st["state"] == "finished", st)
            if foreign:
                ids = {a.id for a in placed}
                check("props another backend left are removed when a run starts",
                      len(foreign) > 5 and not ids & set(foreign), (len(foreign), len(ids & set(foreign))))
                foreign = None
            check("%s: all %d props placed, run.json says so" % (name, len(items)),
                  len(placed) == len(items) == meta["placed"] == sum(s["props"] for s in meta["closures"]),
                  (len(placed), len(items), meta["placed"]))
            check("%s: the kinds of props" % name,
                  models == {want_model[x["kind"]] for x in closures} | {scenario.ARROW}, models)
            # Each prop where planned, on the lane it closes (the lane id where it stands; lane ids
            # can change from one road to the next, so on the closure's own road).
            off, moved = 0, 0.0
            for a in placed:
                loc = a.get_location()
                model, tf, num = min(items, key=lambda i: i[1].location.distance(loc))
                moved = max(moved, tf.location.distance(loc))
                wp = cmap.get_waypoint(loc)
                s = summary[num - 1]
                if wp.road_id == s["road_id"] and wp.lane_id != s["lane_id"]:
                    off += 1
            check("%s: every prop where planned (< 0.1 m), on the lane it closes" % name, off == 0 and moved < 0.1,
                  (off, round(moved, 3)))

        # ---- the algorithm gets them; driving into the closure is a collision --------
        cfg["scenario"] = {"enabled": True, "closures": [dict(distance_m=150, lane=0, taper_m=30, length_m=60, kind="cones")]}
        cfg["sync"]["duration"] = 40.0
        info, st = run(c, cfg)
        folder = info.get("record_dir") or ""
        run_json = json.load(open(os.path.join(folder, "run.json"), encoding="utf-8"))
        k = run_json["kpi"]
        algo = [e["msg"] for e in c.events if e.get("event") == "log" and e.get("level") == "algo"]
        seen = [m for m in algo if m.startswith("STATIC_SEEN")]
        print("INFO %s; kpi collisions %s, first with %s at %s s" % (seen[-1] if seen else "-", k.get("collisions"),
                                                                       k.get("first_collision_with"), k.get("first_collision_t")))
        check("the algorithm got the cones as static objects", seen and int(seen[-1].split()[1]) >= 5, seen)
        check("the lane follower drives into its closed lane: a collision with a cone",
              (k.get("collisions") or 0) > 0 and str(k.get("first_collision_with", "")).startswith("static.prop."),
              (k.get("collisions"), k.get("first_collision_with")))
        rows = read_csv(os.path.join(folder, "log_objects.csv"))
        check("the records keep them (type static in log_objects.csv)",
              any(r.get("type") == "static" for r in rows), len(rows))
        logs = [e["msg"] for e in c.events if e.get("event") == "log"]
        check("the output says what was placed", any(m.startswith("测试场景：第 1 处，出生点前方 150 m，本车道") for m in logs),
              [m for m in logs if "测试场景" in m][:2])
        n_before = len(props_in(client.get_world()))
        check("the props stay after the run (for a look)", n_before > 0, n_before)

        # ---- the next run without the scenario removes them -----------------------------
        cfg["scenario"] = {"enabled": False, "closures": []}
        cfg["sync"]["duration"] = 1.0
        info, st = run(c, cfg)
        left = props_in(client.get_world())
        check("the next run removes them", st["state"] == "finished" and not left, (st["state"], len(left)))
        check("run.json without a scenario: null",
              json.load(open(os.path.join(info["record_dir"], "run.json"), encoding="utf-8"))["scenario"] is None)

        # ---- a lane that is not there: refused before anything is placed --------------
        cfg["scenario"] = {"enabled": True, "closures": [dict(distance_m=200, lane=-2, taper_m=30, length_m=60, kind="cones")]}
        try:
            c.call("cosim_start", config=cfg, timeout=120)
            err = ""
        except Exception as e:  # the backend's error for the request
            err = str(e)
        check("a lane that is not there refuses the run in plain words",
              "左侧第 2 条车道" in err and "那里没有这条车道" in err and "同方向有 4 条车道" in err, err)
        check("... and placed nothing", not props_in(client.get_world()))
        cfg["scenario"] = {"enabled": True, "closures": [dict(distance_m=1, lane=0, taper_m=0, length_m=1, kind="nope")]}
        try:
            c.call("cosim_start", config=cfg, timeout=120)
            err = ""
        except Exception as e:
            err = str(e)
        check("an unknown kind is refused", "类型 'nope' 不认识" in err, err)

        # ---- a backend killed with props in the world: the next one removes them --------
        cfg["scenario"] = {"enabled": True, "closures": PRESETS["封闭本车道（锥桶）"]}
        cfg["sync"]["duration"] = 0.0
        c.events.clear()
        c.call("cosim_start", config=cfg, timeout=300)
        time.sleep(2.0)
        n_live = len(props_in(client.get_world()))
        proc.send_signal(signal.SIGKILL)
        proc.wait(10)
        # A killed backend leaves the world synchronous with nobody ticking it: the next one handles that.
        proc = backend(PORT + 1)
        c2 = Conn(PORT + 1)
        c2.call("connect", host="localhost", port=carla_port, timeout=60, recover=True)
        left = props_in(client.get_world())
        check("killed backend: its %d props removed by the next one (recover)" % n_live, n_live > 0 and not left,
              (n_live, len(left)))
        c2.call("destroy_ego")

        # ---- a map without such a start (the GUI then offers to load Town04) ----------
        info = c2.call("load_map", name="Town10HD_Opt", timeout=400)
        check("Town10: no highway start", info["map"] == "Town10HD_Opt" and info["scenario_start"] is None,
              (info["map"], info.get("scenario_start")))
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            proc.kill()
        shutil.rmtree(tmp, ignore_errors=True)
    print("FAILED: %s" % FAILS if FAILS else "ALL SCENARIO TESTS PASSED")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
