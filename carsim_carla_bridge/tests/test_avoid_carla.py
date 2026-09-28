"""controllers/lane_change_avoid.py (避障示例) on the Town04 highway with a CARLA server:
the mock CarSim driven through each 测试场景 preset of the GUI.

Checks per preset: the run ends normally, no collision, the algorithm changed
lanes, it is back in its start lane at the end (unless that lane is closed
up to the end), it kept a gap to the props, it went past the closures; and
without closures it never changes lanes.

    python tests/test_avoid_carla.py [--port 2000]
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

from test_backend import Conn  # noqa: E402

PORT = 57146
CONTROLLERS = os.path.abspath(os.path.join(HERE, "..", "controllers"))
# lane_change_avoid with the mock CarSim's vehicle, reporting the closest static prop and its lane at the end.
CONTROLLER = '''import sys
sys.path.insert(0, %r)
import path_follower as PF
import lane_change_avoid as LCA
PF.WHEELBASE, PF.STEER_RATIO_INIT, PF.BRAKE_MAX = 2.9, 16.0, 1.0


class Controller(LCA.Controller):
    def reset(self):
        super().reset()
        self.min_gap = None

    def control(self, exports, t, dt, scene):
        for o in scene["objects"]:
            if o["type"] == "static":
                self.min_gap = o["gap"] if self.min_gap is None else min(self.min_gap, o["gap"])
        return super().control(exports, t, dt, scene)

    def finish(self, reason):
        print("AVOID changes %%d home %%d min_gap %%s" %% (self.changes, self.home, self.min_gap))
        super().finish(reason)
''' % CONTROLLERS
PRESETS = {  # the GUI's (panels.cpp DrawPanelTestScene); want: back in the start lane at the end
    "封闭本车道（锥桶）": ([dict(distance_m=250, lane=0, taper_m=40, length_m=100, kind="cones")], True),
    "封闭左侧车道（护栏）": ([dict(distance_m=250, lane=-1, taper_m=30, length_m=120, kind="barrier")], True),
    "连续两处封道": ([dict(distance_m=250, lane=0, taper_m=40, length_m=80, kind="cones"),
                   dict(distance_m=550, lane=-1, taper_m=40, length_m=80, kind="cones")], True),
    "只剩一条车道": ([dict(distance_m=300, lane=k, taper_m=40, length_m=100, kind="cones") for k in (0, 1, 2)], True),
}
FAILS = []


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + (("  " + str(detail)) if detail != "" else ""), flush=True)
    if not cond:
        FAILS.append(name)


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    tmp = tempfile.mkdtemp(prefix="cc_avoid_test_")
    ctrl = os.path.join(tmp, "avoid_mock.py")
    with open(ctrl, "w", encoding="utf-8") as f:
        f.write(CONTROLLER)
    proc = subprocess.Popen([sys.executable, os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT)],
                            cwd=os.path.join(HERE, ".."))
    try:
        c = Conn(PORT)
        c.call("connect", host="localhost", port=carla_port, timeout=60)
        if "Town04" not in c.call("world_info")["map"]:
            c.call("load_map", name="Town04_Opt", timeout=400)
        start = c.call("world_info")["scenario_start"]
        cfg = c.call("default_config")
        cfg["carsim"]["mock"] = True
        cfg["carla"]["spawn_index"] = start["index"]
        cfg["run"]["driver"] = "custom"
        cfg["run"]["controller"] = {"path": ctrl, "entry": "Controller"}
        cfg["run"]["log_path"] = os.path.join(tmp, "runs")
        cfg["collect"]["sample_period"] = 0.1
        cfg["scene"]["collision"] = "log"
        for k in ("left_lane", "right_lane", "in_junction"):
            if k not in cfg["scene"]["lane"]:
                cfg["scene"]["lane"].append(k)
        cfg["sync"]["duration"] = 75.0
        cases = [("没有封道", [], True)] + [(n, cl, home) for n, (cl, home) in PRESETS.items()]
        for name, closures, want_home in cases:
            cfg["scenario"] = {"enabled": bool(closures), "closures": closures}
            c.events.clear()
            info = c.call("cosim_start", config=cfg, timeout=300)
            st = c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in ("finished", "error", "stopped"), 600)
            k = json.load(open(os.path.join(info["record_dir"], "run.json"), encoding="utf-8"))["kpi"]
            algo = [e["msg"] for e in c.events if e.get("event") == "log" and e.get("level") == "algo"]
            rep = [m for m in algo if m.startswith("AVOID")]
            parts = rep[-1].split() if rep else []
            changes, home = (int(parts[2]), int(parts[4])) if parts else (-1, 99)
            min_gap = float(parts[6]) if parts and parts[6] != "None" else None
            print("INFO %s: %s | %s | distance %.0f m, collisions %s" % (
                name, rep[-1] if rep else "-", " / ".join(m for m in algo if "换" in m or "挡住" in m),
                k.get("distance") or 0, k.get("collisions")))
            check("%s: the run ended normally" % name, st["state"] == "finished", st.get("reason", st))
            check("%s: no collision" % name, (k.get("collisions") or 0) == 0,
                  (k.get("collisions"), k.get("first_collision_with"), k.get("first_collision_t")))
            end_m = max([c_["distance_m"] + c_["taper_m"] + c_["length_m"] for c_ in closures] or [0])
            check("%s: went past the closures (%.0f m)" % (name, end_m), (k.get("distance") or 0) > end_m + 20,
                  k.get("distance"))
            if name == "没有封道" or name == "封闭左侧车道（护栏）":
                check("%s: never changes lanes" % name, changes == 0, changes)
            else:
                check("%s: changed lanes around the closure" % name, changes >= 1, changes)
                check("%s: kept > 0.3 m from every cone / barrier" % name, min_gap is not None and min_gap > 0.3, min_gap)
            if want_home:
                check("%s: back in the start lane at the end" % name, home == 0, home)
    finally:
        try:
            c.call("cosim_stop")
        except Exception:
            pass
        proc.terminate()
        try:
            proc.wait(20)
        except subprocess.TimeoutExpired:
            proc.kill()
        shutil.rmtree(tmp, ignore_errors=True)
    print("ALL AVOID TESTS PASSED" if not FAILS else "FAILED: %s" % FAILS)
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
