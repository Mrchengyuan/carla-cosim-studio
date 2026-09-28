"""The lane path follower (controllers/path_follower.py) on a CARLA map with
the mock CarSim (tests/test_offline_path_follower.py has the same algorithm
on synthetic roads, without CARLA).

Runs of 40 s from spawn points spread over the map, the platform's default
settings (the algorithm gets the default scene selection: lane width /
offset / heading error / centre line and the ego's pose), each recorded to a
temp dir. Per run, from run.json and the records: the run ends by its
duration, no algorithm error, no collision, never off the driving lanes, the
lane offset outside junctions (the nearest lane inside one can be a crossing
lane: not a measure of the follower) small, and the car covers the distance
of a car that keeps going (not crawling). Inside junctions: never off the road and the heading
changes smoothly (no swerve). Everything is deleted afterwards.

    python tests/test_path_follower_carla.py [--port 2000] [--spawns 0,20,40] [--keep DIR]

--keep DIR: copy the run folders there (for a closer look), instead of only deleting them.
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
from test_backend import Conn  # noqa: E402

PORT = 57143
DURATION = 40.0
FOLLOWER = os.path.abspath(os.path.join(HERE, "..", "controllers", "path_follower.py"))
# The file is set up for the user's CarSim car (wheelbase, steering ratio, brake pressure in MPa);
# the mock CarSim is 2.9 m, ratio 16 and takes a 0~1 brake pedal as import 2.
CONTROLLER = '''import importlib.util
spec = importlib.util.spec_from_file_location("path_follower", %r)
PF = importlib.util.module_from_spec(spec)
spec.loader.exec_module(PF)
PF.WHEELBASE, PF.STEER_RATIO_INIT, PF.BRAKE_MAX = 2.9, 16.0, 1.0
Controller = PF.Controller
''' % FOLLOWER


FAILS = []


def check(name, cond, detail=""):
    """Like test_backend.check, but goes on: every run is reported."""
    print(("PASS " if cond else "FAIL ") + name + (("  " + str(detail)) if detail != "" else ""))
    if not cond:
        FAILS.append(name)


def run_until_state(c, states, timeout=600):
    return c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in states, timeout)


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def one_run(c, cfg, spawn, summary):
    cfg["carla"]["spawn_index"] = spawn
    c.events.clear()
    info = c.call("cosim_start", config=cfg, timeout=180)
    st = run_until_state(c, ("finished", "error", "stopped"))
    tag = "spawn %d: " % spawn
    msgs = [e for e in c.events if e.get("event") == "log"]
    algo = [e["msg"] for e in msgs if e.get("level") == "algo"]
    check(tag + "run ends by its duration", st["state"] == "finished" and st["detail"].startswith("达到设定的运行时长"),
          (st, [e["msg"] for e in msgs if e.get("level") in ("error", "warn")][-5:]))
    check(tag + "no algorithm error", not any("Traceback" in m or "Error" in m for m in algo), algo[-8:])
    check(tag + "the algorithm's summary at the end", any(m.startswith("路径跟踪结束") for m in algo), algo[-3:])
    folder = info.get("record_dir") or ""
    run = json.load(open(os.path.join(folder, "run.json"), encoding="utf-8"))
    k = run["kpi"] or {}
    main_rows = read_csv(os.path.join(folder, "log.csv"))
    lane_rows = {r["t"]: r for r in read_csv(os.path.join(folder, "log_lane.csv"))}
    outside, inside, jumps = [], [], []
    prev_yaw = None
    for r in main_rows:
        yaw = float(r["ego_Yaw"])
        if prev_yaw is not None:
            jumps.append(abs((yaw - prev_yaw + 180.0) % 360.0 - 180.0))
        prev_yaw = yaw
        lr = lane_rows.get(r["t"])
        if lr is None:
            continue
        (inside if lr["in_junction"] in ("True", "1", "true") else outside).append(abs(float(lr["offset"])))
    rms = math.sqrt(sum(o * o for o in outside) / len(outside)) if outside else float("nan")
    worst = max(outside) if outside else float("nan")
    period = k.get("sample_period") or 0.1
    line = ("spawn %3d: %5.0f m, lane offset outside junctions rms %.3f m max %.3f m (%d samples), "
            "junction samples %d, off lane %.1f s, collisions %s, heading step max %.1f deg/%.2fs" % (
                spawn, k.get("distance") or 0.0, rms, worst, len(outside), len(inside),
                k.get("time_off_lane") or 0.0, k.get("collisions"), max(jumps) if jumps else 0.0, period))
    print("INFO " + line)
    summary.append(line)
    check(tag + "no collision", k.get("collisions") == 0, (k.get("collisions"), k.get("first_collision_with")))
    check(tag + "never off the driving lanes", (k.get("time_off_lane") or 0.0) == 0.0, k.get("time_off_lane"))
    check(tag + "lane offset outside junctions: rms < 0.25 m", len(outside) > 50 and rms < 0.25, (len(outside), rms))
    check(tag + "lane offset outside junctions: max < 0.6 m (0.75 m to the marking)", worst < 0.6, worst)
    # Not crawling. With gentle speed-ups (ACCEL_MAX 1.5 m/s^2) a route of short straights between
    # junction turns never gets back to the set 40 km/h (about 37 km/h, 240 m in 40 s).
    top = max(float(r["Vx"]) for r in main_rows)
    check(tag + "keeps going (top speed > 30 km/h, > 200 m)", top > 30.0 and (k.get("distance") or 0.0) > 200.0,
          (round(top, 1), k.get("distance")))
    # A swerve would show as a big heading step between samples (40 km/h in a
    # 10 m radius turn: ~6 deg per 0.1 s).
    check(tag + "no swerve (heading step < 12 deg per sample)", max(jumps) < 12.0, max(jumps))
    return folder


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    tmp = tempfile.mkdtemp(prefix="cc_pathfollow_test_")
    proc = subprocess.Popen([sys.executable, os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT)],
                            cwd=os.path.join(HERE, ".."), env=dict(os.environ))
    summary = []
    try:
        c = Conn(PORT)
        c.call("connect", host="localhost", port=carla_port, timeout=60)
        n = len(c.call("list_spawn_points"))
        if "--spawns" in sys.argv:
            spawns = [int(s) for s in sys.argv[sys.argv.index("--spawns") + 1].split(",")]
        else:
            spawns = sorted({int(i * n / 12) for i in range(12)})
        print("INFO map %s, %d spawn points, runs from %s" % (c.call("world_info")["map"], n, spawns))
        cfg = c.call("default_config")
        cfg["carsim"]["mock"] = True
        cfg["run"]["driver"] = "custom"
        ctrl = os.path.join(tmp, "path_follower_mock.py")
        with open(ctrl, "w", encoding="utf-8") as f:
            f.write(CONTROLLER)
        cfg["run"]["controller"] = {"path": ctrl, "entry": "Controller"}
        cfg["run"]["log_path"] = os.path.join(tmp, "runs")
        cfg["sync"]["duration"] = DURATION
        cfg["collect"]["sample_period"] = 0.1
        cfg["scene"]["collision"] = "log"
        # Records only: which samples are in a junction (the algorithm's selection stays the default).
        cfg["scene"]["record"]["lane"] = cfg["scene"]["record"]["lane"] + ["in_junction", "curvature", "junction_dist"]
        keep = sys.argv[sys.argv.index("--keep") + 1] if "--keep" in sys.argv else None
        for spawn in spawns:
            folder = one_run(c, cfg, spawn, summary)
            if keep:
                shutil.copytree(folder, os.path.join(keep, "spawn_%d" % spawn), dirs_exist_ok=True)
        c.call("destroy_ego")
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            proc.kill()
        shutil.rmtree(tmp, ignore_errors=True)
    print("\n".join(summary))
    print("FAILED: %s" % FAILS if FAILS else "ALL PATH FOLLOWER TESTS PASSED")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
