"""One result folder per run against a CARLA server (tests/test_offline_runs.py
has the same checks without one).

Mock CarSim, the user's algorithm, 1 s runs. Checks: the run writes
<record dir>/<time>_<algorithm>/ with log.csv, log_objects.csv,
log_lane.csv, config.json, a copy of the algorithm and run.json; run.json
has the map, spawn point, CARLA interface, how the run ended and figures
that agree with log.csv / log_lane.csv; the algorithm's finish(reason) gets
the same reason once; the output names the folder and the figures; a second
run gets a new folder and leaves the first alone; Stop ends with "stopped".
Everything is written to a temp dir, deleted afterwards.

    python tests/test_runs_carla.py [--port 2000]
"""
import csv
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
from test_backend import Conn, check  # noqa: E402

PORT = 57172

# Drives straight; finish() writes the reason it gets.
CONTROLLER = '''
import os
class Controller:
    def control(self, exports, t, dt, scene):
        return [0.4, 0.0, 0.0]
    def finish(self, reason):
        with open(os.environ["RUNS_TEST_LOG"], "a", encoding="utf-8") as f:
            f.write(reason + "\\n")
'''


def run_until_state(c, states, timeout=120):
    return c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in states, timeout)


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def finishes(log):
    if not os.path.exists(log):
        return []
    with open(log, encoding="utf-8") as f:
        return [ln.rstrip("\n") for ln in f]


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    tmp = tempfile.mkdtemp(prefix="cc_runs_test_")
    log = os.path.join(tmp, "finish.txt")
    os.environ["RUNS_TEST_LOG"] = log
    proc = subprocess.Popen([sys.executable, os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT)],
                            cwd=os.path.join(HERE, ".."), env=dict(os.environ))
    try:
        c = Conn(PORT)
        c.call("connect", host="localhost", port=carla_port, timeout=60)
        with open(os.path.join(tmp, "ctrl.py"), "w", encoding="utf-8") as f:
            f.write(CONTROLLER)
        cfg = c.call("default_config")
        check("default record dir", cfg["run"]["log_path"] == "runs", cfg["run"]["log_path"])
        cfg["carsim"]["mock"] = True
        cfg["carla"]["spawn_index"] = 1
        cfg["run"]["driver"] = "custom"
        cfg["run"]["controller"] = {"path": os.path.join(tmp, "ctrl.py"), "entry": "Controller"}
        cfg["run"]["log_path"] = os.path.join(tmp, "runs")
        cfg["sync"]["frame_dt"] = 0.02
        cfg["sync"]["duration"] = 1.0
        cfg["collect"]["sample_period"] = 0.1   # every 5 steps: 11 samples in 1 s
        cfg["scene"]["collision"] = "log"

        # ---- 1: a finished run --------------------------------------------------
        c.events.clear()
        info = c.call("cosim_start", config=cfg, timeout=120)
        st = run_until_state(c, ("finished", "error", "stopped"))
        check("run finished", st["state"] == "finished" and st["detail"].startswith("达到设定的运行时长"), st.get("detail"))
        folder = info.get("record_dir") or ""
        check("the run's own folder <time>_<algorithm>", os.path.dirname(folder) == os.path.join(tmp, "runs")
              and re.match(r"^\d{8}_\d{6}_ctrl$", os.path.basename(folder)), folder)
        names = sorted(os.listdir(folder)) if os.path.isdir(folder) else []
        check("CSV files, config, algorithm copy, run.json",
              names == ["config.json", "ctrl.py", "log.csv", "log_lane.csv", "log_objects.csv", "run.json"], names)
        run = json.load(open(os.path.join(folder, "run.json"), encoding="utf-8"))
        winfo = c.call("world_info")
        check("run.json: map, spawn point, interface, CarSim, end",
              run["map"] == winfo["map"] and run["spawn_index"] == 1 and run["external_api"] == info["external_api"]
              and run["carsim_mock"] is True and run["controller"] == os.path.join(tmp, "ctrl.py")
              and run["end"] == "finished" and run["end_reason"] == st["detail"],
              {k: run[k] for k in ("map", "spawn_index", "external_api", "end", "end_reason")})
        config = json.load(open(os.path.join(folder, "config.json"), encoding="utf-8"))
        check("config.json is the run's config", config["run"]["controller"] == cfg["run"]["controller"]
              and config["sync"]["duration"] == 1.0, config["run"])
        main_rows = read_csv(os.path.join(folder, "log.csv"))
        lane_rows = read_csv(os.path.join(folder, "log_lane.csv"))
        k = run["kpi"] or {}
        dist = sum(math.hypot(float(b["ego_X"]) - float(a["ego_X"]), float(b["ego_Y"]) - float(a["ego_Y"]))
                   for a, b in zip(main_rows, main_rows[1:]))
        check("figures from the record's samples", k.get("samples") == len(main_rows) == 11
              and abs(k["distance"] - dist) < 1e-3 and run["t_start"] == 0.0 and abs(run["t_end"] - 1.0) < 1e-6,
              (k.get("samples"), len(main_rows), k.get("distance"), dist, run["t_end"]))
        if lane_rows:
            offs = [abs(float(r["offset"])) for r in lane_rows]
            rms = math.sqrt(sum(o * o for o in offs) / len(offs))
            check("lane figures agree with log_lane.csv", abs(k["lane_offset_rms"] - rms) < 1e-4
                  and abs(k["lane_offset_max"] - max(offs)) < 1e-4
                  and abs(k["time_off_lane"] - 0.1 * (len(main_rows) - len(lane_rows))) < 1e-6,
                  (k["lane_offset_rms"], rms, k["time_off_lane"]))
        else:
            print("INFO spawn point 1 is off the driving lanes: lane figures not compared")
        check("finish(reason) once, with the end reason", finishes(log) == [st["detail"]], finishes(log))
        msgs = [e["msg"] for e in c.events if e.get("event") == "log"]
        check("the output names the folder and the figures", ("运行记录：%s" % folder) in msgs
              and any(m.startswith("运行指标：") and "行驶距离" in m for m in msgs), msgs[-4:])

        # ---- 2: a second run: a new folder, the first one untouched -------------
        before = open(os.path.join(folder, "log.csv"), encoding="utf-8").read()
        c.events.clear()  # else run 1's "finished" ends the wait at once
        info2 = c.call("cosim_start", config=cfg, timeout=120)
        st = run_until_state(c, ("finished", "error", "stopped"))
        check("second run: new folder, the first left alone", st["state"] == "finished"
              and info2["record_dir"] != folder and os.path.isdir(info2["record_dir"])
              and open(os.path.join(folder, "log.csv"), encoding="utf-8").read() == before,
              (folder, info2.get("record_dir")))

        # ---- 3: Stop ------------------------------------------------------------
        r = json.loads(json.dumps(cfg))
        r["sync"]["duration"] = 0.0
        c.events.clear()
        info3 = c.call("cosim_start", config=r, timeout=120)
        c.wait_event(lambda e: e.get("event") == "telemetry" and e["data"]["t"] > 0.3, 60)
        c.call("cosim_stop")
        run3 = json.load(open(os.path.join(info3["record_dir"], "run.json"), encoding="utf-8"))
        check("Stop: run.json and finish() say so", run3["end"] == "stopped" and run3["end_reason"] == "运行被停止"
              and finishes(log)[-1] == "运行被停止", (run3["end"], run3["end_reason"], finishes(log)[-1:]))
        c.call("destroy_ego")
        print("ALL RUN FOLDER TESTS PASSED")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
        proc.terminate()
        proc.wait(10)


if __name__ == "__main__":
    main()
