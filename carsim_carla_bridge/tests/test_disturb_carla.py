"""干扰注入 with a CARLA server: the path follower (controllers/path_follower.py) on the mock
CarSim, Town10, 20 s, twice: without and with 干扰 (执行器延迟 0.3 s, 测量延迟 0.2 s, 5 km/h
noise on Vx). With it the lane offset gets clearly worse (the delays reach the car); the run
record keeps CarSim's true values (log.csv's Vx is the speed of the recorded positions, not
the noisy one the algorithm saw);
run.json has the settings (None without); the output says what was on.

    python tests/test_disturb_carla.py [--port 2000]
"""
import csv
import json
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
from test_backend import Conn  # noqa: E402

PORT = 57154
FAILS = []


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + (("  " + str(detail)) if detail != "" else ""), flush=True)
    if not cond:
        FAILS.append(name)


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    tmp = tempfile.mkdtemp(prefix="cc_disturb_carla_")
    proc = subprocess.Popen([sys.executable, os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT)],
                            cwd=os.path.join(HERE, ".."))
    try:
        c = Conn(PORT)
        c.call("connect", host="localhost", port=carla_port, timeout=60)
        if c.call("world_info")["map"] != "Town10HD_Opt":
            c.call("load_map", name="Town10HD_Opt", timeout=400)
        cfg = c.call("default_config")
        cfg["carla"]["spawn_index"] = 3
        cfg["carsim"].update(mock=True)
        cfg["run"].update(driver="custom", controller={"path": "controllers/path_follower.py", "entry": "Controller"},
                          log_path=os.path.join(tmp, "runs"))
        cfg["sync"].update(frame_dt=0.05, duration=20.0)
        cfg["collect"]["sample_period"] = 0.1
        res = {}
        for name, dist in (("plain", {"enabled": False}),
                           ("disturbed", {"enabled": True, "act_delay": 0.3, "sense_delay": 0.2, "noise": {"Vx": 5.0}, "seed": 1})):
            cfg["run"]["disturb"] = dict(cfg["run"]["disturb"], **dist)
            c.events.clear()
            info = c.call("cosim_start", config=cfg, timeout=300)
            c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in ("finished", "error", "stopped"), 600)
            folder = info["record_dir"]
            run = json.load(open(os.path.join(folder, "run.json"), encoding="utf-8"))
            rows = list(csv.DictReader(open(os.path.join(folder, "log.csv"), encoding="utf-8")))
            out = open(os.path.join(folder, "output.txt"), encoding="utf-8").read()
            res[name] = (run, rows, out)
            k = run["kpi"] or {}
            print("INFO %s: end %s, lane offset rms %s max %s, distance %s" % (
                name, run["end"], k.get("lane_offset_rms"), k.get("lane_offset_max"), k.get("distance")))
        (r0, rows0, out0), (r1, rows1, out1) = res["plain"], res["disturbed"]
        k0, k1 = r0["kpi"], r1["kpi"]
        check("both runs end by their duration", r0["end"] == "finished" and r1["end"] == "finished", (r0["end"], r1["end"]))
        check("with 0.5 s of delays the lane offset clearly worse (rms x 1.5 or more)",
              k1["lane_offset_rms"] > 1.5 * k0["lane_offset_rms"], (k0["lane_offset_rms"], k1["lane_offset_rms"]))
        # The recorded Vx against the speed from the recorded positions (0.1 s apart): the truth, not ±5 km/h noise.
        t = np.array([float(r["t"]) for r in rows1])
        x, y = (np.array([float(r[k]) for r in rows1]) for k in ("ego_X", "ego_Y"))
        vx = np.array([float(r["Vx"]) for r in rows1])
        moved = np.hypot(np.diff(x), np.diff(y)) / np.diff(t) * 3.6
        err = moved - 0.5 * (vx[1:] + vx[:-1])
        rms = float(np.sqrt(np.mean(err[5:] ** 2)))
        check("the record keeps CarSim's true Vx (= the speed of the recorded positions, rms < 1 km/h; the algorithm saw ±5)",
              rms < 1.0, round(rms, 3))
        check("run.json: the settings with them, None without",
              r0.get("disturb") is None and r1.get("disturb") == {"act_delay": 0.3, "sense_delay": 0.2, "noise": {"Vx": 5.0},
                                                                  "lane_dropout": 0.0, "seed": 1}, r1.get("disturb"))
        check("the output says what was on", "干扰已开启：执行器延迟 0.3 s（6 帧），测量延迟 0.2 s（4 帧），噪声 Vx ±5" in out1
              and "干扰" not in out0)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            proc.kill()
        shutil.rmtree(tmp, ignore_errors=True)
    print("FAILED: %s" % FAILS if FAILS else "ALL DISTURB CARLA TESTS PASSED")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
