"""运行对比's replay (replay_pose) with a CARLA server: after a run with the
mock CarSim (the demo driver: speeds up, weaves), the ego is placed where the
record had it at a few times: its reference point, taken back into the run's
CarSim frame the way the scene does, is the recorded X / Y (< 2 cm) and its
yaw the recorded Yaw (< 0.2 deg); a time past the end holds the last pose;
refused in plain words during a run, for a run on another map, without an
ego.

    python tests/test_replay_carla.py [--port 2000]
"""
import csv
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
import carla  # noqa: E402

from bridge import anchor_frame, front_axle_local  # noqa: E402
from test_backend import Conn  # noqa: E402

PORT = 57152
FAILS = []


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + (("  " + str(detail)) if detail != "" else ""), flush=True)
    if not cond:
        FAILS.append(name)


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    tmp = tempfile.mkdtemp(prefix="cc_replay_test_")
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
        cfg["carla"]["spawn_index"] = 3
        cfg["carsim"].update(mock=True)
        cfg["run"].update(driver="demo", log_path=os.path.join(tmp, "runs"))
        cfg["sync"].update(frame_dt=0.05, duration=6.0)
        cfg["collect"]["sample_period"] = 0.1
        c.events.clear()
        info = c.call("cosim_start", config=cfg, timeout=300)
        c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in ("finished", "error", "stopped"), 600)
        folder = info["record_dir"]
        with open(os.path.join(folder, "log.csv"), newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        rec = {round(float(r["t"]), 3): (float(r["ego_X"]), float(r["ego_Y"]), float(r["ego_Yaw"])) for r in rows}
        world = client.get_world()
        cmap = world.get_map()
        anchor = anchor_frame(cmap, cmap.get_spawn_points()[3])
        R = anchor.R
        anchor_yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
        ego = world.get_actor(c.call("world_info")["ego_id"])
        ref_local = front_axle_local(ego)

        def carsim_pose():
            tf = ego.get_transform()
            p = tf.transform(carla.Location(*map(float, ref_local)))
            g = R.T @ (np.array([p.x, p.y, p.z]) - np.asarray(anchor.origin))
            return float(g[0]), float(-g[1]), (-(tf.rotation.yaw - anchor_yaw) + 180.0) % 360.0 - 180.0

        worst = (0.0, 0.0)
        for t in (0.0, 2.0, 4.5, 6.0):
            r = c.call("replay_pose", folder=folder, t=t)
            for _ in range(4):  # set by another client: it shows here a frame or two later (asynchronous world)
                world.wait_for_tick(5.0)
            X, Y, yaw = carsim_pose()
            want = rec[round(r["t"], 3)]
            d = math.hypot(X - want[0], Y - want[1])
            dy = abs((yaw - want[2] + 180.0) % 360.0 - 180.0)
            worst = (max(worst[0], d), max(worst[1], dy))
        check("replayed poses: the recorded reference point (< 2 cm) and yaw (< 0.2 deg)", worst[0] < 0.02 and worst[1] < 0.2,
              (round(worst[0], 4), round(worst[1], 4)))
        moved = max(math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(list(rec.values()), list(rec.values())[1:]))
        check("... over a run that moved (the demo drive)", math.hypot(rec[max(rec)][0] - rec[0.0][0], rec[max(rec)][1] - rec[0.0][1]) > 5.0,
              round(math.hypot(rec[max(rec)][0] - rec[0.0][0], rec[max(rec)][1] - rec[0.0][1]), 1))
        r = c.call("replay_pose", folder=folder, t=99.0)
        check("a time past the end: held at the last pose", abs(r["t"] - max(rec)) < 1e-6, r)

        # Another map's run: refused, saying which map to load.
        other = os.path.join(tmp, "other_run")
        shutil.copytree(folder, other)
        rj = json.load(open(os.path.join(other, "run.json"), encoding="utf-8"))
        rj["map"] = "Town03"
        json.dump(rj, open(os.path.join(other, "run.json"), "w", encoding="utf-8"))
        try:
            c.call("replay_pose", folder=other, t=1.0)
            err = ""
        except Exception as e:
            err = str(e)
        check("a run on another map: refused, which map to load", "先在“地图与天气”页加载 Town03" in err, err)
        # During a run: refused.
        cfg["sync"]["duration"] = 0.0
        c.call("cosim_start", config=cfg, timeout=300)
        try:
            c.call("replay_pose", folder=folder, t=1.0)
            err = ""
        except Exception as e:
            err = str(e)
        check("during a run: refused", "仿真运行中不能回放" in err, err)
        c.call("cosim_stop", timeout=120)
        c.call("destroy_ego")
        try:
            c.call("replay_pose", folder=folder, t=1.0)
            err = ""
        except Exception as e:
            err = str(e)
        check("without an ego: refused", "还没有主车" in err, err)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            proc.kill()
        shutil.rmtree(tmp, ignore_errors=True)
    print("FAILED: %s" % FAILS if FAILS else "ALL REPLAY TESTS PASSED")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
