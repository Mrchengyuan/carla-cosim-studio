"""KMPPI (controllers/kmppi, ported from ~/Desktop/rl/kmppi) driving the
PyChrono BMW E90 in place of CarSim (carsim.chrono: the backend starts
carsim_service.py --chrono in the "chrono" conda env itself, chrono_local.py)
on the Town04 highway with a CARLA server: 40 s from the
highway start (world_info's scenario_start: four lanes one way; about 800 m
with a straight and a curve of about 74 m radius), 20 m/s reference speed,
frame step 0.05 s (KMPPI's control period).

Checks: the backend starts the Chrono BMW; the run ends by its duration with no
algorithm error and the algorithm's summary; no "导出变量可疑" warning; the Chrono car starts at
20 m/s and holds it (19 ~ 21 m/s); no collision, never off the driving
lanes, lane offset rms < 0.3 m and max < 0.8 m; it covers > 700 m; CARLA's
car follows the Chrono car (the front wheel angles reach CARLA); KMPPI's
candidate trajectories are drawn in CARLA every frame (self.draw: 64
candidates + the weighted mean + the reference, about 790 segments); a
frame step that does not divide 0.05 s ends the run with the algorithm's
plain message. Starts its own backend (port 57145); deletes its temp files.

    python tests/test_kmppi_chrono_carla.py [--port 2000] [--chrono-python PATH] [--shot DIR]

--shot DIR: a chase camera's pictures at t = 10, 20, 30 s saved there (to look at the lines).
"""
import csv
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import queue
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
from test_backend import Conn  # noqa: E402

PORT = 57145
DURATION = 40.0
CONTROLLER = os.path.abspath(os.path.join(HERE, "..", "controllers", "kmppi", "controller.py"))
FAILS = []


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + (("  " + str(detail)) if detail != "" else ""), flush=True)
    if not cond:
        FAILS.append(name)


def chrono_python():
    if "--chrono-python" in sys.argv:
        return sys.argv[sys.argv.index("--chrono-python") + 1]
    for p in (os.path.expanduser("~/.conda/envs/chrono/bin/python"), "/opt/anaconda3/envs/chrono/bin/python"):
        if os.path.isfile(p):
            return p
    raise SystemExit("找不到 chrono conda 环境的 python：用 --chrono-python 指定")


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    tmp = tempfile.mkdtemp(prefix="cc_kmppi_test_")
    proc = subprocess.Popen([sys.executable, os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT)],
                            cwd=os.path.join(HERE, ".."), env=dict(os.environ))
    cam = None
    try:
        c = Conn(PORT)
        c.call("connect", host="localhost", port=carla_port, timeout=60)
        if "Town04" not in c.call("world_info")["map"]:
            c.call("load_map", name="Town04_Opt", timeout=400)
        start = c.call("world_info")["scenario_start"]
        cfg = c.call("default_config")
        check("the Chrono BMW is off by default", cfg["carsim"].get("chrono") is False, cfg["carsim"].get("chrono"))
        cfg["carla"]["spawn_index"] = start["index"]
        cfg["carsim"].update(mock=False, remote=False, chrono=True, chrono_init_speed=20.0, chrono_python=chrono_python())
        cfg["run"].update(driver="custom", controller={"path": CONTROLLER, "entry": "Controller"},
                          log_path=os.path.join(tmp, "runs"))
        cfg["sync"].update(frame_dt=0.05, duration=DURATION)
        cfg["collect"]["sample_period"] = 0.1
        cfg["scene"]["collision"] = "log"
        cfg["scene"]["record"]["lane"] = cfg["scene"]["record"]["lane"] + ["in_junction"]
        c.events.clear()
        t0 = time.time()
        info = c.call("cosim_start", config=cfg, timeout=300)
        shots = []
        if "--shot" in sys.argv:  # a chase camera on the ego, pictures at t = 10, 20, 30 s
            import carla
            client = carla.Client("localhost", carla_port)
            client.set_timeout(30)
            w = client.get_world()
            ego = w.get_actor(c.call("world_info")["ego_id"])
            bp = w.get_blueprint_library().find("sensor.camera.rgb")
            bp.set_attribute("image_size_x", "960")
            bp.set_attribute("image_size_y", "540")
            cam = w.spawn_actor(bp, carla.Transform(carla.Location(x=-7.0, z=4.0), carla.Rotation(pitch=-18)), attach_to=ego)
            q = queue.Queue()
            cam.listen(q.put)
        st = None
        while st is None:
            try:
                st = c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in ("finished", "error", "stopped"), 1)
            except Exception:
                st = None
            if cam is not None:
                while not q.empty():
                    img = q.get()
                    shots.append(img)
                    shots[:] = shots[-400:]
            if time.time() - t0 > DURATION * 20 + 300:
                raise RuntimeError("the run did not end")
        wall = time.time() - t0
        msgs = [e for e in c.events if e.get("event") == "log"]
        algo = [e["msg"] for e in msgs if e.get("level") == "algo"]
        for m in algo:
            print("ALGO " + m)
        check("the run ends by its duration", st["state"] == "finished" and st["detail"].startswith("达到设定的运行时长"),
              (st, [e["msg"] for e in msgs if e.get("level") in ("error", "warn")][-5:]))
        check("the backend started the Chrono BMW", any(e["msg"].startswith("联合仿真开始（Chrono 宝马 E90 代替 CarSim）")
                                                     for e in msgs), [e["msg"] for e in msgs][:3])
        drawn = [e["data"].get("draw_n", 0) for e in c.events if e.get("event") == "telemetry"]
        check("the candidate trajectories are drawn every frame (> 700 segments)",
              len(drawn) > 100 and min(drawn[5:]) > 700, (len(drawn), min(drawn[5:]) if drawn[5:] else None, max(drawn or [0])))
        check("no algorithm error", not any("Traceback" in m or "Error" in m for m in algo), algo[-5:])
        warns = [e["msg"] for e in msgs if e.get("level") == "warn" and "导出变量可疑" in e["msg"]]
        check("the Chrono exports pass the platform's sanity checks", not warns, warns[:2])
        check("the algorithm's summary at the end", any(m.startswith("KMPPI 结束") for m in algo), algo[-2:])
        folder = info.get("record_dir") or ""
        k = json.load(open(os.path.join(folder, "run.json"), encoding="utf-8"))["kpi"] or {}
        rows = read_csv(os.path.join(folder, "log.csv"))
        lane = {r["t"]: r for r in read_csv(os.path.join(folder, "log_lane.csv"))}
        offs = [abs(float(lane[r["t"]]["offset"])) for r in rows
                if r["t"] in lane and lane[r["t"]]["offset"] not in ("", None)
                and lane[r["t"]]["in_junction"] not in ("True", "1", "true")]
        rms = math.sqrt(sum(o * o for o in offs) / len(offs)) if offs else float("nan")
        v = [float(r["Vx"]) / 3.6 for r in rows]  # km/h
        wheel = max(abs(float(r["Steer_L1"])) for r in rows)
        print("INFO %.0f m in %.0f s wall (%.2fx real time), lane offset rms %.3f max %.3f m (%d samples), "
              "speed %.2f ~ %.2f m/s, max front wheel angle %.2f deg, collisions %s, off lane %s s" % (
                  k.get("distance") or 0, wall, DURATION / wall, rms, max(offs) if offs else float("nan"), len(offs),
                  min(v), max(v), wheel, k.get("collisions"), k.get("time_off_lane")))
        check("the Chrono car starts at 20 m/s and holds it (19 ~ 21 m/s)", 19.0 < min(v) and max(v) < 21.0,
              (round(min(v), 2), round(max(v), 2)))
        check("no collision", k.get("collisions") == 0, (k.get("collisions"), k.get("first_collision_with")))
        check("never off the driving lanes", (k.get("time_off_lane") or 0.0) == 0.0, k.get("time_off_lane"))
        check("lane offset rms < 0.3 m, max < 0.8 m", len(offs) > 300 and rms < 0.3 and max(offs) < 0.8,
              (len(offs), round(rms, 3), round(max(offs), 3) if offs else None))
        check("it covers > 700 m", (k.get("distance") or 0.0) > 700.0, k.get("distance"))
        check("it steers through the curve (front wheel angle > 1 deg)", wheel > 1.0, wheel)

        if cam is not None:
            cam.stop()
            cam.destroy()
            cam = None
            out = sys.argv[sys.argv.index("--shot") + 1]
            os.makedirs(out, exist_ok=True)
            f0 = shots[0].frame if shots else 0
            for k, img in enumerate(shots):
                t = (img.frame - f0) * 0.05
                if any(abs(t - x) < 0.026 for x in (10.0, 20.0, 30.0)):
                    img.save_to_disk(os.path.join(out, "kmppi_t%02d.png" % round(t)))
            print("INFO pictures in %s" % out)

        # A frame step that does not divide KMPPI's 0.05 s: the algorithm says so and the run ends.
        cfg["sync"].update(frame_dt=0.02, duration=2.0)
        c.events.clear()
        c.call("cosim_start", config=cfg, timeout=300)
        st = c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in ("finished", "error", "stopped"), 300)
        text = " ".join(e["msg"] for e in c.events if e.get("event") == "log") + " " + str(st.get("detail"))
        check("frame step 0.02 s: refused in plain words", st["state"] == "error" and "仿真步长 0.020 s 不能整除" in text,
              (st, text[-300:]))
        c.call("destroy_ego")
    finally:
        if cam is not None:
            cam.destroy()
        proc.terminate()
        try:
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            proc.kill()
        shutil.rmtree(tmp, ignore_errors=True)
    print("FAILED: %s" % FAILS if FAILS else "ALL KMPPI CHRONO TESTS PASSED")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
