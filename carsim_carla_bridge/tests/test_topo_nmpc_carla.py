"""The multi-topology NMPC (controllers/topo_nmpc) driving the PyChrono BMW E90
in place of CarSim on the Town04 highway with a CARLA server (the backend
starts carsim_service.py --chrono itself), through 测试场景 cases:

    没有障碍      lane keeping 40 s at 72 km/h: never changes lanes, lane offset rms < 0.25 m, max < 0.6 m,
                  speed 19 ~ 21 m/s after the start
    封闭本车道    cones close the start lane 250 m ahead (taper 40 m, 100 m long): changes lanes around them
                  without slowing much, > 0.3 m from every cone, back in the start lane at the end
    前车慢行      a car at 30 km/h 120 m ahead in the start lane: overtakes it on the left, comes back
    旁车切入      a car in the left lane at 50 km/h moves into the ego's lane 35 m ahead of it
    前车急刹      a car at 60 km/h 90 m ahead brakes at 6 m/s^2 to a stop once the ego is 40 m behind it

Every case: the run ends by its duration, no algorithm error, no collision, never off the driving
lanes, never closer than 0.3 m to anything (box gap), the solver keeps up (mean < 250 ms per frame on the
server's Xeon E5-2682 v4; about 25 ms on a current desktop CPU), never rolls backwards (Chrono: torque, no brakes).
A traffic light stands at the junction about 860 m from the start: a run that gets there stops for red
(a longer 前车慢行 does): the speed checks look at the first 38 s.
Starts its own backend (port 57147); deletes its temp files.

    python tests/test_topo_nmpc_carla.py [--port 2000] [--backend-port 57147] [--chrono-python PATH]
                                         [--only NAME] [--keep DIR]

--only NAME: just that case (several test processes, each with its own CARLA and backend port, run the
cases in parallel). --keep DIR: the run records are copied there (log.csv, log_lane.csv, run.json, output.txt).
The curve on this stretch (about 74 m radius, about 650 m from the start) is taken at the planned
√(A_LAT_MAX · R) ≈ 14.9 m/s: the speed checks allow that, nothing below 13.5 m/s.
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
from test_backend import Conn  # noqa: E402

PORT = int(sys.argv[sys.argv.index("--backend-port") + 1]) if "--backend-port" in sys.argv else 57147
V_CURVE_MIN = 13.5   # m/s：弯道按 A_LAT_MAX 降速后的车速下限（R ≈ 74 m：√(3 × 74) ≈ 14.9 m/s）
CTL_DIR = os.path.abspath(os.path.join(HERE, "..", "controllers", "topo_nmpc"))
# The controller, reporting the closest object (box gap), its lane index and lane changes at the end.
WRAPPER = '''import importlib.util, os, sys
D = %r
sys.path.insert(0, D)
spec = importlib.util.spec_from_file_location("topo_nmpc_controller", os.path.join(D, "controller.py"))
K = importlib.util.module_from_spec(spec)
spec.loader.exec_module(K)
FRAME_DT = 0.05


class Controller(K.Controller):
    def reset(self):
        super().reset()
        self.gap_min = None
        self.gap_with = None

    def control(self, exports, t, dt, scene):
        for o in scene.get("objects") or []:
            if o.get("gap") is not None and (self.gap_min is None or o["gap"] < self.gap_min):
                self.gap_min, self.gap_with = o["gap"], o.get("type")
        return super().control(exports, t, dt, scene)

    def finish(self, reason):
        super().finish(reason)
        print("NMPCTEST lane %%d changes %%d gap %%s with %%s solve_ms %%.1f solve_max_ms %%.1f" %% (
            self.lane_idx, self.lane_changes, "%%.2f" %% self.gap_min if self.gap_min is not None else "None",
            self.gap_with, 1000 * self.solve_s / max(self.n_calls, 1), 1000 * self.solve_max))
''' % CTL_DIR

CASES = {
    "没有障碍": dict(duration=40.0, closures=[], actors=[]),
    "封闭本车道": dict(duration=40.0, closures=[dict(distance_m=250, lane=0, taper_m=40, length_m=100, kind="cones")],
                   actors=[]),
    "前车慢行": dict(duration=45.0, closures=[],
                 actors=[dict(type="slow_car", distance_m=120, lane=0, speed_kmh=30, trigger_m=0, param=0)]),
    "旁车切入": dict(duration=30.0, closures=[],
                 actors=[dict(type="cut_in", distance_m=60, lane=-1, speed_kmh=50, trigger_m=35, param=2.5)]),
    "前车急刹": dict(duration=30.0, closures=[],
                 actors=[dict(type="lead_brake", distance_m=90, lane=0, speed_kmh=60, trigger_m=40, param=6)]),
}
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
    only = sys.argv[sys.argv.index("--only") + 1] if "--only" in sys.argv else None
    tmp = tempfile.mkdtemp(prefix="cc_nmpc_test_")
    ctrl = os.path.join(tmp, "nmpc_test.py")
    with open(ctrl, "w", encoding="utf-8") as f:
        f.write(WRAPPER)
    proc = subprocess.Popen([sys.executable, "-X", "faulthandler", os.path.join(HERE, "..", "backend_server.py"),
                             "--port", str(PORT)], cwd=os.path.join(HERE, ".."))
    c = None
    try:
        c = Conn(PORT)
        for attempt in range(6):   # 刚启动的 CARLA：端口已在监听、模拟器还没准备好（几台同时启动时常见）
            try:
                c.call("connect", host="localhost", port=carla_port, timeout=60)
                break
            except RuntimeError as err:
                if attempt == 5:
                    raise
                print("connect: %s，20 s 后重试" % str(err)[:80], flush=True)
                time.sleep(20)
        if "Town04" not in c.call("world_info")["map"]:
            c.call("load_map", name="Town04_Opt", timeout=400)
        start = c.call("world_info")["scenario_start"]
        cfg = c.call("default_config")
        cfg["carla"]["spawn_index"] = start["index"]
        cfg["carsim"].update(mock=False, remote=False, chrono=True, chrono_init_speed=20.0,
                             chrono_python=chrono_python())
        cfg["run"].update(driver="custom", controller={"path": ctrl, "entry": "Controller"},
                          log_path=os.path.join(tmp, "runs"))
        cfg["sync"]["frame_dt"] = 0.05
        cfg["collect"]["sample_period"] = 0.1
        cfg["scene"]["collision"] = "log"
        for key, extra in (("objects", ["rel_yaw", "length", "width", "Vx_global", "Vy_global"]),
                           ("lane", ["left_lane", "right_lane", "in_junction", "light_state", "light_dist"]),
                           ("ego", ["X", "Y", "Yaw", "length", "width"])):
            for k in extra:
                if k not in cfg["scene"][key]:
                    cfg["scene"][key].append(k)
        cfg["scene"]["record"]["lane"] = cfg["scene"]["record"]["lane"] + ["in_junction"]
        for name, case in CASES.items():
            if only and name != only:
                continue
            cfg["sync"]["duration"] = case["duration"]
            cfg["scenario"] = {"enabled": bool(case["closures"] or case["actors"]),
                               "closures": case["closures"], "actors": case["actors"]}
            c.events.clear()
            t0 = time.time()
            info = c.call("cosim_start", config=cfg, timeout=300)
            st = c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in ("finished", "error", "stopped"),
                              case["duration"] * 20 + 300)
            wall = time.time() - t0
            if "--keep" in sys.argv and info.get("record_dir"):
                dst = os.path.join(sys.argv[sys.argv.index("--keep") + 1], name)
                shutil.rmtree(dst, ignore_errors=True)
                shutil.copytree(info["record_dir"], dst)
            msgs = [e for e in c.events if e.get("event") == "log"]
            algo = [e["msg"] for e in msgs if e.get("level") == "algo"]
            folder = info.get("record_dir") or ""
            k = json.load(open(os.path.join(folder, "run.json"), encoding="utf-8"))["kpi"] or {}
            rows = read_csv(os.path.join(folder, "log.csv"))
            lane = {r["t"]: r for r in read_csv(os.path.join(folder, "log_lane.csv"))}
            rep = [m for m in algo if m.startswith("NMPCTEST")]
            p = rep[-1].split() if rep else []
            lane_end, changes = (int(p[2]), int(p[4])) if p else (99, -1)
            gap = float(p[6]) if p and p[6] != "None" else None
            solve_ms = float(p[10]) if p else float("nan")
            v_all = [float(r["Vx"]) / 3.6 for r in rows]
            v = [float(r["Vx"]) / 3.6 for r in rows if float(r["t"]) <= 38.0]   # 红绿灯（约 860 m）之前
            offs = [abs(float(lane[r["t"]]["offset"])) for r in rows
                    if r["t"] in lane and lane[r["t"]]["offset"] not in ("", None)
                    and lane[r["t"]]["in_junction"] not in ("True", "1", "true")]
            rms = math.sqrt(sum(o * o for o in offs) / len(offs)) if offs else float("nan")
            print("INFO %s: %.0f m in %.0f s wall (%.2fx real time), speed %.1f ~ %.1f m/s, lane offset rms %.3f "
                  "max %.3f m, lane changes %d, end lane %+d, closest gap %s, solve %.1f ms" % (
                      name, k.get("distance") or 0, wall, case["duration"] / wall, min(v), max(v), rms,
                      max(offs) if offs else float("nan"), changes, lane_end, gap, solve_ms))
            for m in algo:
                if any(w in m for w in ("决定", "取消", "让行", "进入", "制动", "RSS", "结束")):
                    print("ALGO " + m)
            check("%s: the run ends by its duration" % name,
                  st["state"] == "finished" and st["detail"].startswith("达到设定的运行时长"),
                  (st, [e["msg"] for e in msgs if e.get("level") in ("error", "warn")][-5:]))
            check("%s: no algorithm error" % name, not any("Traceback" in m or "Error" in m for m in algo), algo[-5:])
            check("%s: no collision" % name, (k.get("collisions") or 0) == 0,
                  (k.get("collisions"), k.get("first_collision_with"), k.get("first_collision_t")))
            check("%s: never off the driving lanes" % name, (k.get("time_off_lane") or 0.0) == 0.0, k.get("time_off_lane"))
            check("%s: never rolls backwards (> -0.2 m/s)" % name, min(v_all) > -0.2, round(min(v_all), 2))
            stops = [m for m in algo if "灯：在停止线前停车" in m]
            if stops:
                print("INFO %s: stopped for a light: %s; speed at the end %.1f m/s" % (name, stops[0], v_all[-1]))
            decided = [float(m.split("t = ")[1].split(" s")[0]) for m in algo if "决定换到" in m or "取消换道" in m]
            burst = max([sum(1 for u in decided if 0 <= u - x < 2.0) for x in decided] or [0])
            check("%s: decisions do not flicker (<= 2 lane-change decisions within any 2 s)" % name, burst <= 2,
                  (burst, decided))
            check("%s: the solver keeps up (mean < 250 ms per frame on the server CPU)" % name, solve_ms < 250.0, solve_ms)
            if case["closures"] or case["actors"]:
                check("%s: never closer than 0.3 m to anything" % name, gap is not None and gap > 0.3, gap)
            if name == "没有障碍":
                check("%s: never changes lanes" % name, changes == 0, changes)
                check("%s: lane offset rms < 0.25 m, max < 0.6 m" % name,
                      len(offs) > 300 and rms < 0.25 and max(offs) < 0.6, (round(rms, 3), round(max(offs), 3)))
                straight = v[30:250]   # 3 ~ 25 s：弯道之前的直道（采样 0.1 s）
                check("%s: holds 72 km/h on the straight (19 ~ 21 m/s, 3 ~ 25 s)" % name,
                      19.0 < min(straight) and max(straight) < 21.0, (round(min(straight), 2), round(max(straight), 2)))
                check("%s: slows for the curve only to its planned speed (> %.1f m/s)" % (name, V_CURVE_MIN),
                      min(v) > V_CURVE_MIN, round(min(v), 2))
            if name == "封闭本车道":
                end_m = 250 + 40 + 100
                check("%s: changed lanes around the closure" % name, changes >= 1, changes)
                check("%s: went past the closure (%d m)" % (name, end_m), (k.get("distance") or 0) > end_m + 20,
                      k.get("distance"))
                check("%s: did not slow below the curve speed (%.1f m/s)" % (name, V_CURVE_MIN), min(v) > V_CURVE_MIN,
                      round(min(v), 2))
                check("%s: back in the start lane at the end" % name, lane_end == 0, lane_end)
            if name == "前车慢行":
                check("%s: overtook on the left and came back" % name,
                      changes >= 2 and lane_end == 0 and any("左侧" in m and "决定" in m for m in algo),
                      (changes, lane_end))
                check("%s: kept speed (> %.1f m/s, the curve speed)" % (name, V_CURVE_MIN), min(v) > V_CURVE_MIN,
                      round(min(v), 2))
    finally:
        try:
            if c is not None:
                c.call("cosim_stop")
        except Exception:
            pass
        proc.terminate()
        try:
            proc.wait(20)
        except subprocess.TimeoutExpired:
            proc.kill()
        shutil.rmtree(tmp, ignore_errors=True)
    print("ALL TOPO NMPC CARLA TESTS PASSED" if not FAILS else "FAILED: %s" % FAILS)
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
