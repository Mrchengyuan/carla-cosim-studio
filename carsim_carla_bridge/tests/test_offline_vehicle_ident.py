"""不需要 CARLA：车辆参数辨识（vehicle_ident.py）。

用 KMPPI 的 3DOF 预测模型（轮胎比例因子设成已知的 0.70 / 0.82，和原工程 MATLAB 的一样）做一次 20 m/s 的蛇行，
按 CarSim 的方式写成一次运行的记录（Vx / Vy 是前轴中心的，km/h、deg、deg/s，采样 0.1 s）：
辨识出的比例因子和设的相差 < 3%、R² > 0.95；同样的数据用 m/s、rad、rad/s 记录时结果相同；
一直直行时说转向太少；缺导出变量时说缺哪个；不是运行记录时说清；保存的文件 KMPPI 读得进
（controller.py 的 VEHICLE_JSON）；后端命令走文件线程。

    python tests/test_offline_vehicle_ident.py
"""
import contextlib
import csv
import io
import json
import math
import os
import shutil
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
KDIR = os.path.join(HERE, "..", "controllers", "kmppi")
sys.path.insert(0, KDIR)

import vehicle_ident  # noqa: E402
from kmppi_config import BicycleParams  # noqa: E402
from prediction_model import BicycleModel  # noqa: E402

FAILS = []
M, I, A, B = 1910.0, 3482.0, 1.371, 1.386


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + (("  " + str(detail)) if detail != "" else ""), flush=True)
    if not cond:
        FAILS.append(name)


def make_run(folder, steer_amp=0.03, si=False, drop=None):
    """A run record from the 3DOF model (scales 0.70 / 0.82): 20 m/s, a weave."""
    p = BicycleParams(m=M, I=I, a=A, b=B, front_lateral_scale=0.70, rear_lateral_scale=0.82)
    dt = 0.002
    model = BicycleModel(p, dt, 5.0, 0.5)
    s = np.array([[0.0, 0.0, 0.0, 20.0, 0.0, 0.0]])
    os.makedirs(folder, exist_ok=True)
    units = {"angle": "rad", "speed": "m/s", "rate": "rad/s"} if si else {"angle": "deg", "speed": "km/h", "rate": "deg/s"}
    json.dump({"map": "Town04", "units": units}, open(os.path.join(folder, "run.json"), "w", encoding="utf-8"))
    names = ["t", "frame", "Vx", "Vy", "AVz", "Steer_L1", "Steer_R1"]
    if drop:
        names.remove(drop)
    ang = 1.0 if si else 180.0 / math.pi
    spd = 1.0 if si else 3.6
    with open(os.path.join(folder, "log.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(names)
        n = int(12.0 / dt)
        for i in range(n + 1):
            t = i * dt
            delta = steer_amp * math.sin(2 * math.pi * 0.4 * t) + 0.5 * steer_amp * math.sin(2 * math.pi * 1.1 * t)
            if i % 50 == 0:   # every 0.1 s
                vx, vy, r = s[0, 3], s[0, 4], s[0, 5]
                row = {"t": round(t, 3), "frame": i // 50, "Vx": vx * spd, "Vy": (vy + A * r) * spd,   # the front axle centre's
                       "AVz": r * ang, "Steer_L1": delta * ang, "Steer_R1": delta * ang}
                w.writerow([row[k] for k in names])
            s = model.step(s, np.array([[0.0, delta]]))
    return folder


def err_of(fn):
    try:
        fn()
        return ""
    except Exception as e:  # noqa: BLE001
        return str(e)


def main():
    tmp = tempfile.mkdtemp(prefix="cc_ident_test_")
    try:
        run = make_run(os.path.join(tmp, "weave"))
        r = vehicle_ident.identify(run, M, I, A, B)
        ef = abs(r["front_lateral_scale"] / 0.70 - 1)
        er = abs(r["rear_lateral_scale"] / 0.82 - 1)
        check("a weave at 20 m/s: the tire scales found (0.70 / 0.82, < 3%)", ef < 0.03 and er < 0.03,
              (round(r["front_lateral_scale"], 4), round(r["rear_lateral_scale"], 4)))
        check("... a good fit (R² > 0.95), the samples above 5 m/s, the lateral acceleration said",
              r["r2"] > 0.95 and r["samples"] > 100 and r["ay_max"] > 1.0, (round(r["r2"], 4), r["samples"], round(r["ay_max"], 2)))
        r_si = vehicle_ident.identify(make_run(os.path.join(tmp, "weave_si"), si=True), M, I, A, B)
        check("the same run in m/s, rad, rad/s: the same result",
              abs(r_si["front_lateral_scale"] - r["front_lateral_scale"]) < 1e-6 and
              abs(r_si["rear_lateral_scale"] - r["rear_lateral_scale"]) < 1e-6,
              (r_si["front_lateral_scale"], r["front_lateral_scale"]))
        e = err_of(lambda: vehicle_ident.identify(make_run(os.path.join(tmp, "straight"), steer_amp=0.0), M, I, A, B))
        check("driving straight: too little steering, said", "转向太少" in e, e)
        e = err_of(lambda: vehicle_ident.identify(make_run(os.path.join(tmp, "noavz"), drop="AVz"), M, I, A, B))
        check("an export missing: which one, said", "AVz" in e and "没有导出变量" in e, e)
        e = err_of(lambda: vehicle_ident.identify(tmp, M, I, A, B))
        check("not a run: said", "不是一次运行的记录" in e, e)
        e = err_of(lambda: vehicle_ident.identify(run, 0, I, A, B))
        check("a mass of 0: said", "大于 0" in e, e)

        # Saved: a KMPPI vehicle file controller.py reads (VEHICLE_JSON).
        path = vehicle_ident.save(os.path.join(tmp, "k", "vehicle_identified.json"), run, M, I, A, B, r)
        d = json.load(open(path, encoding="utf-8"))
        check("saved: the keys KMPPI reads", all(k in d for k in ("m", "I", "a", "b", "front_lateral_scale", "rear_lateral_scale",
                                                                  "pac_By", "pac_Cy", "pac_Ey")) and d["fit"]["samples"] == r["samples"], sorted(d))
        import controller as ctl
        ctl.VEHICLE_JSON = path
        ctl.USE_GPU = False
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            c = ctl.Controller()
            c.reset()
        check("controller.py with VEHICLE_JSON: the prediction model uses it, said",
              abs(c.vehicle.front_lateral_scale - r["front_lateral_scale"]) < 1e-12 and
              abs(c.ctrl.dynamics_fn.step.__self__.p.rear_lateral_scale - r["rear_lateral_scale"]) < 1e-12 and
              "车辆参数用" in buf.getvalue(), buf.getvalue().strip()[:120])
        ctl.VEHICLE_JSON = ""
        with contextlib.redirect_stdout(io.StringIO()):
            c = ctl.Controller()
            c.reset()
        check("... without it: the Chrono BMW's", abs(c.vehicle.front_lateral_scale - 1.0232755573116097) < 1e-9,
              c.vehicle.front_lateral_scale)

        import backend_server
        check("the backend command vehicle_identify runs on the file thread",
              "vehicle_identify" in backend_server.IO_CMDS and hasattr(backend_server.Backend, "cmd_vehicle_identify"))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print("FAILED: %s" % FAILS if FAILS else "ALL VEHICLE IDENT TESTS PASSED")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
