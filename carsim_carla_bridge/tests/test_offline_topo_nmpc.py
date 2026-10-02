"""The multi-topology NMPC (controllers/topo_nmpc) without CARLA: closed loops on synthetic
multi-lane roads (tests/topo_nmpc_sim.py) with a plant that is NOT the prediction model
(other Pacejka tyres, 10 % less grip, a 50 ms steering delay and a second-order steering
response, drag), scenes in the platform's format (units, ego, lane, objects within 50 m).

Scenarios (run in parallel, one process each) and what each must show:
    curves        R 80 / R 60 bends: lane offset rms < 0.15 m, max < 0.45 m, slows for the bends
    s_bends_100   100 km/h S-bends (R 250): rms < 0.12 m, max < 0.3 m
    overtake      a car at 36 km/h 55 m ahead: passes it on the LEFT at speed (> 17 m/s), comes back
    closure       cones close the lane (taper + a line along its left edge): goes round, > 0.5 m off
                  every cone, does not slow below 17 m/s
    blocked_left  a slow car ahead, faster cars coming in both neighbour lanes: no collision, waits
                  until the left lane is free (no lane change in the first 3 s), then overtakes on the left
                  and is back up to speed (the lane value beyond the 4 s horizon)
    lead_brake    one lane, the car ahead brakes at 6 m/s^2 to a stop: stops behind it (> 3 m), no reversing
    red_light     one lane, red light 150 m ahead: stops before the line (front axle 144 ~ 149 m), says why
    cut_in        a car moves into the ego's lane 25 m ahead: no collision (> 1 m)
    pedestrian    someone crosses the lane: brakes, > 2 m away, stays in its lane
Every scenario: no collision, the lateral acceleration within the tyre grip (< 7.5 m/s^2), no
algorithm error. Plus unit checks: units (speeds in m/s, angles in rad on the CarSim page), the
"carsim" output (throttle / brake / steering wheel), the model's straight-line equilibrium.

    python tests/test_offline_topo_nmpc.py [-j N]
"""
import contextlib
import importlib.util
import io
import math
import multiprocessing as mp
import os
import sys
import time
import traceback

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CTL_DIR = os.path.join(HERE, "..", "controllers", "topo_nmpc")
sys.path.insert(0, HERE)
sys.path.insert(0, CTL_DIR)


def load():
    spec = importlib.util.spec_from_file_location("topo_nmpc_controller", os.path.join(CTL_DIR, "controller.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def scenarios():
    from topo_nmpc_sim import Actor, Road, Sim, W

    def straight(L=1500):
        return Road([(L, 0.0)])

    def cones(s0, s1, step=4.0):
        out = []
        for i in range(int((s1 - s0) / step) + 1):
            frac = min(1.0, i * step / 30.0)
            out.append(Actor(s0 + i * step, d=-W / 2 + frac * (W - 0.3), kind="static", length=0.4, width=0.4))
        return out

    return {
        "curves": (lambda: Sim(Road([(100, 0.0), (150, 1 / 80), (300, 0.0), (150, -1 / 60), (500, 0)]), v0=15.0), 35, 72),
        "s_bends_100": (lambda: Sim(Road([(150, 0), (200, 1 / 250), (200, -1 / 250), (800, 0)]), v0=27.0), 35, 100),
        "overtake": (lambda: Sim(straight(), v0=20.0, actors=[Actor(60, 0, 10.0)]), 25, 72),
        "closure": (lambda: Sim(straight(), v0=20.0, actors=cones(120, 260)), 20, 72),
        "blocked_left": (lambda: Sim(straight(), v0=20.0, actors=[Actor(50, 0, 12.0), Actor(-40, 1, 28.0),
                                                                    Actor(-45, -1, 26.0)]), 25, 72),
        "lead_brake": (lambda: Sim(Road([(1500, 0.0)], lanes=(0, 0)), v0=20.0,
                                   actors=[Actor(45, 0, 20.0, brake_at=60, brake=6.0)]), 18, 72),
        "red_light": (lambda: Sim(Road([(1500, 0.0)], lanes=(0, 0)), v0=15.0, light=(150.0, [(0, 60, "red")])), 25, 72),
        "cut_in": (lambda: Sim(Road([(1500, 0.0)], lanes=(0, 1)), v0=22.0,
                               actors=[Actor(30, 1, 15.0, cut=(25, 0, 2.0))]), 20, 72),
        "pedestrian": (lambda: Sim(Road([(1500, 0.0)], lanes=(0, 0)), v0=12.0,
                                   actors=[Actor(80, d=-4.0, kind="walker", length=0.5, width=0.5, walk=(50, 1.4))]),
                       15, 72),
    }


def run_one(name):
    """One scenario in its own process: the numbers the checks need."""
    try:
        C = load()
        mk, T, kmh = scenarios()[name]
        C.TARGET_KMH = kmh
        sim = mk()
        ctl = C.Controller()
        buf = io.StringIO()
        t0 = time.time()
        with contextlib.redirect_stdout(buf):
            log = sim.run(ctl, T)
        out = buf.getvalue()
        d = np.array([r["d"] for r in log])
        lane = np.array([r["lane"] for r in log])
        return {"name": name, "wall": time.time() - t0, "out": out,
                "rms": float(np.sqrt(np.mean(d ** 2))), "dmax": float(np.max(np.abs(d))),
                "vmin": float(min(r["v"] for r in log)), "vmax": float(max(r["v"] for r in log)),
                "vend": float(log[-1]["v"]), "s_end": float(log[-1]["s"]),
                "clear": float(min(r["clear"] for r in log)), "aymax": float(max(abs(r["ay"]) for r in log)),
                "lanes": sorted(set(int(x) for x in lane)), "lane_end": int(lane[-1]),
                "first_change_t": next((r["t"] for r in log if r["lane"] != 0), None),
                "lane_at_max": [int(r["lane"]) for r in log],
                "solve_ms": 1000 * ctl.solve_s / max(ctl.n_calls, 1),
                "steer_still": float(max([abs(r["delta"]) for r in log if r["v"] < 0.1] or [0.0]))}
    except Exception:
        return {"name": name, "error": traceback.format_exc()}


FAILS = []


def check(name, cond, detail=""):
    shown = "" if isinstance(detail, str) and detail == "" else "  " + str(detail)
    print(("PASS " if cond else "FAIL ") + name + shown, flush=True)
    if not cond:
        FAILS.append(name)


def unit_checks():
    C = load()
    from nmpc_model import Vehicle, NX, VX
    from topo_nmpc_sim import Road, Sim
    # the model: straight ahead at 20 m/s with zero input stays straight
    v = Vehicle()
    x = np.zeros(NX)
    x[VX] = 20.0
    for _ in range(30):
        x = v.step(x, np.zeros(2), 0.1, lambda s: 0.0 * s)
    check("model: zero input keeps a straight line at 20 m/s", abs(x[1]) < 1e-9 and abs(x[VX] - 20.0) < 1e-9, x[:4].tolist())
    # units: the same scene in m/s and rad gives the same command as in km/h and deg
    sim = Sim(Road([(500, 1 / 200)]), v0=18.0)
    ex, sc = sim.plant.exports(), sim.scene()
    a = C.Controller()
    with contextlib.redirect_stdout(io.StringIO()):
        a.reset()
        u1 = a.control(dict(ex), 0.0, 0.05, sc)
    ex2 = dict(ex, Vx=ex["Vx"] / 3.6, Vy=ex["Vy"] / 3.6, AVz=math.radians(ex["AVz"]), Yaw=math.radians(ex["Yaw"]),
               Steer_L1=math.radians(ex["Steer_L1"]), Steer_R1=math.radians(ex["Steer_R1"]))
    sc2 = dict(sc, units={"angle": "rad", "speed": "m/s", "rate": "rad/s"},
               ego=dict(sc["ego"], Yaw=math.radians(sc["ego"]["Yaw"]), Speed=sc["ego"]["Speed"] / 3.6))
    b = C.Controller()
    with contextlib.redirect_stdout(io.StringIO()):
        b.reset()
        u2 = b.control(ex2, 0.0, 0.05, sc2)
    check("units: m/s and rad on the CarSim page give the same command", np.allclose(u1, u2, atol=1e-6), (list(u1), list(u2)))
    # the "carsim" output: three imports, steering wheel = wheel angle x ratio, throttle in 0..1
    C.OUTPUT = "carsim"
    c = C.Controller()
    with contextlib.redirect_stdout(io.StringIO()):
        c.reset()
        u3 = c.control(dict(ex), 0.0, 0.05, sc)
    C.OUTPUT = "ax_delta"
    check("OUTPUT = carsim: [throttle 0..1, brake, steering wheel deg]",
          len(u3) == 3 and 0.0 <= u3[0] <= 1.0 and u3[1] >= 0.0
          and abs(u3[2] - math.degrees(c.dc) * C.STEER_RATIO_INIT) < 1e-6, u3)


def main():
    jobs = int(sys.argv[sys.argv.index("-j") + 1]) if "-j" in sys.argv else min(9, os.cpu_count() or 1)
    unit_checks()
    names = list(scenarios())
    t0 = time.time()
    with mp.get_context("spawn").Pool(jobs) as pool:
        res = {r["name"]: r for r in pool.map(run_one, names)}
    print("INFO %d scenarios in %.0f s wall with %d processes" % (len(names), time.time() - t0, jobs))
    for n in names:
        r = res[n]
        if "error" in r:
            check("%s: runs" % n, False, r["error"][-600:])
            continue
        print("INFO %-12s rms %.3f max %.3f m  v %.1f..%.1f (end %.1f)  clear %.2f m  |ay| %.2f  lanes %s  end lane %d  "
              "solve %.0f ms  wall %.0f s" % (n, r["rms"], r["dmax"], r["vmin"], r["vmax"], r["vend"], r["clear"],
                                             r["aymax"], r["lanes"], r["lane_end"], r["solve_ms"], r["wall"]))
        out = r["out"]
        check("%s: no algorithm error" % n, "Traceback" not in out, out[-300:])
        check("%s: no collision" % n, r["clear"] > 0.0, r["clear"])
        check("%s: within the tyre grip (|ay| < 7.5 m/s^2)" % n, r["aymax"] < 7.5, r["aymax"])
        if n == "curves":
            check("curves: rms < 0.15 m, max < 0.45 m", r["rms"] < 0.15 and r["dmax"] < 0.45, (r["rms"], r["dmax"]))
            check("curves: slows for the bends (< 17 m/s)", r["vmin"] < 17.0, r["vmin"])
        if n == "s_bends_100":
            check("s_bends_100: rms < 0.12 m, max < 0.3 m", r["rms"] < 0.12 and r["dmax"] < 0.3, (r["rms"], r["dmax"]))
        if n == "overtake":
            check("overtake: passes on the left at speed and comes back",
                  1 in r["lanes"] and -1 not in r["lanes"] and r["lane_end"] == 0 and r["vmin"] > 17.0,
                  (r["lanes"], r["lane_end"], r["vmin"]))
        if n == "closure":
            check("closure: goes round, > 0.5 m off every cone, > 17 m/s",
                  1 in r["lanes"] and r["clear"] > 0.5 and r["vmin"] > 17.0, (r["lanes"], r["clear"], r["vmin"]))
        if n == "blocked_left":
            check("blocked_left: waits while both neighbour lanes have faster cars coming (no change in 3 s)",
                  r["first_change_t"] is None or r["first_change_t"] > 3.0, r["first_change_t"])
            check("blocked_left: overtakes on the left once they have passed, back up to speed (> 18 m/s at the end)",
                  1 in r["lanes"] and -1 not in r["lanes"] and r["vend"] > 18.0, (r["lanes"], r["first_change_t"], r["vend"]))
        if n == "lead_brake":
            check("lead_brake: stops behind it, > 3 m, no reversing", r["vend"] < 0.3 and r["clear"] > 3.0 and r["vmin"] >= 0.0,
                  (r["vend"], r["clear"], r["vmin"]))
        if n == "red_light":
            check("red_light: stops before the line (front axle 144 ~ 149 m)", r["vend"] < 0.3 and 144.0 < r["s_end"] < 149.0,
                  (r["vend"], r["s_end"]))
            check("red_light: says why it stops", "红灯：在停止线前停车" in out, out[:300])
            check("red_light: the steering stays put while stopped (< 0.03 rad)", r["steer_still"] < 0.03, r["steer_still"])
        if n == "cut_in":
            check("cut_in: > 1 m from the car cutting in", r["clear"] > 1.0, r["clear"])
        if n == "pedestrian":
            check("pedestrian: brakes, > 2 m away, stays in its lane", r["clear"] > 2.0 and r["vmin"] < 8.0 and r["lanes"] == [0],
                  (r["clear"], r["vmin"], r["lanes"]))
    print("ALL TOPO NMPC OFFLINE TESTS PASSED" if not FAILS else "FAILED: %s" % FAILS)
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
