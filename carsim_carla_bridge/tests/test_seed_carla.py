"""Traffic seed against a running CARLA: the same seed puts the same cars and
pedestrians in the same places (and, in synchronous mode, they then move the
same way), and every run starts with the traffic lights at the start of their
cycle. Uses mock CarSim, writes no run log and collects nothing.

    python tests/test_seed_carla.py [--port 2000]
"""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
from test_backend import Conn, check  # noqa: E402

PORT = 57181
SPAWN = 3


def run_until(c, states, timeout=60):
    return c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in states, timeout)


def traffic(w, before):
    """(type, colour, x, y) of every traffic car and pedestrian new since `before`."""
    out = []
    for a in w.get_actors():
        if a.id in before or not (a.type_id.startswith("walker.pedestrian.") or
                                  (a.type_id.startswith("vehicle.") and a.attributes.get("role_name") == "autopilot")):
            continue
        loc = a.get_location()
        out.append((a.type_id, a.attributes.get("color"), loc.x, loc.y))
    return out


def same(a, b, tol):
    """The same actors (type, colour) at the same places, within tol metres."""
    if len(a) != len(b):
        return False
    rest = list(b)
    for t, col, x, y in a:
        m = next((r for r in rest if r[0] == t and r[1] == col and abs(r[2] - x) < tol and abs(r[3] - y) < tol), None)
        if m is None:
            return False
        rest.remove(m)
    return True


def lights(w):
    return {tl.id: (str(tl.state), tl.get_elapsed_time()) for tl in w.get_actors().filter("traffic.traffic_light")}


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    import carla
    cl = carla.Client("localhost", carla_port)
    cl.set_timeout(60)
    w = cl.get_world()
    original = w.get_settings()
    proc = subprocess.Popen([sys.executable, os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT)],
                            cwd=os.path.join(HERE, ".."))
    try:
        time.sleep(2)
        c = Conn(PORT)
        c.call("connect", host="localhost", port=carla_port)
        # Synchronous and not ticked while idle: the world moves only when a command or a run ticks it.
        c.call("world_settings", synchronous=True, frame_dt=0.05, idle_tick=False)
        cfg = c.call("default_config")
        cfg["carsim"]["mock"] = True
        cfg["run"]["log_path"] = ""
        cfg["collect"]["enabled"] = False
        cfg["carla"]["spawn_index"] = SPAWN
        cfg["sync"]["frame_dt"] = 0.05
        cfg["sync"]["duration"] = 1.0

        def spawn(seed):
            c.call("clear_traffic")
            c.call("spawn_ego", blueprint="vehicle.tesla.model3", spawn_index=SPAWN)
            before = {a.id for a in w.get_actors()}
            c.call("spawn_traffic", vehicles=15, walkers=15, seed=seed)
            time.sleep(1.0)  # this client's snapshot catches up with the backend's frame
            return before, traffic(w, before)

        cars = lambda t: [x for x in t if x[0].startswith("vehicle.")]
        walkers = lambda t: [x for x in t if x[0].startswith("walker.")]

        # ---- the same seed places the same traffic -----------------------------
        a, b, other = spawn(3)[1], spawn(3)[1], spawn(4)[1]
        check("seed 3 spawns cars and pedestrians", len(cars(a)) >= 10 and len(walkers(a)) >= 5,
              (len(cars(a)), len(walkers(a))))
        check("same seed: same cars (model, colour, place)", same(cars(a), cars(b), 0.2), (len(cars(a)), len(cars(b))))
        check("same seed: same pedestrians (model, place)", same(walkers(a), walkers(b), 0.2),
              (len(walkers(a)), len(walkers(b))))
        check("another seed: other pedestrians", not same(walkers(a), walkers(other), 0.2))

        # ---- every run starts the traffic lights at the start of their cycle ---
        c.call("clear_traffic")
        seen = []
        for idle in (0.0, 3.0):
            # Let the lights run on for a different time before each run.
            c.call("world_settings", idle_tick=True)
            time.sleep(idle)
            c.call("world_settings", idle_tick=False)
            c.events.clear()
            c.call("cosim_start", config=cfg)
            st = run_until(c, ("finished", "error", "stopped"), 120)
            check("short run finishes", st["state"] == "finished", st)
            time.sleep(1.0)
            seen.append(lights(w))
        l0, l1 = seen
        check("the map has traffic lights", len(l0) > 0, len(l0))
        check("every run starts the traffic lights from the start of their cycle",
              l0.keys() == l1.keys() and all(l0[k][0] == l1[k][0] and abs(l0[k][1] - l1[k][1]) < 0.05 for k in l0),
              sum(1 for k in l0 if k in l1 and l0[k][0] != l1[k][0]))

        # ---- in synchronous mode the traffic then moves the same way -----------
        after = []
        for _ in range(2):
            before, _placed = spawn(3)
            c.events.clear()
            c.call("cosim_start", config=cfg)
            st = run_until(c, ("finished", "error", "stopped"), 120)
            check("run with traffic finishes", st["state"] == "finished", st)
            time.sleep(1.0)
            after.append(traffic(w, before))
        check("same seed, synchronous mode: the traffic is in the same places after a run",
              same(cars(after[0]), cars(after[1]), 0.5) and same(walkers(after[0]), walkers(after[1]), 0.5),
              (len(after[0]), len(after[1])))
        c.call("clear_traffic")
        c.call("destroy_ego")
    finally:
        proc.terminate()
        proc.wait()
        w.apply_settings(original)
    print("ALL SEED TESTS PASSED")


if __name__ == "__main__":
    main()
