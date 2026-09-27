"""The training-code path the guides document, against a running CARLA server:

    python tests/test_docs_carla.py [--port 2000]

- The guides' snippet (a GUI-saved config, synchronous mode with
  fixed_delta_seconds = inner_steps * t_step, CarlaVehicleSync with
  settings=st.to_bridge_cfg(d)) drives a mock CarSim whose export order and
  units differ from config.py: every frame the CARLA car's reference point
  and heading are CarSim's Xo / Yo / Zo / Yaw.
- Without settings= the same exports are read in config.py's order and the
  car ends up elsewhere (why the guides pass settings).
- lane.speed_limit is vehicle.get_speed_limit(): the same loop drives the car
  through a speed-limit sign's trigger box and reports whether CARLA updates
  it for a car the bridge moves. Reported, not asserted (the spec marks it
  待确认).
World settings are restored and every actor spawned here is destroyed.
"""
import json
import math
import os
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
from test_backend import check  # noqa: E402

import carla  # noqa: E402
import numpy as np  # noqa: E402
import settings as st  # noqa: E402
from bridge import CarlaVehicleSync, CarSimExports  # noqa: E402
from mock_carsim import MockCarSimEnv  # noqa: E402

SI = {"angle": "rad", "speed": "m/s", "rate": "rad/s", "wheel_spin": "rad/s", "jounce": "m"}
INNER = 20  # CarSim steps per CARLA frame: t_step 0.001 s -> 0.02 s frames


def spawn(world, transforms, bp):
    for tf in transforms:
        v = world.try_spawn_actor(bp, tf)
        if v is not None:
            return v, tf
    return None, None


def run_snippet(world, vehicle, anchor, d, bridge_cfg, steps, action, stop=None):
    """The guides' snippet with a mock CarSim writing d's export order and
    units. Returns the worst distance (m) between the CARLA car's reference
    point and CarSim's Xo / Yo / Zo, and the worst heading error (deg)."""
    env = MockCarSimEnv(d["carsim"]["export_names"], t_step=0.001, units=d["carsim"]["units"])
    env.reset()
    inner_steps = INNER
    frame_dt = inner_steps * env.t_step
    s = world.get_settings()
    s.synchronous_mode, s.fixed_delta_seconds = True, frame_dt
    world.apply_settings(s)
    for _ in range(30):
        world.tick()
    sync = CarlaVehicleSync(world, vehicle, anchor, settings=bridge_cfg)
    ex = CarSimExports(d["carsim"]["export_names"], d["carsim"]["units"])  # what CarSim really wrote
    worst_d = worst_h = 0.0
    try:
        for i in range(steps):
            obs, r, done, info = env.control_step(action, inner_steps)
            sync.sync(obs, env.t_current, frame_dt)
            world.tick()
            p, R = sync.anchor.pose_to_world((ex.raw(obs, "Xo"), ex.raw(obs, "Yo"), ex.raw(obs, "Zo")),
                                             ex.angle(obs, "Yaw"), ex.angle(obs, "Pitch"), ex.angle(obs, "Roll"))
            tf = vehicle.get_transform()
            ref = tf.transform(carla.Location(*[float(c) for c in sync.ref_local]))
            worst_d = max(worst_d, math.dist((ref.x, ref.y, ref.z), [float(c) for c in p]))
            f = tf.get_forward_vector()
            cos = max(-1.0, min(1.0, float(np.dot([f.x, f.y, f.z], R[:, 0]))))
            worst_h = max(worst_h, math.degrees(math.acos(cos)))
            if stop is not None and stop(i):
                break
    finally:
        sync.release()
    return worst_d, worst_h


def probe_speed_limit(world, bp, actors):
    """Drive the snippet's loop (default config) straight through a speed-limit
    sign and report whether get_speed_limit() follows it."""
    m = world.get_map()
    d = st.default_dict()
    signs = list(world.get_actors().filter("traffic.speed_limit.*"))
    print("INFO %d speed-limit signs on %s" % (len(signs), m.name))
    tried = 0
    for sign in signs:
        try:
            limit = float(sign.type_id.rsplit(".", 1)[1])
        except ValueError:
            continue
        stf, box = sign.get_transform(), sign.trigger_volume
        wp = m.get_waypoint(stf.transform(carla.Location(box.location.x, box.location.y, box.location.z)))
        prev = wp.previous(25.0) if wp is not None else []
        if not prev:
            continue
        a = prev[0].transform
        start = carla.Transform(carla.Location(a.location.x, a.location.y, a.location.z + 0.6), a.rotation)
        v = world.try_spawn_actor(bp, start)
        if v is None:
            continue
        actors.append(v)
        try:
            world.tick()
            before = v.get_speed_limit()
            if abs(before - limit) < 0.5:
                continue  # passing this sign would not change it
            tried += 1
            inside = []

            def stop(i):
                c = v.get_transform().transform(carla.Location(v.bounding_box.location.x, v.bounding_box.location.y,
                                                               v.bounding_box.location.z))
                if not inside and box.contains(c, stf):
                    inside.append(i)
                return bool(inside) and i >= inside[0] + 10

            run_snippet(world, v, start, d, st.to_bridge_cfg(d), 400, [0.6, 0.0, 0.0], stop)
            world.tick()
            after = v.get_speed_limit()
            if abs(after - before) > 0.5:
                print("INFO speed_limit: %g -> %g km/h after the bridge moved the car through %s: CARLA updates it"
                      % (before, after, sign.type_id))
                return
            if inside:
                print("INFO speed_limit: stayed %g km/h after the bridge moved the car through %s (sign %g): "
                      "CARLA does NOT update it for a car the bridge moves" % (before, sign.type_id, limit))
                return
            print("INFO %s: the car never entered its trigger box (the road bends?); trying the next sign" % sign.type_id)
        finally:
            v.destroy()
            actors.remove(v)
        if tried >= 5:
            break
    print("INFO speed_limit not probed: no reachable speed-limit sign with another limit on this map")


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    client = carla.Client("localhost", carla_port)
    client.set_timeout(60.0)
    world = client.get_world()
    original = world.get_settings()
    bp = world.get_blueprint_library().find("vehicle.tesla.model3")
    if bp.has_attribute("role_name"):
        bp.set_attribute("role_name", "docs_test")
    actors = []
    tmp = tempfile.mkdtemp(prefix="cc_docs_")
    try:
        # A config the way the GUI saves it, with an export order and units
        # config.py does not have.
        d = st.default_dict()
        d["carsim"]["export_names"] = list(reversed(d["carsim"]["export_names"]))
        d["carsim"]["units"] = dict(SI)
        path = os.path.join(tmp, "cosim_config.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(d, f, ensure_ascii=False, indent=2)
        d = st.load_dict(path)

        points = world.get_map().get_spawn_points()
        v, anchor = spawn(world, points[3:] + points[:3], bp)
        check("a free spawn point for the test car", v is not None)
        actors.append(v)
        dist, head = run_snippet(world, v, anchor, d, st.to_bridge_cfg(d), 150, [0.5, 0.0, 60.0])
        check("the guides' snippet (settings=st.to_bridge_cfg(d), synchronous mode): the CARLA car is where "
              "CarSim says, other export order and SI units", dist < 0.05 and head < 0.5, (dist, head))

        v.destroy()
        actors.remove(v)
        world.tick()
        v, anchor = spawn(world, [anchor], bp)
        check("the test car respawns at the same point", v is not None)
        actors.append(v)
        dist, _ = run_snippet(world, v, anchor, d, None, 150, [0.5, 0.0, 60.0])
        check("... without settings= config.py's export order is read: the car is elsewhere", dist > 2.0, dist)
        v.destroy()
        actors.remove(v)
        world.tick()

        probe_speed_limit(world, bp, actors)
    finally:
        for a in actors:
            try:
                a.destroy()
            except RuntimeError:
                pass
        world.apply_settings(original)
        shutil.rmtree(tmp, ignore_errors=True)
    print("ALL DOCS TESTS PASSED")


if __name__ == "__main__":
    main()
