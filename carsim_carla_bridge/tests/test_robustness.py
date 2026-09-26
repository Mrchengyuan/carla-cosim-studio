"""The backend under abuse: bad user code, bad input, reconnects, failed
starts. After every case the backend must still answer and the world must
hold exactly what it should.

    python tests/test_robustness.py [--port 2000]
"""
import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
from test_backend import Conn, check  # noqa: E402

PORT = 57193


def expect_error(c, cmd, text, **args):
    try:
        c.call(cmd, **args)
    except RuntimeError as e:
        return text in str(e), str(e)[-90:]
    return False, "no error"


def heroes(w):
    return [a for a in w.get_actors().filter("vehicle.*") if a.attributes.get("role_name") == "hero"]


def run_until(c, states, timeout=60):
    return c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in states, timeout)


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    import carla
    cl = carla.Client("localhost", carla_port)
    cl.set_timeout(60)  # the modified CARLA (editor build) answers slowly right after start
    w = cl.get_world()
    tmp = tempfile.mkdtemp(prefix="cc_rob_")
    proc = subprocess.Popen([sys.executable, os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT)],
                            cwd=os.path.join(HERE, ".."))
    try:
        time.sleep(2)
        c = Conn(PORT)
        c.call("connect", host="localhost", port=carla_port)
        base = c.call("default_config")
        base["carsim"]["mock"] = True
        base["run"]["log_path"] = ""
        base["carla"]["spawn_index"] = 3
        c.call("spawn_ego", blueprint="vehicle.tesla.model3", spawn_index=3)

        def ctrl(name, body):
            p = os.path.join(tmp, name)
            with open(p, "w", encoding="utf-8") as f:
                f.write(body)
            cfg = json.loads(json.dumps(base))
            cfg["run"]["controller"] = {"path": p, "entry": "Controller"}
            return cfg

        # ---- user code that calls sys.exit at import (argparse at module level)
        cfg = ctrl("exit_on_import.py", "import sys\nsys.exit(2)\nclass Controller:\n    pass\n")
        ok, msg = expect_error(c, "cosim_start", "sys.exit", config=cfg)
        check("controller calling sys.exit at import: clear error", ok, msg)
        check("backend still answers after that", c.call("ping") == "pong")

        # ---- user code that raises SystemExit / returns NaN during the run ------
        for name, body, text in (
                ("exit_in_control.py", "class Controller:\n    def control(self, e, t, dt):\n        raise SystemExit(3)\n", ""),
                ("nan_out.py", "class Controller:\n    def control(self, e, t, dt):\n        return [float('nan'), 0.0, 0.0]\n", "无效数值"),
                # CarSim itself would fill the missing import (steering) with 0 silently
                ("two_values.py", "class Controller:\n    def control(self, e, t, dt):\n        return [0.3, 0.0]\n", "个导入变量"),
                ("raises.py", "class Controller:\n    def control(self, e, t, dt):\n        raise ValueError('boom')\n",
                 "ValueError: boom（raises.py 第 3 行）")):
            c.events.clear()
            c.call("cosim_start", config=ctrl(name, body))
            st = run_until(c, ("error", "finished", "stopped"))
            check("run with %s ends in error state" % name, st["state"] == "error" and text in st.get("detail", ""),
                  (st["state"], st.get("detail", "")[:60]))
            check("backend answers after %s" % name, c.call("ping") == "pong" and c.call("world_info")["cosim_state"] == "error")

        # ---- malformed request line --------------------------------------------
        c.s.sendall(b"{this is not json\n")
        c.events.clear()
        check("malformed request does not end the backend", c.call("ping") == "pong")
        c.call("ping")
        check("malformed request is reported", any(e.get("event") == "log" and "无法解析" in e.get("msg", "") for e in c.events))

        # ---- stop with nothing running still reports a state --------------------
        c.events.clear()
        c.call("cosim_stop")
        c.call("ping")
        check("cosim_stop always reports the state", any(e.get("event") == "cosim_state" for e in c.events))

        # ---- bad requests never lose the current ego --------------------------
        ego0 = c.call("world_info")["ego_id"]
        ok, msg = expect_error(c, "spawn_ego", "没有车型", blueprint="vehicle.no.such_car", spawn_index=3)
        check("unknown vehicle: clear error", ok, msg)
        check("unknown vehicle: ego kept", c.call("world_info")["ego_id"] == ego0 and len(heroes(w)) == 1)
        c.call("add_sensor", type="imu")
        c.call("views_set", views=[{"id": "p0", "kind": "rgb", "mode": "chase", "width": 160, "height": 90, "fps": 5}])
        bad = json.loads(json.dumps(base))
        bad["carla"]["vehicle"] = "vehicle.no.such_car"
        ok, msg = expect_error(c, "cosim_start", "没有车型", config=bad)
        kinds = [s["type"] for s in c.call("list_sensors")]
        check("run with unknown vehicle: error, ego / sensors kept", ok and c.call("world_info")["ego_id"] == ego0 and kinds == ["imu"],
              (msg, kinds))
        ok, msg = expect_error(c, "views_set", "不能作为视图", views=[{"id": "p1", "kind": "imu"}])
        check("imu as a view is refused before touching the views", ok, msg)
        c.events.clear()
        c.call("views_set", views=[{"id": "p0", "kind": "rgb", "mode": "chase", "width": 160, "height": 90, "fps": 5}])
        check("views still work after the refusal", c.wait_event(lambda e: e.get("event") == "frame", 20)["w"] == 160)
        c.events.clear()
        ok, msg = expect_error(c, "views_set", "", views=[{"id": "p0", "kind": "rgb", "mount": {"x": "not a number"}}])
        c.call("ping")
        va = [e["ids"] for e in c.events if e.get("event") == "views_active"]
        check("a view that cannot be built: error, and the GUI is told no views are left",
              ok and va == [[]] and not [a for a in w.get_actors().filter("sensor.camera.*") if a.parent and a.parent.id == ego0], va)
        c.call("views_set", views=[{"id": "p0", "kind": "rgb", "mode": "chase", "width": 160, "height": 90, "fps": 5}])

        # ---- CARLA interface "auto" works whichever client / server pair it is --
        guess = c.call("world_info")["external_api_server"]  # from the versions at connect
        info = c.call("cosim_start", config=json.loads(json.dumps(base)))
        c.call("cosim_stop")
        if hasattr(carla.Vehicle, "apply_external_state"):  # patched client
            check("auto interface: runs, uses the API only if the server has it; the guess at connect agrees",
                  info["server_api"] in (True, False) and info["external_api"] == info["server_api"]
                  and guess in (None, info["server_api"])
                  and c.call("world_info")["external_api_server"] == info["server_api"], (guess, info["server_api"]))
            if info["server_api"] is False:
                forced = json.loads(json.dumps(base))
                forced["sync"]["use_external_api"] = True
                ok, msg = expect_error(c, "cosim_start", "不是改版", config=forced)
                check("forced patched interface on an original server: clear error", ok, msg)
        else:
            check("auto interface with the original client: compatibility mode",
                  info["external_api"] is False and c.call("world_info")["external_api_server"] is None)

        # ---- measuring every vehicle, trucks with 6 wheels included ------------
        # (the read order matters: the other one crashes development builds of CARLA)
        specs = c.call("vehicle_specs", timeout=600)
        n_bp = len(w.get_blueprint_library().filter("vehicle.*"))
        hgv = specs.get("vehicle.carlamotors.european_hgv", {})
        check("vehicle_specs measures every vehicle, multi-wheel trucks included",
              len(specs) == n_bp and len(hgv.get("wheel_radius_m", [])) == 6 and c.call("ping") == "pong",
              "%d / %d vehicles, european_hgv wheels %d" % (len(specs), n_bp, len(hgv.get("wheel_radius_m", []))))

        # ---- CARLA removes the ego by itself (a car that fell off the map) -----
        # is_alive stays True for an actor someone else destroyed: the backend
        # must notice anyway, end the run with the reason and forget the car.
        other = carla.Client("localhost", carla_port)
        other.set_timeout(60)
        ow = other.get_world()
        c.events.clear()
        c.call("cosim_start", config=json.loads(json.dumps(base)))
        time.sleep(1.0)
        ow.get_actor(c.call("world_info")["ego_id"]).destroy()
        st = run_until(c, ("error", "finished", "stopped"), 30)
        lost = c.wait_event(lambda e: e.get("event") == "ego_lost", 10)
        check("ego removed by CARLA during a run: the run ends with the reason, the ego is forgotten",
              st["state"] == "error" and "主车已不在" in st.get("detail", "") and bool(lost)
              and c.call("world_info")["ego_id"] == 0, st.get("detail", "")[:60])
        c.call("spawn_ego", blueprint="vehicle.tesla.model3", spawn_index=3)
        c.call("views_set", views=[{"id": "p0", "kind": "rgb", "mode": "chase", "width": 160, "height": 90, "fps": 5}])
        c.events.clear()
        ow.get_actor(c.call("world_info")["ego_id"]).destroy()
        lost = c.wait_event(lambda e: e.get("event") == "ego_lost", 15)
        c.call("ping")
        va = [e["ids"] for e in c.events if e.get("event") == "views_active"]
        ok, msg = expect_error(c, "views_set", "请先生成主车",
                               views=[{"id": "p0", "kind": "rgb", "mode": "chase", "width": 160, "height": 90, "fps": 5}])
        check("ego removed while idle: the GUI is told, views are gone, a new view asks for an ego",
              bool(lost) and va == [[]] and ok and c.call("world_info")["ego_id"] == 0, (va, msg))
        c.call("spawn_ego", blueprint="vehicle.tesla.model3", spawn_index=3)
        c.call("views_set", views=[{"id": "p0", "kind": "rgb", "mode": "chase", "width": 160, "height": 90, "fps": 5}])

        # ---- a traffic car thrown by a collision must not freeze the run -------
        # CARLA 0.9.16's traffic manager (inside this process) then loops forever
        # and world.tick() never returns; our patched carla package bounds that
        # loop. The original package cannot be helped (the GUI offers a restart).
        if hasattr(carla.Vehicle, "apply_external_state"):
            c.call("spawn_traffic", vehicles=8, walkers=0, seed=5)
            c.events.clear()
            c.call("cosim_start", config=json.loads(json.dumps(base)))
            ow.wait_for_tick(10)
            car = [a for a in ow.get_actors().filter("vehicle.*") if a.attributes.get("role_name") == "autopilot"][0]
            t1 = c.wait_event(lambda e: e.get("event") == "telemetry", 20)["data"]["t"]
            car.add_impulse(carla.Vector3D(0, 0, 3e5))
            try:
                t2 = c.wait_event(lambda e: e.get("event") == "telemetry" and e["data"]["t"] > t1 + 4.0, 40)["data"]["t"]
            except (TimeoutError, OSError):
                t2 = None
            check("a traffic car thrown into the air does not freeze the run", t2 is not None, "t %.1f -> %s" % (t1, t2))
            c.call("cosim_stop")
            c.call("clear_traffic")

        # ---- co-sim: the car sits on the road, settings are locked, ... ------
        c.call("spawn_ego", blueprint="vehicle.tesla.model3", spawn_index=3)
        c.call("cosim_start", config=json.loads(json.dumps(base)))
        time.sleep(1.5)
        ow.wait_for_tick(10)
        ego = ow.get_actor(c.call("world_info")["ego_id"])
        tf, bb = ego.get_transform(), ego.bounding_box
        gap = tf.location.z + bb.location.z - bb.extent.z - ow.get_map().get_waypoint(tf.location).transform.location.z
        check("co-sim (CarSim height): the car sits on the road, it does not float above the spawn point",
              abs(gap) < 0.15, "bottom %.3f m above the road" % gap)
        ok, msg = expect_error(c, "world_settings", "运行中", synchronous=False)
        check("simulation settings cannot be changed during a run", ok, msg)
        c.call("cosim_stop")
        # keyboard driving: when the GUI stops sending keys, the car does not drive on
        kb0 = json.loads(json.dumps(base))
        kb0["drive"].update({"dynamics": "carla", "carla_driver": "manual"})
        c.events.clear()
        c.call("cosim_start", config=kb0)
        for _ in range(60):
            c.call("manual_control", throttle=1.0, brake=0.0, steer=0.0)
            time.sleep(0.05)
        v_on = max([e["data"]["speed_kmh"] for e in c.events if e.get("event") == "telemetry"] or [0.0])
        time.sleep(3.0)
        c.call("ping")
        v_off = [e["data"]["speed_kmh"] for e in c.events if e.get("event") == "telemetry"][-1]
        c.call("cosim_stop")
        check("keyboard drive: no keys for a while = throttle released (dead man's switch)", v_on > 5.0 and v_off < 0.7 * v_on,
              "%.1f -> %.1f km/h" % (v_on, v_off))
        # collection with an event sensor must not wait 5 s per frame; a reused session name gets its own folder
        col = json.loads(json.dumps(base))
        col["rig"]["sensors"] = [{"name": "cam", "type": "rgb", "x": 1.5, "y": 0.0, "z": 1.6, "pitch": 0.0, "yaw": 0.0, "roll": 0.0,
                                  "attributes": {"image_size_x": 320, "image_size_y": 180, "fov": 90}},
                                 {"name": "bump", "type": "collision", "x": 0.0, "y": 0.0, "z": 0.0, "pitch": 0.0, "yaw": 0.0, "roll": 0.0,
                                  "attributes": {}}]
        col["collect"].update({"enabled": True, "out_dir": os.path.join(tmp, "ds"), "session": "same", "max_frames": 10, "max_gb": 1.0})
        roots = []
        for _ in range(2):
            c.events.clear()
            t0 = time.time()
            c.call("cosim_start", config=col)
            st = run_until(c, ("finished", "error"), 120)
            roots.append(sorted(os.listdir(os.path.join(tmp, "ds"))))
        check("collection with a collision sensor runs at full speed", st["state"] == "finished" and time.time() - t0 < 30,
              "%s in %.0f s" % (st["state"], time.time() - t0))
        check("a reused session name never writes into the old session", roots[-1] == ["same", "same_2"], roots[-1])

        # ---- a large live view does not freeze the run ------------------------
        # A call that waits for CARLA while holding the GIL (get_wheel_steer_angle
        # in the original package) starved the view callback, and CARLA, stuck
        # sending it camera data, never answered: the keyboard drive froze.
        c.call("spawn_ego", blueprint="vehicle.tesla.model3", spawn_index=3)
        c.call("views_set", views=[{"id": "p0", "kind": "rgb", "mode": "chase", "width": 960, "height": 540, "fps": 20}])
        kb = json.loads(json.dumps(base))
        kb["drive"].update({"dynamics": "carla", "carla_driver": "manual"})
        c.events.clear()
        c.call("cosim_start", config=kb)
        t_end, sim_t = time.time() + 10.0, 0.0
        try:
            while time.time() < t_end:
                c.call("manual_control", throttle=0.5, brake=0.0, steer=0.2, timeout=25)
                tel = [e["data"]["t"] for e in c.events if e.get("event") == "telemetry"]
                sim_t = tel[-1] if tel else sim_t
                c.events = []
                time.sleep(0.05)
        except (TimeoutError, OSError):
            pass
        c.call("cosim_stop", timeout=60)
        check("keyboard drive with a 960x540 live view keeps running", sim_t > 2.0, "sim %.1f s in 10 s" % sim_t)
        c.call("views_set", views=[{"id": "p0", "kind": "rgb", "mode": "chase", "width": 160, "height": 90, "fps": 5}])

        # ---- traffic standing on the ego's spawn point does not block a run ----
        c.call("destroy_ego")
        n = c.call("spawn_traffic", vehicles=300, walkers=0, seed=7)["vehicles"]  # every spawn point
        ow.wait_for_tick(10)
        cars = [a.get_location() for a in ow.get_actors().filter("vehicle.*") if a.attributes.get("role_name") == "autopilot"]
        taken = next(i for i, p in enumerate(ow.get_map().get_spawn_points())
                     if any(p.location.distance(l) < 2.0 for l in cars))
        blocked = json.loads(json.dumps(base))
        blocked["carla"]["spawn_index"] = taken
        c.events.clear()
        try:
            c.call("cosim_start", config=blocked)
            started = True
        except RuntimeError as e:
            started = str(e)[-80:]
        c.call("cosim_stop")
        moved = any(e.get("event") == "log" and "挪到别处" in e.get("msg", "") for e in c.events)
        check("a traffic car on the ego's spawn point is moved away and the run starts",
              started is True and moved, (n, started))
        c.call("clear_traffic")
        c.call("views_set", views=[{"id": "p0", "kind": "rgb", "mode": "chase", "width": 160, "height": 90, "fps": 5}])

        # ---- a recording replays without duplicates and leaves nothing behind --
        rec = os.path.join(tmp, "rec.log")
        c.call("spawn_traffic", vehicles=6, walkers=3, seed=8)
        c.call("start_recorder", filename=rec)
        short = json.loads(json.dumps(base))
        short["sync"]["duration"] = 3.0
        c.events.clear()
        c.call("cosim_start", config=short)
        run_until(c, ("finished", "error"))
        c.call("stop_recorder")
        c.call("replay", filename=rec)
        time.sleep(2.0)
        ow.wait_for_tick(10)
        heroes_now = heroes(ow)
        cars = [a for a in ow.get_actors().filter("vehicle.*") if a.attributes.get("role_name") == "autopilot"]
        followed = c.call("world_info")["ego_id"] in [a.id for a in heroes_now]
        c.call("spawn_ego", blueprint="vehicle.tesla.model3", spawn_index=3)
        ow.wait_for_tick(10)
        left = [a for a in ow.get_actors() if a.type_id.startswith(("vehicle.", "walker.", "controller."))
                and a.attributes.get("role_name") != "hero"]
        check("replay: one copy of the recording, the view follows its ego, nothing left after",
              len(heroes_now) == 1 and len(cars) == 6 and followed and not left and len(heroes(ow)) == 1,
              "heroes %d, cars %d, followed %s, left %d" % (len(heroes_now), len(cars), followed, len(left)))
        c.call("views_set", views=[{"id": "p0", "kind": "rgb", "mode": "chase", "width": 160, "height": 90, "fps": 5}])

        # ---- reconnect cleans up what this backend put into the world ----------
        n_traffic = c.call("spawn_traffic", vehicles=5, walkers=0, seed=3)["vehicles"]
        n_before = len(w.get_actors().filter("vehicle.*"))
        c.events.clear()
        c.call("connect", host="localhost", port=carla_port)
        time.sleep(1.0)
        n_after = len(w.get_actors().filter("vehicle.*"))
        c.call("ping")
        va = [e["ids"] for e in c.events if e.get("event") == "views_active"]
        check("reconnect removes our ego, traffic and views", heroes(w) == [] and n_after == n_before - n_traffic - 1 and va and va[-1] == [],
              "vehicles %d -> %d, heroes %d, views %s" % (n_before, n_after, len(heroes(w)), va[-1:]))

        # ---- NaN in outgoing messages ------------------------------------------
        import backend_server as bs
        safe = bs._json_safe({"a": float("nan"), "b": [1.0, float("inf")], "c": {"d": float("-inf")}})
        check("non-finite numbers are sent as null", json.dumps(safe, allow_nan=False) == '{"a": null, "b": [1.0, null], "c": {"d": null}}')
        c.call("destroy_ego")
    finally:
        proc.terminate()
        proc.wait()
        shutil.rmtree(tmp, ignore_errors=True)
    print("ALL ROBUSTNESS TESTS PASSED")


if __name__ == "__main__":
    main()
