"""Sensor extrinsics against a running CARLA server.

With the world in synchronous mode and nothing ticking it (idle tick off),
vehicle_specs spawns and reads its probe cars without a single tick: the
front axle must still come out in the vehicle frame, equal to the one read
from a spawned ego, and a rig preset built from it must sit on the car.
World settings are restored afterwards; no data is collected.

    python tests/test_extrinsics_carla.py [--port 2000]
"""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
from test_backend import Conn, check  # noqa: E402

import carla  # noqa: E402
from bridge import front_axle_local  # noqa: E402

PORT = 57199
IDS = ["vehicle.tesla.model3", "vehicle.audi.tt", "vehicle.lincoln.mkz_2020"]


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    proc = subprocess.Popen([sys.executable, os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT)],
                            cwd=os.path.join(HERE, ".."))
    client = carla.Client("localhost", carla_port)
    client.set_timeout(30.0)
    c = before = None
    try:
        c = Conn(PORT)
        c.call("connect", host="localhost", port=carla_port, timeout=60)
        before = c.call("world_info")
        c.call("world_settings", synchronous=True, frame_dt=0.05, idle_tick=False)

        # ---- vehicle_specs in synchronous mode (probe never ticked)
        specs = c.call("vehicle_specs", ids=IDS, timeout=300)
        for vid in IDS:
            s = specs.get(vid) or {}
            check("%s: front axle in the vehicle frame" % vid,
                  0.3 < s.get("front_axle_x_m", -99) < s.get("length_m", 0) / 2.0, s)
            check("%s: wheelbase" % vid, 2.0 < s.get("wheelbase_m", 0) < 3.6, s.get("wheelbase_m"))

        # ---- a preset built from that measurement sits on the car
        spec = specs[IDS[1]]
        rig = c.call("rig_build", preset="nuscenes", blueprint=IDS[1])
        L, fx = spec["length_m"], spec["front_axle_x_m"]
        xs = [s["x"] for s in rig]
        # Front axle origin: rear bumper about -(L/2 + fx), front bumper L/2 - fx.
        check("nuscenes preset on the car (CarSim frame, front axle origin)",
              all(-L < x < L / 2.0 for x in xs), (min(xs), max(xs), fx))

        # ---- same front axle as read from a spawned (ticked) ego
        ego = c.call("spawn_ego", blueprint=IDS[0], spawn_index=0)
        w = client.get_world()
        actor = None
        end = time.time() + 10.0
        while actor is None and time.time() < end:
            # This client sees the ego with the snapshot of the tick spawn_ego made.
            if w.get_snapshot().find(ego["id"]) is not None:
                actor = w.get_actor(ego["id"])
            else:
                time.sleep(0.1)
        check("ego visible to a second client", actor is not None, ego)
        ref = front_axle_local(actor)
        check("vehicle_specs front axle = spawned ego's", abs(ref[0] - specs[IDS[0]]["front_axle_x_m"]) < 0.05,
              (round(float(ref[0]), 3), specs[IDS[0]]["front_axle_x_m"]))
        c.call("destroy_ego")
        print("ALL EXTRINSICS TESTS PASSED")
    finally:
        if c is not None and before is not None:
            try:
                c.call("world_settings", synchronous=before["synchronous"], frame_dt=before["frame_dt"] or 0.0,
                       idle_tick=before["idle_tick"])
            except Exception as e:  # noqa: BLE001 - report, then stop the backend anyway
                print("could not restore world settings:", e)
        proc.terminate()
        proc.wait(10)


if __name__ == "__main__":
    main()
