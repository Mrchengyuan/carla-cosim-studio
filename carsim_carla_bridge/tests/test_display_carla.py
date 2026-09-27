"""What the GUI shows, on a real CARLA server (mock CarSim, through the backend):

1. The pose of the 车辆状态 tab (telemetry "scene" -> "ego") is CarSim's
   global frame: X / Y / Yaw / Pitch / Roll equal the exports Xo / Yo / Yaw /
   Pitch / Roll the algorithm got at the same time t.
2. Steering: an algorithm steering left (Steer_SW > 0) turns the car left
   (Yaw grows, Steer_L1 > 0) while the telemetry's wheel_steer (CARLA's sign)
   is < 0: the GUI shows -wheel_steer, the sign of Steer_L1.
3. An IMU handed to the algorithm: on stock CARLA (compatibility mode) the
   start logs the gyro warning once; on the modified CARLA it does not.

    python tests/test_display_carla.py [--port 2000]
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
from test_backend import Conn, check  # noqa: E402

import session  # noqa: E402

PORT = 57172

CONTROLLER = '''
import json, os
class Controller:
    def control(self, exports, t, dt, scene):
        with open(os.environ["DISPLAY_TEST_LOG"], "a") as f:
            f.write(json.dumps(dict({k: exports[k] for k in ("Xo", "Yo", "Yaw", "Pitch", "Roll", "Steer_L1")}, t=t,
                                    imu="imu" in scene.get("sensors", {}))) + "\\n")
        return [0.5, 0.0, 90.0 if t > 1.0 else 0.0]   # [throttle, brake, Steer_SW deg, + = left]
'''

IMU = {"name": "imu", "type": "imu", "x": 0, "y": 0, "z": 0.8, "roll": 0, "pitch": 0, "yaw": 0,
       "attributes": {}, "enabled": True}


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    tmp = tempfile.mkdtemp(prefix="cc_display_test_")
    log = os.path.join(tmp, "control.jsonl")
    os.environ["DISPLAY_TEST_LOG"] = log
    proc = subprocess.Popen([sys.executable, os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT)],
                            cwd=os.path.join(HERE, ".."), env=dict(os.environ))
    try:
        c = Conn(PORT)
        c.call("connect", host="localhost", port=carla_port, timeout=60)
        with open(os.path.join(tmp, "ctrl.py"), "w") as f:
            f.write(CONTROLLER)
        cfg = c.call("default_config")
        cfg["carsim"]["mock"] = True
        cfg["carla"]["spawn_index"] = 3
        cfg["run"]["driver"] = "custom"
        cfg["run"]["controller"] = {"path": os.path.join(tmp, "ctrl.py"), "entry": "Controller"}
        cfg["run"]["log_path"] = ""
        cfg["sync"]["frame_dt"] = 0.02
        cfg["sync"]["duration"] = 4.0
        cfg["scene"].update({"sensors": ["imu"], "collision": "off"})
        cfg["rig"]["sensors"] = [dict(IMU)]
        cfg["collect"]["enabled"] = False
        c.events.clear()
        info = c.call("cosim_start", config=cfg, timeout=120)
        st = c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in ("finished", "error", "stopped"), 120)
        check("run finished", st["state"] == "finished", st.get("detail"))
        tel = [e["data"] for e in c.events if e.get("event") == "telemetry"]
        calls = {round(r["t"], 6): r for r in (json.loads(line) for line in open(log))}

        # ---- 1: the GUI pose = CarSim's exports at the same t -------------------
        pairs = [(t["scene"]["ego"], calls[round(t["t"], 6)]) for t in tel
                 if t.get("scene") and round(t["t"], 6) in calls]
        worst = {k: max((abs(eg[k] - ex[e]) for eg, ex in pairs), default=None)
                 for k, e in (("X", "Xo"), ("Y", "Yo"), ("Yaw", "Yaw"), ("Pitch", "Pitch"), ("Roll", "Roll"))}
        check("GUI pose matches Xo / Yo / Yaw / Pitch / Roll", len(pairs) > 50 and worst["X"] < 0.1 and worst["Y"] < 0.1
              and worst["Yaw"] < 0.5 and worst["Pitch"] < 0.3 and worst["Roll"] < 0.3, "%d pairs, %s" % (len(pairs), worst))
        check("the car moved and rolled", pairs and abs(pairs[-1][1]["Xo"]) + abs(pairs[-1][1]["Yo"]) > 5.0
              and max(abs(ex["Roll"]) for _, ex in pairs) > 0.1, pairs[-1][1] if pairs else None)

        # ---- 2: steering signs ------------------------------------------------
        last = tel[-1]
        ex = calls.get(round(last["t"], 6)) or list(calls.values())[-1]
        check("left turn: Yaw grows, Steer_L1 > 0, telemetry wheel_steer < 0 (GUI shows -wheel_steer)",
              ex["Yaw"] > 5.0 and ex["Steer_L1"] > 1.0 and last["wheel_steer"][0] < -1.0
              and abs(-last["wheel_steer"][0] - ex["Steer_L1"]) < 1.0,
              "Yaw %.1f, Steer_L1 %.2f, wheel_steer %.2f" % (ex["Yaw"], ex["Steer_L1"], last["wheel_steer"][0]))

        # ---- 3: IMU warning on stock CARLA only --------------------------------
        warns = [e["msg"] for e in c.events if e.get("event") == "log" and e.get("level") == "warn"
                 and e.get("msg") == session.IMU_STOCK_WARNING]
        imu_given = any(r["imu"] for r in calls.values())
        if info.get("external_api"):
            check("modified CARLA: no IMU warning", imu_given and not warns, warns)
        else:
            check("stock CARLA: the IMU warning once at the start", imu_given and len(warns) == 1, warns)
        c.call("destroy_ego")
        print("ALL DISPLAY TESTS PASSED")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
        if proc.poll() is None:
            proc.terminate()
            proc.wait(10)


if __name__ == "__main__":
    main()
