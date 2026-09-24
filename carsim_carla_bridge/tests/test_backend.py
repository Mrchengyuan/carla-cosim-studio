"""End-to-end test of backend_server.py against a running CARLA server.

Starts the backend as a subprocess, then exercises every GUI-facing command
over the same TCP/JSON protocol the C++ GUI uses.

    python tests/test_backend.py [--with-map-switch]
"""
import json
import os
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PORT = 57199


class Conn:
    def __init__(self, port):
        for _ in range(50):
            try:
                self.s = socket.create_connection(("127.0.0.1", port))
                break
            except OSError:
                time.sleep(0.2)
        self.buf = b""
        self.events = []
        self.next_id = 0

    def _read(self, timeout):
        self.s.settimeout(timeout)
        while b"\n" not in self.buf:
            chunk = self.s.recv(65536)
            if not chunk:
                raise ConnectionError("backend closed")
            self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        return json.loads(line)

    def call(self, cmd, timeout=200, **args):
        self.next_id += 1
        rid = self.next_id
        self.s.sendall((json.dumps({"id": rid, "cmd": cmd, "args": args}) + "\n").encode())
        end = time.time() + timeout
        while True:
            msg = self._read(max(0.1, end - time.time()))
            if msg.get("id") == rid:
                if not msg["ok"]:
                    raise RuntimeError("%s failed: %s" % (cmd, msg["error"]))
                return msg["result"]
            self.events.append(msg)

    def wait_event(self, pred, timeout=60):
        for e in self.events:
            if pred(e):
                return e
        end = time.time() + timeout
        while time.time() < end:
            msg = self._read(max(0.1, end - time.time()))
            self.events.append(msg)
            if pred(msg):
                return msg
        raise TimeoutError("event not received")


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + (("  " + str(detail)) if detail != "" else ""))
    if not cond:
        raise SystemExit(1)


def main():
    proc = subprocess.Popen([sys.executable, os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT)],
                            cwd=os.path.join(HERE, ".."))
    try:
        c = Conn(PORT)
        check("ping", c.call("ping") == "pong")
        info = c.call("connect", host="localhost", port=2000)
        check("connect", info["server_version"].startswith("0.9.16"), info["map"])
        maps = c.call("list_maps")
        check("list_maps", len(maps) >= 8, maps[:5])

        w = c.call("set_weather", preset="WetCloudySunset")
        check("set_weather preset", w["wetness"] > 0 or w["cloudiness"] > 0, w["cloudiness"])
        w = c.call("set_weather", params={"fog_density": 35.0})
        check("set_weather param", abs(w["fog_density"] - 35.0) < 1e-3)
        c.call("set_weather", preset="ClearNoon")

        vehicles = c.call("list_vehicles")
        check("list_vehicles", len(vehicles) > 20, len(vehicles))
        specs = c.call("vehicle_specs", ids=["vehicle.tesla.model3", "vehicle.audi.tt", "vehicle.lincoln.mkz_2020"])
        t3 = specs["vehicle.tesla.model3"]
        check("vehicle_specs tesla", abs(t3["wheel_radius_m"][0] - 0.37) < 0.01 and 2.8 < t3["wheelbase_m"] < 3.1, t3)
        check("vehicle_specs cached", c.call("list_vehicles")[0]["spec"] is not None
              or any(v["spec"] for v in c.call("list_vehicles")))

        pts = c.call("list_spawn_points")
        check("list_spawn_points", len(pts) > 50, len(pts))
        ego = c.call("spawn_ego", blueprint="vehicle.tesla.model3", spawn_index=3)
        check("spawn_ego", ego["id"] > 0, ego)
        check("spectator follow", c.call("spectator", mode="follow") == "follow")

        tr = c.call("spawn_traffic", vehicles=8, walkers=5, seed=1)
        check("spawn_traffic", tr["vehicles"] >= 6 and tr["walkers"] >= 1, tr)

        os.makedirs("/tmp/cc_gen/sens", exist_ok=True)
        cam = c.call("add_sensor", type="rgb", x=-6, z=3, pitch=-15,
                     attributes={"image_size_x": 640, "image_size_y": 360}, save_dir="/tmp/cc_gen/sens/rgb")
        imu = c.call("add_sensor", type="imu", save_dir="/tmp/cc_gen/sens/imu")
        check("add_sensor", cam["id"] > 0 and imu["id"] > 0)

        rec = c.call("start_recorder", filename="/tmp/cc_gen/test_rec.log")
        check("start_recorder", bool(rec))

        cfg = c.call("default_config")
        cfg["carsim"]["mock"] = True
        cfg["sync"]["duration"] = 6.0
        cfg["carla"]["spawn_index"] = 3
        cfg["run"]["log_path"] = "/tmp/cc_gen/gui_cosim.csv"
        start = c.call("cosim_start", config=cfg)
        check("cosim_start", start["inner_steps"] == 20, start)
        tel = c.wait_event(lambda e: e.get("event") == "telemetry" and e["data"]["t"] > 2.0)["data"]
        check("telemetry", tel["speed_kmh"] > 1.0 and len(tel["wheel_steer"]) == 4,
              "t=%.2f v=%.1f km/h rt=%.2fx" % (tel["t"], tel["speed_kmh"], tel["rt_factor"]))
        check("pause", c.call("cosim_pause") == "paused")
        f0 = tel["frame"]
        c.call("cosim_step")
        c.call("cosim_step")
        check("resume", c.call("cosim_resume") == "running")
        done = c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in ("finished", "error"), 120)
        check("cosim finished", done["state"] == "finished", done)
        winfo = c.call("world_info")
        check("world restored to async after cosim", winfo["synchronous"] is False, winfo["synchronous"])

        sens = c.call("list_sensors")
        check("sensors received data", all(s["count"] > 0 for s in sens), [(s["type"], s["count"]) for s in sens])
        check("rgb frames saved", len(os.listdir("/tmp/cc_gen/sens/rgb")) > 0, len(os.listdir("/tmp/cc_gen/sens/rgb")))
        c.call("stop_recorder")
        info_txt = c.call("recorder_info", filename="/tmp/cc_gen/test_rec.log")
        check("recorder file", "Frames" in info_txt or "frames" in info_txt, info_txt[:60].replace("\n", " "))

        actors = c.call("list_actors")
        check("list_actors", any(a["ego"] for a in actors), len(actors))
        c.call("remove_sensor", id=cam["id"])
        check("remove_sensor", len(c.call("list_sensors")) == 1)
        c.call("clear_traffic")
        after = c.call("list_actors", filter="vehicle.*")
        check("clear_traffic", len(after) == 1, len(after))

        if "--with-map-switch" in sys.argv:
            t0 = time.time()
            wi = c.call("load_map", name="Town03", timeout=300)
            check("load_map Town03", wi["map"] == "Town03", "%.0fs" % (time.time() - t0))
            wi = c.call("load_map", name="Town10HD_Opt", timeout=300)
            check("load_map back", wi["map"] == "Town10HD_Opt")
        c.call("destroy_ego")
        check("destroy_ego", c.call("world_info")["ego_id"] == 0)
        print("ALL BACKEND TESTS PASSED")
    finally:
        proc.terminate()
        proc.wait(10)


if __name__ == "__main__":
    main()
