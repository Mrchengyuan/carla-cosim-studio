"""Remote mode on CARLA (carsim.remote): the same co-simulation through a
carsim_service.py --mock on this machine gives what the mock inside the
backend gives, bit for bit: the exports the algorithm gets, the scene's ego
Yaw (= the exports' Yaw), the final Xo / Yo. Also: the .sim / python_carsim_env
paths (on the Windows computer) are not checked on the server, a service
that goes away mid-run ends the run with an error and leaves nothing in
CARLA, and without a service the run is refused before the world changes.

Starts its own backend (port 57141, CarSim service port 57142) and service;
collects nothing, deletes its temp files.

    python tests/test_remote_carla.py [--port 2000]
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

import carla  # noqa: E402

PORT = 57141
CARSIM_PORT = 57142
NO_SERVICE = ("Windows 上的 CarSim 服务没有连上云端：请在 Windows 上双击启动脚本（启动远程仿真.bat），"
              "看到“已连上云端”后再点运行")
LOST = "与 Windows 上的 CarSim 服务的连接断开了（"

# Records what the algorithm gets; accelerates, then turns left.
RECORDER = '''
import json
def control(exports, t, dt, scene):
    e = scene["ego"]
    with open(LOG, "a") as f:
        f.write(json.dumps({"t": t, "ex": exports, "Yaw": e["Yaw"], "X": e["X"], "Y": e["Y"]}) + "\\n")
    return [0.4, 0.0, 120.0 if t > 1.0 else 0.0]
'''
CAM = {"name": "cam", "type": "rgb", "x": 1.5, "y": 0, "z": 1.6, "roll": 0, "pitch": 0, "yaw": 0,
       "attributes": {"image_size_x": 160, "image_size_y": 90, "fov": 90}, "enabled": True}


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    tmp = tempfile.mkdtemp(prefix="cc_remote_test_")
    proc = subprocess.Popen([sys.executable, os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT),
                             "--carsim-port", str(CARSIM_PORT)], cwd=os.path.join(HERE, ".."), env=dict(os.environ))
    service = service_log = None
    client = carla.Client("localhost", carla_port)
    client.set_timeout(30.0)
    try:
        c = Conn(PORT)
        c.call("connect", host="localhost", port=carla_port, timeout=60)
        w = client.get_world()
        check("no service yet", c.call("world_info")["carsim_service"] == {"connected": False, "host": "", "platform": ""})
        service_log = open(os.path.join(tmp, "service.txt"), "w")
        service = subprocess.Popen([sys.executable, "-u", os.path.join(HERE, "..", "carsim_service.py"),
                                    "--port", str(CARSIM_PORT), "--mock"], stdout=service_log, stderr=subprocess.STDOUT)
        end = time.time() + 20
        while not c.call("world_info")["carsim_service"]["connected"] and time.time() < end:
            time.sleep(0.2)
        svc = c.call("world_info")["carsim_service"]
        check("the service connects", svc == {"connected": True, "host": socket.gethostname(), "platform": sys.platform}, svc)

        cfg = c.call("default_config")
        check("carsim.remote in the default config", cfg["carsim"].get("remote") is False, cfg["carsim"])
        cfg["carla"]["spawn_index"] = 3
        cfg["run"].update(driver="custom", log_path="")
        cfg["sync"].update(frame_dt=0.02, duration=5.0)
        cfg["scene"].update(collision="off", ego=["X", "Y", "Yaw"])
        cfg["collect"]["enabled"] = False

        def run(remote, **extra):
            log = os.path.join(tmp, "remote.jsonl" if remote else "local.jsonl")
            ctrl = os.path.join(tmp, "ctrl_%s.py" % ("remote" if remote else "local"))
            with open(ctrl, "w") as f:
                f.write(RECORDER.replace("LOG", repr(log)))
            r = json.loads(json.dumps(cfg))
            r["run"]["controller"] = {"path": ctrl, "entry": "control"}
            # Remote: paths on the Windows computer, which the server does not have.
            r["carsim"].update(mock=not remote, remote=remote, sim_path="C:\\CarSim\\simfile.sim" if remote else "",
                               repo_path="D:\\python_carsim_env" if remote else r["carsim"]["repo_path"])
            for k, v in extra.items():
                r[k].update(v)
            c.events.clear()
            c.call("cosim_start", config=r, timeout=120)
            return log

        def ended(timeout=150):
            return c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in ("finished", "error", "stopped"),
                                timeout)

        def records(log):
            return [json.loads(line) for line in open(log)]

        # ---- 1: the same run with the mock here and through the service ----------
        run(False)
        st_local = ended()
        local = records(os.path.join(tmp, "local.jsonl"))
        run(True)
        st_remote = ended()
        remote = records(os.path.join(tmp, "remote.jsonl"))
        started = [e["msg"] for e in c.events if e.get("event") == "log" and e["msg"].startswith("联合仿真开始")]
        check("both runs finished", st_local["state"] == st_remote["state"] == "finished"
              and "时长" in st_remote.get("detail", ""), (st_local.get("detail"), st_remote.get("detail")))
        check("the backend ran CarSim through the service (not its own mock)",
              started and "模拟 CarSim" not in started[0], started)
        same_t = len(local) == len(remote) > 200 and all(a["t"] == b["t"] for a, b in zip(local, remote))
        check("same steps", same_t, "%d / %d calls" % (len(local), len(remote)))
        diff = max(abs(a["ex"][k] - b["ex"][k]) for a, b in zip(local, remote) for k in a["ex"])
        check("the algorithm got the same exports (CarSim frames and units)",
              diff == 0.0 and all(sorted(a["ex"]) == sorted(b["ex"]) for a, b in zip(local, remote)), "max diff %g" % diff)
        yaw = max(abs(x["Yaw"] - x["ex"]["Yaw"]) for x in remote)
        check("scene ego Yaw == exports Yaw", yaw < 0.5 and abs(remote[-1]["ex"]["Yaw"]) > 5.0,
              "max |ego Yaw - Yaw| %.3f, final Yaw %.1f" % (yaw, remote[-1]["ex"]["Yaw"]))
        a, b = local[-1], remote[-1]
        check("same final Xo / Yo", (a["ex"]["Xo"], a["ex"]["Yo"]) == (b["ex"]["Xo"], b["ex"]["Yo"])
              and abs(b["X"] - b["ex"]["Xo"]) < 0.1 and abs(b["Y"] - b["ex"]["Yo"]) < 0.1,
              "Xo %.3f / %.3f, Yo %.3f / %.3f, ego X %.3f Y %.3f" % (a["ex"]["Xo"], b["ex"]["Xo"], a["ex"]["Yo"],
                                                                     b["ex"]["Yo"], b["X"], b["Y"]))
        scene = max(max(abs(p["X"] - q["X"]), abs(p["Y"] - q["Y"]), abs(p["Yaw"] - q["Yaw"])) for p, q in zip(local, remote))
        check("the scene is the same too", scene < 0.01, "max diff %.4f" % scene)

        # ---- 2: the service goes away mid-run ------------------------------------
        before = len(w.get_actors().filter("sensor.*"))
        sync_before = w.get_settings().synchronous_mode
        run(True, sync={"duration": 0.0}, scene={"sensors": ["cam"]}, rig={"sensors": [dict(CAM)]})
        c.wait_event(lambda e: e.get("event") == "telemetry" and e["data"]["t"] > 1.0, 60)
        during = len(w.get_actors().filter("sensor.*"))
        service.kill()
        service.wait(10)
        st = ended(60)
        if not w.get_settings().synchronous_mode:
            w.wait_for_tick(5.0)  # a snapshot after the clean-up
        after = len(w.get_actors().filter("sensor.*"))
        check("the run ends with the error", st["state"] == "error" and st.get("detail", "").startswith(LOST), st.get("detail"))
        check("nothing of the run left in CARLA", during > before and after == before
              and w.get_settings().synchronous_mode == sync_before, "sensors %d / %d / %d" % (before, during, after))
        gone = c.wait_event(lambda e: e.get("event") == "log" and e["msg"] == "Windows 上的 CarSim 服务断开了", 10)
        check("the GUI's output says so", gone["level"] == "warn")
        check("world_info: not connected", not c.call("world_info")["carsim_service"]["connected"])

        # ---- 3: no service: refused before the world changes ---------------------
        ego = c.call("world_info")["ego_id"]
        try:
            run(True)
            refused = ""
        except RuntimeError as e:
            refused = str(e)
        check("no service: refused in plain words", NO_SERVICE in refused, refused)
        check("the ego was not respawned", c.call("world_info")["ego_id"] == ego and ego != 0, ego)
        c.call("destroy_ego")
        print("ALL REMOTE TESTS PASSED")
    finally:
        if service is not None and service.poll() is None:
            service.kill()
            service.wait(10)
        if service_log is not None:
            service_log.close()
        shutil.rmtree(tmp, ignore_errors=True)
        if proc.poll() is None:
            proc.terminate()
            proc.wait(10)


if __name__ == "__main__":
    main()
