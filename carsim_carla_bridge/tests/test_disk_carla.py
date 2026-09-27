"""Disk safety against a running CARLA server: the CARLA recorder is tracked
(world_info "recording") and stopped when the backend reconnects, recovers
after a crash, starts a replay or exits; the run record writes numbers with
8 significant digits; the legacy raw-sensor commands are gone. Writes a few
small files (recordings of a few seconds, a 5-frame run record) into a temp
dir that is deleted at the end.

    python tests/test_disk_carla.py [--port 2000]
"""
import csv
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from test_backend import Conn, check  # noqa: E402

PORT = 57187


def grows(path, timeout=20.0):
    """Does the file get bigger within timeout (CARLA writes it while recording)?"""
    s0 = os.path.getsize(path) if os.path.exists(path) else 0
    end = time.time() + timeout
    while time.time() < end:
        time.sleep(0.5)
        if os.path.exists(path) and os.path.getsize(path) > s0:
            return True
    return False


def still(path, wait=3.0):
    """Unchanged for `wait` seconds: the recorder is stopped (its file closed)."""
    s0 = os.path.getsize(path)
    time.sleep(wait)
    return os.path.getsize(path) == s0


def sig_digits(cell):
    """Significant digits of a float written in the CSV (None: not a float)."""
    try:
        v = float(cell)
    except ValueError:
        return None
    if ("." not in cell and "e" not in cell.lower()) or not math.isfinite(v):
        return None  # an int (id, frame), nan or inf
    return len(cell.lower().split("e")[0].lstrip("+-").replace(".", "").strip("0"))


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    import carla
    cl = carla.Client("localhost", carla_port)
    cl.set_timeout(60)  # the modified CARLA (editor build) answers slowly right after start
    tmp = tempfile.mkdtemp(prefix="cc_disk_")
    proc = subprocess.Popen([sys.executable, os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT)],
                            cwd=os.path.join(HERE, ".."))
    try:
        time.sleep(2)
        c = Conn(PORT)
        c.call("connect", host="localhost", port=carla_port)
        c.call("spawn_ego", blueprint="vehicle.tesla.model3", spawn_index=3)

        # ---- the raw-sensor commands (own recording path, CARLA frame) are gone
        for cmd in ("add_sensor", "list_sensors", "view_start"):
            try:
                c.call(cmd)
                ok = False
            except RuntimeError as e:
                ok = "未知命令" in str(e)
            check("legacy command %s removed" % cmd, ok)

        # ---- a file CARLA cannot create: clear error, not "recording" ----------
        try:
            c.call("start_recorder", filename=os.path.join(tmp, "no_such_dir", "x.log"))
            ok, msg = False, "no error"
        except RuntimeError as e:
            ok, msg = "无法创建" in str(e), str(e)[-60:]
        check("recording file CARLA cannot create: error, not recording", ok and c.call("world_info")["recording"] == "", msg)

        # ---- reconnecting stops our recording ----------------------------------
        rec1 = os.path.join(tmp, "rec1.log")
        path = c.call("start_recorder", filename=rec1)
        check("world_info reports the recording", c.call("world_info")["recording"] == path, path)
        check("the recording writes its file", grows(rec1))
        c.call("connect", host="localhost", port=carla_port)
        check("reconnect stops the CARLA recorder", c.call("world_info")["recording"] == "" and still(rec1))

        # ---- a recording left by a backend that died: recover stops it ---------
        c.call("spawn_ego", blueprint="vehicle.tesla.model3", spawn_index=3)
        rec2 = os.path.join(tmp, "rec2.log")
        cl.start_recorder(rec2, True)  # what a backend killed while recording leaves running
        check("left-over recording writes its file", grows(rec2))
        c.call("connect", host="localhost", port=carla_port, recover=True)
        check("recover connect stops a left-over recording", still(rec2))

        # ---- run record: 5 frames, numbers with at most 8 significant digits ----
        cfg = c.call("default_config")
        cfg["carsim"]["mock"] = True
        cfg["carla"]["spawn_index"] = 3
        cfg["sync"]["frame_dt"] = 0.02
        cfg["sync"]["duration"] = 0.1
        cfg["run"]["log_path"] = os.path.join(tmp, "run", "log.csv")
        c.events.clear()
        c.call("cosim_start", config=cfg)
        st = c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in ("finished", "error", "stopped"), 60)
        check("5-frame run finished", st["state"] == "finished", (st["state"], st.get("detail")))
        with open(cfg["run"]["log_path"], newline="", encoding="utf-8") as f:
            rows = list(csv.reader(f))
        digits = [sig_digits(v) for r in rows[1:] for v in r[2:]]
        digits = [d for d in digits if d is not None]
        check("run record numbers have at most 8 significant digits", len(rows) > 1 and digits and max(digits) <= 8,
              (len(rows), max(digits) if digits else None))

        # ---- a replay ends a recording in progress -----------------------------
        rec3 = os.path.join(tmp, "rec3.log")
        c.call("start_recorder", filename=rec3)
        time.sleep(2)
        c.call("stop_recorder")
        rec4 = os.path.join(tmp, "rec4.log")
        c.call("start_recorder", filename=rec4)
        c.call("replay", filename=rec3)
        check("replay ends the recording, world_info says so", c.call("world_info")["recording"] == "" and still(rec4))

        # ---- the backend exiting stops our recording ---------------------------
        c.call("spawn_ego", blueprint="vehicle.tesla.model3", spawn_index=3)  # also clears the replay
        rec5 = os.path.join(tmp, "rec5.log")
        c.call("start_recorder", filename=rec5)
        check("recording before exit writes its file", grows(rec5))
        c.call("shutdown")
        proc.wait(20)
        check("backend exit stops the CARLA recorder", still(rec5))
        print("ALL DISK TESTS PASSED")
    finally:
        try:
            cl.stop_recorder()  # never leave CARLA writing a file
        except RuntimeError:
            pass
        if proc.poll() is None:
            proc.terminate()
            proc.wait(10)
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    main()
