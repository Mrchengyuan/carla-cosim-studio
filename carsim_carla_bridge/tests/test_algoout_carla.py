"""The control algorithm's own output, end to end through the backend on
CARLA (mock CarSim): what it prints while loading and in control() arrives
as log events of level "algo" (a burst is cut to 20 lines a second, then
'（省略 N 行）'; backend.log still has every line), an error shows the
traceback of the algorithm's own files before the error line (also for a
start that fails), the telemetry has how long control() took (ctrl_ms,
ctrl_ms_max) and the end of the run says so; runs without the algorithm
have neither.

    python tests/test_algoout_carla.py [--port 2000]
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
from test_backend import Conn, check  # noqa: E402
from test_robustness import expect_error, run_until  # noqa: E402

PORT = 57183


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    tmp = tempfile.mkdtemp(prefix="cc_algoout_")
    log_path = os.path.join(tmp, "backend.log")
    log = open(log_path, "w")
    # Like the GUI: the backend's stdout / stderr go to a log file.
    proc = subprocess.Popen([sys.executable, "-u", os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT)],
                            cwd=os.path.join(HERE, ".."), stdout=log, stderr=subprocess.STDOUT)
    try:
        time.sleep(2)
        c = Conn(PORT)
        c.call("connect", host="localhost", port=carla_port)
        base = c.call("default_config")
        base["carsim"]["mock"] = True
        base["run"]["log_path"] = ""
        base["carla"]["spawn_index"] = 3
        base["sync"]["frame_dt"] = 0.05
        base["sync"]["duration"] = 3.0
        c.call("spawn_ego", blueprint="vehicle.tesla.model3", spawn_index=3)

        def write(rel, body):
            p = os.path.join(tmp, rel)
            os.makedirs(os.path.dirname(p), exist_ok=True)
            with open(p, "w", encoding="utf-8") as f:
                f.write(body)
            return p

        def ctrl(rel, body, entry="control"):
            cfg = json.loads(json.dumps(base))
            cfg["run"]["controller"] = {"path": write(rel, body), "entry": entry}
            return cfg

        def run(cfg):
            c.events.clear()
            c.call("cosim_start", config=cfg)
            return run_until(c, ("error", "finished", "stopped"), 90)

        def logs():
            return [(e["level"], e["msg"]) for e in c.events if e.get("event") == "log"]

        # ---- prints while loading and in control(); the time control() takes ----
        st = run(ctrl("p/c.py", "import time\nprint('loaded 算法')\nN = [0]\n\n\ndef control(e, t, dt):\n"
                                "    N[0] += 1\n    if N[0] == 1:\n        for i in range(30):\n"
                                "            print('burst', i)\n    print('call', N[0])\n    time.sleep(0.02)\n"
                                "    return [0.0, 0.0, 0.0]\n"))
        check("printing run finishes", st["state"] == "finished", st)
        algo = [m for lv, m in logs() if lv == "algo"]
        check("what it printed while loading comes first", algo[:1] == ["loaded 算法"], algo[:3])
        check("a burst is cut to 20 lines a second", "burst 0" in algo and "burst 29" not in algo, algo[:25])
        check("... and says how many lines were left out",
              any(m.startswith("（省略 ") and m.endswith(" 行，全部输出见 backend.log）") for m in algo), algo)
        check("lines of later seconds get through again",
              any(m.startswith("call ") and int(m.split()[1]) > 25 for m in algo), algo[-5:])
        log.flush()
        with open(log_path, encoding="utf-8", errors="replace") as f:
            text = f.read()
        check("backend.log has every line", "burst 29" in text and "call 60\n" in text and "loaded 算法" in text)
        tel = [e["data"] for e in c.events if e.get("event") == "telemetry"]
        check("telemetry: how long control() took, and the longest so far",
              tel and all(t["ctrl_ms"] >= 19.0 and t["ctrl_ms_max"] >= t["ctrl_ms"] for t in tel), tel[-1:])
        summary = [m for lv, m in logs() if m.startswith("算法耗时：")]
        check("the end of the run says how long control() took",
              len(summary) == 1 and summary[0].startswith("算法耗时：control() 调用 60 次，平均 ")
              and summary[0].endswith("；仿真步长 50 ms"), summary)

        # ---- an error in a helper: what it printed, then its own traceback, then the error
        write("e/mpc.py", "def solve(x):\n    return 1.0 / x\n")
        st = run(ctrl("e/my_ctrl.py", "import mpc\n\n\ndef control(e, t, dt):\n    print('about to solve')\n"
                                      "    return [mpc.solve(0.0), 0.0, 0.0]\n"))
        lg = logs()
        err = next((i for i, (lv, m) in enumerate(lg) if lv == "error" and m.startswith("仿真出错")), None)
        trace = next((i for i, (lv, m) in enumerate(lg) if lv == "algo" and m.startswith("出错位置")), None)
        check("run error: stopped with the error", st["state"] == "error" and err is not None, lg[-4:])
        check("run error: the traceback of the own files comes before the error line",
              trace is not None and trace < err and ("algo", "about to solve") in lg[:trace] and
              lg[trace][1].split("\n")[1:] == ["  my_ctrl.py 第 6 行 control：return [mpc.solve(0.0), 0.0, 0.0]",
                                               "  mpc.py 第 2 行 solve：return 1.0 / x",
                                               "ZeroDivisionError: float division by zero"], lg[-4:])

        # ---- a start that fails: what it printed and where, before the state -----
        c.events.clear()
        ok, msg = expect_error(c, "cosim_start", "NameError",
                               config=ctrl("bad/c.py", "print('importing')\nx = undefined_name\n"))
        states = [i for i, e in enumerate(c.events) if e.get("event") == "cosim_state"]
        algo = [(i, e["msg"]) for i, e in enumerate(c.events) if e.get("event") == "log" and e.get("level") == "algo"]
        check("failed start: what it printed and its traceback, before the error state",
              ok and states and [m for _, m in algo][:1] == ["importing"] and len(algo) == 2 and
              "  c.py 第 2 行 模块顶层：x = undefined_name" in algo[1][1] and algo[-1][0] < states[-1], (msg, algo))

        # ---- runs without the algorithm: no control() time ----------------------
        cfg = json.loads(json.dumps(base))
        cfg["run"]["driver"] = "demo"
        cfg["sync"]["duration"] = 0.5
        st = run(cfg)
        tel = [e["data"] for e in c.events if e.get("event") == "telemetry"]
        check("demo driver: no control() time", st["state"] == "finished" and tel and
              not any("ctrl_ms" in t for t in tel) and not any(m.startswith("算法耗时") for _, m in logs()), st)
        c.call("destroy_ego")
    finally:
        proc.terminate()
        proc.wait()
        log.close()
        shutil.rmtree(tmp, ignore_errors=True)
    print("ALL ALGOOUT TESTS PASSED")


if __name__ == "__main__":
    main()
