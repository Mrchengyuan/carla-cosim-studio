"""运行输出 (output.txt) with a CARLA server: a mock-CarSim run with an algorithm that prints
writes the run's 输出 into its record folder: the notes from before the folder existed (the
start), the algorithm's print lines tagged [算法], its traceback tagged, the key figures and
the end; run_output gives it to the GUI; a run without a record folder writes nothing; an
older run folder without output.txt is said in plain words.

    python tests/test_run_output_carla.py [--port 2000]
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
from test_backend import Conn  # noqa: E402

PORT = 57153
FAILS = []
CTRL = '''
class Controller:
    def reset(self):
        self.n = 0
        print("算法启动了")

    def control(self, exports, t, dt):
        self.n += 1
        if self.n % 20 == 0:
            print("第 %d 帧 t = %.2f" % (self.n, t))
        if self.n == 50:
            raise RuntimeError("故意出错")
        return [0.2, 0.0, 0.0]
'''


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + (("  " + str(detail)) if detail != "" else ""), flush=True)
    if not cond:
        FAILS.append(name)


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    tmp = tempfile.mkdtemp(prefix="cc_output_test_")
    ctrl = os.path.join(tmp, "printing_controller.py")
    open(ctrl, "w", encoding="utf-8").write(CTRL)
    proc = subprocess.Popen([sys.executable, os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT)],
                            cwd=os.path.join(HERE, ".."))
    try:
        c = Conn(PORT)
        c.call("connect", host="localhost", port=carla_port, timeout=60)
        if c.call("world_info")["map"] != "Town10HD_Opt":
            c.call("load_map", name="Town10HD_Opt", timeout=400)
        cfg = c.call("default_config")
        cfg["carla"]["spawn_index"] = 3
        cfg["carsim"].update(mock=True)
        cfg["run"].update(driver="custom", controller={"path": ctrl, "entry": "Controller"}, log_path=os.path.join(tmp, "runs"))
        cfg["sync"].update(frame_dt=0.05, duration=10.0)
        c.events.clear()
        try:
            info = c.call("cosim_start", config=cfg, timeout=300)
        except Exception as e:  # noqa: BLE001
            info = {"error": str(e)}
        c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in ("finished", "error", "stopped"), 300)
        folder = info.get("record_dir") or ""
        path = os.path.join(folder, "output.txt")
        text = open(path, encoding="utf-8").read() if os.path.isfile(path) else ""
        lines = text.splitlines()
        check("output.txt in the run folder", bool(lines), path)
        check("... from the start (the notes before the folder existed)", any("联合仿真开始" in l for l in lines),
              lines[:3])
        check("... the algorithm's print lines, tagged [算法]",
              any("[算法] 算法启动了" in l for l in lines) and any("[算法] 第 40 帧" in l for l in lines))
        check("... its error and traceback", any("故意出错" in l for l in lines) and any("printing_controller.py" in l for l in lines))
        check("... the key figures and the end", any("运行指标" in l for l in lines) and lines[-1].split(" ", 1)[1].startswith("运行结束（出错）"),
              lines[-1] if lines else "")
        check("... every message with a time, a message's further lines indented (a traceback)",
              all((len(l) > 9 and l[2] == ":" and l[5] == ":") or l.startswith("    ") for l in lines)
              and any(l.startswith("    ") for l in lines), [l for l in lines if l.startswith("    ")][:2])
        check("the verdict (通过标准: default) in the output and run.json: not passed, it did not finish",
              any("判定：未通过：没有跑完（出错）" in l for l in lines) and
              json.load(open(os.path.join(folder, "run.json"), encoding="utf-8"))["verdict"]["fails"] == ["没有跑完（出错）"])
        r = c.call("run_output", folder=folder)
        check("run_output: the text for the GUI", r["text"] == text and r["lines"] == len(lines) and not r["truncated"], r["lines"])

        # A second run: its own file, nothing of the first one.
        c.events.clear()
        cfg["sync"]["duration"] = 1.0
        info2 = c.call("cosim_start", config=cfg, timeout=300)
        c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in ("finished", "error", "stopped"), 300)
        t2 = open(os.path.join(info2["record_dir"], "output.txt"), encoding="utf-8").read()
        check("a second run: its own output.txt, nothing of the first", "故意出错" not in t2 and "运行结束（完成）" in t2, t2[-80:])
        check("... finished: 判定：通过", "判定：通过" in t2)
        # Without a record folder: nothing written anywhere.
        cfg["run"]["log_path"] = ""
        before = sorted(os.listdir(os.path.join(tmp, "runs")))
        c.events.clear()
        c.call("cosim_start", config=cfg, timeout=300)
        c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in ("finished", "error", "stopped"), 300)
        check("no record folder: no output.txt anywhere", sorted(os.listdir(os.path.join(tmp, "runs"))) == before
              and not os.path.exists(os.path.join(HERE, "..", "output.txt")))
        # An older run folder (no output.txt).
        os.remove(path)
        try:
            c.call("run_output", folder=folder)
            err = ""
        except Exception as e:  # noqa: BLE001
            err = str(e)
        check("an older run without output.txt: said", "没有保存输出" in err, err)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            proc.kill()
        shutil.rmtree(tmp, ignore_errors=True)
    print("FAILED: %s" % FAILS if FAILS else "ALL RUN OUTPUT TESTS PASSED")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
