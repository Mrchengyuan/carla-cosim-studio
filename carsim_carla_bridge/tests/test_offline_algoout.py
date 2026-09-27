"""The control algorithm's own output in the GUI: what it prints while it is
loaded and in every control() call (also still in backend.log), at most 20
lines a second with a '（省略 N 行）' line, the traceback of its own files
when it raises, and how long each control() call takes (算法耗时). Needs no
CARLA server.

    python tests/test_offline_algoout.py
"""

import io
import logging
import os
import shutil
import sys
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
sys.dont_write_bytecode = True  # files rewritten within the same second must not come from a stale .pyc

import backend_server  # noqa: E402
import session  # noqa: E402
import settings as st  # noqa: E402
from backend_server import Backend  # noqa: E402
from test_offline_carsim import VEHICLE, FakeSync, FakeWorld, cfg  # noqa: E402


class Ex:
    """The bridge's export table: name -> index into the CarSim output."""

    def __init__(self, names):
        self.index = {n: i for i, n in enumerate(names)}

    def raw(self, obs, n):
        return obs[self.index[n]]


class Case(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="cc_algoout_")
        self.sys_path = list(sys.path)
        session._last_folder = None
        self.real = io.StringIO()  # stands for backend.log
        p = mock.patch.multiple(sys, stdout=self.real, stderr=self.real)
        p.start()
        self.addCleanup(p.stop)

    def tearDown(self):
        sys.path[:] = self.sys_path
        for name, m in list(sys.modules.items()):
            f = getattr(m, "__file__", None)
            if name == "user_controller" or (isinstance(f, str) and f.startswith(self.tmp)):
                del sys.modules[name]
        session._last_folder = None
        shutil.rmtree(self.tmp, ignore_errors=True)

    def write(self, rel, body):
        p = os.path.join(self.tmp, rel)
        os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p, "w", encoding="utf-8") as f:
            f.write(body)
        return p

    def load(self, rel, body=None, entry="control", scene=None):
        if body is not None:
            self.write(rel, body)
        d = st.load_dict(None, {})
        d["run"]["controller"] = {"path": os.path.join(self.tmp, rel), "entry": entry}
        d["carsim"]["repo_path"] = self.tmp
        self.out = session.AlgoOutput()
        return session.load_controller(d, Ex(["Xo", "Vx"]), None, scene, self.out)

    def fails(self, rel, body, entry="control"):
        """The error of loading and calling it once; what was taken for the GUI."""
        with self.assertRaises(RuntimeError) as cm:
            self.load(rel, body, entry)([0.0, 36.0], 0.0)
        return str(cm.exception), self.out.take(end=True)


# ------------------------------------------------------------------ output
class OutputTests(Case):
    def test_prints_reach_the_gui_and_still_the_log(self):
        ctl = self.load("a/c.py", "import sys, warnings\nprint('loading 你好')\n\n\nclass Controller:\n"
                                  "    def reset(self):\n        print('reset')\n\n"
                                  "    def control(self, e, t, dt):\n        print('vx', e['Vx'])\n"
                                  "        print('careful', file=sys.stderr)\n"
                                  "        warnings.warn('odd value')\n        return [0.0, 0.0, 0.0]\n", "Controller")
        self.assertEqual(self.out.take(), ["loading 你好", "reset"])
        ctl([0.0, 36.0], 0.0)
        lines = self.out.take()
        self.assertEqual(lines[:2], ["vx 36.0", "careful"])
        self.assertTrue(any("UserWarning: odd value" in ln for ln in lines), lines)
        self.assertIs(sys.stdout, self.real)  # put back after the call
        self.assertIs(sys.stderr, self.real)
        log = self.real.getvalue()
        for text in ("loading 你好", "reset", "vx 36.0", "careful", "odd value"):
            self.assertIn(text, log)  # backend.log keeps everything
        self.assertEqual(self.out.take(), [])

    def test_other_threads_are_not_the_algorithm(self):
        """The backend's other threads (heartbeat, sockets) may print while
        control() runs: that goes to the log only."""
        ctl = self.load("a/c.py", "import threading\nINSIDE, GO = threading.Event(), threading.Event()\n\n\n"
                                  "def control(e, t, dt):\n    print('mine')\n    INSIDE.set()\n    GO.wait(5)\n"
                                  "    return [0.0, 0.0, 0.0]\n")
        th = threading.Thread(target=ctl, args=([0.0, 0.0], 0.0), daemon=True)
        th.start()
        mod = sys.modules["user_controller"]
        self.assertTrue(mod.INSIDE.wait(5))
        print("the backend's heartbeat")  # sys.stdout is the algorithm's capture right now
        mod.GO.set()
        th.join(5)
        self.assertEqual(self.out.take(), ["mine"])
        self.assertIn("the backend's heartbeat", self.real.getvalue())

    def test_at_most_20_lines_a_second_then_how_many_were_left_out(self):
        ctl = self.load("a/c.py", "def control(e, t, dt):\n    for i in range(50):\n        print('line', i)\n"
                                  "    return [0.0, 0.0, 0.0]\n")
        now = [100.0]
        self.out.clock = lambda: now[0]
        ctl([0.0, 0.0], 0.0)
        self.assertEqual(self.out.take(), ["line %d" % i for i in range(20)])
        now[0] += 0.5
        self.assertEqual(self.out.take(), [])  # same second: nothing yet
        now[0] += 0.6
        self.assertEqual(self.out.take(), ["（省略 30 行，全部输出见 backend.log）"])
        ctl([0.0, 0.0], 0.02)
        lines = self.out.take(end=True)  # the run ends: the count comes right away
        self.assertEqual(lines, ["line %d" % i for i in range(20)] + ["（省略 30 行，全部输出见 backend.log）"])
        self.assertEqual(self.out.take(end=True), [])
        self.assertEqual(self.real.getvalue().count("\nline 49\n"), 2)  # all of it in backend.log

    def test_nobody_taking_the_lines_keeps_them_bounded(self):
        """run_cosim.py never takes them."""
        ctl = self.load("a/c.py", "def control(e, t, dt):\n    for i in range(50):\n        print('line', i)\n"
                                  "    return [0.0, 0.0, 0.0]\n")
        now = [0.0]
        self.out.clock = lambda: now[0]
        for _ in range(100):
            ctl([0.0, 0.0], 0.0)
            now[0] += 1.1
        self.assertLessEqual(len(self.out.take(end=True)), session.AlgoOutput.PENDING)

    def test_unfinished_and_very_long_lines(self):
        ctl = self.load("a/c.py", "def control(e, t, dt):\n    print('x =', end=' ')\n    print(1.5, end='')\n"
                                  "    print('\\t' + 'y' * 5000)\n    return [0.0, 0.0, 0.0]\n")
        ctl([0.0, 0.0], 0.0)
        self.assertEqual(self.out.take(), [("x = 1.5 " + "y" * 5000)[:session.AlgoOutput.MAX_LEN] + " …"])
        ctl = self.load("b/c.py", "def control(e, t, dt):\n    print('no newline', end='')\n    return [0.0, 0.0, 0.0]\n")
        ctl([0.0, 0.0], 0.0)
        self.assertEqual(self.out.take(), ["no newline"])  # shown after the call, not held back

    def test_a_logging_handler_made_on_the_first_run_still_shows(self):
        """logging.basicConfig() on the first load keeps the stream it got
        (sys.stderr then); on the next runs it does nothing."""
        root = logging.getLogger()
        kept = list(root.handlers)
        self.addCleanup(lambda: setattr(root, "handlers", kept))
        root.handlers = []
        body = ("import logging\nlogging.basicConfig(level=logging.INFO, format='%(message)s')\n\n\n"
                "def control(e, t, dt):\n    logging.info('logged %s', t)\n    return [0.0, 0.0, 0.0]\n")
        for run in range(2):
            ctl = self.load("l/c.py", body)
            ctl([0.0, 0.0], float(run))
            self.assertEqual(self.out.take(), ["logged %.1f" % run])
        logging.info("the backend's own")  # outside control(): the log only
        self.assertEqual(self.out.take(), [])
        self.assertIn("the backend's own", self.real.getvalue())

    def test_no_capture_without_an_output(self):
        """load_controller as before (run_cosim.py, older callers): prints go to the stream only."""
        self.write("a/c.py", "def control(e, t, dt):\n    print('plain')\n    return [0.0, 0.0, 0.0]\n")
        d = st.load_dict(None, {})
        d["run"]["controller"] = {"path": os.path.join(self.tmp, "a/c.py"), "entry": "control"}
        d["carsim"]["repo_path"] = self.tmp
        session.load_controller(d, Ex(["Xo", "Vx"]))([0.0, 0.0], 0.0)
        self.assertIn("plain", self.real.getvalue())


# --------------------------------------------------------------- traceback
class TracebackTests(Case):
    def test_frames_of_own_files_outermost_first(self):
        self.write("h/mpc.py", "import json\n\n\ndef solve(x):\n    json.dumps(x)\n    return 1.0 / x\n")
        msg, lines = self.fails("h/my_ctrl.py", "import mpc\n\n\ndef control(e, t, dt):\n"
                                                "    print('before')\n    return [mpc.solve(0.0), 0.0, 0.0]\n")
        self.assertIn("（mpc.py 第 6 行，由 my_ctrl.py 第 6 行调用）", msg)  # the one-line error as before
        self.assertEqual(lines[0], "before")
        self.assertEqual(lines[1].split("\n"), [
            "出错位置（只列你的文件，外层在前）：",
            "  my_ctrl.py 第 6 行 control：return [mpc.solve(0.0), 0.0, 0.0]",
            "  mpc.py 第 6 行 solve：return 1.0 / x",
            "ZeroDivisionError: float division by zero"])
        self.assertNotIn("session.py", lines[1])

    def test_library_frames_are_left_out_and_causes_kept(self):
        msg, lines = self.fails("c/c.py", "import json\n\n\ndef control(e, t, dt):\n    try:\n"
                                          "        json.loads('{bad')\n    except ValueError as err:\n"
                                          "        raise RuntimeError('config broken') from err\n")
        trace = lines[-1].split("\n")
        self.assertEqual(trace[1], "  c.py 第 6 行 control：json.loads('{bad')")
        self.assertTrue(trace[2].startswith("JSONDecodeError: "), trace)
        self.assertEqual(trace[3:], ["上面的异常引起了下面的异常：",
                                     "  c.py 第 8 行 control：raise RuntimeError('config broken') from err",
                                     "RuntimeError: config broken"])
        self.assertNotIn("decoder.py", lines[-1])

    def test_errors_while_loading(self):
        self.write("d/pkg/__init__.py", "")
        self.write("d/pkg/tool.py", "X = 1\nY = undefined_name\n")
        with self.assertRaisesRegex(RuntimeError, "NameError"):
            self.load("d/c.py", "print('importing')\nfrom pkg import tool\n")
        lines = self.out.take(end=True)
        self.assertEqual(lines[0], "importing")
        self.assertEqual(lines[1].split("\n")[1:], ["  c.py 第 2 行 模块顶层：from pkg import tool",
                                                    "  %s 第 2 行 模块顶层：Y = undefined_name" % os.path.join("pkg", "tool.py"),
                                                    "NameError: name 'undefined_name' is not defined"])
        with self.assertRaisesRegex(RuntimeError, "SyntaxError"):
            self.load("e/c.py", "def control(e, t, dt):\n    return [0.0, 0.0, 0.0\n")
        trace = self.out.take(end=True)[-1].split("\n")
        self.assertIn("  c.py 第 2 行：return [0.0, 0.0, 0.0", trace)

    def test_deep_recursion_is_shortened(self):
        msg, lines = self.fails("r/c.py", "def f(n):\n    return f(n + 1)\n\n\ndef control(e, t, dt):\n    return f(0)\n")
        trace = lines[-1].split("\n")
        self.assertLess(len(trace), 20)
        self.assertEqual(trace[1], "  c.py 第 6 行 control：return f(0)")
        self.assertTrue(any(ln.startswith("  ……（中间省略 ") for ln in trace), trace)
        self.assertTrue(trace[-1].startswith("RecursionError"))

    def test_sys_exit_is_traced_too(self):
        msg, lines = self.fails("x/c.py", "import sys\n\n\ndef control(e, t, dt):\n    sys.exit(3)\n")
        self.assertIn("sys.exit(3)", msg)
        self.assertEqual(lines[-1].split("\n")[1:], ["  c.py 第 5 行 control：sys.exit(3)", "SystemExit: 3"])

    def test_no_trace_for_what_the_backend_reports(self):
        """Wrong entry, a wrong return value: no frame of the user's is involved."""
        with self.assertRaisesRegex(ValueError, "没有找到"):
            self.load("n/c.py", "def control(e, t, dt):\n    return [0.0, 0.0, 0.0]\n", "Controller")
        self.assertEqual(self.out.take(end=True), [])
        msg, lines = self.fails("n/d.py", "def control(e, t, dt):\n    pass\n")
        self.assertIn("返回了 None", msg)
        self.assertEqual(lines, [])


# ---------------------------------------------------------------- 算法耗时
class ControlTimeTests(Case):
    def test_time_of_control_alone(self):
        def slow_scene():
            time.sleep(0.05)
            return {"ego": {}}
        ctl = self.load("t/c.py", "import time\n\n\ndef control(e, t, dt, scene):\n    time.sleep(0.02)\n"
                                  "    return [0.0, 0.0, 0.0]\n", scene=slow_scene)
        self.assertIsNone(ctl.ms)
        ctl([0.0, 0.0], 0.0)
        self.assertGreaterEqual(ctl.ms, 19.0)
        self.assertLess(ctl.ms, 45.0)  # not the time the scene took

    def test_session_telemetry_and_totals(self):
        path = self.write("s/c.py", "import time\nN = [0]\n\n\ndef control(e, t, dt):\n    N[0] += 1\n"
                                    "    print('call', N[0])\n    time.sleep(0.03 if N[0] == 2 else 0.001)\n"
                                    "    return [0.0, 0.0, 0.0]\n")
        with mock.patch.object(session, "CarlaVehicleSync", FakeSync), \
                mock.patch.object(session, "start_scene", lambda s, *a, **k: {"collisions": []}), \
                mock.patch.object(session, "scene_step", lambda *a, **k: {}):
            s = session.CoSimSession(FakeWorld(), VEHICLE, None,
                                     cfg(run={"driver": "custom", "controller": {"path": path, "entry": "control"}}))
            self.addCleanup(lambda: s.stop(release_vehicle=False))
            s.start()
            tels = [s.step() for _ in range(3)]
        self.assertEqual(s.algo_out.take(), ["call 1", "call 2", "call 3"])
        self.assertGreaterEqual(tels[1]["ctrl_ms"], 29.0)
        self.assertLess(tels[2]["ctrl_ms"], tels[1]["ctrl_ms"])
        self.assertEqual(tels[2]["ctrl_ms_max"], tels[1]["ctrl_ms"])  # the longest so far
        self.assertEqual(s.ctrl_n, 3)
        self.assertEqual(s.ctrl_ms_max, tels[1]["ctrl_ms"])
        self.assertAlmostEqual(s.ctrl_ms_max_t, 0.02)  # t handed to that call
        self.assertAlmostEqual(s.ctrl_ms_sum, sum(t["ctrl_ms"] for t in tels))

    def test_no_time_for_the_test_drivers(self):
        with mock.patch.object(session, "CarlaVehicleSync", FakeSync), \
                mock.patch.object(session, "start_scene", lambda s, *a, **k: {"collisions": []}), \
                mock.patch.object(session, "scene_step", lambda *a, **k: {}):
            s = session.CoSimSession(FakeWorld(), VEHICLE, None, cfg())  # demo driver
            self.addCleanup(lambda: s.stop(release_vehicle=False))
            s.start()
            tel = s.step()
        self.assertNotIn("ctrl_ms", tel)
        self.assertEqual(s.ctrl_n, 0)


# ------------------------------------------------------------------ backend
class Session:
    """A CoSimSession stand-in whose algorithm prints."""

    def __init__(self, *args, fail_start=False, fail_step=False):
        self.algo_out = session.AlgoOutput()
        self.algo_out.path = os.path.abspath(__file__)  # its frames count as the algorithm's
        self.fail_start, self.fail_step = fail_start, fail_step
        self.scene, self.end_reason, self.frame, self.n_frames, self.done = None, "", 0, 0, False
        self.d = {"sync": {"frame_dt": 0.02}}
        self.ctrl_n, self.ctrl_ms_sum, self.ctrl_ms_max, self.ctrl_ms_max_t = 0, 0.0, 0.0, 0.0

    def start(self):
        with self.algo_out:
            print("loading")
            if self.fail_start:
                raise NameError("name 'x' is not defined")
        return {"server_api": None, "mock": True, "external_api": False, "reference_point": [0, 0, 0],
                "inner_steps": 1, "frame_dt": 0.02, "t_stop": 0.0, "t": 0.0, "collisions": []}

    def step(self):
        self.frame += 1
        with self.algo_out:
            print("step", self.frame)
            if self.fail_step:
                raise ZeroDivisionError("float division by zero")
        self.ctrl_n, self.ctrl_ms_sum, self.ctrl_ms_max, self.ctrl_ms_max_t = 2, 3.0, 2.5, 0.02
        return {"t": 0.02 * self.frame, "frame": self.frame, "world_frame": self.frame, "done": False, "rt_factor": 1.0}

    def stop(self, release_vehicle=True):
        pass


class BackendTests(unittest.TestCase):
    def setUp(self):
        self.backend = Backend()
        self.events = []
        self.backend.emit = self.events.append
        settings = SimpleNamespace(synchronous_mode=False, fixed_delta_seconds=None)
        self.backend.world = SimpleNamespace(get_settings=lambda: settings, apply_settings=lambda s: None,
                                             tick=lambda: 1, reset_all_traffic_lights=lambda: None)
        for p in (mock.patch.object(backend_server.traceback, "print_exc"),
                  mock.patch.multiple(sys, stdout=io.StringIO())):
            p.start()
            self.addCleanup(p.stop)

    def logs(self):
        return [(e["level"], e["msg"]) for e in self.events if e.get("event") == "log"]

    def test_each_frame_sends_what_was_printed(self):
        self.backend.session, self.backend.cosim_state = Session(), "running"
        self.backend._cosim_frame()
        self.backend._cosim_frame()
        self.assertEqual([m for lv, m in self.logs() if lv == "algo"], ["step 1", "step 2"])

    def test_error_in_control_prints_and_trace_come_first(self):
        self.backend.session, self.backend.cosim_state = Session(fail_step=True), "running"
        self.backend._cosim_frame()
        logs = self.logs()
        self.assertEqual(logs[0], ("algo", "step 1"))
        self.assertEqual(logs[1][0], "algo")
        self.assertIn("ZeroDivisionError: float division by zero", logs[1][1])
        self.assertIn("test_offline_algoout.py", logs[1][1])
        self.assertEqual(logs[2][0], "error")
        self.assertTrue(logs[2][1].startswith("仿真出错："), logs[2])
        self.assertEqual(self.events[-1]["event"], "cosim_state")
        self.assertEqual(self.events[-1]["state"], "error")

    def test_failed_start_shows_what_it_printed(self):
        self.backend.cmd_spawn_ego = lambda *args: setattr(self.backend, "ego", SimpleNamespace(is_alive=False))
        with mock.patch.object(backend_server, "CoSimSession", lambda *a: Session(fail_start=True)), \
                mock.patch.object(backend_server.rigmod, "build_preset", lambda *a: []), \
                mock.patch.object(backend_server.rigmod, "spec_of", lambda ego: {}), \
                mock.patch.object(backend_server.rigmod, "preset_reference", lambda *a: [0.0, 0.0, 0.0]):
            with self.assertRaisesRegex(NameError, "'x'"):
                self.backend.cmd_cosim_start({"drive": {"dynamics": "cosim"}, "carsim": {"mock": True},
                                              "run": {"driver": "demo"}})
        algo = [m for lv, m in self.logs() if lv == "algo"]
        self.assertEqual(algo[0], "loading")
        self.assertIn("NameError", algo[1])
        states = [e for e in self.events if e.get("event") == "cosim_state"]
        self.assertEqual(states[-1]["state"], "error")
        self.assertGreater(self.events.index(states[-1]), max(i for i, e in enumerate(self.events)
                                                              if e.get("level") == "algo"))

    def test_started_run_shows_the_load_output(self):
        self.backend.cmd_spawn_ego = lambda *args: setattr(self.backend, "ego", SimpleNamespace(is_alive=False))
        with mock.patch.object(backend_server, "CoSimSession", lambda *a: Session()), \
                mock.patch.object(backend_server.rigmod, "build_preset", lambda *a: []), \
                mock.patch.object(backend_server.rigmod, "spec_of", lambda ego: {}), \
                mock.patch.object(backend_server.rigmod, "preset_reference", lambda *a: [0.0, 0.0, 0.0]):
            self.backend.cmd_cosim_start({"drive": {"dynamics": "cosim"}, "carsim": {"mock": True},
                                          "run": {"driver": "demo"}})
        logs = self.logs()
        self.assertEqual(logs[0], ("algo", "loading"))  # before "联合仿真开始"
        self.assertTrue(logs[1][1].startswith("联合仿真开始"), logs)

    def test_run_end_gives_the_left_out_count_and_the_control_time(self):
        ses = Session()
        self.backend.session, self.backend.cosim_state = ses, "running"
        now = [0.0]
        ses.algo_out.clock = lambda: now[0]
        with ses.algo_out:
            for i in range(25):
                print("burst", i)
        self.backend._cosim_frame()  # step 1: prints one more, limit still reached
        self.assertEqual(len([1 for lv, m in self.logs() if lv == "algo"]), 20)
        self.backend.cmd_cosim_stop()
        logs = self.logs()
        self.assertIn(("algo", "（省略 6 行，全部输出见 backend.log）"), logs)
        self.assertIn(("info", "算法耗时：control() 调用 2 次，平均 1.50 ms，最长 2.50 ms（t = 0.02 s）；仿真步长 20 ms"), logs)
        self.assertEqual(self.events[-1]["state"], "stopped")

    def test_other_runs_have_no_control_time_line(self):
        self.backend.session, self.backend.cosim_state = SimpleNamespace(stop=lambda release_vehicle=True: None), "running"
        self.backend.cmd_cosim_stop()
        self.assertFalse([m for lv, m in self.logs() if "算法耗时" in m])


if __name__ == "__main__":
    unittest.main()
