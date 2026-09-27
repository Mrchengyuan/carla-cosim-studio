"""GUI fixes that need no CARLA server: the GUI and the backend agree on the
protocol ("hello"), a hung backend prints every thread's stack when asked over
its connection (Windows has no SIGUSR1) without the worker, world_info says
whether the ego drives on CARLA's autopilot, and the GUI's platform layer
(cosim_gui/tests/platform_test.cpp: connect time-outs, ending the backend,
exit reasons; with MinGW installed also compiled for Windows).

    python tests/test_offline_guimisc.py
"""

import json
import os
import re
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
BRIDGE = os.path.join(HERE, "..")
GUI = os.path.join(HERE, "..", "..", "cosim_gui")
sys.path.insert(0, BRIDGE)

import backend_server  # noqa: E402
from backend_server import Backend  # noqa: E402


class ProtocolTests(unittest.TestCase):
    def test_hello_is_the_protocol_the_gui_was_built_for(self):
        self.assertEqual(Backend().cmd_hello(), {"protocol": backend_server.PROTOCOL})
        with open(os.path.join(GUI, "src", "app.cpp"), encoding="utf-8") as f:
            m = re.search(r"constexpr int kBackendProtocol = (\d+);", f.read())
        self.assertIsNotNone(m, "kBackendProtocol not found in app.cpp")
        self.assertEqual(int(m.group(1)), backend_server.PROTOCOL, "raise both together")


class AutopilotStateTests(unittest.TestCase):
    def backend(self):
        b = Backend()
        settings = SimpleNamespace(synchronous_mode=False, fixed_delta_seconds=None, no_rendering_mode=False)
        b.world = SimpleNamespace(get_settings=lambda: settings, get_map=lambda: SimpleNamespace(name="Carla/Maps/Town10HD_Opt"),
                                  get_weather=lambda: None, get_actors=lambda: [])
        b._weather_dict = lambda wp: {}
        calls = []
        b.ego = SimpleNamespace(id=7, is_alive=True, set_autopilot=lambda on, port: calls.append(("autopilot", on)),
                                apply_control=lambda c: None, set_target_velocity=lambda v: None,
                                set_target_angular_velocity=lambda v: None)
        b.tm = SimpleNamespace(get_port=lambda: 8000)
        return b, calls

    def test_world_info_reports_autopilot_until_the_run_parks_the_ego(self):
        b, calls = self.backend()
        self.assertIs(b.cmd_world_info()["ego_autopilot"], False)
        b.ego_autopilot = True
        self.assertIs(b.cmd_world_info()["ego_autopilot"], True)
        b._park_ego()  # the end of every run
        self.assertIn(("autopilot", False), calls)
        self.assertIs(b.cmd_world_info()["ego_autopilot"], False)

    def test_no_ego_no_autopilot(self):
        b, _ = self.backend()
        b.ego_autopilot = True
        b.ego = None
        self.assertIs(b.cmd_world_info()["ego_autopilot"], False)


class DumpStacksTests(unittest.TestCase):
    def test_socket_thread_answers_dump_stacks_itself(self):
        sent = []
        with mock.patch.object(backend_server.faulthandler, "dump_traceback") as dump:
            self.assertTrue(backend_server._socket_thread_request({"id": 7, "cmd": "dump_stacks"}, sent.append))
            dump.assert_called_once_with(all_threads=True)
            self.assertEqual(sent, [{"id": 7, "ok": True, "result": True}])
            self.assertFalse(backend_server._socket_thread_request({"id": 8, "cmd": "ping"}, sent.append))
        self.assertEqual(len(sent), 1, "other requests go to the worker")

    def test_backend_prints_stacks_while_its_worker_is_busy(self):
        """A real backend process (no CARLA; a free port, not 57100) whose worker
        sleeps in a command: dump_stacks is answered at once and the log names it."""
        with socket.socket() as probe:
            probe.bind(("127.0.0.1", 0))
            port = probe.getsockname()[1]
        tmp = tempfile.mkdtemp(prefix="cc_guimisc_")
        log_path = os.path.join(tmp, "backend.log")
        starter = ("import sys, time\n"
                   "import backend_server as b\n"
                   "def cmd_test_sleep(self, s=3.0):\n"
                   "    time.sleep(s)\n"
                   "    return True\n"
                   "b.Backend.cmd_test_sleep = cmd_test_sleep\n"
                   "b.serve(int(sys.argv[1]))\n")
        with open(log_path, "w") as log:
            proc = subprocess.Popen([sys.executable, "-u", "-c", starter, str(port)], cwd=BRIDGE,
                                    stdout=log, stderr=subprocess.STDOUT)
        conn = None
        try:
            for _ in range(100):
                try:
                    conn = socket.create_connection(("127.0.0.1", port), timeout=1)
                    break
                except OSError:
                    time.sleep(0.1)
            self.assertIsNotNone(conn, "backend did not listen")
            conn.settimeout(10)
            rf = conn.makefile("rb")

            def send(i, cmd, **args):
                conn.sendall((json.dumps({"id": i, "cmd": cmd, "args": args}) + "\n").encode())

            def reply():
                while True:
                    msg = json.loads(rf.readline())
                    if "id" in msg:
                        return msg

            send(1, "hello")
            self.assertEqual(reply(), {"id": 1, "ok": True, "result": {"protocol": backend_server.PROTOCOL}})
            send(2, "test_sleep", s=3.0)
            time.sleep(0.3)  # the worker is in it now
            t0 = time.time()
            send(3, "dump_stacks")
            r = reply()
            self.assertEqual(r["id"], 3, "answered before the busy worker's reply")
            self.assertTrue(r["ok"])
            self.assertLess(time.time() - t0, 2.0)
            self.assertEqual(reply()["id"], 2)
            send(4, "shutdown")
            self.assertEqual(reply()["id"], 4)
            proc.wait(10)
            with open(log_path, encoding="utf-8", errors="replace") as f:
                text = f.read()
            self.assertIn("most recent call first", text)
            self.assertIn("cmd_test_sleep", text, "the worker's stack is in the log")
        finally:
            if conn is not None:
                conn.close()
            if proc.poll() is None:
                proc.kill()
                proc.wait(5)
            shutil.rmtree(tmp, ignore_errors=True)


class PlatformTests(unittest.TestCase):
    @unittest.skipIf(os.name == "nt", "the test program is for POSIX")
    def test_platform_layer(self):
        cxx = shutil.which("c++") or shutil.which("g++") or shutil.which("clang++")
        if not cxx:
            self.skipTest("no C++ compiler")
        tmp = tempfile.mkdtemp(prefix="cc_guimisc_cxx_")
        try:
            exe = os.path.join(tmp, "platform_test")
            build = subprocess.run([cxx, "-std=c++17", "-Wall", "-I", os.path.join(GUI, "src"),
                                    os.path.join(GUI, "tests", "platform_test.cpp"),
                                    os.path.join(GUI, "src", "platform.cpp"), "-o", exe, "-pthread"],
                                   capture_output=True, text=True)
            self.assertEqual(build.returncode, 0, build.stderr)
            run = subprocess.run([exe], capture_output=True, text=True, timeout=60)
            print(run.stdout, end="")
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_platform_compiles_for_windows(self):
        cxx = shutil.which("x86_64-w64-mingw32-g++-posix") or shutil.which("x86_64-w64-mingw32-g++")
        if not cxx:
            self.skipTest("no MinGW cross compiler")
        tmp = tempfile.mkdtemp(prefix="cc_guimisc_mingw_")
        try:
            build = subprocess.run([cxx, "-std=c++17", "-Wall", "-c", os.path.join(GUI, "src", "platform.cpp"),
                                    "-o", os.path.join(tmp, "platform.o")], capture_output=True, text=True)
            self.assertEqual(build.returncode, 0, build.stderr)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    unittest.main(verbosity=2)
