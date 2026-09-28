"""Remote mode, what the GUI needs from the backend (no CARLA server needed):
a GUI whose hello asks for JPEG (remote_backend: through an SSH tunnel) gets
its live view frames as JPEG of quality 80 (decoded back: same size, nearly
the same pixels, a fraction of the bytes), any other GUI raw RGB as before;
"path_status" checks paths on the backend's machine on the IO thread,
relative ones from the bridge directory like a run; "controller_browse"
(the 控制算法 “浏览…” of a GUI on another computer) lists a folder there with
its .py files, their entries and docstrings, read with ast and never run;
"restart_backend" is
answered by the socket thread (also while the worker is busy), which then
ends the backend with exit code 3 (real backend processes on a free port, no
CARLA). The GUI's JPEG decoding (cosim_gui/src/jpeg_decode.cpp, stb_image)
runs as cosim_gui/tests/jpeg_decode_test.cpp on the backend's frame, built
with the system C++ compiler (skipped without one).

    python tests/test_offline_remote_gui.py
"""

import base64
import io
import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from types import SimpleNamespace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
BRIDGE = os.path.realpath(os.path.join(HERE, ".."))
GUI = os.path.realpath(os.path.join(HERE, "..", "..", "cosim_gui"))
sys.path.insert(0, BRIDGE)

import backend_server  # noqa: E402
import settings as st  # noqa: E402
from backend_server import Backend  # noqa: E402
from session import browse_controllers, check_run_files  # noqa: E402


def camera_image(w=320, h=180):
    """Smooth gradients and a flat block, like a camera picture."""
    y, x = np.mgrid[0:h, 0:w]
    img = np.stack([x * 255 // (w - 1), y * 255 // (h - 1), (x + y) * 255 // (w + h - 2)], 2).astype(np.uint8)
    img[h // 4:h // 2, w // 4:w // 2] = (200, 40, 40)
    return img


def view_frame(backend, rgb):
    """The frame event the GUI gets for one picture of a camera view (CARLA
    hands the callback BGRA)."""
    sent = []
    backend.emit = sent.append
    bgra = np.concatenate([rgb[:, :, ::-1], np.full(rgb.shape[:2] + (1,), 255, np.uint8)], 2)
    data = SimpleNamespace(raw_data=bgra.tobytes(), width=rgb.shape[1], height=rgb.shape[0], frame=5, timestamp=1.0)
    backend.views._on_data("p0", {"kind": "rgb", "period": 0.0, "last": 0.0, "buf": []}, data)
    frames = [m for m in sent if m.get("event") == "frame"]
    assert len(frames) == 1, sent
    return frames[0]


class JpegFrameTests(unittest.TestCase):
    def test_hello_jpeg_sends_jpeg_frames(self):
        from PIL import Image
        b = Backend()
        self.assertEqual(b.cmd_hello(jpeg=True), {"protocol": backend_server.PROTOCOL})
        rgb = camera_image()
        f = view_frame(b, rgb)
        self.assertNotIn("rgb", f)
        self.assertEqual((f["view"], f["w"], f["h"], f["frame"]), ("p0", 320, 180, 5))
        data = base64.b64decode(f["jpeg"])
        img = Image.open(io.BytesIO(data))
        self.assertEqual((img.format, img.size, img.mode), ("JPEG", (320, 180), "RGB"))
        self.assertLess(np.abs(np.asarray(img).astype(int) - rgb.astype(int)).mean(), 4.0)
        self.assertLess(len(data), rgb.nbytes / 5, "a fraction of the raw bytes")
        ref = io.BytesIO()
        Image.fromarray(rgb).save(ref, format="JPEG", quality=80)
        self.assertEqual(data, ref.getvalue(), "Pillow, quality 80")

    def test_other_guis_get_raw_rgb(self):
        b = Backend()
        rgb = camera_image()
        f = view_frame(b, rgb)  # no hello yet
        self.assertNotIn("jpeg", f)
        self.assertEqual(base64.b64decode(f["rgb"]), rgb.tobytes())
        b.cmd_hello(jpeg=True)
        b.cmd_hello()  # the next GUI's hello: a local GUI sends no arguments
        f = view_frame(b, rgb)
        self.assertNotIn("jpeg", f)
        self.assertEqual(base64.b64decode(f["rgb"]), rgb.tobytes())


class PathStatusTests(unittest.TestCase):
    def test_paths_on_the_backend_machine(self):
        b = Backend()
        b.submit({"id": 1, "cmd": "path_status", "args": {"paths": ["x.py"]}}, lambda msg: None)
        self.assertEqual((b.io_requests.qsize(), b.requests.qsize()), (1, 0), "served by the IO thread")
        cwd = os.getcwd()
        with tempfile.TemporaryDirectory(prefix="cc_remote_gui_") as tmp:
            os.makedirs(os.path.join(tmp, "controllers"))
            open(os.path.join(tmp, "controllers", "ctl.py"), "w").close()
            os.chdir(tmp)  # the backend's working directory: the bridge directory
            try:
                here = os.getcwd()
                ctl_dir = os.path.join(here, "controllers")
                r = b.cmd_path_status(paths=["controllers/ctl.py", ctl_dir, "missing.py", ""])
                # A run takes a relative controller path the same way.
                d = st.default_dict()
                d["drive"]["dynamics"] = "cosim"
                d["run"]["driver"] = "custom"
                d["carsim"]["mock"] = True
                d["run"]["controller"]["path"] = "missing.py"
                with self.assertRaises(ValueError) as cm:
                    check_run_files(d)
            finally:
                os.chdir(cwd)
        self.assertEqual(r["controllers/ctl.py"],
                         {"resolved": os.path.join(here, "controllers", "ctl.py"), "exists": True, "is_file": True})
        self.assertEqual(r[ctl_dir], {"resolved": ctl_dir, "exists": True, "is_file": False})
        self.assertEqual(r["missing.py"], {"resolved": os.path.join(here, "missing.py"), "exists": False, "is_file": False})
        self.assertEqual(r[""], {"resolved": "", "exists": False, "is_file": False})
        self.assertIn(r["missing.py"]["resolved"], str(cm.exception))


class ControllerBrowseTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="cc_browse_")
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.base = os.path.join(self.tmp, "bridge")
        ctl = os.path.join(self.base, "controllers")
        for d in ("sub", "__pycache__", ".hidden"):
            os.makedirs(os.path.join(ctl, d))
        os.makedirs(os.path.join(self.tmp, "elsewhere"))
        self.marker = os.path.join(self.tmp, "ran.txt")
        files = {
            "a_class.py": '"""类写法。\n\n第二行不显示。"""\nclass Controller:\n    def control(self, exports, t, dt):\n'
                          '        return [0, 0, 0]\n',
            "b_func.py": "def control(exports, t, dt):\n    return [0, 0, 0]\n",
            "c_two.py": "class Slow:\n    def control(self, e, t, dt):\n        pass\n\n\n"
                        "class Fast:\n    def control(self, e, t, dt):\n        pass\n",
            "d_none.py": "def helper():\n    pass\n",
            "e_bad.py": "def control(:\n",
            # Run, it would leave a file behind: listing never runs a file.
            "f_side.py": "open(%r, 'w').write('ran')\nclass Controller:\n    def control(self, e, t, dt):\n        pass\n"
                         % self.marker,
            "notes.txt": "not python\n",
        }
        for name, text in files.items():
            with open(os.path.join(ctl, name), "w", encoding="utf-8") as f:
                f.write(text)
        with open(os.path.join(ctl, "g_gbk.py"), "wb") as f:  # Chinese in a string, saved as GBK
            f.write('class Controller:\n    def control(self, e, t, dt):\n        print("车速")\n'.encode("gbk"))
        with open(os.path.join(self.tmp, "elsewhere", "far.py"), "w") as f:
            f.write("def control(e, t, dt):\n    return [0, 0, 0]\n")

    def by_name(self, r):
        return {f["name"]: f for f in r["files"]}

    def test_the_controllers_folder_its_files_entries_and_docs(self):
        r = browse_controllers("", base=self.base)
        self.assertEqual((r["folder"], r["shown"], r["parent"], r["note"]),
                         (os.path.join(self.base, "controllers"), "controllers", self.base, ""))
        self.assertEqual(r["dirs"], ["sub"])  # no __pycache__, no hidden folders
        f = self.by_name(r)
        self.assertEqual(sorted(f), ["a_class.py", "b_func.py", "c_two.py", "d_none.py", "e_bad.py", "f_side.py",
                                     "g_gbk.py"])  # .py files only
        self.assertEqual({n: v["entries"] for n, v in f.items()},
                         {"a_class.py": ["Controller"], "b_func.py": ["control"], "c_two.py": ["Slow", "Fast"],
                          "d_none.py": [], "e_bad.py": [], "f_side.py": ["Controller"], "g_gbk.py": []})
        self.assertEqual(f["a_class.py"]["path"], "controllers/a_class.py")
        self.assertEqual(f["a_class.py"]["doc"], "类写法。")
        self.assertIn("第 1 行有语法错误", f["e_bad.py"]["error"])
        self.assertIn("第 3 行有语法错误", f["g_gbk.py"]["error"])  # what loading it would say too
        self.assertEqual(f["b_func.py"]["error"], "")
        self.assertFalse(os.path.exists(self.marker), "a file was run")

    def test_a_file_path_shows_its_folder_and_a_missing_one_the_controllers(self):
        self.assertEqual(browse_controllers("controllers/b_func.py", base=self.base)["shown"], "controllers")
        self.assertEqual(browse_controllers("controllers/sub", base=self.base)["shown"], "controllers/sub")
        r = browse_controllers("nope/x.py", base=self.base)
        self.assertEqual(r["shown"], "controllers")
        self.assertIn("nope/x.py 不存在", r["note"])

    def test_up_to_the_bridge_folder_and_out_of_it(self):
        r = browse_controllers(self.base, base=self.base)
        self.assertEqual((r["shown"], r["dirs"], r["files"]), (".", ["controllers"], []))
        far = os.path.join(self.tmp, "elsewhere")
        r = browse_controllers(far, base=self.base)
        self.assertEqual(r["shown"], far)  # outside the bridge folder: absolute paths
        self.assertEqual(self.by_name(r)["far.py"]["path"], os.path.join(far, "far.py"))
        self.assertIsNone(browse_controllers("/", base=self.base)["parent"])

    def test_on_the_io_thread_and_over_the_socket(self):
        b = Backend()
        b.submit({"id": 1, "cmd": "controller_browse", "args": {"path": ""}}, lambda msg: None)
        self.assertEqual((b.io_requests.qsize(), b.requests.qsize()), (1, 0), "served by the IO thread")
        be = BackendProcess(lambda port: ["backend_server.py", "--port", str(port)])
        self.addCleanup(be.close)
        be.send(1, "hello", jpeg=True)
        self.assertEqual(be.reply()["result"], {"protocol": backend_server.PROTOCOL})
        be.send(2, "controller_browse", path="controllers/examples/ex4_function.py")
        r = be.reply()
        self.assertTrue(r["ok"], r)
        f = self.by_name(r["result"])["ex4_function.py"]
        self.assertEqual((r["result"]["shown"], f["path"], f["entries"]),
                         ("controllers/examples", "controllers/examples/ex4_function.py", ["control"]))


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class BackendProcess:
    """A real backend process without CARLA on a free port (not 57100 / 57120),
    talked to like the GUI does."""

    def __init__(self, argv):
        self.tmp = tempfile.mkdtemp(prefix="cc_remote_gui_")
        self.log_path = os.path.join(self.tmp, "backend.log")
        port = free_port()
        with open(self.log_path, "w") as log:
            self.proc = subprocess.Popen([sys.executable, "-u"] + argv(port), cwd=BRIDGE,
                                         stdout=log, stderr=subprocess.STDOUT)
        self.conn = None
        for _ in range(150):
            try:
                self.conn = socket.create_connection(("127.0.0.1", port), timeout=1)
                break
            except OSError:
                if self.proc.poll() is not None:
                    break
                time.sleep(0.1)
        if self.conn is None:
            self.close()
            raise AssertionError("backend did not listen")
        self.conn.settimeout(10)
        self.rf = self.conn.makefile("rb")

    def send(self, i, cmd, **args):
        self.conn.sendall((json.dumps({"id": i, "cmd": cmd, "args": args}) + "\n").encode())

    def reply(self):
        while True:
            line = self.rf.readline()
            if not line:
                raise AssertionError("the backend closed the connection")
            msg = json.loads(line)
            if "id" in msg:
                return msg

    def log(self):
        with open(self.log_path, encoding="utf-8", errors="replace") as f:
            return f.read()

    def close(self):
        if self.conn is not None:
            self.conn.close()
        if self.proc.poll() is None:
            self.proc.kill()
            self.proc.wait(5)
        shutil.rmtree(self.tmp, ignore_errors=True)


class RestartBackendTests(unittest.TestCase):
    def test_socket_thread_restarts_the_backend_with_exit_code_3(self):
        be = BackendProcess(lambda port: ["backend_server.py", "--port", str(port)])
        self.addCleanup(be.close)
        be.send(1, "hello", jpeg=True)
        self.assertEqual(be.reply(), {"id": 1, "ok": True, "result": {"protocol": backend_server.PROTOCOL}})
        be.send(2, "path_status", paths=["controllers/example_controller.py", "no_such_controller.py"])
        r = be.reply()
        self.assertTrue(r["ok"], r)
        self.assertEqual(r["result"]["controllers/example_controller.py"],
                         {"resolved": os.path.join(BRIDGE, "controllers", "example_controller.py"),
                          "exists": True, "is_file": True})
        self.assertEqual(r["result"]["no_such_controller.py"]["exists"], False)
        be.send(3, "restart_backend")
        self.assertEqual(be.reply(), {"id": 3, "ok": True, "result": True})
        self.assertEqual(be.proc.wait(10), 3)
        text = be.log()
        self.assertIn("most recent call first", text, "every thread's stack is in the log")
        self.assertIn("restart_backend: exiting with code 3", text)

    def test_restart_backend_while_the_worker_is_busy(self):
        starter = ("import sys, time\n"
                   "import backend_server as b\n"
                   "def cmd_test_sleep(self, s=5.0):\n"
                   "    time.sleep(s)\n"
                   "    return True\n"
                   "b.Backend.cmd_test_sleep = cmd_test_sleep\n"
                   "b.serve(int(sys.argv[1]))\n")
        be = BackendProcess(lambda port: ["-c", starter, str(port)])
        self.addCleanup(be.close)
        be.send(1, "test_sleep", s=5.0)
        time.sleep(0.3)  # the worker is in it now
        t0 = time.time()
        be.send(2, "restart_backend")
        r = be.reply()
        self.assertEqual(r["id"], 2, "answered before the busy worker's reply")
        self.assertTrue(r["ok"])
        self.assertEqual(be.proc.wait(10), 3)
        self.assertLess(time.time() - t0, 3.0, "without waiting for the worker")
        self.assertIn("cmd_test_sleep", be.log(), "the worker's stack is in the log")


class GuiJpegDecodeTests(unittest.TestCase):
    def test_gui_decodes_the_backends_jpeg_frame(self):
        cxx = shutil.which("c++") or shutil.which("g++") or shutil.which("clang++")
        if not cxx:
            self.skipTest("no C++ compiler")
        b = Backend()
        b.cmd_hello(jpeg=True)
        rgb = camera_image()
        f = view_frame(b, rgb)
        with tempfile.TemporaryDirectory(prefix="cc_remote_gui_cxx_") as tmp:
            jpg, raw = os.path.join(tmp, "frame.jpg"), os.path.join(tmp, "frame.rgb")
            with open(jpg, "wb") as fh:
                fh.write(base64.b64decode(f["jpeg"]))
            with open(raw, "wb") as fh:
                fh.write(rgb.tobytes())
            exe = os.path.join(tmp, "jpeg_decode_test")
            build = subprocess.run([cxx, "-std=c++17", "-Wall", "-I", os.path.join(GUI, "src"),
                                    "-I", os.path.join(GUI, "third_party", "stb"),
                                    os.path.join(GUI, "tests", "jpeg_decode_test.cpp"),
                                    os.path.join(GUI, "src", "jpeg_decode.cpp"), "-o", exe],
                                   capture_output=True, text=True)
            self.assertEqual(build.returncode, 0, build.stderr[-3000:])
            run = subprocess.run([exe, jpg, raw, str(f["w"]), str(f["h"])], capture_output=True, text=True, timeout=60)
            print(run.stdout, end="")
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            self.assertIn(" 0 failed", run.stdout)


if __name__ == "__main__":
    unittest.main(verbosity=2)
