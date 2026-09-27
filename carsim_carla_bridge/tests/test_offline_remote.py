"""Remote mode (carsim.remote) without CARLA: CarSim runs in the CarSim
service (carsim_service.py) on the user's Windows computer, the run on the
server. Real sockets on free ports and a real carsim_service.py process
(--mock, or python_carsim_env on a fake solver); CARLA is faked as in
test_offline_carsim.py.

    python tests/test_offline_remote.py
"""

import ast
import contextlib
import io
import json
import math
import os
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
BRIDGE = os.path.join(HERE, "..")
sys.path.insert(0, HERE)
sys.path.insert(0, BRIDGE)

import backend_server  # noqa: E402
import carsim_local  # noqa: E402
import carsim_remote  # noqa: E402
import run_cosim  # noqa: E402
import session as ses  # noqa: E402
import settings as st  # noqa: E402
import test_offline_carsim as oc  # noqa: E402  (its fake CARLA and fake CarSim solver)
from carsim_local import RemoteCarSimError  # noqa: E402
from mock_carsim import MockCarSimEnv  # noqa: E402
from test_backend import Conn  # noqa: E402

SERVICE = os.path.join(BRIDGE, "carsim_service.py")
# The texts the user reads (REMOTE_SPEC.md).
NO_SERVICE = ("Windows 上的 CarSim 服务没有连上云端：请在 Windows 上双击启动脚本（启动远程仿真.bat），"
              "看到“已连上云端”后再点运行")
LOST = "与 Windows 上的 CarSim 服务的连接断开了（"


def wait(cond, timeout=10.0):
    end = time.time() + timeout
    while not cond():
        if time.time() > end:
            raise AssertionError("timed out waiting")
        time.sleep(0.02)


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def watch(conn):
    """[(time, cmd), ...] of what the backend sends on conn from now on."""
    sent, send = [], conn.lines.send

    def record(msg):
        sent.append((time.monotonic(), msg.get("cmd")))
        send(msg)
    conn.lines.send = record
    return sent


def listening(pid):
    """The TCP ports process pid listens on (Linux: /proc)."""
    inodes, ports = set(), set()
    for fd in os.listdir("/proc/%d/fd" % pid):
        with contextlib.suppress(OSError):
            link = os.readlink("/proc/%d/fd/%s" % (pid, fd))
            if link.startswith("socket:["):
                inodes.add(link[8:-1])
    for table in ("/proc/net/tcp", "/proc/net/tcp6"):
        with contextlib.suppress(OSError), open(table) as f:
            for cols in (line.split() for line in f.readlines()[1:]):
                if cols[3] == "0A" and cols[9] in inodes:  # 0A = LISTEN
                    ports.add(int(cols[1].rsplit(":", 1)[1], 16))
    return ports


def remote_cfg(**sync):
    """A remote run's config: paths on the Windows computer, which the server does not have."""
    d = st.load_dict(None, {"carsim": {"remote": True, "sim_path": "C:\\CarSim\\simfile.sim",
                                       "repo_path": "D:\\python_carsim_env"},
                            "run": {"driver": "demo", "log_path": ""}, "sync": dict({"duration": 2.0}, **sync)})
    return d


class FakeService:
    """A CarSim service by hand: says hello, then answer(request) -> result (None: no reply)."""

    def __init__(self, port, answer=lambda req: None, host="pc-1", protocol=carsim_local.SERVICE_PROTOCOL):
        self.sock = socket.create_connection(("127.0.0.1", port))
        self.lines = carsim_local.JsonLines(self.sock)
        self.lines.send({"type": "hello", "role": "carsim", "protocol": protocol, "platform": "win32",
                         "host": host, "python": "3.10.11"})
        self.hello = self.lines.recv(5)
        self.answer, self.requests, self.closed = answer, [], threading.Event()
        threading.Thread(target=self._serve, daemon=True).start()

    def _serve(self):
        try:
            while True:
                req = self.lines.recv()
                if req is None:
                    break
                self.requests.append(req)
                if "cmd" not in req:  # a notice (e.g. "replaced"): nothing to answer
                    continue
                res = self.answer(req)
                if res is not None:
                    self.lines.send({"id": req["id"], "ok": True, "result": res})
        except (OSError, ValueError):
            pass
        self.closed.set()

    def close(self):
        with contextlib.suppress(OSError):
            self.sock.shutdown(socket.SHUT_RDWR)  # wakes _serve: a bare close() would not end the connection
        self.sock.close()


def mock_answers(d, obs=None):
    """answer() of a service with the mock: open / reset / step / close, step obs from obs(env) if given."""
    env = carsim_local.mock_env(d)

    def answer(req):
        if req["cmd"] == "open":
            return {"config": env.config, "sim_path": "C:\\CarSim\\simfile.sim"}
        if req["cmd"] == "reset":
            return {"obs": list(env.reset()), "t_current": env.t_current, "config": env.config}
        if req["cmd"] == "step":
            o, _, done, info = env.control_step(req["action"], req["inner"])
            return {"obs": obs(o) if obs else list(o), "done": done, "info": info, "t_current": env.t_current}
        return {}
    return answer


class LinkCase(unittest.TestCase):
    """A ServiceLink (the backend's end) on a free port, as carsim_remote.SERVICE."""

    def setUp(self):
        self.link = carsim_remote.ServiceLink()
        self.port = self.link.listen(0)
        p = mock.patch.object(carsim_remote, "SERVICE", self.link)
        p.start()
        self.addCleanup(p.stop)
        self.addCleanup(lambda: self.link.conn and self.link.conn.close("test over"))

    def service(self, *args, port=None):
        """carsim_service.py connecting to the link; returns (process, console text reader)."""
        fd, path = tempfile.mkstemp(prefix="cc_service_", suffix=".txt")
        self.addCleanup(os.remove, path)
        with os.fdopen(fd, "wb") as log:  # read below through a file of its own: never the process's offset
            p = subprocess.Popen([sys.executable, "-u", SERVICE, "--port", str(port or self.port)] + list(args),
                                 stdout=log, stderr=subprocess.STDOUT, cwd=BRIDGE)

        def stop():
            if p.poll() is None:
                p.kill()
            p.wait()
        self.addCleanup(stop)

        def console():  # a line may be half written
            with open(path, "rb") as f:
                return f.read().decode("utf-8", errors="replace")
        return p, console

    def fake(self, *args, **kw):
        f = FakeService(self.port, *args, **kw)
        self.addCleanup(f.close)
        return f

    def connected(self, host=None):
        wait(lambda: self.link.status()["connected"] and (host is None or self.link.status()["host"] == host))


# ------------------------------------------------------------------ tests
class ProtocolTests(LinkCase):
    def test_hello_open_reset_step_close(self):
        _, console = self.service("--mock")
        self.connected()
        self.assertEqual(self.link.status(), {"connected": True, "host": socket.gethostname(), "platform": sys.platform})
        d = remote_cfg()
        carsim_remote.check(d, self.link)  # the mock: nothing to check (the paths are made up)
        env = carsim_remote.RemoteCarSimEnv(d, self.link)
        local = carsim_local.mock_env(d)  # the same run on this computer
        self.assertEqual(env.config, local.config)
        self.assertEqual(env.sim_path, "")  # --mock: no .sim
        self.assertEqual(env.reset(), local.reset())
        self.assertEqual(env.t_current, local.t_current)
        for k in range(40):
            a = [0.5, 0.0, 3.0 * k]
            self.assertEqual(env.control_step(a, 25), local.control_step(a, 25))  # (obs, 0.0, done, info), bit for bit
            self.assertEqual(env.t_current, local.t_current)
        env.close()
        wait(lambda: "运行结束" in console())
        self.assertEqual(console().splitlines(), ["正在连接云端…", "已连上云端（127.0.0.1:%d），等待运行" % self.port,
                                                  "运行开始：模拟 CarSim", "运行结束"])
        self.assertTrue(self.link.status()["connected"])  # ready for the next run

    def test_service_errors_are_passed_on_unwrapped(self):
        _, console = self.service()  # real CarSim: the .sim is checked on the service's computer
        self.connected()
        d = remote_cfg()
        d["carsim"]["sim_path"] = ""
        with self.assertRaises(ValueError) as local:
            carsim_local.check_carsim(d)
        with self.assertRaises(RemoteCarSimError) as cm:
            ses.make_env(d)
        self.assertEqual(str(cm.exception), str(local.exception))
        wait(lambda: str(local.exception) in console())  # the same words in the service's window
        self.assertTrue(self.link.status()["connected"])

    def test_a_bad_sim_is_refused_before_the_world_changes(self):
        _, console = self.service()  # real CarSim: the service checks the .sim on its computer
        self.connected()
        b = backend_server.Backend()
        b.world, spawned = object(), []
        b.cmd_spawn_ego = lambda *a: spawned.append(a)
        with self.assertRaises(RemoteCarSimError) as cm:
            b.cmd_cosim_start({"carsim": {"remote": True, "sim_path": "C:\\CarSim\\simfile.sim"}, "run": {"driver": "demo"}})
        msg = str(cm.exception)
        self.assertTrue(msg.startswith("CarSim .sim 文件不存在："), msg)
        self.assertEqual(spawned, [])  # like a bad .sim of CarSim on the server
        wait(lambda: msg in console())

    def test_reset_env_does_not_wrap_the_services_reason(self):
        class Remote:
            def reset(self):
                raise RemoteCarSimError("CarSim 没能开始这次运行：License not available")

        class Local:
            config, sim_path = None, "C:\\x.sim"

            def reset(self):
                raise RuntimeError("Configuration did not report valid import/export counts.")
        with self.assertRaises(RemoteCarSimError) as cm:
            ses.reset_env(Remote())
        self.assertEqual(str(cm.exception), "CarSim 没能开始这次运行：License not available")
        with self.assertRaisesRegex(RuntimeError, "^CarSim 没能开始这次运行：RuntimeError: Configuration"):
            ses.reset_env(Local())

    def test_a_request_without_an_answer_ends_the_connection(self):
        d = remote_cfg()
        answers = mock_answers(d)
        fake = self.fake(lambda req: None if req["cmd"] == "step" else answers(req))
        self.connected()
        env = carsim_remote.RemoteCarSimEnv(d, self.link)
        env.reset()
        t0 = time.time()
        with mock.patch.dict(carsim_remote.TIMEOUTS, step=0.3), self.assertRaises(RemoteCarSimError) as cm:
            env.control_step([0.0, 0.0, 0.0], 10)
        self.assertEqual(str(cm.exception), LOST + "0.3 s 没有回应）")
        self.assertLess(time.time() - t0, 5.0)
        self.assertTrue(fake.closed.wait(5))  # the service starts over
        self.assertFalse(self.link.status()["connected"])
        env.close()  # nothing left to close: no request, no error


@unittest.skipUnless(oc.can_build(), "needs Linux, a C compiler and python_carsim_env")
class RealCarSimTests(LinkCase):
    """python_carsim_env on the fake solver of test_offline_carsim.py, in the service's process."""

    def setUp(self):
        super().setUp()
        self.tmp = tempfile.TemporaryDirectory(prefix="cc_remote_")
        self.addCleanup(self.tmp.cleanup)
        repo = os.path.join(self.tmp.name, "repo")
        os.makedirs(repo)
        oc.copy_repo(repo)
        oc.build_fake_solver(self.tmp.name)
        self.d = remote_cfg()
        self.d["carsim"].update(export_names=oc.NAMES, repo_path=repo)
        self.console = self.service()[1]
        self.connected()

    def test_a_run_of_python_carsim_env(self):
        self.d["carsim"]["sim_path"] = oc.write_sim(os.path.join(self.tmp.name, "simfile.sim"), self.tmp.name,
                                                    nexp=len(oc.NAMES), tstop=0.1)
        env = ses.make_env(self.d)
        self.assertEqual(env.sim_path, self.d["carsim"]["sim_path"])  # as the service's computer names it
        self.assertEqual(len(ses.reset_env(env)), len(oc.NAMES))
        self.assertEqual((env.config["n_import"], env.config["t_step"]), (3, 0.001))
        done, n = False, 0
        while not done and n < 20:
            _, _, done, info = env.control_step([0.0, 0.0, 0.0], 20)
            n += 1
        self.assertEqual((done, n, info["return_code"]), (True, 5, 1))  # TSTOP
        self.assertAlmostEqual(env.t_current, 0.1)
        env.close()
        wait(lambda: "运行结束" in self.console())
        self.assertIn("运行开始：%s" % self.d["carsim"]["sim_path"], self.console())

    def test_the_solvers_reason_comes_through_once(self):
        self.d["carsim"]["sim_path"] = oc.write_sim(os.path.join(self.tmp.name, "nolicence.sim"), self.tmp.name, mode=1)
        env = ses.make_env(self.d)
        with self.assertRaises(RemoteCarSimError) as cm:
            ses.reset_env(env)
        msg = str(cm.exception)
        self.assertTrue(msg.startswith("CarSim 没能开始这次运行：License not available (fake solver)"), msg)
        self.assertEqual(msg.count("CarSim 没能开始这次运行"), 1)

    def test_relative_paths_are_the_services(self):
        self.d["carsim"]["sim_path"] = oc.write_sim(os.path.join(self.tmp.name, "rel.sim"), ".")
        with self.assertRaises(ValueError) as local:
            carsim_local.check_carsim(self.d)
        self.assertIn("其中的相对路径 . 是按后端的工作目录", str(local.exception))
        with self.assertRaises(RemoteCarSimError) as cm:
            ses.check_run_files(self.d)  # before the world changes, as with CarSim on the server
        msg = str(cm.exception)
        self.assertIn("找不到 CarSim 求解器", msg)
        self.assertIn("其中的相对路径 . 是按 CarSim 服务的工作目录 %s 解析的" % os.path.realpath(BRIDGE), msg)


class NoServiceTests(unittest.TestCase):
    def setUp(self):
        p = mock.patch.object(carsim_remote, "SERVICE", carsim_remote.ServiceLink())  # nothing connected
        p.start()
        self.addCleanup(p.stop)

    def test_the_run_is_refused_in_plain_words(self):
        d = remote_cfg()
        for fn in (ses.check_run_files, ses.make_env):
            with self.subTest(fn=fn.__name__), self.assertRaises(RemoteCarSimError) as cm:
                fn(d)
            self.assertEqual(str(cm.exception), NO_SERVICE)

    def test_backend_refuses_before_the_world_changes(self):
        b = backend_server.Backend()
        b.world, spawned = object(), []
        b.cmd_spawn_ego = lambda *a: spawned.append(a)
        with self.assertRaises(RemoteCarSimError) as cm:
            b.cmd_cosim_start({"carsim": {"remote": True, "sim_path": "C:\\CarSim\\simfile.sim"}, "run": {"driver": "demo"}})
        self.assertEqual(str(cm.exception), NO_SERVICE)
        self.assertEqual(spawned, [])

    def test_command_line_refuses_before_carla(self):
        with tempfile.TemporaryDirectory() as tmp:
            conf = os.path.join(tmp, "remote.json")
            with open(conf, "w", encoding="utf-8") as f:
                json.dump({"carsim": {"remote": True, "sim_path": ""}, "run": {"driver": "demo"}}, f)
            err = io.StringIO()
            with mock.patch.object(run_cosim.carla, "Client") as client, \
                    mock.patch.object(sys, "argv", ["run_cosim.py", "--config", conf]), \
                    contextlib.redirect_stderr(err), self.assertRaises(SystemExit):
                run_cosim.main()
            client.assert_not_called()
            self.assertIn(NO_SERVICE, err.getvalue())  # not "--sim ... is required": the .sim is on Windows


class PreflightTests(LinkCase):
    def setUp(self):
        super().setUp()
        self.service_pc = self.fake(mock_answers(remote_cfg()))
        self.connected()

    def test_windows_paths_are_not_checked_here(self):
        with mock.patch.object(ses, "check_carsim", side_effect=AssertionError("checked on the server")):
            ses.check_run_files(remote_cfg())
            d = remote_cfg()
            d["carsim"]["mock"] = True  # the mock goes first: no service needed either
            ses.check_run_files(d)
        self.assertEqual([r["cmd"] for r in self.service_pc.requests], ["check"])  # the service checks them
        self.assertEqual(self.service_pc.requests[0]["carsim"]["repo_path"], "D:\\python_carsim_env")

    def test_backend_starts_the_run(self):
        started = []

        class Session:
            done = False

            def __init__(self, w, ego, anchor, d):
                self.d, self.scene = d, None

            def start(self):
                started.append(self.d["carsim"]["sim_path"])
                return {"external_api": True, "server_api": True, "reference_point": [0, 0, 0], "t_step": 0.001,
                        "inner_steps": 20, "frame_dt": 0.02, "t_stop": 0.0, "mock": False, "warnings": []}
        b = backend_server.Backend()
        b.world = oc.FakeWorld()
        b.cmd_spawn_ego = lambda *a: setattr(b, "ego", object())
        with mock.patch.object(backend_server, "CoSimSession", Session), \
                mock.patch.object(backend_server.rigmod, "spec_of", lambda v: None), \
                mock.patch.object(ses, "check_carsim", side_effect=AssertionError("checked on the server")):
            b.cmd_cosim_start({"carsim": {"remote": True, "sim_path": "C:\\CarSim\\simfile.sim",
                                          "repo_path": "D:\\python_carsim_env"}, "run": {"driver": "demo"}})
        self.assertEqual(started, ["C:\\CarSim\\simfile.sim"])
        self.assertEqual(b.cosim_state, "running")

    def test_run_json_names_the_sim_on_windows(self):
        with mock.patch.object(ses, "CarlaVehicleSync", oc.FakeSync), \
                mock.patch.object(ses, "start_scene", lambda s, *a, **k: {"collisions": []}):
            s = ses.CoSimSession(oc.FakeWorld(), oc.VEHICLE, None, remote_cfg())
            self.addCleanup(lambda: s.stop(release_vehicle=False))
            s.start()
            info = ses._run_json(s)
        self.assertEqual((info["carsim_sim"], info["carsim_mock"]), ("C:\\CarSim\\simfile.sim", False))

    def test_world_info_reports_the_service(self):
        b = backend_server.Backend()
        settings = SimpleNamespace(synchronous_mode=False, fixed_delta_seconds=None, no_rendering_mode=False)
        b.world = SimpleNamespace(get_settings=lambda: settings, get_map=lambda: SimpleNamespace(name="Carla/Maps/Town10HD_Opt"),
                                  get_weather=lambda: None, get_actors=lambda: [])
        b._weather_dict = lambda wp: {}
        self.assertEqual(b.cmd_world_info()["carsim_service"], {"connected": True, "host": "pc-1", "platform": "win32"})
        self.service_pc.close()
        wait(lambda: not self.link.status()["connected"])
        self.assertEqual(b.cmd_world_info()["carsim_service"], {"connected": False, "host": "", "platform": ""})


class DispatchTests(LinkCase):
    def test_make_env(self):
        d = remote_cfg()
        d["carsim"]["mock"] = True  # mock first, even with remote: no service
        self.assertIsInstance(ses.make_env(d), MockCarSimEnv)
        with self.assertRaises(RemoteCarSimError) as cm:  # remote: the service, not CarSim here
            ses.make_env(remote_cfg())
        self.assertEqual(str(cm.exception), NO_SERVICE)
        self.fake(mock_answers(remote_cfg()))
        self.connected()
        self.assertIsInstance(ses.make_env(remote_cfg()), carsim_remote.RemoteCarSimEnv)
        local = remote_cfg()
        local["carsim"]["remote"] = False
        with mock.patch.object(ses, "check_carsim", side_effect=LookupError("local check")):  # the tests' patch point
            with self.assertRaisesRegex(LookupError, "local check"):
                ses.make_env(local)


class SessionTests(LinkCase):
    """A whole run (fake CARLA) through the service equals the same run with the mock here."""

    def setUp(self):
        super().setUp()
        oc.FakeSync.handed = []
        for p in (mock.patch.object(ses, "CarlaVehicleSync", oc.FakeSync),
                  mock.patch.object(ses, "start_scene", lambda s, *a, **k: {"collisions": []}),
                  mock.patch.object(ses, "scene_step", lambda *a, **k: {})):
            p.start()
            self.addCleanup(p.stop)

    def run_once(self, d, frames=60):
        oc.FakeSync.handed = []
        s = ses.CoSimSession(oc.FakeWorld(), oc.VEHICLE, None, d)
        info = s.start()
        tel = [s.step() for _ in range(frames)]
        seen = list(sys.modules["user_controller"].seen)
        s.stop(release_vehicle=False, end="finished")
        return info, [(t["t"], t["action"]) for t in tel], seen, list(oc.FakeSync.handed)

    def test_same_values_as_the_mock_here(self):
        self.service("--mock")
        self.connected()
        with tempfile.TemporaryDirectory() as tmp:
            ctrl = os.path.join(tmp, "ctrl.py")
            with open(ctrl, "w") as f:
                f.write("seen = []\n\ndef control(exports, t, dt):\n    seen.append((t, dt, dict(exports)))\n"
                        "    return [0.4, 0.0, 120.0 * (t > 0.3)]\n")
            runs = []
            for remote in (False, True):
                d = remote_cfg(frame_dt=0.03, duration=0.0)
                d["carsim"].update(mock=not remote, remote=remote)
                d["run"].update(driver="custom", controller={"path": ctrl, "entry": "control"})
                runs.append(self.run_once(d))
        (info_a, tel_a, seen_a, pose_a), (info_b, tel_b, seen_b, pose_b) = runs
        self.assertEqual((info_a["mock"], info_b["mock"]), (True, False))  # to the backend a remote run is real CarSim
        self.assertEqual({k: info_a[k] for k in ("t_step", "inner_steps", "frame_dt")},
                         {k: info_b[k] for k in ("t_step", "inner_steps", "frame_dt")})
        self.assertEqual(seen_a, seen_b)  # what control() got: CarSim's names, units and values
        self.assertGreater(seen_b[-1][2]["Vx"], 1.0)
        self.assertNotEqual(seen_b[-1][2]["Yaw"], 0.0)
        self.assertEqual(tel_a, tel_b)
        self.assertEqual(pose_a, pose_b)  # what the car in CARLA was given


class DisconnectTests(LinkCase):
    def test_lost_service_ends_the_run_through_the_error_path(self):
        proc, console = self.service("--mock")
        self.connected()
        released = []

        class Sync(oc.FakeSync):
            def release(self):
                released.append(True)
        b = backend_server.Backend()
        b.world = oc.FakeWorld()
        logs = []
        b.emit = lambda m: logs.append(m) if m.get("event") in ("log", "cosim_state") else None
        self.link.on_change = b._carsim_service_changed
        with mock.patch.object(ses, "CarlaVehicleSync", Sync), \
                mock.patch.object(ses, "start_scene", lambda s, *a, **k: {"collisions": []}), \
                mock.patch.object(ses, "scene_step", lambda *a, **k: {}), \
                mock.patch.object(backend_server.traceback, "print_exc"):
            b.session = ses.CoSimSession(b.world, oc.VEHICLE, None, remote_cfg(duration=0.0))
            b.session.start()
            b.cosim_state = "running"
            b._cosim_frame()
            self.assertEqual((b.cosim_state, b.session.frame), ("running", 1))
            proc.kill()  # the service's window closed, or the network went
            wait(lambda: any(m.get("msg") == "Windows 上的 CarSim 服务断开了" for m in logs))
            b._cosim_frame()
        self.assertEqual(b.cosim_state, "error")
        self.assertTrue(b.cosim_detail.startswith(LOST), b.cosim_detail)
        self.assertIsNone(b.session)
        self.assertEqual(released, [True])  # the car let go: nothing of the run left in CARLA
        msgs = [m.get("msg", "") for m in logs if m["event"] == "log"]
        self.assertIn("Windows 上的 CarSim 服务断开了", msgs)
        self.assertTrue(any(m.startswith("仿真出错：" + LOST) for m in msgs), msgs)
        self.assertEqual(logs[-1], {"event": "cosim_state", "state": "error", "detail": b.cosim_detail})


class NullTests(LinkCase):
    def test_null_is_nan_and_the_nan_checks_stop_the_run(self):
        d = remote_cfg(duration=0.0)
        yaw = d["carsim"]["export_names"].index("Yaw")
        self.fake(mock_answers(d, obs=lambda o: [None if i == yaw else v for i, v in enumerate(o)]))
        self.connected()
        env = carsim_remote.RemoteCarSimEnv(d, self.link)
        env.reset()
        obs = env.control_step([0.0, 0.0, 0.0], 10)[0]
        self.assertTrue(math.isnan(obs[yaw]))
        with mock.patch.object(ses, "CarlaVehicleSync", oc.FakeSync), \
                mock.patch.object(ses, "start_scene", lambda s, *a, **k: {"collisions": []}):
            s = ses.CoSimSession(oc.FakeWorld(), oc.VEHICLE, None, d)
            s.start()
            with self.assertRaisesRegex(RuntimeError, "CarSim 输出了无效数值（NaN / 无穷大），仿真已停止：Yaw"):
                s.step()
            s.stop(release_vehicle=False, end="error")

    def test_non_finite_values_travel_as_null(self):
        with socket.socket() as srv:
            srv.bind(("127.0.0.1", 0))
            srv.listen(1)
            a = carsim_local.JsonLines(socket.create_connection(srv.getsockname()))
            b = carsim_local.JsonLines(srv.accept()[0])
            a.send({"obs": (float("nan"), float("inf"), 1.5), "t_current": float("-inf")})
            self.assertEqual(b.recv(5), {"obs": [None, None, 1.5], "t_current": None})
            a.sock.close()
            self.assertIsNone(b.recv(5))
            b.sock.close()


class ConnectionTests(LinkCase):
    def test_a_newer_connection_replaces_the_older(self):
        d = remote_cfg()
        old = self.fake(mock_answers(d), host="pc-1")
        self.connected("pc-1")
        env = carsim_remote.RemoteCarSimEnv(d, self.link)
        env.reset()
        new = self.fake(mock_answers(d), host="pc-2")
        self.assertEqual(new.hello, {"type": "hello", "ok": True, "protocol": carsim_local.SERVICE_PROTOCOL})
        self.connected("pc-2")
        self.assertTrue(old.closed.wait(5))  # the backend closed the older one
        with self.assertRaises(RemoteCarSimError) as cm:
            env.control_step([0.0, 0.0, 0.0], 10)  # its run is gone with it
        self.assertEqual(str(cm.exception), LOST + "Windows 上的服务重新连上了云端，这次运行的 CarSim 已经结束）")
        self.assertEqual(self.link.status(), {"connected": True, "host": "pc-2", "platform": "win32"})
        env2 = carsim_remote.RemoteCarSimEnv(d, self.link)
        self.assertEqual(len(env2.reset()), len(d["carsim"]["export_names"]))

    def test_a_replaced_service_stops_instead_of_taking_over_again(self):
        old, old_console = self.service("--mock")
        self.connected()
        new, _ = self.service("--mock")
        wait(lambda: old.poll() is not None)  # the older one gave up
        self.assertEqual(old.returncode, 0)
        self.assertIn("另一个 CarSim 服务", old_console())
        conn = self.link.conn
        time.sleep(3)  # longer than the retry interval: nobody takes over again
        self.assertIs(self.link.conn, conn)
        self.assertTrue(self.link.status()["connected"])
        self.assertIsNone(new.poll())

    def test_only_the_carsim_service_of_this_protocol(self):
        self.fake(host="pc-1")
        self.connected("pc-1")
        other = self.fake(host="pc-2", protocol=99)
        self.assertFalse(other.hello["ok"])
        self.assertIn("版本不一致", other.hello["error"])
        self.assertTrue(other.closed.wait(5))
        stranger = socket.create_connection(("127.0.0.1", self.port))  # e.g. something else on the port
        stranger.sendall(b'{"cmd": "hello"}\n')
        stranger.settimeout(5)
        self.assertEqual(stranger.recv(100), b"")  # closed without a word
        stranger.close()
        self.assertEqual(self.link.status()["host"], "pc-1")  # neither replaced the service

    def test_service_retries_quietly_reconnects_and_exits_on_ctrl_c(self):
        port = free_port()
        proc, console = self.service("--mock", port=port)
        time.sleep(4.5)  # two tries with nothing listening
        link = carsim_remote.ServiceLink()
        self.assertEqual(link.listen(port), port)
        wait(lambda: link.status()["connected"])
        link.conn.close("test")  # the backend went away (e.g. restarted)
        wait(lambda: "连接断开，正在重连…" in console())
        wait(lambda: link.status()["connected"])  # back by itself
        proc.send_signal(signal.SIGINT)
        self.assertEqual(proc.wait(10), 0)
        self.assertEqual(console().splitlines(), ["正在连接云端…", "已连上云端（127.0.0.1:%d），等待运行" % port,
                                                  "连接断开，正在重连…", "已连上云端（127.0.0.1:%d），等待运行" % port,
                                                  "已退出"])


class PingTests(LinkCase):
    """A network that died without a word: pings while idle, never during a run."""

    def setUp(self):
        for p in (mock.patch.object(carsim_remote, "PING", 0.1), mock.patch.dict(carsim_remote.TIMEOUTS, ping=0.5)):
            p.start()
            self.addCleanup(p.stop)
        super().setUp()

    def test_the_service_answers_pings(self):
        _, console = self.service("--mock")
        self.connected()
        self.assertEqual(self.link.need().request("ping"), {})
        sent = watch(self.link.need())
        time.sleep(1.0)
        self.assertGreaterEqual([cmd for _, cmd in sent].count("ping"), 4)
        self.assertTrue(self.link.status()["connected"])
        self.assertEqual(console().splitlines(), ["正在连接云端…", "已连上云端（127.0.0.1:%d），等待运行" % self.port])

    def test_a_silent_service_is_dropped(self):
        b = backend_server.Backend()
        logs = []
        b.emit = logs.append
        self.link.on_change = b._carsim_service_changed
        fake = self.fake()  # nothing comes back, and no FIN either
        self.connected()
        wait(lambda: not self.link.status()["connected"], 5)
        self.assertEqual([r["cmd"] for r in fake.requests], ["ping"])
        self.assertTrue(fake.closed.wait(5))  # the backend closed the socket
        self.assertIn({"event": "log", "level": "warn", "msg": "Windows 上的 CarSim 服务断开了"}, logs)
        with self.assertRaises(RemoteCarSimError) as cm:
            ses.check_run_files(remote_cfg())
        self.assertEqual(str(cm.exception), NO_SERVICE)

    def test_pings_never_interleave_with_a_run(self):
        d = remote_cfg()
        answers = mock_answers(d)

        def slow(req):
            if req["cmd"] == "step":
                time.sleep(0.25)  # longer than PING: the step is in flight when a ping would be due
            return answers(req)
        self.fake(slow)
        self.connected()
        sent = watch(self.link.need())
        env = carsim_remote.RemoteCarSimEnv(d, self.link)
        env.reset()
        start = time.monotonic()
        for _ in range(4):
            env.control_step([0.3, 0.0, 0.0], 10)
        end = time.monotonic()
        time.sleep(0.5)  # idle again
        pings = [t for t, cmd in sent if cmd == "ping"]
        self.assertEqual([t for t in pings if start <= t <= end], [])
        self.assertGreaterEqual(len([t for t in pings if t > end]), 2)
        self.assertEqual([cmd for _, cmd in sent if cmd != "ping"], ["open", "reset", "step", "step", "step", "step"])
        env.close()


class BackendCliTests(unittest.TestCase):
    def backend(self, *args):
        """backend_server.py on a free port (no CARLA needed): (process, port, GUI connection)."""
        port = free_port()
        log = tempfile.TemporaryFile()
        p = subprocess.Popen([sys.executable, os.path.join(BRIDGE, "backend_server.py"), "--port", str(port)] + list(args),
                             cwd=BRIDGE, stdout=log, stderr=subprocess.STDOUT)

        def stop():
            if p.poll() is None:
                p.terminate()
                p.wait(20)
            log.close()
        self.addCleanup(stop)
        c = Conn(port)
        self.addCleanup(c.s.close)
        return p, port, c

    @unittest.skipUnless(os.path.isdir("/proc/self/fd"), "needs Linux /proc")
    def test_no_carsim_port_by_default(self):
        # Only the remote session's backend listens for the service: not the desktop GUI's, not the tests'.
        p, port, _ = self.backend()
        self.assertEqual(listening(p.pid), {port})

    def test_carsim_port_option(self):
        carsim_port = free_port()
        p, port, c = self.backend("--carsim-port", str(carsim_port))
        if os.path.isdir("/proc/self/fd"):
            self.assertEqual(listening(p.pid), {port, carsim_port})
        service = subprocess.Popen([sys.executable, SERVICE, "--port", str(carsim_port), "--mock"], cwd=BRIDGE,
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.addCleanup(service.wait)
        self.addCleanup(service.kill)
        e = c.wait_event(lambda m: m.get("event") == "log" and "CarSim 服务" in m.get("msg", ""), 20)
        self.assertEqual(e["msg"], "Windows 上的 CarSim 服务已连上（%s）" % socket.gethostname())
        service.kill()
        e = c.wait_event(lambda m: m.get("event") == "log" and m.get("level") == "warn", 20)
        self.assertEqual(e["msg"], "Windows 上的 CarSim 服务断开了")


class StandaloneTests(unittest.TestCase):
    """What the Windows package ships (carsim_service.py, carsim_local.py, mock_carsim.py)."""

    def test_without_carla_or_numpy(self):
        code = ("import sys\nsys.modules['carla'] = sys.modules['numpy'] = None\nsys.path.insert(0, %r)\n"
                "import carsim_local, carsim_service\n"
                "env = carsim_local.make_env({'carsim': {'mock': True, 'export_names': ['Xo', 'Vx'], 'units': {}},"
                " 'sync': {'duration': 1.0}})\n"
                "env.reset()\nprint(env.control_step([1.0, 0.0, 0.0], 100)[0][1] > 0)\n"
                "print(sorted(m for m in ('bridge', 'session', 'carsim_remote', 'scene') if m in sys.modules))\n" % BRIDGE)
        r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=60)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(r.stdout.split(), ["True", "[]"])

    def test_python_38_syntax(self):
        for name in ("carsim_service.py", "carsim_local.py", "mock_carsim.py"):
            with open(os.path.join(BRIDGE, name), encoding="utf-8") as f:
                ast.parse(f.read(), name, feature_version=(3, 8))


if __name__ == "__main__":
    unittest.main()
