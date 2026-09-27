"""CarSim first-run checks that need no CARLA server: plain-language errors
for a wrong .sim / python_carsim_env / solver, the frame period aligned to
CarSim's t_step, the end reason, the t0 checks and the real-time factor.

    python tests/test_offline_carsim.py

The solver tests build a tiny fake CarSim solver (.so) with the C compiler and
run the real python_carsim_env against it (a copy in a temp dir; the original
is only read). They are skipped without a compiler or python_carsim_env.
"""

import contextlib
import copy
import io
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

import backend_server  # noqa: E402
import run_cosim  # noqa: E402
import session as ses  # noqa: E402
import settings as st  # noqa: E402
from bridge import REQUIRED_EXPORTS, CarSimExports  # noqa: E402
from mock_carsim import MockCarSimEnv  # noqa: E402

NAMES = list(REQUIRED_EXPORTS)  # Xo first

# A CarSim solver stand-in with the VS API python_carsim_env uses.
# "FAKE_<KEY> value" lines in the .sim (write_sim) set its run.
FAKE_SOLVER_C = r"""
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static int n_exp, err, mode;
static double t_stop, t_step, stop_at, x0;
static double param(const char* sim, const char* key, double d) {
  char line[512];
  size_t n = strlen(key);
  FILE* f = fopen(sim, "r");
  if (!f) return d;
  while (fgets(line, sizeof line, f))
    if (strncmp(line, key, n) == 0 && line[n] == ' ') { d = atof(line + n + 1); break; }
  fclose(f);
  return d;
}
int vs_run(const char* p) { return 0; }
void vs_initialize(double t, int a, int b) {}
void vs_read_configuration(const char* p, int* ni, int* ne, double* t0, double* t1, double* dt) {
  /* 1: no license (the solver says why), 2: no I/O, no message, 3: an error without a message at FAKE_STOP_AT */
  mode = (int)param(p, "FAKE_MODE", 0);
  err = mode == 1;
  n_exp = mode == 1 || mode == 2 ? 0 : (int)param(p, "FAKE_NEXP", 8);
  t_stop = param(p, "FAKE_TSTOP", 2.0);
  t_step = param(p, "FAKE_TSTEP", 0.001);
  stop_at = param(p, "FAKE_STOP_AT", -1.0);
  x0 = param(p, "FAKE_X0", 0.0);
  *ni = n_exp ? 3 : 0; *ne = n_exp; *t0 = 0.0; *t1 = t_stop; *dt = t_step;
}
static void fill(double* e) { for (int i = 0; i < n_exp; ++i) e[i] = 0.0; if (n_exp) e[0] = x0; }
int vs_integrate_io(double t, double* im, double* ex) {
  fill(ex);
  if (stop_at >= 0 && t + t_step >= stop_at - 1e-9) { err = mode == 3; return 1; }  /* a stop condition of the model */
  return t + t_step >= t_stop - 1e-9;                            /* TSTOP */
}
void vs_copy_export_vars(double* e) { fill(e); }
int vs_terminate_run(double t) { return 0; }
int vs_error_occurred(void) { return err; }
void vs_set_opt_error_dialog(int on) {}
const char* vs_get_error_message(void) { return err && mode == 1 ? "License not available (fake solver)" : ""; }
#ifndef NO_ROAD_L
double vs_road_l(double x, double y) { return 0.0; }
#endif
"""


def carsim_repo():
    """The user's python_carsim_env (only read), or None."""
    for p in (os.environ.get("PYTHON_CARSIM_ENV", ""), os.path.join(HERE, "..", "..", "python_carsim_env"),
              os.path.expanduser("~/carla_carsim/python_carsim_env")):
        if p and os.path.isfile(os.path.join(p, "carsim_env.py")):
            return os.path.abspath(p)
    return None


def copy_repo(dst):
    """carsim_env.py + vs_solver.py in dst, so importing them writes nothing next to the original."""
    src = carsim_repo()
    for f in ("carsim_env.py", "vs_solver.py"):
        shutil.copy(os.path.join(src, f), dst)
    return dst


def build_fake_solver(folder, product_ver="2020", defines=()):
    """libcarsim.so.<ver> in folder, as vs_solver.get_dll_path names it on Linux."""
    c = os.path.join(folder, "fake_vs.c")
    with open(c, "w") as f:
        f.write(FAKE_SOLVER_C)
    out = os.path.join(folder, "libcarsim.so.%s" % product_ver)
    subprocess.run(["cc", "-shared", "-fPIC", "-o", out, c] + ["-D" + d for d in defines], check=True)
    return out


def write_sim(path, progdir, **fake):
    """A .sim naming the solver in progdir; fake: FAKE_<KEY> settings of the fake solver."""
    extra = "".join("FAKE_%s %s\n" % (k.upper(), v) for k, v in fake.items())
    with open(path, "w") as f:
        f.write("SIMFILE\n\nPROGDIR %s\nPRODUCT_ID CarSim\nPRODUCT_VER 2020\nVEHICLE_CODE i_i\n%sEND\n" % (progdir, extra))
    return path


def can_build():
    return sys.platform == "linux" and shutil.which("cc") is not None and carsim_repo() is not None


def cfg(**over):
    base = {"carsim": {"mock": True, "export_names": NAMES}, "run": {"driver": "demo", "log_path": ""},
            "sync": {"frame_dt": 0.02, "duration": 0.0}}
    for k, v in over.items():
        base.setdefault(k, {}).update(v)
    return st.load_dict(None, base)


# ------------------------------------------------------------------ fakes
class FakeWorld:
    def __init__(self):
        self.settings = SimpleNamespace(synchronous_mode=False, fixed_delta_seconds=None)
        self.frame = 0

    def get_settings(self):
        return copy.copy(self.settings)

    def apply_settings(self, s):
        self.settings = copy.copy(s)

    def tick(self):
        self.frame += 1
        return self.frame

    def reset_all_traffic_lights(self):
        pass


class FakeSync:
    """CarlaVehicleSync without CARLA: keeps what it was handed."""
    handed = []

    def __init__(self, world, vehicle, anchor, use_external_api=None, settings=None):
        self.ex = CarSimExports(settings.EXPORT_NAMES, settings.UNITS)
        self.wheel_radius_m = [0.35] * 4
        self.ref_local = [0.0, 0.0, 0.0]
        self.external_api, self.server_api = True, True

    def sync(self, obs, t, dt):
        FakeSync.handed.append((list(obs), t, dt))
        return SimpleNamespace(velocity=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                               wheel_steer=[0.0] * 4, wheel_rotation=[0.0] * 4, wheel_suspension=[])

    def release(self):
        pass


VEHICLE = SimpleNamespace(get_transform=lambda: SimpleNamespace(
    location=SimpleNamespace(x=0.0, y=0.0, z=0.0), rotation=SimpleNamespace(pitch=0.0, yaw=0.0, roll=0.0)),
    bounding_box=SimpleNamespace(location=SimpleNamespace(z=0.7), extent=SimpleNamespace(z=0.7)))


class SessionCase(unittest.TestCase):
    """CoSimSession on a fake world: CarSim (mock or fake solver) is real, CARLA is not."""

    def setUp(self):
        FakeSync.handed = []
        for p in (mock.patch.object(ses, "CarlaVehicleSync", FakeSync),
                  mock.patch.object(ses, "start_scene", lambda s, *a, **k: {"collisions": []}),
                  mock.patch.object(ses, "scene_step", lambda *a, **k: {})):
            p.start()
            self.addCleanup(p.stop)

    def session(self, d, env=None):
        s = ses.CoSimSession(FakeWorld(), VEHICLE, None, d)
        if env is not None:
            p = mock.patch.object(ses, "make_env", lambda d: env)
            p.start()
            self.addCleanup(p.stop)
        self.addCleanup(lambda: s.stop(release_vehicle=False))
        return s


# ------------------------------------------------------------------ tests
class FramePeriodTests(SessionCase):
    def test_frame_dt_aligned_to_t_step_everywhere(self):
        with tempfile.TemporaryDirectory() as tmp:
            ctrl = os.path.join(tmp, "dt_ctrl.py")
            with open(ctrl, "w") as f:
                f.write("seen = []\ndef control(exports, t, dt):\n    seen.append(dt)\n    return [0.0, 0.0, 0.0]\n")
            d = cfg(sync={"frame_dt": 0.0333, "duration": 1.0},
                    run={"driver": "custom", "controller": {"path": ctrl, "entry": "control"}})
            s = self.session(d)
            info = s.start()  # mock CarSim: t_step 0.001
            self.assertEqual((info["inner_steps"], info["frame_dt"]), (33, 0.033))
            self.assertEqual(s.world.settings.fixed_delta_seconds, 0.033)
            self.assertEqual(d["sync"]["frame_dt"], 0.033)  # what scene / records / sampling read
            self.assertEqual(s.n_frames, 30)
            tel = s.step()
            self.assertAlmostEqual(tel["t"], 0.033, places=9)  # CarSim moved exactly one CARLA frame
            self.assertEqual(sys.modules["user_controller"].seen, [0.033])
            self.assertEqual(FakeSync.handed[-1][2], 0.033)

    def test_multiple_stays_as_set(self):
        s = self.session(cfg(sync={"frame_dt": 0.02}))
        info = s.start()
        self.assertEqual((info["inner_steps"], info["frame_dt"]), (20, 0.02))

    def test_frame_shorter_than_t_step_becomes_one_step(self):
        s = self.session(cfg(sync={"frame_dt": 0.0004}))
        self.assertEqual(s.start()["frame_dt"], 0.001)

    def test_long_frame_capped_at_carla_limit(self):
        info = self.session(cfg(sync={"frame_dt": 0.2})).start()  # run_cosim / backend refuse it before this
        self.assertEqual((info["inner_steps"], info["frame_dt"]), (100, 0.1))

    def test_aligned_frame_stays_within_carla_limit(self):
        s = self.session(cfg(sync={"frame_dt": 0.1}), MockCarSimEnv(NAMES, t_step=0.0007, t_stop=1e9))
        info = s.start()
        self.assertEqual(info["inner_steps"], 142)
        self.assertLessEqual(info["frame_dt"], ses.MAX_FRAME_DT)


class StartChecksTests(SessionCase):
    def test_nan_at_t0_never_reaches_carla(self):
        class NanStart(MockCarSimEnv):
            def reset(self):
                obs = list(super().reset())
                obs[0] = float("nan")
                return tuple(obs)
        s = self.session(cfg(), NanStart(NAMES))
        with self.assertRaisesRegex(RuntimeError, "初始状态.*Xo"):
            s.start()
        self.assertEqual(FakeSync.handed, [])

    def test_warns_when_carsim_starts_off_origin(self):
        class Station(MockCarSimEnv):
            def reset(self):
                super().reset()
                self.x, self.y = 150.0, -3.5
                return self._exports()
        self.assertEqual(self.session(cfg()).start()["warnings"], [])
        info = self.session(cfg(), Station(NAMES)).start()
        self.assertEqual(len(info["warnings"]), 1)
        self.assertIn("不在原点", info["warnings"][0])
        self.assertIn("离出生点 150.0 m", info["warnings"][0])
        self.assertNotIn("方向差", info["warnings"][0])

    def test_heading_only_warning_names_the_heading(self):
        class Turned(MockCarSimEnv):
            def reset(self):
                super().reset()
                self.psi = math.radians(90.0)
                return self._exports()
        info = self.session(cfg(), Turned(NAMES)).start()
        self.assertEqual(len(info["warnings"]), 1)
        self.assertIn("车头方向与出生点方向差 90°", info["warnings"][0])
        self.assertNotIn("离出生点", info["warnings"][0])

    def test_left_over_solver_error_does_not_refuse_a_good_start(self):
        class Stale(MockCarSimEnv):
            solver = SimpleNamespace(dll_handle=SimpleNamespace(vs_error_occurred=lambda: 1,
                                                                vs_get_error_message=lambda: b"old error"))
        self.assertEqual(self.session(cfg(), Stale(NAMES)).start()["warnings"], [])

    def test_start_reports_t_stop_and_mock(self):
        info = self.session(cfg(sync={"duration": 5.0})).start()
        self.assertTrue(info["mock"])
        self.assertEqual(info["t_stop"], 6.0)


class RealTimeFactorTests(SessionCase):
    def test_wall_clock_skips_pauses(self):
        now = [100.0]
        with mock.patch.object(ses.time, "perf_counter", lambda: now[0]):
            c = ses.WallClock()
            now[0] = 102.0
            c.pause()
            now[0] = 150.0
            self.assertEqual(c.elapsed(), 2.0)
            c.resume()
            c.resume()
            now[0] = 151.0
            self.assertEqual(c.elapsed(), 3.0)

    def test_rt_factor_from_t_start_without_pauses(self):
        class LateStart(MockCarSimEnv):
            def reset(self):
                obs = super().reset()
                self.t_current = 5.0  # a .sim with t_start = 5 s
                return obs
        now = [100.0]
        with mock.patch.object(ses.time, "perf_counter", lambda: now[0]):
            s = self.session(cfg(), LateStart(NAMES, t_stop=1e9))
            s.start()
            now[0] = 100.02
            self.assertAlmostEqual(s.step()["rt_factor"], 1.0, places=6)
            s.clock.pause()
            now[0] = 110.0
            s.clock.resume()
            now[0] = 110.02
            self.assertAlmostEqual(s.step()["rt_factor"], 1.0, places=6)


class BackendTests(unittest.TestCase):
    def backend(self):
        b = backend_server.Backend()
        b.world = object()
        b.spawned = []
        b.cmd_spawn_ego = lambda *a: b.spawned.append(a)
        return b

    def test_bad_config_refused_before_world_changes(self):
        with tempfile.TemporaryDirectory() as tmp:
            par = os.path.join(tmp, "Run_all.par")
            with open(par, "w") as f:
                f.write("PARSFILE\nOPT_ERROR_DIALOG 0\nEND\n")
            for over, text in (
                    ({"sync": {"frame_dt": 0.5}}, "仿真步长 0.5 s 超出范围"),
                    ({"sync": {"frame_dt": 0, "duration": 5}}, "仿真步长 0 s 超出范围"),
                    ({"sync": {"frame_dt": "abc"}}, "仿真步长不是数字"),
                    ({"run": {"driver": "pid"}}, "未知的驾驶方式 'pid'"),
                    ({"drive": {"dynamics": "carla", "carla_driver": "pid"}}, "未知的驾驶方式 'pid'"),
                    ({"drive": {"dynamics": "physx"}}, "未知的动力学"),
                    ({"carsim": {"export_names": ["Xo", "Yo"]}}, "缺少必需的 Zo"),
                    ({"run": {"driver": "custom", "controller": {"path": os.path.join(tmp, "no.py")}}}, "控制算法文件不存在"),
                    ({"carsim": {"mock": False, "sim_path": ""}}, "没有设置 CarSim .sim 文件"),
                    ({"carsim": {"mock": False, "sim_path": os.path.join(tmp, "no.sim")}}, "CarSim .sim 文件不存在"),
                    ({"carsim": {"mock": False, "sim_path": par}}, "不是 CarSim 生成的 .sim"),
                    ({"carsim": {"mock": False, "sim_path": write_sim(os.path.join(tmp, "a.sim"), tmp),
                                 "repo_path": tmp}}, "python_carsim_env 目录不对")):
                with self.subTest(text=text):
                    b = self.backend()
                    base = {"carsim": {"mock": True, "export_names": NAMES}, "run": {"driver": "demo"}}
                    for k, v in over.items():
                        base.setdefault(k, {}).update(v)
                    with self.assertRaisesRegex(ValueError, text):
                        b.cmd_cosim_start(base)
                    self.assertEqual(b.spawned, [])

    def test_pause_and_step_count_running_time_only(self):
        b = self.backend()
        b.session = SimpleNamespace(clock=ses.WallClock())
        b.cosim_state = "running"
        b.cmd_cosim_pause()
        self.assertIsNotNone(b.session.clock._paused_at)
        during = []
        b._cosim_frame = lambda always_emit=False: during.append(b.session.clock._paused_at)
        b.cmd_cosim_step()
        self.assertEqual(during, [None])  # the step's own time counts
        self.assertIsNotNone(b.session.clock._paused_at)
        b.cmd_cosim_resume()
        self.assertIsNone(b.session.clock._paused_at)

    def test_start_log_names_mock_alignment_t_stop_and_warnings(self):
        class World(FakeWorld):
            pass

        class Session:
            def __init__(self, w, ego, anchor, d):
                self.d, self.scene = d, None

            def start(self):
                self.d["sync"]["frame_dt"] = 0.033
                return {"external_api": True, "server_api": True, "reference_point": [0, 0, 0], "t_step": 0.001,
                        "inner_steps": 33, "frame_dt": 0.033, "t_stop": 12.0, "mock": self.d["carsim"]["mock"],
                        "warnings": ["CarSim 的初始位置不在原点"]}

        for mock_on in (True, False):
            b = self.backend()
            b.world = World()
            logs = []
            b.emit = lambda m: logs.append(m) if m.get("event") == "log" else None
            b.cmd_spawn_ego = lambda *a: setattr(b, "ego", object())
            with mock.patch.object(backend_server, "CoSimSession", Session), \
                    mock.patch.object(backend_server, "check_run_files", lambda d: None), \
                    mock.patch.object(backend_server.rigmod, "spec_of", lambda v: None):
                b.cmd_cosim_start({"carsim": {"mock": mock_on, "export_names": NAMES}, "run": {"driver": "demo"},
                                   "sync": {"frame_dt": 0.0333}})
            text = "\n".join(m["msg"] for m in logs)
            self.assertEqual("联合仿真开始（模拟 CarSim）" in text, mock_on)
            self.assertIn("仿真步长已对齐到 CarSim t_step（0.001 s）的整数倍：0.0333 s → 0.033 s", text)
            self.assertEqual("结束时间 t = 12.0 s" in text, not mock_on)
            self.assertTrue(any(m["level"] == "warn" and "不在原点" in m["msg"] for m in logs))


class CliTests(unittest.TestCase):
    def test_bad_config_refused_before_carla(self):
        with tempfile.TemporaryDirectory() as tmp:
            conf = os.path.join(tmp, "carla_dyn.json")
            with open(conf, "w") as f:
                json.dump({"drive": {"dynamics": "carla"}, "run": {"driver": "pid"}}, f)
            for argv, text in ((["--mock", "--frame-dt", "0.2"], "仿真步长 0.2 s 超出范围"),
                               (["--mock", "--frame-dt", "0"], "仿真步长 0 s 超出范围"),
                               (["--mock", "--controller", os.path.join(tmp, "no.py")], "控制算法文件不存在"),
                               (["--sim", os.path.join(tmp, "no.sim"), "--driver", "demo"], "CarSim .sim 文件不存在"),
                               (["--mock", "--config", conf], "未知的驾驶方式 'pid'")):  # the CLI always runs CarSim
                with self.subTest(text=text):
                    err = io.StringIO()
                    with mock.patch.object(run_cosim.carla, "Client") as client, \
                            mock.patch.object(sys, "argv", ["run_cosim.py"] + argv), \
                            contextlib.redirect_stderr(err), self.assertRaises(SystemExit):
                        run_cosim.main()
                    client.assert_not_called()
                    self.assertIn(text, err.getvalue())


@unittest.skipUnless(can_build(), "needs Linux, a C compiler and python_carsim_env")
class FakeSolverTests(SessionCase):
    """The real python_carsim_env on a fake solver: what the user reads when CarSim cannot start or stops."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp(prefix="cc_carsim_")
        cls.repo = os.path.join(cls.tmp, "repo")
        os.makedirs(cls.repo)
        copy_repo(cls.repo)
        cls.prog = os.path.join(cls.tmp, "prog")
        os.makedirs(cls.prog)
        build_fake_solver(cls.prog)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp, ignore_errors=True)

    def real(self, **fake):
        """A run config on the fake solver; fake: its FAKE_<KEY> settings."""
        sim = write_sim(os.path.join(tempfile.mkdtemp(dir=self.tmp), "simfile.sim"), self.prog, nexp=len(NAMES), **fake)
        return cfg(carsim={"mock": False, "sim_path": sim, "repo_path": self.repo})

    def test_missing_solver_with_relative_progdir(self):
        with tempfile.TemporaryDirectory() as tmp:
            d = self.real()
            d["carsim"]["sim_path"] = write_sim(os.path.join(tmp, "rel.sim"), ".")
            with self.assertRaisesRegex(ValueError, "找不到 CarSim 求解器.*相对路径 \\."):
                ses.check_run_files(d)

    def test_loader_reason_is_kept(self):
        with tempfile.TemporaryDirectory() as tmp:
            with open(os.path.join(tmp, "libcarsim.so.2020"), "w") as f:
                f.write("not a shared library\n")
            d = self.real()
            d["carsim"]["sim_path"] = write_sim(os.path.join(tmp, "bad.sim"), tmp)
            with self.assertRaises(RuntimeError) as cm:
                ses.make_env(d)
            msg = str(cm.exception)
            self.assertIn("无法加载 CarSim 求解器", msg)
            self.assertNotIn("WinDLL", msg)
            self.assertTrue("ELF" in msg or "too short" in msg, msg)

    def test_missing_solver_function_is_named(self):
        with tempfile.TemporaryDirectory() as tmp:
            build_fake_solver(tmp, defines=["NO_ROAD_L"])
            d = self.real()
            d["carsim"]["sim_path"] = write_sim(os.path.join(tmp, "old.sim"), tmp)
            with self.assertRaisesRegex(RuntimeError, "缺少 python_carsim_env 要用的函数：vs_road_l"):
                ses.check_run_files(d)

    def test_solver_message_when_the_run_cannot_start(self):
        with self.assertRaisesRegex(RuntimeError, "CarSim 没能开始这次运行：License not available \\(fake solver\\)"):
            self.session(self.real(mode=1)).start()

    def test_no_io_variables_in_plain_words(self):
        with self.assertRaisesRegex(RuntimeError, "没有读出导入 / 导出变量"):
            self.session(self.real(mode=2)).start()

    def run_to_end(self, s, limit=500):
        for _ in range(limit):
            tel = s.step()
            if tel["done"]:
                return tel
        self.fail("run did not end")

    def test_model_stop_is_not_reported_as_t_stop(self):
        s = self.session(self.real(tstop=1.0, stop_at=0.1))
        info = s.start()
        self.assertEqual((info["t_stop"], info["mock"]), (1.0, False))
        tel = self.run_to_end(s)
        self.assertAlmostEqual(tel["t"], 0.1, places=6)
        self.assertIn("CarSim 模型请求停止", s.end_reason)
        self.assertIn("t = 0.10 s", s.end_reason)

    def test_reaching_t_stop_is_a_normal_end(self):
        s = self.session(self.real(tstop=0.1))
        s.start()
        tel = self.run_to_end(s)
        self.assertAlmostEqual(tel["t"], 0.1, places=6)
        self.assertEqual(s.end_reason, "")

    def test_solver_error_without_message(self):
        s = self.session(self.real(mode=3, tstop=1.0, stop_at=0.1))
        s.start()
        with self.assertRaisesRegex(RuntimeError, "CarSim 报错（t = 0.10 s）：求解器没有给出错误信息"):
            self.run_to_end(s)

    def test_off_origin_start_from_the_solver(self):
        self.assertIn("不在原点", self.session(self.real(x0=150)).start()["warnings"][0])


if __name__ == "__main__":
    unittest.main()
