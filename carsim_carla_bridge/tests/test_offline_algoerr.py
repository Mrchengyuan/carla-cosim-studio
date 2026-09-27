"""Error messages and reloading of the user's control algorithm; start and
worker failures that must tell the GUI why. Needs no CARLA server."""

import os
import shutil
import sys
import tempfile
import threading
import time
import types
import unittest
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.dont_write_bytecode = True  # files rewritten within the same second must not come from a stale .pyc

import backend_server  # noqa: E402
from bridge import CarSimExports  # noqa: E402
import session  # noqa: E402
import settings as st  # noqa: E402
from backend_server import Backend  # noqa: E402


class Ex:
    """The bridge's export table: name -> index into the CarSim output."""

    def __init__(self, names):
        self.index = {n: i for i, n in enumerate(names)}

    def raw(self, obs, n):
        return obs[self.index[n]]


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="cc_algoerr_")
        self.sys_path = list(sys.path)
        session._last_folder = None

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

    def load(self, rel, entry="Controller", scene=None, scene_sel=None, rig=None):
        d = st.load_dict(None, {})
        d["run"]["controller"] = {"path": os.path.join(self.tmp, rel), "entry": entry}
        d["carsim"]["repo_path"] = self.tmp
        if scene_sel is not None:
            d["scene"].update(scene_sel)
        if rig is not None:
            d["rig"]["sensors"] = rig
        return session.load_controller(d, Ex(["Xo", "Vx"]), None, (lambda: scene) if scene is not None else None)

    def error(self, ctl):
        with self.assertRaises(RuntimeError) as cm:
            ctl([0.0, 36.0], 0.0)
        return str(cm.exception)

    # ------------------------------------------------------------ A5: KeyError
    def test_unticked_scene_key_points_to_the_scene_page(self):
        self.write("a/c.py", "class Controller:\n    def control(self, exports, t, dt, scene):\n"
                             "        o = scene['objects'][0]\n        return [0.0, 0.0, o['gap']]\n")
        ctl = self.load("a/c.py", scene={"objects": [{"id": 1, "type": "vehicle", "rel_x": 5.0}], "ego": {}},
                        scene_sel={"objects": ["id", "type", "rel_x"]})
        msg = self.error(ctl)
        self.assertIn("KeyError: 'gap'（c.py 第 4 行）", msg)
        self.assertIn("场景里没有 'gap'", msg)
        self.assertIn("场景信息", msg)
        self.assertNotIn("CarSim 动力学", msg)

    def test_unticked_ego_key_that_could_be_an_export_names_both_pages(self):
        self.write("a/c.py", "def control(exports, t, dt, scene):\n    return [scene['ego']['Speed'], 0.0, 0.0]\n")
        ctl = self.load("a/c.py", "control", scene={"objects": [], "ego": {"X": 1.0}}, scene_sel={"ego": ["X"]})
        msg = self.error(ctl)
        self.assertIn("场景里没有 'Speed'", msg)
        self.assertIn("导出变量里没有 'Speed'", msg)

    def test_no_lane_selected(self):
        self.write("a/c.py", "def control(exports, t, dt, scene):\n    return [scene['lane']['offset'], 0.0, 0.0]\n")
        ctl = self.load("a/c.py", "control", scene={"objects": [], "ego": {}}, scene_sel={"lane": []})
        self.assertIn("场景里没有 'lane'：“场景信息”页车道一栏没有勾选", self.error(ctl))

    def test_missing_sensor_data_points_to_the_scene_and_rig_pages(self):
        self.write("a/c.py", "def control(exports, t, dt, scene):\n    return [scene['sensors']['cam']['data'], 0, 0]\n")
        rig = [{"name": "cam", "type": "camera_rgb", "enabled": False}, {"name": "lidar", "type": "lidar"}]
        lidar = {"lidar": {"type": "lidar", "data": None}}

        def msg(ticked, sensors=None):
            scene = {"objects": [], "ego": {}}
            if sensors is not None:
                scene["sensors"] = sensors
            m = self.error(self.load("a/c.py", "control", scene=scene, scene_sel={"sensors": ticked}, rig=rig))
            self.assertNotIn("CarSim 动力学", m)
            return m

        # ticked, but disabled in the rig: the scene has no 'sensors' at all
        self.assertIn("场景里没有 'sensors'：“场景信息”页勾选给算法的传感器（cam）在“传感器套件”里没有启用", msg(["cam"]))
        self.assertIn("场景里没有 'sensors'：“场景信息”页没有勾选给算法的传感器", msg([]))
        # only another sensor ticked: the scene has just that one
        self.assertIn("场景里没有 'cam'：没有在“场景信息”页给这个传感器勾选“给算法”", msg(["lidar"], lidar))
        # ticked together with another one, but disabled in the rig
        self.assertIn("场景里没有 'cam'：这个传感器在“传感器套件”里没有启用", msg(["cam", "lidar"], lidar))

    def test_missing_export_still_points_to_the_carsim_page(self):
        self.write("a/c.py", "def control(exports, t, dt):\n    return [exports['LatErr'], 0.0, 0.0]\n")
        msg = self.error(self.load("a/c.py", "control"))
        self.assertIn("导出变量里没有 'LatErr'", msg)
        self.assertIn("CarSim 动力学", msg)
        self.assertNotIn("场景", msg)

    def test_other_key_errors_get_no_hint(self):
        self.write("a/c.py", "def control(exports, t, dt):\n    return [{}[0], 0.0, 0.0]\n")
        msg = self.error(self.load("a/c.py", "control"))
        self.assertIn("KeyError: 0（c.py 第 2 行）", msg)
        self.assertNotIn("导出变量里没有", msg)
        self.write("b/c.py", "def control(exports, t, dt, scene):\n    return [exports['Vx'] * 0 + {'a': 1}['Vx'], 0, 0]\n")
        msg = self.error(self.load("b/c.py", "control", scene={"objects": [], "ego": {}}))
        self.assertNotIn("导出变量里没有", msg)  # Vx is an export: that lookup did not fail
        self.assertNotIn("场景里没有", msg)

    # ------------------------------------------------- A6 / G13: reloading
    def test_helper_in_a_sub_package_is_reloaded(self):
        self.write("algo/ctrl.py", "from mpc.solver import gain\n\ndef control(e, t, dt):\n    return [gain(), 0.0, 0.0]\n")
        self.write("algo/mpc/__init__.py", "")
        self.write("algo/mpc/solver.py", "def gain():\n    return 0.1\n")
        self.assertEqual(self.load("algo/ctrl.py", "control")([0.0, 0.0], 0.0)[0], 0.1)
        self.write("algo/mpc/solver.py", "def gain():\n    return 0.25  # edited\n")
        self.assertEqual(self.load("algo/ctrl.py", "control")([0.0, 0.0], 0.0)[0], 0.25)

    def test_same_helper_name_in_two_folders(self):
        for folder, v in (("a", 1.0), ("b", 2.0)):
            self.write(folder + "/ctrl.py", "import utils\n\ndef control(e, t, dt):\n    return [utils.V, 0.0, 0.0]\n")
            self.write(folder + "/utils.py", "V = %r\n" % v)
        a, b = os.path.join(self.tmp, "a"), os.path.join(self.tmp, "b")
        self.assertEqual(self.load("a/ctrl.py", "control")([0.0, 0.0], 0.0)[0], 1.0)
        self.assertEqual(self.load("b/ctrl.py", "control")([0.0, 0.0], 0.0)[0], 2.0)
        self.assertEqual(sys.path[0], b)
        self.assertNotIn(a, sys.path)
        self.assertEqual(self.load("a/ctrl.py", "control")([0.0, 0.0], 0.0)[0], 1.0)

    def test_purge_keeps_libraries_and_the_backend_modules(self):
        self.write("x/ctrl.py", "def control(e, t, dt):\n    return [0.0, 0.0, 0.0]\n")
        lib = types.ModuleType("cc_algoerr_lib")
        lib.__file__ = self.write("x/venv/lib/python3.10/site-packages/cc_algoerr_lib.py", "")
        helper = types.ModuleType("cc_algoerr_helper")
        helper.__file__ = self.write("x/sub/cc_algoerr_helper.py", "")
        sys.modules.update(cc_algoerr_lib=lib, cc_algoerr_helper=helper)
        try:
            self.load("x/ctrl.py", "control")
            self.assertIs(sys.modules.get("cc_algoerr_lib"), lib)
            self.assertNotIn("cc_algoerr_helper", sys.modules)
        finally:
            sys.modules.pop("cc_algoerr_lib", None)
        root = os.path.dirname(session._HERE)  # the algorithm at the repo root
        self.assertFalse(session._own_file(os.path.join(session._HERE, "config.py"), root))
        self.assertTrue(session._own_file(os.path.join(session._HERE, "controllers", "my.py"), root))
        self.assertFalse(session._own_file(os.__file__, os.path.dirname(os.path.dirname(os.__file__))))

    def test_name_clash_with_a_loaded_module(self):
        import config  # noqa: F401  (the backend's own, as in the backend)
        self.write("c/ctrl.py", "import config\n\ndef control(e, t, dt):\n    return [config.KP, 0.0, 0.0]\n")
        self.write("c/config.py", "KP = 0.5\n")
        with self.assertRaisesRegex(RuntimeError, "你的 config.py 和后端已经加载的同名模块.*改个名字"):
            self.load("c/ctrl.py", "control")
        # imported inside a function of a helper module: found too
        self.write("e/ctrl.py", "import helper\n\ndef control(e, t, dt):\n    return helper.go()\n")
        self.write("e/helper.py", "def go():\n    import settings\n    return [0.0, 0.0, 0.0]\n")
        self.write("e/settings.py", "")
        with self.assertRaisesRegex(RuntimeError, "settings.py"):
            self.load("e/ctrl.py", "control")
        # a file of that name that the algorithm does not import is fine
        self.write("d/ctrl.py", "def control(e, t, dt):\n    return [0.1, 0.0, 0.0]\n")
        self.write("d/config.py", "KP = 1\n")
        self.assertEqual(self.load("d/ctrl.py", "control")([0.0, 0.0], 0.0)[0], 0.1)
        # ... also when another script lying in the folder imports it
        self.write("d/train.py", "import config\n")
        self.assertEqual(self.load("d/ctrl.py", "control")([0.0, 0.0], 0.0)[0], 0.1)
        # reached through a package and its relative import: found
        self.write("g/ctrl.py", "from pkg import run\n\ndef control(e, t, dt):\n    return run()\n")
        self.write("g/pkg/__init__.py", "from .inner import run\n")
        self.write("g/pkg/inner.py", "def run():\n    import config\n    return [config.KP, 0.0, 0.0]\n")
        self.write("g/config.py", "KP = 1\n")
        with self.assertRaisesRegex(RuntimeError, "你的 config.py"):
            self.load("g/ctrl.py", "control")

    def test_algorithm_folder_stays_before_python_carsim_env(self):
        """make_env puts python_carsim_env first; an import inside control()
        must still get the algorithm's own file of that name."""
        repo = os.path.dirname(self.write("repo/carsim_env.py", (
            "class CarSimEnv:\n    def __init__(self, sim):\n"
            "        self.config, self.t_current = {'n_import': 3, 't_step': 0.01}, 0.0\n\n"
            "    def reset(self):\n        return [0.0] * 64\n")))
        self.write("repo/helper.py", "V = 2.0\n")
        self.write("algo/helper.py", "V = 1.0\n")
        path = self.write("algo/ctrl.py", "def control(e, t, dt):\n    import helper\n    return [helper.V, 0.0, 0.0]\n")
        d = st.load_dict(None, {})
        d["run"].update(driver="custom", controller={"path": path, "entry": "control"}, record_dir="")
        d["carsim"].update(mock=False, sim_path=os.path.join(repo, "x.sim"), repo_path=repo)
        settings = SimpleNamespace(synchronous_mode=False, fixed_delta_seconds=None)
        world = SimpleNamespace(get_settings=lambda: settings, apply_settings=lambda s: None, tick=lambda: 1)
        bfg = st.to_bridge_cfg(d)
        sync = SimpleNamespace(ex=CarSimExports(bfg.EXPORT_NAMES, bfg.UNITS), wheel_radius_m=[0.35] * 4,
                               sync=lambda *a: SimpleNamespace(velocity=None),
                               ref_local=[0.0, 0.0, 0.0], external_api=False, server_api=None)
        box = SimpleNamespace(location=SimpleNamespace(z=0.7), extent=SimpleNamespace(z=0.7))
        ses = session.CoSimSession(world, SimpleNamespace(bounding_box=box), None, d)
        with mock.patch.object(session, "CarlaVehicleSync", lambda *a, **k: sync), \
                mock.patch.object(session, "check_carsim",
                                  lambda d: (d["carsim"]["sim_path"], session._carsim_module(d["carsim"]["repo_path"]))), \
                mock.patch.object(session, "start_scene", lambda *a, **k: {"collisions": []}):
            ses.start()
        self.assertEqual(sys.path[0], os.path.dirname(path))
        self.assertIn(repo, sys.path)
        self.assertEqual(ses.driver(ses.obs, 0.0)[0], 1.0)

    # ------------------------------------------------------- A17: SystemExit
    def test_sys_exit_in_control(self):
        self.write("a/c.py", "import sys\n\ndef control(e, t, dt):\n    sys.exit(3)\n")
        msg = self.error(self.load("a/c.py", "control"))
        self.assertIn("sys.exit(3)", msg)
        self.assertIn("（c.py 第 4 行）", msg)

    # --------------------------------------------- A18: errors in helper files
    def test_error_in_a_helper_names_both_lines(self):
        self.write("h/mpc.py", "def solve(x):\n    return 1.0 / x\n")
        self.write("h/my_ctrl.py", "import mpc\n\n\ndef control(e, t, dt):\n    return [mpc.solve(0.0), 0.0, 0.0]\n")
        msg = self.error(self.load("h/my_ctrl.py", "control"))
        self.assertIn("ZeroDivisionError", msg)
        self.assertIn("（mpc.py 第 2 行，由 my_ctrl.py 第 5 行调用）", msg)

    def test_error_in_a_helper_at_load_time(self):
        self.write("h/pkg/__init__.py", "")
        self.write("h/pkg/tool.py", "X = 1\nY = undefined_name\n")
        self.write("h/my_ctrl.py", "from pkg import tool\n\ndef control(e, t, dt):\n    return [0.0, 0.0, 0.0]\n")
        with self.assertRaises(RuntimeError) as cm:
            self.load("h/my_ctrl.py", "control")
        self.assertIn("NameError", str(cm.exception))
        self.assertIn("（%s 第 2 行，由 my_ctrl.py 第 1 行调用）" % os.path.join("pkg", "tool.py"), str(cm.exception))

    # ---------------------------------------------- A19: entry not found
    def test_entry_not_found_lists_candidates(self):
        self.write("a/c.py", "def control(e, t, dt):\n    return [0.0, 0.0, 0.0]\n\n\nclass MyCtrl:\n"
                             "    def control(self, e, t, dt):\n        return [0.0, 0.0, 0.0]\n\n\nclass Helper:\n    pass\n")
        with self.assertRaises(ValueError) as cm:
            self.load("a/c.py")
        msg = str(cm.exception)
        self.assertIn("c.py 里没有找到 Controller", msg)
        self.assertIn("control（函数）", msg)
        self.assertIn("MyCtrl（类）", msg)
        self.assertNotIn("Helper", msg)
        self.write("b/c.py", "X = 1\n")
        with self.assertRaisesRegex(ValueError, "“入口”要填"):
            self.load("b/c.py")

    # --------------------------------------- B5: the heartbeat names control()
    def test_busy_heartbeat_points_at_the_user_line(self):
        self.write("s/slow.py", "import threading\nGO = threading.Event()\n\n\ndef control(e, t, dt):\n"
                                "    GO.wait(10)\n    return [0.0, 0.0, 0.0]\n")
        ctl = self.load("s/slow.py", "control")
        self.assertIsNone(session.control_busy(ctl))
        self.assertIsNone(session.control_busy(None))
        self.assertIsNone(session.control_busy(lambda obs, t: [0, 0, 0]))  # the backend's own drivers
        t0 = time.time()
        th = threading.Thread(target=ctl, args=([0.0, 0.0], 0.0), daemon=True)
        th.start()
        busy = None
        while busy is None and time.time() - t0 < 5:
            busy = session.control_busy(ctl)
            time.sleep(0.01)
        try:
            self.assertIsNotNone(busy)
            self.assertGreaterEqual(busy[0], t0)
            self.assertEqual(busy[1], "（slow.py 第 6 行）")
        finally:
            sys.modules["user_controller"].GO.set()
            th.join(5)
        self.assertIsNone(session.control_busy(ctl))


class BackendReasonTests(unittest.TestCase):
    """E2 / B13: the cosim_state event says why, the GUI banner shows it."""

    def test_failed_start_reports_the_reason(self):
        backend = Backend()
        events = []
        backend.emit = events.append
        settings = SimpleNamespace(synchronous_mode=False, fixed_delta_seconds=None)
        backend.world = SimpleNamespace(get_settings=lambda: settings, apply_settings=lambda s: None,
                                        tick=lambda: 1, reset_all_traffic_lights=lambda: None)
        backend.cmd_spawn_ego = lambda *args: setattr(backend, "ego", SimpleNamespace(is_alive=False))
        why = "加载控制算法出错：NameError: name 'x' is not defined（c.py 第 1 行）"

        class Session:
            def __init__(self, *args):
                self.scene = None

            def start(self):
                raise RuntimeError(why)

            def stop(self, release_vehicle=True):
                pass

        with mock.patch.object(backend_server, "CoSimSession", Session), \
                mock.patch.object(backend_server.rigmod, "build_preset", lambda *a: []), \
                mock.patch.object(backend_server.rigmod, "spec_of", lambda ego: {}), \
                mock.patch.object(backend_server.rigmod, "preset_reference", lambda *a: [0.0, 0.0, 0.0]), \
                mock.patch.object(backend_server.traceback, "print_exc"):
            with self.assertRaisesRegex(RuntimeError, "NameError"):
                backend.cmd_cosim_start({"drive": {"dynamics": "cosim"}, "carsim": {"mock": True},
                                         "run": {"driver": "demo"}})  # passes the pre-flight
        states = [e for e in events if e.get("event") == "cosim_state"]
        self.assertEqual(states[-1]["state"], "error")
        self.assertEqual(states[-1]["detail"], why)

    def test_worker_error_reports_the_reason(self):
        backend = Backend()
        events = []
        backend.emit = events.append
        backend.cosim_state = "running"
        calls = []

        def iteration():
            calls.append(1)
            if len(calls) == 1:
                raise ValueError("boom")
            time.sleep(0.05)

        backend._worker_iteration = iteration
        with mock.patch.object(backend_server.traceback, "print_exc"):
            threading.Thread(target=backend.run_worker, daemon=True).start()
            end = time.time() + 5
            while len(calls) < 2 and time.time() < end:
                time.sleep(0.01)
        states = [e for e in events if e.get("event") == "cosim_state"]
        self.assertTrue(states)
        self.assertEqual(states[0]["detail"], "后端内部错误：ValueError: boom")


if __name__ == "__main__":
    unittest.main()
