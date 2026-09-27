"""The docs agree with the code (no CARLA server needed): relative links
resolve, the training-code snippet in the guides builds CarlaVehicleSync from
the GUI's config in synchronous mode (run once against fakes), command-line
--sim examples say how long they run, the bottom-panel tabs, and the
sampling, lidar, speed-limit and scene-panel statements of the spec."""

import glob
import inspect
import json
import os
import re
import sys
import tempfile
import types
import unittest
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)

import config  # noqa: E402
import settings as st  # noqa: E402

REPO = os.path.normpath(os.path.join(HERE, "..", ".."))
BRIDGE = os.path.join(REPO, "carsim_carla_bridge")
MANUAL = os.path.join(REPO, "docs", "界面操作手册.md")
SPEC = os.path.join(REPO, "docs", "场景与数据接口.md")
GUIDES = [os.path.join(REPO, "docs", "Ubuntu使用指南.md"), os.path.join(REPO, "docs", "Windows使用指南.md"),
          os.path.join(BRIDGE, "README.md")]
DOCS = sorted(glob.glob(os.path.join(REPO, "docs", "*.md")) + glob.glob(os.path.join(BRIDGE, "docs", "*.md"))
              + [os.path.join(REPO, "README.md"), os.path.join(BRIDGE, "README.md"),
                 os.path.join(REPO, "cosim_gui", "README.md")])
CN = "零一二三四五六七八九"


def read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def rel(path):
    return os.path.relpath(path, REPO)


class LinkTests(unittest.TestCase):
    def test_relative_links_resolve(self):
        bad = []
        for doc in DOCS:
            text = re.sub(r"```.*?```", "", read(doc), flags=re.S)  # code is not markdown
            for target in re.findall(r"\]\(([^)\s]+)\)", text):
                if re.match(r"[a-z]+:", target) or target.startswith("#"):
                    continue
                path = os.path.normpath(os.path.join(os.path.dirname(doc), target.split("#")[0]))
                if not path.startswith(REPO + os.sep):
                    continue  # a GitHub page such as ../../releases
                if not os.path.exists(path):
                    bad.append("%s: %s" % (rel(doc), target))
        self.assertEqual(bad, [])

    def test_config_py_points_to_the_export_variable_list(self):
        names = re.findall(r"docs/\S+\.md", read(os.path.join(BRIDGE, "config.py")))
        self.assertTrue(names)
        for n in names:
            self.assertTrue(os.path.exists(os.path.join(BRIDGE, n)), n)


class _Settings:
    synchronous_mode = False
    fixed_delta_seconds = None


class _World:
    def __init__(self, log):
        self.log, self.s = log, _Settings()

    def get_settings(self):
        s = _Settings()
        s.synchronous_mode, s.fixed_delta_seconds = self.s.synchronous_mode, self.s.fixed_delta_seconds
        return s

    def apply_settings(self, s):
        self.s = s
        self.log.append(("apply", s.synchronous_mode, s.fixed_delta_seconds))

    def tick(self):
        self.log.append(("tick",))


class _Env:
    t_step, t_current = 0.001, 0.0

    def control_step(self, action, inner_steps):
        self.t_current += inner_steps * self.t_step
        return (0.0,), 0.0, False, {}


class SnippetTests(unittest.TestCase):
    """The training-code snippet: the GUI's export order / units reach the
    bridge, the world is synchronous with one frame = inner_steps * t_step."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="cc_docs_")
        d = st.default_dict()
        d["carsim"]["export_names"] = list(reversed(d["carsim"]["export_names"]))
        d["carsim"]["units"] = {"angle": "rad", "speed": "m/s", "rate": "rad/s", "wheel_spin": "rad/s", "jounce": "m"}
        self.path = os.path.join(self.tmp.name, "cosim_config.json")
        with open(self.path, "w", encoding="utf-8") as f:
            json.dump(d, f)
        self.d = st.load_dict(self.path)

    def tearDown(self):
        self.tmp.cleanup()

    def test_gui_config_reaches_the_bridge(self):
        cfg = st.to_bridge_cfg(self.d)
        self.assertEqual(cfg.EXPORT_NAMES, self.d["carsim"]["export_names"])
        self.assertEqual(cfg.UNITS, self.d["carsim"]["units"])
        self.assertNotEqual(cfg.EXPORT_NAMES, list(config.EXPORT_NAMES))  # what settings=None would read
        import bridge
        self.assertIn("settings", inspect.signature(bridge.CarlaVehicleSync.__init__).parameters)

    def test_guides_snippet_runs(self):
        for doc in GUIDES:
            blocks = [b for b in re.findall(r"```python\n(.*?)```", read(doc), flags=re.S) if "CarlaVehicleSync(" in b]
            self.assertEqual(len(blocks), 1, rel(doc))
            log = []

            class Sync:
                def __init__(self, world, vehicle, anchor, export_names=None, use_external_api=None, settings=None):
                    log.append(("init", export_names, settings))

                def sync(self, obs, sim_time, dt):
                    log.append(("sync", sim_time, dt))

            fake = types.ModuleType("bridge")
            fake.CarlaVehicleSync = Sync
            env = _Env()
            ns = {"env": env, "world": _World(log), "vehicle": object(), "anchor_transform": object(),
                  "action": [0.2, 0.0, 0.0], "inner_steps": 20}
            paths = []
            path0 = list(sys.path)
            try:
                with mock.patch.dict(sys.modules, {"bridge": fake}), \
                        mock.patch.object(st, "load_dict", side_effect=lambda p, *a, **k: paths.append(p) or self.d):
                    exec(compile(blocks[0], rel(doc), "exec"), ns)
            finally:
                sys.path[:] = path0
            self.assertTrue(paths and paths[0].endswith("cosim_config.json"), (rel(doc), paths))
            kinds = [e[0] for e in log]
            self.assertIn("init", kinds, rel(doc))
            init = log[kinds.index("init")]
            cfg = init[2]
            self.assertIsNotNone(cfg, "%s: CarlaVehicleSync without settings= reads config.py" % rel(doc))
            self.assertEqual(cfg.EXPORT_NAMES, self.d["carsim"]["export_names"], rel(doc))
            self.assertEqual(cfg.UNITS, self.d["carsim"]["units"], rel(doc))
            applied = [e for e in log[:kinds.index("init")] if e[0] == "apply"]
            self.assertTrue(applied, "%s: synchronous mode is set before the bridge" % rel(doc))
            self.assertTrue(applied[-1][1], rel(doc))
            self.assertAlmostEqual(applied[-1][2], 20 * env.t_step, msg=rel(doc))
            sync = log[kinds.index("sync")]
            self.assertAlmostEqual(sync[2], applied[-1][2], msg=rel(doc))  # the bridge's dt = CARLA's frame
            self.assertAlmostEqual(sync[1], env.t_current, msg=rel(doc))
            self.assertEqual(kinds[-1], "tick", rel(doc))


class CliTests(unittest.TestCase):
    def test_sim_examples_say_how_long(self):
        # A command-line run without --duration or --config lasts 20 s.
        src = read(os.path.join(BRIDGE, "run_cosim.py"))
        self.assertIn('o["sync"]["duration"] = 20.0', src)
        bad = []
        for path in DOCS + [os.path.join(BRIDGE, "run_cosim.py")]:
            for no, line in enumerate(read(path).splitlines(), 1):
                if "run_cosim.py" in line and re.search(r"--sim\b", line) and "--duration" not in line:
                    bad.append("%s:%d: %s" % (rel(path), no, line.strip()[:90]))
        self.assertEqual(bad, [])


class GuiTabTests(unittest.TestCase):
    def tabs(self):
        src = read(os.path.join(REPO, "cosim_gui", "src", "ui_layout.cpp"))
        start = src.index('BeginTabBar("docktabs")')
        end = src.index("EndTabBar()", start)
        body = src[src.rfind("\nvoid ", 0, start):end]
        names = []
        for arg in re.findall(r"BeginTabItem\(([^,]+),", src[start:end]):
            m = re.search(r'"  ([一-鿿]+)', arg)
            if not m:  # a label built before the tab bar
                m = re.search(r"\b%s\s*=[^;]*?\"  ([一-鿿]+)" % re.escape(arg.strip().split(".")[0]), body)
            names.append(m.group(1))
        return src, names

    def test_manual_and_menu_list_the_bottom_tabs(self):
        src, names = self.tabs()
        self.assertEqual(len(names), 4, names)
        manual = read(MANUAL)
        self.assertIn(" ".join("[%s]" % n for n in names), manual)
        self.assertIn("%s个页签" % CN[len(names)], manual)
        self.assertIn("（%s）" % " / ".join(names), manual)  # Ctrl+L
        self.assertIn("输出**（底部面板的第%s个页签）" % CN[names.index("输出") + 1], manual)
        menu = next(line for line in src.splitlines() if "底部面板（" in line and "MenuItem" in line)
        for n in names:
            self.assertIn(n[-2:], menu, n)

    def test_manual_names_the_driving_options(self):
        panels = read(os.path.join(REPO, "cosim_gui", "src", "panels.cpp"))
        self.assertIn('"CarSim 联合仿真"', panels)
        self.assertIn("测试用驾驶方式", panels)
        flow_a = next(line for line in read(MANUAL).splitlines() if line.startswith("连接 → ") and "模拟 CarSim" in line)
        self.assertIn("CarSim 联合仿真", flow_a)
        self.assertIn("测试用驾驶方式", flow_a)


class SpecTests(unittest.TestCase):
    def test_sample_period_zero(self):
        d = st.default_dict()
        d["collect"].update(sample_period=0.0, capture_every=2)
        self.assertEqual(st.sample_every(d), 2)  # an older config's capture_every still counts
        line = next(line for line in read(SPEC).splitlines() if "默认 0 = 每帧采样" in line)
        self.assertIn("capture_every", line)

    def test_lidar_points_per_sweep(self):
        # One full sweep per frame: CARLA casts points_per_second * frame_dt rays per tick.
        for name, pat in (("scene.py", r'"rotation_frequency", str\(1\.0 / self\.frame_dt\)'),
                          ("collector.py", r'"rotation_frequency", str\(1\.0 / c\["frame_dt"\]\)')):
            self.assertTrue(re.search(pat, read(os.path.join(BRIDGE, name))), name)
        for doc in (SPEC, MANUAL):
            self.assertIn("一圈的点数 = 每秒点数 × 仿真步长", read(doc), rel(doc))

    def test_speed_limit_is_carlas_vehicle_limit(self):
        self.assertIn("self.ego.get_speed_limit()", read(os.path.join(BRIDGE, "scene.py")))
        row = next(line for line in read(SPEC).splitlines() if line.startswith("| `speed_limit`"))
        self.assertIn("限速牌", row)
        self.assertIn("待确认", row)

    def test_scene_panel_shows_every_key(self):
        import scene
        obj = {k: 1.0 for k in scene.OBJECT_KEYS}
        obj.update(id=7, type="vehicle", model="vehicle.tesla.model3", parked=False, dist=5.0)
        view = scene.gui_view({"objects": [obj], "ego": {"X": 0.0, "Y": 0.0}, "collisions": []})
        self.assertEqual(set(view["objects"][0]), set(scene.OBJECT_KEYS))
        row = next(line for line in read(SPEC).splitlines() if line.startswith("| 底部面板 → 场景"))
        self.assertIn("不受", row)
        self.assertNotIn("算法这一帧收到", row)

    def test_run_record_units_and_overwrite(self):
        spec = read(SPEC)
        self.assertIn("文件里不写单位", spec)
        self.assertIn("**覆盖**", spec)


class InstructionTests(unittest.TestCase):
    def test_export_list_points_to_the_carsim_page(self):
        text = read(os.path.join(BRIDGE, "docs", "CarSim导出变量清单.md"))
        self.assertIn("CarSim 动力学", text)
        bad = [line for line in text.splitlines() if "config.py" in line and "默认" not in line]
        self.assertEqual(bad, [])
        self.assertNotIn("需要按你的 .sim 修改", read(os.path.join(BRIDGE, "README.md")))

    def test_readme_directory_map(self):
        readme = read(os.path.join(REPO, "README.md"))
        row = next(line for line in readme.splitlines() if line.startswith("| `scripts/` |"))
        for p in glob.glob(os.path.join(REPO, "scripts", "*.sh")) + glob.glob(os.path.join(REPO, "scripts", "windows", "*.bat")):
            self.assertIn("`%s`" % os.path.basename(p), row)
        for p in glob.glob(os.path.join(REPO, "carla_patches", "*.patch")):
            self.assertIn(os.path.basename(p), readme)
        self.assertNotIn("| `docs/` | 截图 |", readme)
        stock = next(line for line in readme.splitlines() if "`get_velocity()`" in line and "❌" in line)
        self.assertIn("陀螺仪", stock)  # the accelerometer comes from position differences

    def test_restart_test_follows_env_ports(self):
        import test_carla_restart as t
        with mock.patch.dict(os.environ, {"CARLA_PORT": "2100", "CARLA_MOD_PORT": "3100"}):
            self.assertEqual((t.server_port(False), t.server_port(True)), (2100, 3100))
        with mock.patch.dict(os.environ):
            os.environ.pop("CARLA_PORT", None)
            os.environ.pop("CARLA_MOD_PORT", None)
            self.assertEqual((t.server_port(False), t.server_port(True)), (2000, 3000))


if __name__ == "__main__":
    unittest.main()
