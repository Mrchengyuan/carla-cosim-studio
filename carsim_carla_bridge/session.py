"""One co-simulation run, stepped frame by frame.

Shared by run_cosim.py (CLI loop) and backend_server.py (GUI backend), so
both run exactly the same code path.
"""

import ast
import contextlib
import ctypes
import importlib.util
import inspect
import math
import os
import sys
import threading
import time
import traceback

import carla

import rig as rigmod
import settings as st
from bridge import REQUIRED_EXPORTS, CarlaVehicleSync, ExportCheck, front_axle_local
from collector import DISK_RESERVE_GB
from drivers import ManualDriver, RouteFollower
from scene import ALWAYS_OBJECT_KEYS, EGO_KEYS, LANE_KEYS, OBJECT_KEYS, Recorder, SceneProvider, gui_view


def demo_driver(t):
    """Accelerate, then a slalom: exercises steer, roll, pitch and spin."""
    throttle = 0.6 if t < 6.0 else 0.25
    brake = 0.4 if 14.0 < t < 15.5 else 0.0
    steer_sw = 0.0 if t < 3.0 else 90.0 * math.sin(2 * math.pi * 0.25 * (t - 3.0))
    return [0.0 if brake else throttle, brake, steer_sw]


# Longest CARLA frame: PhysX substeps cover at most 10 x 0.01 s per frame.
MAX_FRAME_DT = 0.1
# What python_carsim_env's get_api reads from the solver (a missing one is a
# bare AttributeError there).
_VS_API = ("vs_run", "vs_initialize", "vs_read_configuration", "vs_integrate_io", "vs_copy_export_vars",
           "vs_terminate_run", "vs_error_occurred", "vs_set_opt_error_dialog", "vs_get_error_message", "vs_road_l")


def _carsim_module(repo_path):
    """carsim_env (and its vs_solver) from the configured python_carsim_env folder."""
    repo = os.path.abspath(repo_path)
    if not os.path.isfile(os.path.join(repo, "carsim_env.py")):
        raise ValueError("python_carsim_env 目录不对：%s 里没有 carsim_env.py（“CarSim 动力学”页）" % repo)
    if repo in sys.path:
        sys.path.remove(repo)
    sys.path.insert(0, repo)
    # A different python_carsim_env folder than last run: import that one.
    old = sys.modules.get("carsim_env")
    if old is not None and os.path.dirname(os.path.abspath(getattr(old, "__file__", ""))) != repo:
        for m in ("carsim_env", "vs_solver"):
            sys.modules.pop(m, None)
    import carsim_env
    return carsim_env


def check_carsim(d):
    """The .sim, python_carsim_env and the solver the .sim names, checked in
    plain words: python_carsim_env reports these as a bare TypeError,
    AttributeError or ModuleNotFoundError. Returns (.sim path, carsim_env)."""
    c = d["carsim"]
    if not c["sim_path"]:
        raise ValueError("没有设置 CarSim .sim 文件（“CarSim 动力学”页；没有 CarSim 时可勾选“模拟 CarSim”）")
    sim = os.path.abspath(c["sim_path"])
    if not os.path.isfile(sim):
        raise ValueError(("CarSim .sim 文件填的是目录：%s" if os.path.isdir(sim) else "CarSim .sim 文件不存在：%s") % sim)
    with open(sim, "rb") as f:
        lines = [ln.strip() for ln in f.read(1 << 20).decode("latin-1").splitlines()]
    lib_key = "SOFILE" if sys.platform == "linux" else "DLLFILE"  # as vs_solver.get_dll_path

    def value(key):
        return next((ln[len(key):].strip() for ln in lines if ln.startswith(key)), None)
    if value("VEHICLE_CODE") is None or (value("PROGDIR") is None and value(lib_key) is None):
        raise ValueError("%s 不是 CarSim 生成的 .sim 文件（里面没有 VEHICLE_CODE / PROGDIR）；"
                         "请选 CarSim 生成的 simfile.sim，不是 .par / .cpar 等其他文件" % sim)
    carsim_env = _carsim_module(c["repo_path"])
    try:
        dll = importlib.import_module("vs_solver").vs_solver().get_dll_path(sim)
    except Exception as e:
        raise ValueError("读不了 .sim 文件 %s：%s: %s" % (sim, type(e).__name__, e))
    if not dll or not os.path.isfile(dll):
        rel = [v for v in (value("PROGDIR"), value(lib_key)) if v and not os.path.isabs(v)]
        raise ValueError("找不到 CarSim 求解器：%s（由 .sim 里的 PROGDIR / %s 得出%s）" % (
            dll, lib_key, "；其中的相对路径 %s 是按后端的工作目录 %s 解析的，请在 .sim 里写绝对路径"
            % (" ".join(rel), os.getcwd()) if rel else ""))
    try:
        lib = ctypes.CDLL(dll)
    except OSError as e:
        # python_carsim_env retries with ctypes.WinDLL, which Linux does not
        # have: the loader's own reason would be lost.
        raise RuntimeError("无法加载 CarSim 求解器 %s：%s（Python 是 %d 位，求解器要同样位数，例如 carsim_64.dll；"
                           "求解器依赖的 CarSim 安装文件也要在）" % (dll, e, 8 * ctypes.sizeof(ctypes.c_void_p)))
    missing = [n for n in _VS_API if not hasattr(lib, n)]
    if missing:
        raise RuntimeError("CarSim 求解器 %s 缺少 python_carsim_env 要用的函数：%s（选错了 DLL，或求解器版本不对）"
                           % (dll, "、".join(missing)))
    return sim, carsim_env


def check_run_config(d):
    """Refuse a run config in plain words before anything in the world changes."""
    s = d["sync"]
    try:
        s["frame_dt"] = float(s["frame_dt"])
    except (TypeError, ValueError):
        raise ValueError("仿真步长不是数字：%r" % (s["frame_dt"],))
    if not 0 < s["frame_dt"] <= MAX_FRAME_DT:
        raise ValueError("仿真步长 %g s 超出范围：要大于 0、不超过 %g s（CARLA 的物理每帧最多算 %g s）"
                         % (s["frame_dt"], MAX_FRAME_DT, MAX_FRAME_DT))
    dyn = d["drive"]["dynamics"]
    if dyn == "carla":
        if d["drive"]["carla_driver"] not in ("route", "autopilot", "manual"):
            raise ValueError("未知的驾驶方式 %r（CARLA 物理下可选 route、autopilot、manual）" % d["drive"]["carla_driver"])
        return
    if dyn != "cosim":
        raise ValueError("未知的动力学 %r（可选 cosim = CarSim 联合仿真、carla = CARLA 物理）" % dyn)
    if d["run"]["driver"] not in ("custom", "demo", "route", "manual"):
        raise ValueError("未知的驾驶方式 %r（CarSim 联合仿真下可选 custom、demo、route、manual）" % d["run"]["driver"])
    missing = [n for n in REQUIRED_EXPORTS if n not in d["carsim"]["export_names"]]
    if missing:
        raise ValueError("导出变量里缺少必需的 %s（“CarSim 动力学”页）" % "、".join(missing))


def check_run_files(d):
    """The files a CarSim run needs, checked before the world changes (a
    start that fails later has already respawned the ego)."""
    if d["drive"]["dynamics"] != "cosim":
        return
    if d["run"]["driver"] == "custom":
        path = os.path.abspath(d["run"]["controller"]["path"])
        if not os.path.isfile(path):
            raise ValueError("控制算法文件不存在：%s" % path)
    if not d["carsim"]["mock"]:
        check_carsim(d)


def make_env(d):
    c = d["carsim"]
    if c["mock"]:
        from mock_carsim import MockCarSimEnv
        dur = d["sync"]["duration"]
        return MockCarSimEnv(c["export_names"], t_stop=dur + 1.0 if dur > 0 else 1e9, units=c["units"])
    sim, carsim_env = check_carsim(d)
    try:
        return carsim_env.CarSimEnv(sim)
    except Exception as e:
        raise RuntimeError("加载 CarSim 失败：%s: %s" % (type(e).__name__, e)) from e


def _vs_error(env):
    """The CarSim solver's own error message; '' without one (and for the mock)."""
    dll = getattr(getattr(env, "solver", None), "dll_handle", None)
    try:
        if dll is None or not dll.vs_error_occurred():
            return ""
        raw = dll.vs_get_error_message()
        return raw.decode("mbcs" if os.name == "nt" else "utf-8", errors="replace").strip() if raw else ""
    except Exception:
        return ""


def _reset_failed(env, err):
    """Why CarSim could not start the run: the solver's own message, else what python_carsim_env saw."""
    cfg = env.config if isinstance(getattr(env, "config", None), dict) else {}
    why = _vs_error(env)
    if not why and cfg and not (cfg.get("n_import", 0) > 0 and cfg.get("n_export", 0) > 0):
        why = "没有读出导入 / 导出变量（导入 %s 个、导出 %s 个）" % (cfg.get("n_import"), cfg.get("n_export"))
    elif not why:
        why = "%s: %s" % (type(err).__name__, err)
    sim = str(getattr(env, "sim_path", ""))
    return ("CarSim 没能开始这次运行：%s。常见原因：CarSim 许可证不可用；.sim 引用的数据文件路径不对；"
            ".sim 里没有设导入 / 导出变量%s" % (
                why, "；路径 %s 里有中文等非 ASCII 字符，CarSim 求解器可能读不到" % sim if not sim.isascii() else ""))


def reset_env(env):
    """env.reset() with CarSim's own reason when the .sim cannot run:
    python_carsim_env never asks the solver and only says that the import /
    export counts are invalid. A reset that returned is a started run: the
    solver's error flag may be left over from an earlier failed start."""
    try:
        return env.reset()
    except Exception as e:
        raise RuntimeError(_reset_failed(env, e)) from e


def make_driver(d, ex, n_imports=None, scene=None, output=None):
    drv = d["run"]["driver"]
    if drv == "demo":
        return lambda obs, t: demo_driver(t)
    if drv == "custom":
        return load_controller(d, ex, n_imports, scene, output)
    raise ValueError("unknown CarSim driver '%s'" % drv)


_HERE = os.path.dirname(os.path.abspath(__file__))
# Python's own library: a conda / Windows install may sit inside the algorithm's
# folder (e.g. the algorithm right in the home folder).
_PYLIB = tuple({os.path.normcase(os.path.join(os.path.abspath(p), ""))
                for p in (sys.prefix, sys.base_prefix, sys.exec_prefix, sys.base_exec_prefix)})
_START_PATH = set(sys.path)  # e.g. PYTHONPATH: never taken off sys.path again
_last_folder = None  # the algorithm folder of the previous load


def _own_file(f, folder):
    """Is file f the algorithm's own code in folder or a sub-folder? Not a
    library (a venv inside it) and not one of the backend's modules (the
    algorithm may sit next to them or at the repo root)."""
    if f.startswith("<"):  # <frozen importlib._bootstrap>, <string>
        return False
    f, top = os.path.normcase(os.path.abspath(f)), os.path.normcase(os.path.join(folder, ""))
    if not f.startswith(top) or f.startswith(_PYLIB) or os.path.dirname(f) == os.path.normcase(_HERE):
        return False
    parts = f[len(top):].split(os.sep)
    return "site-packages" not in parts and "dist-packages" not in parts


def _short(f, folder):
    f = os.path.abspath(f)
    return os.path.relpath(f, folder) if f.startswith(os.path.join(folder, "")) else os.path.basename(f)


def _where(frames, path):
    """'（mpc.py 第 57 行，由 my_ctrl.py 第 12 行调用）': the innermost line in the
    algorithm's own files and, when that is a helper module, the line of the
    algorithm file that led there. frames: (file, line), outermost first."""
    folder = os.path.dirname(path)
    mine = [(f, n) for f, n in frames if os.path.abspath(f) == path or _own_file(f, folder)]
    if not mine:
        return ""
    f, n = mine[-1]
    where = "%s 第 %d 行" % (_short(f, folder), n)
    top = next((m for g, m in reversed(mine) if os.path.abspath(g) == path), None)
    if os.path.abspath(f) != path and top is not None:
        where += "，由 %s 第 %d 行调用" % (os.path.basename(path), top)
    return "（%s）" % where


def _tb(e):
    return [(f.filename, f.lineno) for f in traceback.extract_tb(e.__traceback__)]


def _user_error(what, e, path):
    """'控制算法出错：ValueError: boom（my_ctrl.py 第 12 行）': the user needs the
    lines of their own files, not the backend's."""
    msg, where = str(e), _where(_tb(e), path)
    if isinstance(e, SyntaxError) and e.lineno:  # possibly in a module the algorithm imports
        msg, where = e.msg, "（%s 第 %d 行）" % (_short(e.filename or path, os.path.dirname(path)), e.lineno)
    return "%s：%s: %s%s" % (what, type(e).__name__, msg, where)


def _own_trace(e, path):
    """The traceback of e in the algorithm's own files (and of the exceptions
    it came from), outermost first, for the 输出 page:

        出错位置（只列你的文件，外层在前）：
          my_ctrl.py 第 12 行 control：u = mpc.solve(x)
          mpc.py 第 2 行 solve：return 1.0 / x
        ZeroDivisionError: float division by zero

    Library and backend frames are left out; '' when no frame is the user's."""
    folder = os.path.dirname(path)

    def own(f):
        return os.path.abspath(f) == path or _own_file(f, folder)
    chain = [(e, "")]  # (exception, how it follows the one printed before it)
    while len(chain) < 5:
        x = chain[-1][0]
        if x.__cause__ is not None:
            nxt, how = x.__cause__, "上面的异常引起了下面的异常："
        elif x.__context__ is not None and not x.__suppress_context__:
            nxt, how = x.__context__, "处理上面的异常时又出错："
        else:
            break
        if any(nxt is c for c, _ in chain):
            break
        chain[-1] = (x, how)
        chain.append((nxt, ""))
    out = []
    for x, how in reversed(chain):  # the first cause first, like Python
        frames = [(f.filename, f.lineno, f.name, f.line) for f in traceback.extract_tb(x.__traceback__) if own(f.filename)]
        if isinstance(x, SyntaxError) and x.filename and x.lineno and own(x.filename):
            frames.append((x.filename, x.lineno, "", x.text))
        if not frames:
            continue
        if out and how:
            out.append(how)
        if len(frames) > 12:  # e.g. a RecursionError
            frames = frames[:3] + [len(frames) - 11] + frames[-8:]
        for fr in frames:
            if isinstance(fr, int):
                out.append("  ……（中间省略 %d 层）" % fr)
                continue
            f, n, name, text = fr
            out.append("  %s 第 %d 行%s：%s" % (_short(f, folder), n, " 模块顶层" if name == "<module>" else
                                                (" " + name if name else ""), (text or "").strip()[:200]))
        msg = str(x)
        out.append("%s: %s" % (type(x).__name__, msg[:500]) if msg else type(x).__name__)
    return "出错位置（只列你的文件，外层在前）：\n" + "\n".join(out) if out else ""


class _AlgoStream:
    """sys.stdout / sys.stderr while the algorithm runs (AlgoOutput): every
    write goes on to the real stream (backend.log); the algorithm thread's
    writes are also kept for the GUI. Anything else (encoding, fileno,
    isatty ...) is the real stream's. One pair for every run: a logging
    handler the algorithm made on its first run keeps writing into it."""
    real = None
    out = None  # the AlgoOutput capturing right now

    def __init__(self, i):
        self._i = i

    def write(self, s):
        if self.real is not None:
            self.real.write(s)
        out = self.out
        if out is not None and out._thread == threading.get_ident() and isinstance(s, str):
            out._feed(self._i, s)
        return len(s)

    def flush(self):
        if self.real is not None:
            self.real.flush()

    def __getattr__(self, name):
        return getattr(self.real, name)


_ALGO_STREAMS = (_AlgoStream(0), _AlgoStream(1))


class AlgoOutput:
    """What the user's algorithm prints (print(), sys.stderr, warnings) while
    it is loaded and while its control() runs, for the GUI's 输出 page, and
    the traceback of its own files when it raises. `with out:` around the
    algorithm's code swaps sys.stdout / sys.stderr for that time only, and
    keeps only the writes of the calling thread (not the backend's other
    threads); everything still reaches the real streams (backend.log). At
    most RATE lines a second are kept, then one '（省略 N 行）' line."""
    RATE = 20      # lines a second for the GUI
    PENDING = 200  # kept until take(): run_cosim.py never takes them
    MAX_LEN = 1000  # characters of one line
    clock = time.monotonic

    def __init__(self):
        self.path = ""  # the algorithm file (load_controller): the traceback shows its folder's frames
        self._part = ["", ""]  # a line not ended yet, per stream
        self._lines, self._n, self._skipped, self._window = [], 0, 0, None
        self._thread = self._saved = None

    def __enter__(self):
        self._saved = (sys.stdout, sys.stderr)
        for s, real in zip(_ALGO_STREAMS, self._saved):
            if real is not s:
                s.real = real
            s.out = self
        sys.stdout, sys.stderr = _ALGO_STREAMS
        self._thread = threading.get_ident()
        return self

    def __exit__(self, et, e, tb):
        self._thread = None
        for s in _ALGO_STREAMS:
            s.out = None
        sys.stdout, sys.stderr = self._saved
        for i, part in enumerate(self._part):  # print(..., end=""): shown now
            if part:
                self._part[i] = ""
                self._add(part)
        if e is not None and self.path:
            trace = _own_trace(e, self.path)
            if trace:
                self._lines.append(trace)  # not limited: the run stops with it
        return False

    def _feed(self, i, s):
        *done, part = (self._part[i] + s).split("\n")
        for line in done:
            self._add(line)
        self._part[i] = part[:self.MAX_LEN + 1]  # the rest of a very long line is not shown anyway

    def _add(self, line):
        now = self.clock()
        if self._window is None or now - self._window >= 1.0:
            self._roll(now)
        if self._n < self.RATE and len(self._lines) < self.PENDING:
            self._n += 1
            line = line.rstrip("\r").expandtabs(4)
            self._lines.append(line if len(line) <= self.MAX_LEN else line[:self.MAX_LEN] + " …")
        else:
            self._skipped += 1

    def _roll(self, now):
        if self._skipped and len(self._lines) < self.PENDING:
            self._lines.append("（省略 %d 行，全部输出见 backend.log）" % self._skipped)
            self._skipped = 0
        self._window, self._n = now, 0

    def take(self, end=False):
        """The lines to show since the last take; at the end of a run (end)
        also how many lines the limit left out."""
        if self._skipped and (end or self.clock() - self._window >= 1.0):
            self._roll(self.clock())
        lines, self._lines = self._lines, []
        return lines


def _scene_miss(k, sel, rig=()):
    """Why the scene handed to control() has no key k: it is not ticked
    "给算法" on the 场景信息 page, or a ticked sensor is no enabled sensor of
    the rig (rig: the 传感器套件 sensors, if known). '' when k is no scene key."""
    sensors = sel.get("sensors") or ()
    if k == "lane" and not sel.get("lane"):
        return "“场景信息”页车道一栏没有勾选“给算法”的量"
    if k == "sensors":  # there only when a ticked sensor is an enabled one of the rig
        if not sensors:
            return "“场景信息”页没有勾选给算法的传感器"
        return "“场景信息”页勾选给算法的传感器（%s）在“传感器套件”里没有启用或已经删掉" % "、".join(sensors)
    if k in sensors:  # scene["sensors"] has every ticked sensor the rig has enabled
        return "这个传感器在“传感器套件”里没有启用或已经删掉"
    if any(s.get("name") == k for s in rig):
        return "没有在“场景信息”页给这个传感器勾选“给算法”"
    if k == "collisions" and sel.get("collision", "log") == "off":
        return "“场景信息”页碰撞选了“不检测”"
    for keys, ticked in ((EGO_KEYS, sel.get("ego") or ()), (LANE_KEYS, sel.get("lane") or ()),
                         (OBJECT_KEYS, list(ALWAYS_OBJECT_KEYS) + list(sel.get("objects") or ()))):
        if k in keys and k not in ticked:
            return "没有在“场景信息”页给它勾选“给算法”"
    return ""


def _name_clashes(path):
    """Modules of the algorithm's folder that its code imports under a name
    sys.modules already holds from another file (the backend's config.py,
    CARLA's agents package, the standard library): `import config` would
    silently return that one. Its code: the algorithm file and the files of
    the folder it imports, directly or through them (not every script lying
    in the folder). [(name, own file, the loaded file), ...]"""
    folder = os.path.dirname(path)
    taken = []
    for n in sorted(os.listdir(folder)):
        own = os.path.join(folder, n) if n.endswith(".py") else os.path.join(folder, n, "__init__.py")
        name = n[:-3] if n.endswith(".py") else n
        m = sys.modules.get(name)
        if m is None or not os.path.isfile(own):
            continue
        f = getattr(m, "__file__", None)
        if not (isinstance(f, str) and os.path.normcase(os.path.abspath(f)) == os.path.normcase(own)):
            taken.append((name, n if n.endswith(".py") else n + os.sep, f))
    if not taken:
        return []

    def files(base, name):  # module a.b.c under base: a/__init__.py, a/b/__init__.py, a/b/c.py
        out = []
        for part in name.split("."):
            base = os.path.join(base, part)
            f = next((g for g in (base + ".py", os.path.join(base, "__init__.py")) if os.path.isfile(g)), None)
            if f is None:
                break
            out.append(f)
        return out

    imported, todo, seen = set(), [path], set()
    while todo:
        f = todo.pop()
        if f in seen:
            continue
        seen.add(f)
        try:
            with open(f, "rb") as fh:
                tree = ast.parse(fh.read())
        except (OSError, SyntaxError, ValueError):
            continue  # a syntax error is reported by the import itself
        for node in ast.walk(tree):  # also imports inside functions
            mods = []
            if isinstance(node, ast.Import):
                imported.update(a.name.split(".")[0] for a in node.names)
                mods = [(folder, a.name) for a in node.names]
            elif isinstance(node, ast.ImportFrom):
                base = folder
                if node.level:  # from . import x: relative to the package of f
                    base = os.path.dirname(f)
                    for _ in range(node.level - 1):
                        base = os.path.dirname(base)
                elif node.module:
                    imported.add(node.module.split(".")[0])
                pre = node.module + "." if node.module else ""
                mods = ([(base, node.module)] if node.module else []) + \
                       [(base, pre + a.name) for a in node.names if a.name != "*"]  # a.name may be a module
            todo += [g for base, name in mods for g in files(base, name) if _own_file(g, folder)]
    return [t for t in taken if t[0] in imported]


def control_busy(driver):
    """While the user's control() (a load_controller() driver) runs: (since
    when, '（my_ctrl.py 第 42 行）' where it is now), else None. For the
    backend's "busy" heartbeat: a slow control() is not a hung CARLA."""
    running = getattr(driver, "running", None)
    if running is None:
        return None
    since, thread = running
    frame, stack = sys._current_frames().get(thread), []
    while frame is not None:
        stack.append((frame.f_code.co_filename, frame.f_lineno))
        frame = frame.f_back
    return since, _where(stack[::-1], driver.path)


def _wants_scene(fn):
    """control(exports, t, dt, scene): a 4th parameter (or *args) asks for the scene."""
    try:
        ps = list(inspect.signature(fn).parameters.values())
    except (TypeError, ValueError):
        return False
    if any(p.kind == p.VAR_POSITIONAL for p in ps):
        return True
    return len([p for p in ps if p.kind in (p.POSITIONAL_ONLY, p.POSITIONAL_OR_KEYWORD)]) >= 4


def load_controller(d, ex, n_imports=None, scene=None, output=None):
    """The user's control algorithm, loaded fresh from its file at every run
    start (edits apply on the next run). The entry is a class (instantiated,
    reset() called if present, then control() every frame) or a function:

        control(exports, t, dt) -> values for the CarSim imports, .sim order
        control(exports, t, dt, scene) -> the same, also given what is around
                                          the car (scene.py; scene() returns it)

    exports is {name: value} of every CarSim export, in CarSim units.
    See controllers/example_controller.py. output: an AlgoOutput that gets
    what the algorithm prints (loading, reset(), every control() call) and
    its traceback. The returned driver's .ms is how long the last
    control() call took (ms).
    """
    global _last_folder
    c = d["run"]["controller"]
    path = os.path.abspath(c["path"])
    if not os.path.isfile(path):
        raise ValueError("控制算法文件不存在：%s" % path)
    # "Reloaded every run" also for the helper modules it imports from its
    # folder and sub-folders, and none of a previous algorithm folder's
    # modules (e.g. its utils.py) stays in the way.
    folder = os.path.dirname(path)
    for old in {folder, _last_folder} - {None}:
        for name, m in list(sys.modules.items()):
            f = getattr(m, "__file__", None)
            if isinstance(f, str) and _own_file(f, old):
                del sys.modules[name]
    # Let the algorithm import its neighbours (before anything else) and
    # python_carsim_env modules.
    if _last_folder not in (None, folder) and _last_folder not in _START_PATH and _last_folder in sys.path:
        sys.path.remove(_last_folder)
    _last_folder = folder
    repo = os.path.abspath(d["carsim"]["repo_path"])
    if repo not in sys.path:
        sys.path.insert(0, repo)
    if folder in sys.path:
        sys.path.remove(folder)
    sys.path.insert(0, folder)
    importlib.invalidate_caches()  # files added since the last run
    clash = _name_clashes(path)
    if clash:
        name, own, other = clash[0]
        raise RuntimeError("你的 %s 和后端已经加载的同名模块（%s）冲突：import %s 拿到的是那个模块，不是你的文件。"
                           "请把 %s 改个名字（import 语句一起改）" % (own, other or "Python 内置模块", name, own))
    spec = importlib.util.spec_from_file_location("user_controller", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["user_controller"] = mod  # dataclasses & co. look the module up here
    entry = c.get("entry") or "Controller"
    cap = contextlib.nullcontext() if output is None else output
    if output is not None:
        output.path = path
    try:
        with cap:
            spec.loader.exec_module(mod)
            obj = getattr(mod, entry, None)
            if obj is None:
                found = ["%s（%s）" % (n, "类" if isinstance(v, type) else "函数") for n, v in vars(mod).items()
                         if not n.startswith("_") and (n == "control" and callable(v) or
                                                       isinstance(v, type) and callable(getattr(v, "control", None)))]
                raise ValueError("%s 里没有找到 %s；%s" % (
                    os.path.basename(path), entry, "找到了：%s——把“入口”改成其中一个" % "、".join(found) if found
                    else "“入口”要填带 control() 方法的类名，或 control(exports, t, dt) 这样的函数名"))
            if isinstance(obj, type):
                obj = obj()
                if hasattr(obj, "reset"):
                    obj.reset()
                obj = obj.control
    except SystemExit as e:  # e.g. argparse at module level: must not end the backend
        raise RuntimeError("控制算法 %s 在加载时调用了 sys.exit（常见原因：模块顶层用了 argparse）：%s%s"
                           % (os.path.basename(path), e, _where(_tb(e), path)))
    except ValueError as e:
        if "没有找到" in str(e):
            raise
        raise RuntimeError(_user_error("加载控制算法出错", e, path)) from e
    except Exception as e:
        raise RuntimeError(_user_error("加载控制算法出错", e, path)) from e
    dt = d["sync"]["frame_dt"]
    names = list(ex.index)
    with_scene = _wants_scene(obj)

    def control(obs, t):
        exports = {n: ex.raw(obs, n) for n in names}
        control.running = (time.time(), threading.get_ident())  # for control_busy()
        try:
            args = (exports, t, dt, scene() if scene else None) if with_scene else (exports, t, dt)
            with cap:
                t0 = time.perf_counter()
                out = obj(*args)
                control.ms = (time.perf_counter() - t0) * 1000.0  # the algorithm's own time (GUI: 算法耗时)
        except KeyError as e:
            k = e.args[0] if len(e.args) == 1 and isinstance(e.args[0], str) else None
            no_export = "导出变量里没有 %r（导出变量在“CarSim 动力学”页设置，现有：%s）" % (
                k, "、".join(names[:12]) + (" ..." if len(names) > 12 else ""))
            why = _scene_miss(k, d.get("scene") or {}, (d.get("rig") or {}).get("sensors") or ()) \
                if with_scene and k is not None else ""
            if why:  # scene objects / lane only have the keys ticked for the algorithm
                hint = "——场景里没有 %r：%s" % (k, why)
                if k in EGO_KEYS and k not in exports:  # could also be meant as a CarSim export
                    hint += "；" + no_export
            else:
                hint = "——" + no_export if k is not None and k not in exports else ""
            raise RuntimeError(_user_error("控制算法出错", e, path) + hint) from e
        except SystemExit as e:  # sys.exit() / argparse in control(): must not end the backend
            raise RuntimeError("控制算法在 control() 里调用了 sys.exit(%s)%s"
                               % ("" if e.code is None else repr(e.code), _where(_tb(e), path))) from e
        except Exception as e:
            raise RuntimeError(_user_error("控制算法出错", e, path)) from e
        finally:
            control.running = None
        n = n_imports() if n_imports else None
        want = "按 .sim 里导入变量的顺序返回 %s 个数，例如 [油门, 制动, 方向盘角]" % (n or "若干")
        if out is None:
            raise RuntimeError("控制算法的 control() 返回了 None（是不是忘了 return？）；应%s" % want)
        if isinstance(out, (dict, str, bytes)):
            raise RuntimeError("控制算法的 control() 返回了 %s；应%s" % (type(out).__name__, want))
        try:
            vals = [float(v) for v in out]
        except (TypeError, ValueError):
            raise RuntimeError("控制算法的 control() 返回值不是一组数字：%s；应%s" % (repr(out)[:80], want))
        if n and len(vals) != n:
            # CarSim would silently fill missing imports with 0 (e.g. no steering).
            raise RuntimeError("控制算法返回了 %d 个值，但 .sim 里有 %d 个导入变量；应%s" % (len(vals), n, want))
        return vals
    control.path, control.running, control.ms = path, None, None
    return control


def start_scene(ses, anchor, ref_local=None, t=0.0, ego_velocity=None):
    """The SceneProvider and the run record of a session, then the run's first
    tick: step 0 (time t) is the first scene the algorithm gets and the first
    sample of the records. The rig sensors run when the algorithm asked for
    some of them or data is being collected (one set for both). Returns the
    telemetry fields of step 0 (scene_step)."""
    d = ses.d
    rp = d["sync"]["reference_point"]
    if ref_local is None:
        ref_local = front_axle_local(ses.vehicle) if rp == "front_axle" else [float(v) for v in rp]
    ref = [float(v) for v in ref_local]
    rig = d["rig"]["sensors"] or rigmod.build_preset(d["rig"]["preset"], rigmod.spec_of(ses.vehicle), ref)
    if d["rig"]["sensors"] and d["rig"].get("frame") == "carla":  # an older config: car centre, y right
        rig = [rigmod.to_carsim(s, ref) for s in rig]
    rig = [c for c in rig if c.get("enabled", True)]
    wanted = set(d["scene"].get("sensors") or [])
    sensors = rig if d["collect"].get("enabled") else [c for c in rig if c["name"] in wanted]
    sp = SceneProvider(ses.world, ses.vehicle, d, anchor, ref_local, sensors)
    try:
        sp.start()
    except BaseException:
        sp.stop()
        raise
    ses.scene = sp
    if d["run"]["log_path"]:
        ses.recorder = Recorder(Recorder.run_paths(d["run"]["log_path"]), sp.s, ses.export_names(), ses.n_actions(),
                                min_free_gb=DISK_RESERVE_GB)
    # Sensors spawned before a tick deliver that frame: the first scene has their data.
    return scene_step(ses, ses.world.tick(), t, ego_velocity)


def scene_step(ses, world_frame, t, ego_velocity=None):
    """After a tick: update the scene, record a sample, handle contacts.
    Samples are every N run steps from step 0 (t0, t0 + N dt, ...), like
    CarSim's output interval; world_frame is CARLA's own counter (not tied to
    the run) and only names them. Returns telemetry fields."""
    scene = ses.scene.update(world_frame, t, ego_velocity)
    every = st.sample_every(ses.d)
    warning = ""
    if ses.recorder is not None and ses.frame % every == 0:  # the data collector samples the same steps
        ses.recorder.write(ses.scene.record_view(), ses.exports(), ses.last_action)
        if ses.recorder.stopped:  # the disk is (nearly) full: the run goes on without its record
            warning, ses.recorder = ses.recorder.stopped, None
    new = [c for c in scene["collisions"] if c["new"]]
    policy = ses.d["scene"].get("collision", "log")
    if policy == "off":
        new = []
    elif new and policy == "stop" and not ses.end_reason:
        c = new[0]
        ses.end_reason = "碰撞：撞到 %s（id %s）" % (c["model"], c["id"])
    gv = gui_view(scene, ses.scene._ego_box)
    if policy == "off":
        gv["collisions"] = []  # not checked: nothing to show either
    out = {"scene": gv, "collisions": new}
    if warning:
        out["warning"] = warning  # for the log, once
    return out


class WallClock:
    """Wall time of a run without its pauses, for the real-time factor."""

    def __init__(self):
        self._t0, self._paused_at = time.perf_counter(), None

    def pause(self):
        if self._paused_at is None:
            self._paused_at = time.perf_counter()

    def resume(self):
        if self._paused_at is not None:
            self._t0 += time.perf_counter() - self._paused_at
            self._paused_at = None

    def elapsed(self):
        return (time.perf_counter() if self._paused_at is None else self._paused_at) - self._t0


class CoSimSession:
    """Lock-step CarSim + CARLA run for one already-spawned vehicle."""

    def __init__(self, world, vehicle, anchor, d):
        self.world, self.vehicle, self.anchor, self.d = world, vehicle, anchor, d
        self.env = self.sync = self.driver = None
        self.recorder = None
        self._original_settings = None
        self.frame = 0
        self.done = False
        self.scene = None
        self.end_reason = ""
        self.last_action = None  # what control() returned last (CarSim imports)
        self.state = None        # the SyncedState of the current step
        self.algo_out = AlgoOutput()  # what the user's algorithm prints, for the GUI
        # How long the user's control() took: calls, sum, longest and its t (ms, s).
        self.ctrl_n, self.ctrl_ms_sum, self.ctrl_ms_max, self.ctrl_ms_max_t = 0, 0.0, 0.0, 0.0

    # --------------------------------------------------------------- lifecycle
    def start(self):
        d, w = self.d, self.world
        # CarSim first: its t_step sets the frame period everything below uses.
        self.env = make_env(d)
        self.obs = reset_env(self.env)
        # The .sim defines how many exports / imports there are, in which order.
        n_exp, n_imp = self.env.config.get("n_export"), self.env.config.get("n_import")
        names = d["carsim"]["export_names"]
        if n_exp and len(names) != int(n_exp):
            raise RuntimeError("导出变量个数不一致：界面里列了 %d 个，.sim 里有 %d 个。顺序和个数必须与 .sim 的导出变量一致"
                               "（“CarSim 动力学”页），否则位姿会用错变量" % (len(names), int(n_exp)))
        if d["run"]["driver"] != "custom" and n_imp and int(n_imp) != 3:
            raise RuntimeError("测试用驾驶方式只给 3 个导入变量（油门、制动、方向盘角），.sim 里有 %d 个；"
                               "请用你自己的控制算法按 .sim 的导入顺序返回" % int(n_imp))
        self._check_exports(self.obs, at_start=True)
        t_step = float(self.env.config["t_step"])
        if not t_step > 0:
            raise RuntimeError("CarSim 给出的 t_step = %r，无法运行" % t_step)
        frame_dt = d["sync"]["frame_dt"]
        self.inner = max(1, int(round(frame_dt / t_step)))
        # Never past CARLA's longest frame (run_cosim and the backend refuse a longer frame_dt first).
        self.inner = min(self.inner, max(1, int((MAX_FRAME_DT + 1e-9) / t_step)))
        if abs(self.inner * t_step - frame_dt) > 1e-9:
            # CarSim advances whole t_steps: CARLA's frame, the controller's dt
            # and the frame count all use that period, so the clocks agree.
            frame_dt = d["sync"]["frame_dt"] = round(self.inner * t_step, 12)
        self._original_settings = w.get_settings()
        s = w.get_settings()
        s.synchronous_mode = True
        s.fixed_delta_seconds = frame_dt
        w.apply_settings(s)

        ext = d["sync"]["use_external_api"]
        self.sync = CarlaVehicleSync(
            w, self.vehicle, self.anchor,
            use_external_api=None if ext == "auto" else bool(ext),
            settings=st.to_bridge_cfg(d))
        drv = d["run"]["driver"]
        self.command_driver = None
        self._speed = 0.0
        if drv in ("route", "manual"):
            dr = d["drive"]
            if drv == "manual":  # the route is planned below, from CarSim's t0 pose
                self.command_driver = ManualDriver()
            sw_max = float(d["sync"]["steering_wheel_max_deg"])
            scale = float(dr.get("brake_scale", 1.0))
            self.driver = lambda obs, t: self.command_driver.step(
                self.vehicle, self._speed, frame_dt).to_carsim(sw_max, scale)
        else:
            self.driver = make_driver(d, self.sync.ex, lambda: self.env.config.get("n_import"),
                                      lambda: self.scene.view(), self.algo_out)
        folder = os.path.dirname(getattr(self.driver, "path", ""))
        if folder:  # make_env put python_carsim_env first: the algorithm's own modules go before it again
            if folder in sys.path:
                sys.path.remove(folder)
            sys.path.insert(0, folder)
        # CarSim's origin is the spawn point: a .sim that starts elsewhere (e.g.
        # at a road station) puts the car that far from the spawn point.
        ex = self.sync.ex
        x0, y0, yaw0 = ex.raw(self.obs, "Xo"), ex.raw(self.obs, "Yo"), ex.angle(self.obs, "Yaw")
        self.warnings = []
        dist, dyaw = math.hypot(x0, y0), abs((yaw0 + 180.0) % 360.0 - 180.0)
        off = (["车会从离出生点 %.1f m 的地方出发" % dist] if dist > 5.0 else []) + \
              (["车头方向与出生点方向差 %.0f°" % dyaw] if dyaw > 10.0 else [])
        if off:
            self.warnings.append("CarSim 的初始位姿不在原点（Xo = %.1f m，Yo = %.1f m，Yaw = %.1f°）：CarSim 原点放在出生点上、"
                                 "x 轴沿出生点方向，%s。想从出生点沿出生点方向出发，把 .sim 里的初始位置和航向设为 0，"
                                 "或换一个出生点" % (x0, y0, yaw0, "，".join(off)))
        # Put the car at CarSim's t0 pose (relative to its origin, the spawn
        # point) before the first control() call, so its scene shows the real start.
        self.state = self.sync.sync(self.obs, self.env.t_current, frame_dt)
        t0 = self.env.t_current
        tel0 = start_scene(self, self.anchor, self.sync.ref_local, t0, self.state.velocity)
        if drv == "route":  # from where the car is now (CarSim's t0 pose), not from the spawn point
            dest = int(d["drive"].get("destination_index", -1))
            pts = w.get_map().get_spawn_points()
            self.command_driver = RouteFollower(
                w, self.vehicle, d["drive"]["target_speed_kmh"],
                destination=pts[dest].location if 0 <= dest < len(pts) else None)
        # duration <= 0: run until stopped (or until CarSim reaches t_stop).
        self.n_frames = max(1, int(round(d["sync"]["duration"] / frame_dt))) if d["sync"]["duration"] > 0 else 0
        self._t0 = self.env.t_current  # t_start of the .sim, not always 0
        # Only the count is known to match the .sim: warn when the values do
        # not look like the named variables (now, and once the car moves).
        # Height mode "ground" puts the car on the CARLA road: Zo is not checked.
        bb = self.vehicle.bounding_box
        z0 = float(self.sync.ref_local[2]) - (bb.location.z - bb.extent.z)
        self.export_check = ExportCheck(self.sync.ex, self.sync.wheel_radius_m,
                                        None if d["sync"]["z_mode"] == "ground" else z0)
        self.warnings += self.export_check(self.obs)
        self.clock = WallClock()
        self.done = bool(self.end_reason)  # e.g. touching something at t0 with "stop on collision": no step past it
        return {"external_api": self.sync.external_api, "server_api": self.sync.server_api,
                "reference_point": [round(float(x), 3) for x in self.sync.ref_local],
                "t_step": t_step, "inner_steps": self.inner, "frame_dt": frame_dt,
                "t_stop": float(self.env.config.get("t_stop") or 0.0), "mock": bool(d["carsim"]["mock"]),
                "warnings": self.warnings, "t": t0, "collisions": tel0["collisions"], "warning": tel0.get("warning", "")}

    def stop(self, release_vehicle=True):
        """Best effort: one failing step (CarSim or CARLA gone) must not skip the rest."""
        def step(fn):
            try:
                fn()
            except Exception as e:
                print("CoSimSession.stop: %s" % e, flush=True)
        if self.env is not None:
            step(self.env.close)
        if self.scene is not None:
            step(self.scene.stop)
        if self.recorder is not None:
            step(self.recorder.close)
            self.recorder = None
        if release_vehicle and self.sync is not None:
            step(self.sync.release)
        if self._original_settings is not None:
            orig, self._original_settings = self._original_settings, None
            step(lambda: self.world.apply_settings(orig))

    # -------------------------------------------------------------------- step
    def step(self):
        """Advance one CARLA frame. Returns a telemetry dict."""
        env, frame_dt = self.env, self.d["sync"]["frame_dt"]
        action = self.driver(self.obs, env.t_current)
        ms = getattr(self.driver, "ms", None)  # the user's control() only, not the test drivers
        if ms is not None:
            self.ctrl_n += 1
            self.ctrl_ms_sum += ms
            if ms >= self.ctrl_ms_max:
                self.ctrl_ms_max, self.ctrl_ms_max_t = ms, env.t_current
        if not all(math.isfinite(a) for a in action):
            raise RuntimeError("控制算法输出了无效数值（NaN / 无穷大）：%s" % list(action))
        self.last_action = [float(a) for a in action]
        prev, t_prev = self.obs, env.t_current
        self.obs, _, done, info = env.control_step(action, self.inner)
        # A non-zero return without a model stop is an error, with or without a message.
        if info.get("error") or (info.get("return_code") and "end_reason" not in info):
            raise RuntimeError("CarSim 报错（t = %.2f s）：%s" % (env.t_current, info.get("error") or "求解器没有给出错误信息"))
        self._check_exports(self.obs)
        if done and info.get("end_reason") and not self.end_reason:
            # CarSim also returns "stop" at TSTOP: only earlier is a stop of the model's own.
            t_stop, t_step = float(env.config.get("t_stop") or 0.0), float(env.config["t_step"])
            if not t_stop > 0 or env.t_current < t_stop - 1.5 * t_step:
                self.end_reason = "CarSim 模型请求停止（.sim 里的事件 / 停止条件），t = %.2f s，.sim 结束时间 %.2f s" % (
                    env.t_current, t_stop)
        warnings = []
        if not self.export_check.done and env.t_current - self._t0 >= 1.0:
            warnings = self.export_check(self.obs, prev, env.t_current - t_prev)
        state = self.state = self.sync.sync(self.obs, env.t_current, frame_dt)
        world_frame = self.world.tick()
        self.frame += 1
        # CarSim's velocity: the original CARLA reports 0 for the teleported car.
        scene_tel = scene_step(self, world_frame, env.t_current, state.velocity)
        v = state.velocity
        self._speed = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)

        snap = self.vehicle.get_transform()
        self.done = bool(done) or (self.n_frames > 0 and self.frame >= self.n_frames) or bool(self.end_reason)
        wall = self.clock.elapsed()
        v = state.velocity
        return {**scene_tel,
            "t": env.t_current,
            "frame": self.frame,
            "n_frames": self.n_frames,
            "rt_factor": (env.t_current - self._t0) / wall if wall > 0 else 0.0,
            "speed_kmh": math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2) * 3.6,
            "location": [snap.location.x, snap.location.y, snap.location.z],
            "rotation": [snap.rotation.pitch, snap.rotation.yaw, snap.rotation.roll],
            "wheel_steer": list(state.wheel_steer),
            "wheel_rotation": list(state.wheel_rotation),
            "wheel_suspension_mm": [x * 1000.0 for x in state.wheel_suspension],
            "action": [float(a) for a in action],
            **({"ctrl_ms": ms, "ctrl_ms_max": self.ctrl_ms_max} if ms is not None else {}),
            "warnings": warnings,
            "world_frame": world_frame,
            "dynamics": "CarSim",
            "done": self.done,
        }

    def _check_exports(self, obs, at_start=False):
        """Never hand NaN / inf to CARLA as a pose; stop with a clear reason."""
        bad = [n for n, v in zip(self.d["carsim"]["export_names"], obs) if not math.isfinite(float(v))]
        if bad and at_start:
            raise RuntimeError("CarSim 的初始状态里有无效数值（NaN / 无穷大）：%s。检查 .sim 的初始条件，"
                               "以及导出变量的顺序是否与 .sim 一致" % ", ".join(bad[:6]))
        if bad:
            raise RuntimeError("CarSim 输出了无效数值（NaN / 无穷大），仿真已停止：%s" % ", ".join(bad[:6]))

    def export_names(self):
        return list(self.d["carsim"]["export_names"])

    def exports(self):
        """CarSim exports of the current step, CarSim units."""
        ex = self.sync.ex
        return {n: ex.raw(self.obs, n) for n in ex.index}

    def n_actions(self):
        """How many values control() returns (the .sim's imports)."""
        return int(self.env.config.get("n_import") or 0)

    def ego_motion(self):
        """CarSim's velocity for ego/<frame>.json (CARLA world frame, m/s and
        deg/s, like get_velocity()): stock CARLA reads 0 for the teleported car."""
        v, w = self.state.velocity, self.state.angular_velocity
        return {"velocity": [v.x, v.y, v.z], "angular_velocity": [w.x, w.y, w.z]}


class CarlaDriveSession:
    """Same interface as CoSimSession, but CARLA PhysX drives the vehicle."""

    def __init__(self, world, vehicle, d, traffic_manager, anchor=None):
        self.world, self.vehicle, self.d, self.tm = world, vehicle, d, traffic_manager
        self.anchor = anchor  # the spawn point: CarSim's origin, also without CarSim
        self.frame = 0
        self.done = False
        self.command_driver = None
        self._original_settings = None
        self.scene = self.recorder = None
        self.end_reason = ""
        self.last_action = None  # no CarSim imports with CARLA dynamics

    def start(self):
        d, w = self.d, self.world
        dt = d["sync"]["frame_dt"]
        self._original_settings = w.get_settings()
        s = w.get_settings()
        s.synchronous_mode, s.fixed_delta_seconds = True, dt
        w.apply_settings(s)
        if self.tm is not None:  # only the autopilot needs the traffic manager
            self.tm.set_synchronous_mode(True)
        # Front wheel angles for the telemetry are computed the way PhysX does
        # (steer x max angle x speed curve for the inner wheel, Ackermann for
        # the outer one), not read with get_wheel_steer_angle(): that call
        # waits for the server holding the GIL (original carla package), which
        # deadlocks with the sensor callbacks of large live views.
        pc = self.vehicle.get_physics_control()
        self._steer_geo = None
        if len(pc.wheels) >= 4:
            inv = self.vehicle.get_transform().get_inverse_matrix()
            pos = [[sum(inv[r][k] * p[k] for k in range(4)) for r in range(2)]
                   for p in ([w.position.x / 100.0, w.position.y / 100.0, w.position.z / 100.0, 1.0] for w in pc.wheels[:4])]
            wheelbase = (pos[0][0] + pos[1][0]) / 2 - (pos[2][0] + pos[3][0]) / 2
            track = abs(pos[1][1] - pos[0][1])
            curve = sorted((p.x, p.y) for p in pc.steering_curve) or [(0.0, 1.0)]
            self._steer_geo = (pc.wheels[0].max_steer_angle, curve, wheelbase, track)
        tel0 = start_scene(self, self.anchor)  # t = 0, before any driver acts
        dr = d["drive"]
        self.mode = dr["carla_driver"]
        if self.mode == "autopilot":
            self.vehicle.set_autopilot(True, self.tm.get_port())
            self.tm.vehicle_percentage_speed_difference(self.vehicle, float(dr.get("tm_speed_diff_pct", 0.0)))
            self.tm.ignore_lights_percentage(self.vehicle, 100.0 if dr.get("tm_ignore_lights") else 0.0)
        elif self.mode == "route":
            dest = int(dr.get("destination_index", -1))
            pts = w.get_map().get_spawn_points()
            self.command_driver = RouteFollower(w, self.vehicle, dr["target_speed_kmh"],
                                                destination=pts[dest].location if 0 <= dest < len(pts) else None)
        else:
            self.command_driver = ManualDriver()
        self.n_frames = max(1, int(round(d["sync"]["duration"] / dt))) if d["sync"]["duration"] > 0 else 0
        self.clock = WallClock()
        self.done = bool(self.end_reason)  # step 0 already ended the run (collision stop)
        return {"external_api": False, "server_api": None, "reference_point": [0, 0, 0], "t_step": dt, "inner_steps": 1,
                "frame_dt": dt, "dynamics": "CARLA", "t": 0.0, "collisions": tel0["collisions"],
                "warning": tel0.get("warning", "")}

    def _wheel_angles(self, steer, speed_kmh):
        """[FL, FR] steer angle, deg, + = right (as get_wheel_steer_angle)."""
        if self._steer_geo is None or abs(steer) < 1e-4:
            return [0.0, 0.0]
        max_deg, curve, wheelbase, track = self._steer_geo
        k = curve[-1][1]
        for (x0, y0), (x1, y1) in zip(curve, curve[1:]):
            if speed_kmh <= x1:
                k = y0 + (y1 - y0) * max(0.0, speed_kmh - x0) / max(1e-6, x1 - x0)
                break
        if speed_kmh <= curve[0][0]:
            k = curve[0][1]
        inner = min(abs(steer), 1.0) * max_deg * k
        outer = math.degrees(math.atan(wheelbase / (wheelbase / math.tan(math.radians(inner)) + track))) \
            if wheelbase > 0 and inner > 1e-3 else inner
        # Turning right: the right wheel is the inner one.
        return [outer, inner] if steer > 0 else [-inner, -outer]

    def stop(self, release_vehicle=True):
        try:
            if getattr(self, "mode", None) == "autopilot":  # start() may have failed before setting it
                self.vehicle.set_autopilot(False, self.tm.get_port())
        except RuntimeError:
            pass
        if self.scene is not None:
            self.scene.stop()
        if self.recorder is not None:
            self.recorder.close()
            self.recorder = None
        if self._original_settings is not None:
            self.world.apply_settings(self._original_settings)
            self._original_settings = None

    def step(self):
        dt = self.d["sync"]["frame_dt"]
        v = self.vehicle.get_velocity()
        speed = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
        if self.command_driver is not None:
            self.vehicle.apply_control(self.command_driver.step(self.vehicle, speed, dt).to_carla())
        world_frame = self.world.tick()
        self.frame += 1
        t = self.frame * dt
        scene_tel = scene_step(self, world_frame, t)
        tf = self.vehicle.get_transform()
        c = self.vehicle.get_control()
        steer = self._wheel_angles(c.steer, speed * 3.6)
        self.done = (self.n_frames > 0 and self.frame >= self.n_frames) or bool(self.end_reason)
        wall = self.clock.elapsed()
        try:
            red = self.vehicle.is_at_traffic_light() and \
                self.vehicle.get_traffic_light_state() == carla.TrafficLightState.Red
        except RuntimeError:
            red = False
        return {**scene_tel, "t": t, "frame": self.frame, "n_frames": self.n_frames, "at_red_light": red,
                "rt_factor": t / wall if wall > 0 else 0.0, "speed_kmh": speed * 3.6,
                "location": [tf.location.x, tf.location.y, tf.location.z],
                "rotation": [tf.rotation.pitch, tf.rotation.yaw, tf.rotation.roll],
                "wheel_steer": steer + [0.0, 0.0], "wheel_rotation": [], "wheel_suspension_mm": [],
                "action": [c.throttle, c.brake, c.steer], "world_frame": world_frame,
                "dynamics": "CARLA", "done": self.done}

    def export_names(self):
        return []

    def exports(self):
        return {}

    def n_actions(self):
        return 0
