"""CarSim on this computer, and the wire format of the CarSim service link.

Shared by the backend (session.py) and the CarSim service
(carsim_service.py, which runs CarSim on the user's Windows computer in
remote mode, carsim.remote): only the standard library, so the service
imports it without carla or numpy. mock_carsim and python_carsim_env are
imported when a run needs them.
"""

import ctypes
import importlib
import json
import math
import os
import socket
import sys
import time

# What python_carsim_env's get_api reads from the solver (a missing one is a
# bare AttributeError there).
_VS_API = ("vs_run", "vs_initialize", "vs_read_configuration", "vs_integrate_io", "vs_copy_export_vars",
           "vs_terminate_run", "vs_error_occurred", "vs_set_opt_error_dialog", "vs_get_error_message", "vs_road_l")


class RemoteCarSimError(RuntimeError):
    """An error of the CarSim service (or of the link to it), already in the
    user's words: passed on as it is, never wrapped again."""


def find_repo(repo_path, sim_path, path=os.path, isfile=None):
    """The python_carsim_env folder: the configured one when it holds
    carsim_env.py, else the .sim's folder or the nearest folder above it that
    does (the .sim usually sits in python_carsim_env). None: not found.
    path: os.path, or ntpath / posixpath to check another system's rules."""
    isfile = isfile or path.isfile
    if repo_path:
        repo = path.abspath(repo_path)
        if isfile(path.join(repo, "carsim_env.py")):
            return repo
    folder = path.dirname(path.abspath(sim_path)) if sim_path else ""
    while folder:
        if isfile(path.join(folder, "carsim_env.py")):
            return folder
        up = path.dirname(folder)
        if up == folder:
            break
        folder = up
    return None


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


def check_carsim(d, service=False):
    """The .sim, python_carsim_env and the solver the .sim names, checked in
    plain words: python_carsim_env reports these as a bare TypeError,
    AttributeError or ModuleNotFoundError. Returns (.sim path, carsim_env).
    service: checked by the CarSim service (remote mode), not the backend."""
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
    repo = find_repo(c["repo_path"], sim)
    if repo is None:
        raise ValueError("python_carsim_env 目录不对：%s 里没有 carsim_env.py，.sim 所在的文件夹（%s）和它的上层文件夹里也没有；"
                         "请在“CarSim 动力学”页填 python_carsim_env 的位置" % (
                             os.path.abspath(c["repo_path"]) if c["repo_path"] else "（未填）", os.path.dirname(sim)))
    carsim_env = _carsim_module(repo)
    try:
        dll = importlib.import_module("vs_solver").vs_solver().get_dll_path(sim)
    except Exception as e:
        raise ValueError("读不了 .sim 文件 %s：%s: %s" % (sim, type(e).__name__, e))
    if not dll or not os.path.isfile(dll):
        rel = [v for v in (value("PROGDIR"), value(lib_key)) if v and not os.path.isabs(v)]
        raise ValueError("找不到 CarSim 求解器：%s（由 .sim 里的 PROGDIR / %s 得出%s）" % (
            dll, lib_key, "；其中的相对路径 %s 是按%s的工作目录 %s 解析的，请在 .sim 里写绝对路径"
            % (" ".join(rel), " CarSim 服务" if service else "后端", os.getcwd()) if rel else ""))
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


def mock_env(d):
    """The stand-in for CarSim (carsim.mock), in the units of the CarSim page."""
    from mock_carsim import MockCarSimEnv
    c, dur = d["carsim"], d["sync"]["duration"]
    return MockCarSimEnv(c["export_names"], t_stop=dur + 1.0 if dur > 0 else 1e9, units=c["units"])


def open_carsim(sim, carsim_env):
    """python_carsim_env's CarSimEnv on a .sim that check_carsim passed."""
    try:
        return carsim_env.CarSimEnv(sim)
    except Exception as e:
        raise RuntimeError("加载 CarSim 失败：%s: %s" % (type(e).__name__, e)) from e


def make_env(d, service=False):
    """CarSim on this computer: the mock, else the checked .sim (service: see check_carsim)."""
    if d["carsim"]["mock"]:
        return mock_env(d)
    return open_carsim(*check_carsim(d, service))


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
    except RemoteCarSimError:
        raise  # the CarSim service ran reset_env already
    except Exception as e:
        raise RuntimeError(_reset_failed(env, e)) from e


# ---------------------------------------------------------- the service link
# Backend (carsim_remote.py) <-> CarSim service (carsim_service.py): JSON
# lines, UTF-8. Raise with any change of their messages.
SERVICE_PROTOCOL = 2


def json_safe(o):
    """Replace non-finite floats with None and unknown objects with their str()."""
    if isinstance(o, float):
        return o if math.isfinite(o) else None
    if isinstance(o, dict):
        return {str(k): json_safe(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [json_safe(v) for v in o]
    if o is None or isinstance(o, (str, int, bool)):
        return o
    try:
        return json_safe(float(o))  # numpy scalars
    except (TypeError, ValueError):
        return str(o)


class JsonLines:
    """One end of the link: a JSON object per line. TCP_NODELAY: a small
    message each way every CARLA frame must not wait for more data."""

    def __init__(self, sock):
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock, self._buf = sock, b""

    def send(self, msg):
        # NaN / inf are no JSON: null (the backend reads null back as nan).
        self.sock.sendall((json.dumps(json_safe(msg), ensure_ascii=False, allow_nan=False) + "\n").encode("utf-8"))

    def recv(self, timeout=None):
        """The next message; None when the other end closed. socket.timeout
        when none is complete within timeout s (what came so far is kept)."""
        end = None if timeout is None else time.monotonic() + timeout
        while b"\n" not in self._buf:
            left = None if end is None else end - time.monotonic()
            if left is not None and left <= 0:
                raise socket.timeout("timed out")
            self.sock.settimeout(left)
            chunk = self.sock.recv(65536)
            if not chunk:
                return None
            self._buf += chunk
        line, self._buf = self._buf.split(b"\n", 1)
        return json.loads(line.decode("utf-8"))
