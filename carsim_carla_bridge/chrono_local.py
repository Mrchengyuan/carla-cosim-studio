"""The PyChrono BMW E90 in place of CarSim on this machine (carsim.chrono,
the CarSim page's "Chrono 宝马"): the backend starts carsim_service.py
--chrono itself, in a Python that has pychrono (the "chrono" conda env),
and runs it through a CarSim service link of its own (carsim_remote's
ServiceLink on a free 127.0.0.1 port): so it works the same with or without
remote mode, and never takes the place of a Windows computer's CarSim
service. The service process lives as long as this backend (Linux:
PDEATHSIG) and serves one run after the other; its initial speed comes with
each run (carsim.chrono_init_speed).
"""

import glob
import os
import subprocess
import sys
import threading
import time

import carsim_remote

HERE = os.path.dirname(os.path.abspath(__file__))
CONNECT_TIMEOUT = 60.0   # s: the first start imports pychrono
LOG = os.path.join(HERE, "chrono_service.log")  # gitignored like backend.log

_lock = threading.Lock()
_link = None
_port = None
_proc = None


def find_python(given=""):
    """A Python with pychrono: carsim.chrono_python, $CHRONO_PYTHON, else a conda env named chrono."""
    cands = [given, os.environ.get("CHRONO_PYTHON", "")]
    for root in ("/opt/anaconda3", "/opt/miniconda3", "~/anaconda3", "~/miniconda3", "~/miniforge3", "~/.conda"):
        cands.append(os.path.join(os.path.expanduser(root), "envs", "chrono", "bin", "python"))
    cands += sorted(glob.glob(os.path.expanduser("~/*/envs/chrono/bin/python")))
    for p in cands:
        if p and os.path.isfile(p) and os.access(p, os.X_OK):
            return p
    raise ValueError("找不到装了 PyChrono 的 Python（Chrono 宝马要用）：请建 conda 环境 chrono 并装 pychrono 10.0"
                     "（conda create -n chrono -c projectchrono -c conda-forge pychrono=10.0.0 python=3.12），"
                     "或在“CarSim 动力学”页填它的 python 路径")


def _pdeathsig():
    """The service ends with this backend (Linux)."""
    try:
        import ctypes
        import signal
        ctypes.CDLL("libc.so.6").prctl(1, signal.SIGTERM)  # PR_SET_PDEATHSIG
    except Exception:
        pass


def link(d):
    """The link to a running Chrono service, started (and waited for) if
    needed. ValueError / RuntimeError in plain words."""
    global _link, _port, _proc
    with _lock:
        if _link is None:
            _link = carsim_remote.ServiceLink()
            _port = _link.listen(0)
        if _link.status()["connected"] and _proc is not None and _proc.poll() is None:
            return _link
        py = find_python((d.get("carsim") or {}).get("chrono_python", ""))
        if _proc is not None and _proc.poll() is None:
            _proc.terminate()
        log = open(LOG, "a", encoding="utf-8")
        log.write("\n==== %s  %s\n" % (time.strftime("%Y-%m-%d %H:%M:%S"), py))
        log.flush()
        _proc = subprocess.Popen([py, "-u", os.path.join(HERE, "carsim_service.py"), "--chrono", "--port", str(_port)],
                                 cwd=HERE, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                 preexec_fn=_pdeathsig if sys.platform.startswith("linux") else None)
        log.close()
        end = time.time() + CONNECT_TIMEOUT
        while time.time() < end:
            if _link.status()["connected"]:
                return _link
            if _proc.poll() is not None:
                break
            time.sleep(0.2)
        tail = ""
        try:
            with open(LOG, encoding="utf-8", errors="replace") as f:
                tail = "".join(f.readlines()[-6:]).strip()
        except OSError:
            pass
        raise RuntimeError("Chrono 宝马没有启动起来（%s）%s" % (
            "进程已退出" if _proc.poll() is not None else "%g 秒内没有连上" % CONNECT_TIMEOUT,
            "：\n" + tail if tail else "；详见 %s" % LOG))


def stop():
    """Ends the service process (backend exit)."""
    global _proc
    with _lock:
        if _proc is not None and _proc.poll() is None:
            _proc.terminate()
            try:
                _proc.wait(10)
            except subprocess.TimeoutExpired:
                _proc.kill()
        _proc = None
