"""CarSim service: CarSim on this computer for a co-simulation whose CARLA,
backend and control algorithm run on a server (remote mode, carsim.remote).

It connects out to the backend (on Windows through the SSH tunnel the
launcher opens: 127.0.0.1:57121 here is the server's 127.0.0.1:57121; this
computer never listens), and then runs the CarSim steps the backend asks
for, one round trip per CARLA frame. It reconnects by itself; Ctrl+C ends it.

    python carsim_service.py [--host 127.0.0.1] [--port 57121] [--mock]

--mock: the simple built-in vehicle model instead of CarSim, whatever the
run's config says (tests).

Messages (JSON, one per line, UTF-8; floats that are not finite are null):
  service -> backend  {"type": "hello", "role": "carsim", "protocol": 2, "platform", "host", "python"}
  backend -> service  {"type": "hello", "ok": true, "protocol": 2}     (ok false + "error": closed)
  backend -> service  {"id": n, "cmd": "check", "carsim": {the run's carsim config}, "duration": s}
                          -> {}  (.sim, python_carsim_env and solver checked here before the backend
                                  changes its world; with the mock nothing to check)
                      {"id": n, "cmd": "open", "carsim": {...}, "duration": s}
                          -> {"config": CarSimEnv.config, "sim_path": the .sim here}
                      {"id": n, "cmd": "reset"}  -> {"obs": [...], "t_current": s, "config": {...}}
                      {"id": n, "cmd": "step", "action": [...], "inner": k}
                          -> {"obs": [...], "done": bool, "info": {...}, "t_current": s}
                      {"id": n, "cmd": "close"}  -> {}
                      {"id": n, "cmd": "ping"}   -> {}  (the backend, after 5 s without a request)
  replies             {"id": n, "ok": true, "result": ...} or {"id": n, "ok": false, "error": "<中文>"}
The values are CarSim's own (its frames and units, the .sim's export order):
nothing here converts them.

Only the standard library and carsim_local / mock_carsim next to this file
(python_carsim_env and numpy for real CarSim); Python 3.8 or newer.
"""

import argparse
import platform
import socket
import sys
import time

from carsim_local import SERVICE_PROTOCOL, JsonLines, check_carsim, make_env, reset_env

RETRY = 2.0          # s between connection attempts
HELLO_TIMEOUT = 10.0  # for the backend's answer to the hello

_state = [None]  # the connection state line printed last


def state(line):
    """A connection state, printed when it changes (not at every try)."""
    if line != _state[0]:
        _state[0] = line
        print(line, flush=True)


class CarSim:
    """CarSim of the backend's current run; stays open between its requests."""

    def __init__(self, mock):
        self.mock, self.env = mock, None

    def _cfg(self, carsim, duration):
        """The run's config for carsim_local (--mock: the mock, whatever the run says)."""
        return {"carsim": dict(carsim, mock=bool(carsim.get("mock")) or self.mock),
                "sync": {"duration": float(duration)}}

    def check(self, carsim, duration):
        """The run's pre-flight here, as check_run_files does with CarSim on the server."""
        d = self._cfg(carsim, duration)
        if not d["carsim"]["mock"]:
            check_carsim(d, service=True)
        return {}

    def open(self, carsim, duration):
        self.close()
        self.env = make_env(self._cfg(carsim, duration), service=True)
        sim = str(getattr(self.env, "sim_path", "") or "")
        print("运行开始：%s" % (sim or "模拟 CarSim"), flush=True)
        return {"config": self.env.config, "sim_path": sim}

    def reset(self):
        obs = reset_env(self._env())  # CarSim's own reason when the .sim cannot run
        return {"obs": list(obs), "t_current": self.env.t_current, "config": self.env.config}

    def step(self, action, inner):
        obs, _, done, info = self._env().control_step(action, int(inner))
        return {"obs": list(obs), "done": bool(done), "info": info, "t_current": self.env.t_current}

    def close(self):
        if self.env is not None:
            env, self.env = self.env, None
            try:
                env.close()
            finally:
                print("运行结束", flush=True)
        return {}

    def _env(self):
        if self.env is None:
            raise RuntimeError("CarSim 服务上没有打开的运行（云端的后端没有先发 open）")
        return self.env

    def handle(self, req):
        cmd = req.get("cmd")
        if cmd == "ping":
            return {}
        if cmd == "check":
            return self.check(req["carsim"], req.get("duration", 0.0))
        if cmd == "open":
            return self.open(req["carsim"], req.get("duration", 0.0))
        if cmd == "reset":
            return self.reset()
        if cmd == "step":
            return self.step(req["action"], req["inner"])
        if cmd == "close":
            return self.close()
        raise RuntimeError("CarSim 服务不认识命令 %r：Windows 上的服务和云端的后端版本不一致" % cmd)


def serve(lines, carsim, where):
    """Hello, then the backend's requests until the connection ends. False
    when there was no backend to take the hello (e.g. the tunnel is up but
    the backend is not)."""
    lines.send({"type": "hello", "role": "carsim", "protocol": SERVICE_PROTOCOL, "platform": sys.platform,
                "host": socket.gethostname(), "python": platform.python_version()})
    try:
        hello = lines.recv(HELLO_TIMEOUT)
    except (OSError, ValueError):
        return False
    if not isinstance(hello, dict):
        return False
    if not hello.get("ok"):
        state(str(hello.get("error") or "云端的后端拒绝了连接"))
        return False
    state("已连上云端（%s），等待运行" % where)
    while True:
        try:
            req = lines.recv(0.5)  # back in Python twice a second: Ctrl+C on Windows
        except socket.timeout:
            continue
        except (OSError, ValueError):
            return True
        if req is None:
            return True
        if not isinstance(req, dict):
            continue
        try:
            reply = {"id": req.get("id"), "ok": True, "result": carsim.handle(req)}
        except Exception as e:
            err = str(e) or type(e).__name__
            print(err, flush=True)
            reply = {"id": req.get("id"), "ok": False, "error": err}
        try:
            lines.send(reply)
        except OSError:
            return True


def run(host, port, mock):
    """Connect, serve, reconnect, until Ctrl+C."""
    carsim = CarSim(mock)
    state("正在连接云端…")
    while True:
        try:
            sock = socket.create_connection((host, port), timeout=RETRY)
        except OSError:
            time.sleep(RETRY)
            continue
        try:
            served = serve(JsonLines(sock), carsim, "%s:%d" % (host, port))
        except OSError:
            served = False
        finally:
            try:
                carsim.close()  # a run the backend can no longer step
            except Exception as e:
                print(str(e) or type(e).__name__, flush=True)
            sock.close()
        if served:
            state("连接断开，正在重连…")
        time.sleep(RETRY)


def main():
    for stream in (sys.stdout, sys.stderr):  # Chinese lines also into a redirected log (cp936 on Windows)
        if stream is not None and hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="backslashreplace")
    ap = argparse.ArgumentParser(description="CarSim 服务：远程模式下在这台电脑上运行 CarSim")
    ap.add_argument("--host", default="127.0.0.1", help="the backend (through the SSH tunnel: 127.0.0.1)")
    ap.add_argument("--port", type=int, default=57121, help="the backend's CarSim service port")
    ap.add_argument("--mock", action="store_true", help="the built-in vehicle model instead of CarSim")
    a = ap.parse_args()
    try:
        run(a.host, a.port, a.mock)
    except KeyboardInterrupt:
        print("已退出", flush=True)


if __name__ == "__main__":
    main()
