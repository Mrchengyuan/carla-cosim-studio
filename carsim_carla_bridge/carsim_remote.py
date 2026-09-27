"""The backend's end of the CarSim service, for carsim.remote: CarSim runs
on the user's Windows computer (carsim_service.py), CARLA, the backend and
the control algorithm on the server.

The service connects out to the backend (through the SSH tunnel to the
server's 127.0.0.1:57121; the Windows computer never listens), and the
backend asks it for one round of CarSim steps per CARLA frame. The messages
are in carsim_service.py. SERVICE is the backend's listening end;
RemoteCarSimEnv has the surface of python_carsim_env's CarSimEnv that
session.py uses, so the run, the algorithm and the records see CarSim's own
values (frames, units, the .sim's export order) exactly as with CarSim here.
"""

import os
import queue
import socket
import threading
import time

from carsim_local import SERVICE_PROTOCOL, JsonLines, RemoteCarSimError

NO_SERVICE = ("Windows 上的 CarSim 服务没有连上云端：请在 Windows 上双击启动脚本（启动远程仿真.bat），"
              "看到“已连上云端”后再点运行")
LOST = "与 Windows 上的 CarSim 服务的连接断开了（%s）"
# How long the backend waits for an answer, s: loading CarSim and starting its run can be slow.
TIMEOUTS = {"check": 120.0, "open": 120.0, "reset": 120.0, "step": 30.0, "close": 10.0, "ping": 10.0}
# s without a request before the backend pings the service: a network that died
# without a word (no FIN / RST) is noticed between runs too.
PING = 5.0
HELLO_TIMEOUT = 10.0  # for the first line of a new connection
RETRY = 2.0           # s between tries to listen on a port in use


class _Conn:
    """One connection of the service, after its hello. A thread reads it and
    another pings it when idle, so a service that goes away is noticed
    between runs too (world_info)."""

    def __init__(self, sock, lines, hello, on_close):
        self.sock, self.lines, self._on_close = sock, lines, on_close
        self.host, self.platform = str(hello.get("host") or ""), str(hello.get("platform") or "")
        self.why = ""  # why it closed, for LOST
        self.closed = threading.Event()
        self._replies = queue.Queue()
        self._lock = threading.Lock()  # one request at a time, pings included
        self._n = 0
        self._last = time.monotonic()  # when the last request ended

    def start(self):
        threading.Thread(target=self._read, daemon=True).start()
        threading.Thread(target=self._ping, daemon=True).start()

    def _read(self):
        why = "Windows 上的服务已退出，或网络断了"
        try:
            while True:
                msg = self.lines.recv()
                if msg is None:
                    break
                self._replies.put(msg)
        except (OSError, ValueError) as e:
            why = "网络出错：%s" % e if isinstance(e, OSError) else "收到无法解析的消息：%s" % e
        self.close(why)

    def close(self, why):
        if self.closed.is_set():
            return
        self.why = why
        self.closed.set()
        self._replies.put(None)  # wakes a request waiting for its reply
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.sock.close()
        self._on_close(self)

    def request(self, cmd, **args):
        """The service's result for cmd; RemoteCarSimError with its own words
        when it failed, with LOST when the connection is gone or it did not
        answer in time (the connection is closed then: the service starts over)."""
        with self._lock:
            try:
                return self._request(cmd, args)
            finally:
                self._last = time.monotonic()

    def _request(self, cmd, args):
        timeout = TIMEOUTS[cmd]
        if self.closed.is_set():
            raise RemoteCarSimError(LOST % self.why)
        self._n += 1
        try:
            self.lines.send(dict(args, id=self._n, cmd=cmd))
        except OSError as e:
            self.close("网络出错：%s" % e)
            raise RemoteCarSimError(LOST % self.why)
        end = time.monotonic() + timeout
        while True:
            try:
                msg = self._replies.get(timeout=max(0.0, end - time.monotonic()))
            except queue.Empty:
                self.close("%g s 没有回应" % timeout)
                raise RemoteCarSimError(LOST % self.why)
            if msg is None:
                raise RemoteCarSimError(LOST % self.why)
            if isinstance(msg, dict) and msg.get("id") == self._n:
                break
        if not msg.get("ok"):
            raise RemoteCarSimError(str(msg.get("error") or "Windows 上的 CarSim 服务出错，没有给出原因"))
        return msg.get("result")

    def _ping(self):
        """A ping after PING s without a request; no answer in time closes the
        connection like a service that went away. Never during a request (a
        run's steps keep it busy anyway)."""
        while not self.closed.wait(PING):
            if time.monotonic() - self._last < PING or not self._lock.acquire(blocking=False):
                continue
            try:
                self._request("ping", {})
            except RemoteCarSimError:
                pass  # closed, with its reason
            finally:
                self._lock.release()


class ServiceLink:
    """Where the CarSim service connects: one at a time, a newer connection
    replaces the older one (the service was restarted, or its network
    changed). on_change(status()) when it connects or goes away."""

    def __init__(self):
        self.conn = None
        self.on_change = lambda status: None
        self._lock = threading.Lock()

    def listen(self, port):
        """Accept the service on 127.0.0.1:port, from a thread of its own
        (port 0: a free one). A port in use (another backend on this
        machine) is tried again every RETRY s. Returns the port, or None
        while it is in use."""
        try:
            srv = self._bind(port)
        except OSError as e:
            print("CarSim service port 127.0.0.1:%d not available (%s): trying again every %g s" % (port, e, RETRY),
                  flush=True)
            srv = None
        threading.Thread(target=self._accept, args=(srv, port), daemon=True).start()
        return srv.getsockname()[1] if srv is not None else None

    @staticmethod
    def _bind(port):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # As backend_server.serve: on Windows SO_REUSEADDR lets a second one listen on the same port.
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE if os.name == "nt" else socket.SO_REUSEADDR, 1)
        try:
            srv.bind(("127.0.0.1", port))
            srv.listen(4)
        except OSError:
            srv.close()
            raise
        return srv

    def _accept(self, srv, port):
        while srv is None:
            time.sleep(RETRY)
            try:
                srv = self._bind(port)
                print("CarSim service port 127.0.0.1:%d listening" % port, flush=True)
            except OSError:
                pass
        while True:
            try:
                sock, _ = srv.accept()
            except OSError:
                time.sleep(0.1)
                continue
            threading.Thread(target=self._hello, args=(sock,), daemon=True).start()

    def _hello(self, sock):
        """The service's hello, then it is the service (the older one is closed)."""
        lines = JsonLines(sock)
        try:
            hello = lines.recv(HELLO_TIMEOUT)
            if not (isinstance(hello, dict) and hello.get("type") == "hello" and hello.get("role") == "carsim"):
                sock.close()  # not the CarSim service
                return
            if hello.get("protocol") != SERVICE_PROTOCOL:
                lines.send({"type": "hello", "ok": False, "protocol": SERVICE_PROTOCOL,
                            "error": "Windows 上的 CarSim 服务（协议 %s）和云端的后端（协议 %d）版本不一致："
                                     "请用同一个安装包里的启动脚本和服务" % (hello.get("protocol"), SERVICE_PROTOCOL)})
                sock.close()
                return
            lines.send({"type": "hello", "ok": True, "protocol": SERVICE_PROTOCOL})
        except (OSError, ValueError):  # socket.timeout included
            sock.close()
            return
        sock.settimeout(None)
        conn = _Conn(sock, lines, hello, self._closed)
        with self._lock:
            old, self.conn = self.conn, conn
        if old is not None:
            old.close("Windows 上的服务重新连上了云端，这次运行的 CarSim 已经结束")
        conn.start()
        print("CarSim service connected: %s (%s, Python %s)" % (conn.host, conn.platform, hello.get("python")),
              flush=True)
        self.on_change(self.status())

    def _closed(self, conn):
        with self._lock:
            if conn is not self.conn:
                return  # replaced: the newer one is the service
            self.conn = None
        print("CarSim service gone: %s" % conn.why, flush=True)
        self.on_change(self.status())

    def status(self):
        """For world_info: {"connected", "host", "platform"}."""
        conn = self.conn
        up = conn is not None and not conn.closed.is_set()
        return {"connected": up, "host": conn.host if up else "", "platform": conn.platform if up else ""}

    def need(self):
        """The connected service, else RemoteCarSimError(NO_SERVICE)."""
        conn = self.conn
        if conn is None or conn.closed.is_set():
            raise RemoteCarSimError(NO_SERVICE)
        return conn


SERVICE = ServiceLink()


def check(d, link=None):
    """The run's .sim, python_carsim_env and solver, checked by the service
    before the backend changes its world (check_run_files, as with CarSim here)."""
    (link or SERVICE).need().request("check", carsim=d["carsim"], duration=d["sync"]["duration"])


def _num(v):
    return float("nan") if v is None else float(v)  # null: not finite on the service


class RemoteCarSimEnv:
    """CarSim of a carsim.remote run, on the service connection it was
    opened on: reset, control_step, close, config, t_current, sim_path as
    python_carsim_env's CarSimEnv. A newer connection is not this run's."""

    def __init__(self, d, link=None):
        self.conn = (link or SERVICE).need()
        r = self.conn.request("open", carsim=d["carsim"], duration=d["sync"]["duration"])
        self.config = r.get("config")
        self.sim_path = str(r.get("sim_path") or "")  # on the Windows computer, as the service resolved it
        self.t_current = 0.0
        self._open = True

    def reset(self):
        r = self.conn.request("reset")
        self.config, self.t_current = r.get("config"), _num(r["t_current"])
        return tuple(_num(v) for v in r["obs"])

    def control_step(self, action, inner_steps):
        # One round trip per CARLA frame: the inner CarSim steps run on the service.
        r = self.conn.request("step", action=[float(a) for a in action], inner=int(inner_steps))
        self.t_current = _num(r["t_current"])
        return tuple(_num(v) for v in r["obs"]), 0.0, bool(r["done"]), r.get("info") or {}

    def close(self):
        """Ends the service's run. Without the connection there is nothing
        to end: the service closes CarSim itself when it loses the backend."""
        if self._open and not self.conn.closed.is_set():
            self._open = False
            self.conn.request("close")
