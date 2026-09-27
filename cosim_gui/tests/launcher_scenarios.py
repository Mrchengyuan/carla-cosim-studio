"""The remote launcher (carla_cosim_launcher, 启动远程仿真.exe in the Windows
package) on Linux, with real clicks (--tour), against this server itself: the
package's SSH login goes to 127.0.0.1:22 with the restricted key (scripts/
install_remote_key.sh), so the real remote_session.sh starts the modified
CARLA and the backend; the launcher's local ports are 58120 / 58121 (this
server's own 57120 / 57121 are the backend's).

Scenarios: checks pass; no Python / Python without numpy (the button installs
it, from the Tsinghua mirror); a port taken by another program / by an old
tunnel (the button ends it); files missing; no ssh; a wrong key, a wrong host
key, a refused connection (each fails with its own hint, no retrying);
start -> ready -> stop -> start again -> quit (asks first); the SSH connection
dropped while ready (reconnects by itself, also when ssh says "Connection
reset" as a network drop does); the launcher killed (-9) while
ready (nothing it started is left, the server's session ends); the GUI's own
tour through the launcher (--gui-tour: its co-simulation runs use the
launcher's CarSim service); the light theme and a small window.

Needs: Xvfb, ImageMagick (convert), the Linux builds in cosim_gui/build,
the restricted key in ~/.config/carla_cosim_studio/remote_key.

    python3 cosim_gui/tests/launcher_scenarios.py [--out DIR] [--only a,b] [--gui-tour]
"""
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
BUILD = os.path.join(ROOT, "cosim_gui", "build")
BRIDGE = os.path.join(ROOT, "carsim_carla_bridge")
KEY = os.path.expanduser("~/.config/carla_cosim_studio/remote_key")
PY = os.path.join(ROOT, "venv_build", "bin", "python")  # has numpy
STATE = os.path.expanduser("~/.cache/carla_cosim_studio")
DISPLAY = ":57"
PORTS = "58120,58121"
FAILS = []


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + (("  " + str(detail)) if detail != "" else ""), flush=True)
    if not cond:
        FAILS.append(name)


def make_package(dest, python=PY, port=22, key=KEY, known=None):
    os.makedirs(os.path.join(dest, "service"))
    os.makedirs(os.path.join(dest, "ssh"))
    os.makedirs(os.path.join(dest, "fonts"))
    for f in ("carla_cosim_launcher", "carla_cosim_studio"):
        shutil.copy2(os.path.join(BUILD, f), dest)
    shutil.copy2(os.path.join(ROOT, "cosim_gui", "third_party", "fonts", "fa-solid-900.ttf"), os.path.join(dest, "fonts"))
    for f in ("carsim_service.py", "carsim_local.py", "mock_carsim.py"):
        shutil.copy2(os.path.join(BRIDGE, f), os.path.join(dest, "service"))
    shutil.copy2(key, os.path.join(dest, "ssh", "remote_key"))
    os.chmod(os.path.join(dest, "ssh", "remote_key"), 0o644)  # the launcher tightens it
    if known is None:
        known = subprocess.run(["ssh-keyscan", "-p", "22", "127.0.0.1"], capture_output=True, text=True).stdout
    with open(os.path.join(dest, "ssh", "known_hosts"), "w") as f:
        f.write(known)
    with open(os.path.join(dest, "cosim_studio_prefs.json"), "w") as f:
        json.dump({"remote_backend": True, "auto_start_backend": False, "backend_port": 58120,
                   "carla_host": "localhost", "carla_port": 3000, "python": "python", "backend_dir": "",
                   "last_config": "", "dark_theme": True}, f)
    with open(os.path.join(dest, "remote_launcher.json"), "w") as f:
        json.dump({"host": "127.0.0.1", "port": port, "user": os.environ.get("USER", "easyai"), "python": python,
                   "dark_theme": True}, f)
    with open(os.path.join(dest, "使用说明.txt"), "w") as f:
        f.write("test\n")
    return dest


def run(pkg, tour, script, args=(), env=None, timeout=600, during=None):
    """Runs the launcher's --tour script; during(proc, tour_dir) runs alongside. Returns the state json."""
    os.makedirs(tour, exist_ok=True)
    e = dict(os.environ, DISPLAY=DISPLAY)
    e.update(env or {})
    cmd = [os.path.join(pkg, "carla_cosim_launcher"), "--tour", tour, "--tour-script", script,
           "--local-ports", PORTS, "--size", "1100x780"] + list(args)
    fresh_log()
    with open(os.path.join(tour, "stdout.txt"), "w") as out:
        p = subprocess.Popen(cmd, cwd=pkg, env=e, stdout=out, stderr=subprocess.STDOUT)
        if during:
            during(p, tour)
        try:
            p.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            p.send_signal(signal.SIGTERM)
            p.wait(30)
            check("%s: finished in time" % os.path.basename(tour), False)
    for f in os.listdir(tour):
        if f.endswith(".ppm"):
            subprocess.run(["convert", os.path.join(tour, f), os.path.join(tour, f[:-4] + ".png")])
            os.remove(os.path.join(tour, f))
    path = os.path.join(tour, "launcher_state.json")
    return json.load(open(path, encoding="utf-8")) if os.path.exists(path) else {}


def fresh_log():
    """The launcher starts a new remote_launch.log; the old one must not answer wait_log."""
    for f in ("remote_launch.log", "remote_launch.log.prev"):
        try:
            os.remove(os.path.join(STATE, f))
        except OSError:
            pass


def ours_running(tag):
    """Processes of a scenario's package, or its service (--port 58121) / tunnel (58120:)."""
    out = subprocess.run(["pgrep", "-af", "."], capture_output=True, text=True).stdout.splitlines()
    return [l for l in out if (tag in l or "--port 58121" in l or "58120:127.0.0.1" in l) and "pgrep" not in l]


def texts(st, src=None):
    return [l["text"] for l in st.get("log", []) if src is None or l["src"] == src]


def checks(st):
    return {c["name"]: c for c in st.get("checks", [])}


def children_of(pid):
    out = subprocess.run(["ps", "-o", "pid=", "--ppid", str(pid)], capture_output=True, text=True).stdout
    return [int(x) for x in out.split()]


def wait_log(needle, timeout=240, since=0):
    """Waits for a line in the launcher's log file (remote_launch.log)."""
    path = os.path.join(STATE, "remote_launch.log")
    end = time.time() + timeout
    while time.time() < end:
        try:
            with open(path, encoding="utf-8", errors="replace") as f:
                if needle in f.read()[since:]:
                    return True
        except OSError:
            pass
        time.sleep(0.5)
    return False


def listening(port):
    s = socket.socket()
    try:
        s.settimeout(0.3)
        s.connect(("127.0.0.1", port))
        return True
    except OSError:
        return False
    finally:
        s.close()


# ----------------------------------------------------------------------------- scenarios
def sc_checks_ok(tmp, out):
    st = run(make_package(os.path.join(tmp, "p_ok")), os.path.join(out, "checks_ok"), "snap", ["--no-auto-start"])
    check("checks pass: idle, all five ok", st.get("phase") == "idle" and all(c["st"] == "ok" for c in st["checks"]), st.get("checks"))


def sc_no_python(tmp, out):
    st = run(make_package(os.path.join(tmp, "p_nopy"), python="/nonexistent/python3"), os.path.join(out, "no_python"), "snap")
    c = checks(st)
    check("no Python: python fails with a hint, nothing started", st.get("phase") == "idle" and c["Python"]["st"] == "fail"
          and "选择" in c["Python"]["hint"] and not any("连接云端" in t for t in texts(st)), c.get("Python"))


def sc_numpy(tmp, out):
    venv = os.path.join(tmp, "venv_nonumpy")
    subprocess.run([sys.executable, "-m", "venv", venv], check=True)
    st = run(make_package(os.path.join(tmp, "p_np"), python=os.path.join(venv, "bin", "python")), os.path.join(out, "numpy"),
             "numpy", ["--no-auto-start"], timeout=900)
    c = checks(st)
    check("numpy missing -> 安装 numpy -> installed, checks pass", c.get("numpy", {}).get("st") == "ok"
          and any("numpy 安装完成" in t for t in texts(st)) and st.get("phase") == "idle", (c.get("numpy"), texts(st)[-3:]))


def sc_port_other(tmp, out):
    srv = subprocess.Popen([sys.executable, "-m", "http.server", "58120", "--bind", "127.0.0.1"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(1.0)
        st = run(make_package(os.path.join(tmp, "p_port")), os.path.join(out, "port_other"), "snap")
    finally:
        srv.terminate()
        srv.wait()
    c = checks(st)["本机端口"]
    check("port taken by another program: fails, names it, no connection", c["st"] == "fail" and "58120" in c["detail"]
          and "python" in c["detail"] and "关掉它" in c["hint"] and st.get("phase") == "idle", c)


def sc_old_tunnel(tmp, out):
    fake = os.path.join(tmp, "fakebin")
    os.makedirs(fake, exist_ok=True)
    ssh = os.path.join(fake, "ssh")  # a script's process name is the script's: "ssh", like a stale tunnel
    with open(ssh, "w") as f:
        f.write("#!%s\nimport socket, time\ns = socket.socket()\ns.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)\n"
                "s.bind(('127.0.0.1', 58120)); s.listen(5)\ntime.sleep(600)\n" % sys.executable)
    os.chmod(ssh, 0o755)
    p = subprocess.Popen([ssh])
    try:
        time.sleep(1.0)
        st = run(make_package(os.path.join(tmp, "p_old")), os.path.join(out, "old_tunnel"), "oldtunnel", ["--no-auto-start"])
        ended = p.poll() is not None or (time.sleep(1.0) or p.poll() is not None)
    finally:
        if p.poll() is None:
            p.kill()
    c = checks(st)["本机端口"]
    check("old tunnel: 结束旧连接 ends it, the port is free", ended and c["st"] == "ok"
          and any("结束了占用端口的旧连接" in t for t in texts(st)), (ended, c))


def sc_missing_files(tmp, out):
    pkg = make_package(os.path.join(tmp, "p_files"))
    os.remove(os.path.join(pkg, "service", "carsim_service.py"))
    st = run(pkg, os.path.join(out, "missing_files"), "snap")
    c = checks(st)["启动包文件"]
    check("missing file: named, hint to unzip everything", c["st"] == "fail" and "carsim_service.py" in c["detail"]
          and "解压" in c["hint"], c)


def sc_no_ssh(tmp, out):
    st = run(make_package(os.path.join(tmp, "p_nossh")), os.path.join(out, "no_ssh"), "snap", env={"PATH": "/nonexistent"})
    c = checks(st)["SSH 客户端"]
    check("no ssh: fails with the OpenSSH hint", c["st"] == "fail" and "OpenSSH" in c["hint"], c)


def sc_bad_key(tmp, out):
    k = os.path.join(tmp, "other_key")
    subprocess.run(["ssh-keygen", "-q", "-t", "ed25519", "-N", "", "-f", k], check=True)
    st = run(make_package(os.path.join(tmp, "p_badkey"), key=k), os.path.join(out, "bad_key"), "snap")
    check("wrong key: failed, the key hint, no retry", st.get("phase") == "failed" and "钥匙" in st["fail"]["hint"]
          and st.get("reconnects") == 0, st.get("fail"))


def sc_bad_host_key(tmp, out):
    k = os.path.join(tmp, "hostkey")
    subprocess.run(["ssh-keygen", "-q", "-t", "ed25519", "-N", "", "-f", k], check=True)
    pub = open(k + ".pub").read().split()
    st = run(make_package(os.path.join(tmp, "p_hostkey"), known="127.0.0.1 %s %s\n" % (pub[0], pub[1])),
             os.path.join(out, "bad_host_key"), "snap")
    check("wrong host key: failed, the identity hint", st.get("phase") == "failed" and "身份" in st["fail"]["hint"], st.get("fail"))


def sc_refused(tmp, out):
    st = run(make_package(os.path.join(tmp, "p_refused"), port=1), os.path.join(out, "refused"), "snap")
    check("refused: failed, the refused hint", st.get("phase") == "failed" and "拒绝" in st["fail"]["hint"], st.get("fail"))


def sc_main(tmp, out):
    st = run(make_package(os.path.join(tmp, "p_main")), os.path.join(out, "main"), "main")
    t = texts(st)
    check("main: ready, stop, again, quit asks: all tour steps", st.get("tour_steps_done", 0) >= 6, (st.get("tour_steps_done"), t[-5:]))
    # Started again at once, the server may still be ending the last session (its backend stops
    # slowly after CARLA's first start): it replaces it, and the service, which may have reached
    # the old backend meanwhile, reconnects to the new one. So: at least twice, connected at the end.
    service = [x for x in texts(st, 2) if x.startswith(("已连上云端", "连接断开", "正在连接云端"))]
    check("main: ready twice, the GUI opened twice, the service connected (again) each time",
          sum("云端就绪" in x for x in t) == 2 and sum(x == "打开仿真界面" for x in t) == 2
          and sum(x.startswith("已连上云端（127.0.0.1:58121）") for x in t) >= 2
          and service[-1].startswith("已连上云端"), t)
    time.sleep(3)
    left = ours_running("/p_main/")
    check("main: nothing left running, ports free", not listening(58120) and not listening(58121) and not left, left)
    log = open(os.path.join(STATE, "remote_backend.log"), encoding="utf-8", errors="replace").read()
    check("main: the server's session ended in order", "远程会话结束" in log.splitlines()[-1] or "远程会话结束" in log[-400:], log[-300:])


def sc_reconnect(tmp, out):
    pkg = make_package(os.path.join(tmp, "p_reconnect"))
    info = {}

    def during(p, tour):
        if not wait_log("云端就绪", 240):
            return
        time.sleep(3)
        for c in children_of(p.pid):
            cmd = open("/proc/%d/cmdline" % c).read()
            if "remote_key" in cmd:
                info["killed"] = c
                os.kill(c, signal.SIGKILL)  # like a dropped network: the connection is simply gone
    st = run(pkg, os.path.join(out, "reconnect"), "reconnect", during=during)
    t = texts(st)
    check("dropped connection: reconnected by itself (the service too), ready again, stopped at the end",
          "killed" in info and "重新连接云端（第 1 次）…" in t and "已重新连上云端（重连 1 次）" in t
          and sum(x.startswith("已连上云端（127.0.0.1:58121）") for x in t) == 2
          and st.get("tour_steps_done", 0) >= 5 and st.get("phase") == "stopped", (info, st.get("phase"), t[-8:]))


def sc_reconnect_msg(tmp, out):
    """Like a real network drop: ssh says "Connection reset" as it ends. That is no reason to give up."""
    wrap = os.path.join(tmp, "wrapbin")
    os.makedirs(wrap, exist_ok=True)
    with open(os.path.join(wrap, "ssh"), "w") as f:
        f.write('#!/bin/bash\n/usr/bin/ssh "$@"\nrc=$?\n'
                'echo "client_loop: send disconnect: Connection reset by peer" >&2\nexit $rc\n')
    os.chmod(os.path.join(wrap, "ssh"), 0o755)
    info = {}

    def during(p, tour):
        if not wait_log("云端就绪", 240):
            return
        time.sleep(3)
        for c in children_of(p.pid):
            for g in children_of(c):  # the real ssh under the wrapper
                if "remote_key" in open("/proc/%d/cmdline" % g).read():
                    info["killed"] = g
                    os.kill(g, signal.SIGKILL)
    st = run(make_package(os.path.join(tmp, "p_reconnect_msg")), os.path.join(out, "reconnect_msg"), "reconnect",
             env={"PATH": wrap + ":" + os.environ["PATH"]}, during=during)
    t = texts(st)
    check("dropped connection with ssh's 'Connection reset': still reconnects",
          "killed" in info and any("Connection reset" in x for x in t) and "已重新连上云端（重连 1 次）" in t
          and st.get("phase") == "stopped", (info, st.get("phase"), t[-8:]))


def sc_kill9(tmp, out):
    pkg = make_package(os.path.join(tmp, "p_kill9"))
    e = dict(os.environ, DISPLAY=DISPLAY)
    fresh_log()
    p = subprocess.Popen([os.path.join(pkg, "carla_cosim_launcher"), "--local-ports", PORTS], cwd=pkg, env=e,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    ok = wait_log("云端就绪", 240) and wait_log("已连上云端（127.0.0.1:58121）", 30)
    time.sleep(3)
    kids = children_of(p.pid)
    p.kill()
    p.wait()
    time.sleep(3)
    left = [k for k in kids if os.path.exists("/proc/%d" % k)] + ours_running("/p_kill9/")
    log = ""
    for _ in range(20):
        log = open(os.path.join(STATE, "remote_backend.log"), encoding="utf-8", errors="replace").read()
        if "远程会话结束" in log[-300:]:
            break
        time.sleep(1)
    check("launcher killed (-9): its ssh / service / GUI are gone, ports free, the server's session ended",
          ok and len(kids) == 3 and not left and not listening(58120) and "远程会话结束" in log[-300:],
          (ok, kids, left, log[-200:]))


def sc_gui_tour(tmp, out):
    gt = os.path.join(out, "gui_tour")
    os.makedirs(gt, exist_ok=True)
    st = run(make_package(os.path.join(tmp, "p_guitour")), os.path.join(out, "launcher_gui_tour"), "main",
             ["--gui-arg", "--tour", "--gui-arg", gt], timeout=3600)
    t = texts(st, 3)
    steps = [x for x in t if x.startswith("[info] TOUR step")]
    bad = [x for x in t if "TIMEOUT" in x or "click target missing" in x]
    check("the GUI's tour through the launcher: all steps ok, then the launcher stops and starts again",
          steps and "[info] TOUR DONE" in t and not bad and st.get("tour_steps_done", 0) >= 6,
          (len(steps), bad[:3], t[-3:]))
    print("INFO GUI tour steps: %d" % len(steps))


def sc_light_small(tmp, out):
    pkg = make_package(os.path.join(tmp, "p_light"))
    st = run(pkg, os.path.join(out, "light"), "light", ["--no-auto-start"])
    check("light theme: toggled and back", os.path.exists(os.path.join(out, "light", "light.png")))
    st = run(pkg, os.path.join(out, "small"), "snap", ["--no-auto-start", "--size", "960x620"])
    check("small window renders", os.path.exists(os.path.join(out, "small", "snap.png")) and st.get("phase") == "idle")


SCENARIOS = [("checks_ok", sc_checks_ok), ("no_python", sc_no_python), ("port_other", sc_port_other),
             ("old_tunnel", sc_old_tunnel), ("missing_files", sc_missing_files), ("no_ssh", sc_no_ssh),
             ("bad_key", sc_bad_key), ("bad_host_key", sc_bad_host_key), ("refused", sc_refused),
             ("light_small", sc_light_small), ("main", sc_main), ("reconnect", sc_reconnect),
             ("reconnect_msg", sc_reconnect_msg), ("kill9", sc_kill9),
             ("numpy", sc_numpy), ("gui_tour", sc_gui_tour)]


def main():
    out = sys.argv[sys.argv.index("--out") + 1] if "--out" in sys.argv else "/tmp/cc_launcher_scenarios"
    only = sys.argv[sys.argv.index("--only") + 1].split(",") if "--only" in sys.argv else None
    shutil.rmtree(out, ignore_errors=True)
    os.makedirs(out)
    tmp = tempfile.mkdtemp(prefix="cc_lscen_")
    xvfb = None
    if subprocess.run(["xdpyinfo", "-display", DISPLAY], capture_output=True).returncode != 0:
        xvfb = subprocess.Popen(["Xvfb", DISPLAY, "-screen", "0", "1400x1000x24"], stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)
        time.sleep(1.5)
    try:
        for name, fn in SCENARIOS:
            if only and name not in only:
                continue
            if name == "gui_tour" and "--gui-tour" not in sys.argv and not only:
                continue
            print("== %s" % name, flush=True)
            t0 = time.time()
            fn(tmp, out)
            print("   (%.0f s)" % (time.time() - t0), flush=True)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
        if xvfb:
            xvfb.terminate()
    print("FAILED: %s" % FAILS if FAILS else "ALL LAUNCHER SCENARIOS PASSED")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
