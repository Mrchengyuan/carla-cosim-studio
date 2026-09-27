"""Checks for the remote mode's server scripts and Windows package that need no
CARLA server: scripts/remote_session.sh (the SSH forced command), the key
installer, the package builder, the Windows .bat scripts and the launcher's
connection (statically; cosim_gui/tests/launcher_scenarios.py runs the launcher).
Nothing real is started or stopped: tmux, ss, pkill, pgrep and the backend's
Python are stubs; keys and authorized_keys files are temporary ones.

    python tests/test_offline_ops.py
"""

import json
import os
import re
import shutil
import signal
import stat
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
SCRIPTS = os.path.join(ROOT, "scripts")
BRIDGE = os.path.join(ROOT, "carsim_carla_bridge")
WINDOWS = os.path.join(SCRIPTS, "windows")
LAUNCHER = os.path.join(ROOT, "cosim_gui", "src", "launcher.cpp")
READY_LINES = ["CARLA 已就绪", "后端已就绪", "就绪"]


def read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def stub(path, body):
    with open(path, "w") as f:
        f.write("#!/bin/bash\n" + body + "\n")
    os.chmod(path, 0o755)


# ss: the ports listed in $FAKE_NET (a file per listening port: "name pid";
# name "-" = another user's process, shown without users:).
SS = r'''
want=""
for a in "$@"; do case "$a" in "sport = :"*) want="${a#sport = :}" ;; esac; done
for f in "$FAKE_NET"/*; do
  [ -f "$f" ] || continue
  port=${f##*/}
  [ -n "$want" ] && [ "$want" != "$port" ] && continue
  read -r name pid < "$f"
  users=""; [ "$name" = "-" ] || users="users:((\"$name\",pid=$pid,fd=7))"
  echo "LISTEN 0      128        127.0.0.1:$port      0.0.0.0:*    $users"
done'''

# tmux: records; "new" makes the session and puts CARLA on port 3000 a second later.
TMUX = r'''
echo "tmux $*" >> "$FAKE_CALLS"
case "$1" in
  has-session) [ -f "$FAKE_TMUX" ] ;;
  new) : > "$FAKE_TMUX"; (sleep 1; echo "UE4Editor 4242" > "$FAKE_NET/3000") >/dev/null 2>&1 & ;;
  *) exit 1 ;;
esac'''

# The backend's Python: --help lists --carsim-port when FAKE_CARSIM_PORT is set;
# backend_server.py listens (a file in $FAKE_NET), exits with the code written
# into $FAKE_CTL, and on SIGTERM. Anything else (the "is it CARLA" question) fails.
PYTHON = r'''
case " $* " in
  *" --help "*) echo "usage: backend_server.py [-h] [--port PORT] [--exit-with-client]${FAKE_CARSIM_PORT:+ [--carsim-port CARSIM_PORT]}"; exit 0 ;;
  *backend_server.py*) ;;
  *) echo "python $*" >> "$FAKE_CALLS"; exit 1 ;;
esac
echo "backend $$ $*" >> "$FAKE_CALLS"
trap 'echo "term $$" >> "$FAKE_CALLS"; rm -f "$FAKE_NET/57120"; exit 0' TERM
echo "python3 $$" > "$FAKE_NET/57120"
while :; do
  if [ -f "$FAKE_CTL" ]; then
    rc=$(cat "$FAKE_CTL"); rm -f "$FAKE_CTL" "$FAKE_NET/57120"
    echo "exit $$ $rc" >> "$FAKE_CALLS"; exit "$rc"
  fi
  sleep 0.1
done'''


class Session:
    """remote_session.sh running with a pipe as its stdin (the SSH channel)."""

    def __init__(self, env):
        self.p = subprocess.Popen(["bash", os.path.join(SCRIPTS, "remote_session.sh")], env=env,
                                  stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                  start_new_session=True)
        self.lines = []
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        for raw in self.p.stdout:
            self.lines.append(raw.decode("utf-8", "replace").rstrip("\n"))

    def wait_for(self, line, count=1, timeout=30):
        end = time.time() + timeout
        while time.time() < end:
            if self.lines.count(line) >= count:
                return True
            time.sleep(0.05)
        raise AssertionError("no %r (x%d) in %r" % (line, count, self.lines))

    def end_session(self, timeout=20):
        """The SSH session ends: its stdin closes."""
        self.p.stdin.close()
        return self.p.wait(timeout)

    def close(self):
        """Whatever the test did: nothing of it keeps running."""
        if self.p.poll() is None:
            try:
                os.killpg(self.p.pid, signal.SIGKILL)
            except OSError:
                pass
            self.p.wait(10)
        self.reader.join(10)
        self.p.stdin.close()
        self.p.stdout.close()


class RemoteSessionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="cc_ops_")
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.bin = os.path.join(self.tmp, "bin")
        self.net = os.path.join(self.tmp, "net")
        os.mkdir(self.bin)
        os.mkdir(self.net)
        self.calls = os.path.join(self.tmp, "calls")
        self.ctl = os.path.join(self.tmp, "ctl")
        stub(os.path.join(self.bin, "ss"), SS)
        stub(os.path.join(self.bin, "tmux"), TMUX)
        for name in ("pkill", "pgrep"):
            stub(os.path.join(self.bin, name), 'echo "%s $*" >> "$FAKE_CALLS"\nexit 1' % name)
        self.python = os.path.join(self.tmp, "python")
        stub(self.python, PYTHON)
        self.state = os.path.join(self.tmp, "carla_cosim_studio")
        self.env = dict(os.environ, PATH=self.bin + os.pathsep + os.environ["PATH"], XDG_CACHE_HOME=self.tmp,
                        COSIM_PYTHON=self.python, CARLA_MOD_PORT="3000", FAKE_NET=self.net, FAKE_CALLS=self.calls,
                        FAKE_CTL=self.ctl, FAKE_TMUX=os.path.join(self.tmp, "tmux_session"), FAKE_CARSIM_PORT="1",
                        # Were anything real reached, there would be no CARLA to start.
                        CARLA_SRC=os.path.join(self.tmp, "no_carla_src"), UE4_ROOT=os.path.join(self.tmp, "no_ue4"),
                        CARLA_ROOT=os.path.join(self.tmp, "no_carla"))
        self.env.pop("COSIM_ROOT", None)
        self.sessions = []

    def tearDown(self):
        for s in self.sessions:
            s.close()

    def carla_running(self, name="UE4Editor"):
        with open(os.path.join(self.net, "3000"), "w") as f:
            f.write("%s 4242\n" % name)

    def start(self):
        s = Session(self.env)
        self.sessions.append(s)
        return s

    def recorded(self, prefix=""):
        if not os.path.exists(self.calls):
            return []
        return [c for c in read(self.calls).splitlines() if c.startswith(prefix)]

    def wait_calls(self, prefix, count, timeout=15):
        end = time.time() + timeout
        while time.time() < end and len(self.recorded(prefix)) < count:
            time.sleep(0.05)
        self.assertGreaterEqual(len(self.recorded(prefix)), count, self.recorded())

    def pid_of(self, what):
        with open(os.path.join(self.state, what + ".pid")) as f:
            return int(f.read())

    def test_progress_lines_then_stops_backend_when_stdin_closes(self):
        self.carla_running()
        s = self.start()
        s.wait_for("就绪")
        self.assertTrue(s.lines[0].startswith("已连上云端服务器（"), s.lines)
        self.assertEqual(s.lines[1:], READY_LINES)  # CARLA was running: nothing to start
        (backend,) = self.recorded("backend ")
        pid = int(backend.split()[1])
        self.assertTrue(backend.endswith("-u backend_server.py --port 57120 --carsim-port 57121"), backend)
        self.assertEqual(self.pid_of("remote_backend"), pid)
        self.assertEqual(self.pid_of("remote_session"), s.p.pid)
        self.assertFalse(self.recorded("tmux new") + self.recorded("pkill") + self.recorded("python "))
        self.assertEqual(s.end_session(), 0)
        self.assertEqual(self.recorded("term "), ["term %d" % pid])
        self.assertFalse(os.path.exists(os.path.join(self.state, "remote_backend.pid")))
        self.assertFalse(os.path.exists(os.path.join(self.state, "remote_session.pid")))
        log = read(os.path.join(self.state, "remote_backend.log"))
        for line in READY_LINES + ["远程会话结束"]:
            self.assertIn(line, log)
        # The next session keeps this log as .prev; CARLA was never stopped.
        s2 = self.start()
        s2.wait_for("就绪")
        s2.end_session()
        self.assertIn("远程会话结束", read(os.path.join(self.state, "remote_backend.log.prev")))
        self.assertFalse(self.recorded("pkill") + self.recorded("tmux kill"))

    def test_backend_without_carsim_port_gets_only_its_port(self):
        self.env.pop("FAKE_CARSIM_PORT")
        self.carla_running()
        s = self.start()
        s.wait_for("就绪")
        (backend,) = self.recorded("backend ")
        self.assertTrue(backend.endswith("-u backend_server.py --port 57120"), backend)
        s.end_session()

    def test_backend_restarted_on_exit_code_3_and_after_a_crash_then_sighup(self):
        self.carla_running()
        s = self.start()
        s.wait_for("就绪")
        with open(self.ctl, "w") as f:
            f.write("3")  # the GUI's "重启后端"
        s.wait_for("后端正在重启…")
        s.wait_for("后端已就绪", 2)
        with open(self.ctl, "w") as f:
            f.write("1")  # a crash right after start: restarted after a back-off
        s.wait_for("后端已退出（退出码 1），2 秒后重新启动（服务器上的记录：%s）"
                   % os.path.join(self.state, "remote_backend.log"))
        s.wait_for("后端已就绪", 3)
        starts = self.recorded("backend ")
        self.assertEqual(len(starts), 3, starts)
        self.assertEqual(s.lines.count("就绪"), 1, s.lines)
        last = int(starts[-1].split()[1])
        self.assertEqual(self.pid_of("remote_backend"), last)
        os.kill(s.p.pid, signal.SIGHUP)  # the SSH connection is gone
        self.assertEqual(s.p.wait(20), 0)
        self.assertEqual(self.recorded("term "), ["term %d" % last])

    def test_new_session_replaces_the_running_one(self):
        self.carla_running()
        a = self.start()
        a.wait_for("就绪")
        (first,) = self.recorded("backend ")
        b = self.start()
        b.wait_for("就绪")
        self.assertEqual(a.p.wait(20), 0)  # ended by b, after stopping its backend
        self.assertIn("云端还有上一次的连接，先结束它…", b.lines)
        starts = self.recorded("backend ")
        self.assertEqual(len(starts), 2, starts)
        self.assertEqual(self.recorded("term "), ["term " + first.split()[1]])
        # Its backend was stopped before b's started (the port is b's now).
        self.assertLess(self.recorded().index("term " + first.split()[1]), self.recorded().index(starts[1]))
        self.assertEqual(self.pid_of("remote_session"), b.p.pid)
        self.assertEqual(self.pid_of("remote_backend"), int(starts[1].split()[1]))
        self.assertEqual(b.end_session(), 0)
        self.assertEqual(len(self.recorded("term ")), 2)

    def test_starts_the_modified_carla_when_its_port_is_free(self):
        s = self.start()
        s.wait_for("就绪", timeout=40)
        self.assertEqual(s.lines[1:], ["正在启动 CARLA（首次约 1 分钟）…"] + READY_LINES)
        (new,) = self.recorded("tmux new")
        self.assertIn("-d -s carla_mod env COSIM_ROOT='%s' " % ROOT, new)
        for part in ("CARLA_SRC='%s'" % self.env["CARLA_SRC"], "UE4_ROOT='%s'" % self.env["UE4_ROOT"],
                     "CARLA_MOD_PORT='3000'", "bash '%s/carla_mod_server.sh' > '%s/remote_carla.log' 2>&1"
                     % (SCRIPTS, self.state)):
            self.assertIn(part, new)
        s.end_session()
        # CARLA stays for the next session.
        self.assertFalse(self.recorded("pkill") + self.recorded("tmux kill"))

    def test_other_program_on_carlas_port(self):
        self.carla_running("node")
        s = self.start()
        self.assertEqual(s.p.wait(20), 1)
        time.sleep(0.2)
        self.assertIn("[错误] 服务器上的端口 3000 被其他程序（node）占用，不是 CARLA。"
                      "请告诉服务器管理员（scripts/env.sh 里的 CARLA_MOD_PORT）", s.lines)
        # Asked its version once (it is not CARLA), nothing started.
        self.assertEqual(len(self.recorded("python ")), 1, self.recorded())
        self.assertFalse(self.recorded("backend ") + self.recorded("tmux new"))


class InstallKeyTests(unittest.TestCase):
    OTHER = ('ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABAQC7 someone@laptop\n'
             'no-pty,command="/usr/bin/uptime" ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIOther monitor')  # no newline

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="cc_ops_key_")
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.key = os.path.join(self.tmp, "config", "carla_cosim_studio", "remote_key")
        self.env = dict(os.environ, REMOTE_KEY=self.key, HOME=os.path.join(self.tmp, "home"))
        self.env.pop("COSIM_ROOT", None)

    def install(self, auth):
        return subprocess.run(["bash", os.path.join(SCRIPTS, "install_remote_key.sh"), auth], env=self.env,
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=60)

    def expected_line(self):
        kind, blob = read(self.key + ".pub").split()[:2]
        # permitlisten: no ssh -R (port-forwarding would allow it; 1 can never be listened on)
        return ('restrict,port-forwarding,permitopen="127.0.0.1:57120",permitopen="127.0.0.1:57121",'
                'permitlisten="127.0.0.1:1",command="%s/scripts/remote_session.sh" %s %s carla-cosim-remote'
                % (ROOT, kind, blob))

    def test_appends_the_restricted_line_once(self):
        auth = os.path.join(self.tmp, "authorized_keys")
        with open(auth, "w") as f:
            f.write(self.OTHER)
        os.chmod(auth, 0o600)
        p = self.install(auth)
        self.assertEqual(p.returncode, 0, p.stdout)
        self.assertEqual(stat.S_IMODE(os.stat(self.key).st_mode), 0o600)
        self.assertEqual(stat.S_IMODE(os.stat(self.key + ".pub").st_mode), 0o600)
        self.assertEqual(stat.S_IMODE(os.stat(os.path.dirname(self.key)).st_mode), 0o700)
        self.assertTrue(read(self.key + ".pub").startswith("ssh-ed25519 "))
        line = self.expected_line()
        self.assertEqual(read(auth), self.OTHER + "\n" + line + "\n")  # other lines untouched
        self.assertEqual(stat.S_IMODE(os.stat(auth).st_mode), 0o600)
        key_before = read(self.key)
        for _ in range(2):  # again: nothing changes, not even the key
            p = self.install(auth)
            self.assertEqual(p.returncode, 0, p.stdout)
            self.assertIn(b"unchanged", p.stdout)
            self.assertEqual(read(auth), self.OTHER + "\n" + line + "\n")
            self.assertEqual(read(self.key), key_before)

    def test_new_file_and_key_already_there_with_other_options(self):
        auth = os.path.join(self.tmp, "new", "authorized_keys")
        self.assertEqual(self.install(auth).returncode, 0)
        line = self.expected_line()
        self.assertEqual(read(auth), line + "\n")
        self.assertEqual(stat.S_IMODE(os.stat(auth).st_mode), 0o600)
        # The same key with other options (e.g. edited by hand): left alone.
        edited = line.replace("restrict,", "restrict,no-touch-required,") + "\n"
        with open(auth, "w") as f:
            f.write(edited)
        p = self.install(auth)
        self.assertEqual(p.returncode, 0)
        self.assertIn(b"other options", p.stdout)
        self.assertEqual(read(auth), edited)


class PackageTests(unittest.TestCase):
    KNOWN = "[i.easy-ai.cloud]:32122 ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIHostKey"

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="cc_ops_pkg_")
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.key = os.path.join(self.tmp, "remote_key")
        with open(self.key, "w") as f:
            f.write("-----BEGIN OPENSSH PRIVATE KEY-----\nnot a real key %s\n-----END OPENSSH PRIVATE KEY-----\n"
                    % os.getpid())
        self.exe = os.path.join(self.tmp, "carla_cosim_studio.exe")
        self.launcher = os.path.join(self.tmp, "carla_cosim_launcher.exe")
        for path, fill in ((self.exe, b"\0"), (self.launcher, b"\1")):
            with open(path, "wb") as f:
                f.write(b"MZ" + fill * 64)
        self.env = dict(os.environ, REMOTE_KNOWN_HOSTS="# i.easy-ai.cloud:32122 SSH-2.0\n" + self.KNOWN + "\n")
        self.env.pop("COSIM_ROOT", None)

    def build(self, out, *extra):
        return subprocess.run(["bash", os.path.join(SCRIPTS, "build_remote_package.sh"), "--exe", self.exe,
                               "--launcher", self.launcher, "--key", self.key, "--out", out] + list(extra),
                              env=self.env,
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=120)

    def repo_state(self):
        out = subprocess.run(["git", "status", "--porcelain", "--ignored"], cwd=ROOT, stdout=subprocess.PIPE).stdout
        names = []
        for root, dirs, files in os.walk(ROOT):  # symlinks (CARLA, carla_src) are not followed
            dirs[:] = [d for d in dirs if d != ".git"]
            names += [os.path.join(root, f) for f in files
                      if f in ("remote_key", "known_hosts") or f.startswith("CARLA_CoSim_Studio_Remote")]
        return out, names

    def test_layout_and_contents(self):
        before = self.repo_state()
        out = os.path.join(self.tmp, "out", "CARLA_CoSim_Studio_Remote.zip")
        p = self.build(out)
        self.assertEqual(p.returncode, 0, p.stdout.decode())
        self.assertEqual(self.repo_state(), before)  # nothing written into the repository
        top = "CARLA_CoSim_Studio_Remote/"
        service = [f for f in ("carsim_service.py", "carsim_local.py", "mock_carsim.py")
                   if os.path.exists(os.path.join(BRIDGE, f))]
        self.assertIn("mock_carsim.py", service)
        for f in ("carsim_service.py", "carsim_local.py"):  # the backend part adds them
            if f not in service:
                self.assertIn(("warning: %s/%s is missing" % (BRIDGE, f)).encode(), p.stdout)
        expected = {top + n for n in ["启动远程仿真.exe", "carla_cosim_studio.exe", "fonts/fa-solid-900.ttf",
                                      "remote_launcher.json", "cosim_studio_prefs.json", "ssh/remote_key",
                                      "ssh/known_hosts", "使用说明.txt"]
                    + ["service/" + f for f in service]}
        z = zipfile.ZipFile(out)
        self.addCleanup(z.close)
        self.assertEqual(set(z.namelist()), expected)
        for info in z.infolist():  # UTF-8 names: Windows' own unzip shows them right
            if any(ord(c) > 127 for c in info.filename):
                self.assertTrue(info.flag_bits & 0x800, info.filename)
        self.assertEqual(stat.S_IMODE(os.stat(out).st_mode), 0o600)  # it holds the private key
        # The unzipped folder next to the zip is the same.
        pkg = os.path.join(self.tmp, "out", "CARLA_CoSim_Studio_Remote")
        for name in expected:
            with open(os.path.join(self.tmp, "out", name), "rb") as f:
                self.assertEqual(f.read(), z.read(name), name)

        for name, src in (("启动远程仿真.exe", self.launcher), ("carla_cosim_studio.exe", self.exe)):
            with open(src, "rb") as f:
                self.assertEqual(z.read(top + name), f.read(), name)
        # The server, fixed (its host key is in ssh/known_hosts); the launcher adds the Python and the theme.
        self.assertEqual(json.loads(z.read(top + "remote_launcher.json")),
                         {"host": "i.easy-ai.cloud", "port": 32122, "user": "easyai", "python": "", "dark_theme": True})
        with open(self.key, "rb") as f:
            self.assertEqual(z.read(top + "ssh/remote_key"), f.read())
        self.assertEqual(z.read(top + "ssh/known_hosts").decode(), self.KNOWN + "\n")
        with open(os.path.join(ROOT, "cosim_gui", "third_party", "fonts", "fa-solid-900.ttf"), "rb") as f:
            self.assertEqual(z.read(top + "fonts/fa-solid-900.ttf"), f.read())
        for f in service:
            self.assertEqual(z.read(top + "service/" + f).decode(), read(os.path.join(BRIDGE, f)))
        notes = z.read(top + "使用说明.txt")
        self.assertTrue(notes.startswith(b"\xef\xbb\xbf"))  # Notepad
        self.assertEqual(notes.count(b"\n"), notes.count(b"\r\n"))
        self.assertIn("启动远程仿真.exe", notes.decode("utf-8-sig"))
        self.assertNotIn(".bat", notes.decode("utf-8-sig"))

        prefs = json.loads(z.read(top + "cosim_studio_prefs.json"))
        self.assertEqual({k: prefs[k] for k in ("remote_backend", "auto_start_backend", "backend_port",
                                                  "carla_host", "carla_port")},
                         {"remote_backend": True, "auto_start_backend": False, "backend_port": 57120,
                          "carla_host": "localhost", "carla_port": 3000})
        # Every key the GUI has a default for, with a value of the same type
        # (the GUI drops a value of another type).
        src = read(os.path.join(ROOT, "cosim_gui", "src", "app.cpp"))
        defaults = src[src.index("  prefs_ = {{"):]
        defaults = defaults[:defaults.index("};")]
        keys = re.findall(r'\{"(\w+)",', defaults)
        self.assertIn("backend_port", keys)
        for k in keys:
            self.assertIn(k, prefs)
        self.assertEqual((prefs["python"], prefs["backend_dir"], prefs["last_config"], prefs["dark_theme"]),
                         ("python", "", "", True))

        # Built again: replaced, not merged with the old one.
        with open(os.path.join(pkg, "stale.txt"), "w") as f:
            f.write("old")
        self.assertEqual(self.build(out).returncode, 0)
        self.assertFalse(os.path.exists(os.path.join(pkg, "stale.txt")))
        with zipfile.ZipFile(out) as z2:
            self.assertEqual(set(z2.namelist()), expected)

    def test_refuses_the_repository_and_other_hosts(self):
        before = self.repo_state()
        for out in (os.path.join(ROOT, "CARLA_CoSim_Studio_Remote.zip"),
                    os.path.join(ROOT, "cosim_gui", "build-win", "pkg", "x.zip")):
            p = self.build(out)
            self.assertNotEqual(p.returncode, 0)
            self.assertIn(b"inside the repository", p.stdout)
        self.assertEqual(self.repo_state(), before)
        out = os.path.join(self.tmp, "out", "x.zip")
        self.env["REMOTE_KNOWN_HOSTS"] = "[other.host]:22 ssh-ed25519 AAAA"
        p = self.build(out)
        self.assertNotEqual(p.returncode, 0)
        self.assertIn(b"not for [i.easy-ai.cloud]:32122", p.stdout)
        self.env["REMOTE_KNOWN_HOSTS"] = "[other.host]:2222 ssh-ed25519 AAAA"
        p = self.build(out, "--host", "other.host", "--ssh-port", "2222", "--user", "me")
        self.assertEqual(p.returncode, 0, p.stdout.decode())
        with zipfile.ZipFile(out) as z:
            self.assertEqual(json.loads(z.read("CARLA_CoSim_Studio_Remote/remote_launcher.json"))["host"], "other.host")
            self.assertEqual(json.loads(z.read("CARLA_CoSim_Studio_Remote/remote_launcher.json"))["port"], 2222)
        shutil.rmtree(os.path.dirname(out))
        for bad in (("--ssh-port", "22x"), ("--host", "a b"), ("--user", "a\"b")):
            p = self.build(out, *bad)
            self.assertNotEqual(p.returncode, 0, bad)
        self.env["REMOTE_KNOWN_HOSTS"] = ""
        self.assertNotEqual(self.build(out).returncode, 0)
        self.assertFalse(os.path.exists(os.path.dirname(out)) and os.listdir(os.path.dirname(out)))


PATH_ENV = {"LOCALAPPDATA", "APPDATA", "TEMP", "TMP", "USERPROFILE", "SystemRoot", "ProgramFiles", "ComSpec"}


def bat_path_vars(files):
    """Variables set from a path, or from another path variable (any of the files)."""
    paths = set(PATH_ENV)
    sets = []
    for path in files:
        for line in read(path).splitlines():
            m = re.match(r'\s*(?:if .*?\s)?set "(\w+)=(.*)"', line, re.I)
            if m:
                sets.append(m.groups())
    changed = True
    while changed:
        changed = False
        for name, value in sets:
            if name not in paths and ("\\" in value or "%~" in value or any("%" + v + "%" in value for v in paths)):
                paths.add(name)
                changed = True
    return paths


def bat_problems(path, paths):
    """Pitfalls cmd only shows at run time: ( ) not balanced (outside quotes,
    ^-escapes and the echo( idiom), and a path variable used unquoted inside
    ( ) (a ")" in the path, e.g. "Program Files (x86)", ends the block)."""
    problems, depth = [], 0
    for no, line in enumerate(read(path).splitlines(), 1):
        s = line.strip()
        if re.match(r"(rem\b|::|:\w)", s, re.I):
            continue
        in_quote, i, block_line, unquoted = False, 0, depth > 0, []
        while i < len(s):
            c = s[i]
            if c == '"':
                in_quote = not in_quote
            elif not in_quote:
                if c == "^":
                    i += 1
                elif c == "(" and not re.search(r"echo$", s[:i], re.I):
                    depth += 1
                    block_line = True
                elif c == ")":
                    depth -= 1
                    if depth < 0:
                        problems.append("%d: ) without (" % no)
                        depth = 0
                elif c == "%":
                    m = re.match(r"%(~\w*\d|\w+)%?", s[i:])
                    if m:
                        unquoted.append(m.group(1))
                        i += len(m.group(0)) - 1
            i += 1
        if in_quote:
            problems.append("%d: unbalanced quote" % no)
        if block_line:
            for v in unquoted:
                if v in paths or v.startswith("~"):
                    problems.append("%d: %%%s%% unquoted inside ( )" % (no, v))
    if depth:
        problems.append("( without ) at the end")
    return problems


class BatTests(unittest.TestCase):
    def test_no_block_pitfalls(self):
        bats = sorted(os.path.join(WINDOWS, f) for f in os.listdir(WINDOWS) if f.endswith(".bat"))
        self.assertGreaterEqual(len(bats), 4)
        paths = bat_path_vars(bats)  # env.bat sets what the others use
        self.assertTrue({"CROOT", "STUDIO_EXE"} <= paths, paths)
        for path in bats:
            with self.subTest(bat=os.path.basename(path)):
                self.assertEqual(bat_problems(path, paths), [])

    def test_checker_finds_the_pitfalls(self):
        tmp = tempfile.mkdtemp(prefix="cc_ops_bat_")
        self.addCleanup(shutil.rmtree, tmp, True)
        bad = os.path.join(tmp, "bad.bat")
        with open(bad, "w") as f:
            f.write('set "EXE=%~dp0x.exe"\nif exist "%EXE%" (\n  echo found %EXE%\n  echo(ok\n)\n'
                    'if 1==1 (echo a ^) b)\n(\n')
        self.assertEqual(bat_problems(bad, bat_path_vars([bad])),
                         ["3: %EXE% unquoted inside ( )", "( without ) at the end"])



class LauncherConnectionTests(unittest.TestCase):
    def test_the_ssh_command(self):
        src = read(LAUNCHER)
        start = src.index("void Launcher::StartSsh()")
        ssh = src[start:src.index("\n}\n", start)]
        for opt in ('"-F", "none"', '"-i", "remote_key"', '"UserKnownHostsFile=known_hosts"',
                    '"StrictHostKeyChecking=yes"', '"BatchMode=yes"', '"IdentitiesOnly=yes"',
                    '"ServerAliveInterval=15"', '"ExitOnForwardFailure=yes"', '"-T"',
                    '":127.0.0.1:57120"', '":127.0.0.1:57121"', 'user_ + "@" + host_'):
            self.assertIn(opt, ssh)
        self.assertNotIn('"-n"', ssh)  # stdin open: the server ends the session when it closes
        self.assertIn("ssh_->Start(argv, ssh_cwd_", ssh)  # relative key / known_hosts: their folder
        # The ports agree with the server side and the key's permitopen, and the GUI's default.
        session = read(os.path.join(SCRIPTS, "remote_session.sh"))
        self.assertIn("GUI_PORT=57120", session)
        self.assertIn("CARSIM_PORT=57121", session)
        key = read(os.path.join(SCRIPTS, "install_remote_key.sh"))
        self.assertIn(r'permitopen=\"127.0.0.1:57120\",permitopen=\"127.0.0.1:57121\"', key)
        self.assertIn("local_gui_port_ = 57120, local_carsim_port_ = 57121", read(LAUNCHER.replace(".cpp", ".h")))
        # The server script's lines the launcher follows.
        for line in ("已连上云端服务器", "CARLA 已就绪", "后端已就绪", "就绪", "[错误]"):
            self.assertIn(line, session)
            self.assertIn('"%s"' % line, src)
        # The CarSim service's state lines the launcher follows.
        service = read(os.path.join(BRIDGE, "carsim_service.py"))
        for line in ("已连上云端", "运行开始", "运行结束", "正在连接云端", "连接断开", "另一个 CarSim 服务"):
            self.assertIn(line, service)
            self.assertIn('"%s' % line, src)


if __name__ == "__main__":
    unittest.main()
