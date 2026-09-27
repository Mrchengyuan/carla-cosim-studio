"""The launcher and stop scripts against a real, running CARLA server. CARLA is
neither started, stopped nor connected to here: tmux, pkill, pgrep, zenity, the
GUI and the backend Python are stubs for the scripts; the real pgrep and ss
only read.

- start_studio.sh recognises the CARLA on the port by the listening process's
  name and opens the GUI on that port, without connecting to CARLA itself;
- the stop pattern of carla_stop_lib.sh matches the command line of the server
  listening on the port (this installation's CARLA_ROOT / CARLA_SRC, resolved
  through symlinks).

Start CARLA with this installation's scripts first (scripts/carla_server.sh,
or scripts/carla_mod_server.sh for --mod; from another checkout export its
CARLA_ROOT / CARLA_SRC), then:

    python tests/test_tests_carla.py            # original CARLA on 2000
    python tests/test_tests_carla.py --mod      # modified CARLA on 3000
"""
import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from test_backend import check  # noqa: E402

SCRIPTS = os.path.join(HERE, "..", "..", "scripts")


def read(path):
    with open(path) as f:
        return f.read()


def stub(path, body):
    with open(path, "w") as f:
        f.write("#!/bin/bash\n" + body + "\n")
    os.chmod(path, 0o755)


def main():
    mod = "--mod" in sys.argv
    kind, port = ("mod", 3000) if mod else ("stock", 2000)
    tmp = tempfile.mkdtemp(prefix="cc_tests_carla_")
    try:
        bin_, calls, pats = os.path.join(tmp, "bin"), os.path.join(tmp, "calls"), os.path.join(tmp, "patterns")
        os.mkdir(bin_)
        for name in ("tmux", "pgrep", "zenity"):
            stub(os.path.join(bin_, name), 'echo "%s $*" >> "%s"\nexit 1' % (name, calls))
        stub(os.path.join(bin_, "pkill"), 'echo "pkill $*" >> "%s"\nprintf "%%s\\n" "${@: -1}" >> "%s"' % (calls, pats))
        stub(os.path.join(tmp, "gui"), 'echo "gui $*" >> "%s"' % calls)
        stub(os.path.join(tmp, "python"), 'echo "python $*" >> "%s"\nexit 1' % calls)
        env = dict(os.environ, PATH=bin_ + os.pathsep + os.environ["PATH"], XDG_CACHE_HOME=tmp,
                   STUDIO_BIN=os.path.join(tmp, "gui"), COSIM_PYTHON=os.path.join(tmp, "python"),
                   CARLA_PORT="2000", CARLA_MOD_PORT="3000")

        p = subprocess.run(["bash", os.path.join(SCRIPTS, "start_studio.sh")] + (["mod"] if mod else []),
                           env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=120)
        got = read(calls).splitlines() if os.path.exists(calls) else []
        check("start_studio.sh opens the GUI on the running CARLA", p.returncode == 0 and any(
            c.startswith("gui ") and "--carla-port %d --auto-connect" % port in c for c in got),
            "exit %d, calls %s, output %s" % (p.returncode, got, p.stdout.decode(errors="replace")[-300:]))
        check("CARLA recognised by its process name, not by connecting", not [
            c for c in got if c.startswith(("python", "tmux", "pkill", "zenity"))], got)

        subprocess.run(["bash", "-c", 'source "%s"; stop_carla_server %s test_tests_carla'
                        % (os.path.join(SCRIPTS, "carla_stop_lib.sh"), kind)], env=env, check=True, timeout=60)
        pat = read(pats).splitlines()[0]
        listener = subprocess.run(["ss", "-ltnpH", "sport = :%d" % port], stdout=subprocess.PIPE, text=True).stdout
        pid = re.search(r"pid=(\d+)", listener)
        found = subprocess.run(["pgrep", "-f", pat], stdout=subprocess.PIPE, text=True).stdout.split()
        check("the %s stop pattern matches the server on port %d" % (kind, port), pid and pid.group(1) in found,
              "pattern %s, listener %s, matches %s" % (pat, listener.strip(), found))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print("ALL LAUNCHER TESTS PASSED")


if __name__ == "__main__":
    main()
