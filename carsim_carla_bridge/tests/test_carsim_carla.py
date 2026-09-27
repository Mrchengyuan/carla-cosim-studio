"""CarSim first-run behaviour against a running CARLA server: a bad config is
refused before the ego is respawned, the frame period is aligned to CarSim's
t_step (CARLA runs on it too), mock CarSim is named in the log, pauses do not
count in the real-time factor, and runs on a fake CarSim solver (the real
python_carsim_env, see test_offline_carsim.py) end with the right reason.

    python tests/test_carsim_carla.py [--port 2000]
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
from test_backend import Conn, check  # noqa: E402
from test_offline_carsim import NAMES, build_fake_solver, can_build, copy_repo, write_sim  # noqa: E402

PORT = 57186


def expect_error(c, cmd, text, **args):
    try:
        c.call(cmd, **args)
    except RuntimeError as e:
        return text in str(e), str(e)[-90:]
    return False, "no error"


def run_until(c, states, timeout=60):
    return c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in states, timeout)


def logs(c):
    return "\n".join(e.get("msg", "") for e in c.events if e.get("event") == "log")


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    import carla
    cl = carla.Client("localhost", carla_port)
    cl.set_timeout(60)
    w = cl.get_world()
    tmp = tempfile.mkdtemp(prefix="cc_carsim_carla_")
    proc = subprocess.Popen([sys.executable, os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT)],
                            cwd=os.path.join(HERE, ".."))
    try:
        time.sleep(2)
        c = Conn(PORT)
        c.call("connect", host="localhost", port=carla_port)
        base = c.call("default_config")
        base["carsim"]["mock"] = True
        base["run"]["log_path"] = ""
        base["run"]["driver"] = "demo"
        base["carla"]["spawn_index"] = 3
        c.call("spawn_ego", blueprint="vehicle.tesla.model3", spawn_index=3)

        def cfg(**sections):
            d = json.loads(json.dumps(base))
            for k, v in sections.items():
                d[k].update(v)
            return d

        # ---- refused before the world changes: the ego is not respawned -------
        ego0 = c.call("world_info")["ego_id"]
        for name, bad, text in (
                ("frame_dt above CARLA's 0.1 s", cfg(sync={"frame_dt": 0.5}), "仿真步长"),
                ("unknown driver", cfg(run={"driver": "pid"}), "未知的驾驶方式"),
                ("missing .sim", cfg(carsim={"mock": False, "sim_path": os.path.join(tmp, "no.sim")}), "不存在"),
                ("wrong python_carsim_env", cfg(carsim={"mock": False, "sim_path": write_sim(os.path.join(tmp, "x.sim"), tmp),
                                                        "repo_path": tmp}), "python_carsim_env 目录不对")):
            ok, msg = expect_error(c, "cosim_start", text, config=bad)
            check("%s refused in plain words" % name, ok, msg)
            check("%s: ego kept" % name, c.call("world_info")["ego_id"] == ego0 and c.call("world_info")["cosim_state"] != "running")

        # ---- frame period aligned to t_step, CARLA on the same clock ----------
        c.events.clear()
        info = c.call("cosim_start", config=cfg(sync={"frame_dt": 0.0333, "duration": 0.0}))
        check("frame_dt aligned to t_step", info["frame_dt"] == 0.033 and info["inner_steps"] == 33, info)
        check("CARLA runs on the aligned period", abs(w.get_settings().fixed_delta_seconds - 0.033) < 1e-9,
              w.get_settings().fixed_delta_seconds)
        tel = c.wait_event(lambda e: e.get("event") == "telemetry" and e["data"]["frame"] >= 10)["data"]
        check("CarSim time = frames x aligned period", abs(tel["t"] - tel["frame"] * 0.033) < 1e-6, tel["t"])
        c.call("cosim_stop")
        text = logs(c)
        check("log names mock CarSim and the alignment", "联合仿真开始（模拟 CarSim）" in text and "仿真步长已对齐" in text, text[-200:])

        # ---- real-time factor without the pause --------------------------------
        c.events.clear()
        c.call("cosim_start", config=cfg(sync={"frame_dt": 0.02, "duration": 0.0}))
        before = c.wait_event(lambda e: e.get("event") == "telemetry" and e["data"]["t"] > 1.0)["data"]
        c.call("cosim_pause")
        time.sleep(3.0)
        c.call("cosim_resume")
        c.events.clear()
        after = c.wait_event(lambda e: e.get("event") == "telemetry" and e["data"]["t"] > before["t"] + 0.5)["data"]
        check("real-time factor ignores the pause", after["rt_factor"] > 0.5 * before["rt_factor"],
              "%.2f before, %.2f after a 3 s pause" % (before["rt_factor"], after["rt_factor"]))
        c.call("cosim_stop")

        # ---- the real python_carsim_env on a fake solver -----------------------
        if can_build():
            repo, prog = os.path.join(tmp, "repo"), os.path.join(tmp, "prog")
            os.makedirs(repo)
            os.makedirs(prog)
            copy_repo(repo)
            build_fake_solver(prog)

            def solver_run(name, **fake):
                sim = write_sim(os.path.join(tmp, name), prog, nexp=len(NAMES), **fake)
                return cfg(carsim={"mock": False, "sim_path": sim, "repo_path": repo, "export_names": NAMES},
                           sync={"frame_dt": 0.02, "duration": 0.0})

            c.events.clear()
            c.call("cosim_start", config=solver_run("stop.sim", tstop=5.0, stop_at=1.0, x0=40))
            st = run_until(c, ("finished", "error"))
            check("model stop is reported as such", st["state"] == "finished" and "CarSim 模型请求停止" in st.get("detail", ""),
                  st.get("detail"))
            text = logs(c)
            check("t_stop and the off-origin start are logged", "结束时间 t = 5.0 s" in text and "不在原点" in text, text[-200:])

            c.events.clear()
            c.call("cosim_start", config=solver_run("tstop.sim", tstop=1.0))
            st = run_until(c, ("finished", "error"))
            check("reaching t_stop is a normal end", st["state"] == "finished" and "结束时间" in st.get("detail", ""),
                  st.get("detail"))

            ok, msg = expect_error(c, "cosim_start", "License not available", config=solver_run("lic.sim", mode=1))
            check("solver's own message when the run cannot start", ok, msg)
            check("backend answers after that", c.call("ping") == "pong")
        else:
            print("SKIP fake-solver runs (needs Linux, a C compiler and python_carsim_env)")
        c.call("destroy_ego")
    finally:
        proc.terminate()
        proc.wait()
        shutil.rmtree(tmp, ignore_errors=True)
    print("ALL CARSIM TESTS PASSED")


if __name__ == "__main__":
    main()
