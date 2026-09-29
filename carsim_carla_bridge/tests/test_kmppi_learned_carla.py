"""KMPPI + 学习世界模型（controllers/kmppi_learned，移植自 ~/Desktop/rl/kmppi_dream）驱动 PyChrono 宝马 E90
（carsim.chrono）在 Town04 高速上，CARLA 服务器：从高速起点（world_info 的 scenario_start）40 s，
参考车速 20 m/s，仿真步长 0.05 s（KMPPI 的控制周期）。世界模型是在这台车的数据上训练的。

检查：后台启动了 Chrono 宝马；运行按时长结束、算法没有出错、有算法的总结；Chrono 的导出变量通过平台的检查；
车从 20 m/s 起步并保持；没有碰撞、一直在行车道上、车道中心偏差的均方根和最大值在界限内；跑出 > 700 m；过弯时前轮
转角 > 1°；候选轨迹每帧画进 CARLA、也到界面的“轨迹”页；诊断量（self.debug）到界面和 log_debug.csv；
仿真步长设成 0.02 s 时按算法的 FRAME_DT 用 0.05 s；--models 时把附带的每个权重都跑一遍并列出结果。
自己起后台（端口 57146）；删掉临时文件。

    python tests/test_kmppi_learned_carla.py [--port 2000] [--chrono-python PATH] [--models]
"""
import csv
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
from test_backend import Conn  # noqa: E402

PORT = 57146
DURATION = 40.0
KDIR = os.path.abspath(os.path.join(HERE, "..", "controllers", "kmppi_learned"))
CONTROLLER = os.path.join(KDIR, "controller.py")
FAILS = []
RMS_LIMIT, MAX_LIMIT = 0.1, 0.5   # m，车道中心偏差（实测 0.022 / 0.168；KMPPI 的物理模型版在这条路上是 0.035 / 0.16）


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + (("  " + str(detail)) if detail != "" else ""), flush=True)
    if not cond:
        FAILS.append(name)


def chrono_python():
    if "--chrono-python" in sys.argv:
        return sys.argv[sys.argv.index("--chrono-python") + 1]
    for p in (os.path.expanduser("~/.conda/envs/chrono/bin/python"), "/opt/anaconda3/envs/chrono/bin/python"):
        if os.path.isfile(p):
            return p
    raise SystemExit("找不到 chrono conda 环境的 python：用 --chrono-python 指定")


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def drive(c, cfg, controller, duration, events=True):
    """一次运行，返回 (info, 结束事件, 用时, 算法输出)。"""
    cfg["run"]["controller"] = {"path": controller, "entry": "Controller"}
    cfg["sync"]["duration"] = duration
    c.events.clear()
    t0 = time.time()
    info = c.call("cosim_start", config=cfg, timeout=300)
    st = None
    while st is None:
        try:
            st = c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in ("finished", "error", "stopped"), 1)
        except Exception:
            st = None
        if time.time() - t0 > duration * 30 + 300:
            raise RuntimeError("the run did not end")
    wall = time.time() - t0
    algo = [e["msg"] for e in c.events if e.get("event") == "log" and e.get("level") == "algo"]
    return info, st, wall, algo


def summarize(info, wall, duration):
    folder = info.get("record_dir") or ""
    k = json.load(open(os.path.join(folder, "run.json"), encoding="utf-8"))["kpi"] or {}
    rows = read_csv(os.path.join(folder, "log.csv"))
    lane = {r["t"]: r for r in read_csv(os.path.join(folder, "log_lane.csv"))}
    offs = [abs(float(lane[r["t"]]["offset"])) for r in rows
            if r["t"] in lane and lane[r["t"]]["offset"] not in ("", None)
            and lane[r["t"]]["in_junction"] not in ("True", "1", "true")]
    rms = math.sqrt(sum(o * o for o in offs) / len(offs)) if offs else float("nan")
    v = [float(r["Vx"]) / 3.6 for r in rows]  # km/h
    wheel = max(abs(float(r["Steer_L1"])) for r in rows)
    return {"k": k, "rows": rows, "offs": offs, "rms": rms, "mx": max(offs) if offs else float("nan"),
            "v": v, "wheel": wheel, "dist": k.get("distance") or 0.0, "rt": duration / wall, "folder": folder}


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    tmp = tempfile.mkdtemp(prefix="cc_kmppi_learned_")
    proc = subprocess.Popen([sys.executable, os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT)],
                            cwd=os.path.join(HERE, ".."), env=dict(os.environ))
    try:
        c = Conn(PORT)
        c.call("connect", host="localhost", port=carla_port, timeout=60)
        if "Town04" not in c.call("world_info")["map"]:
            c.call("load_map", name="Town04_Opt", timeout=400)
        start = c.call("world_info")["scenario_start"]
        cfg = c.call("default_config")
        cfg["carla"]["spawn_index"] = start["index"]
        cfg["carsim"].update(mock=False, remote=False, chrono=True, chrono_init_speed=20.0, chrono_python=chrono_python())
        cfg["run"].update(driver="custom", log_path=os.path.join(tmp, "runs"))
        cfg["sync"].update(frame_dt=0.05)
        cfg["collect"]["sample_period"] = 0.1
        cfg["scene"]["collision"] = "log"
        cfg["scene"]["record"]["lane"] = cfg["scene"]["record"]["lane"] + ["in_junction"]

        info, st, wall, algo = drive(c, cfg, CONTROLLER, DURATION)
        msgs = [e for e in c.events if e.get("event") == "log"]
        for m in algo:
            print("ALGO " + m)
        check("the run ends by its duration", st["state"] == "finished" and st["detail"].startswith("达到设定的运行时长"),
              (st, [e["msg"] for e in msgs if e.get("level") in ("error", "warn")][-5:]))
        check("the backend started the Chrono BMW", any(e["msg"].startswith("联合仿真开始（Chrono 宝马 E90 代替 CarSim）")
                                                     for e in msgs), [e["msg"] for e in msgs][:3])
        check("the algorithm says it loaded the world model on the GPU with the CUDA graph",
              any("学习世界模型" in m and "CUDA graph" in m for m in algo), algo[:2])
        drawn = [e["data"].get("draw_n", 0) for e in c.events if e.get("event") == "telemetry"]
        check("the candidate trajectories are drawn every frame (> 700 segments)",
              len(drawn) > 100 and min(drawn[5:]) > 700, (len(drawn), min(drawn[5:]) if drawn[5:] else None, max(drawn or [0])))
        sent = [e["data"]["draw"] for e in c.events if e.get("event") == "telemetry" and "draw" in e["data"]]
        check("the GUI gets them too (轨迹 tab: 64 candidates, 64 end dots, reference, mean, best)",
              len(sent) > 100 and all(len(d) == 131 for d in sent[5:]), (len(sent), {len(d) for d in sent}))
        dbg = [e["data"]["debug"] for e in c.events if e.get("event") == "telemetry" and "debug" in e["data"]]
        check("the diagnostics reach the GUI (self.debug: ESS about 16, compute time)",
              len(dbg) > 100 and abs(dbg[-1].get("ESS", 0) - 16.0) < 1.0 and dbg[-1].get("计算耗时 (ms)", 0) > 0,
              (len(dbg), dbg[-1] if dbg else None))
        s = summarize(info, wall, DURATION)
        drows = read_csv(os.path.join(s["folder"], "log_debug.csv"))
        check("... and the run record (log_debug.csv, a row per sample)",
              len(drows) > 300 and {"ESS", "温度", "最小代价", "前轮转角 (rad)"} <= set(drows[0]), (len(drows), list(drows[0])[:6]))
        check("no algorithm error", not any("Traceback" in m or "Error" in m for m in algo), algo[-5:])
        warns = [e["msg"] for e in msgs if e.get("level") == "warn" and "导出变量可疑" in e["msg"]]
        check("the Chrono exports pass the platform's sanity checks", not warns, warns[:2])
        check("the algorithm's summary at the end", any(m.startswith("KMPPI（学习世界模型）结束") for m in algo), algo[-2:])
        print("INFO %.0f m in %.0f s wall (%.2fx real time), lane offset rms %.3f max %.3f m (%d samples), "
              "speed %.2f ~ %.2f m/s, max front wheel angle %.2f deg, collisions %s, off lane %s s" % (
                  s["dist"], wall, s["rt"], s["rms"], s["mx"], len(s["offs"]), min(s["v"]), max(s["v"]), s["wheel"],
                  s["k"].get("collisions"), s["k"].get("time_off_lane")))
        check("the Chrono car starts at 20 m/s and holds it (18.5 ~ 21.5 m/s)", 18.5 < min(s["v"]) and max(s["v"]) < 21.5,
              (round(min(s["v"]), 2), round(max(s["v"]), 2)))
        check("no collision", s["k"].get("collisions") == 0, (s["k"].get("collisions"), s["k"].get("first_collision_with")))
        check("never off the driving lanes", (s["k"].get("time_off_lane") or 0.0) == 0.0, s["k"].get("time_off_lane"))
        check("lane offset rms < %.1f m, max < %.1f m" % (RMS_LIMIT, MAX_LIMIT),
              len(s["offs"]) > 300 and s["rms"] < RMS_LIMIT and s["mx"] < MAX_LIMIT, (len(s["offs"]), round(s["rms"], 3), round(s["mx"], 3)))
        check("it covers > 700 m", s["dist"] > 700.0, s["dist"])
        check("it steers through the curve (front wheel angle > 1 deg)", s["wheel"] > 1.0, s["wheel"])

        # 页面上设 0.02 s：按算法的 FRAME_DT（0.05 s）运行并说明。
        cfg["sync"]["frame_dt"] = 0.02
        info, st, wall, algo = drive(c, cfg, CONTROLLER, 2.0)
        logs = [e["msg"] for e in c.events if e.get("event") == "log"]
        check("the page's 0.02 s: the run uses FRAME_DT 0.05 s and says so",
              st["state"] == "finished" and abs(info.get("frame_dt", 0) - 0.05) < 1e-9 and
              "仿真步长按控制算法 controller.py 里的 FRAME_DT 用 0.05 s（界面上设的是 0.02 s）" in logs,
              (st, info.get("frame_dt"), [m for m in logs if "FRAME_DT" in m or "步长" in m][:3]))
        cfg["sync"]["frame_dt"] = 0.05

        if "--models" in sys.argv:   # 附带的每个权重跑一遍（每个 40 s）
            table = []
            models = ["models/world_model.pt", "models/world_model_wide.pt", "models/wm_wide_H2_128.pt", "models/wm_wide_H2_256.pt"]
            models += ["models/peml/" + f for f in sorted(os.listdir(os.path.join(KDIR, "models", "peml"))) if f.endswith(".pt")]
            for m in models:
                cfg["run"]["params"] = {CONTROLLER: {"MODEL": m}}
                info, st, wall, algo = drive(c, cfg, CONTROLLER, DURATION)
                r = summarize(info, wall, DURATION)
                table.append((m, st["state"], r["rms"], r["mx"], r["dist"], r["k"].get("collisions"), r["k"].get("time_off_lane")))
                print("MODEL %-34s %-9s rms %.3f max %.3f m  %.0f m  collisions %s  off lane %s s  (%.2fx)" % (
                    m, st["state"], r["rms"], r["mx"], r["dist"], r["k"].get("collisions"), r["k"].get("time_off_lane"), r["rt"]), flush=True)
            check("every shipped weight ran to the end", all(t[1] == "finished" for t in table), [t[:2] for t in table])
        c.call("destroy_ego")
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            proc.kill()
        shutil.rmtree(tmp, ignore_errors=True)
    print("FAILED: %s" % FAILS if FAILS else "ALL KMPPI LEARNED CARLA TESTS PASSED")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
