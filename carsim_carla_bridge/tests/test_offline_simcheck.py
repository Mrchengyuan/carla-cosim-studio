"""不需要 CARLA：“检查 .sim”（simcheck.py）。

模拟 CarSim：启动、导出 / 导入变量个数、t_step、初始车速都报告；KMPPI（REF_SPEED = 20 m/s）从 0 起步时提醒，
初始车速 20 m/s 时不提醒；算法的 FRAME_DT 用来核对步长。用假的 CarSim 环境检查每种问题：导出变量个数不一致、
测试用驾驶方式的导入不是 3 个、t_step 不能整除仿真步长、运行时长超过 .sim 的结束时间、t = 0 有 NaN、
导出变量顺序错（俯仰读到了车速）、CarSim 启动不了；试启动后一定关闭。Chrono 宝马（服务器上的 conda 环境 chrono）：
真的启动一次，初始车速 72 km/h。

    python tests/test_offline_simcheck.py
"""
import copy
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)
os.chdir(ROOT)

import chrono_local  # noqa: E402
import settings  # noqa: E402
import simcheck  # noqa: E402

FAILS = []
KMPPI = "controllers/kmppi/controller.py"


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + (("  " + str(detail)) if detail != "" else ""), flush=True)
    if not cond:
        FAILS.append(name)


def texts(r, level=None):
    return [i["text"] for i in r["items"] if level is None or i["level"] == level]


class FakeEnv:
    """A CarSim env with what a .sim would give."""
    closed = 0

    def __init__(self, names, n_export=None, n_import=3, t_step=0.0005, t_stop=20.0, values=None, fail=None):
        self.names, self.fail = names, fail
        self.config = {"n_export": len(names) if n_export is None else n_export, "n_import": n_import,
                       "t_step": t_step, "t_stop": t_stop}
        self.values = values or {}

    def reset(self):
        if self.fail:
            raise RuntimeError(self.fail)
        return tuple(float(self.values.get(n, 0.0)) for n in self.names)

    def close(self):
        FakeEnv.closed += 1


def cfg(**kw):
    d = settings.default_dict()
    for k, v in kw.items():
        a, b = k.split("__")
        d[a][b] = v
    return d


def main():
    names = settings.default_dict()["carsim"]["export_names"]
    # The mock.
    d = cfg(carsim__mock=True, sync__duration=10.0)
    r = simcheck.check(d)
    check("mock: starts, no error, the counts and the step said", r["ok"] and any("模拟 CarSim 已启动" in t for t in texts(r))
          and any("导出变量 %d 个" % len(names) in t for t in texts(r)) and any("t_step" in t for t in texts(r)), texts(r))
    check("... the initial speed", any("初始车速 0.0 km/h" in t for t in texts(r)) and r["summary"]["init_speed"] == 0.0)
    d = cfg(carsim__mock=True, sync__duration=10.0)
    d["run"]["controller"] = {"path": KMPPI, "entry": "Controller"}
    r = simcheck.check(d)
    check("KMPPI (REF_SPEED 20 m/s) from 0 km/h: a warning with the speed to set", any("REF_SPEED = 20" in t and "72 km/h" in t
                                                                                     for t in texts(r, "warn")), texts(r, "warn"))
    check("... its FRAME_DT used for the step", any("算法的 FRAME_DT" in t and "0.05 s" in t for t in texts(r, "ok")), texts(r, "ok")[:4])
    d["carsim"]["mock_init_speed"] = 20.0
    r = simcheck.check(d)
    check("... from 20 m/s: no warning", not texts(r, "warn") and any("初始车速 72.0 km/h" in t for t in texts(r)), texts(r, "warn"))

    # Every kind of problem, with a made-up CarSim.
    real = cfg(sync__duration=10.0, sync__frame_dt=0.02)
    real["carsim"]["sim_path"] = "/x/simfile.sim"

    def run(env, d=None):
        return simcheck.check(copy.deepcopy(d or real), make_env=lambda _d: env)
    r = run(FakeEnv(names))
    check("a made-up CarSim that matches: no problem", r["ok"] and not texts(r, "warn"), texts(r))
    r = run(FakeEnv(names, n_export=30))
    check("30 exports in the .sim, 26 on the page: an error", not r["ok"] and any("界面列了 26 个，.sim 里有 30 个" in t
                                                                                 for t in texts(r, "error")), texts(r, "error"))
    dm = copy.deepcopy(real)
    dm["run"]["driver"] = "demo"
    r = run(FakeEnv(names, n_import=4), dm)
    check("a test driver and 4 imports: an error", any("只给 3 个导入变量" in t for t in texts(r, "error")))
    r = run(FakeEnv(names, t_step=0.003))
    check("t_step 0.003 s, frame 0.02 s: aligned, said", any("不是 CarSim t_step" in t and "0.021" in t for t in texts(r, "warn")),
          texts(r, "warn"))
    r = run(FakeEnv(names, t_stop=5.0))
    check("duration 10 s past the .sim's end 5 s: said", any("超过 .sim 的结束时间" in t for t in texts(r, "warn")), texts(r, "warn"))
    r = run(FakeEnv(names, values={"Yo": float("nan")}))
    check("NaN at t = 0: an error", any("NaN" in t and "Yo" in t for t in texts(r, "error")))
    r = run(FakeEnv(names, values={"Pitch": 72.0}))
    check("the exports in another order (Pitch reads 72): the suspicious-export warning",
          any("导出变量可疑" in t and "Pitch" in t for t in texts(r, "warn")), texts(r, "warn"))
    FakeEnv.closed = 0
    run(FakeEnv(names))
    r = run(FakeEnv(names, fail="CarSim 没能开始这次运行：许可证不可用"))
    check("CarSim cannot start: an error with its reason", not r["ok"] and any("启动不了" in t and "许可证不可用" in t
                                                                          for t in texts(r, "error")), texts(r))
    check("... every started one closed again", FakeEnv.closed == 1, FakeEnv.closed)

    # The Chrono BMW, started for real.
    py = chrono_local.find_python("")
    if py:
        d = cfg(carsim__chrono=True, sync__duration=10.0)
        d["run"]["controller"] = {"path": KMPPI, "entry": "Controller"}
        d["carsim"]["chrono_init_speed"] = 20.0
        try:
            r = simcheck.check(d)
        finally:
            chrono_local.stop()
        v = r["summary"].get("init_speed")
        check("Chrono BMW: started, 72 km/h, no warning", r["ok"] and v is not None and abs(v - 20.0) < 0.5 and not texts(r, "warn"),
              (v, texts(r, "warn"), texts(r, "error")))
    else:
        print("INFO no chrono environment: its check skipped")
    print("FAILED: %s" % FAILS if FAILS else "ALL SIMCHECK TESTS PASSED")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
