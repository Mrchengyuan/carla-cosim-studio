"""不需要 CARLA：干扰注入（disturb.py，驾驶模式页“干扰”）。

通过平台加载算法的同一个函数（session.load_controller）跑一个记下自己看到什么的算法：
执行器延迟 0.1 s（步长 0.02 s = 5 帧）：CarSim 收到的是 5 帧前的输出，最初 5 帧是第一次的输出；
测量延迟 0.06 s：算法看到的导出变量和场景是 3 帧前的；噪声：标准差和设的一致（±10%）、没加噪声的变量不变、
同一个种子两次完全一样、换种子不一样；车道信息丢失 30%：约 30% 的帧 scene["lane"] 为 None，场景其余不变；
关闭或全为 0 时算法看到的就是原样；超出范围、噪声加在没有的导出变量上时说清；运行前的提示和 run.json 的记录。

    python tests/test_offline_disturb.py
"""
import os
import shutil
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)
os.chdir(ROOT)

import bridge  # noqa: E402
import disturb  # noqa: E402
import session  # noqa: E402
import settings  # noqa: E402

FAILS = []
CTRL = '''
SEEN = []
class Controller:
    def control(self, exports, t, dt, scene):
        SEEN.append((t, dict(exports), scene))
        return [t, 0.0, 0.0]   # the output says which frame it is from
'''


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + (("  " + str(detail)) if detail != "" else ""), flush=True)
    if not cond:
        FAILS.append(name)


def run(dist, n=400, dt=0.02):
    """The algorithm loaded by the platform, n frames: (what it saw, what reached CarSim, the driver)."""
    d = settings.default_dict()
    d["run"]["controller"] = {"path": CTRL_PATH, "entry": "Controller"}
    d["sync"]["frame_dt"] = dt
    d["run"]["disturb"] = dict(settings.default_dict()["run"]["disturb"], **dist)
    names = d["carsim"]["export_names"]
    ex = bridge.CarSimExports(names, d["carsim"]["units"])
    frame = {"k": 0}

    def scene():
        return {"t": frame["k"], "lane": {"offset": 0.1 * frame["k"]}, "objects": [frame["k"]]}
    drv = session.load_controller(d, ex, lambda: 3, scene)
    mod = sys.modules["user_controller"]
    sent = []
    for k in range(n):
        frame["k"] = k
        obs = [float(k)] * len(names)   # every export = the frame number
        sent.append(drv(obs, k * dt))
    return mod.SEEN, sent, drv


def main():
    global CTRL_PATH
    tmp = tempfile.mkdtemp(prefix="cc_disturb_test_")
    CTRL_PATH = os.path.join(tmp, "seeing_controller.py")
    open(CTRL_PATH, "w", encoding="utf-8").write(CTRL)

    seen, sent, _ = run({"enabled": False, "act_delay": 0.1, "noise": {"Vx": 1.0}})
    check("off: the algorithm sees the exports and the scene as they are, CarSim gets its output at once",
          all(s[1]["Vx"] == k and s[2]["t"] == k for k, s in enumerate(seen)) and all(o[0] == k * 0.02 for k, o in enumerate(sent)))
    seen, sent, _ = run({"enabled": True})
    check("on but all 0: the same as off", all(s[1]["Vx"] == k for k, s in enumerate(seen)) and all(o[0] == k * 0.02 for k, o in enumerate(sent)))

    seen, sent, _ = run({"enabled": True, "act_delay": 0.1})
    check("执行器延迟 0.1 s at 0.02 s (5 frames): CarSim gets the output of 5 frames ago, the first one before",
          all(abs(o[0] - max(0, k - 5) * 0.02) < 1e-12 for k, o in enumerate(sent)), [round(o[0], 2) for o in sent[:8]])
    seen, sent, _ = run({"enabled": True, "sense_delay": 0.06})
    check("测量延迟 0.06 s (3 frames): the exports and the scene of 3 frames ago, the first ones before",
          all(s[1]["Vx"] == max(0, k - 3) and s[2]["t"] == max(0, k - 3) for k, s in enumerate(seen)),
          [s[1]["Vx"] for s in seen[:6]])

    seen, _, _ = run({"enabled": True, "noise": {"Vx": 0.5, "AVz": 2.0}, "seed": 3}, n=3000)
    ev = np.array([s[1]["Vx"] - k for k, s in enumerate(seen)])
    ea = np.array([s[1]["AVz"] - k for k, s in enumerate(seen)])
    check("噪声: the standard deviations set (±10%), mean about 0", abs(ev.std() / 0.5 - 1) < 0.1 and abs(ea.std() / 2.0 - 1) < 0.1
          and abs(ev.mean()) < 0.05, (round(ev.std(), 3), round(ea.std(), 3)))
    check("... the other exports untouched", all(s[1]["Yaw"] == k for k, s in enumerate(seen)))
    again, _, _ = run({"enabled": True, "noise": {"Vx": 0.5, "AVz": 2.0}, "seed": 3}, n=3000)
    other, _, _ = run({"enabled": True, "noise": {"Vx": 0.5, "AVz": 2.0}, "seed": 4}, n=3000)
    check("... the same seed: the same noise; another seed: other noise",
          [s[1]["Vx"] for s in again] == [s[1]["Vx"] for s in seen] and [s[1]["Vx"] for s in other] != [s[1]["Vx"] for s in seen])

    seen, _, drv = run({"enabled": True, "lane_dropout": 0.3}, n=3000)
    lost = [s[2]["lane"] is None for s in seen]
    check("车道信息丢失 30%: about 30% of the frames without scene['lane'], the rest of the scene kept",
          abs(np.mean(lost) - 0.3) < 0.03 and all(s[2]["objects"] == [k] for k, s in enumerate(seen)) and drv.disturb.dropped == sum(lost),
          round(float(np.mean(lost)), 3))

    d = settings.default_dict()
    d["run"]["disturb"].update(enabled=True, act_delay=0.1, noise={"Vx": 0.5}, lane_dropout=0.1, seed=2)
    notes = session.check_run_config(d)
    check("before the run: what is on, said", any(n.startswith("干扰已开启") and "执行器延迟 0.1 s（5 帧）" in n and "Vx ±0.5" in n
                                                 for n in notes), notes)
    d2 = settings.default_dict()
    d2["run"]["disturb"].update(enabled=True, act_delay=0.1)
    d2["run"]["driver"] = "demo"
    check("... a test driver: not used, said", any("只作用在自己的控制算法上" in n for n in session.check_run_config(d2)))

    def refused(**kw):
        d = settings.default_dict()
        d["run"]["disturb"].update(enabled=True, **kw)
        try:
            session.check_run_config(d)
            return ""
        except ValueError as e:
            return str(e)
    e1, e2, e3 = refused(act_delay=2.0), refused(lane_dropout=1.5), refused(noise={"Speed_x": 1.0})
    check("out of range or an export that is not there: refused in plain words",
          "执行器延迟 2 s 不在" in e1 and "丢失概率 1.5" in e2 and "Speed_x" in e3, (e1, e2, e3))
    check("the settings in run.json (disturb.config)", disturb.config(d) == {"act_delay": 0.1, "sense_delay": 0.0, "noise": {"Vx": 0.5},
                                                                            "lane_dropout": 0.1, "seed": 2})
    shutil.rmtree(tmp, ignore_errors=True)
    print("FAILED: %s" % FAILS if FAILS else "ALL DISTURB TESTS PASSED")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
