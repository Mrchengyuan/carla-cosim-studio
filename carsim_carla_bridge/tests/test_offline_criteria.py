"""不需要 CARLA：新的运行指标（scene.RunKpi：最小 TTC、最大加速 / 减速、最大 jerk）和通过标准（criteria.py）。

RunKpi：72 km/h 匀速后以 5 m/s² 刹车：最大减速 5、加速 0、jerk = 5 / 0.1 s = 50 m/s³；m/s 的单位设置结果相同；
前方 20 m、以 10 m/s 接近的目标：TTC 2 s；远离的、旁边车道的目标不算；没有目标时 TTC 为空。
判定：默认（跑完、不碰撞、不出车道）；被停止、出错、碰撞、出车道各说原因；上下限（TTC ≥、减速 ≤ 等）违反时说出数值；
没有数据的一项不算违反；全部关掉时通过；运行指标的一行文字里有新指标。

    python tests/test_offline_criteria.py
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

import criteria  # noqa: E402
import settings  # noqa: E402
from scene import RunKpi, kpi_text  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + (("  " + str(detail)) if detail != "" else ""), flush=True)
    if not cond:
        FAILS.append(name)


def obj(rel_x, rel_y, rel_vx, gap):
    return {"rel_x": rel_x, "rel_y": rel_y, "rel_yaw": 0.0, "length": 4.5, "width": 1.8, "rel_vx": rel_vx, "gap": gap}


def drive(speed_unit="km/h", objects=()):
    k = RunKpi(0.1, True, "deg", speed_unit)
    f = 1.0 if speed_unit == "km/h" else 1 / 3.6
    v = 20.0
    for i in range(40):
        if i >= 20:
            v = max(0.0, v - 0.5)   # 5 m/s² braking
        k.sample({"t": i * 0.1, "ego": {"X": i, "Y": 0.0, "Speed": v * 3.6 * f, "width": 1.8},
                  "lane": {"offset": 0.1, "heading_err": 0.0}, "objects": list(objects)}, {})
    return k.result()


def main():
    r = drive()
    check("braking at 5 m/s² from 72 km/h: 最大减速 5, 加速 0, jerk 50 m/s³ (a step over 0.1 s)",
          abs(r["decel_max"] - 5.0) < 1e-6 and r["accel_max"] == 0.0 and abs(r["jerk_max"] - 50.0) < 1e-6,
          (r["accel_max"], r["decel_max"], r["jerk_max"]))
    r2 = drive("m/s")
    check("the same in m/s units", abs(r2["decel_max"] - 5.0) < 1e-6 and abs(r2["jerk_max"] - 50.0) < 1e-6)
    check("nothing ahead: TTC empty", r["ttc_min"] is None)
    r = drive(objects=[obj(25.0, 0.0, -36.0, 20.0)])
    check("20 m ahead, closing at 10 m/s: TTC 2 s", abs(r["ttc_min"] - 2.0) < 1e-6, r["ttc_min"])
    r = drive(objects=[obj(25.0, 0.0, 36.0, 20.0), obj(25.0, 3.5, -72.0, 20.0)])
    check("ahead but moving away, or closing in the next lane: no TTC", r["ttc_min"] is None, r["ttc_min"])
    r = drive("m/s", objects=[obj(25.0, 0.0, -10.0, 20.0)])
    check("... closing speed in m/s units", abs(r["ttc_min"] - 2.0) < 1e-6, r["ttc_min"])
    txt = kpi_text(drive(objects=[obj(25.0, 0.0, -36.0, 20.0)]))
    check("the output line has TTC, accel / decel and jerk", "最小 TTC 2.00 s" in txt and "减速 5.00 m/s²" in txt and "jerk 50.0" in txt, txt)

    good = {"collisions": 0, "time_off_lane": 0.0, "ttc_min": 3.0, "decel_max": 4.0, "lane_offset_max": 0.2}
    v = criteria.verdict(good, "finished")
    check("default criteria, a clean finished run: passed", v["passed"] and v["checked"] == ["跑完设定的时长", "没有碰撞", "不开出车道"], v)
    fails = {e: criteria.verdict(good, e)["fails"] for e in ("stopped", "error")}
    check("stopped / error: not passed, said", fails == {"stopped": ["没有跑完（被停止）"], "error": ["没有跑完（出错）"]}, fails)
    v = criteria.verdict(dict(good, collisions=2, time_off_lane=1.5), "finished")
    check("collisions and off the lane: both said", v["fails"] == ["碰撞 2 次", "开出车道 1.50 s"], v["fails"])
    c = settings.default_dict()["criteria"]
    c["limits"].update(ttc_min=4.0, decel_max=3.0, lane_offset_max=0.5, jerk_max=10.0)
    v = criteria.verdict(good, "finished", c)
    check("limits: TTC 3 s < 4 s and deceleration 4 > 3 said with the values; a figure without data (jerk) not a failure",
          v["fails"] == ["最小碰撞时间 TTC 3 s < 4 s", "最大减速度 4 m/s² > 3 m/s²"] and len(v["checked"]) == 7, v)
    check("... in one line", criteria.text(v) == "判定：未通过：最小碰撞时间 TTC 3 s < 4 s；最大减速度 4 m/s² > 3 m/s²", criteria.text(v))
    off = {"finished": False, "no_collision": False, "on_lane": False, "limits": {}}
    check("everything off: passed, even stopped with collisions", criteria.verdict(dict(good, collisions=3), "stopped", off)["passed"])
    check("no key figures at all: said", criteria.verdict(None, "finished")["fails"] == ["没有运行指标（没有采样）"])
    check("the settings' default: the three checks, no limits", settings.default_dict()["criteria"] == criteria.DEFAULT)
    print("FAILED: %s" % FAILS if FAILS else "ALL CRITERIA TESTS PASSED")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
