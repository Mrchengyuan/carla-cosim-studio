"""“检查 .sim”（CarSim 动力学页）：运行前把 CarSim 试启动一次（和运行时同一条路：本机 CarSim、
远程的 CarSim 服务、Chrono 宝马或模拟 CarSim），读 .sim 实际给出的设置和 t = 0 的导出变量，
和界面上的设置逐项对照，用平常的话说出哪里不一致。不需要 CARLA，不动世界。

检查的项（每项 ok / warn / error）：
  CarSim 能否启动（.sim、python_carsim_env、求解器、许可证：运行时同样的报错）
  导出变量个数 = 界面列表的个数；必需的导出变量都在
  导入变量个数：测试用驾驶方式要 3 个；自己的算法要按这个个数返回
  t_step，和仿真步长（算法的 FRAME_DT 优先）能不能整除
  t_stop 和运行时长
  t = 0 的状态：初始车速；算法参数里有 REF_SPEED（m/s，例如 KMPPI）时，初始车速离它太远要提醒
  t = 0 的导出变量是否可信（NaN、俯仰 / 侧倾 > 10°、左右前轮转角不一致：顺序或单位错了）
"""
import math
import os

import bridge
import session
from carsim_local import reset_env

OK, WARN, ERROR = "ok", "warn", "error"


def _item(level, text):
    return {"level": level, "text": text}


def _source(d):
    c = d["carsim"]
    if c["mock"]:
        return "模拟 CarSim"
    if c.get("chrono"):
        return "Chrono 宝马 E90（服务器）"
    if c.get("remote"):
        return "CarSim（远程：你电脑上的 CarSim 服务），.sim = %s" % c["sim_path"]
    return "CarSim，.sim = %s" % os.path.abspath(c["sim_path"]) if c["sim_path"] else "CarSim"


def check(d, make_env=None):
    """{"items": [{"level", "text"}], "summary": {...}, "ok": no error}. make_env: for tests."""
    items = []
    c, names = d["carsim"], list(d["carsim"]["export_names"])
    try:
        env = (make_env or session.make_env)(d)
        obs = reset_env(env)
    except Exception as e:  # noqa: BLE001 (every reason CarSim could not start, in plain words already)
        items.append(_item(ERROR, "%s 启动不了：%s" % (_source(d), e)))
        return {"items": items, "summary": {}, "ok": False}
    try:
        cfg = dict(getattr(env, "config", None) or {})
        items.append(_item(OK, "%s 已启动（试启动后已关闭）" % _source(d)))
        n_exp, n_imp = int(cfg.get("n_export") or 0), int(cfg.get("n_import") or 0)
        t_step, t_stop = float(cfg.get("t_step") or 0.0), float(cfg.get("t_stop") or 0.0)
        summary = {"n_export": n_exp, "n_import": n_imp, "t_step": t_step, "t_stop": t_stop}

        # Exports: the count, the names the platform needs.
        if n_exp != len(names):
            items.append(_item(ERROR, "导出变量个数不一致：界面列了 %d 个，.sim 里有 %d 个。“CarSim 动力学”页的导出变量要和 .sim 的"
                                      "导出变量同样的个数、同样的顺序" % (len(names), n_exp)))
        else:
            items.append(_item(OK, "导出变量 %d 个，和界面的列表个数一致（顺序只能靠下面 t = 0 的数值判断）" % n_exp))
        missing = [n for n in bridge.REQUIRED_EXPORTS if n not in names]
        if missing:
            items.append(_item(ERROR, "导出变量里缺少必需的 %s" % "、".join(missing)))

        # Imports.
        drv = d["run"]["driver"]
        if drv != "custom" and n_imp != 3:
            items.append(_item(ERROR, "测试用驾驶方式只给 3 个导入变量（油门、制动、方向盘角），.sim 里有 %d 个" % n_imp))
        elif drv == "custom":
            items.append(_item(OK, "导入变量 %d 个：你的算法 control() 要按 .sim 的导入顺序返回 %d 个值" % (n_imp, n_imp)))
        else:
            items.append(_item(OK, "导入变量 3 个（油门、制动、方向盘角）"))

        # t_step and the frame step (the algorithm's FRAME_DT first, as a run does).
        dt = float(d["sync"]["frame_dt"])
        path = d["run"]["controller"]["path"]
        want = session.algorithm_frame_dt(path) if drv == "custom" and os.path.isfile(path) else None
        if want is not None:
            dt = float(session.params_for(d).get("FRAME_DT", want))
        if not t_step > 0:
            items.append(_item(ERROR, "CarSim 给出的 t_step = %r，无法运行" % t_step))
        else:
            n = max(1, int(round(dt / t_step)))
            if abs(n * t_step - dt) > 1e-9:
                items.append(_item(WARN, "仿真步长 %g s 不是 CarSim t_step（%g s）的整数倍：运行时会对齐到 %g s" % (
                    dt, t_step, round(n * t_step, 12))))
            else:
                items.append(_item(OK, "t_step %g s，仿真步长 %g s = %d 个 CarSim 步%s" % (
                    t_step, dt, n, "（算法的 FRAME_DT）" if want is not None else "")))
        dur = float(d["sync"]["duration"] or 0.0)
        if t_stop > 0 and not c["mock"] and not c.get("chrono"):
            if dur == 0:
                items.append(_item(OK, "运行时长 0（一直运行）：最晚在 .sim 的结束时间 t = %g s 停止" % t_stop))
            elif dur > t_stop + 1e-9:
                items.append(_item(WARN, "运行时长 %g s 超过 .sim 的结束时间 t = %g s：会在 %g s 停止（在 CarSim 里把结束时间调长）" % (
                    dur, t_stop, t_stop)))
            else:
                items.append(_item(OK, "运行时长 %g s，.sim 的结束时间 t = %g s" % (dur, t_stop)))

        # t = 0.
        bad = [n for n, v in zip(names, obs) if not math.isfinite(float(v))]
        if bad:
            items.append(_item(ERROR, "t = 0 的导出变量里有无效数值（NaN / 无穷大）：%s" % "、".join(bad[:6])))
        elif not missing and n_exp == len(names):
            ex = bridge.CarSimExports(names, c["units"])
            v = ex.speed(obs, "Vx") if ex.has("Vx") else None
            if v is not None:
                summary["init_speed"] = v
                items.append(_item(OK, "初始车速 %.1f km/h（%.2f m/s）" % (v * 3.6, v)))
                ref = session.params_for(d).get("REF_SPEED")
                if ref is None and drv == "custom" and os.path.isfile(path):
                    ref = next((p["value"] for p in session.controller_params(path).get("params", [])
                                if p["name"] == "REF_SPEED" and isinstance(p["value"], (int, float))), None)
                if isinstance(ref, (int, float)) and ref > 0 and v < 0.5 * ref:
                    items.append(_item(WARN, "算法的参考车速 REF_SPEED = %g m/s（%g km/h），初始车速只有 %.1f km/h：这类算法"
                                             "（例如 KMPPI）要从接近参考车速起步，把 .sim 的初始车速设为约 %g km/h%s" % (
                                                 ref, ref * 3.6, v * 3.6, ref * 3.6,
                                                 "（Chrono / 模拟 CarSim：本页的初始车速）" if c["mock"] or c.get("chrono") else "")))
            warns = bridge.ExportCheck(ex, z0=None)(obs)
            items.extend(_item(WARN, w) for w in warns)
            if not warns:
                items.append(_item(OK, "t = 0 的导出变量看起来可信（俯仰、侧倾、前轮转角正常；其余要等车动起来才能核对）"))
        return {"items": items, "summary": summary, "ok": not any(i["level"] == ERROR for i in items)}
    finally:
        try:
            env.close()
        except Exception:  # noqa: BLE001
            pass
