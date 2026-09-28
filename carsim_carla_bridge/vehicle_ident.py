"""车辆参数辨识：从一次运行的记录（CarSim / Chrono 宝马 / 模拟 CarSim 的导出变量）辨识 KMPPI 预测模型
（3DOF 动态自行车 + Pacejka，controllers/kmppi/prediction_model.py）的前 / 后轴轮胎侧向力比例因子。

方法和原工程 kmppi_chrono/identify_plant.py 一样：质量 m、横摆惯量 I、质心到前 / 后轴 a / b 已知（CarSim
车辆参数里查），由 3DOF 力平衡反推每轴的侧向力，再除以 Pacejka(侧偏角) 得比例因子。原工程只用阶跃转向的
稳态点（vy'、r' 为 0）；这里用记录里的每一个样本（vy'、r' 用差分），两个方程一起做最小二乘：

    m (vy' + vx r)   = 2 sf Pac(αf) cos δ + 2 sr Pac(αr)
    I r' / (L / 2)   = (a · 2 sf Pac(αf) cos δ − b · 2 sr Pac(αr)) / (L / 2)
    αf = δ − atan2(vy + a r, vx)，αr = −atan2(vy − b r, vx)，δ = 前轮平均转角

用到的导出变量：Vx、Vy（参考点 = 前轴中心，换到质心：vy = Vy − a r）、AVz、Steer_L1、Steer_R1，
单位按那次运行的 run.json。要有足够的转向（侧向加速度最大至少 1 m/s²），车速 > 5 m/s 的样本才用。
"""
import json
import math
import os

import numpy as np

import runs as runsmod

NEED = ("Vx", "Vy", "AVz", "Steer_L1", "Steer_R1")
MIN_SPEED = 5.0     # m/s，低于它的样本不用（侧偏角没有意义）
MIN_AY = 1.0        # m/s²，这次运行最大的侧向加速度至少要这么大
PAC = {"pac_By": 19.4, "pac_Cy": 1.3, "pac_Ey": -1.0, "pac_D_ratio": 0.95}   # 和 kmppi_config.BicycleParams 相同


def pacejka(alpha, B, C, D, E):
    Ba = B * alpha
    return D * np.sin(C * np.arctan(Ba - E * (Ba - np.arctan(Ba))))


def _series(folder):
    folder = os.path.abspath(folder)
    run = runsmod._read_json(os.path.join(folder, "run.json"))
    if run is None:
        raise ValueError("不是一次运行的记录（没有 run.json）：%s" % folder)
    rows = runsmod._rows(os.path.join(folder, "log.csv"))
    if not rows:
        raise ValueError("这次运行的记录里没有数据（log.csv 是空的）")
    missing = [n for n in NEED if n not in rows[0]]
    if missing:
        raise ValueError("这次运行的记录里没有导出变量 %s：要用 CarSim（或 Chrono 宝马、模拟 CarSim）跑、"
                         "“场景信息”页的记录里保留全部导出变量" % "、".join(missing))
    u = run.get("units") or {}
    deg = math.pi / 180.0 if u.get("angle", "deg") == "deg" else 1.0
    mps = 1.0 / 3.6 if u.get("speed", "km/h") == "km/h" else 1.0
    rps = math.pi / 180.0 if u.get("rate", "deg/s") == "deg/s" else 1.0
    cols = {k: [] for k in ("t",) + NEED}
    for r in rows:
        v = [runsmod._num(r.get(k)) for k in cols]
        if None in v:
            continue
        for k, x in zip(cols, v):
            cols[k].append(x)
    s = {k: np.asarray(v, dtype=float) for k, v in cols.items()}
    return {"t": s["t"], "vx": s["Vx"] * mps, "vy_ref": s["Vy"] * mps, "r": s["AVz"] * rps,
            "delta": 0.5 * (s["Steer_L1"] + s["Steer_R1"]) * deg}


def identify(folder, m, I, a, b, pac=None):
    """辨识 front_lateral_scale、rear_lateral_scale。返回 {"front_lateral_scale", "rear_lateral_scale",
    "samples", "ay_max", "alpha_f_max", "alpha_r_max", "r2", "rms_force"}。"""
    m, I, a, b = float(m), float(I), float(a), float(b)
    if min(m, I, a, b) <= 0:
        raise ValueError("质量、横摆惯量、质心到前 / 后轴的距离都要大于 0")
    p = dict(PAC, **(pac or {}))
    s = _series(folder)
    t = s["t"]
    if len(t) < 5:
        raise ValueError("这次运行的记录太短（%d 行）" % len(t))
    vx, r, delta = s["vx"], s["r"], s["delta"]
    vy = s["vy_ref"] - a * r                      # 前轴中心 → 质心
    vy_dot, r_dot = np.gradient(vy, t), np.gradient(r, t)
    ay = vy_dot + vx * r
    use = vx > MIN_SPEED
    use[0] = use[-1] = False                      # 端点的差分是单边的
    if use.sum() < 5:
        raise ValueError("车速超过 %.0f m/s 的样本太少（%d 个）" % (MIN_SPEED, int(use.sum())))
    ay_max = float(np.max(np.abs(ay[use])))
    if ay_max < MIN_AY:
        raise ValueError("这次运行转向太少，辨识不出轮胎：最大侧向加速度 %.2f m/s²，至少要 %.0f m/s²"
                         "（用有弯道的路线，或者蛇行 / 阶跃转向）" % (ay_max, MIN_AY))
    D = p["pac_D_ratio"] * m * 9.81 / 4.0
    af = delta - np.arctan2(vy + a * r, vx)
    ar = -np.arctan2(vy - b * r, vx)
    pf = 2.0 * pacejka(af, p["pac_By"], p["pac_Cy"], D, p["pac_Ey"]) * np.cos(delta)
    pr = 2.0 * pacejka(ar, p["pac_By"], p["pac_Cy"], D, p["pac_Ey"])
    h = 0.5 * (a + b)
    A = np.vstack([np.stack([pf, pr], axis=1)[use], np.stack([a * pf / h, -b * pr / h], axis=1)[use]])
    y = np.concatenate([(m * ay)[use], (I * r_dot / h)[use]])
    (sf, sr), *_ = np.linalg.lstsq(A, y, rcond=None)
    res = y - A @ np.array([sf, sr])
    r2 = 1.0 - float(np.sum(res ** 2)) / max(float(np.sum((y - y.mean()) ** 2)), 1e-12)
    return {"front_lateral_scale": float(sf), "rear_lateral_scale": float(sr), "samples": int(use.sum()),
            "ay_max": ay_max, "alpha_f_max": float(np.max(np.abs(af[use]))), "alpha_r_max": float(np.max(np.abs(ar[use]))),
            "r2": r2, "rms_force": float(np.sqrt(np.mean(res ** 2)))}


def save(path, folder, m, I, a, b, result, pac=None):
    """写成 KMPPI 读的车辆参数文件（和 bmw_e90_identified.json 同样的键）。"""
    p = dict(PAC, **(pac or {}))
    out = {"m": float(m), "I": float(I), "a": float(a), "b": float(b),
           "pac_By": p["pac_By"], "pac_Cy": p["pac_Cy"], "pac_Ey": p["pac_Ey"],
           "front_lateral_scale": result["front_lateral_scale"], "rear_lateral_scale": result["rear_lateral_scale"],
           "identified_from": os.path.abspath(folder),
           "fit": {k: result[k] for k in ("samples", "ay_max", "alpha_f_max", "alpha_r_max", "r2", "rms_force")}}
    path = os.path.abspath(path)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    return path
