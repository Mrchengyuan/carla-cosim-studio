"""不需要 CARLA：KMPPI + 学习世界模型（controllers/kmppi_learned，移植自 ~/Desktop/rl/kmppi_dream）。

  · 你的文件原样：kmppi_controller.py、dream_config.py 和 ~/rl/kmppi_dream 里的逐字节相同（有 ~/rl 时核对），
    world_model.py 只差 import 那一行；
  · CUDA graph 版推演（fast_rollout.py）和原函数（world_model.py 的 rollout_cost_torch）算的一样：同一组 40 次闭环计算，
    每条候选的代价相对差 < 1e-5（float32 舍入以内）、输出的 [ax, delta] 差 < 1e-5、画线用的候选位置差 < 1e-3 m；
    并且快得多；
  · 参考：直路上 (T, 6) 的 [x, y, yaw, vx, vy, r]，vy 是世界模型稳态表的值；
  · 出错要说清楚：权重不存在、仿真步长不能整除 0.05 s、没有车道信息；
  · 附带的每个权重（world_model、world_model_wide、wm_wide_H2_128 / 256、peml/*）都能加载、给出有限的输出。
没有 NVIDIA 显卡的机器上只检查 CPU 的部分。

    python tests/test_offline_kmppi_learned.py
"""
import contextlib
import io
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
KDIR = os.path.abspath(os.path.join(HERE, "..", "controllers", "kmppi_learned"))
sys.path.insert(0, KDIR)

import torch  # noqa: E402

import controller as ctl  # noqa: E402

FAILS = []
UNITS = {"angle": "deg", "speed": "km/h", "rate": "deg/s"}


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + (("  " + str(detail)) if detail != "" else ""), flush=True)
    if not cond:
        FAILS.append(name)


def quiet(fn, *a, **k):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        out = fn(*a, **k)
    return out, buf.getvalue()


def lane(offset=0.0, curve=0.0):
    """2 m 间隔、前方 60 m 的中心线（自车坐标）；车在车道里偏 offset（中心线在车的左边 offset），curve = 曲率 1/m。"""
    s = np.arange(0.0, 62.0, 2.0)
    if abs(curve) < 1e-9:
        return [[float(x), float(offset)] for x in s]
    R = 1.0 / curve
    return [[float(R * math.sin(x / R)), float(offset + R * (1 - math.cos(x / R)))] for x in s]


def scene(offset=0.0, curve=0.0):
    return {"units": UNITS, "lane": {"center_rel": lane(offset, curve), "offset": -offset}}


def new(graph, device="auto", model=None):
    ctl.USE_CUDA_GRAPH, ctl.DEVICE = graph, device
    if model is not None:
        ctl.MODEL = model
    c = ctl.Controller()
    _, out = quiet(c.reset)
    return c, out


def closed_loop(c, n=40):
    """同一串合成的“状态”上 n 次计算：车道在偏移和弯道里变，速度在 20 m/s 附近变。"""
    out = []
    for i in range(n):
        ex = {"Vx": 72.0 + 2.0 * math.sin(0.3 * i), "Vy": 0.2 * math.cos(0.2 * i), "AVz": 1.0 * math.sin(0.25 * i)}
        sc = scene(0.4 * math.sin(0.15 * i), 0.01 * math.sin(0.1 * i))
        a, _ = quiet(c.control, ex, i * 0.05, 0.05, sc)
        out.append((np.array(a), c.ctrl.last_total_cost.copy()))
    return out


def main():
    # ---- 你的文件原样
    R = os.path.expanduser("~/rl/kmppi_dream")
    if os.path.isdir(R):
        same = lambda a, b: open(os.path.join(R, a), "rb").read() == open(os.path.join(KDIR, b), "rb").read()
        check("kmppi_controller.py 和 dream_config.py 与 ~/rl/kmppi_dream 的逐字节相同",
              same("kmppi_controller.py", "kmppi_controller.py") and same("config.py", "dream_config.py"))
        a = open(os.path.join(R, "world_model.py"), encoding="utf-8").read()
        b = open(os.path.join(KDIR, "world_model.py"), encoding="utf-8").read()
        check("world_model.py 只差 import 那一行（config → dream_config）",
              a.replace("from config import WorldModelConfig", "from dream_config import WorldModelConfig") == b)
    else:
        print("INFO 没有 ~/rl/kmppi_dream：跳过原文件核对")

    cuda = torch.cuda.is_available()
    print("INFO CUDA:", cuda)
    if cuda:
        # ---- CUDA graph 版和原函数一样
        a, txt = new(graph=False)
        b, txt_b = new(graph=True)
        check("graph = False: 原来的路径；graph = True: 说明用了 CUDA graph", b.graph is not None and a.graph is None
              and "CUDA graph" in txt_b and "CUDA graph" not in txt, txt_b.strip())
        ra, rb = closed_loop(a), closed_loop(b)
        dc = max(float(np.max(np.abs(x[1] - y[1]) / np.maximum(np.abs(x[1]), 1e-9))) for x, y in zip(ra, rb))
        du = max(float(np.max(np.abs(x[0] - y[0]))) for x, y in zip(ra, rb))
        check("40 次闭环计算：每条候选的代价相对差 < 1e-5", dc < 1e-5, dc)
        check("... 输出的 [ax, delta] 差 < 1e-5", du < 1e-5, du)
        check("... 而且它转了向（这段合成运行做了事）", max(abs(x[0][1]) for x, in [(r,) for r in ra]) > 1e-3,
              max(abs(r[0][1]) for r in ra))
        xy_graph = b.graph.positions()
        xy_replay = a._replay()
        dxy = float(np.max(np.abs(xy_graph - xy_replay)))
        check("画线用的候选位置（CUDA graph 留下的 / 逐步再推一遍的）差 < 1e-3 m", xy_graph.shape == (2048, 33, 2) and dxy < 1e-3,
              (xy_graph.shape, dxy))
        times = {}
        for name, c in (("原路径", a), ("CUDA graph", b)):
            t0 = time.perf_counter()
            for i in range(10):
                quiet(c.control, {"Vx": 72.0, "Vy": 0.0, "AVz": 0.0}, i * 0.05, 0.05, scene())
            times[name] = (time.perf_counter() - t0) / 10 * 1000
        check("CUDA graph 版每个周期 < 70 ms，比原路径快一倍以上（一个周期 ms）",
              times["CUDA graph"] < 70.0 and times["CUDA graph"] < 0.5 * times["原路径"], {k: round(v, 1) for k, v in times.items()})
    # ---- CPU
    c, txt = new(graph=True, device="cpu")
    check("DEVICE = cpu: 走 CPU（没有 CUDA graph），说明", c.graph is None and c.device.startswith("CPU") and "推演在 CPU" in txt, txt.strip())
    out, _ = quiet(c.control, {"Vx": 72.0, "Vy": 0.0, "AVz": 0.0}, 0.0, 0.05, scene())
    check("... 输出是有限的 [ax, delta]", len(out) == 2 and all(math.isfinite(x) for x in out), out)
    if cuda:
        r_cpu = closed_loop(c, 10)
        b2, _ = new(graph=True)
        r_gpu = closed_loop(b2, 10)
        d = max(float(np.max(np.abs(x[0] - y[0]))) for x, y in zip(r_cpu, r_gpu))
        check("CPU 和 GPU 的输出一致（10 次，差 < 1e-2：float32 的舍入在闭环里放大得有限）", d < 1e-2, d)

    # ---- 参考（直路）
    c, _ = new(graph=False, device="cpu")
    h = c.ref.horizon(lane(0.0), (-ctl.A_CG, 0.0))
    check("直路上的参考：(T, 6)、y ≈ 0、yaw ≈ 0、vx = 20 m/s、r = 0、vy 是稳态表的值（≈ 0）",
          h.shape == (33, 6) and np.abs(h[:, 1]).max() < 1e-6 and np.abs(h[:, 2]).max() < 1e-6 and np.allclose(h[:, 3], 20.0)
          and np.abs(h[:, 5]).max() < 1e-9 and np.abs(h[:, 4]).max() < 0.05, (h.shape, np.abs(h[:, 4]).max()))
    hc = c.ref.horizon(lane(0.0, 0.01), (-ctl.A_CG, 0.0))
    check("左弯（曲率 0.01）：r = v·κ = 0.2 rad/s、yaw 往左转、vy 是表里的值", abs(hc[10, 5] - 0.2) < 0.01 and hc[-1, 2] > 0.05
          and abs(hc[10, 4] - np.interp(0.01, c.ref._vy_kappa, c.ref._vy_val)) < 1e-3, (hc[10, 5], hc[-1, 2], hc[10, 4]))

    # ---- 出错要说清楚
    old = ctl.MODEL
    try:
        ctl.MODEL, ctl.DEVICE = "models/nope.pt", "cpu"
        c = ctl.Controller()
        try:
            quiet(c.reset)
            msg = ""
        except ValueError as e:
            msg = str(e)
        check("权重不存在：说清楚是哪个文件", "世界模型权重不存在" in msg and "nope.pt" in msg, msg)
    finally:
        ctl.MODEL = old
    c, _ = new(graph=False, device="cpu")
    try:
        quiet(c.control, {"Vx": 72.0, "Vy": 0.0, "AVz": 0.0}, 0.0, 0.03, scene())
        msg = ""
    except ValueError as e:
        msg = str(e)
    check("仿真步长 0.03 s 不能整除 0.05 s：说清楚怎么改", "不能整除" in msg and "0.05" in msg, msg)
    c, _ = new(graph=False, device="cpu")
    out, txt = quiet(c.control, {"Vx": 72.0, "Vy": 0.0, "AVz": 0.0}, 0.0, 0.05, {"units": UNITS, "lane": None})
    out2, txt2 = quiet(c.control, {"Vx": 72.0, "Vy": 0.0, "AVz": 0.0}, 0.05, 0.05, {"units": UNITS, "lane": None})
    check("没有车道信息：ax 归零、说一次", out[0] == 0.0 and "没有车道信息" in txt and txt2 == "", (out, txt.strip(), txt2))
    c, _ = new(graph=False, device="cpu")
    out, txt = quiet(c.control, {"Vx": 20.0, "Vy": 0.0, "AVz": 0.0}, 0.0, 0.05, scene())
    check("车速离参考太远：说一次（世界模型在接近参考车速的数据上训练）", "离参考车速" in txt and "训练" in txt, txt.strip())

    # ---- 附带的每个权重
    models = ["models/world_model.pt", "models/world_model_wide.pt", "models/wm_wide_H2_128.pt", "models/wm_wide_H2_256.pt"]
    models += ["models/peml/" + f for f in sorted(os.listdir(os.path.join(KDIR, "models", "peml"))) if f.endswith(".pt")]
    bad = []
    for m in models:
        try:
            c, _ = new(graph=cuda, device="auto", model=m)
            r = closed_loop(c, 3)
            if not all(np.all(np.isfinite(x[0])) for x in r):
                bad.append((m, "NaN"))
        except Exception as e:  # noqa: BLE001
            bad.append((m, "%s: %s" % (type(e).__name__, e)))
    ctl.MODEL = "models/world_model.pt"
    check("附带的 %d 个权重都能加载、给出有限的输出" % len(models), not bad, bad)

    print("FAILED: %s" % FAILS if FAILS else "ALL KMPPI LEARNED OFFLINE TESTS PASSED")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
