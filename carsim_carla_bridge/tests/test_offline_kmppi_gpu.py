"""不需要 CARLA：KMPPI 的 GPU 推演（controllers/kmppi/kmppi_gpu.py）和 CPU 版一致。

同一个状态序列上，CPU 版和 GPU 版各做 40 次闭环计算（每次 2 次 refinement，同样的随机数）：
每条候选的代价相对差 < 1e-10、输出的 [ax, delta] 差 < 1e-10；记下的候选轨迹（画线用）差 < 1e-9 m；
controller.py 用 GPU、USE_GPU = False 时用 CPU、没有 CuPy 时退回 CPU 并说明；GPU 版比 CPU 快。
没有 NVIDIA 显卡或 CuPy 的机器上只检查退回 CPU 的部分。

    python tests/test_offline_kmppi_gpu.py
"""
import contextlib
import io
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
KDIR = os.path.join(HERE, "..", "controllers", "kmppi")
sys.path.insert(0, KDIR)

import kmppi_gpu  # noqa: E402
from kmppi_config import KMPPIConfig  # noqa: E402
from kmppi_controller import ReferenceBox, build_kmppi  # noqa: E402
from prediction_model import BicycleModel  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + (("  " + str(detail)) if detail != "" else ""), flush=True)
    if not cond:
        FAILS.append(name)


class Rec:
    def __init__(self, step):
        self.step, self.on, self.xy = step, False, []

    def __call__(self, state, action):
        nxt = self.step(state, action)
        if self.on:
            self.xy.append(nxt[:, :2].copy())
        return nxt


def make(gpu):
    cfg = KMPPIConfig(plant="chrono", vehicle_params="chrono_bmw_e90")
    v = cfg.resolved_vehicle()
    m = BicycleModel(v, cfg.dt, cfg.ax_max, cfg.delta_max)
    box = ReferenceBox(cfg.T)
    rec = Rec(m.step)
    c = build_kmppi(cfg, rec, box, np.random.default_rng(cfg.rng_seed))
    if gpu:
        c._rollout_cost = kmppi_gpu.GPURollout(c, m, cfg, box, rec)
    return cfg, m, box, rec, c


def closed_loop(gpu, n=40):
    cfg, m, box, rec, c = make(gpu)
    s = np.array([-1.4, 0.4, 0.03, 19.0, 0.1, 0.02])
    out = []
    for i in range(n):
        tt = (np.arange(cfg.T) + 1) * cfg.dt + i * cfg.dt
        box.val = np.stack([s[0] + 20.0 * (tt - i * cfg.dt), 0.5 * np.sin(0.4 * tt), 0.2 * np.cos(0.4 * tt),
                            np.full_like(tt, 20.0), np.zeros_like(tt), 0.08 * np.cos(0.4 * tt)], axis=1)
        for k in range(cfg.num_refinement_steps):
            last = k == cfg.num_refinement_steps - 1
            rec.on, rec.xy = last, []
            u = c.command(s, shift_horizon=(k == 0), commit_action=last)
        out.append((u.copy(), c.last_total_cost.copy(), np.stack(rec.xy, axis=1)))
        s = m.step(s[None], u[None])[0]
    return out, c


def main():
    ok, why = kmppi_gpu.available()
    print("INFO GPU:", ok, why)
    sys.path.insert(0, KDIR)
    import controller as ctl
    if ok:
        a, ca = closed_loop(False)
        b, cb = closed_loop(True)
        dc = max(float(np.max(np.abs(x[1] - y[1]) / np.maximum(np.abs(x[1]), 1e-12))) for x, y in zip(a, b))
        du = max(float(np.max(np.abs(x[0] - y[0]))) for x, y in zip(a, b))
        dxy = max(float(np.max(np.abs(x[2] - y[2]))) for x, y in zip(a, b))
        check("40 closed-loop commands: every candidate's cost the same (relative < 1e-10)", dc < 1e-10, dc)
        check("... the [ax, delta] output the same (< 1e-10)", du < 1e-10, du)
        check("... the recorded candidate positions (drawing) the same (< 1e-9 m)", dxy < 1e-9, dxy)
        check("... and it steered (a run that did something)", max(abs(x[0][1]) for x in a) > 1e-3,
              max(abs(x[0][1]) for x in a))
        s = np.array([-1.4, 0.3, 0.05, 19.0, 0.1, 0.02])
        times = {}
        for name, c in (("cpu", ca), ("gpu", cb)):
            t0 = time.perf_counter()
            for _ in range(10):
                c.command(s)
            times[name] = (time.perf_counter() - t0) / 10 * 1000
        check("GPU faster than CPU (ms per command)", times["gpu"] < 0.6 * times["cpu"], {k: round(v, 1) for k, v in times.items()})
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            c = ctl.Controller()
            c.reset()
        check("controller.py: the rollout on the GPU, said", c.device.startswith("GPU") and "推演在 GPU" in buf.getvalue(),
              buf.getvalue().strip())
    # USE_GPU = False: CPU.
    ctl.USE_GPU = False
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        c = ctl.Controller()
        c.reset()
    check("USE_GPU = False: CPU", c.device == "CPU" and not isinstance(c.ctrl.__dict__.get("_rollout_cost"), kmppi_gpu.GPURollout),
          buf.getvalue().strip())
    # No CuPy: back to the CPU, the reason said.
    ctl.USE_GPU = True
    saved = (kmppi_gpu._cp, kmppi_gpu._why)
    kmppi_gpu._cp, kmppi_gpu._why = None, "ModuleNotFoundError: No module named 'cupy'"
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            c = ctl.Controller()
            c.reset()
    finally:
        kmppi_gpu._cp, kmppi_gpu._why = saved
    check("no CuPy: the CPU, the reason said", c.device == "CPU" and "GPU 不可用" in buf.getvalue() and "cupy" in buf.getvalue(),
          buf.getvalue().strip()[:160])
    print("FAILED: %s" % FAILS if FAILS else "ALL KMPPI GPU TESTS PASSED")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
