"""KMPPI + 学习世界模型（移植自 ~/Desktop/rl/kmppi_dream）：沿 CARLA 车道中心线恒速行驶。

和 ../kmppi/controller.py 是同一个接到平台上的一层，区别只有一处：预测模型不是 3DOF 物理自行车，
而是你训练的神经网络世界模型（world_model.py：输入最近 H 步的 [vx, vy, r, ax, delta]，输出下一步的速度增量）。
算法本身原样照搬，没有改动：
    kmppi_controller.py   kmppi_dream 版 KMPPI（RBF 核支撑点、导数动作提升、对称采样、自适应温度；
                          dynamics 可以是有状态的学习模型；整段推演和代价留在 torch 设备上）
    world_model.py        WorldModel / LearnedDynamics（只有一行 import 改了：config → dream_config）
    dream_config.py       kmppi_dream/config.py，只是改了名（和平台的 config.py 同名冲突）
    models/               你训练好的权重：world_model.pt（八字入口默认）、world_model_wide.pt（赛道入口默认）、
                          wm_wide_H2_128 / 256、peml/（PEML 训练的动力学：peml_2tr、mse_branches、cc_b1_s100）
换掉的只有参考轨迹：原工程是八字，这里是 CARLA 的车道中心线（lane_reference_wm.py，vy 参考同样查世界模型的稳态表）。

被控对象必须是模型训练时的那台车：PyChrono 宝马 E90（“CarSim 动力学”页的“Chrono 宝马（服务器）”）。世界模型是
从那台车的数据学出来的，换成真实 CarSim 的车就不对了——要用 CarSim 的车，得先用那台车的运行数据重新训练模型。
    状态   原工程的 [X, Y, yaw, vx, vy, r] 是质心的。Chrono 宝马导出的是参考点（前轴中心）的量：
           自车坐标里质心在 (-A_CG, 0)，vy_质心 = Vy - A_CG·r。
    参考   scene["lane"]["center_rel"]（“场景信息”页默认就勾选了给算法）。
    输出   [纵向加速度 ax (m/s²), 前轮转角 delta (rad，左为正)] —— 原工程被控对象的输入。
    历史   世界模型要最近 H-1 步真实的 [vx, vy, r, ax, delta]（run_kmppi.py 的 recent），每个控制周期记一次。
    周期   KMPPI 每 0.05 s 算一次；FRAME_DT 让平台自动把仿真步长设为 0.05 s。
    设备   DEVICE = "auto"：有 NVIDIA 显卡就用（RTX 3090），否则 CPU。
    速度   世界模型的整段推演原来一步发出几十个小 GPU 算子，K = 2048、T = 33 一次要 ~70 ms（一个周期两次）：
           显卡上用 fast_rollout.py 把它录成 CUDA graph（逐行同一个计算，tests/test_offline_kmppi_learned.py 核对代价），
           每次约几 ms，画线要的每步位置也是同一次推演留下的。USE_CUDA_GRAPH = False 或 CPU 时走原来的路径。
    画线   和 ../kmppi/controller.py 一样（候选轨迹、加权平均、最好的一条、参考轨迹）。
    曲线   ESS、温度、最小代价、加权平均代价、ax、前轮转角、计算耗时放在 self.debug 里。
单位跟随“CarSim 动力学”页（scene["units"]）。
"""
import math
import os
import time

import numpy as np
import torch

from dream_config import KMPPIConfig
from fast_rollout import GraphRollout
from kmppi_controller import ReferenceBox, build_kmppi
from lane_reference_wm import LaneReference
from world_model import LearnedDynamics, WorldModel

FRAME_DT = 0.05      # s，这个算法要的仿真步长：平台运行时自动使用（界面上的设置不用改）
MODEL = "models/world_model.pt"   # 世界模型权重（相对这个文件夹；也可以填绝对路径，如 models/peml/peml_2tr.pt）
DEVICE = "auto"      # "auto"：有显卡用显卡；"cpu"；"cuda"
USE_CUDA_GRAPH = True  # 显卡上把整段推演录成 CUDA graph（fast_rollout.py，算的和原函数一样，快很多）；False = 原来的逐步发射
REF_SPEED = 20.0     # m/s，参考车速（原工程 20 m/s）
A_CG = 1.371         # m，Chrono 宝马 E90 的质心到前轴距离（实测，和 bmw_e90_identified 一致）
KAPPA_MAX = 0.04     # 1/m，参考 vy 的稳态表覆盖的最大曲率（转弯半径 25 m；超出的曲率取表的两端）
PRINT_EVERY = 5.0    # s，每隔多久在“输出”页打印一行状态
DRAW_CANDIDATES = 64  # 每次画多少条候选轨迹（0 = 不画）
DRAW_EVERY = 3       # 每隔几步取一个点（T = 33 步 → 12 个点）


def _color(u):
    """0 → 蓝，0.5 → 品红，1 → 红（u 为按权重的相对位置）。"""
    u = min(1.0, max(0.0, u))
    return [int(40 + 215 * u), 60, int(255 - 215 * u)]


class Controller:
    def reset(self):
        cfg = KMPPIConfig()      # T = 33、K = 2048、其余都是 kmppi_dream 的默认值
        cfg.ref_speed = REF_SPEED
        self.cfg = cfg
        here = os.path.dirname(os.path.abspath(__file__))
        path = MODEL if os.path.isabs(MODEL) else os.path.join(here, MODEL)
        if not os.path.isfile(path):
            raise ValueError("世界模型权重不存在：%s（MODEL 填 models/ 下的文件，或绝对路径）" % path)
        if DEVICE == "auto":
            device = "cuda" if torch.cuda.is_available() else "cpu"
        else:
            device = DEVICE
        self.wm = wm = WorldModel.load(path, device=device)
        if not np.isclose(wm.dt, cfg.dt):
            raise ValueError("世界模型的 dt = %g s，和控制周期 %g s 不一致" % (wm.dt, cfg.dt))
        # 参考 vy：世界模型的稳态表（run_kmppi.py 的 build_reference）
        kap = np.linspace(-KAPPA_MAX, KAPPA_MAX, 41)
        _, vy_ss, _ = wm.steady_state_vy_table(kap, cfg.ref_speed, cfg.delta_max, cfg.ax_max)
        self.ref = LaneReference(cfg, kap, vy_ss)
        self.dyn = LearnedDynamics(wm, cfg.K)
        self.ref_box = ReferenceBox(cfg.T)
        # dyn.step 直接交给 KMPPI（不包一层）：它靠 dynamics_fn.__self__ 认出学习模型，走留在设备上的整段推演
        self.ctrl = build_kmppi(cfg, self.dyn.step, self.ref_box, np.random.default_rng(cfg.rng_seed))
        self._last = None
        self.graph = None
        if wm.device.type == "cuda" and USE_CUDA_GRAPH:
            self.graph = GraphRollout(self.dyn, self.ctrl, self.ref_box, cfg)
            self.ctrl._rollout_cost = self.graph
        else:
            orig = self.ctrl._rollout_cost

            def spy(seq):   # 记下最后一轮的候选动作序列和起点（画线用）；代价照旧由原函数算
                self._last = (seq, self.ctrl.current_state.copy(), self.ctrl.applied_action.copy())
                return orig(seq)
            self.ctrl._rollout_cost = spy
        self.device = "%s（%s%s）" % ("GPU" if wm.device.type == "cuda" else "CPU", wm.device,
                                     "，CUDA graph" if self.graph is not None else "")
        print("KMPPI（学习世界模型）：%s，H = %d、隐层 %d × %d，K = %d 条候选、T = %d 步，推演在 %s 上" % (
            os.path.relpath(path, here) if path.startswith(here) else path, wm.H, wm.cfg.hidden, wm.cfg.layers,
            cfg.K, cfg.T, self.device))
        self.pick_rng = np.random.default_rng(1)   # 挑画哪几条（和算法的随机数分开，不影响结果）
        self.draw = None
        self.debug = None
        self.every = None           # 每隔几帧算一次（第一帧时按帧步长定）
        self.frame = 0
        self.action = [0.0, 0.0]
        self.recent = []            # 最近的真实 [vx, vy, r, ax, delta]（世界模型的历史窗口）
        self.told_no_lane = False
        self.told_slow = False
        self.next_print = 0.0
        self.n_calls = 0
        self.compute_s = 0.0
        self.compute_max = 0.0
        self.offsets = []
        self.speeds = []
        self.ess = []

    def _units(self, scene):
        u = scene.get("units") or {}
        deg = 1.0 if u.get("angle", "deg") == "deg" else 180.0 / math.pi   # 导出的角度 → deg
        kmh = 1.0 if u.get("speed", "km/h") == "km/h" else 3.6              # 导出的车速 → km/h
        dps = 1.0 if u.get("rate", "deg/s") == "deg/s" else 180.0 / math.pi  # 导出的角速度 → deg/s
        return kmh, dps

    def control(self, exports, t, dt, scene):
        if self.every is None:
            n = int(round(self.cfg.dt / dt))
            if n < 1 or abs(n * dt - self.cfg.dt) > 1e-6:
                raise ValueError("KMPPI 的控制周期是 %.2f s，仿真步长 %.3f s 不能整除它：请在“联合仿真”页的"
                                 "“运行设置”把仿真步长设为 0.05 s（或 0.025、0.01 s）" % (self.cfg.dt, dt))
            self.every = n
        k = self.frame
        self.frame += 1
        if k % self.every:
            return list(self.action)   # 两次计算之间 ax、delta 保持（零阶保持）
        kmh, dps = self._units(scene)
        vx = exports["Vx"] * kmh / 3.6
        r = math.radians(exports["AVz"] * dps)
        vy = exports["Vy"] * kmh / 3.6 - A_CG * r   # 参考点（前轴）的侧向速度 → 质心的
        lane = scene.get("lane")
        pts = (lane or {}).get("center_rel") or []
        if len(pts) < 2:
            if not self.told_no_lane:
                print("t = %.1f s：没有车道信息（不在行车道上）：前轮转角保持、ax 归零（保持车速，不再加速）" % t)
                self.told_no_lane = True
            self.action = [0.0, self.action[1]]
            return list(self.action)
        self.told_no_lane = False

        if not self.told_slow and vx < 0.5 * REF_SPEED:
            print("t = %.1f s：车速 %.1f m/s 离参考车速 %.0f m/s 太远。世界模型是在接近参考车速的数据上训练的，"
                  "低速时预测不可信、会乱打方向：“CarSim 动力学”页把初始车速设为 %.0f km/h" % (
                      t, vx, REF_SPEED, REF_SPEED * 3.6))
            self.told_slow = True
        state = np.array([-A_CG, 0.0, 0.0, vx, vy, r])
        t0 = time.perf_counter()
        self.ref_box.t_now = t
        self.ref_box.val = self.ref.horizon(pts, (-A_CG, 0.0))
        H = self.wm.H
        if H > 1:
            hist = np.array(self.recent[-(H - 1):]) if self.recent else np.array([[*state[3:6], 0.0, 0.0]])
            self.dyn.set_history(hist)
        for i in range(self.cfg.num_refinement_steps):
            last = i == self.cfg.num_refinement_steps - 1
            act = self.ctrl.command(state, shift_horizon=(i == 0), commit_action=last)
        self.recent.append([*state[3:6], float(act[0]), float(act[1])])
        spent = time.perf_counter() - t0
        self.debug = {"ESS": self.ctrl.last_effective_sample_size, "温度": self.ctrl.last_temperature,
                      "最小代价": self.ctrl.last_min_cost, "加权平均代价": self.ctrl.last_mean_cost,
                      "ax (m/s²)": float(act[0]), "前轮转角 (rad)": float(act[1]), "计算耗时 (ms)": spent * 1000}
        if DRAW_CANDIDATES > 0 and (self.graph is not None or self._last is not None):
            self.draw = self._lines(state)
        self.action = [float(act[0]), float(act[1])]

        self.n_calls += 1
        self.compute_s += spent
        self.compute_max = max(self.compute_max, spent)
        off = lane.get("offset")
        if off is not None:
            self.offsets.append(off)
        self.speeds.append(vx)
        self.ess.append(self.ctrl.last_effective_sample_size)
        if t >= self.next_print:
            self.next_print = t + PRINT_EVERY
            print("t = %.1f s  车速 %.2f m/s（参考 %.1f）  横向偏差 %s  ax %+.2f m/s²  前轮转角 %+.4f rad  ESS %.1f  计算 %.0f ms" % (
                t, vx, REF_SPEED, "%.3f m" % off if off is not None else "—", self.action[0], self.action[1],
                self.ctrl.last_effective_sample_size, spent * 1000))
        return list(self.action)

    def _replay(self):
        """最后一轮 K 条候选每步的位置 (K, T, 2)。CUDA graph 版推演时已经留下；否则把最后一轮的候选动作序列
        在世界模型里再推一遍——和 LearnedDynamics.rollout_cost_torch 的推演同一个循环（提升 + 一步预测），只是把位置留下来。"""
        if self.graph is not None:
            return self.graph.positions()
        seq, s0, a0 = self._last
        dyn, cfg = self.dyn, self.cfg
        dev = dyn.device
        K, T = seq.shape[0], seq.shape[1]
        U = torch.from_numpy(np.asarray(seq, dtype=np.float32)).to(dev)
        st = torch.from_numpy(np.asarray(s0, dtype=np.float32)).to(dev).reshape(1, -1).repeat(K, 1)
        act = torch.from_numpy(np.asarray(a0, dtype=np.float32)).to(dev).reshape(1, -1).repeat(K, 1)
        a_min = torch.from_numpy(np.asarray(self.ctrl.actual_action_min, dtype=np.float32)).to(dev)
        a_max = torch.from_numpy(np.asarray(self.ctrl.actual_action_max, dtype=np.float32)).to(dev)
        dyn.begin_rollout(K)
        xy = []
        with torch.no_grad():
            for i in range(T):
                act = dyn._lift(act, U[:, i, :], st, cfg.dt, a_min, a_max, self.ctrl.steer_scale)
                st = dyn._step_torch(st, act)
                xy.append(st[:, :2])
        return torch.stack(xy, dim=1).cpu().numpy().astype(np.float64)

    def _lines(self, state):
        """这一次的候选轨迹、加权平均轨迹和参考轨迹，画线格式（自车坐标，m）。"""
        xy = self._replay()                                   # (K, T, 2)
        w = np.asarray(self.ctrl.last_weights, dtype=float)
        start = np.repeat(state[None, None, :2], xy.shape[0], axis=0)
        full = np.concatenate([start, xy], axis=1)
        xy = full[:, ::DRAW_EVERY]
        if (self.cfg.T % DRAW_EVERY) != 0:
            xy = np.concatenate([xy, full[:, -1:]], axis=1)
        n = min(DRAW_CANDIDATES, len(w))
        order = np.argsort(-w)
        top = order[: n // 2][::-1]                         # 权重从低到高：最高的最后画（在上层）
        rest = self.pick_rng.choice(order[n // 2:], n - len(top), replace=False)
        pick = np.concatenate([rest, top])
        lw = np.log(np.maximum(w[pick], 1e-300))
        lo, hi = float(lw.min()), float(lw.max())
        cols = [_color((lw[j] - lo) / (hi - lo) if hi > lo else 1.0) for j in range(len(pick))]
        lines = [{"points": xy[k].tolist(), "color": cols[j], "width": 0.015} for j, k in enumerate(pick)]
        lines += [{"points": [xy[k][-1].tolist()], "color": cols[j], "width": 0.12} for j, k in enumerate(pick)]  # 末端的点
        ref = np.vstack([state[None, :2], self.ref_box.val[:, :2]])
        lines.append({"points": ref.tolist(), "color": [40, 230, 90], "width": 0.06})
        mean = np.einsum("k,ktd->td", w, xy)
        lines.append({"points": mean.tolist(), "color": [255, 215, 0], "width": 0.09})
        best = int(order[0])                                # 代价最低、权重最高的一条：加粗，画在最上层
        lines.append({"points": xy[best].tolist(), "color": [255, 30, 30], "width": 0.12})
        return lines

    def finish(self, reason):
        if not self.n_calls:
            return
        o = np.abs(np.asarray(self.offsets)) if self.offsets else np.zeros(1)
        v = np.asarray(self.speeds)
        print("KMPPI（学习世界模型）结束（%s）：车道中心偏差 均方根 %.3f m、最大 %.3f m；车速 %.2f ~ %.2f m/s；平均 ESS %.1f；"
              "每次计算（%s）平均 %.0f ms、最长 %.0f ms（%d 次）" % (
                  reason, math.sqrt(float(np.mean(o ** 2))), float(np.max(o)), float(v.min()), float(v.max()),
                  float(np.nanmean(self.ess)), self.device, self.compute_s / self.n_calls * 1000, self.compute_max * 1000, self.n_calls))
