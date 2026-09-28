"""KMPPI 路径跟踪（移植自 ~/Desktop/rl/kmppi，Python 版本 ~/Desktop/rl/kmppi_chrono）：沿 CARLA 车道中心线恒速行驶。

算法本身原样照搬，没有改动：
    kmppi_controller.py   KMPPIController.m / make_kmppi_14dof_controller.m / kmppi_14dof_cost.m：
                          RBF 核参数化（P = 9 个支撑点，T = 33 步 = 1.65 s）、导数动作提升 [jerk, 转向角速度]、
                          对称采样 K = 2048、自适应温度（ESS ≈ 16）、每周期 2 次 refinement
    prediction_model.py   kmppi_14dof_prediction_dynamics.m：3DOF 动态自行车 + Pacejka 侧向轮胎
    kmppi_config.py       kmppi_14dof_config.m 等：全部参数，车辆参数取 Chrono 宝马 E90（bmw_e90_identified.json）
只有参考轨迹换了：原工程是八字，这里是 CARLA 的车道中心线（lane_reference.py）。

这个文件是接到仿真平台上的一层：
    状态   原工程的 [X, Y, yaw, vx, vy, r] 是质心的。CarSim（和替身 Chrono 宝马）导出的是参考点
           （前轴中心）的量：在自车坐标里质心在 (-a, 0)，vy_质心 = Vy - a·r（a 为质心到前轴距离）。
           状态和参考都在当前的自车坐标里（每个周期重新取），yaw = 0。
    参考   scene["lane"]["center_rel"]（“场景信息”页默认就勾选了给算法）。
    输出   [纵向加速度 ax (m/s²), 前轮转角 delta (rad，左为正)] —— 原工程被控对象的输入，
           由 Chrono 宝马替身里原样的命令适配层变成四轮扭矩和转向输入（chrono_bmw/chrono_plant.py）。
    周期   KMPPI 每 0.05 s 算一次（原工程的控制周期），中间各帧保持上一次的输出（零阶保持）。
           仿真步长要能整除 0.05 s（0.05、0.025、0.01 s ...；推荐 0.05 s：每帧算一次）。
    画线   每次计算后把候选轨迹画进 CARLA 画面（self.draw，平台每帧画出来；采集数据时不画）：
           从 K 条推演里挑 DRAW_CANDIDATES 条（权重最高的一半 + 其余随机一半），颜色按权重
           从蓝（低）到红（高），末端各有一个同色的点（看得出末端怎么分布）；最好的一条（代价最低、权重最高）加粗画成亮红色；黄色粗线是按权重
           平均的轨迹（实际执行的就是它的第一步）；绿色是参考轨迹。都是质心的轨迹。
           推演的状态由包在预测模型外面的一层记下来（_Recorder），算法本身不动。
单位跟随“CarSim 动力学”页（scene["units"]）。
"""
import math
import time

import numpy as np

from kmppi_config import KMPPIConfig
from kmppi_controller import ReferenceBox, build_kmppi
from lane_reference import LaneReference
from prediction_model import BicycleModel

REF_SPEED = 20.0     # m/s，参考车速（原工程 20 m/s）
PRINT_EVERY = 5.0    # s，每隔多久在“输出”页打印一行状态
DRAW_CANDIDATES = 64  # 每次画多少条候选轨迹（0 = 不画）
DRAW_EVERY = 3       # 每隔几步取一个点（T = 33 步 → 12 个点）


class _Recorder:
    """包在预测模型 step 外面：on 时记下每一步 K 条推演的位置 (K, 2)。"""

    def __init__(self, step):
        self.step, self.on, self.xy = step, False, []

    def __call__(self, state, action):
        nxt = self.step(state, action)
        if self.on:
            self.xy.append(nxt[:, :2].copy())
        return nxt


def _color(u):
    """0 → 蓝，0.5 → 品红，1 → 红（u 为按权重的相对位置）。"""
    u = min(1.0, max(0.0, u))
    return [int(40 + 215 * u), 60, int(255 - 215 * u)]


class Controller:
    def reset(self):
        cfg = KMPPIConfig(plant="chrono", vehicle_params="chrono_bmw_e90")   # T = 33（原工程 14DOF / Chrono 用的时域）
        cfg.ref_speed = REF_SPEED
        self.cfg = cfg
        self.vehicle = cfg.resolved_vehicle()
        model = BicycleModel(self.vehicle, cfg.dt, cfg.ax_max, cfg.delta_max)
        self.ref_box = ReferenceBox(cfg.T)
        self.rec = _Recorder(model.step)
        self.ctrl = build_kmppi(cfg, self.rec, self.ref_box, np.random.default_rng(cfg.rng_seed))
        self.pick_rng = np.random.default_rng(1)   # 挑画哪几条（和算法的随机数分开，不影响结果）
        self.draw = None
        self.ref = LaneReference(cfg, self.vehicle)
        self.every = None           # 每隔几帧算一次（第一帧时按帧步长定）
        self.frame = 0
        self.action = [0.0, 0.0]
        self.told_no_lane = False
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
        return deg, kmh, dps

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
            return list(self.action)            # 两次计算之间保持（零阶保持）

        _, kmh, dps = self._units(scene)
        a = self.vehicle.a
        vx = exports["Vx"] * kmh / 3.6
        r = math.radians(exports["AVz"] * dps)
        vy = exports["Vy"] * kmh / 3.6 - a * r   # 参考点（前轴）的侧向速度 → 质心的
        lane = scene.get("lane")
        pts = (lane or {}).get("center_rel") or []
        if len(pts) < 2:
            if not self.told_no_lane:
                print("t = %.1f s：没有车道信息（不在行车道上），保持上一次的输出" % t)
                self.told_no_lane = True
            return list(self.action)
        self.told_no_lane = False

        state = np.array([-a, 0.0, 0.0, vx, vy, r])
        t0 = time.perf_counter()
        self.ref_box.t_now = t
        self.ref_box.val = self.ref.horizon(pts, (-a, 0.0))
        for i in range(self.cfg.num_refinement_steps):
            last = i == self.cfg.num_refinement_steps - 1
            self.rec.on, self.rec.xy = last and DRAW_CANDIDATES > 0, []
            act = self.ctrl.command(state, shift_horizon=(i == 0), commit_action=last)
        self.rec.on = False
        spent = time.perf_counter() - t0
        if DRAW_CANDIDATES > 0 and self.rec.xy:
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

    def _lines(self, state):
        """这一次的候选轨迹、加权平均轨迹和参考轨迹，画线格式（自车坐标，m）。"""
        xy = np.stack(self.rec.xy, axis=1)                    # (K, T, 2)
        w = np.asarray(self.ctrl.last_weights, dtype=float)
        start = np.repeat(state[None, None, :2], xy.shape[0], axis=0)
        xy = np.concatenate([start, xy], axis=1)[:, ::DRAW_EVERY]
        if (self.cfg.T % DRAW_EVERY) != 0:
            xy = np.concatenate([xy, np.concatenate([start, np.stack(self.rec.xy, axis=1)], axis=1)[:, -1:]], axis=1)
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
        print("KMPPI 结束（%s）：车道中心偏差 均方根 %.3f m、最大 %.3f m；车速 %.2f ~ %.2f m/s；平均 ESS %.1f；"
              "每次计算平均 %.0f ms、最长 %.0f ms（%d 次）" % (
                  reason, math.sqrt(float(np.mean(o ** 2))), float(np.max(o)), float(v.min()), float(v.max()),
                  float(np.nanmean(self.ess)), self.compute_s / self.n_calls * 1000, self.compute_max * 1000, self.n_calls))
