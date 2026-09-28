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


class Controller:
    def reset(self):
        cfg = KMPPIConfig(plant="chrono", vehicle_params="chrono_bmw_e90")   # T = 33（原工程 14DOF / Chrono 用的时域）
        cfg.ref_speed = REF_SPEED
        self.cfg = cfg
        self.vehicle = cfg.resolved_vehicle()
        model = BicycleModel(self.vehicle, cfg.dt, cfg.ax_max, cfg.delta_max)
        self.ref_box = ReferenceBox(cfg.T)
        self.ctrl = build_kmppi(cfg, model.step, self.ref_box, np.random.default_rng(cfg.rng_seed))
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
            act = self.ctrl.command(state, shift_horizon=(i == 0),
                                    commit_action=(i == self.cfg.num_refinement_steps - 1))
        spent = time.perf_counter() - t0
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

    def finish(self, reason):
        if not self.n_calls:
            return
        o = np.abs(np.asarray(self.offsets)) if self.offsets else np.zeros(1)
        v = np.asarray(self.speeds)
        print("KMPPI 结束（%s）：车道中心偏差 均方根 %.3f m、最大 %.3f m；车速 %.2f ~ %.2f m/s；平均 ESS %.1f；"
              "每次计算平均 %.0f ms、最长 %.0f ms（%d 次）" % (
                  reason, math.sqrt(float(np.mean(o ** 2))), float(np.max(o)), float(v.min()), float(v.max()),
                  float(np.nanmean(self.ess)), self.compute_s / self.n_calls * 1000, self.compute_max * 1000, self.n_calls))
