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
    输出   OUTPUT = "ax_delta"：[纵向加速度 ax (m/s²), 前轮转角 delta (rad，左为正)] —— 原工程被控对象的输入，
           由 Chrono 宝马替身里原样的命令适配层变成四轮扭矩和转向输入（chrono_bmw/chrono_plant.py）。
           OUTPUT = "carsim"：真实 CarSim 的导入 [油门 0~1, 制动（主缸压力 MPa）, 方向盘转角 deg]，
           换算层和原工程 14DOF 的适配层同一思路（加速度 → 驱动 / 制动，前轮转角 → 方向盘角）：
             转向  方向盘转角 = 前轮转角 × 传动比；传动比从 STEER_RATIO_INIT 起，运行中用 CarSim 导出的
                   Steer_SW ÷ 前轮平均转角（Steer_L1、Steer_R1）自动修正（和 path_follower.py 一样）。
             纵向  KMPPI 的 ax 积分成目标车速，PI（按 km/h 调）加 ax 前馈 → 油门或制动；制动满量程 BRAKE_MAX。
           每帧都换算（KMPPI 两次计算之间 ax、delta 保持，油门、制动随实际车速更新）。
           注意预测模型的车辆参数还是 Chrono 宝马 E90 的（kmppi_config.py / bmw_e90_identified.json），
           换成你的 CarSim 车时要按那台车重新标定（质量、惯量、前后轴距、轮胎比例因子）。
    周期   KMPPI 每 0.05 s 算一次（原工程的控制周期），中间各帧保持上一次的输出（零阶保持）。
           下面的 FRAME_DT 让平台自动把仿真步长设为 0.05 s（每帧算一次），不用在界面上改；
           去掉它时，仿真步长要能整除 0.05 s（0.05、0.025、0.01 s ...）。
    画线   每次计算后把候选轨迹画进 CARLA 画面（self.draw，平台每帧画出来；采集数据时不画）：
           从 K 条推演里挑 DRAW_CANDIDATES 条（权重最高的一半 + 其余随机一半），颜色按权重
           从蓝（低）到红（高），末端各有一个同色的点（看得出末端怎么分布）；最好的一条（代价最低、权重最高）加粗画成亮红色；黄色粗线是按权重
           平均的轨迹（实际执行的就是它的第一步）；绿色是参考轨迹。都是质心的轨迹。
           推演的状态由包在预测模型外面的一层记下来（_Recorder），算法本身不动。
    曲线   ESS、温度、最小代价、加权平均代价、ax、前轮转角、计算耗时放在 self.debug 里：
           界面“曲线”面板第二排实时显示，运行记录里是 log_debug.csv。
    车辆   预测模型的车辆参数默认是 Chrono 宝马 E90 的（bmw_e90_identified.json）。换成 CarSim 的车时，先用这台车
           跑一次有弯道的运行，在“运行对比”页的“车辆参数辨识”里填质量、横摆惯量、质心到前 / 后轴距离，点“辨识”，
           再点“保存并用于 KMPPI”：写出 vehicle_identified.json，并把下面的 VEHICLE_JSON 设成它。
    GPU    USE_GPU = True 时 K 条候选的推演和代价在 NVIDIA 显卡上算（kmppi_gpu.py，要装 CuPy：
           pip install cupy-cuda12x）：RTX 3090 上每次计算约 11 ms，CPU 约 55 ms（每个控制周期算 2 次，
           CPU 版比 0.05 s 的周期慢，仿真跟不上实时）。结果和 CPU 版一致到舍入误差（代价相对差 ~1e-13，
           tests/test_offline_kmppi_gpu.py 核对）。没有显卡或 CuPy 时自动用 CPU，并在“输出”页说明原因。
单位跟随“CarSim 动力学”页（scene["units"]）。
"""
import json
import math
import os
import time

import numpy as np

from kmppi_config import KMPPIConfig
from kmppi_controller import ReferenceBox, build_kmppi
from lane_reference import LaneReference
from prediction_model import BicycleModel

FRAME_DT = 0.05      # s，这个算法要的仿真步长：平台运行时自动使用（界面上的设置不用改）
OUTPUT = "ax_delta"  # "ax_delta"：Chrono 宝马（[ax, 前轮转角]）；"carsim"：真实 CarSim（[油门, 制动, 方向盘转角]）
# OUTPUT = "carsim" 时的换算（已按当前的 CarSim 车辆设好，换车时改）：
STEER_RATIO_INIT = 19.0   # 方向盘 / 前轮 传动比初值（运行中用导出变量自动修正）
BRAKE_MAX = 8.0           # 导入变量 2 满量程：主缸压力 MPa（制动踏板 0~1 时改成 1）
ACCEL_FF = 0.1            # 每 m/s² 的加速度前馈（油门为正，制动为负，按车调）
KP, KI = 0.08, 0.02       # 车速 PI（按 km/h 调的，和 path_follower.py 一样）
V_TARGET_SLACK = 2.0      # m/s，目标车速最多比实际车速快 / 慢这么多（执行器饱和时不越积越多）
REF_SPEED = 20.0     # m/s，参考车速（原工程 20 m/s）
PRINT_EVERY = 5.0    # s，每隔多久在“输出”页打印一行状态
DRAW_CANDIDATES = 64  # 每次画多少条候选轨迹（0 = 不画）
DRAW_EVERY = 3       # 每隔几步取一个点（T = 33 步 → 12 个点）
USE_GPU = True       # 推演放在 NVIDIA 显卡上（没有显卡或 CuPy 时自动用 CPU）
VEHICLE_JSON = ""    # 车辆参数文件（“运行对比”页辨识出来的；相对路径以这个文件夹为准）；空 = Chrono 宝马 E90


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
        if VEHICLE_JSON:
            here = os.path.dirname(os.path.abspath(__file__))
            path = VEHICLE_JSON if os.path.isabs(VEHICLE_JSON) else os.path.join(here, VEHICLE_JSON)
            with open(path, encoding="utf-8") as f:
                ident = json.load(f)
            for key in ("m", "I", "a", "b", "front_lateral_scale", "rear_lateral_scale", "pac_By", "pac_Cy", "pac_Ey"):
                if key in ident:
                    setattr(self.vehicle, key, float(ident[key]))
            v = self.vehicle
            print("KMPPI：车辆参数用 %s：m %.0f kg、I %.0f kg·m²、a %.3f m、b %.3f m、轮胎比例因子 前 %.3f 后 %.3f" % (
                path, v.m, v.I, v.a, v.b, v.front_lateral_scale, v.rear_lateral_scale))
        model = BicycleModel(self.vehicle, cfg.dt, cfg.ax_max, cfg.delta_max)
        self.ref_box = ReferenceBox(cfg.T)
        self.rec = _Recorder(model.step)
        self.ctrl = build_kmppi(cfg, self.rec, self.ref_box, np.random.default_rng(cfg.rng_seed))
        self.device = "CPU"
        if USE_GPU:
            import kmppi_gpu
            ok, why = kmppi_gpu.available()
            if ok:
                self.ctrl._rollout_cost = kmppi_gpu.GPURollout(self.ctrl, model, cfg, self.ref_box, self.rec)
                self.device = "GPU（%s）" % why
            else:
                print("KMPPI：GPU 不可用（%s），推演用 CPU：每个周期约要算 0.1 s，超过 %.2f s 的控制周期，"
                      "仿真会比实时慢（结果不变）" % (why, cfg.dt))
        print("KMPPI：K = %d 条候选、T = %d 步，推演在 %s 上" % (cfg.K, cfg.T, self.device))
        self.pick_rng = np.random.default_rng(1)   # 挑画哪几条（和算法的随机数分开，不影响结果）
        self.draw = None
        self.debug = None
        self.ref = LaneReference(cfg, self.vehicle)
        self.every = None           # 每隔几帧算一次（第一帧时按帧步长定）
        self.frame = 0
        self.action = [0.0, 0.0]
        self.told_no_lane = False
        self.told_slow = False
        self.next_print = 0.0
        self.n_calls = 0
        self.compute_s = 0.0
        self.compute_max = 0.0
        self.offsets = []
        self.speeds = []
        self.ess = []
        self.ratio = STEER_RATIO_INIT   # OUTPUT = "carsim"
        self.v_target = None
        self.integral = 0.0

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
        deg, kmh, dps = self._units(scene)
        if k % self.every:
            return self._output(exports, dt, deg, kmh)   # 两次计算之间 ax、delta 保持（零阶保持）

        a = self.vehicle.a
        vx = exports["Vx"] * kmh / 3.6
        r = math.radians(exports["AVz"] * dps)
        vy = exports["Vy"] * kmh / 3.6 - a * r   # 参考点（前轴）的侧向速度 → 质心的
        lane = scene.get("lane")
        pts = (lane or {}).get("center_rel") or []
        if len(pts) < 2:
            if not self.told_no_lane:
                print("t = %.1f s：没有车道信息（不在行车道上）：前轮转角保持、ax 归零（保持车速，不再加速）" % t)
                self.told_no_lane = True
            self.action = [0.0, self.action[1]]
            return self._output(exports, dt, deg, kmh)
        self.told_no_lane = False

        if not self.told_slow and vx < 0.5 * REF_SPEED:
            # Like the original project (it starts at the reference speed): the prediction model's slip
            # angles are meaningless near standstill.
            print("t = %.1f s：车速 %.1f m/s 离参考车速 %.0f m/s 太远。KMPPI（和原工程一样）要从接近参考车速起步，"
                  "低速时预测模型的侧偏角不可信、会乱打方向：CarSim 的 .sim 里把初始车速设为 %.0f km/h"
                  "（模拟 CarSim / Chrono 宝马：“CarSim 动力学”页的初始车速）" % (t, vx, REF_SPEED, REF_SPEED * 3.6))
            self.told_slow = True
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
        # The diagnostics as curves (界面“曲线”, log_debug.csv): as run_kmppi.py's diag columns.
        self.debug = {"ESS": self.ctrl.last_effective_sample_size, "温度": self.ctrl.last_temperature,
                      "最小代价": self.ctrl.last_min_cost, "加权平均代价": self.ctrl.last_mean_cost,
                      "ax (m/s²)": float(act[0]), "前轮转角 (rad)": float(act[1]), "计算耗时 (ms)": spent * 1000}
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
        return self._output(exports, dt, deg, kmh)

    def _output(self, exports, dt, deg, kmh):
        """这一帧的输出：OUTPUT = "ax_delta" 时就是 [ax, delta]；"carsim" 时换算成 [油门, 制动, 方向盘转角 deg]。"""
        if OUTPUT != "carsim":
            return list(self.action)
        ax, delta = self.action
        # 传动比：方向盘转角 ÷ 前轮平均转角，只在前轮转角够大、同号时更新（一阶滤波）
        try:
            sw = exports["Steer_SW"] * deg
            wheel = 0.5 * (exports["Steer_L1"] + exports["Steer_R1"]) * deg
            if abs(wheel) > 0.5 and sw * wheel > 0 and 5.0 < sw / wheel < 40.0:
                self.ratio += 0.02 * (sw / wheel - self.ratio)
        except KeyError:
            pass
        # 纵向：ax 积分成目标车速，PI + 前馈；执行器饱和时不再累积积分
        v = exports["Vx"] * kmh / 3.6
        if self.v_target is None:
            self.v_target = v
        self.v_target = min(v + V_TARGET_SLACK, max(v - V_TARGET_SLACK, self.v_target + ax * dt))
        err = (self.v_target - v) * 3.6
        u = ACCEL_FF * ax + KP * err + KI * self.integral
        if 0.0 < u < 1.0 or (u >= 1.0 and err < 0) or (u <= 0.0 and err > 0):
            self.integral = max(-100.0, min(100.0, self.integral + err * dt))
        throttle = max(0.0, min(1.0, u))
        brake = max(0.0, min(1.0, -u)) * BRAKE_MAX
        return [throttle, brake, math.degrees(delta) * self.ratio]

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
              "每次计算（%s）平均 %.0f ms、最长 %.0f ms（%d 次）" % (
                  reason, math.sqrt(float(np.mean(o ** 2))), float(np.max(o)), float(v.min()), float(v.max()),
                  float(np.nanmean(self.ess)), self.device, self.compute_s / self.n_calls * 1000, self.compute_max * 1000, self.n_calls))
