"""
KMPPI 闭环配置（Python 移植版）

来源: ~/Desktop/rl/kmppi/kmppi_14dof_config.m + vehicle14dof_params.m (MATLAB)。
数值默认值与 MATLAB 工程一致, 见 README.md 中的对照表。

两套预测模型参数:
  - "mathworks14dof": MATLAB 工程里为 MathWorks 14DOF Simscape 车标定的参数 (m=1600, I=3200, L=3.0)。
  - "chrono_bmw_e90" : 为 PyChrono BMW_E90 (TMeasy 轮胎) 准备的参数, 几何/质量来自 Chrono 实测,
                       轮胎侧向比例因子由 identify_plant.py 辨识后写入 bmw_e90_identified.json。
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field, asdict, replace
from typing import Optional, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
IDENTIFIED_JSON = os.path.join(HERE, "bmw_e90_identified.json")


@dataclass
class BicycleParams:
    """3DOF 动态自行车预测模型参数 (对应 vehicle14dof_params.m 中用到的子集)。"""
    m: float = 1600.0          # 整车质量 [kg]
    I: float = 3200.0          # 横摆惯量 [kg m^2]
    a: float = 1.5             # 质心到前轴 [m]
    b: float = 1.5             # 质心到后轴 [m]
    g: float = 9.81
    pac_By: float = 19.4       # Pacejka 侧向 B, C, E; D = 0.95*m*g/4 (每轮静载的 95%)
    pac_Cy: float = 1.3
    pac_Ey: float = -1.0
    pac_D_ratio: float = 0.95
    front_lateral_scale: float = 0.70   # 前轴每轮侧向力比例因子 (标定量)
    rear_lateral_scale: float = 0.82    # 后轴每轮侧向力比例因子 (标定量)

    @property
    def L(self) -> float:
        return self.a + self.b

    @property
    def pac_Dy(self) -> float:
        return self.pac_D_ratio * self.m * self.g / 4.0


def mathworks14dof_params() -> BicycleParams:
    return BicycleParams()


def chrono_bmw_e90_params() -> BicycleParams:
    """
    Chrono BMW_E90 + TMeasy 的几何/质量 (2026-09-05 实测: GetMass=1910 kg, Izz=3482, lf=1.371, lr=1.386)。
    轮胎比例因子: 若存在 bmw_e90_identified.json 则读取辨识结果, 否则沿用 MATLAB 的 0.70/0.82。
    """
    p = BicycleParams(m=1910.0, I=3482.0, a=1.371, b=1.386)
    if os.path.exists(IDENTIFIED_JSON):
        with open(IDENTIFIED_JSON) as f:
            ident = json.load(f)
        for k in ("m", "I", "a", "b", "front_lateral_scale", "rear_lateral_scale", "pac_By", "pac_Cy", "pac_Ey"):
            if k in ident:
                setattr(p, k, float(ident[k]))
    return p


@dataclass
class ChronoPlantConfig:
    """PyChrono BMW_E90 被控对象设置。"""
    step_size: float = 1e-3        # 多体积分步长 [s]
    tire_step_size: float = 1e-3   # 轮胎子步 [s]
    friction: float = 0.9          # 路面摩擦系数
    settle_time: float = 1.0       # 起步稳定时间 [s], 结束时刻定义为参考轨迹的 t=0, 位置原点
    drive_mode: str = "torque"     # "torque": 断开传动系, 四轮直接加扭矩 (与 14DOF 适配层一致); "throttle": 走发动机
    wheel_radius: float = 0.319    # 轮胎半径 [m]
    # 纵向命令适配: 每轮扭矩 = -(ax*accel_gain + rolling_comp) * m * r_w / 4  (负号: 轴上向前驱动为负)
    accel_gain: float = 1.0        # 对应 MATLAB 的 1/0.862, 由 identify_plant.py 辨识后覆盖
    rolling_comp: float = 0.0      # 对应 MATLAB 的 0.128 m/s^2, 由 identify_plant.py 辨识后覆盖
    # 转向命令适配: steering_input = delta / steer_gain (前轮平均转角 [rad] 每单位输入)
    steer_gain: float = 0.436      # 由 identify_plant.py 标定
    steer_table_u: Optional[Tuple[float, ...]] = None      # 可选非线性查表 (u -> delta)
    steer_table_delta: Optional[Tuple[float, ...]] = None
    vis: bool = False              # Irrlicht 3D 窗口
    vis_every: int = 10            # 每多少个内步渲染一帧

    def load_identified(self) -> "ChronoPlantConfig":
        if os.path.exists(IDENTIFIED_JSON):
            with open(IDENTIFIED_JSON) as f:
                ident = json.load(f)
            for k in ("accel_gain", "rolling_comp", "steer_gain", "wheel_radius"):
                if k in ident:
                    setattr(self, k, float(ident[k]))
            if "steer_table_u" in ident and "steer_table_delta" in ident:
                self.steer_table_u = tuple(ident["steer_table_u"])
                self.steer_table_delta = tuple(ident["steer_table_delta"])
        return self


@dataclass
class KMPPIConfig:
    """对应 kmppi_14dof_config.m。"""
    # --- 时间 ---
    dt: float = 0.05                 # 控制周期 [s]
    T_sim: Optional[float] = None    # None => 自动取完整一圈八字时长 (path_length / ref_speed = 76.215 s)
    rng_seed: int = 9
    # --- 参考轨迹: Gerono 八字 x=R sin(th), y=R sin(th) cos(th), 弧长恒速参数化 ---
    R_traj: float = 250.0
    ref_speed: float = 20.0
    constant_speed_reference: bool = True
    # --- 执行器约束 ---
    delta_max: float = 0.09          # 前轮转角 [rad]
    ax_max: float = 4.0              # 纵向加速度 [m/s^2]
    jerk_max: float = 2.0            # [m/s^3]
    steer_rate_max: float = 0.25     # [rad/s]
    # --- KMPPI ---
    K: int = 2048                    # 样本数
    T: int = 33                      # 预测步数 (14DOF 用 33, 3DOF 基准用 60)
    num_support_pts: int = 9         # RBF 支撑点数 P
    lambda_: float = 0.01            # 基础温度 (也用于噪声修正项)
    adaptive_temperature: bool = True
    target_effective_sample_size: float = 16.0
    sigma_RBF: float = 4.0
    noise_std: Tuple[float, float] = (0.010, 0.006)   # [jerk, steer_rate] 探索标准差
    reuse_antithetic_samples: bool = True
    num_refinement_steps: int = 2
    # --- 代价 ---
    Qdiag: Tuple[float, ...] = (200.0, 200.0, 700.0, 50.0, 40.0, 500.0)  # [x, y, yaw, vx, vy, r]
    Rdiag: Tuple[float, float] = (0.08, 2.0)        # 实际动作 [ax, delta] 代价
    rateRdiag: Tuple[float, float] = (0.2, 2.0)     # 导数动作 [jerk, steer_rate] 代价
    # --- 被控对象 / 预测模型 ---
    plant: str = "chrono"            # "chrono" | "3dof"
    vehicle_params: str = "auto"     # "auto" | "mathworks14dof" | "chrono_bmw_e90"
    chrono: ChronoPlantConfig = field(default_factory=ChronoPlantConfig)

    def resolved_vehicle(self) -> BicycleParams:
        name = self.vehicle_params
        if name == "auto":
            name = "chrono_bmw_e90" if self.plant == "chrono" else "mathworks14dof"
        if name == "mathworks14dof":
            return mathworks14dof_params()
        if name == "chrono_bmw_e90":
            return chrono_bmw_e90_params()
        raise ValueError(f"unknown vehicle_params {name!r}")

    def to_dict(self) -> dict:
        d = asdict(self)
        d["vehicle_resolved"] = asdict(self.resolved_vehicle())
        return d
