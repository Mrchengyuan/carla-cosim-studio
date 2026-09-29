"""
KMPPI + 学习世界模型 (kmppi_dream) 配置。

与 ~/Desktop/rl/kmppi_chrono 的区别: 预测模型不再是 3DOF 物理自行车, 而是从 Chrono BMW_E90 数据训练的
神经网络世界模型 (world_model.py)。KMPPI 参数、参考轨迹、执行器约束、Chrono 被控对象接口保持一致。
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass, field, asdict
from typing import Optional, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
IDENTIFIED_JSON = os.path.join(HERE, "bmw_e90_identified.json")
DEFAULT_MODEL_PATH = os.path.join(HERE, "models", "world_model.pt")


@dataclass
class ChronoPlantConfig:
    """PyChrono BMW_E90 被控对象设置 (同 kmppi_chrono)。"""
    step_size: float = 1e-3
    tire_step_size: float = 1e-3
    friction: float = 0.9
    settle_time: float = 1.0
    drive_mode: str = "torque"
    wheel_radius: float = 0.319
    accel_gain: float = 1.0
    rolling_comp: float = 0.0
    steer_gain: float = 0.436
    steer_table_u: Optional[Tuple[float, ...]] = None
    steer_table_delta: Optional[Tuple[float, ...]] = None
    vis: bool = False
    vis_every: int = 10
    aero_cd: float = 0.30          # 风阻: F = 0.5*rho*Cd*A*v^2 (BMW E90 模板默认没有风阻, 高速必须加)
    aero_area: float = 2.2
    air_density: float = 1.2
    terrain_size: float = 0.0      # 0 => 按 R_traj 自动; 跑赛道时由运行器按赛道范围设置

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
class WorldModelConfig:
    """学习世界模型: 输入最近 history 步的 [vx, vy, r, ax, delta], 输出下一步 [dvx, dvy, dr]。"""
    history: int = 2                 # 历史窗口长度 H (H=1 为马尔可夫模型; H>1 让模型能推断侧倾/轮胎弛豫等隐状态)
    hidden: int = 128
    layers: int = 2
    rollout_len: int = 8             # 训练时的多步展开长度 (抑制误差累积)
    epochs: int = 60
    batch: int = 512
    lr: float = 2e-3
    weight_decay: float = 1e-5
    val_fraction: float = 0.15
    seed: int = 0
    path: str = DEFAULT_MODEL_PATH


@dataclass
class KMPPIConfig:
    dt: float = 0.05
    T_sim: Optional[float] = None
    rng_seed: int = 9
    # 参考轨迹
    R_traj: float = 250.0
    ref_speed: float = 20.0
    constant_speed_reference: bool = True
    # 执行器约束
    delta_max: float = 0.09
    ax_max: float = 4.0
    jerk_max: float = 2.0
    steer_rate_max: float = 0.25
    # KMPPI
    K: int = 2048
    T: int = 33
    num_support_pts: int = 9
    lambda_: float = 0.01
    adaptive_temperature: bool = True
    target_effective_sample_size: float = 16.0
    sigma_RBF: float = 4.0
    noise_std: Tuple[float, float] = (0.010, 0.006)
    reuse_antithetic_samples: bool = True
    num_refinement_steps: int = 2
    consistent_temperature: bool = False   # KMPPI 改进 2: 修正项与权重温度一致 (见 NOTES.md 第 4 节)
    # 代价
    Qdiag: Tuple[float, ...] = (200.0, 200.0, 700.0, 50.0, 40.0, 500.0)
    Rdiag: Tuple[float, float] = (0.08, 2.0)
    rateRdiag: Tuple[float, float] = (0.2, 2.0)
    frenet_position_cost: bool = False   # 位置代价在参考 Frenet 系 (横向 Q_lat, 纵向 Q_lon), 替代全局 x/y 项
    Q_lon: float = 20.0
    Q_lat: float = 200.0
    v_cap: float = 0.0               # 速度上限 (剖面 v_max + 余量), 超出部分 Q_vcap 二次惩罚; 0 = 不用
    Q_vcap: float = 0.0
    Q_boundary: float = 0.0          # 赛道边界二次惩罚权重 (跑赛道时 >0)
    boundary_margin: float = 0.5     # 离边界的安全余量 [m]
    # 速度相关转角限幅: delta_max(v) = clip(steer_limit_factor * mu*g*L / v^2, delta_min_bound, delta_max)
    speed_dependent_steer_limit: bool = False
    steer_limit_factor: float = 2.0    # 1.3 在 Monza Curva Grande (54 m/s) 卡死转向; 转向柔度+侧偏要求比运动学大得多
    delta_min_bound: float = 0.003   # 60 m/s 时物理转角上限约 0.007 rad, 下限必须低于它
    # KMPPI 改进 A1: 转向导数动作按速度归一化. 物理转向角速度 = u_steer * delta_lim(v)/delta_lim(steer_ref_speed), rollout 内逐步按 v 限幅
    steer_speed_scaling: bool = False
    steer_ref_speed: float = 20.0
    steer_gain_max: float = 1e9   # A1 增益上限: 低速时 delta_lim 饱和在 delta_max, 增益到 2.46, 可用它限制低速段的转向速率/噪声放大
    # KMPPI 改进 A2: 支撑点近密远疏, grid = (T-1)*(p/(P-1))^support_power (1.0 = 均匀); 核长度尺度随局部间距缩放 (Gibbs 核)
    support_power: float = 1.0
    # KMPPI 改进 B2: 支撑点更新步长 support += update_step * sum_k w_k eps_k
    update_step: float = 1.0
    # KMPPI 改进 B1: 可微世界模型的梯度精修. 每次采样后取 rollout 代价最低的 grad_elite 条样本, 对其支撑点值做 grad_steps 步 Adam
    # (步长 grad_lr 以噪声 sigma 为单位, 信赖域 |dz| <= grad_trust sigma), 只接受代价下降的结果, 再参与加权平均. 0 = 关闭
    grad_steps: int = 0
    grad_elite: int = 64
    grad_lr: float = 0.5
    grad_trust: float = 2.0
    # 被控对象 / 世界模型
    chrono: ChronoPlantConfig = field(default_factory=ChronoPlantConfig)
    world_model: WorldModelConfig = field(default_factory=WorldModelConfig)

    def to_dict(self) -> dict:
        return asdict(self)
