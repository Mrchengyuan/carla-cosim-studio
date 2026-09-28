"""
KMPPI: 带 RBF 核参数化与导数动作提升 (input lifting) 的模型预测路径积分控制器。
逐函数移植自 ~/Desktop/rl/kmppi/KMPPIController.m (MATLAB), 语义保持一致:

  * 优化变量 u = [jerk, steer_rate] 只在 P 个支撑点上采样, 用 RBF 核插值到 T 步时域;
  * 实际动作 a = [ax, delta] 由 a_{t+1} = clip(a_t + dt * u_t) 积分得到 (input lifting);
  * 对称采样 (epsilon, -epsilon) + 跨周期复用同一组标准正态样本 (common random numbers);
  * 自适应温度: 若 ESS = 1/sum(w^2) < 目标值, 用几何二分把温度抬到刚好满足目标 ESS;
  * 代价 = 沿 rollout 的 sum(时变状态代价 + 实际动作代价 + 导数动作代价) + MPPI 噪声修正项;
  * 滚动时域: 支撑点值沿时间轴平移一步 (核回归插值), 每个控制周期可做多次 refinement。
"""
from __future__ import annotations

from typing import Callable, Optional

import numpy as np


class KMPPIController:
    def __init__(self, dynamics_fn: Callable, params: dict, rng: Optional[np.random.Generator] = None):
        self.dynamics_fn = dynamics_fn                    # (state (K,nx), action (K,nu)) -> (K,nx)
        self.time_cost_fn: Optional[Callable] = None      # (state, action, t_index) -> (K,)
        self.nx = int(params["nx"])
        self.nu = int(params["nu"])
        self.K = int(params["K"])
        self.T = int(params["T"])
        self.P = int(params["num_support_pts"])
        self.lambda_ = float(params["lambda_"])
        if self.lambda_ <= 0:
            raise ValueError("lambda must be positive")
        self.adaptive_temperature = bool(params["adaptive_temperature"])
        self.target_ess = float(params["target_effective_sample_size"])
        if not (1 <= self.target_ess <= self.K):
            raise ValueError("target ESS must be within [1, K]")
        if not (2 <= self.P <= self.T):
            raise ValueError("support count must satisfy 2 <= P <= T")

        cov = np.asarray(params["noise_sigma"], dtype=float)
        if cov.ndim == 0:
            cov = cov * np.eye(self.nu)
        cov = 0.5 * (cov + cov.T)
        self.noise_sigma = cov
        self.chol_factor = np.linalg.cholesky(cov)           # 抛异常即非正定
        self.noise_sigma_inv = np.linalg.inv(cov)
        self.is_diagonal_sigma = bool(np.all(cov[~np.eye(self.nu, dtype=bool)] == 0))
        self.sigma_sqrt_diag = np.sqrt(np.diag(cov))
        self.noise_mu = np.asarray(params.get("noise_mu", np.zeros(self.nu)), dtype=float).reshape(self.nu)

        self.rng = rng if rng is not None else np.random.default_rng()
        self.reuse_antithetic_samples = bool(params["reuse_antithetic_samples"])
        self.standard_normal_bank = self._antithetic_normal(self.K, self.P)

        self.optimizer_min = np.asarray(params["u_min"], dtype=float).reshape(self.nu)
        self.optimizer_max = np.asarray(params["u_max"], dtype=float).reshape(self.nu)
        self.input_lifting = bool(params["input_lifting"])
        self.action_dt = float(params["action_dt"])
        self.actual_action_min = np.asarray(params["action_min"], dtype=float).reshape(self.nu)
        self.actual_action_max = np.asarray(params["action_max"], dtype=float).reshape(self.nu)
        self.rate_weight = np.asarray(params["rate_weight"], dtype=float).reshape(self.nu)
        self.sigma_rbf = float(params["sigma_RBF"])

        # RBF 核插值矩阵: 支撑点网格 -> 时域网格
        self.support_grid = np.linspace(0.0, self.T - 1, self.P)
        self.horizon_grid = np.arange(self.T, dtype=float)
        Kss = self._rbf(self.support_grid, self.support_grid) + 1e-8 * np.eye(self.P)
        self.interpolation_matrix = np.linalg.solve(Kss, self._rbf(self.horizon_grid, self.support_grid).T).T  # (T,P)
        # 滚动时域平移矩阵: 在 support_grid+1 处插值
        self.shift_matrix = np.linalg.solve(Kss, self._rbf(self.support_grid + 1.0, self.support_grid).T).T  # (P,P)

        self.nominal_sequence = np.zeros((self.T, self.nu))
        self.support_values = np.zeros((self.P, self.nu))
        self.current_state = np.zeros(self.nx)
        self.applied_action = np.zeros(self.nu)
        self.last_total_cost: Optional[np.ndarray] = None
        self.last_weights: Optional[np.ndarray] = None
        self.last_temperature = self.lambda_
        self.last_effective_sample_size = np.nan
        self.last_min_cost = np.nan
        self.last_mean_cost = np.nan   # 加权均值 sum(w * cost)

    # ------------------------------------------------------------------ 公共接口
    def set_cost_fn_t(self, fn: Callable):
        self.time_cost_fn = fn

    def reset(self):
        self.nominal_sequence[:] = 0.0
        self.support_values[:] = 0.0
        self.applied_action[:] = 0.0
        self.last_total_cost = None
        self.last_weights = None
        self.last_temperature = self.lambda_
        self.last_effective_sample_size = np.nan

    def command(self, state: np.ndarray, shift_horizon: bool = True, commit_action: bool = True) -> np.ndarray:
        state = np.asarray(state, dtype=float).reshape(-1)
        if state.size != self.nx:
            raise ValueError("state dimension mismatch")
        if shift_horizon:
            self._shift_nominal_horizon()
        self.current_state = state
        total_cost, support_noise = self._compute_total_cost_batch()
        weights = self._compute_weights(total_cost)
        weighted_noise = np.einsum("k,kpu->pu", weights, support_noise)
        self.support_values = self.support_values + weighted_noise
        self.nominal_sequence = self.interpolation_matrix @ self.support_values
        u0 = np.clip(self.nominal_sequence[0], self.optimizer_min, self.optimizer_max)
        self.nominal_sequence[0] = u0
        if self.input_lifting:
            actual = np.clip(self.applied_action + self.action_dt * u0, self.actual_action_min, self.actual_action_max)
        else:
            actual = u0
        if commit_action:
            self.applied_action = actual.copy()
        self.last_total_cost = total_cost
        self.last_weights = weights
        self.last_min_cost = float(np.min(total_cost))
        self.last_mean_cost = float(np.sum(weights * total_cost))
        return actual

    # ------------------------------------------------------------------ 内部
    def _rbf(self, g1: np.ndarray, g2: np.ndarray) -> np.ndarray:
        d2 = (g1.reshape(-1, 1) - g2.reshape(1, -1)) ** 2
        return np.exp(-d2 / (2.0 * self.sigma_rbf ** 2))

    def _antithetic_normal(self, count: int, support_count: int) -> np.ndarray:
        half = count // 2
        pos = self.rng.standard_normal((half, support_count, self.nu))
        bank = np.concatenate([pos, -pos], axis=0)
        if count % 2 == 1:
            bank = np.concatenate([bank, np.zeros((1, support_count, self.nu))], axis=0)
        return bank

    def _shift_nominal_horizon(self):
        self.support_values = self.shift_matrix @ self.support_values
        self.nominal_sequence = self.interpolation_matrix @ self.support_values

    def _sample_noise(self) -> np.ndarray:
        z = self.standard_normal_bank if self.reuse_antithetic_samples else self._antithetic_normal(self.K, self.P)
        if self.is_diagonal_sigma:
            return z * self.sigma_sqrt_diag.reshape(1, 1, -1) + self.noise_mu.reshape(1, 1, -1)
        flat = z.reshape(-1, self.nu) @ self.chol_factor.T + self.noise_mu
        return flat.reshape(self.K, self.P, self.nu)

    def _bound_optimizer(self, values: np.ndarray) -> np.ndarray:
        return np.clip(values, self.optimizer_min, self.optimizer_max)

    def _sample_perturbed_sequences(self):
        raw = self._sample_noise()                                     # (K,P,nu)
        center = self.support_values.reshape(1, self.P, self.nu)
        perturbed_support = self._bound_optimizer(center + raw)
        support_noise = perturbed_support - center
        perturbed_sequence = np.einsum("tp,kpu->ktu", self.interpolation_matrix, perturbed_support)
        perturbed_sequence = self._bound_optimizer(perturbed_sequence)  # (K,T,nu)
        sequence_noise = perturbed_sequence - self.nominal_sequence.reshape(1, self.T, self.nu)
        return perturbed_sequence, support_noise, sequence_noise

    def _compute_noise_correction(self, sequence_noise: np.ndarray) -> np.ndarray:
        if self.is_diagonal_sigma:
            inv_var = 1.0 / (self.sigma_sqrt_diag ** 2)
            return self.lambda_ * sequence_noise * inv_var.reshape(1, 1, -1)
        flat = sequence_noise.reshape(-1, self.nu) @ self.noise_sigma_inv
        return self.lambda_ * flat.reshape(self.K, self.T, self.nu)

    def _rollout_cost(self, perturbed_sequence: np.ndarray) -> np.ndarray:
        state = np.repeat(self.current_state.reshape(1, -1), self.K, axis=0)
        actual = np.repeat(self.applied_action.reshape(1, -1), self.K, axis=0)
        cost = np.zeros(self.K)
        for t in range(self.T):
            u = perturbed_sequence[:, t, :]
            if self.input_lifting:
                actual = np.clip(actual + self.action_dt * u, self.actual_action_min, self.actual_action_max)
                cost += np.sum(u ** 2 * self.rate_weight, axis=1)
            else:
                actual = u
            state = self.dynamics_fn(state, actual)
            if self.time_cost_fn is None:
                raise RuntimeError("time_cost_fn not set")
            cost += self.time_cost_fn(state, actual, t)
        return cost

    def _compute_total_cost_batch(self):
        perturbed_sequence, support_noise, sequence_noise = self._sample_perturbed_sequences()
        correction = self._compute_noise_correction(sequence_noise)
        correction_cost = np.einsum("tu,ktu->k", self.nominal_sequence, correction)
        rollout = self._rollout_cost(perturbed_sequence)
        return rollout + correction_cost, support_noise

    def _compute_weights(self, total_cost: np.ndarray) -> np.ndarray:
        centered = total_cost - np.min(total_cost)
        temperature = self.lambda_

        def weights_at(temp):
            w = np.exp(-centered / temp)
            w /= np.sum(w)
            return w, 1.0 / np.sum(w ** 2)

        weights, ess = weights_at(temperature)
        if self.adaptive_temperature and ess < self.target_ess:
            lower = temperature
            upper = max(float(np.max(centered)), 2.0 * lower)
            for _ in range(40):
                cand = np.sqrt(lower * upper)
                _, cand_ess = weights_at(cand)
                if cand_ess < self.target_ess:
                    lower = cand
                else:
                    upper = cand
            temperature = upper
            weights, ess = weights_at(temperature)
        self.last_temperature = temperature
        self.last_effective_sample_size = ess
        return weights


def build_kmppi(cfg, model_step_fn, reference_box, rng) -> KMPPIController:
    """对应 make_kmppi_14dof_controller.m。reference_box.val 是 (T,6) 的未来参考。"""
    params = dict(
        nx=6, nu=2, K=cfg.K, T=cfg.T, num_support_pts=cfg.num_support_pts,
        lambda_=cfg.lambda_, adaptive_temperature=cfg.adaptive_temperature,
        target_effective_sample_size=cfg.target_effective_sample_size,
        noise_sigma=np.diag(np.asarray(cfg.noise_std) ** 2),
        reuse_antithetic_samples=cfg.reuse_antithetic_samples,
        u_min=[-cfg.jerk_max, -cfg.steer_rate_max], u_max=[cfg.jerk_max, cfg.steer_rate_max],
        sigma_RBF=cfg.sigma_RBF, input_lifting=True, action_dt=cfg.dt,
        action_min=[-cfg.ax_max, -cfg.delta_max], action_max=[cfg.ax_max, cfg.delta_max],
        rate_weight=cfg.rateRdiag,
    )
    Q = np.asarray(cfg.Qdiag, dtype=float)
    R = np.asarray(cfg.Rdiag, dtype=float)

    def time_cost(state, action, t_index):
        ref = reference_box.val[t_index]
        yaw_err = np.arctan2(np.sin(state[:, 2] - ref[2]), np.cos(state[:, 2] - ref[2]))
        err = np.stack([state[:, 0] - ref[0], state[:, 1] - ref[1], yaw_err,
                        state[:, 3] - ref[3], state[:, 4] - ref[4], state[:, 5] - ref[5]], axis=1)
        return np.sum(err ** 2 * Q, axis=1) + np.sum(action ** 2 * R, axis=1)

    ctrl = KMPPIController(model_step_fn, params, rng=rng)
    ctrl.set_cost_fn_t(time_cost)
    return ctrl


class ReferenceBox:
    """对应 RefHolder.m: 控制器与参考生成器共享的 (T,6) 未来参考。"""

    def __init__(self, T: int):
        self.val = np.zeros((T, 6))
        self.t_now = 0.0
