"""
KMPPI: 带 RBF 核参数化与导数动作提升 (input lifting) 的模型预测路径积分控制器。
逐函数移植自 ~/Desktop/rl/kmppi/KMPPIController.m (MATLAB), 语义保持一致 (kmppi_dream 版: dynamics 可以是有状态的学习世界模型):

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
        # 温度一致性: MATLAB 原版噪声修正项用固定 lambda_, 权重温度自适应 -> 两处不一致。
        # consistent_temperature=True 时修正项与权重用同一温度: w ∝ exp(-rollout/T) * exp(-u^T Σ^-1 ε), 修正项在指数里与 T 无关。
        self.consistent_temperature = bool(params.get("consistent_temperature", False))
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

        # RBF 核插值矩阵: 支撑点网格 -> 时域网格 (改进 A2: support_power>1 时近密远疏, 核长度尺度随局部间距缩放)
        self.support_power = float(params.get("support_power", 1.0))
        self.support_grid = (self.T - 1) * np.linspace(0.0, 1.0, self.P) ** self.support_power
        self.horizon_grid = np.arange(self.T, dtype=float)
        uniform_spacing = (self.T - 1) / (self.P - 1)
        self.support_ls = self.sigma_rbf * np.gradient(self.support_grid) / uniform_spacing   # 均匀网格时恒等于 sigma_RBF
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
        self.cost_params = None        # 设备上 rollout 用: dict(Q, R, reference_box)
        self.prior_support = None      # 策略先验给的支撑点值 (P, nu); command() 里与平移后的名义序列混合
        self.prior_blend = 0.0
        self.update_step = float(params.get("update_step", 1.0))   # 改进 B2: 更新阻尼
        # 改进 A1: steer_scale = dict(c, dmin, dmax, ref) 交给设备端 rollout 逐步缩放/限幅转向; steer_gain 是当前周期提交动作用的增益
        self.steer_scale = None
        self.steer_gain = 1.0
        # 改进 B1: 梯度精修 (见 _polish_elites)
        self.grad_steps = int(params.get("grad_steps", 0))
        self.grad_elite = int(params.get("grad_elite", 64))
        self.grad_lr = float(params.get("grad_lr", 0.3))
        self.grad_trust = float(params.get("grad_trust", 2.0))
        self.last_polish_gain = np.nan     # 精英样本精修后的平均代价下降
        self.last_polish_accept = np.nan   # 被接受 (代价下降) 的比例

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
            if self.prior_support is not None and self.prior_blend > 0:
                self.support_values = (1.0 - self.prior_blend) * self.support_values + self.prior_blend * self._bound_optimizer(self.prior_support)
                self.nominal_sequence = self.interpolation_matrix @ self.support_values
        self.current_state = state
        total_cost, support_noise = self._compute_total_cost_batch()
        weights = self._compute_weights(total_cost)
        weighted_noise = np.einsum("k,kpu->pu", weights, support_noise)
        self.support_values = self.support_values + self.update_step * weighted_noise
        self.nominal_sequence = self.interpolation_matrix @ self.support_values
        u0 = np.clip(self.nominal_sequence[0], self.optimizer_min, self.optimizer_max)
        self.nominal_sequence[0] = u0
        if self.input_lifting:
            u_eff = u0.copy()
            u_eff[1] *= self.steer_gain
            actual = np.clip(self.applied_action + self.action_dt * u_eff, self.actual_action_min, self.actual_action_max)
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
    def _length_scale(self, g: np.ndarray) -> np.ndarray:
        return np.interp(np.asarray(g, dtype=float), self.support_grid, self.support_ls)

    def _rbf(self, g1: np.ndarray, g2: np.ndarray) -> np.ndarray:
        """Gibbs 非平稳 RBF 核; 长度尺度处处相等 (均匀支撑点) 时严格退化为原版 exp(-d^2 / (2 sigma^2))。"""
        l1 = self._length_scale(g1).reshape(-1, 1)
        l2 = self._length_scale(g2).reshape(1, -1)
        d2 = (np.asarray(g1, dtype=float).reshape(-1, 1) - np.asarray(g2, dtype=float).reshape(1, -1)) ** 2
        s2 = l1 ** 2 + l2 ** 2
        return np.sqrt(2.0 * l1 * l2 / s2) * np.exp(-d2 / s2)

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
        owner = getattr(self.dynamics_fn, "__self__", None)
        if owner is not None and self.cost_params is not None and self.cost_params.get("race") is not None:
            rp = self.cost_params["race"]
            return owner.rollout_cost_race(perturbed_sequence, self.current_state, self.applied_action, self.action_dt,
                                           self.actual_action_min, self.actual_action_max, self.rate_weight, self.input_lifting,
                                           self.cost_params["window"], rp, steer_scale=self.steer_scale)
        if owner is not None and hasattr(owner, "rollout_cost") and self.cost_params is not None:
            cp = self.cost_params   # 学习世界模型: 整段 rollout + 代价留在 torch 设备上 (CPU/MPS)
            return owner.rollout_cost(perturbed_sequence, self.current_state, self.applied_action, self.action_dt,
                                      self.actual_action_min, self.actual_action_max, self.rate_weight, self.input_lifting,
                                      cp["reference_box"].val, cp["Q"], cp["R"], cp.get("Q_boundary", 0.0), cp.get("boundary_margin", 0.0),
                                      cp.get("frenet", False), cp.get("Q_lon", 0.0), cp.get("Q_lat", 0.0), cp.get("v_cap", 0.0), cp.get("Q_vcap", 0.0),
                                      steer_scale=self.steer_scale)
        if owner is not None and hasattr(owner, "begin_rollout"):
            owner.begin_rollout(self.K)   # 有状态的学习模型: 每次 rollout 用真实历史重置
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

    def _torch_cost_fn(self):
        """B1 用: 返回 U (E,T,nu) tensor -> (E,) 代价 tensor 的闭包 (与 _rollout_cost 同一代价)。"""
        owner = getattr(self.dynamics_fn, "__self__", None)
        if owner is None or self.cost_params is None:
            return None
        if self.cost_params.get("race") is not None and hasattr(owner, "rollout_cost_race_torch"):
            rp = self.cost_params["race"]
            return lambda U: owner.rollout_cost_race_torch(U, self.current_state, self.applied_action, self.action_dt, self.actual_action_min,
                                                           self.actual_action_max, self.rate_weight, self.input_lifting, self.cost_params["window"], rp,
                                                           steer_scale=self.steer_scale)
        if hasattr(owner, "rollout_cost_torch"):
            cp = self.cost_params
            return lambda U: owner.rollout_cost_torch(U, self.current_state, self.applied_action, self.action_dt, self.actual_action_min,
                                                      self.actual_action_max, self.rate_weight, self.input_lifting, cp["reference_box"].val, cp["Q"], cp["R"],
                                                      cp.get("Q_boundary", 0.0), cp.get("boundary_margin", 0.0), cp.get("frenet", False), cp.get("Q_lon", 0.0),
                                                      cp.get("Q_lat", 0.0), cp.get("v_cap", 0.0), cp.get("Q_vcap", 0.0), steer_scale=self.steer_scale)
        return None

    def _polish_elites(self, rollout, support_noise, sequence_noise):
        """B1: 取 rollout 代价最低的 grad_elite 条样本, 沿可微世界模型的梯度精修其支撑点值, 只接受代价下降的, 写回样本集。"""
        owner = getattr(self.dynamics_fn, "__self__", None)
        cost_fn = self._torch_cost_fn()
        if owner is None or cost_fn is None or not hasattr(owner, "polish_support"):
            return rollout, support_noise, sequence_noise
        E = int(min(self.grad_elite, self.K))
        idx = np.argpartition(rollout, E - 1)[:E]
        center = self.support_values.reshape(1, self.P, self.nu)
        refined, refined_cost = owner.polish_support(center + support_noise[idx], self.interpolation_matrix, self.optimizer_min, self.optimizer_max,
                                                     self.sigma_sqrt_diag, self.grad_steps, self.grad_lr, self.grad_trust, cost_fn)
        ok = np.isfinite(refined_cost) & (refined_cost < rollout[idx])
        self.last_polish_accept = float(np.mean(ok))
        self.last_polish_gain = float(np.mean(rollout[idx][ok] - refined_cost[ok])) if np.any(ok) else 0.0
        if np.any(ok):
            sel = idx[ok]
            support_noise[sel] = refined[ok] - center
            seq = self._bound_optimizer(np.einsum("tp,kpu->ktu", self.interpolation_matrix, refined[ok]))
            sequence_noise[sel] = seq - self.nominal_sequence.reshape(1, self.T, self.nu)
            rollout[sel] = refined_cost[ok]
        return rollout, support_noise, sequence_noise

    def _compute_total_cost_batch(self):
        perturbed_sequence, support_noise, sequence_noise = self._sample_perturbed_sequences()
        rollout = self._rollout_cost(perturbed_sequence)
        if self.grad_steps > 0 and self.grad_elite > 0:
            rollout, support_noise, sequence_noise = self._polish_elites(rollout, support_noise, sequence_noise)
        correction = self._compute_noise_correction(sequence_noise)
        correction_cost = np.einsum("tu,ktu->k", self.nominal_sequence, correction)   # = lambda_ * u^T Σ^-1 ε
        if self.consistent_temperature:
            self._last_corr_unit = correction_cost / self.lambda_        # u^T Σ^-1 ε (无温度)
            return rollout, support_noise
        self._last_corr_unit = None
        return rollout + correction_cost, support_noise

    def _compute_weights(self, total_cost: np.ndarray) -> np.ndarray:
        centered = total_cost - np.min(total_cost)
        temperature = self.lambda_
        corr = getattr(self, "_last_corr_unit", None)
        corr_shift = 0.0 if corr is None else corr - np.min(corr)

        def weights_at(temp):
            # Cost and density correction may attain minima at different samples.
            # Stabilize their combined logits; centering each term separately can
            # otherwise underflow every weight even when all inputs are finite.
            log_weights = -centered / temp - corr_shift
            log_weights -= np.max(log_weights)
            w = np.exp(log_weights)
            w /= np.sum(w)
            return w, 1.0 / np.sum(w ** 2)

        weights, ess = weights_at(temperature)
        if self.adaptive_temperature and ess < self.target_ess:
            lower = temperature
            upper = max(float(np.max(centered)), 2.0 * lower)
            search_target = self.target_ess
            if corr is None:
                # Pure Gibbs ESS increases toward K, but max(cost spread) is
                # not necessarily a feasible upper bracket for a high target.
                # ESS=K is an infinite-temperature limit for unequal costs;
                # solve that endpoint to a relative1e-12 numerical tolerance.
                if search_target == self.K:
                    search_target *= 1.0 - 1e-12
                _, upper_ess = weights_at(upper)
                while upper_ess < search_target:
                    upper *= 2.0
                    if not np.isfinite(upper):
                        raise FloatingPointError("Cannot bracket the requested Gibbs ESS at a finite temperature")
                    _, upper_ess = weights_at(upper)
            # Fixed nonuniform density correction does not have this monotonic property.
            for _ in range(40):
                cand = np.sqrt(lower * upper)
                _, cand_ess = weights_at(cand)
                if cand_ess < search_target:
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
        consistent_temperature=bool(getattr(cfg, "consistent_temperature", False)),
        support_power=float(getattr(cfg, "support_power", 1.0)), update_step=float(getattr(cfg, "update_step", 1.0)),
        grad_steps=int(getattr(cfg, "grad_steps", 0)), grad_elite=int(getattr(cfg, "grad_elite", 64)),
        grad_lr=float(getattr(cfg, "grad_lr", 0.3)), grad_trust=float(getattr(cfg, "grad_trust", 2.0)),
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
    frenet = bool(getattr(cfg, "frenet_position_cost", False))
    if frenet:
        Q = Q.copy(); Q[0] = 0.0; Q[1] = 0.0     # 全局 x/y 项由 Frenet 项替代
    ctrl.cost_params = dict(Q=Q, R=R, reference_box=reference_box,
                            Q_boundary=float(getattr(cfg, "Q_boundary", 0.0)), boundary_margin=float(getattr(cfg, "boundary_margin", 0.0)),
                            frenet=frenet, Q_lon=float(getattr(cfg, "Q_lon", 0.0)), Q_lat=float(getattr(cfg, "Q_lat", 0.0)),
                            v_cap=float(getattr(cfg, "v_cap", 0.0)), Q_vcap=float(getattr(cfg, "Q_vcap", 0.0)))
    return ctrl


class ReferenceBox:
    """对应 RefHolder.m: 控制器与参考生成器共享的 (T,6) 未来参考。"""

    def __init__(self, T: int):
        self.val = np.zeros((T, 6))
        self.t_now = 0.0
