"""
控制器内部 3DOF 非线性动态自行车预测模型 (移植自 kmppi_14dof_prediction_dynamics.m)。

状态 x = [X, Y, yaw, vx, vy, r], 动作 a = [ax, delta], 显式欧拉, 步长 dt。
侧向轮胎: Pacejka 魔术公式 (B, C, D, E), 每轴 2 个轮, 前/后各乘一个比例因子。
纵向: Fx = m * ax (理想瞬时响应)。
"""
from __future__ import annotations

import numpy as np

from kmppi_config import BicycleParams


def pacejka_lateral(alpha, B, C, D, E):
    Ba = B * alpha
    return D * np.sin(C * np.arctan(Ba - E * (Ba - np.arctan(Ba))))


def wrap_angle(a):
    return np.arctan2(np.sin(a), np.cos(a))


class BicycleModel:
    def __init__(self, p: BicycleParams, dt: float, ax_max: float, delta_max: float):
        self.p = p
        self.dt = dt
        self.ax_max = ax_max
        self.delta_max = delta_max

    def step(self, state: np.ndarray, action: np.ndarray) -> np.ndarray:
        """state (K,6), action (K,2) -> next state (K,6)。"""
        p, dt = self.p, self.dt
        x = state[:, 0]
        y = state[:, 1]
        yaw = state[:, 2]
        vx = np.maximum(state[:, 3], 0.5)
        vy = state[:, 4]
        r = state[:, 5]
        ax = np.clip(action[:, 0], -self.ax_max, self.ax_max)
        delta = np.clip(action[:, 1], -self.delta_max, self.delta_max)

        alpha_f = delta - np.arctan2(vy + p.a * r, vx)
        alpha_r = -np.arctan2(vy - p.b * r, vx)
        Fy_f = 2.0 * p.front_lateral_scale * pacejka_lateral(alpha_f, p.pac_By, p.pac_Cy, p.pac_Dy, p.pac_Ey)
        Fy_r = 2.0 * p.rear_lateral_scale * pacejka_lateral(alpha_r, p.pac_By, p.pac_Cy, p.pac_Dy, p.pac_Ey)
        Fx = p.m * ax

        cd, sd = np.cos(delta), np.sin(delta)
        vx_dot = (Fx - Fy_f * sd + p.m * vy * r) / p.m
        vy_dot = (Fy_f * cd + Fy_r - p.m * vx * r) / p.m
        r_dot = (p.a * Fy_f * cd - p.b * Fy_r) / p.I
        cy, sy = np.cos(yaw), np.sin(yaw)
        x_dot = vx * cy - vy * sy
        y_dot = vx * sy + vy * cy

        nxt = np.empty_like(state)
        nxt[:, 0] = x + dt * x_dot
        nxt[:, 1] = y + dt * y_dot
        nxt[:, 2] = wrap_angle(yaw + dt * r)
        nxt[:, 3] = np.maximum(vx + dt * vx_dot, 0.0)
        nxt[:, 4] = vy + dt * vy_dot
        nxt[:, 5] = r + dt * r_dot
        return nxt


class BicyclePlant:
    """3DOF 基准被控对象 (对应 vehicle3dof_sfun.m): 与预测模型完全同源。"""

    def __init__(self, model: BicycleModel, initial_state: np.ndarray):
        self.model = model
        self.state = np.asarray(initial_state, dtype=float).reshape(1, 6).copy()

    def get_state(self) -> np.ndarray:
        return self.state[0].copy()

    def step(self, ax: float, delta: float) -> np.ndarray:
        self.state = self.model.step(self.state, np.array([[ax, delta]]))
        return self.get_state()

    def audit(self):
        return None
