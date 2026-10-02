"""预测模型：Frenet 坐标（沿参考线的弧长 s、横向偏差 ey）下的动态自行车 + Pacejka 侧向轮胎 + 执行器滞后。

状态 x（9 个，全部是质心的量）：
    0 s     沿参考线的弧长，m            1 ey    离参考线的横向距离（左为正），m
    2 epsi  航向 − 参考线方向，rad       3 vx    纵向车速，m/s
    4 vy    侧向车速，m/s                5 r     横摆角速度，rad/s
    6 delta 实际前轮转角，rad            7 ax    实际纵向加速度（驱动 / 制动力 ÷ 质量），m/s²
    8 dc    前轮转角指令，rad
输入 u（2 个）：
    0 ax_cmd  纵向加速度指令，m/s²       1 dc_rate 前轮转角指令的变化率，rad/s
执行器：delta 以时间常数 TAU_DELTA 跟随 dc，ax 以 TAU_AX 跟随 ax_cmd（转向、动力系统的滞后）。
低速（vx < V_DYN_LO）动态自行车模型变病态：vy、r 平滑过渡到运动学模型的值。
所有函数都按最后一维取状态 / 输入，前面的维度任意（多个方案、多个时刻一起算）。
"""
import numpy as np

NX, NU = 9, 2
S, EY, EPSI, VX, VY, R, DELTA, AX, DC = range(NX)
V_DYN_LO, V_DYN_HI = 3.0, 6.0   # m/s，低于 LO 纯运动学，高于 HI 纯动态，中间线性过渡
TAU_KIN = 0.08                  # s，运动学区里 vy、r 向运动学值收敛的时间常数


class Vehicle:
    """预测模型的车辆参数（默认 Chrono 宝马 E90：bmw_e90_identified.json 的辨识结果）。"""

    def __init__(self, m=1910.0, I=3482.0, a=1.371, b=1.386, pac_By=19.4, pac_Cy=1.3, pac_Ey=-1.0,
                 pac_D_ratio=0.95, front_scale=1.023, rear_scale=1.025, tau_delta=0.10, tau_ax=0.20, g=9.81):
        self.m, self.I, self.a, self.b = m, I, a, b
        self.By, self.Cy, self.Ey, self.D_ratio = pac_By, pac_Cy, pac_Ey, pac_D_ratio
        self.front_scale, self.rear_scale = front_scale, rear_scale
        self.tau_delta, self.tau_ax, self.g = tau_delta, tau_ax, g

    @property
    def L(self):
        return self.a + self.b

    def axle_peak(self):
        """前、后轴侧向力峰值（未乘比例因子），N：静载 × D_ratio。"""
        mg = self.m * self.g
        return self.D_ratio * mg * self.b / self.L, self.D_ratio * mg * self.a / self.L

    def pacejka(self, alpha):
        """单位峰值的 Pacejka 魔术公式侧向力（无量纲，−1 ~ 1）。"""
        Ba = self.By * alpha
        return np.sin(self.Cy * np.arctan(Ba - self.Ey * (Ba - np.arctan(Ba))))

    def slips(self, vx, vy, r, delta):
        vxe = np.maximum(vx, 1.0)
        return delta - np.arctan2(vy + self.a * r, vxe), -np.arctan2(vy - self.b * r, vxe)

    def tire_forces(self, vx, vy, r, delta):
        af, ar = self.slips(vx, vy, r, delta)
        Df, Dr = self.axle_peak()
        return Df * self.front_scale * self.pacejka(af), Dr * self.rear_scale * self.pacejka(ar)

    # ------------------------------------------------------------------ dynamics
    def deriv(self, x, u, kappa_of_s):
        s, ey, ep, vx, vy, r, de, ax, dc = (x[..., i] for i in range(NX))
        Fyf, Fyr = self.tire_forces(vx, vy, r, de)
        cd, sd = np.cos(de), np.sin(de)
        # 动态
        vx_dot = ax + r * vy - Fyf * sd / self.m
        vx_dot = np.where(vx_dot < 0.0, vx_dot * np.clip(vx / 0.5, 0.0, 1.0), vx_dot)  # 制动只能停住，不会倒车
        vy_dyn = (Fyf * cd + Fyr) / self.m - r * vx
        r_dyn = (self.a * Fyf * cd - self.b * Fyr) / self.I
        # 运动学（低速）
        beta = np.arctan(self.b / self.L * np.tan(de))
        vabs = np.maximum(vx, 0.0)
        r_k = vabs * np.cos(beta) * np.tan(de) / self.L
        vy_k = vabs * np.tan(beta)
        w = np.clip((vx - V_DYN_LO) / (V_DYN_HI - V_DYN_LO), 0.0, 1.0)
        vy_dot = w * vy_dyn + (1.0 - w) * (vy_k - vy) / TAU_KIN
        r_dot = w * r_dyn + (1.0 - w) * (r_k - r) / TAU_KIN
        # Frenet 运动学
        kap = kappa_of_s(s)
        ce, se = np.cos(ep), np.sin(ep)
        s_dot = (vx * ce - vy * se) / np.maximum(1.0 - kap * ey, 0.2)
        ey_dot = vx * se + vy * ce
        ep_dot = r - kap * s_dot
        return np.stack([s_dot, ey_dot, ep_dot, vx_dot, vy_dot, r_dot,
                         (dc - de) / self.tau_delta, (u[..., 0] - ax) / self.tau_ax, u[..., 1]], axis=-1)

    def step(self, x, u, dt, kappa_of_s):
        """RK2（中点法）积分一步。"""
        k1 = self.deriv(x, u, kappa_of_s)
        k2 = self.deriv(x + 0.5 * dt * k1, u, kappa_of_s)
        return x + dt * k2

    def lateral_accel(self, x):
        return x[..., VX] * x[..., R]
