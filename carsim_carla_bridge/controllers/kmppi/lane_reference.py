"""KMPPI 的参考时域：CARLA 车道中心线 + 恒速（代替原工程的八字参考 figure8_reference.py）。

原工程的参考是按时间走的八字（弧长恒速参数化）；这里的路是 CARLA 给的车道中心线
scene["lane"]["center_rel"]（自车坐标：x 向前、y 向左，原点在 CarSim 参考点 = 前轴中心，
每 2 m 一个点，前方约 50 m）。每个控制周期：把质心投影到中心线上得到弧长 s0，
未来第 k 步（k = 1..T）的参考点取在 s0 + v_ref·k·dt 处：

    [x, y, yaw]  中心线上该弧长处的位置和切线方向（自车坐标）
    vx           v_ref
    vy, r        由该处曲率按动态自行车稳态关系求得 —— bicycle_reference 与
                 _invert_lateral_force 逐行照搬原工程（dynamic_bicycle_reference.m）

曲率由中心线点列的切线方向对弧长求导（在 ±2 m 内平均一次）。
"""
from __future__ import annotations

import numpy as np

from kmppi_config import BicycleParams, KMPPIConfig
from prediction_model import pacejka_lateral

EXTEND_BACK = 10.0   # m，中心线第一个点在前轴处：往后延长这么多，质心（在前轴后面）才投影得上


class LaneReference:
    def __init__(self, cfg: KMPPIConfig, vehicle: BicycleParams):
        self.cfg = cfg
        self.p = vehicle
        self.v_ref = cfg.ref_speed
        # --- 后轮侧向力反查表 (dynamic_bicycle_reference.m: invert_lateral_force) ---
        p = self.p
        slip_full = np.linspace(0.0, 0.3, 6001)
        force_full = pacejka_lateral(slip_full, p.pac_By, p.pac_Cy, p.pac_Dy, p.pac_Ey)
        peak = int(np.argmax(force_full))
        self._slip_grid = slip_full[: peak + 1]
        self._force_grid = force_full[: peak + 1]

    # ------------------------------------------------------------------ 动态自行车稳态参考（原样）
    def _invert_lateral_force(self, force: np.ndarray) -> np.ndarray:
        sign = np.sign(force)
        bounded = np.minimum(np.abs(force), 0.995 * self._force_grid[-1])
        return sign * np.interp(bounded, self._force_grid, self._slip_grid)

    def bicycle_reference(self, curvature: np.ndarray, speed: np.ndarray):
        p = self.p
        speed = np.maximum(speed, 0.5)
        r_ref = speed * curvature
        F_lat = p.m * speed ** 2 * curvature
        rear_per_tire = 0.5 * (p.a / p.L) * F_lat
        alpha_r = self._invert_lateral_force(rear_per_tire)
        vy_ref = p.b * r_ref - speed * np.tan(alpha_r)
        return vy_ref, r_ref

    # ------------------------------------------------------------------ 车道中心线
    def horizon(self, center_rel, cg_xy) -> np.ndarray:
        """(T,6) 未来参考 [x, y, yaw, vx, vy, r]（自车坐标），对应未来 1..T 步。
        center_rel: 中心线点列 [[x, y], ...]；cg_xy: 质心在自车坐标里的位置。"""
        pts = np.asarray(center_rel, dtype=float)[:, :2]
        seg = np.diff(pts, axis=0)
        keep = np.hypot(seg[:, 0], seg[:, 1]) > 1e-6          # 去掉重合点
        pts = np.vstack([pts[:1], pts[1:][keep]])
        if len(pts) < 2:
            raise ValueError("车道中心线点太少")
        d0 = pts[1] - pts[0]
        pts = np.vstack([pts[0] - d0 / np.hypot(*d0) * EXTEND_BACK, pts])
        seg = np.diff(pts, axis=0)
        seg_len = np.hypot(seg[:, 0], seg[:, 1])
        s_pts = np.concatenate([[0.0], np.cumsum(seg_len)])
        heading = np.unwrap(np.arctan2(seg[:, 1], seg[:, 0]))   # 每段的方向
        # 质心投影到折线上的弧长
        c = np.asarray(cg_xy, dtype=float)
        k = np.clip(np.einsum("ij,ij->i", c - pts[:-1], seg) / seg_len ** 2, 0.0, 1.0)
        foot = pts[:-1] + k[:, None] * seg
        i = int(np.argmin(np.hypot(foot[:, 0] - c[0], foot[:, 1] - c[1])))
        s0 = s_pts[i] + k[i] * seg_len[i]
        # 参考弧长；超出中心线末端时停在末端（前方路不够长）
        s_ref = np.minimum(s0 + self.v_ref * self.cfg.dt * np.arange(1, self.cfg.T + 1), s_pts[-1])
        x = np.interp(s_ref, s_pts, pts[:, 0])
        y = np.interp(s_ref, s_pts, pts[:, 1])
        s_mid = 0.5 * (s_pts[:-1] + s_pts[1:])
        yaw = np.interp(s_ref, s_mid, heading)
        # 曲率：相邻两段方向差 / 两段中点间距，再在 ±2 m 内平均
        if len(heading) >= 2:
            kappa_v = np.diff(heading) / np.maximum(np.diff(s_mid), 1e-6)
            s_v = s_pts[1:-1]
            kappa = np.array([np.mean(kappa_v[np.abs(s_v - s) <= 2.0 + 1e-9]) if np.any(np.abs(s_v - s) <= 2.0 + 1e-9)
                              else np.interp(s, s_v, kappa_v) for s in s_ref])
        else:
            kappa = np.zeros_like(s_ref)
        speed = np.full_like(s_ref, self.v_ref)
        vy_ref, r_ref = self.bicycle_reference(kappa, speed)
        out = np.zeros((self.cfg.T, 6))
        out[:, 0], out[:, 1], out[:, 2] = x, y, np.arctan2(np.sin(yaw), np.cos(yaw))
        out[:, 3], out[:, 4], out[:, 5] = speed, vy_ref, r_ref
        return out
