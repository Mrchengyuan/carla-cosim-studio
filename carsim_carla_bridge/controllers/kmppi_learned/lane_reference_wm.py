"""KMPPI（学习世界模型版）的参考时域：CARLA 车道中心线 + 恒速。

和 ../kmppi/lane_reference.py 是同一套几何（中心线投影、按弧长取未来 T 步的参考点、曲率），
只有横向速度参考 vy_ref 不同：原工程的学习模型版（kmppi_dream/figure8_reference.py）不用物理自行车，
vy_ref 是查表 vy_ref(κ)——表由世界模型自己的稳态关系给出（WorldModel.steady_state_vy_table：
对每个曲率二分找到使稳态横摆角速度 r = v·κ 的恒定前轮转角，记下稳态 vy），r_ref = v·κ 与模型无关。

    [x, y, yaw]  中心线上该弧长处的位置和切线方向（自车坐标：x 向前、y 向左）
    vx           v_ref
    vy, r        vy_ref(κ)（查表）、v_ref·κ
"""
from __future__ import annotations

import numpy as np

EXTEND_BACK = 10.0   # m，中心线第一个点在前轴处：往后延长这么多，质心（在前轴后面）才投影得上


class LaneReference:
    def __init__(self, cfg, kappa_grid, vy_grid):
        """cfg：dream_config.KMPPIConfig；kappa_grid / vy_grid：世界模型的稳态表（参考车速下）。"""
        self.cfg = cfg
        self.v_ref = cfg.ref_speed
        order = np.argsort(kappa_grid)
        self._vy_kappa = np.asarray(kappa_grid, dtype=float)[order]
        self._vy_val = np.asarray(vy_grid, dtype=float)[order]

    def vy_r_reference(self, curvature, speed):
        """和 figure8_reference.Figure8Reference.vy_r_reference 一样：r = v·κ，vy 查表。"""
        return np.interp(curvature, self._vy_kappa, self._vy_val), speed * curvature

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
        vy_ref, r_ref = self.vy_r_reference(kappa, speed)
        out = np.zeros((self.cfg.T, 6))
        out[:, 0], out[:, 1], out[:, 2] = x, y, np.arctan2(np.sin(yaw), np.cos(yaw))
        out[:, 3], out[:, 4], out[:, 5] = speed, vy_ref, r_ref
        return out
