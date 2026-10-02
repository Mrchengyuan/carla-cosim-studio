"""参考线：CARLA 车道中心线（scene["lane"]["center_rel"]）→ 以自车质心为原点的 Frenet 坐标。

    重采样  按弧长每 DS 米一个点，算航向、曲率（曲率再做一次滑动平均，去掉折线的锯齿）。
    外推    场景只给前方 50 m，预测时域内高速要走 100 m 以上：末端按最后一段的曲率（逐渐回零）接着画。
    投影    质心投影到参考线上：s = 0 就在那里，ey、epsi 是质心离线的横向距离和航向差。
    记忆    路口里车道读数会跳到另一条车道（直行、转弯车道重叠）。新读数和记住的路径（全局坐标）
            在前方对不上、又不是平移了大约一个车道宽（真的换了道）时，继续用记住的。
"""
import math

import numpy as np

DS = 1.0              # m，重采样间距
SMOOTH = 5            # 曲率滑动平均的点数
JUMP_M = 1.0          # m，前方读数离记住的路径超过这个：读数跳了
LANE_SHIFT_TOL = 0.6  # m，整条线平移 ≈ 一个车道宽（误差在这以内）算换道


def resample(pts, ds=DS):
    """折线按弧长重采样：返回 (N, 2)。"""
    p = np.asarray(pts, dtype=float)
    seg = np.hypot(*np.diff(p, axis=0).T)
    keep = np.concatenate([[True], seg > 1e-6])
    p = p[keep]
    if len(p) < 2:
        return p
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(p, axis=0).T))])
    n = max(2, int(s[-1] / ds) + 1)
    si = np.linspace(0.0, s[-1], n)
    return np.stack([np.interp(si, s, p[:, 0]), np.interp(si, s, p[:, 1])], axis=1)


class Path:
    """一条参考线（自车质心坐标系：x 向前、y 向左），s = 0 在质心的投影点。"""

    def __init__(self, xy, need_ahead):
        xy = resample(xy)
        if len(xy) < 3:
            raise ValueError("参考线点太少")
        d = np.diff(xy, axis=0)
        psi = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
        psi = np.concatenate([psi, psi[-1:]])
        kap = np.gradient(psi) / DS
        if len(kap) >= SMOOTH:
            kap = np.convolve(np.pad(kap, SMOOTH // 2, mode="edge"), np.ones(SMOOTH) / SMOOTH, mode="valid")
        s = np.arange(len(xy)) * DS
        # 质心投影
        s0, ey0, i0 = self._project_xy(xy, s, psi, 0.0, 0.0)
        # 末端外推：曲率在 30 m 内线性回零
        ahead = s[-1] - s0
        if ahead < need_ahead:
            n_ext = int(math.ceil((need_ahead - ahead) / DS)) + 1
            k_end = kap[-1]
            ext_k = k_end * np.clip(1.0 - np.arange(1, n_ext + 1) * DS / 30.0, 0.0, 1.0)
            ext_psi = psi[-1] + np.cumsum(ext_k) * DS
            ext_xy = xy[-1] + np.cumsum(np.stack([np.cos(ext_psi), np.sin(ext_psi)], axis=1) * DS, axis=0)
            xy = np.concatenate([xy, ext_xy])
            psi = np.concatenate([psi, ext_psi])
            kap = np.concatenate([kap, ext_k])
            s = np.arange(len(xy)) * DS
        self.xy, self.psi, self.kap = xy, psi, kap
        self.s = s - s0                      # 质心投影点 s = 0
        self.ey0 = ey0
        self.epsi0 = _wrap(0.0 - float(np.interp(0.0, self.s, psi)))

    @staticmethod
    def _project_xy(xy, s, psi, px, py):
        """点 (px, py) 投影到折线：返回 (s, 左为正的横向距离, 段号)。线外的点按首、尾两段的延长线算。"""
        a, b = xy[:-1], xy[1:]
        d = b - a
        L2 = np.maximum(np.sum(d * d, axis=1), 1e-12)
        t = ((px - a[:, 0]) * d[:, 0] + (py - a[:, 1]) * d[:, 1]) / L2
        tc = np.clip(t, 0.0, 1.0)
        tc[0] = min(t[0], 1.0)
        tc[-1] = max(t[-1], 0.0)
        qx, qy = a[:, 0] + tc * d[:, 0], a[:, 1] + tc * d[:, 1]
        dist = np.hypot(px - qx, py - qy)
        i = int(np.argmin(dist))
        L = math.sqrt(L2[i])
        lat = (d[i, 0] * (py - a[i, 1]) - d[i, 1] * (px - a[i, 0])) / L
        return float(s[i] + tc[i] * L), float(lat), i

    def project(self, px, py):
        s, lat, i = self._project_xy(self.xy, self.s, self.psi, px, py)
        return s, lat, float(self.psi[i])

    def kappa(self, s):
        return np.interp(s, self.s, self.kap)

    def heading(self, s):
        return np.interp(s, self.s, self.psi)

    def to_xy(self, s, ey):
        """Frenet (s, ey) → 质心坐标系 (x, y)。"""
        x = np.interp(s, self.s, self.xy[:, 0])
        y = np.interp(s, self.s, self.xy[:, 1])
        ps = np.interp(s, self.s, self.psi)
        return x - ey * np.sin(ps), y + ey * np.cos(ps)


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _dist_to_line(p, line):
    a, b = line[:-1], line[1:]
    d = b - a
    L2 = np.maximum(np.sum(d * d, axis=1), 1e-12)
    t = np.clip(((p[0] - a[:, 0]) * d[:, 0] + (p[1] - a[:, 1]) * d[:, 1]) / L2, 0.0, 1.0)
    return float(np.min(np.hypot(p[0] - a[:, 0] - t * d[:, 0], p[1] - a[:, 1] - t * d[:, 1])))


def _signed_offset(p, line):
    """点 p 在折线 line 的哪一侧、多远（左为正）。"""
    a, b = line[:-1], line[1:]
    d = b - a
    L2 = np.maximum(np.sum(d * d, axis=1), 1e-12)
    t = np.clip(((p[0] - a[:, 0]) * d[:, 0] + (p[1] - a[:, 1]) * d[:, 1]) / L2, 0.0, 1.0)
    dist = np.hypot(p[0] - a[:, 0] - t * d[:, 0], p[1] - a[:, 1] - t * d[:, 1])
    i = int(np.argmin(dist))
    return float((d[i, 0] * (p[1] - a[i, 1]) - d[i, 1] * (p[0] - a[i, 0])) / math.sqrt(L2[i]))


class LaneMemory:
    """记住上一条中心线（全局坐标），判断新读数是 正常 / 换道（平移一个车道宽）/ 跳变。"""

    def __init__(self):
        self.glob = None      # (N, 2) 全局坐标

    def reset(self):
        self.glob = None

    def update(self, pts_rel, pose, width):
        """pts_rel：自车坐标（参考点原点）的中心线；pose：(X, Y, yaw rad) 或 None。
        返回 (要用的点列（自车坐标）, 事件)：事件 None / "hold"（沿记住的走）/ +1 / −1（换到左 / 右边的车道）。"""
        if pose is None:
            return pts_rel, None
        X, Y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)

        def to_glob(p):
            p = np.asarray(p, dtype=float)
            return np.stack([X + c * p[:, 0] - s * p[:, 1], Y + s * p[:, 0] + c * p[:, 1]], axis=1)

        def to_rel(g):
            dx, dy = g[:, 0] - X, g[:, 1] - Y
            return np.stack([c * dx + s * dy, -s * dx + c * dy], axis=1)

        kept = None
        if self.glob is not None:
            rel = to_rel(self.glob)
            i = 0
            while i + 1 < len(rel) and rel[i + 1, 0] < -8.0:   # 丢掉车后 8 m 以外的
                i += 1
            self.glob = self.glob[i:]
            rel = rel[i:]
            kept = rel if len(rel) >= 3 and rel[-1, 0] > 15.0 else None
        if pts_rel is None or len(pts_rel) < 2:
            if kept is not None:
                return kept, "hold"
            self.glob = None
            return None, None
        new = np.asarray(pts_rel, dtype=float)
        event = None
        if kept is not None:
            near = new[(new[:, 0] > -2.0) & (new[:, 0] < min(25.0, kept[-1, 0] - 2.0))]
            if len(near):
                offs = np.array([_signed_offset(p, kept) for p in near])
                if np.max(np.abs(offs)) > JUMP_M:
                    if width and np.all(np.abs(np.abs(offs) - width) < LANE_SHIFT_TOL) and \
                            np.all(np.sign(offs) == np.sign(offs[0])):
                        event = 1 if offs[0] > 0 else -1          # 新中心线在左边一个车道宽：换到了左边车道
                    else:
                        return kept, "hold"
        self.glob = to_glob(new)
        return new, event


class Obstacle:
    """一个障碍物在 Frenet 下的当前状态和常速预测。"""
    __slots__ = ("s", "d", "vs", "vd", "half_s", "half_d", "kind", "id", "speed")

    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


def obstacles_frenet(objs, path, a_front, yaw_ego, conv_speed, conv_angle, ego_v_ref):
    """场景障碍物 → [Obstacle]（参考线坐标）。
    objs：scene["objects"]；a_front：质心到参考点（前轴）距离（点从参考点坐标换到质心坐标：x + a）；
    速度优先用全局速度旋转到自车坐标（不依赖 CARLA 给的自车速度），没有时用 rel_vx/rel_vy + 自车速度。"""
    out = []
    for o in objs or []:
        if "rel_x" not in o or "rel_y" not in o:
            continue
        px, py = float(o["rel_x"]) + a_front, float(o["rel_y"])
        if px < -30.0 or px > 160.0:
            continue
        s, d, psi = path.project(px, py)
        if "Vx_global" in o and "Vy_global" in o and o["Vx_global"] is not None:
            vgx, vgy = float(o["Vx_global"]) * conv_speed, float(o["Vy_global"]) * conv_speed
            c, sn = math.cos(yaw_ego), math.sin(yaw_ego)
            vx, vy = c * vgx + sn * vgy, -sn * vgx + c * vgy
        else:
            vx = float(o.get("rel_vx", 0.0)) * conv_speed + ego_v_ref[0]
            vy = float(o.get("rel_vy", 0.0)) * conv_speed + ego_v_ref[1]
        if o.get("type") == "static" or o.get("parked"):
            vx = vy = 0.0
        vs = vx * math.cos(psi) + vy * math.sin(psi)
        vd = -vx * math.sin(psi) + vy * math.cos(psi)
        L, W = float(o.get("length", 4.5)), float(o.get("width", 1.9))
        phi = math.radians(float(o.get("rel_yaw", 0.0)) * conv_angle) - psi
        hs = abs(L / 2 * math.cos(phi)) + abs(W / 2 * math.sin(phi))
        hd = abs(L / 2 * math.sin(phi)) + abs(W / 2 * math.cos(phi))
        out.append(Obstacle(s=s, d=d, vs=vs, vd=vd, half_s=hs, half_d=hd, kind=o.get("type", "vehicle"),
                            id=o.get("id"), speed=math.hypot(vx, vy)))
    return out
