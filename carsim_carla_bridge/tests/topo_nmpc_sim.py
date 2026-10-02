"""Offline closed-loop harness for controllers/topo_nmpc (tests/test_offline_topo_nmpc.py): a multi-lane road,
a plant that is NOT the prediction model, scenes in the platform format.

Plant: dynamic bicycle at the CG with other Pacejka tyres (B 16, C 1.4, 10 % less grip), the steering
command delayed STEER_DELAY then a second-order response, ax through a first-order lag plus drag.
Exports like the Chrono BMW stand-in: reference point = front axle centre, Vx / Vy km/h, AVz deg/s,
Yaw deg, Steer_L1 / R1 deg. Objects reach the scene within 50 m of the ego (as in CARLA).
"""
import math

import numpy as np

W = 3.5


class Road:
    """道路基准线（车道 0 的中心线）按曲率分段：segments = [(长度 m, 曲率 1/m)]；lanes = (最右车道号, 最左车道号)。"""

    def __init__(self, segments, lanes=(-1, 2), width=W, ds=0.5):
        k = []
        for L, kap in segments:
            k += [kap] * int(round(L / ds))
        k = np.array(k)
        psi = np.concatenate([[0.0], np.cumsum(k[:-1] * ds)])
        xy = np.concatenate([[[0.0, 0.0]], np.cumsum(np.stack([np.cos(psi), np.sin(psi)], 1) * ds, 0)[:-1]])
        self.xy, self.psi, self.kap, self.ds = xy, psi, k, ds
        self.s = np.arange(len(k)) * ds
        self.lanes, self.width = lanes, width

    def point(self, s, d):
        x = np.interp(s, self.s, self.xy[:, 0])
        y = np.interp(s, self.s, self.xy[:, 1])
        p = np.interp(s, self.s, self.psi)
        return x - d * np.sin(p), y + d * np.cos(p), p

    def project(self, x, y, s_guess):
        lo = max(0, int((s_guess - 30) / self.ds))
        hi = min(len(self.s), int((s_guess + 30) / self.ds) + 1)
        P = self.xy[lo:hi]
        i = lo + int(np.argmin(np.hypot(P[:, 0] - x, P[:, 1] - y)))
        i = min(max(i, 0), len(self.s) - 2)
        p = self.psi[i]
        dx, dy = x - self.xy[i, 0], y - self.xy[i, 1]
        s = self.s[i] + dx * math.cos(p) + dy * math.sin(p)
        d = -dx * math.sin(p) + dy * math.cos(p)
        return s, d


class Plant:
    def __init__(self, x, y, yaw, v, m=1910.0, I=3482.0, a=1.371, b=1.386, B=16.0, C=1.4, D_scale=0.9,
                 steer_delay=0.05, steer_wn=18.0, tau_ax=0.25):
        self.X, self.Y, self.yaw = x, y, yaw
        self.vx, self.vy, self.r = v, 0.0, 0.0
        self.m, self.I, self.a, self.b = m, I, a, b
        self.B, self.C, self.Ds = B, C, D_scale
        self.delta, self.delta_dot = 0.0, 0.0
        self.ax = 0.0
        self.wn, self.tau_ax = steer_wn, tau_ax
        self.queue = [0.0] * max(1, int(round(steer_delay / 0.005)))
        self.t = 0.0

    def tire(self, alpha, Fz):
        return self.Ds * Fz * math.sin(self.C * math.atan(self.B * alpha))

    def step(self, ax_cmd, delta_cmd, dt, h=0.005):
        for _ in range(int(round(dt / h))):
            self.queue.append(delta_cmd)
            dcmd = self.queue.pop(0)
            # 转向：二阶（阻尼比 0.8）
            acc = self.wn ** 2 * (dcmd - self.delta) - 2 * 0.8 * self.wn * self.delta_dot
            self.delta_dot += acc * h
            self.delta += self.delta_dot * h
            self.ax += (ax_cmd - self.ax) / self.tau_ax * h
            g = 9.81
            L = self.a + self.b
            vxe = max(self.vx, 1.0)
            af = self.delta - math.atan2(self.vy + self.a * self.r, vxe)
            ar = -math.atan2(self.vy - self.b * self.r, vxe)
            Fyf = self.tire(af, self.m * g * self.b / L)
            Fyr = self.tire(ar, self.m * g * self.a / L)
            drag = 0.4 * self.vx ** 2 / self.m + 0.1
            vx_dot = self.ax - drag * (self.vx > 0.1) + self.r * self.vy - Fyf * math.sin(self.delta) / self.m
            if self.vx < 0.5 and vx_dot < 0:
                vx_dot *= max(self.vx, 0.0) / 0.5
            if self.vx > 3.0:
                vy_dot = (Fyf * math.cos(self.delta) + Fyr) / self.m - self.r * self.vx
                r_dot = (self.a * Fyf * math.cos(self.delta) - self.b * Fyr) / self.I
            else:   # 低速：运动学
                rk = self.vx * math.tan(self.delta) / L
                vy_dot = (rk * self.b - self.vy) / 0.05
                r_dot = (rk - self.r) / 0.05
            self.vx = max(0.0, self.vx + vx_dot * h)
            self.vy += vy_dot * h
            self.r += r_dot * h
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            self.X += (c * self.vx - s * self.vy) * h
            self.Y += (s * self.vx + c * self.vy) * h
            self.yaw += self.r * h
            self.t += h

    def front_axle(self):
        return self.X + self.a * math.cos(self.yaw), self.Y + self.a * math.sin(self.yaw)

    def exports(self):
        fx, fy = self.front_axle()
        return {"Xo": fx, "Yo": fy, "Yaw": math.degrees(self.yaw), "Vx": self.vx * 3.6,
                "Vy": (self.vy + self.a * self.r) * 3.6, "AVz": math.degrees(self.r),
                "Steer_L1": math.degrees(self.delta), "Steer_R1": math.degrees(self.delta)}


class Actor:
    """沿车道走的目标：lane 车道号、s0 起点、v 车速 m/s；brake_at：自车离它多近时开始以 brake m/s² 刹停；
    cut：(触发距离, 目标车道, 用时 s)。static=True：锥桶（d 横向位置直接给）。walker：(触发距离, 横向速度)。"""

    def __init__(self, s0, lane=0, v=0.0, d=None, kind="vehicle", length=4.8, width=1.9, brake_at=None,
                 brake=0.0, cut=None, walk=None):
        self.s, self.v = s0, v
        self.d = lane * W if d is None else d
        self.kind, self.length, self.width = kind, length, width
        self.brake_at, self.brake, self.cut, self.walk = brake_at, brake, cut, walk
        self.braking = False
        self.cut_t = None
        self.d0 = self.d
        self.vd = 0.0
        self.id = id(self)

    def step(self, dt, ego_s):
        if self.brake_at is not None and self.s - ego_s < self.brake_at:
            self.braking = True
        if self.braking:
            self.v = max(0.0, self.v - self.brake * dt)
        self.vd = 0.0
        if self.cut is not None and self.cut_t is None and self.s - ego_s < self.cut[0]:
            self.cut_t = 0.0
        if self.cut_t is not None and self.cut_t < self.cut[2]:
            self.cut_t += dt
            target = self.cut[1] * W
            self.vd = (target - self.d0) / self.cut[2]
        if self.walk is not None and self.s - ego_s < self.walk[0]:
            self.vd = self.walk[1]
        self.d += self.vd * dt
        self.s += self.v * dt


class Sim:
    def __init__(self, road, start_lane=0, v0=20.0, actors=(), light=None, s0=5.0):
        self.road = road
        x, y, p = road.point(s0, start_lane * W)
        # Plant 的位置是质心：前轴在 s0 处
        self.plant = Plant(x - 1.371 * math.cos(p), y - 1.371 * math.sin(p), p, v0)
        self.actors = list(actors)
        self.light = light            # (s 停止线, [(t0, t1, state)])
        self.s_ego = s0
        self.t = 0.0
        self.log = []

    def ego_frenet(self):
        fx, fy = self.plant.front_axle()
        s, d = self.road.project(fx, fy, self.s_ego)
        self.s_ego = s
        return s, d

    def lane_of(self, d):
        lo, hi = self.road.lanes
        return int(min(hi, max(lo, round(d / W))))

    def scene(self):
        road = self.road
        s, d = self.ego_frenet()
        ln = self.lane_of(d)
        fx, fy = self.plant.front_axle()
        yaw = self.plant.yaw
        c, sn = math.cos(yaw), math.sin(yaw)

        def rel(x, y):
            dx, dy = x - fx, y - fy
            return c * dx + sn * dy, -sn * dx + c * dy

        ss = s + np.arange(0, 51, 2.0)
        cx, cy, _ = road.point(ss, ln * W)
        center = [list(rel(x, y)) for x, y in zip(cx, cy)]
        _, _, p_here = road.point(s, ln * W)
        lo, hi = road.lanes
        lane = {"width": W, "offset": d - ln * W, "heading_err": math.degrees(yaw - p_here),
                "center_rel": center, "left_lane": "same" if ln < hi else "none",
                "right_lane": "same" if ln > lo else "none", "in_junction": False,
                "light_state": None, "light_dist": None}
        if self.light is not None:
            sl, phases = self.light
            st = next((p[2] for p in phases if p[0] <= self.t < p[1]), "green")
            if 0 < sl - s < 50:
                lane["light_state"], lane["light_dist"] = st, sl - s
        objs = []
        for a in self.actors:
            if math.hypot(a.s - s, a.d - d) > 50.0:   # 场景信息：50 m 内
                continue
            ax_, ay_, pa = road.point(a.s, a.d)
            rx, ry = rel(ax_, ay_)
            vgx = a.v * math.cos(pa) - a.vd * math.sin(pa)
            vgy = a.v * math.sin(pa) + a.vd * math.cos(pa)
            gap = max(0.0, math.hypot(rx, ry) - 3.0)
            objs.append({"id": a.id, "type": a.kind, "rel_x": rx, "rel_y": ry, "rel_yaw": math.degrees(pa - yaw),
                         "length": a.length, "width": a.width, "Vx_global": vgx * 3.6, "Vy_global": vgy * 3.6,
                         "gap": gap, "dist": math.hypot(rx, ry)})
        ego = {"X": fx, "Y": fy, "Yaw": math.degrees(yaw), "Speed": self.plant.vx * 3.6, "length": 4.6, "width": 1.85}
        return {"units": {"angle": "deg", "speed": "km/h", "rate": "deg/s"}, "ego": ego, "lane": lane, "objects": objs}

    def clearance(self):
        """自车包围盒到每个目标包围盒的最小距离（粗略：都按沿道路方向的矩形），m。"""
        s, d = self.ego_frenet()
        s_c = s - 1.371   # 质心大约
        best = float("inf")
        for a in self.actors:
            ds = abs(a.s - s_c) - (a.length + 4.6) / 2
            dd = abs(a.d - d) - (a.width + 1.85) / 2
            best = min(best, max(ds, dd) if ds > 0 or dd > 0 else min(ds, dd))
        return best

    def run(self, ctl, T, dt=0.05):
        ctl.reset()
        out = None
        while self.t < T:
            sc = self.scene()
            ex = self.plant.exports()
            out = ctl.control(ex, self.t, dt, sc)
            self.plant.step(out[0], out[1], dt)
            for a in self.actors:
                a.step(dt, self.s_ego)
            self.t += dt
            s, d = self.ego_frenet()
            self.log.append({"t": self.t, "s": s, "d": d, "v": self.plant.vx, "ax": out[0], "delta": out[1],
                             "clear": self.clearance(), "lane": self.lane_of(d),
                             "ay": self.plant.vx * self.plant.r})
        ctl.finish("达到时长")
        return self.log
