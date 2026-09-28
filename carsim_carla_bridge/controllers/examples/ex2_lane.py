"""例 2：沿车道行驶（简化版纯跟踪），定速 30 km/h。"""
import math

WHEELBASE = 2.66      # 轴距，m
STEER_RATIO = 19.0    # 方向盘转角 / 前轮转角
LOOKAHEAD = 8.0       # 前视距离，m


class Controller:
    def reset(self):
        self.integral = 0.0

    def control(self, exports, t, dt, scene):
        # ---- 纵向：定速 30 km/h ----
        v = exports["Vx"]
        err = 30.0 - v
        self.integral = min(max(self.integral + err * dt, -100.0), 100.0)
        u = 0.08 * err + 0.02 * self.integral
        throttle = min(max(u, 0.0), 1.0)
        brake = min(max(-u, 0.0), 1.0) * 8.0

        # ---- 横向：瞄准车道中心线上、离后轴 8 m 的点（纯跟踪） ----
        lane = scene["lane"]
        if lane is None:                             # 不在行车道上（停车场、开出路外）
            return [0.0, 2.0, 0.0]                   # 松油门、轻踩制动、方向盘回正
        # center_rel 的原点在前轴；纯跟踪从后轴算，所以 x 加一个轴距
        pts = [(px + WHEELBASE, py) for px, py in lane["center_rel"]]
        x, y = next(((px, py) for px, py in pts if px > 0 and math.hypot(px, py) >= LOOKAHEAD), pts[-1])
        delta = math.atan2(2.0 * WHEELBASE * y, x * x + y * y)   # 前轮转角，rad，左为正
        steer = math.degrees(delta) * STEER_RATIO                # 方向盘转角，度
        steer = min(max(steer, -540.0), 540.0)
        return [throttle, brake, steer]
