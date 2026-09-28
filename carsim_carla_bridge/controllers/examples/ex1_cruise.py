"""例 1：定速 30 km/h 直行，方向盘不动。"""


class Controller:
    def reset(self):
        self.integral = 0.0                          # 每次运行开始时清零

    def control(self, exports, t, dt, scene):
        v = exports["Vx"]                            # 车速，km/h
        err = 30.0 - v                               # 目标 30 km/h
        self.integral = min(max(self.integral + err * dt, -100.0), 100.0)
        u = 0.08 * err + 0.02 * self.integral        # PI 控制
        throttle = min(max(u, 0.0), 1.0)             # 油门 0~1
        brake = min(max(-u, 0.0), 1.0) * 8.0         # 制动：主缸压力 0~8 MPa
        steer = 0.0                                  # 方向盘转角，度，左为正
        return [throttle, brake, steer]
