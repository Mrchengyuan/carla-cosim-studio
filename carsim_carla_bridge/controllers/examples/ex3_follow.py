"""例 3：沿车道行驶 + 跟车：前面有车就按 1.5 s 车距减速。"""
import math

WHEELBASE = 2.66
STEER_RATIO = 19.0
LOOKAHEAD = 8.0
CRUISE_KMH = 40.0     # 没有前车时的车速
TIME_GAP = 1.5        # 期望车距 = 5 m + 车速 × 1.5 s


class Controller:
    def reset(self):
        self.integral = 0.0

    def control(self, exports, t, dt, scene):
        v = exports["Vx"]                            # km/h

        # ---- 找本车道正前方最近的车 ----
        # objects 已按距离从近到远排好；rel_x 前为正，rel_y 左为正，gap 是两车包围盒的间距
        front = [o for o in scene["objects"] if o["rel_x"] > 0 and abs(o["rel_y"]) < 1.8]
        target = CRUISE_KMH
        if front:
            gap = front[0]["gap"]                    # m
            want = 5.0 + TIME_GAP * v / 3.6          # 期望车距，m
            # 车距比期望小就降低目标车速（车距每差 1 m 降 3 km/h），最低 0
            target = min(CRUISE_KMH, max(0.0, v + 3.0 * (gap - want)))

        # ---- 纵向：PI 跟踪目标车速 ----
        err = target - v
        self.integral = min(max(self.integral + err * dt, -100.0), 100.0)
        u = 0.08 * err + 0.02 * self.integral
        throttle = min(max(u, 0.0), 1.0)
        brake = min(max(-u, 0.0), 1.0) * 8.0

        # ---- 横向：同例 2 ----
        lane = scene["lane"]
        if lane is None:
            return [0.0, 2.0, 0.0]
        pts = [(px + WHEELBASE, py) for px, py in lane["center_rel"]]
        x, y = next(((px, py) for px, py in pts if px > 0 and math.hypot(px, py) >= LOOKAHEAD), pts[-1])
        steer = math.degrees(math.atan2(2.0 * WHEELBASE * y, x * x + y * y)) * STEER_RATIO
        return [throttle, brake, min(max(steer, -540.0), 540.0)]
