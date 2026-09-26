"""用场景信息的控制算法示例：沿车道行驶，前方有车或行人就跟车 / 停车。

control() 写成 4 个参数，就会每帧收到 scene（一个字典），只写 3 个参数的算法
照旧只收到前 3 个。scene 里每个键的含义、单位见 docs/场景与数据接口.md。

一切按 CarSim：自车坐标系原点在 CarSim 参考点（默认前轴中心的地面），
x 向前、y 向左（左为正）；速度 km/h、角度 deg（跟随“CarSim 动力学”页的单位）。

这个示例只用“场景信息”页默认勾选的量：
    障碍物  rel_x, rel_y（相对位置 m）, rel_vx（相对纵向速度）, gap（包围盒间距 m）
    车道    center_rel（前方车道中心线）
"""
import math


class Controller:
    TARGET_KMH = 30.0     # 巡航车速
    WHEELBASE = 2.9       # m，换成你的车
    STEER_RATIO = 16.0    # 方向盘角 / 前轮角，换成你的车
    MIN_GAP = 6.0         # m，停车时与前车保持的距离（车身之间）
    TIME_GAP = 1.5        # s
    LANE_HALF = 1.8       # m，判断“在本车道”的横向范围

    def reset(self):
        pass

    def control(self, exports, t, dt, scene):
        v = exports["Vx"] / 3.6  # m/s（CarSim 的 Vx 是 km/h）
        lane = scene.get("lane")
        center = lane.get("center_rel") if lane else None

        # 横向：纯跟踪车道中心线（自车坐标系，y 左为正）。
        steer_sw = 0.0
        if center and len(center) >= 2:
            look = max(5.0, 0.8 * v)
            tx, ty = next((p for p in center if math.hypot(p[0], p[1]) >= look), center[-1])
            delta = math.atan(self.WHEELBASE * 2.0 * ty / max(1.0, tx * tx + ty * ty))  # + = 左
            steer_sw = math.degrees(delta) * self.STEER_RATIO

        # 纵向：本车道前方最近的目标（列表已按距离从近到远排好）。
        lead = None
        for o in scene["objects"]:
            if o["rel_x"] <= 0:
                continue
            lane_y = self._lane_y(center, o["rel_x"]) if center else 0.0
            if abs(o["rel_y"] - lane_y) < self.LANE_HALF:
                lead = o
                break
        a_cmd = 0.6 * (self.TARGET_KMH / 3.6 - v)
        if lead is not None:
            gap = lead["gap"]
            want = self.MIN_GAP + self.TIME_GAP * v
            a_follow = 0.4 * (gap - want) + 0.9 * lead["rel_vx"] / 3.6
            a_cmd = min(a_cmd, a_follow)
            if gap < self.MIN_GAP * 0.5:
                a_cmd = min(a_cmd, -6.0)
        throttle = max(0.0, min(1.0, a_cmd / 3.0))
        brake = max(0.0, min(1.0, -a_cmd / 8.0))
        return [throttle, brake, steer_sw]

    @staticmethod
    def _lane_y(center, x):
        best = min(center, key=lambda p: abs(p[0] - x))
        return best[1] if abs(best[0] - x) < 5.0 else center[-1][1]
