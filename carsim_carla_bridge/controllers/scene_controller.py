"""用场景信息的控制算法示例：沿车道行驶，前方有车或障碍物就跟车 / 停车。

control() 写成 4 个参数，就会每帧收到 scene（周围目标、前方车道、可选的
传感器数据），不写第 4 个参数的算法照旧只收到前 3 个。

scene 的内容（坐标都在自车坐标系：原点 = 自车包围盒中心的地面，x 向前、
y 向左、z 向上，单位 m、m/s、deg）：

    scene["ego"]         {"speed", "length", "width", "height"}
    scene["objects"]     周围目标，由近到远，每个是
                         {"id", "type": "vehicle"/"walker"/"static", "type_id",
                          "moving", "x", "y", "z", "yaw", "length", "width", "height",
                          "vx", "vy"（相对自车的速度）, "speed", "distance"}
    scene["lane"]        {"center": [[x, y], ...] 前方车道中心线, "width",
                          "offset" 自车在车道中心左侧多少 m, "heading_error",
                          "left_marking", "right_marking", "speed_limit" km/h,
                          "traffic_light": "red"/"yellow"/"green"/None, "in_junction"}
    scene["sensors"]     {传感器名: {"type", "frame", "data"}}（“场景信息”页打开“传感器数据”时才有）
    scene["collisions"]  这一帧和自车包围盒重叠的目标

界面「场景信息」页可以设置范围、要哪些信息，运行时底部「场景」标签显示算法
这一帧收到的目标。
"""
import math


class Controller:
    TARGET_KMH = 30.0     # 巡航车速
    WHEELBASE = 2.9       # m，换成你的车
    STEER_RATIO = 16.0    # 方向盘角 / 前轮角，换成你的车
    MIN_GAP = 6.0         # m，停车时与前车保持的距离（车身之间）
    TIME_GAP = 1.5        # s

    def reset(self):
        self.integral = 0.0

    def control(self, exports, t, dt, scene):
        v = exports["Vx"] / 3.6  # m/s
        lane = scene.get("lane")

        # 横向：纯跟踪车道中心线。
        steer_sw = 0.0
        if lane and len(lane["center"]) >= 2:
            look = max(5.0, 0.8 * v)
            target = next((p for p in lane["center"] if math.hypot(p[0], p[1]) >= look), lane["center"][-1])
            ld2 = max(1.0, target[0] ** 2 + target[1] ** 2)
            delta = math.atan(self.WHEELBASE * 2.0 * target[1] / ld2)  # + = 左
            steer_sw = math.degrees(delta) * self.STEER_RATIO

        # 纵向：本车道前方最近的目标。
        half = scene["ego"]["width"] / 2 + 0.3
        lead = None
        for o in scene["objects"]:
            if o["x"] <= 0 or o["z"] - o["height"] / 2 > scene["ego"]["height"]:
                continue  # behind, or high above the car
            lane_y = self._lane_y(lane, o["x"]) if lane else 0.0
            if abs(o["y"] - lane_y) < half + o["width"] / 2:
                lead = o
                break
        v_set = self.TARGET_KMH / 3.6
        if lane and lane.get("traffic_light") == "red":
            v_set = 0.0
        a_cmd = 0.6 * (v_set - v)
        if lead is not None:
            gap = lead["x"] - lead["length"] / 2 - scene["ego"]["length"] / 2
            want = self.MIN_GAP + self.TIME_GAP * v
            a_follow = 0.4 * (gap - want) + 0.9 * lead["vx"]
            a_cmd = min(a_cmd, a_follow)
            if gap < self.MIN_GAP * 0.5:
                a_cmd = min(a_cmd, -6.0)
        throttle = max(0.0, min(1.0, a_cmd / 3.0))
        brake = max(0.0, min(1.0, -a_cmd / 8.0))
        return [throttle, brake, steer_sw]

    @staticmethod
    def _lane_y(lane, x):
        pts = lane["center"]
        best = min(pts, key=lambda p: abs(p[0] - x))
        return best[1] if abs(best[0] - x) < 5.0 else pts[-1][1]
