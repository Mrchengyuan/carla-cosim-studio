"""路径跟踪控制算法：沿 CARLA 地图的车道中心线行驶（纯跟踪 pure pursuit + 定速 PI）。

道路信息全部来自 CARLA：scene["lane"]["center_rel"] 是自车前方 50 m 的车道中心线点列
（自车坐标系：x 向前、y 向左，原点在 CarSim 参考点，默认前轴中心，单位 m），
“场景信息”页默认就勾选了给算法，不用额外设置。车辆本身的状态来自 CarSim 的导出变量。

    横向  纯跟踪：在中心线上取前视距离处的点，按自行车模型算出需要的前轮转角，
          再乘方向盘传动比，得到方向盘转角（导入变量 3）。前视距离按车速取约 1 s 的行程：
          对转向滞后不敏感（纯跟踪比 Stanley 等算法稳，适合事先不知道车辆动力学的情况）。
          急弯里前、后轮走的圆不一样（内轮差），目标点往弯内挪一点，让前、后轮偏离中心线大致相等。
          传动比在运行中自动估计：方向盘转角 Steer_SW ÷ 左右前轮平均转角 Steer_L1 / Steer_R1
          （CarSim 模型里的转向传动、阿克曼都已包含），不用手填。
    路口  车道信息取的是离车最近的车道；路口里直行、转弯车道重叠，读数可能跳到另一条车道上。
          算法记住上一段中心线（用场景里自车的 X / Y / Yaw 换算到全局坐标），新读到的中心线
          和记住的对不上时继续沿记住的走，短暂没有车道信息时也一样；过了路口两者又一致，照常更新。
          遇到岔路走转弯最少的那条（场景信息就是这样取中心线的）。
    纵向  PI 定速到 TARGET_KMH；前方有弯时按侧向加速度不超过 A_LAT_MAX 提前降速。
          目标车速降低立即生效，升高按 ACCEL_MAX 慢慢升：起步、出弯加速平顺，
          路口里车道读数来回跳时也不会油门、制动来回切换。
          不看红绿灯、不避让其他车辆和行人：只做路径跟踪。

需要的导出变量：Vx（或场景里自车的 Speed）；估计传动比用 Steer_SW、Steer_L1、Steer_R1
（没有时按 STEER_RATIO_INIT）。单位跟随“CarSim 动力学”页（scene["units"]）。
返回 [油门 0~1, 制动, 方向盘转角 deg（左为正）]，与 .sim 导入变量（REPLACE）的顺序一致。

下面几个常数已按当前的 CarSim 车辆设好（换车时改）：WHEELBASE（轴距）、TARGET_KMH（目标车速）、
BRAKE_MAX（导入变量 2 的量程：主缸压力 IMP_PCON_BK 时按 MPa；制动踏板 0~1 时改成 1）。
"""
import math

WHEELBASE = 2.66           # m，轴距。从 CarSim 运行数据估计（转弯时 车速×tan(前轮转角)÷横摆角速度，按车速回归到 0），
                           # 请到 CarSim 车辆参数里核对
TARGET_KMH = 40.0          # km/h，目标车速
A_LAT_MAX = 2.0            # m/s²，弯道里允许的侧向加速度（决定过弯车速）
MIN_KMH = 10.0             # km/h，弯道降速的下限
BRAKE_MAX = 8.0            # 导入变量 2 满量程：主缸压力 IMP_PCON_BK，MPa（8 MPa 约为紧急制动）；制动踏板 0~1 时改成 1
ACCEL_MAX = 1.5            # m/s²，目标车速升高的快慢（降低不限）
STEER_RATIO_INIT = 19.0    # 方向盘 / 前轮 传动比初值（当前车辆约 19；运行中用 CarSim 导出变量自动修正）
STEER_SW_MAX = 540.0       # deg，方向盘转角限幅
STEER_RATE_MAX = 540.0     # deg/s，方向盘转速限幅（真实转向系统和驾驶员都转不了更快）
# 前视距离是最主要的调节量：越短跟得越紧（弯道切角小），太短会左右摆（CarSim 的轮胎、转向系统有滞后）。
# 1 s 在转向滞后 0.35 s、60 km/h 时仍然稳定；在真实 CarSim 模型上车身左右摆就加大，弯道切角大就减小（不要低于 0.8）。
LOOKAHEAD_TIME = 1.0       # s，前视距离 = 车速 × 这个时间（不小于 LOOKAHEAD_MIN）
LOOKAHEAD_MIN = 4.0        # m，前视距离下限（从后轴算）
LOOKAHEAD_MAX = 25.0       # m，前视距离上限
OFFTRACK_COMP = 0.25       # 内轮差补偿：目标点往弯内挪 轴距² × 曲率 × 这个系数（0 = 后轴走在中心线上）
PATH_SWITCH_M = 1.0        # m，新读到的中心线离记住的路径超过这个距离：当作车道读数跳变，沿记住的路径走
KP, KI = 0.08, 0.02        # 定速 PI（按 km/h 调的）
PRINT_EVERY = 5.0          # s，每隔多久在“输出”页打印一行状态


def _curvature(p0, p1, p2):
    """三点外接圆的曲率（左弯为正），1/m。"""
    ax, ay = p1[0] - p0[0], p1[1] - p0[1]
    bx, by = p2[0] - p1[0], p2[1] - p1[1]
    cross = ax * by - ay * bx
    d = math.hypot(ax, ay) * math.hypot(bx, by) * math.hypot(p2[0] - p0[0], p2[1] - p0[1])
    return 2.0 * cross / d if d > 1e-9 else 0.0


def _dist_to_line(p, line):
    """点到折线的最短距离，m。"""
    best = float("inf")
    for (ax, ay), (bx, by) in zip(line, line[1:]):
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        k = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
        best = min(best, math.hypot(p[0] - ax - k * dx, p[1] - ay - k * dy))
    return best


def _reach(pts):
    """点列最远一点离后轴多远，m（自车坐标，原点在前轴）。"""
    return math.hypot(pts[-1][0] + WHEELBASE, pts[-1][1]) if pts else 0.0


class Controller:
    def reset(self):
        self.integral = 0.0
        self.target = None          # km/h，限制了升速快慢的目标车速
        self.ratio = STEER_RATIO_INIT
        self.sw_last = 0.0          # deg
        self.path = []              # 记住的中心线，全局坐标 [(X, Y)]
        self.holding = False        # 正在沿记住的路径走（读数和它对不上 / 没有车道信息）
        self.told = False           # 已在“输出”页说过在沿记住的路径走
        self.agree_since = None     # 读数和路径从这时起一直一致（读数来回跳时不刷屏）
        self.on_lane = True
        self.next_print = 0.0
        self.n = 0
        self.offset_sq = 0.0
        self.offset_max = 0.0

    # ------------------------------------------------------------------ helpers
    def _units(self, scene):
        u = scene.get("units") or {}
        deg = 1.0 if u.get("angle", "deg") == "deg" else 180.0 / math.pi   # 导出的角度 → deg
        kmh = 1.0 if u.get("speed", "km/h") == "km/h" else 3.6              # 导出的车速 → km/h
        return deg, kmh

    def _pose(self, exports, scene, deg):
        """自车在全局坐标里的位置、航向 (X, Y, yaw rad)：场景里自车的 X / Y / Yaw，没有时用导出变量；都没有时 None。"""
        e = scene.get("ego") or {}
        for src, keys in ((e, ("X", "Y", "Yaw")), (exports, ("Xo", "Yo", "Yaw"))):
            if all(k in src and src[k] is not None for k in keys):
                return src[keys[0]], src[keys[1]], math.radians(src[keys[2]] * deg)
        return None

    def _learn_ratio(self, exports, deg):
        """方向盘 / 前轮 传动比：只在前轮转角够大、读数可信时更新（一阶滤波）。"""
        try:
            sw = exports["Steer_SW"] * deg
            wheel = 0.5 * (exports["Steer_L1"] + exports["Steer_R1"]) * deg
        except KeyError:
            return
        if abs(wheel) > 0.5 and sw * wheel > 0:
            r = sw / wheel
            if 5.0 < r < 40.0:
                self.ratio += 0.02 * (r - self.ratio)

    def _choose_path(self, pts, pose, lookahead, t):
        """这一帧跟哪条中心线（自车坐标）：新读到的，或记住的。"""
        if pose is None:            # 没有自车位姿：不记路径，只用这一帧的读数
            return pts
        X, Y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        kept = [((gx - X) * c + (gy - Y) * s, -(gx - X) * s + (gy - Y) * c) for gx, gy in self.path]
        i = 0                       # 丢掉后轴后面的点（留一个）
        while i + 1 < len(kept) and kept[i + 1][0] + WHEELBASE <= 0.0:
            i += 1
        kept, self.path = kept[i:], self.path[i:]
        usable = len(kept) >= 2 and _reach(kept) > lookahead + 5.0
        if pts and usable:
            # 只比较两条都覆盖到的一段（前视点附近及以内）
            span = min(lookahead + 10.0, _reach(kept) - 2.0)
            near = [p for p in pts if p[0] + WHEELBASE > 0.0 and math.hypot(p[0] + WHEELBASE, p[1]) <= span]
            jumped = any(_dist_to_line(p, kept) > PATH_SWITCH_M for p in near)
        else:
            jumped = False
        if pts and not jumped:
            if self.agree_since is None:
                self.agree_since = t
            if self.told and t - self.agree_since >= 1.0:
                print("t = %.1f s：车道读数和路径又一致了，照常更新" % t)
                self.told = False
            self.holding = False
            self.path = [(X + c * x - s * y, Y + s * x + c * y) for x, y in pts]
            return pts
        self.agree_since = None
        if usable:
            if not self.told:
                print("t = %.1f s：%s，沿记住的路径继续行驶" % (
                    t, "车道读数跳到了另一条车道（路口里常见）" if pts else "暂时没有车道信息"))
                self.told = True
            self.holding = True
            return kept
        self.holding = self.told = False
        self.path = []
        return pts

    def _target(self, pts, lookahead):
        """中心线上与后轴相距 lookahead 的点（后轴坐标，线性插值）；点不够远时取最后一个。"""
        prev = None
        for x, y in pts:
            xr = x + WHEELBASE          # 点列原点在前轴：换到后轴
            d = math.hypot(xr, y)
            if d >= lookahead and xr > 0:
                if prev is None or prev[1] >= lookahead or d <= prev[1]:
                    return xr, y
                (px, py), pd = prev
                k = (lookahead - pd) / (d - pd)
                return px + k * (xr - px), py + k * (y - py)
            prev = ((xr, y), d)
        return prev[0] if prev else None

    def _steer(self, pts, lookahead):
        """纯跟踪：前轮转角（rad，左为正）；None = 没有可用的点。"""
        tgt = self._target(pts, lookahead)
        if tgt is None:
            return None
        x, y = tgt
        n = math.hypot(x, y)
        if n < 1e-6:
            return 0.0
        # 内轮差补偿：车所在处的曲率，目标点沿垂直于视线的方向往弯内挪
        c = OFFTRACK_COMP * WHEELBASE * WHEELBASE * self._curvature_here(pts)
        x, y = x - c * y / n, y + c * x / n
        return math.atan(2.0 * WHEELBASE * y / (x * x + y * y))

    def _curvature_here(self, pts):
        """中心线在车所在处（前后轴中点附近）的曲率，1/m，左弯为正。"""
        best, kappa = None, 0.0
        for i in range(len(pts) - 2):
            d = abs(pts[i + 1][0] + WHEELBASE / 2)
            if best is None or d < best:
                best, kappa = d, _curvature(pts[i], pts[i + 1], pts[i + 2])
        return kappa

    def _curve_speed(self, pts, v_ms):
        """前方一段中心线上的最大曲率 → 允许车速（km/h）。"""
        horizon = max(15.0, v_ms * 4.0)
        kmax = 0.0
        for i in range(len(pts) - 2):
            if pts[i][0] > horizon:
                break
            kmax = max(kmax, abs(_curvature(pts[i], pts[i + 1], pts[i + 2])))
        if kmax < 1e-4:
            return TARGET_KMH
        return max(MIN_KMH, min(TARGET_KMH, math.sqrt(A_LAT_MAX / kmax) * 3.6))

    # ------------------------------------------------------------------ control
    def control(self, exports, t, dt, scene):
        deg, kmh = self._units(scene)
        v_kmh = (exports["Vx"] if "Vx" in exports else scene["ego"]["Speed"]) * kmh
        v_ms = max(0.0, v_kmh / 3.6)
        self._learn_ratio(exports, deg)
        lane = scene.get("lane")
        pts = [(p[0], p[1]) for p in ((lane or {}).get("center_rel") or [])]
        if len(pts) < 2:
            pts = []
        lookahead = min(LOOKAHEAD_MAX, max(LOOKAHEAD_MIN, LOOKAHEAD_TIME * v_ms))
        pts = self._choose_path(pts, self._pose(exports, scene, deg), lookahead, t)

        if not pts:
            # 不在车道上（停车场、开出路外）且没有记住的路径：方向盘慢慢回正，低速行驶。
            if self.on_lane:
                print("t = %.1f s：没有车道信息（不在行车道上），低速、方向盘回正" % t)
            self.on_lane = False
            target_kmh = MIN_KMH
            sw = self.sw_last * 0.95
        else:
            if not self.on_lane:
                print("t = %.1f s：回到车道" % t)
            self.on_lane = True
            delta = self._steer(pts, lookahead)   # 前轮转角，左为正
            sw = self.sw_last if delta is None else math.degrees(delta) * self.ratio
            target_kmh = self._curve_speed(pts, v_ms)
        off = (lane or {}).get("offset")
        if off is not None:
            self.n += 1
            self.offset_sq += off * off
            self.offset_max = max(self.offset_max, abs(off))
        sw = max(self.sw_last - STEER_RATE_MAX * dt, min(self.sw_last + STEER_RATE_MAX * dt, sw))
        sw = max(-STEER_SW_MAX, min(STEER_SW_MAX, sw))
        self.sw_last = sw

        # 目标车速：降低立即生效，升高每秒最多 ACCEL_MAX（从当前车速起步）
        if self.target is None:
            self.target = v_kmh
        self.target = min(target_kmh, self.target + ACCEL_MAX * 3.6 * max(dt, 0.0))
        target_kmh = self.target

        # 纵向：PI 定速；执行器饱和时不再累积积分（防积分饱和）
        err = target_kmh - v_kmh
        u = KP * err + KI * self.integral
        if 0.0 < u < 1.0 or (u >= 1.0 and err < 0) or (u <= 0.0 and err > 0):
            self.integral = max(-100.0, min(100.0, self.integral + err * dt))
        throttle = max(0.0, min(1.0, u))
        brake = max(0.0, min(1.0, -u)) * BRAKE_MAX

        if t >= self.next_print:
            self.next_print = t + PRINT_EVERY
            print("t = %.1f s  车速 %.1f km/h（目标 %.1f）  横向偏差 %s  方向盘 %.1f°  传动比 %.1f" % (
                t, v_kmh, target_kmh, "%.2f m" % off if off is not None else "—", sw, self.ratio))
        return [throttle, brake, sw]   # 方向盘转角 deg（和 python_carsim_env、示例算法一样，不随导出单位变）

    def finish(self, reason):
        if self.n:
            print("路径跟踪结束（%s）：车道中心偏差 均方根 %.2f m，最大 %.2f m；估计的方向盘传动比 %.1f" % (
                reason, math.sqrt(self.offset_sq / self.n), self.offset_max, self.ratio))
