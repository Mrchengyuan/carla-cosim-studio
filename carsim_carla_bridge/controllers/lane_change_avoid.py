"""避障示例：沿车道行驶，前方车道被锥桶、护栏、停着的车挡住时换道绕过去，过去以后再换回原车道。

配合“测试场景”页的高速施工封道用（Town04 高速起点 + 任一预设）。路径跟踪、车速控制都沿用
path_follower.py（同一个文件夹里，要一起复制），这里只决定“开在哪条车道上”：

    换道  把 path_follower 跟踪的车道中心线整体往左 / 右平移，平移量在 LC_TIME 秒内从 0 变到
          一个车道宽，纯跟踪自然就开出一条平顺的换道轨迹。车越过车道线后，场景里的车道信息
          （scene["lane"]）换成新车道，平移量随之减去一个车道宽，所以轨迹是连续的。
    判断  障碍物按离车道中心线的横向距离分到各条车道上。静止的障碍物（锥桶、护栏、导向牌、
          停着的车、站着的行人）挡住本车道时才换道；换道前沿“计划的换道轨迹”逐点检查：
          轨迹两侧各 半个车宽 + CLEARANCE 以内不能有任何东西（渐变段的锥桶、封闭段两边的锥桶、
          旁边车道的车都算），旁边车道后方也不能有更快的车开过来。左右都行时选能一直开得更远的那条。
    回来  原车道从车尾到前方 50 m 都空了、换回去的轨迹也碰不到东西（封闭段边上那排锥桶还在时
          就碰得到），就一条一条换回原车道。
    减速  沿计划轨迹前方有东西（左右都换不了、来不及换、前车慢）时，按包围盒间距 gap 限速：
          能在离它 STOP_GAP 处以 A_STOP 的减速度停下；前面是开着的车时再加上它的车速（跟车）。

不看红绿灯。高速上的施工封道之类可以；路口里车道信息会跳，路口里不开始换道。

要在“场景信息”页勾选（给算法）：
    车道    width、offset、center_rel（默认已勾），left_lane、right_lane（默认没勾：要勾上，
            用来知道左 / 右边有没有同向车道）
    障碍物  type、rel_x、rel_y、rel_vx、gap（默认已勾）；种类里勾上“施工锥桶 / 护栏”（默认已勾）
    自车    width（默认已勾）
车辆参数（轴距、目标车速、制动量程等）在 path_follower.py 开头改。
"""
import math

import path_follower as pf

LC_TIME = 3.0          # s，换一次道用的时间（平移量从 0 到一个车道宽）
CLEARANCE = 0.5        # m，车身两侧到障碍物中心至少留多少（含锥桶自身的半径）
REAR = 6.0             # m，换道时参考点后方多远以内也要没有东西（车身后半段也会扫过）
BEHIND_CHECK = 40.0    # m，换道时看后方多远的来车
BEHIND_TTC = 4.0       # s，后方来车几秒内会追上就不换
AHEAD = 50.0           # m，看多远（场景信息给的就是 50 m 内的障碍物）
STILL_MS = 1.0         # m/s，比这慢的障碍物算静止
A_STOP = 3.0           # m/s²，前方有东西时按这个减速度停车
STOP_GAP = 5.0         # m，停在离障碍物多远处


def _lateral(pts, x, y):
    """点 (x, y) 离中心线 pts 的横向距离（左为正），m：取最近的一段，线外的点按首、尾两段延长线算。"""
    best, lat = None, y
    n = len(pts)
    for i in range(n - 1):
        (ax, ay), (bx, by) = pts[i], pts[i + 1]
        dx, dy = bx - ax, by - ay
        L = math.hypot(dx, dy)
        if L < 1e-6:
            continue
        k = ((x - ax) * dx + (y - ay) * dy) / (L * L)
        kc = k if (i == 0 and k < 0) or (i == n - 2 and k > 1) else max(0.0, min(1.0, k))
        d = math.hypot(x - ax - kc * dx, y - ay - kc * dy)
        if best is None or d < best:
            best, lat = d, (dx * (y - ay) - dy * (x - ax)) / L
    return lat


def _shift(pts, s):
    """中心线整体往左平移 s（m，右为负）。"""
    if abs(s) < 1e-6:
        return pts
    out = []
    for i, (x, y) in enumerate(pts):
        (ax, ay), (bx, by) = pts[max(0, i - 1)], pts[min(len(pts) - 1, i + 1)]
        tx, ty = bx - ax, by - ay
        L = math.hypot(tx, ty) or 1.0
        out.append((x - s * ty / L, y + s * tx / L))
    return out


def _ease(u):
    u = max(0.0, min(1.0, u))
    return u * u * (3.0 - 2.0 * u)


class Controller(pf.Controller):
    def reset(self):
        super().reset()
        self.shift = 0.0          # m，现在跟踪的线相对场景车道中心的平移（左为正）
        self.goal = 0.0           # m，要平移到多少（整数个车道宽）
        self.home = 0             # 原车道相对场景车道（车现在所在的）：+1 = 在左边一条，-1 = 右边一条
        self.last_offset = None
        self.last_width = None
        self.cap_kmh = None       # 这一帧因前方障碍物限的车速
        self.stuck_told = False
        self.changes = 0

    # ------------------------------------------------------------------ 障碍物
    def _objects(self, scene, pts, v_ms, kmh):
        """[(沿路距离 x, 离中心线横向 d, gap, 自身纵向车速 m/s, 是否静止)]，只要车尾后 REAR 到前方 AHEAD 的。"""
        out = []
        for o in scene.get("objects") or ():
            x = o["rel_x"]
            if not -REAR - pf.WHEELBASE <= x <= AHEAD:
                continue
            v_obj = o["rel_vx"] / kmh / 3.6 + v_ms   # 导出单位 → km/h → m/s，加回自车车速
            out.append((x, _lateral(pts, x, o["rel_y"]), o["gap"], v_obj, abs(v_obj) < STILL_MS))
        return out

    def _plan(self, x, c0, c1, v_ms):
        """从横向位置 c0 换到 c1 的计划轨迹在前方 x 处的横向位置（纯跟踪大约晚 1 s 跟上）。"""
        L = max(15.0, v_ms * (LC_TIME + pf.LOOKAHEAD_TIME))
        return c0 + (c1 - c0) * _ease(x / L)

    def _hits(self, objs, c0, c1, v_ms, half, still_only=False, x_min=None):
        """沿 c0 → c1 的计划轨迹，最近的会碰到的障碍物的沿路距离；没有时 None。"""
        near = None
        for x, d, _, _, still in objs:
            if still_only and not still:
                continue
            if x_min is not None and x < x_min:
                continue
            if abs(d - self._plan(max(0.0, x), c0, c1, v_ms)) < half + CLEARANCE:
                near = x if near is None else min(near, x)
        return near

    def _traffic_behind(self, objs, lane_d, w, v_ms):
        """目标车道后方有没有更快、BEHIND_TTC 秒内追上来的车。"""
        for x, d, _, v_obj, still in objs:
            if still or abs(d - lane_d) > w / 2:
                continue
            if -BEHIND_CHECK <= x <= 0.0 and v_obj > v_ms and -x / (v_obj - v_ms) < BEHIND_TTC:
                return True
        return False

    # ------------------------------------------------------------------ 决策
    def _decide(self, t, lane, objs, v_ms, half, w):
        offset = lane["offset"]
        if lane.get("in_junction"):
            return
        settled = abs(self.goal) < 0.1 * w and abs(offset - self.goal) < 0.5
        if not settled:
            return
        sides = [k for k, key in ((1, "left_lane"), (-1, "right_lane")) if lane[key] == "same"]
        block = self._hits(objs, offset, 0.0, v_ms, half, still_only=True, x_min=0.0)

        def clear_move(k):
            return (self._hits(objs, offset, k * w, v_ms, half, x_min=-REAR - pf.WHEELBASE) is None
                    and not self._traffic_behind(objs, k * w, w, v_ms))

        if block is not None:
            best = None
            for k in sides:
                if not clear_move(k):
                    continue
                far = self._hits(objs, k * w, k * w, v_ms, half, still_only=True, x_min=0.0)
                reach = AHEAD + 1.0 if far is None else far   # 一直空着的车道排在最前
                if (far is None or far > block + 10.0) and (best is None or reach > best[1]):
                    best = (k, reach)
            if best is not None:
                k = best[0]
                self.goal = k * w
                self.changes += 1
                self.stuck_told = False
                print("t = %.1f s：本车道前方 %.0f m 被挡住，向%s换道" % (t, block, "左" if k > 0 else "右"))
            elif not self.stuck_told:
                self.stuck_told = True
                print("t = %.1f s：本车道前方 %.0f m 被挡住，左右都换不了，减速" % (t, block))
            return
        if self.home != 0:
            k = 1 if self.home > 0 else -1
            if k in sides and clear_move(k) and \
                    self._hits(objs, k * w, k * w, v_ms, half, still_only=True, x_min=-REAR - pf.WHEELBASE) is None:
                self.goal = k * w
                self.changes += 1
                print("t = %.1f s：原车道已经空了，向%s换回去" % (t, "左" if k > 0 else "右"))

    def _follow_lane_change(self, lane):
        """车越过车道线、场景车道换成了旁边那条：平移量、原车道都跟着换算，轨迹不跳。"""
        off, w = lane["offset"], lane["width"]
        if self.last_offset is not None:
            jump = off - self.last_offset
            if abs(jump) > 0.6 * w:
                k = 1 if jump < 0 else -1          # offset 从 +w/2 跳到 -w/2：到了左边一条
                self.shift -= k * w
                self.goal -= k * w
                self.home -= k                     # 原车道相对新车道往反方向挪一条
        self.last_offset, self.last_width = off, w

    # ------------------------------------------------------------------ control
    def control(self, exports, t, dt, scene):
        deg, kmh = self._units(scene)
        v_ms = max(0.0, (exports["Vx"] if "Vx" in exports else scene["ego"]["Speed"]) * kmh / 3.6)
        lane = scene.get("lane")
        pts = [(p[0], p[1]) for p in ((lane or {}).get("center_rel") or [])]
        self.cap_kmh = None
        if lane is None or len(pts) < 2:
            self.last_offset = None
            return super().control(exports, t, dt, scene)
        w = lane["width"] or 3.5
        half = scene["ego"]["width"] / 2.0
        self._follow_lane_change(lane)
        objs = self._objects(scene, pts, v_ms, kmh)
        self._decide(t, lane, objs, v_ms, half, w)

        # 平移量往目标走：一个车道宽用 LC_TIME 秒
        step = w / LC_TIME * max(dt, 0.0)
        self.shift += max(-step, min(step, self.goal - self.shift))

        # 沿现在的计划轨迹，前方最近的东西 → 限速
        cap = None
        for x, d, gap, v_obj, still in objs:
            if x <= 0.0 or abs(d - self._plan(x, lane["offset"], self.goal, v_ms)) >= half + CLEARANCE:
                continue
            v = math.sqrt(2.0 * A_STOP * max(0.0, gap - STOP_GAP)) + (0.0 if still else max(0.0, v_obj))
            cap = v if cap is None else min(cap, v)
        self.cap_kmh = None if cap is None else cap * 3.6

        shifted = dict(scene)
        # offset 也换成相对平移后的线：path_follower 打印、统计的就是跟踪误差
        shifted["lane"] = dict(lane, center_rel=[list(p) for p in _shift(pts, self.shift)],
                               offset=lane["offset"] - self.shift)
        return super().control(exports, t, dt, shifted)

    def _curve_speed(self, pts, v_ms):
        v = super()._curve_speed(pts, v_ms)
        return v if self.cap_kmh is None else min(v, self.cap_kmh)

    def finish(self, reason):
        print("避障：共换道 %d 次%s" % (self.changes, "" if self.home == 0 else "，结束时不在原车道"))
        super().finish(reason)
