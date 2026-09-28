"""Test scenarios placed on the CARLA map for a run (the "测试场景" page):
work zones that close lanes with traffic cones or barriers.

A closure is given relative to the ego's spawn point, so it lands on the
same place every run: distance_m along the road ahead of the spawn point
(through junctions the branch that turns least, like the scene's lane),
lane relative to the lane the car starts in (0 = that lane, -1 the first
one to the left, 1 the first to the right; same driving direction only),
taper_m of cones / barriers crossing the lane diagonally (the lane narrows
from its outer side: from the right for the start lane and the lanes to the
right, from the left for the lanes to the left), then length_m closed, with
a line along each boundary that has a drivable lane beside it, and an arrow
board behind the taper. Moving actors (动态目标: a slow car, a car that
brakes, one that cuts in, a pedestrian crossing) are further down. kind "cones" (static.prop.constructioncone) or
"barrier" (static.prop.streetbarrier).

The GUI's presets are made for a start like the Town04 highway's: four lanes
one way, the car in the second from the left, a long road ahead without a
junction. find_start() finds it on the map (the spawn point's number is not
the same in every CARLA build: 41 in the 0.9.16 release, 39 in one built
from source).

The props are static (no physics: the co-simulated car, which CARLA cannot
stop, never pushes them away), tagged role_name "cosim_scenario" so a
backend that restarts after a crash finds them, and reach the algorithm as
scene objects of type "static" (scene.py). layout() works on the map only:
a closure that cannot be placed (no such lane, the road ends) is refused
before anything in the world changes.
"""

import math

import carla

ROLE = "cosim_scenario"
MODELS = {"cones": "static.prop.constructioncone", "barrier": "static.prop.streetbarrier"}
ARROW = "static.prop.trafficwarning"
# Spacing along the taper / along the closed stretch, m. A barrier is 1.21 m long.
SPACING = {"cones": (3.0, 6.0), "barrier": (1.3, 2.5)}
KIND_NAMES = {"cones": "锥桶", "barrier": "护栏"}
LIMITS = {"distance_m": (1.0, 5000.0), "taper_m": (0.0, 300.0), "length_m": (1.0, 3000.0)}
STEP = 1.0  # m, walking along the road
START_LANES = (1, 2)   # the presets' start: lanes to the left, to the right (four lanes, the second from the left)
START_MIN_FREE = 300.0  # m without a junction ahead, at least
START_SCAN = 1500.0     # m, looked ahead at most


def lane_name(k):
    return "本车道" if k == 0 else "%s侧第 %d 条车道" % ("左" if k < 0 else "右", abs(k))


def check_closures(closures):
    """The closures of a config, checked and normalised (numbers as numbers);
    ValueError in plain words otherwise."""
    if not isinstance(closures, list):
        raise ValueError("测试场景的封道列表格式不对（应为列表）")
    out = []
    for i, c in enumerate(closures, 1):
        if not isinstance(c, dict):
            raise ValueError("测试场景第 %d 处封道格式不对" % i)
        n = {}
        for key, (lo, hi) in LIMITS.items():
            try:
                v = float(c.get(key))
            except (TypeError, ValueError):
                raise ValueError("测试场景第 %d 处封道的 %s 不是数字：%r" % (i, key, c.get(key)))
            if not (lo <= v <= hi) or math.isnan(v):
                raise ValueError("测试场景第 %d 处封道的 %s = %g 超出范围 %g ~ %g" % (i, key, v, lo, hi))
            n[key] = v
        try:
            n["lane"] = int(c.get("lane", 0))
        except (TypeError, ValueError):
            raise ValueError("测试场景第 %d 处封道的车道不是整数：%r" % (i, c.get("lane")))
        if c.get("kind", "cones") not in MODELS:
            raise ValueError("测试场景第 %d 处封道的类型 %r 不认识（可选 cones 锥桶、barrier 护栏）" % (i, c.get("kind")))
        n["kind"] = c.get("kind", "cones")
        out.append(n)
    return out


def _same_way(a, b):
    return b is not None and b.lane_type == carla.LaneType.Driving and (a.lane_id > 0) == (b.lane_id > 0)


def _side(wp, k):
    """The k-th lane to the left (k < 0) or right (k > 0) of wp, same direction; None if there is none."""
    cur = wp
    for _ in range(abs(k)):
        nxt = cur.get_left_lane() if k < 0 else cur.get_right_lane()
        if not _same_way(wp, nxt):
            return None
        cur = nxt
    return cur


def _lanes_around(wp):
    """(lanes to the left, lanes to the right) of wp in its direction."""
    n = [0, 0]
    for i, k in ((0, -1), (1, 1)):
        while _side(wp, k * (n[i] + 1)) is not None:
            n[i] += 1
    return n


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def _ahead(wp, dist):
    """The waypoint dist m further along the lane (the branch that turns least); None where the road ends."""
    cur, d = wp, 0.0
    while d < dist - 1e-6:
        step = min(STEP, dist - d)
        nxt = cur.next(step)
        if not nxt:
            return None
        cur = min(nxt, key=lambda n: abs(_wrap(n.transform.rotation.yaw - cur.transform.rotation.yaw)))
        d += step
    return cur


def free_ahead(wp, cap=START_SCAN, step=2.0):
    """How far the lane goes on from wp before a junction (or its end), up to cap m."""
    cur, d = wp, 0.0
    while d < cap:
        nxt = cur.next(step)
        if not nxt:
            break
        cur = min(nxt, key=lambda n: abs(_wrap(n.transform.rotation.yaw - cur.transform.rotation.yaw)))
        if cur.is_junction:
            break
        d += step
    return d


def find_start(cmap):
    """The spawn point for the presets: START_LANES around it, the longest road
    ahead without a junction (at least START_MIN_FREE). {"index", "free_m",
    "lanes", "from_left"}, or None when the map has none."""
    best = None
    for i, p in enumerate(cmap.get_spawn_points()):
        wp = cmap.get_waypoint(p.location, project_to_road=True, lane_type=carla.LaneType.Driving)
        if wp is None or wp.is_junction or tuple(_lanes_around(wp)) != START_LANES:
            continue
        free = free_ahead(wp)
        if free >= START_MIN_FREE and (best is None or free > best[0]):
            best = (free, i)
    if best is None:
        return None
    return {"index": best[1], "free_m": round(best[0]), "lanes": sum(START_LANES) + 1, "from_left": START_LANES[0] + 1}


def _at(wp, lateral, yaw_extra=0.0):
    """A transform on the road beside wp: lateral m to its right (negative: left)."""
    tf = wp.transform
    r = tf.get_right_vector()
    loc = carla.Location(tf.location.x + r.x * lateral, tf.location.y + r.y * lateral, tf.location.z + 0.02)
    return carla.Transform(loc, carla.Rotation(yaw=tf.rotation.yaw + yaw_extra))


def layout(cmap, spawn, closures):
    """What to place: [(blueprint id, carla.Transform, closure number)] and a
    summary per closure. spawn: the ego's spawn transform. ValueError when a
    closure cannot be placed (the world is not touched)."""
    closures = check_closures(closures)
    start = cmap.get_waypoint(spawn.location, project_to_road=True, lane_type=carla.LaneType.Driving)
    if start is None:
        raise ValueError("出生点不在行车道上，放不了测试场景")
    left, right = _lanes_around(start)
    items, summary = [], []
    for num, c in enumerate(closures, 1):
        what = "测试场景第 %d 处封道（%s，出生点前方 %g m）" % (num, lane_name(c["lane"]), c["distance_m"])
        base = _ahead(start, c["distance_m"])
        if base is None:
            raise ValueError("%s：出生点前方不到 %g m 路就到头了" % (what, c["distance_m"]))
        lane0 = _side(base, c["lane"])
        if lane0 is None:
            l2, r2 = _lanes_around(base)
            raise ValueError("%s：那里没有这条车道。那里同方向有 %d 条车道，车在左数第 %d 条（出生时同方向 %d 条，在左数第 %d 条）"
                             % (what, l2 + r2 + 1, l2 + 1, left + right + 1, left + 1))
        model = MODELS[c["kind"]]
        sp_taper, sp_line = SPACING[c["kind"]]
        # The taper starts on the lane's outer side: the right for the start lane and those to its
        # right, the left for those to its left; it ends on the other boundary.
        outer = 1.0 if c["lane"] >= 0 else -1.0
        total = c["taper_m"] + c["length_m"]
        count = 0
        # Along the taper: diagonally across the lane.
        if c["taper_m"] > 0:
            n = max(2, int(round(c["taper_m"] / sp_taper)) + 1)
            half = lane0.lane_width / 2 - 0.25
            for j in range(n):
                s = c["taper_m"] * j / (n - 1)
                wp = _ahead(lane0, s)
                if wp is None:
                    raise ValueError("%s：封道范围内路就到头了" % what)
                lat = outer * half * (1.0 - 2.0 * j / (n - 1))
                slope = math.degrees(math.atan2(-2.0 * outer * half, c["taper_m"]))  # the diagonal's direction
                items.append((model, _at(wp, lat, slope if c["kind"] == "barrier" else 0.0), num))
                count += 1
        # Along the closed stretch: a line on each boundary with a drivable lane beside it. The outer
        # boundary's line starts where the taper starts (traffic passes it there already).
        for side in (-1.0, 1.0):
            neighbour = _side(lane0, -1 if side < 0 else 1)
            if neighbour is None:
                continue
            s0 = 0.0 if side == outer else c["taper_m"]
            s = s0 + (sp_line if side == outer and c["taper_m"] > 0 else 0.0)
            while s <= total + 1e-6:
                wp = _ahead(lane0, s)
                if wp is None:
                    raise ValueError("%s：封道范围内路就到头了" % what)
                items.append((model, _at(wp, side * (lane0.lane_width / 2 - 0.25)), num))
                count += 1
                s += sp_line
        # The arrow board, behind the taper in the closed lane, its lamp panel towards the traffic.
        wp = _ahead(lane0, min(total, c["taper_m"] + 6.0))
        if wp is not None:
            items.append((ARROW, _at(wp, 0.0, -90.0), num))
            count += 1
        summary.append(dict(c, number=num, props=count, lane_id=lane0.lane_id, road_id=lane0.road_id))
    return items, summary


def spawn(client, world, items):
    """Places the layout (one batch, no physics, role ROLE); the actor ids."""
    lib = world.get_blueprint_library()
    bps = {}
    cmds = []
    for model, tf, _ in items:
        bp = bps.get(model)
        if bp is None:
            bp = bps[model] = lib.find(model)
            if bp.has_attribute("role_name"):
                bp.set_attribute("role_name", ROLE)
        cmds.append(carla.command.SpawnActor(bp, tf).then(
            carla.command.SetSimulatePhysics(carla.command.FutureActor, False)))
    ids = []
    for r in client.apply_batch_sync(cmds, False):
        if not r.error:
            ids.append(r.actor_id)
    return ids


def remove(client, ids):
    if ids:
        client.apply_batch_sync([carla.command.DestroyActor(i) for i in ids], False)


def leftovers(world):
    """Scenario props and moving actors in the world (e.g. left by a backend that crashed)."""
    return [a for a in world.get_actors() if a.type_id.startswith(("static.prop.", "vehicle.", "walker.pedestrian."))
            and a.attributes.get("role_name") == ROLE]


# ---------------------------------------------------------------------------
# Moving actors (the 测试场景 page's 动态目标): vehicles and pedestrians the
# backend places kinematically every frame along the lanes from the spawn
# point, so each run is the same (no CARLA physics, no traffic manager).
#   slow_car    a car ahead in lane `lane` at speed_kmh, the whole run
#   lead_brake  a car ahead at speed_kmh; once the ego is within trigger_m of
#               it, it brakes at param m/s^2 to a stop
#   cut_in      a car in lane `lane` (next to the ego's: -1 / 1) at speed_kmh;
#               once the ego is within trigger_m, it moves into the ego's
#               starting lane over param s
#   pedestrian  someone at the side of the ego's lane distance_m ahead (the
#               right for lane >= 0, the left otherwise); once the ego is
#               within trigger_m, crosses the lane at speed_kmh
# distance_m along the road from the spawn point like a closure's. Their
# velocity is given to the scene (stock CARLA reads 0 for actors placed like
# this), so the algorithm sees their real speed.
ACTOR_TYPES = ("slow_car", "lead_brake", "cut_in", "pedestrian")
ACTOR_NAMES = {"slow_car": "前车慢行", "lead_brake": "前车急刹", "cut_in": "旁车切入", "pedestrian": "行人横穿"}
ACTOR_LIMITS = {"distance_m": (1.0, 5000.0), "speed_kmh": (0.0, 200.0), "trigger_m": (0.0, 500.0), "param": (0.0, 20.0)}
CAR_MODEL = "vehicle.lincoln.mkz_2020"
WALKER_MODEL = "walker.pedestrian.0001"
PATH_STEP = 1.0     # m between the precomputed path points
PATH_MAX = 3000.0   # m of road a car can drive along at most
WALK_MARGIN = 1.5   # m beyond the lane's edge a pedestrian starts / ends


def check_actors(actors):
    """The moving actors of a config, checked and normalised; ValueError in plain words otherwise."""
    if not isinstance(actors, list):
        raise ValueError("测试场景的动态目标列表格式不对（应为列表）")
    out = []
    for i, a in enumerate(actors, 1):
        if not isinstance(a, dict):
            raise ValueError("测试场景第 %d 个动态目标格式不对" % i)
        kind = a.get("type")
        if kind not in ACTOR_TYPES:
            raise ValueError("测试场景第 %d 个动态目标的类型 %r 不认识（可选 %s）" % (
                i, kind, "、".join("%s %s" % (t, ACTOR_NAMES[t]) for t in ACTOR_TYPES)))
        n = {"type": kind}
        for key, (lo, hi) in ACTOR_LIMITS.items():
            try:
                v = float(a.get(key, 0.0))
            except (TypeError, ValueError):
                raise ValueError("测试场景第 %d 个动态目标的 %s 不是数字：%r" % (i, key, a.get(key)))
            if not (lo <= v <= hi) or math.isnan(v):
                raise ValueError("测试场景第 %d 个动态目标的 %s = %g 超出范围 %g ~ %g" % (i, key, v, lo, hi))
            n[key] = v
        try:
            n["lane"] = int(a.get("lane", 0))
        except (TypeError, ValueError):
            raise ValueError("测试场景第 %d 个动态目标的车道不是整数：%r" % (i, a.get("lane")))
        if kind == "cut_in" and n["lane"] == 0:
            raise ValueError("测试场景第 %d 个动态目标（旁车切入）要在相邻车道：车道填 -1（左侧）或 1（右侧）" % i)
        if kind in ("lead_brake", "cut_in") and n["param"] <= 0:
            raise ValueError("测试场景第 %d 个动态目标（%s）的参数要大于 0（%s）" % (
                i, ACTOR_NAMES[kind], "减速度 m/s²" if kind == "lead_brake" else "切入用时 s"))
        out.append(n)
    return out


def plan_actors(cmap, spawn, actors):
    """Where each moving actor goes, worked out on the map (the world is not touched):
    [{"type", ..., "path": [(x, y, z, yaw deg)], "width"}]; ValueError when one cannot be placed."""
    actors = check_actors(actors)
    start = cmap.get_waypoint(spawn.location, project_to_road=True, lane_type=carla.LaneType.Driving)
    if start is None:
        raise ValueError("出生点不在行车道上，放不了测试场景")
    plans = []
    for num, a in enumerate(actors, 1):
        what = "测试场景第 %d 个动态目标（%s，出生点前方 %g m）" % (num, ACTOR_NAMES[a["type"]], a["distance_m"])
        base = _ahead(start, a["distance_m"])
        if base is None:
            raise ValueError("%s：出生点前方不到 %g m 路就到头了" % (what, a["distance_m"]))
        lane_k = 0 if a["type"] in ("lead_brake", "pedestrian") else a["lane"]
        wp = _side(base, lane_k)
        if wp is None:
            l2, r2 = _lanes_around(base)
            raise ValueError("%s：那里没有%s。那里同方向有 %d 条车道" % (what, lane_name(lane_k), l2 + r2 + 1))
        path = []
        cur, d = wp, 0.0
        length = 1.0 if a["type"] == "pedestrian" else PATH_MAX
        while True:
            tf = cur.transform
            path.append((tf.location.x, tf.location.y, tf.location.z, tf.rotation.yaw))
            if d >= length:
                break
            nxt = cur.next(PATH_STEP)
            if not nxt:
                break
            cur = min(nxt, key=lambda n: abs(_wrap(n.transform.rotation.yaw - cur.transform.rotation.yaw)))
            d += PATH_STEP
        plans.append(dict(a, number=num, path=path, width=wp.lane_width, lane_id=wp.lane_id, road_id=wp.road_id))
    return plans


class Movers:
    """The moving actors of a run: spawned once (physics off, role ROLE), then step(dt)
    before every world tick places them for that frame; velocities: {actor id: (vx, vy, vz)}
    in CARLA's world frame for the scene."""

    def __init__(self, client, world, plans, ego):
        self.client, self.world, self.ego = client, world, ego
        self.plans = plans
        self.velocities = {}
        self.ids = []
        self.started = False
        lib = world.get_blueprint_library()
        cmds = []
        for p in plans:
            bp = lib.find(WALKER_MODEL if p["type"] == "pedestrian" else CAR_MODEL)
            if bp.has_attribute("role_name"):
                bp.set_attribute("role_name", ROLE)
            if bp.has_attribute("is_invincible"):
                bp.set_attribute("is_invincible", "false")
            loc, yaw = self._pose(p, self._state0(p))
            cmds.append(carla.command.SpawnActor(bp, carla.Transform(loc, carla.Rotation(yaw=yaw))).then(
                carla.command.SetSimulatePhysics(carla.command.FutureActor, False)))
        self.state = []
        for p, r in zip(plans, client.apply_batch_sync(cmds, False)):
            if r.error:
                self.state.append(None)
            else:
                self.ids.append(r.actor_id)
                self.state.append(dict(self._state0(p), id=r.actor_id))

    @staticmethod
    def _state0(p):
        if p["type"] == "pedestrian":
            side = 1.0 if p["lane"] >= 0 else -1.0
            edge = p["width"] / 2.0 + WALK_MARGIN
            return {"s": 0.0, "lat": side * edge, "v": 0.0, "lat_v": 0.0, "go": False, "t_go": 0.0,
                    "lat0": side * edge, "lat1": -side * edge}
        return {"s": 0.0, "lat": 0.0, "v": p["speed_kmh"] / 3.6, "lat_v": 0.0, "go": False, "t_go": 0.0,
                "lat0": 0.0, "lat1": -p["lane"] * p["width"] if p["type"] == "cut_in" else 0.0}

    @staticmethod
    def _at_s(p, s):
        """Position and yaw on the path at arc length s (interpolated; held at its end)."""
        path = p["path"]
        f = min(max(s / PATH_STEP, 0.0), len(path) - 1.0)
        i = int(f)
        j = min(i + 1, len(path) - 1)
        k = f - i
        (x0, y0, z0, h0), (x1, y1, z1, h1) = path[i], path[j]
        return x0 + k * (x1 - x0), y0 + k * (y1 - y0), z0 + k * (z1 - z0), h0 + k * _wrap(h1 - h0)

    def _pose(self, p, st):
        x, y, z, yaw = self._at_s(p, st["s"])
        r = math.radians(yaw)
        rx, ry = -math.sin(r), math.cos(r)  # CARLA's right vector (y right)
        lift = 0.95 if p["type"] == "pedestrian" else 0.05
        heading = yaw
        if p["type"] == "pedestrian":
            heading = yaw + (90.0 if st["lat1"] > st["lat0"] else -90.0)
        elif st["v"] > 0.1 and st["lat_v"]:
            heading = yaw + math.degrees(math.atan2(st["lat_v"], st["v"]))
        return carla.Location(x + rx * st["lat"], y + ry * st["lat"], z + lift), heading

    def step(self, dt):
        """Before a world tick: advance every actor by dt (the first call only places them)."""
        first = not self.started
        self.started = True
        try:
            eloc = self.ego.get_transform().location
        except RuntimeError:
            eloc = None
        batch = []
        for p, st in zip(self.plans, self.state):
            if st is None:
                continue
            loc_now, _ = self._pose(p, st)
            if not first:
                gap = loc_now.distance(eloc) if eloc is not None else 1e9
                if not st["go"] and p["type"] != "slow_car" and gap <= p["trigger_m"]:
                    st["go"] = True
                t = p["type"]
                st["lat_v"] = 0.0
                if t == "lead_brake" and st["go"]:
                    st["v"] = max(0.0, st["v"] - p["param"] * dt)
                if t == "cut_in" and st["go"] and st["t_go"] < p["param"]:
                    st["t_go"] = min(p["param"], st["t_go"] + dt)
                    u = st["t_go"] / p["param"]
                    lat = st["lat0"] + (st["lat1"] - st["lat0"]) * (1 - math.cos(math.pi * u)) / 2.0
                    st["lat_v"] = (lat - st["lat"]) / dt
                    st["lat"] = lat
                if t == "pedestrian" and st["go"]:
                    step = p["speed_kmh"] / 3.6 * dt
                    d = st["lat1"] - st["lat"]
                    move = math.copysign(min(step, abs(d)), d)
                    st["lat"] += move
                    st["lat_v"] = move / dt
                if t != "pedestrian":
                    st["s"] = min(st["s"] + st["v"] * dt, (len(p["path"]) - 1) * PATH_STEP)
                    if st["s"] >= (len(p["path"]) - 1) * PATH_STEP:
                        st["v"] = 0.0  # the end of its road: it stops there
            loc, heading = self._pose(p, st)
            batch.append(carla.command.ApplyTransform(st["id"], carla.Transform(loc, carla.Rotation(yaw=heading))))
            # Its velocity in CARLA's world frame: along the road plus the sideways part.
            _, _, _, yaw = self._at_s(p, st["s"])
            r = math.radians(yaw)
            fwd = st["v"] if p["type"] != "pedestrian" else 0.0
            self.velocities[st["id"]] = (math.cos(r) * fwd - math.sin(r) * st["lat_v"],
                                         math.sin(r) * fwd + math.cos(r) * st["lat_v"], 0.0)
        if batch:
            # Synchronous: in place before the tick that follows (asynchronous calls can land a frame late).
            self.client.apply_batch_sync(batch, False)

    def summary(self):
        return [{"number": p["number"], "type": p["type"], "name": ACTOR_NAMES[p["type"]], "distance_m": p["distance_m"],
                 "lane": p["lane"], "speed_kmh": p["speed_kmh"], "trigger_m": p["trigger_m"], "param": p["param"],
                 "placed": st is not None} for p, st in zip(self.plans, self.state)]
