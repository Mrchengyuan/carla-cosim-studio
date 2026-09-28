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
board behind the taper. kind "cones" (static.prop.constructioncone) or
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
    """Scenario props in the world (e.g. left by a backend that crashed)."""
    return [a for a in world.get_actors().filter("static.prop.*") if a.attributes.get("role_name") == ROLE]
