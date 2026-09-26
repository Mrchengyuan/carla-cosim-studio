"""What the ego sees each frame, handed to the user's control algorithm.

    control(exports, t, dt, scene)

scene is a dict (keys depend on the "scene" settings):

    scene["ego"]        {"speed": m/s, "length", "width", "height": m}
    scene["objects"]    everything around the ego within range_m, nearest first:
        {"id", "type": "vehicle" | "walker" | "static", "type_id",
         "moving": False for parked cars and map objects,
         "x", "y", "z": centre of the object's box,
         "yaw": deg, "length", "width", "height": m,
         "vx", "vy": velocity relative to the ego, m/s,
         "speed": the object's own speed, m/s, "distance": m}
    scene["lane"]       the ego's lane ahead:
        {"center": [[x, y], ...] every lane_step_m up to lane_ahead_m,
         "width": m, "offset": ego distance left of the lane centre, m,
         "heading_error": ego heading minus lane heading, deg (+ = left),
         "left_marking", "right_marking": CARLA marking types,
         "speed_limit": km/h, "traffic_light": "red" | "yellow" | "green" | None,
         "in_junction": bool}
    scene["sensors"]    {rig sensor name: {"type", "frame", "data"}} (see _convert)
    scene["collisions"] objects whose box overlaps the ego's box this frame

Every position is in the ego frame: origin at the centre of the ego's box on
the ground, x forward, y left, z up (right-handed, like CarSim), meters.
The scene is the state after the previous CARLA frame, like a real sensor.
"""

import math
import queue
import time

import numpy as np

import carla

import rig as rigmod
import settings as st

# Map objects that stand where a car can drive into them. Vegetation and
# buildings are left out: tens of thousands of them, with boxes far larger
# than the part that reaches the road.
MAP_VEHICLES = ("Car", "Truck", "Bus", "Motorcycle", "Bicycle")
MAP_STATIC = ("Poles", "Fences", "Walls", "GuardRail", "TrafficSigns", "TrafficLight", "Static", "Dynamic")
EVENT_TYPES = ("collision", "lane_invasion")


def _wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0


def _overlap(a, b):
    """2D oriented boxes (cx, cy, yaw_rad, half_len, half_wid) intersect (separating axes)."""
    ax, ay, at, al, aw = a
    bx, by, bt, bl, bw = b
    d = (bx - ax, by - ay)
    for t in (at, bt):
        u = (math.cos(t), math.sin(t))
        v = (-u[1], u[0])
        for axis in (u, v):
            ra = al * abs(math.cos(at) * axis[0] + math.sin(at) * axis[1]) + aw * abs(-math.sin(at) * axis[0] + math.cos(at) * axis[1])
            rb = bl * abs(math.cos(bt) * axis[0] + math.sin(bt) * axis[1]) + bw * abs(-math.sin(bt) * axis[0] + math.cos(bt) * axis[1])
            if abs(d[0] * axis[0] + d[1] * axis[1]) > ra + rb:
                return False
    return True


class SceneProvider:
    """Builds the scene after every world tick. Optionally owns the rig sensors
    (the data collector then reads their data from here instead of spawning
    a second set)."""

    def __init__(self, world, ego, settings, sensor_cfgs, frame_dt):
        s = st.default_dict()["scene"]
        s.update(settings or {})
        self.world, self.ego, self.s, self.frame_dt = world, ego, s, float(frame_dt)
        self.sensor_cfgs = [c for c in (sensor_cfgs or []) if c.get("enabled", True)] if s["sensors"] else []
        self.actors, self.queues = [], []
        self.datas, self.frame = [], None   # raw rig data of the last frame (collector)
        self.errors = []
        self.latest = None
        self._kinds = {}                    # actor id -> (type, type_id, bbox) or None
        self._map = None                    # static map objects: arrays
        self._touching = set()              # ids overlapping the ego last frame

    # --------------------------------------------------------------- lifecycle
    def start(self, ego_velocity=None):
        w = self.world
        self.map = w.get_map()
        eb = self.ego.bounding_box
        self._ego_box = (eb.location.x, eb.location.y, eb.extent.x, eb.extent.y, eb.extent.z)
        if self.s["objects"] and self.s["map_objects"]:
            rows, meta = [], []
            for kind, labels in (("vehicle", MAP_VEHICLES), ("static", MAP_STATIC)):
                for lab in labels:
                    L = getattr(carla.CityObjectLabel, lab, None)
                    if L is None:
                        continue
                    for o in w.get_environment_objects(L):
                        b = o.bounding_box
                        e = b.extent
                        if max(e.x, e.y) > 15.0 or e.z < 0.075:
                            continue  # merged meshes; road decals and manhole covers (1 cm high)
                        wp = self.map.get_waypoint(b.location, project_to_road=True, lane_type=carla.LaneType.Any)
                        if wp is not None and b.location.z - e.z - wp.transform.location.z > 2.5:
                            continue  # overhead: street light arms, signs and lights over the road
                        rows.append((b.location.x, b.location.y, b.location.z, b.rotation.yaw, e.x, e.y, e.z))
                        meta.append((o.id, kind, "map." + lab))
            self._map = (np.array(rows, dtype=np.float64).reshape(-1, 7), meta)
        bl = w.get_blueprint_library()
        for c in self.sensor_cfgs:
            bp = bl.find(rigmod.SENSOR_BLUEPRINTS[c["type"]])
            for k, v in c.get("attributes", {}).items():
                if bp.has_attribute(k):
                    bp.set_attribute(k, str(v))
            if c["type"] == "lidar" and bp.has_attribute("rotation_frequency"):
                bp.set_attribute("rotation_frequency", str(1.0 / self.frame_dt))  # one sweep per frame
            tf = carla.Transform(carla.Location(c["x"], c["y"], c["z"]),
                                 carla.Rotation(pitch=c["pitch"], yaw=c["yaw"], roll=c["roll"]))
            a = w.spawn_actor(bp, tf, attach_to=self.ego)
            q = queue.Queue()
            a.listen(q.put)
            self.actors.append(a)
            self.queues.append(q)
        self.latest = self.update(None, ego_velocity)

    def stop(self):
        for a in self.actors:
            try:
                a.stop()
                a.destroy()
            except RuntimeError:
                pass
        self.actors, self.queues = [], []

    # -------------------------------------------------------------- per frame
    def update(self, frame, ego_velocity=None):
        """Call right after world.tick() (frame = its return value).
        ego_velocity: the ego's world velocity when CARLA does not know it
        (co-simulation on the original CARLA teleports the car: speed 0)."""
        if frame is not None:
            self.frame, self.datas = frame, self._fetch(frame)
        w, s = self.world, self.s
        snap = w.get_snapshot()
        es = snap.find(self.ego.id)
        etf = es.get_transform() if es is not None else self.ego.get_transform()
        ev = ego_velocity if ego_velocity is not None else \
            es.get_velocity() if es is not None else self.ego.get_velocity()
        # Ego frame: box centre on the ground, x forward, y left (right-handed).
        yaw = -math.radians(etf.rotation.yaw)
        cy, sy = math.cos(yaw), math.sin(yaw)
        bx, by, el, ew, eh = self._ego_box
        ox = etf.location.x + math.cos(-yaw) * bx - math.sin(-yaw) * by
        oy = -(etf.location.y + math.sin(-yaw) * bx + math.cos(-yaw) * by)
        oz = etf.location.z
        evx, evy = ev.x, -ev.y

        def to_ego(x, y):
            dx, dy = x - ox, -y - oy
            return cy * dx + sy * dy, -sy * dx + cy * dy

        def rel_vel(vx, vy):
            dx, dy = vx - evx, -vy - evy
            return cy * dx + sy * dy, -sy * dx + cy * dy

        scene = {"frame": frame, "ego": {"speed": math.hypot(ev.x, ev.y), "length": 2 * el, "width": 2 * ew,
                                         "height": 2 * eh}}
        r = float(s["range_m"])
        objs = []
        if s["objects"]:
            for a in snap:
                if a.id == self.ego.id:
                    continue
                k = self._kinds.get(a.id, False)
                if k is False:
                    k = self._kinds[a.id] = self._kind(a.id)
                if k is None:
                    continue
                tf = a.get_transform()
                kind, type_id, bb = k
                # (transform() overwrites the point it is given: pass a copy)
                c = tf.transform(carla.Location(bb.location.x, bb.location.y, bb.location.z))
                x, y = to_ego(c.x, c.y)
                dist = math.hypot(x, y)
                if dist > r:
                    continue
                v = a.get_velocity()
                vx, vy = rel_vel(v.x, v.y)
                objs.append({"id": a.id, "type": kind, "type_id": type_id, "moving": True,
                             "x": x, "y": y, "z": c.z - oz,
                             "yaw": _wrap(-(tf.rotation.yaw + bb.rotation.yaw) + etf.rotation.yaw),
                             "length": 2 * bb.extent.x, "width": 2 * bb.extent.y, "height": 2 * bb.extent.z,
                             "vx": vx, "vy": vy, "speed": math.hypot(v.x, v.y), "distance": dist})
            if self._map is not None and len(self._map[1]):
                # Hundreds of map objects: all at once.
                arr, meta = self._map
                dx, dy = arr[:, 0] - ox, -arr[:, 1] - oy
                xe, ye = cy * dx + sy * dy, -sy * dx + cy * dy
                dist = np.hypot(xe, ye)
                idx = np.nonzero(dist <= r)[0]
                vx, vy = rel_vel(0.0, 0.0)
                yaws = (-arr[idx, 3] + etf.rotation.yaw + 180.0) % 360.0 - 180.0
                for i, x, y, z, yw, ex, ey, ez, dd in zip(
                        idx.tolist(), xe[idx].tolist(), ye[idx].tolist(), (arr[idx, 2] - oz).tolist(), yaws.tolist(),
                        arr[idx, 4].tolist(), arr[idx, 5].tolist(), arr[idx, 6].tolist(), dist[idx].tolist()):
                    oid, kind, type_id = meta[i]
                    objs.append({"id": oid, "type": kind, "type_id": type_id, "moving": False,
                                 "x": x, "y": y, "z": z, "yaw": yw,
                                 "length": 2 * ex, "width": 2 * ey, "height": 2 * ez,
                                 "vx": vx, "vy": vy, "speed": 0.0, "distance": dd})
            objs.sort(key=lambda o: o["distance"])
        scene["objects"] = objs
        scene["collisions"] = self._collisions(objs, el, ew, eh)
        if s["lane"]:
            scene["lane"] = self._lane(etf, to_ego)
        if self.sensor_cfgs:
            scene["sensors"] = {c["name"]: {"type": c["type"], "frame": getattr(d, "frame", frame),
                                            "data": self._convert(c["type"], d)}
                                for c, d in zip(self.sensor_cfgs, self.datas or [None] * len(self.sensor_cfgs))}
        self.latest = scene
        return scene

    def _kind(self, actor_id):
        a = self.world.get_actor(actor_id)
        if a is None:
            return None
        t = a.type_id
        if t.startswith("vehicle."):
            kind = "vehicle"
        elif t.startswith("walker.pedestrian"):
            kind = "walker"
        elif t.startswith("static.prop"):
            kind = "static"
        else:
            return None
        return kind, t, a.bounding_box

    def _collisions(self, objs, el, ew, eh):
        """Objects whose box overlaps the ego's (the ego's pose comes from
        CarSim, so CARLA itself never stops the car on contact)."""
        hits, touching = [], set()
        ego = (0.0, 0.0, 0.0, el, ew)
        for o in objs:
            if o["distance"] > (o["length"] + o["width"]) / 2 + el + ew:
                continue  # cannot reach the ego's box
            if o["z"] - o["height"] / 2 > 2 * eh or o["z"] + o["height"] / 2 < 0.05:
                continue  # above the car (a sign arm) or flat on the ground
            if _overlap(ego, (o["x"], o["y"], math.radians(o["yaw"]), o["length"] / 2, o["width"] / 2)):
                touching.add(o["id"])
                hits.append({"id": o["id"], "type": o["type"], "type_id": o["type_id"],
                             "new": o["id"] not in self._touching})
        self._touching = touching
        return hits

    def _lane(self, etf, to_ego):
        wp = self.map.get_waypoint(etf.location, project_to_road=True, lane_type=carla.LaneType.Driving)
        if wp is None:
            return None
        step, ahead = max(0.5, float(self.s["lane_step_m"])), float(self.s["lane_ahead_m"])
        pts, cur, d = [], wp, 0.0
        while cur is not None and d <= ahead + 1e-6:
            pts.append(list(to_ego(cur.transform.location.x, cur.transform.location.y)))
            nxt = cur.next(step)
            if not nxt:
                break
            # At a split, keep the branch that turns least (the lane goes on).
            cur = min(nxt, key=lambda n: abs(_wrap(n.transform.rotation.yaw - cur.transform.rotation.yaw)))
            d += step
        lx, ly = to_ego(wp.transform.location.x, wp.transform.location.y)
        light = None
        try:
            if self.ego.is_at_traffic_light():
                light = str(self.ego.get_traffic_light_state()).lower()
                light = light if light in ("red", "yellow", "green") else None
        except RuntimeError:
            pass
        try:
            limit = float(self.ego.get_speed_limit())
        except RuntimeError:
            limit = None
        return {"center": pts, "width": wp.lane_width, "offset": -ly,
                "heading_error": _wrap(-(etf.rotation.yaw - wp.transform.rotation.yaw)),
                "left_marking": str(wp.left_lane_marking.type), "right_marking": str(wp.right_lane_marking.type),
                "speed_limit": limit, "traffic_light": light, "in_junction": wp.is_junction,
                "road_id": wp.road_id, "lane_id": wp.lane_id}

    # ------------------------------------------------------------------ sensors
    def _fetch(self, frame):
        """Raw data of every rig sensor for this frame (None if it never came)."""
        datas = []
        for c, q in zip(self.sensor_cfgs, self.queues):
            if c["type"] in EVENT_TYPES:  # fire now and then: take what is there
                ev = []
                while True:
                    try:
                        d = q.get_nowait()
                    except queue.Empty:
                        break
                    if d.frame <= frame:
                        ev.append(d)
                datas.append(ev or None)
                continue
            d, deadline = None, time.time() + 5.0
            while time.time() < deadline:
                try:
                    d = q.get(timeout=max(0.01, deadline - time.time()))
                except queue.Empty:
                    break
                if d.frame >= frame:
                    break
            if d is None or d.frame != frame:
                self.errors.append("%s: 第 %d 帧数据缺失" % (c["name"], frame))
                del self.errors[:-100]
                d = None
            datas.append(d)
        return datas

    @staticmethod
    def _convert(kind, d):
        """numpy / dict view of a measurement, right-handed like the rest:
        rgb HxWx3 uint8 (RGB); depth HxW float32 m; semantic HxW uint8 tag;
        instance HxWx3 uint8 (R tag, G+B*256 object id); lidar Nx4 float32
        [x fwd, y left, z up, intensity] in the sensor frame; radar Nx4
        float32 [depth m, azimuth rad (+ left), altitude rad, velocity m/s];
        imu / gnss dicts; collision / lane_invasion lists of events."""
        if d is None:
            return None
        if kind in rigmod.CAMERA_TYPES:
            bgra = np.frombuffer(d.raw_data, dtype=np.uint8).reshape(d.height, d.width, 4)
            if kind == "depth":
                b = bgra.astype(np.float32)
                return (b[:, :, 2] + b[:, :, 1] * 256.0 + b[:, :, 0] * 65536.0) / (256.0 ** 3 - 1) * 1000.0
            if kind == "semantic":
                return bgra[:, :, 2].copy()
            return np.ascontiguousarray(bgra[:, :, 2::-1])
        if kind == "lidar":
            p = np.frombuffer(d.raw_data, dtype=np.float32).reshape(-1, 4).copy()
            p[:, 1] *= -1.0
            return p
        if kind == "radar":
            p = np.frombuffer(d.raw_data, dtype=np.float32).reshape(-1, 4)  # velocity, azimuth, altitude, depth
            return np.stack([p[:, 3], -p[:, 1], p[:, 2], p[:, 0]], axis=1) if len(p) else np.zeros((0, 4), np.float32)
        if kind == "imu":
            a, g = d.accelerometer, d.gyroscope
            return {"accel": [a.x, -a.y, a.z], "gyro": [-g.x, g.y, -g.z], "compass": d.compass}
        if kind == "gnss":
            return {"lat": d.latitude, "lon": d.longitude, "alt": d.altitude}
        out = []
        for ev in d:
            e = {"frame": ev.frame}
            other = getattr(ev, "other_actor", None)
            if other is not None:
                e["other_id"], e["other_type_id"] = other.id, other.type_id
            if hasattr(ev, "crossed_lane_markings"):
                e["markings"] = [str(m.type) for m in ev.crossed_lane_markings]
            out.append(e)
        return out


def gui_view(scene, max_objects=60):
    """Compact copy for the GUI: no sensor arrays, rounded numbers."""
    if not scene:
        return None
    r = lambda v: round(float(v), 2)
    objs = scene.get("objects", [])
    # Every moving object first (a parked car or pole nearby must not push a
    # car further away out of the GUI's list), then the nearest static ones.
    moving = [o for o in objs if o["moving"]][:max_objects]
    keep = moving + [o for o in objs if not o["moving"]][:max_objects - len(moving)]
    keep.sort(key=lambda o: o["distance"])
    out = {"objects": [{"id": o["id"], "type": o["type"], "type_id": o["type_id"], "moving": o["moving"],
                        "x": r(o["x"]), "y": r(o["y"]), "yaw": r(o["yaw"]), "length": r(o["length"]),
                        "width": r(o["width"]), "vx": r(o["vx"]), "vy": r(o["vy"]), "speed": r(o["speed"]),
                        "distance": r(o["distance"])} for o in keep],
           "n_objects": len(scene.get("objects", [])),
           "ego": {k: r(v) for k, v in scene["ego"].items()},
           "collisions": scene.get("collisions", [])}
    lane = scene.get("lane")
    if lane:
        out["lane"] = {k: v for k, v in lane.items() if k != "center"}
        out["lane"]["center"] = [[r(p[0]), r(p[1])] for p in lane["center"]]
    if "sensors" in scene:
        out["sensors"] = {n: {"type": v["type"], "frame": v["frame"],
                              "shape": list(getattr(v["data"], "shape", ())) or (len(v["data"]) if isinstance(v["data"], list) else None)}
                          for n, v in scene["sensors"].items()}
    return out
