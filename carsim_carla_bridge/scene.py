"""What is around the car, handed to the control algorithm and recorded.

    control(exports, t, dt, scene)

Everything follows CarSim (docs/场景与数据接口.md has the full table):
  * global frame = CarSim's global frame (origin on the spawn point, x along
    its heading, y LEFT, z up), the frame of the exports Xo / Yo / Yaw;
  * ego frame = CarSim's vehicle frame, origin at the CarSim reference point
    (the point Xo / Yo describe), x forward, y left;
  * units = the CarSim export units set on the "CarSim 动力学" page
    (speeds km/h or m/s, angles deg or rad); lengths m.

Obstacles are CARLA vehicles and walkers, plus the parked cars that are part
of the map, within RANGE_M of the ego. The algorithm gets the keys selected
in "scene" (ego / objects / lane), the records keep the ones in
"scene" -> "record" (two separate lists).
"""

import csv
import math
import os
import queue
import time

import numpy as np

import carla

import rig as rigmod
import settings as st
from bridge import anchor_frame, front_axle_local

RANGE_M = 50.0
LANE_STEP_M = 2.0
EVENT_TYPES = ("collision", "lane_invasion")
MAP_VEHICLES = ("Car", "Truck", "Bus", "Motorcycle", "Bicycle")  # parked cars modelled into the map

EGO_KEYS = ("X", "Y", "Z", "Yaw", "Vx_global", "Vy_global", "Speed", "length", "width", "height")
OBJECT_KEYS = ("id", "type", "parked", "model", "length", "width", "height", "X", "Y", "Z", "Yaw",
               "Vx_global", "Vy_global", "Speed", "rel_x", "rel_y", "rel_yaw", "rel_vx", "rel_vy", "dist", "gap")
LANE_KEYS = ("width", "offset", "heading_err", "curvature", "center_rel", "center_global", "center_curvature",
             "left_marking", "right_marking", "left_lane", "right_lane", "speed_limit", "in_junction",
             "junction_dist", "light_state", "light_dist")
LANE_LISTS = ("center_rel", "center_global", "center_curvature")  # not in the CSV files
ALWAYS_OBJECT_KEYS = ("id", "type")  # needed to tell objects apart


def _wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0


def _corners(cx, cy, yaw, hl, hw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(cx + c * a - s * b, cy + s * a + c * b) for a, b in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]


def _overlap(pa, pb):
    """Convex polygons intersect (separating axis test)."""
    for poly in (pa, pb):
        for i in range(len(poly)):
            (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % len(poly)]
            nx, ny = y1 - y2, x2 - x1
            a = [nx * x + ny * y for x, y in pa]
            b = [nx * x + ny * y for x, y in pb]
            if max(a) < min(b) or max(b) < min(a):
                return False
    return True


def _seg_dist(p, a, b):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / max(1e-12, dx * dx + dy * dy)))
    return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy)


def box_gap(pa, pb):
    """Shortest distance between two rectangles in the plane; 0 when they touch."""
    if _overlap(pa, pb):
        return 0.0
    d = min(_seg_dist(p, q[i], q[(i + 1) % 4]) for p, q in ((p, pb) for p in pa) for i in range(4))
    return min(d, min(_seg_dist(p, pa[i], pa[(i + 1) % 4]) for p in pb for i in range(4)))


class Units:
    """CarSim's export units (settings "carsim" -> "units")."""

    def __init__(self, units):
        units = units or {}
        self.speed = 3.6 if units.get("speed", "km/h") == "km/h" else 1.0   # from m/s
        self.angle = 1.0 if units.get("angle", "deg") == "deg" else math.pi / 180.0  # from deg
        self.rate = 1.0 if units.get("rate", "deg/s") == "deg/s" else math.pi / 180.0  # from deg/s


class SceneProvider:
    """Builds the scene after every world tick (all keys, CarSim frames and
    units); view() is the selected part the algorithm gets. Also runs the
    rig sensors the algorithm or the data collector needs (one set for both)."""

    def __init__(self, world, ego, d, anchor=None, ref_local=None, sensor_cfgs=()):
        """anchor: carla.Transform of CarSim's origin (the spawn point; the
        ego's pose when None); ref_local: CarSim reference point in the CARLA
        vehicle frame (front axle on the ground when None)."""
        s = st.default_dict()["scene"]
        s.update(d.get("scene") or {})
        self.world, self.ego, self.s = world, ego, s
        self.frame_dt = float(d["sync"]["frame_dt"])
        self.units = Units(d["carsim"].get("units"))
        self.anchor_tf = None
        self._anchor_src = anchor
        self.ref_local = None if ref_local is None else np.asarray(ref_local, dtype=float)
        self.sensor_cfgs = [c for c in sensor_cfgs if c.get("enabled", True)]
        self.algo_sensors = set(s.get("sensors") or [])
        self.actors, self.queues = [], []
        self.datas, self.frame = [], None   # raw rig data of the last frame (collector)
        self.errors = []
        self.latest = self._view = self._record_view = None
        self._kinds = {}                    # actor id -> (type, model, bbox) or None
        self._parked = []                   # map parked cars: (id, model, centre world, yaw, extent)
        self._touching = set()
        self._yaw = None                    # ego yaw, continuous like CarSim's (not wrapped)

    # --------------------------------------------------------------- lifecycle
    def start(self, t=0.0, ego_velocity=None):
        w = self.world
        self.map = w.get_map()
        src = self._anchor_src or self.ego.get_transform()
        self.anchor = anchor_frame(self.map, src)
        R = self.anchor.R
        self._anchor_yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
        self._to_local = R.T
        if self.ref_local is None:
            self.ref_local = front_axle_local(self.ego)
        eb = self.ego.bounding_box
        self._ego_ext = (eb.extent.x, eb.extent.y, eb.extent.z)
        # Ego box centre in the ego frame (CarSim axes: y left).
        self._ego_box = (eb.location.x - self.ref_local[0], -(eb.location.y - self.ref_local[1]))
        if "parked" in (self.s.get("object_types") or []):
            for lab in MAP_VEHICLES:
                L = getattr(carla.CityObjectLabel, lab, None)
                if L is None:
                    continue
                for o in w.get_environment_objects(L):
                    b = o.bounding_box
                    self._parked.append((o.id, "map." + lab, (b.location.x, b.location.y, b.location.z),
                                         b.rotation.yaw, (b.extent.x, b.extent.y, b.extent.z)))
        bl = w.get_blueprint_library()
        for c in self.sensor_cfgs:
            bp = bl.find(rigmod.SENSOR_BLUEPRINTS[c["type"]])
            for k, v in c.get("attributes", {}).items():
                if bp.has_attribute(k):
                    bp.set_attribute(k, str(v))
            if c["type"] == "lidar" and bp.has_attribute("rotation_frequency"):
                bp.set_attribute("rotation_frequency", str(1.0 / self.frame_dt))  # one sweep per frame
            tf = rigmod.mount_transform(c, self.ref_local)
            a = w.spawn_actor(bp, tf, attach_to=self.ego)
            q = queue.Queue()
            a.listen(q.put)
            self.actors.append(a)
            self.queues.append(q)
        self.update(None, t, ego_velocity)
        self._touching = set()  # an overlap already there at the start counts as a new contact

    def stop(self):
        for a in self.actors:
            try:
                a.stop()
                a.destroy()
            except RuntimeError:
                pass
        self.actors, self.queues = [], []

    # --------------------------------------------------------------- frames
    def _global(self, x, y, z):
        """CARLA world point -> CarSim global frame (m)."""
        p = self._to_local @ (np.array([x, y, z]) - self.anchor.origin)
        return float(p[0]), float(-p[1]), float(p[2])

    def _global_vec(self, vx, vy, vz):
        v = self._to_local @ np.array([vx, vy, vz])
        return float(v[0]), float(-v[1])

    def _global_yaw(self, carla_yaw):
        return _wrap(-(carla_yaw - self._anchor_yaw))

    # -------------------------------------------------------------- per frame
    def update(self, frame, t=0.0, ego_velocity=None):
        """Call right after world.tick() (frame = its return value), t = CarSim time."""
        if frame is not None:
            self.frame, self.datas = frame, self._fetch(frame)
        su, au = self.units.speed, self.units.angle
        snap = self.world.get_snapshot()
        es = snap.find(self.ego.id)
        if es is None and self.latest is not None:
            # The ego left the world (fell off the map, deleted): the backend
            # ends the run; never report a scene around a stale pose.
            return self.latest
        etf = es.get_transform() if es is not None else self.ego.get_transform()
        ev = ego_velocity if ego_velocity is not None else \
            es.get_velocity() if es is not None else self.ego.get_velocity()
        rw = etf.transform(carla.Location(*map(float, self.ref_local)))  # reference point, world
        EX, EY, EZ = self._global(rw.x, rw.y, rw.z)
        eyaw = self._global_yaw(etf.rotation.yaw)
        # Continuous like CarSim's Yaw export (it keeps counting past +-180).
        self._yaw = eyaw if self._yaw is None else self._yaw + _wrap(eyaw - self._yaw)
        evx, evy = self._global_vec(ev.x, ev.y, ev.z)
        ps = math.radians(eyaw)
        cp, sp = math.cos(ps), math.sin(ps)

        def rel(dx, dy):
            return cp * dx + sp * dy, -sp * dx + cp * dy

        el, ew, eh = self._ego_ext
        ego_poly = _corners(self._ego_box[0], self._ego_box[1], 0.0, el, ew)
        scene = {"t": t, "frame": frame,
                 "ego": {"X": EX, "Y": EY, "Z": EZ, "Yaw": self._yaw * au, "Vx_global": evx * su, "Vy_global": evy * su,
                         "Speed": math.hypot(evx, evy) * su, "length": 2 * el, "width": 2 * ew, "height": 2 * eh}}
        types = set(self.s.get("object_types") or [])
        objs = []

        def add(oid, kind, parked, model, ext, c, yaw_g, vg):
            X, Y, Z = c
            x, y = rel(X - EX, Y - EY)
            dist = math.hypot(x, y)
            if dist > RANGE_M:
                return
            ryaw = _wrap(yaw_g - eyaw)
            rvx, rvy = rel(vg[0] - evx, vg[1] - evy)
            gap = box_gap(ego_poly, _corners(x, y, math.radians(ryaw), ext[0], ext[1]))
            objs.append({"id": oid, "type": kind, "parked": parked, "model": model,
                         "length": 2 * ext[0], "width": 2 * ext[1], "height": 2 * ext[2],
                         "X": X, "Y": Y, "Z": Z, "Yaw": yaw_g * au,
                         "Vx_global": vg[0] * su, "Vy_global": vg[1] * su, "Speed": math.hypot(*vg) * su,
                         "rel_x": x, "rel_y": y, "rel_yaw": ryaw * au, "rel_vx": rvx * su, "rel_vy": rvy * su,
                         "dist": dist, "gap": gap, "_z0": Z - ext[2] - EZ, "_z1": Z + ext[2] - EZ})

        if types & {"vehicle", "walker"}:
            for a in snap:
                if a.id == self.ego.id:
                    continue
                k = self._kinds.get(a.id, False)
                if k is False:
                    k = self._kind(a.id)
                    if k is not False:  # False: not known to the client yet, ask again next frame
                        self._kinds[a.id] = k
                if not k or k[0] not in types:  # None: not an obstacle; False: not known yet
                    continue
                kind, model, bb = k
                tf = a.get_transform()
                # (transform() overwrites the point it is given: pass a copy)
                cw = tf.transform(carla.Location(bb.location.x, bb.location.y, bb.location.z))
                if cw.distance(rw) > RANGE_M + 10.0:
                    continue
                v = a.get_velocity()
                add(a.id, kind, False, model, (bb.extent.x, bb.extent.y, bb.extent.z),
                    self._global(cw.x, cw.y, cw.z), self._global_yaw(tf.rotation.yaw + bb.rotation.yaw),
                    self._global_vec(v.x, v.y, v.z))
        for oid, model, c, yaw, ext in self._parked:
            if math.hypot(c[0] - rw.x, c[1] - rw.y) > RANGE_M + 10.0:
                continue
            add(oid, "vehicle", True, model, ext, self._global(*c), self._global_yaw(yaw), (0.0, 0.0))
        objs.sort(key=lambda o: o["dist"])
        scene["objects"] = objs
        scene["collisions"] = self._collisions(objs, eh)
        for o in objs:
            del o["_z0"], o["_z1"]
        if self.s.get("lane") or (self.s.get("record") or {}).get("lane"):
            scene["lane"] = self._lane(rw, eyaw, EX, EY, rel)
        if any(c["name"] in self.algo_sensors for c in self.sensor_cfgs):
            scene["sensors"] = {c["name"]: {"type": c["type"], "data": self._convert(c, d)}
                                for c, d in zip(self.sensor_cfgs, self.datas or [None] * len(self.sensor_cfgs))
                                if c["name"] in self.algo_sensors}
        self.latest, self._view, self._record_view = scene, None, None
        return scene

    def _select(self, sel, sensors):
        sc = self.latest
        v = {"t": sc["t"], "frame": sc["frame"],
             "ego": {k: sc["ego"][k] for k in sel.get("ego") or () if k in sc["ego"]}}
        keys = [k for k in OBJECT_KEYS if k in ALWAYS_OBJECT_KEYS or k in (sel.get("objects") or ())]
        v["objects"] = [{k: o[k] for k in keys} for o in sc["objects"]]
        if sel.get("lane") and "lane" in sc:
            v["lane"] = None if sc["lane"] is None else {k: sc["lane"][k] for k in sel["lane"] if k in sc["lane"]}
        if self.s.get("collision", "log") != "off":
            v["collisions"] = sc["collisions"]
        if sensors and "sensors" in sc:
            v["sensors"] = sc["sensors"]
        return v

    def view(self):
        """The keys of the latest scene selected for the algorithm."""
        if self._view is None and self.latest is not None:
            self._view = self._select(self.s, True)
        return self._view

    def record_view(self):
        """The keys of the latest scene selected for the records (no sensor data)."""
        if self._record_view is None and self.latest is not None:
            self._record_view = self._select(self.s.get("record") or {}, False)
        return self._record_view

    def _kind(self, actor_id):
        a = self.world.get_actor(actor_id)
        if a is None:
            return False
        t = a.type_id
        if t.startswith("vehicle."):
            return "vehicle", t, a.bounding_box
        if t.startswith("walker.pedestrian"):
            return "walker", t, a.bounding_box
        return None

    def _collisions(self, objs, eh):
        """Objects touching the ego's box (CarSim's car drives through them in CARLA)."""
        hits, touching = [], set()
        for o in objs:
            if o["gap"] > 0.0 or o["_z0"] > 2 * eh or o["_z1"] < 0.05:  # apart, above or below the car (bridges)
                continue
            touching.add(o["id"])
            hits.append({"id": o["id"], "type": o["type"], "model": o["model"], "new": o["id"] not in self._touching})
        self._touching = touching
        return hits

    def _lane(self, rw, eyaw, EX, EY, rel):
        wp = self.map.get_waypoint(rw, project_to_road=True, lane_type=carla.LaneType.Driving)
        if wp is None or wp.transform.location.distance(rw) > wp.lane_width / 2 + 1.5:
            return None  # not on a driving lane (parking lot, off the road)
        au, su = self.units.angle, self.units.speed
        wps, cur, d = [wp], wp, 0.0
        while d + LANE_STEP_M <= RANGE_M + 1e-6:
            nxt = cur.next(LANE_STEP_M)
            if not nxt:
                break
            # At a split, keep the branch that turns least.
            cur = min(nxt, key=lambda n: abs(_wrap(n.transform.rotation.yaw - cur.transform.rotation.yaw)))
            wps.append(cur)
            d += LANE_STEP_M
        glob = [self._global(p.transform.location.x, p.transform.location.y, p.transform.location.z)[:2] for p in wps]
        yaws = [self._global_yaw(p.transform.rotation.yaw) for p in wps]
        curv = [math.radians(_wrap(yaws[i + 1] - yaws[i])) / LANE_STEP_M for i in range(len(wps) - 1)]
        curv.append(curv[-1] if curv else 0.0)
        _, ly = rel(glob[0][0] - EX, glob[0][1] - EY)
        junction = next((i * LANE_STEP_M for i, p in enumerate(wps) if p.is_junction), None)

        def side(n):
            if n is None or n.lane_type != carla.LaneType.Driving:
                return "none"
            return "same" if (n.lane_id > 0) == (wp.lane_id > 0) else "opposite"

        def marking(m):
            name = str(m.type)
            return "None" if name.upper() == "NONE" else name

        light, light_dist = None, None
        try:
            lms = wp.get_landmarks_of_type(RANGE_M, "1000001", False)
            for lm in sorted(lms, key=lambda m: m.distance):
                tl = self.world.get_traffic_light(lm)
                if tl is not None:
                    state = str(tl.get_state()).lower()
                    light, light_dist = (state if state in ("red", "yellow", "green") else None), lm.distance
                    break
        except RuntimeError:
            pass
        try:
            limit = float(self.ego.get_speed_limit()) / 3.6 * su
        except RuntimeError:
            limit = None
        return {"width": wp.lane_width, "offset": -ly,
                "heading_err": _wrap(eyaw - yaws[0]) * au, "curvature": curv[0],
                "center_rel": [list(rel(X - EX, Y - EY)) for X, Y in glob], "center_global": [list(p) for p in glob],
                "center_curvature": curv,
                "left_marking": marking(wp.left_lane_marking), "right_marking": marking(wp.right_lane_marking),
                "left_lane": side(wp.get_left_lane()), "right_lane": side(wp.get_right_lane()),
                "speed_limit": limit, "in_junction": wp.is_junction, "junction_dist": junction,
                "light_state": light, "light_dist": light_dist}


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

    def _convert(self, c, d):
        """numpy / dict view of one measurement, CarSim axes (x forward,
        y left, z up) and CarSim units (see docs/场景与数据接口.md 4.6)."""
        if d is None:
            return None
        kind, su, au = c["type"], self.units.speed, self.units.angle
        if kind in rigmod.CAMERA_TYPES:
            bgra = np.frombuffer(d.raw_data, dtype=np.uint8).reshape(d.height, d.width, 4)
            if kind == "depth":
                return depth_m(bgra, c)
            if kind == "semantic":
                return bgra[:, :, 2].copy()
            return np.ascontiguousarray(bgra[:, :, 2::-1])
        if kind == "lidar":
            return lidar_iso(d.raw_data)
        if kind == "radar":
            return radar_iso(d.raw_data, su, au)
        if kind == "imu":
            a, g = d.accelerometer, d.gyroscope
            ru = self.units.rate
            return {"accel": [a.x, -a.y, a.z], "gyro": [math.degrees(-g.x) * ru, math.degrees(g.y) * ru,
                                                         math.degrees(-g.z) * ru],
                    "compass": math.degrees(d.compass) * au}
        if kind == "gnss":
            return {"lat": d.latitude, "lon": d.longitude, "alt": d.altitude}
        out = []
        for ev in d:
            e = {"frame": ev.frame}
            other = getattr(ev, "other_actor", None)
            if other is not None:
                e["other_id"], e["other_model"] = other.id, other.type_id
            if hasattr(ev, "crossed_lane_markings"):
                e["markings"] = [str(m.type) for m in ev.crossed_lane_markings]
            out.append(e)
        return out


def depth_m(bgra, cfg):
    """CARLA's depth encoding -> metres, clipped at the camera's max_distance."""
    b = bgra.astype(np.float32)
    m = (b[:, :, 2] + b[:, :, 1] * 256.0 + b[:, :, 0] * 65536.0) / (256.0 ** 3 - 1) * 1000.0
    far = float(cfg.get("attributes", {}).get("max_distance", 0) or 0)
    return np.minimum(m, far) if far > 0 else m


def lidar_iso(raw):
    """CARLA lidar buffer -> N x 4 float32 [x forward, y LEFT, z up, intensity]."""
    p = np.frombuffer(raw, dtype=np.float32).reshape(-1, 4).copy()
    p[:, 1] *= -1.0
    return p


def radar_iso(raw, speed_unit=3.6, angle_unit=1.0):
    """CARLA radar buffer -> N x 4 float32 [distance m, azimuth (+ left),
    elevation, radial velocity]; angles and speed in CarSim units."""
    p = np.frombuffer(raw, dtype=np.float32).reshape(-1, 4)  # velocity, azimuth, altitude, depth (rad, m/s)
    if not len(p):
        return np.zeros((0, 4), np.float32)
    return np.stack([p[:, 3], -np.degrees(p[:, 1]) * angle_unit, np.degrees(p[:, 2]) * angle_unit,
                     p[:, 0] * speed_unit], axis=1).astype(np.float32)


# ----------------------------------------------------------------- recording
class Recorder:
    """The selected scene keys and CarSim exports as CSV files, one row per
    sample (per object in the objects file). paths: {"main", "objects",
    "lane"}; the lane file only when scalar lane keys are selected."""

    def __init__(self, paths, settings, export_names):
        """settings: the "scene" settings; the keys come from its "record" part."""
        s = dict(settings.get("record") or {}, exports_all=settings.get("exports_all", True),
                 exports=settings.get("exports") or [])
        self.ego_keys = [k for k in EGO_KEYS if k in (s.get("ego") or ())]
        self.obj_keys = [k for k in OBJECT_KEYS if k in ALWAYS_OBJECT_KEYS or k in (s.get("objects") or ())]
        self.lane_keys = [k for k in LANE_KEYS if k in (s.get("lane") or ()) and k not in LANE_LISTS]
        self.exports = list(export_names) if s.get("exports_all", True) else \
            [n for n in export_names if n in (s.get("exports") or ())]
        self.files = []
        try:
            self._open(paths)
        except BaseException:
            self.close()  # the files opened before the one that failed
            raise

    def _open(self, paths):
        def open_csv(path, header):
            os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
            f = open(path, "w", newline="", encoding="utf-8")
            w = csv.writer(f)
            w.writerow(header)
            self.files.append(f)
            return w
        self.main = open_csv(paths["main"], ["t", "frame"] + ["ego_" + k for k in self.ego_keys] + self.exports)
        self.obj = open_csv(paths["objects"], ["t", "frame"] + self.obj_keys)
        self.lane = open_csv(paths["lane"], ["t", "frame"] + self.lane_keys) if self.lane_keys else None
        self.paths = [f.name for f in self.files]

    @staticmethod
    def run_paths(log_path):
        base = os.path.splitext(log_path)[0]
        return {"main": log_path, "objects": base + "_objects.csv", "lane": base + "_lane.csv"}

    def write(self, scene, exports):
        t, fr = round(scene["t"], 6), scene["frame"]
        e = scene["ego"]
        self.main.writerow([t, fr] + [e.get(k) for k in self.ego_keys] + [exports.get(n) for n in self.exports])
        for o in scene["objects"]:
            self.obj.writerow([t, fr] + [o.get(k) for k in self.obj_keys])
        if self.lane is not None and scene.get("lane"):
            self.lane.writerow([t, fr] + [scene["lane"].get(k) for k in self.lane_keys])

    def selected_exports(self, exports):
        return {n: exports.get(n) for n in self.exports}

    def close(self):
        for f in self.files:
            try:
                f.close()
            except OSError:
                pass
        self.files = []


def gui_view(scene, ego_box=(0.0, 0.0), max_objects=60):
    """Compact copy for the GUI: every object key (the display does not
    depend on the selection), no sensor arrays, rounded numbers. ego_box:
    centre of the ego's box in the ego frame (the origin is the reference point)."""
    if not scene:
        return None
    r = lambda v: round(float(v), 2) if isinstance(v, (int, float)) and not isinstance(v, bool) else v
    objs = scene["objects"]
    # Moving objects first (parked cars nearby must not push a car further
    # away out of the list), then the nearest parked ones.
    keep = [o for o in objs if not o["parked"]][:max_objects]
    keep += [o for o in objs if o["parked"]][:max_objects - len(keep)]
    keep.sort(key=lambda o: o["dist"])
    out = {"objects": [{k: (o[k] if k == "id" else r(o[k])) for k in OBJECT_KEYS} for o in keep],
           "n_objects": len(objs),
           "ego": dict({k: r(v) for k, v in scene["ego"].items()}, box_x=r(ego_box[0]), box_y=r(ego_box[1])),
           "collisions": scene.get("collisions", [])}
    lane = scene.get("lane")
    if lane:
        out["lane"] = {k: r(v) for k, v in lane.items() if k not in LANE_LISTS}
        out["lane"]["center_rel"] = [[r(p[0]), r(p[1])] for p in lane["center_rel"]]
    if "sensors" in scene:
        out["sensors"] = {n: {"type": v["type"],
                              "shape": list(getattr(v["data"], "shape", ())) or (len(v["data"]) if isinstance(v["data"], list) else None)}
                          for n, v in scene["sensors"].items()}
    return out
