"""Live views for the GUI viewport: several sensors on the ego streamed at
once (camera, semantic / depth / instance, lidar and radar as bird's-eye
images). Each view is one sensor; frames go to the GUI as raw RGB.
"""

import base64
import math
import threading
import time

import numpy as np
import carla

import rig as rigmod

# Camera mounts for the preset viewpoints (CARLA vehicle frame, metres / deg).
CAMERA_MOUNTS = {
    "chase": carla.Transform(carla.Location(x=-6.5, z=2.8), carla.Rotation(pitch=-12)),
    "hood": carla.Transform(carla.Location(x=0.6, z=1.45)),
    "wheel": carla.Transform(carla.Location(x=3.6, y=-2.6, z=0.9), carla.Rotation(pitch=-8, yaw=145)),
    "top": carla.Transform(carla.Location(z=22.0), carla.Rotation(pitch=-90)),
}
CAMERA_KINDS = ("rgb", "depth", "semantic", "instance")
VIEW_KINDS = CAMERA_KINDS + ("lidar", "radar")  # sensors that can be shown as an image

# Height colour map for lidar points: blue (low) -> cyan -> green -> yellow -> red (high).
_STOPS = np.array([[40, 90, 255], [0, 210, 255], [60, 220, 90], [255, 220, 40], [255, 70, 50]], dtype=np.float32)
_LUT = np.stack([np.interp(np.linspace(0, 4, 256), np.arange(5), _STOPS[:, c]) for c in range(3)], 1).astype(np.uint8)


def _transform(m, vehicle=None):
    """Mount -> carla.Transform. A rig sensor's mount ("frame": "carsim") is in
    CarSim's vehicle frame, relative to the reference point "ref"."""
    if m.get("frame") == "carsim" and vehicle is not None:
        from bridge import front_axle_local
        ref = m.get("ref", "front_axle")
        ref_local = front_axle_local(vehicle) if ref == "front_axle" else [float(v) for v in ref]
        return rigmod.mount_transform(m, ref_local)
    return carla.Transform(carla.Location(float(m.get("x", 0)), float(m.get("y", 0)), float(m.get("z", 0))),
                           carla.Rotation(pitch=float(m.get("pitch", 0)), yaw=float(m.get("yaw", 0)),
                                          roll=float(m.get("roll", 0))))


def default_mount(kind, vehicle):
    """Roof-centre lidar and front-bumper radar, fitted to the vehicle."""
    bb = vehicle.bounding_box
    if kind == "lidar":
        return {"x": bb.location.x, "y": 0.0, "z": bb.location.z + bb.extent.z + 0.25}
    return {"x": bb.location.x + bb.extent.x + 0.05, "y": 0.0, "z": max(0.4, bb.location.z)}


class _Bev:
    """Bird's-eye canvas: x forward (up), y right; range rings every 10 m."""

    def __init__(self, size, rng, vehicle):
        self.size, self.rng = size, float(rng)
        img = np.full((size, size, 3), (16, 18, 22), dtype=np.uint8)
        yy, xx = np.mgrid[0:size, 0:size]
        r = np.hypot(xx - size / 2, yy - size / 2) / (size / 2) * self.rng
        for ring in np.arange(10.0, self.rng + 0.1, 10.0):
            img[np.abs(r - ring) < self.rng / size * 0.8] = (48, 54, 62)
        img[size // 2, :] = (34, 38, 44)
        img[:, size // 2] = (34, 38, 44)
        # Ego footprint.
        ext = vehicle.bounding_box.extent
        hx, hy = self.px(ext.x, ext.y)
        lx, ly = self.px(-ext.x, -ext.y)
        img[min(hy, ly):max(hy, ly) + 1, [min(hx, lx), max(hx, lx)]] = (230, 232, 236)
        img[[min(hy, ly), max(hy, ly)], min(hx, lx):max(hx, lx) + 1] = (230, 232, 236)
        self.bg = img

    def px(self, x, y):
        s = self.size / 2
        return int(round(s + y / self.rng * s)), int(round(s - x / self.rng * s))

    def draw(self, x, y, colors, dot=1):
        img = self.bg.copy()
        s = self.size / 2
        u = np.round(s + y / self.rng * s).astype(np.int32)
        v = np.round(s - x / self.rng * s).astype(np.int32)
        for du in range(-(dot // 2), dot // 2 + 1):
            for dv in range(-(dot // 2), dot // 2 + 1):
                uu, vv = u + du, v + dv
                m = (uu >= 0) & (uu < self.size) & (vv >= 0) & (vv < self.size)
                img[vv[m], uu[m]] = colors[m]
        return img


class ViewStreamer:
    def __init__(self, emit):
        self.emit = emit
        self.views = {}          # id -> {"actor", "spec", "period", "last", ...}
        self.lock = threading.Lock()

    def specs(self):
        return [dict(v["spec"]) for v in self.views.values()]

    def stop(self, notify=True):
        with self.lock:
            views, self.views = self.views, {}
        for v in views.values():
            v["dead"] = True  # late callbacks of this actor are ignored
            try:
                v["actor"].stop()
                v["actor"].destroy()
            except RuntimeError:
                pass
        if views and notify:
            # The GUI must not keep showing the last frame of views that are gone.
            self.emit({"event": "views_active", "ids": []})

    def set(self, world, vehicle, specs):
        """Replace all views. spec: {id, kind, mode?, mount?, attrs?, width, height, fps}."""
        for spec in specs:  # validate everything before touching the world
            if spec.get("kind", "rgb") not in VIEW_KINDS:
                raise ValueError("不能作为视图显示的传感器类型：%s" % spec.get("kind"))
            if not spec.get("id"):
                raise ValueError("视图缺少 id")
        ids = [spec["id"] for spec in specs]
        if len(set(ids)) != len(ids):  # the second would replace the first and leak its sensor
            raise ValueError("视图 id 重复：%s" % ids)
        self.stop(notify=False)
        try:
            out = [self._add(world, vehicle, spec) for spec in specs]
        except Exception:
            self.stop(notify=False)  # no half-built set of views
            self.emit({"event": "views_active", "ids": []})  # the old ones are gone too
            raise
        self.emit({"event": "views_active", "ids": [o["id"] for o in out]})
        return out

    def _add(self, world, vehicle, spec):
        bl = world.get_blueprint_library()
        kind = spec.get("kind", "rgb")
        bp = bl.find(rigmod.SENSOR_BLUEPRINTS[kind])
        attrs = dict(rigmod.default_attrs(kind))
        attrs.update(spec.get("attrs") or {})
        w, h = int(spec.get("width", 640)), int(spec.get("height", 360))
        mount = spec.get("mount")
        if kind in CAMERA_KINDS:
            attrs["image_size_x"], attrs["image_size_y"] = w, h
            if not mount and "fov" not in (spec.get("attrs") or {}):
                attrs["fov"] = 60 if spec.get("mode") == "wheel" else 90
            tf = _transform(mount, vehicle) if mount else CAMERA_MOUNTS.get(spec.get("mode", "chase"), CAMERA_MOUNTS["chase"])
        else:
            if kind == "lidar":
                attrs.setdefault("rotation_frequency", 10.0)
            tf = _transform(mount or default_mount(kind, vehicle), vehicle)
        for k, val in attrs.items():
            if bp.has_attribute(k):
                bp.set_attribute(k, str(val))
        v = {"spec": dict(spec), "kind": kind, "period": 1.0 / max(1.0, float(spec.get("fps", 12))),
             "last": 0.0, "mount": tf, "buf": [], "sweep": 0.0}
        if kind in ("lidar", "radar"):
            size = min(w, h)
            rng = float(attrs.get("range", 60.0))
            if kind == "lidar":
                rng = min(rng, 80.0)
                v["sweep"] = 1.0 / max(1.0, float(attrs.get("rotation_frequency", 10.0)))
            else:
                v["sweep"] = 0.1
            v["bev"] = _Bev(size, rng, vehicle)
        v["actor"] = actor = world.spawn_actor(bp, tf, attach_to=vehicle)
        vid = spec["id"]
        with self.lock:
            self.views[vid] = v
        # Bound to this view object, not its id: a late frame of a replaced
        # view must not be drawn with the new view's settings.
        actor.listen(lambda data, v=v, vid=vid: self._on_data(vid, v, data))
        return {"id": vid, "kind": kind, "actor": actor.id}

    # ------------------------------------------------------------ rendering
    def _on_data(self, vid, v, data):
        if v.get("dead"):
            return
        now = time.time()
        try:
            if v["kind"] in ("lidar", "radar"):
                # Collect one full sweep before drawing (the sensor returns a slice per tick).
                v["buf"].append((data.timestamp, self._points(v, data)))
                v["buf"] = [b for b in v["buf"] if data.timestamp - b[0] < v["sweep"] - 1e-6]
            if now - v["last"] < v["period"]:
                return
            v["last"] = now
            rgb = self._render(v, data)
        except Exception as e:  # never kill the sensor thread
            print("view %s render failed: %s" % (vid, e), flush=True)
            return
        self.emit({"event": "frame", "view": vid, "w": int(rgb.shape[1]), "h": int(rgb.shape[0]), "frame": data.frame,
                   "rgb": base64.b64encode(np.ascontiguousarray(rgb).tobytes()).decode("ascii")})

    def _points(self, v, data):
        m = v["mount"]
        yaw = math.radians(m.rotation.yaw)
        if v["kind"] == "lidar":
            p = np.frombuffer(data.raw_data, dtype=np.float32).reshape(-1, 4)
            x, y, z = p[:, 0], p[:, 1], p[:, 2]
            val = z  # colour by height relative to the sensor
        else:
            d = np.frombuffer(data.raw_data, dtype=np.float32).reshape(-1, 4)  # velocity, azimuth, altitude, depth
            vel, az, alt, dep = d[:, 0], d[:, 1], d[:, 2], d[:, 3]
            x, y = dep * np.cos(alt) * np.cos(az), dep * np.cos(alt) * np.sin(az)
            val = vel
        c, s = math.cos(yaw), math.sin(yaw)
        return np.stack([m.location.x + c * x - s * y, m.location.y + s * x + c * y, val], 1)

    def _render(self, v, data):
        kind = v["kind"]
        if kind in CAMERA_KINDS:
            if kind == "semantic":
                data.convert(carla.ColorConverter.CityScapesPalette)
            elif kind == "depth":
                data.convert(carla.ColorConverter.LogarithmicDepth)
            return np.frombuffer(data.raw_data, dtype=np.uint8).reshape(data.height, data.width, 4)[:, :, 2::-1]
        pts = np.concatenate([b[1] for b in v["buf"]]) if v["buf"] else np.zeros((0, 3), np.float32)
        if kind == "lidar":
            idx = np.clip((pts[:, 2] + 2.5) / 4.0 * 255, 0, 255).astype(np.int32)
            return v["bev"].draw(pts[:, 0], pts[:, 1], _LUT[idx], dot=1)
        # Radar: velocity is relative to the sensor: red = closing in, blue = moving
        # away, white = same speed (a static object reads as closing while the ego drives).
        vel = pts[:, 2]
        t = np.clip(np.abs(vel) / 10.0, 0, 1)[:, None]
        white = np.array([235, 235, 235], np.float32)
        tint = np.where(vel[:, None] < 0, np.array([255, 70, 60], np.float32), np.array([70, 140, 255], np.float32))
        cols = (white * (1 - t) + tint * t).astype(np.uint8)
        return v["bev"].draw(pts[:, 0], pts[:, 1], cols, dot=3)
