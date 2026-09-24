"""Synchronised multi-sensor data collection.

The world runs in synchronous mode; after every world.tick() the collector
waits for each sensor's measurement *of that frame* (so all modalities in a
sample share one timestamp), then hands the sample to a writer thread. The
simulation only blocks when the writer falls behind (bounded queue), which
keeps samples complete instead of silently dropping them.

Output layout (one folder per session):
    <out_dir>/<session>/
        meta.json               map, weather, rig, settings, conventions
        calib.json              per sensor: extrinsic (sensor->ego 4x4) + camera K
        <sensor_name>/<frame>.jpg|png|bin|npy|csv
        labels/<frame>.json     3D boxes (ego frame) + 2D boxes per camera
        ego/<frame>.json        pose, velocity, acceleration, control, CarSim state

Safety: start() refuses to run when the estimated size exceeds the free disk
space minus a reserve, and stop conditions (frames / seconds / GB) are
enforced every frame.
"""

import json
import math
import os
import queue
import shutil
import threading
import time

import numpy as np
import carla

import rig as rigmod

DISK_RESERVE_GB = 10.0


def disk_info(path):
    p = os.path.abspath(path or ".")
    while not os.path.exists(p):
        p = os.path.dirname(p)
    u = shutil.disk_usage(p)
    return {"path": p, "free_gb": u.free / 1e9, "total_gb": u.total / 1e9}


def _tf_dict(t):
    return {"x": t.location.x, "y": t.location.y, "z": t.location.z,
            "roll": t.rotation.roll, "pitch": t.rotation.pitch, "yaw": t.rotation.yaw}


def _vec(v):
    return [v.x, v.y, v.z]


def camera_K(w, h, fov):
    f = w / (2.0 * math.tan(math.radians(fov) / 2.0))
    return [[f, 0.0, w / 2.0], [0.0, f, h / 2.0], [0.0, 0.0, 1.0]]


class DataCollector:
    def __init__(self, world, ego, sensors, cfg, extra_state=None, emit=None):
        """sensors: rig sensor dicts; cfg: collect settings; extra_state():
        dict merged into ego/<frame>.json (e.g. CarSim exports)."""
        self.world, self.ego, self.cfg = world, ego, cfg
        self.sensor_cfgs = [s for s in sensors if s.get("enabled", True)]
        self.extra_state = extra_state or (lambda: {})
        self.emit = emit or (lambda m: None)
        self.actors, self.queues = [], []
        self.q = queue.Queue(maxsize=48)
        self.writer = None
        self.frames = self.bytes = 0
        self.errors = []
        self.done = False
        self.stop_reason = ""
        self._t0 = None

    # ----------------------------------------------------------- planning
    def estimate(self):
        fmt = self.cfg.get("image_format", "jpg")
        per_frame = sum(rigmod.bytes_per_frame(s, fmt) for s in self.sensor_cfgs) + 20000  # labels + ego
        hz = 1.0 / (self.cfg["frame_dt"] * max(1, int(self.cfg.get("capture_every", 1))))
        max_frames = int(self.cfg.get("max_frames", 0) or 0)
        max_s = float(self.cfg.get("max_seconds", 0) or 0)
        n = max_frames if max_frames else (int(max_s * hz) if max_s else 0)
        total = per_frame * n if n else None
        cap = float(self.cfg.get("max_gb", 0) or 0) * 1e9
        if cap and (total is None or total > cap):
            total = cap
        return {"bytes_per_frame": per_frame, "mb_per_s": per_frame * hz / 1e6,
                "gb_per_hour": per_frame * hz * 3600 / 1e9, "frames": n,
                "total_gb": (total / 1e9) if total else None}

    # ---------------------------------------------------------- lifecycle
    def start(self):
        c = self.cfg
        est = self.estimate()
        di = disk_info(c["out_dir"])
        if est["total_gb"] is None:
            raise RuntimeError("必须设置停止条件（帧数、时长或容量上限），防止写满磁盘")
        if est["total_gb"] > di["free_gb"] - DISK_RESERVE_GB:
            raise RuntimeError("预计需要 %.1f GB，磁盘只剩 %.1f GB（需保留 %.0f GB），请减少采集量"
                               % (est["total_gb"], di["free_gb"], DISK_RESERVE_GB))
        name = c.get("session") or time.strftime("session_%Y%m%d_%H%M%S")
        self.root = os.path.join(c["out_dir"], name)
        os.makedirs(os.path.join(self.root, "labels"), exist_ok=True)
        os.makedirs(os.path.join(self.root, "ego"), exist_ok=True)

        bl = self.world.get_blueprint_library()
        calib = {"ego_frame": "CARLA vehicle frame: origin at ground under the car centre, "
                              "x forward, y right, z up (left-handed), meters",
                 "sensors": {}}
        for s in self.sensor_cfgs:
            bp = bl.find(rigmod.SENSOR_BLUEPRINTS[s["type"]])
            for k, v in s.get("attributes", {}).items():
                if bp.has_attribute(k):
                    bp.set_attribute(k, str(v))
            if s["type"] == "lidar" and bp.has_attribute("rotation_frequency"):
                # One full sweep per captured frame.
                bp.set_attribute("rotation_frequency", str(1.0 / c["frame_dt"]))
            tf = carla.Transform(carla.Location(s["x"], s["y"], s["z"]),
                                 carla.Rotation(pitch=s["pitch"], yaw=s["yaw"], roll=s["roll"]))
            actor = self.world.spawn_actor(bp, tf, attach_to=self.ego)
            qq = queue.Queue()
            actor.listen(qq.put)
            self.actors.append(actor)
            self.queues.append(qq)
            os.makedirs(os.path.join(self.root, s["name"]), exist_ok=True)
            entry = {"type": s["type"], "blueprint": bp.id, "extrinsic_sensor_to_ego": tf.get_matrix(),
                     "mount": {k: s[k] for k in ("x", "y", "z", "roll", "pitch", "yaw")},
                     "attributes": s.get("attributes", {})}
            if s["type"] in rigmod.CAMERA_TYPES:
                a = s["attributes"]
                entry["K"] = camera_K(int(a["image_size_x"]), int(a["image_size_y"]), float(a["fov"]))
            calib["sensors"][s["name"]] = entry
        with open(os.path.join(self.root, "calib.json"), "w") as f:
            json.dump(calib, f, indent=2)
        w = self.world
        meta = {"map": w.get_map().name, "weather": {k: getattr(w.get_weather(), k) for k in
                                                     ("cloudiness", "precipitation", "sun_altitude_angle", "fog_density", "wetness")},
                "ego_blueprint": self.ego.type_id, "collect": c, "rig": self.sensor_cfgs,
                "started": time.strftime("%Y-%m-%d %H:%M:%S"),
                "formats": {"lidar_bin": "float32 [x, y, z, intensity] in the lidar frame (CARLA axes)",
                            "depth_png": "raw CARLA encoding: depth_m = (R + G*256 + B*65536) / (256^3 - 1) * 1000",
                            "semantic_png": "R channel = CARLA semantic tag",
                            "instance_png": "R = semantic tag, G + B*256 = object id",
                            "radar": "rows of [velocity m/s, azimuth rad, altitude rad, depth m]"}}
        with open(os.path.join(self.root, "meta.json"), "w") as f:
            json.dump(meta, f, indent=2, ensure_ascii=False)
        self.writer = threading.Thread(target=self._write_loop, daemon=True)
        self.writer.start()
        self._t0 = time.time()
        self._last_emit = 0.0
        return {"root": self.root, "estimate": est, "disk": di}

    def stop(self):
        for a in self.actors:
            try:
                a.stop()
                a.destroy()
            except RuntimeError:
                pass
        self.actors = []
        if self.writer is not None:
            self.q.put(None)
            self.writer.join(timeout=60)
            self.writer = None
        self._emit_stats(force=True)

    # --------------------------------------------------------------- tick
    def on_tick(self, frame):
        """Call right after world.tick(); frame = its return value."""
        if self.done:
            return
        datas = []
        for s, qq in zip(self.sensor_cfgs, self.queues):
            d = None
            deadline = time.time() + 5.0
            while time.time() < deadline:
                try:
                    d = qq.get(timeout=max(0.01, deadline - time.time()))
                except queue.Empty:
                    break
                if d.frame >= frame:
                    break
            if s["type"] in ("collision", "lane_invasion"):
                d = d if (d is not None and d.frame == frame) else None  # event sensors fire rarely
            elif d is None or d.frame != frame:
                self.errors.append("%s: 第 %d 帧数据缺失" % (s["name"], frame))
                d = None
            datas.append(d)
        every = max(1, int(self.cfg.get("capture_every", 1)))
        if frame % every == 0:
            sample = (frame, datas, self._ego_state(frame), self._labels() if self.cfg.get("labels", True) else None)
            self.q.put(sample)       # blocks if the writer falls behind
            self.frames += 1
        self._check_limits()
        self._emit_stats()

    def _check_limits(self):
        c = self.cfg
        if c.get("max_frames") and self.frames >= int(c["max_frames"]):
            self.done, self.stop_reason = True, "达到帧数上限"
        elif c.get("max_seconds") and time.time() - self._t0 >= float(c["max_seconds"]):
            self.done, self.stop_reason = True, "达到时长上限"
        elif c.get("max_gb") and self.bytes >= float(c["max_gb"]) * 1e9:
            self.done, self.stop_reason = True, "达到容量上限"
        elif disk_info(self.root)["free_gb"] < DISK_RESERVE_GB:
            self.done, self.stop_reason = True, "磁盘剩余空间不足，已自动停止"

    def _emit_stats(self, force=False):
        now = time.time()
        if not force and now - self._last_emit < 1.0:
            return
        self._last_emit = now
        el = max(1e-6, now - self._t0) if self._t0 else 1.0
        self.emit({"event": "collect_stats", "frames": self.frames, "bytes": self.bytes,
                   "mb_per_s": self.bytes / el / 1e6, "elapsed": el, "queue": self.q.qsize(),
                   "errors": self.errors[-5:], "root": getattr(self, "root", ""),
                   "done": self.done, "reason": self.stop_reason})

    # ------------------------------------------------------------- ground truth
    def _ego_state(self, frame):
        v = self.ego
        t = v.get_transform()
        c = v.get_control()
        snap = self.world.get_snapshot()
        st = {"frame": frame, "timestamp": snap.timestamp.elapsed_seconds, "pose": _tf_dict(t),
              "velocity": _vec(v.get_velocity()), "angular_velocity": _vec(v.get_angular_velocity()),
              "acceleration": _vec(v.get_acceleration()),
              "control": {"throttle": c.throttle, "steer": c.steer, "brake": c.brake, "gear": c.gear}}
        try:
            st.update(self.extra_state())
        except Exception as e:  # never lose a frame over optional data
            st["extra_error"] = str(e)
        return st

    def _labels(self):
        ego_tf = self.ego.get_transform()
        inv = np.array(ego_tf.get_inverse_matrix())
        radius = float(self.cfg.get("label_radius", 80.0))
        objs = []
        for a in self.world.get_actors():
            tid = a.type_id
            if a.id == self.ego.id or not (tid.startswith("vehicle.") or tid.startswith("walker.pedestrian")):
                continue
            t = a.get_transform()
            if t.location.distance(ego_tf.location) > radius:
                continue
            bb = a.bounding_box
            c_world = t.transform(bb.location)
            c_ego = inv @ np.array([c_world.x, c_world.y, c_world.z, 1.0])
            objs.append({"id": a.id, "type_id": tid,
                         "class": "pedestrian" if tid.startswith("walker") else a.attributes.get("base_type", "car") or "car",
                         "center_ego": c_ego[:3].tolist(), "extent": _vec(bb.extent),
                         "yaw_ego": (t.rotation.yaw - ego_tf.rotation.yaw + 180.0) % 360.0 - 180.0,
                         "world": _tf_dict(t), "velocity": _vec(a.get_velocity())})
        return {"objects": objs}

    # ----------------------------------------------------------------- writer
    def _write_loop(self):
        fmt = self.cfg.get("image_format", "jpg")
        quality = int(self.cfg.get("jpg_quality", 90))
        pc_fmt = self.cfg.get("pointcloud_format", "bin")
        from PIL import Image
        while True:
            item = self.q.get()
            if item is None:
                return
            frame, datas, ego, labels = item
            try:
                for s, d in zip(self.sensor_cfgs, datas):
                    if d is None:
                        continue
                    base = os.path.join(self.root, s["name"], "%06d" % frame)
                    k = s["type"]
                    if k in rigmod.CAMERA_TYPES:
                        bgra = np.frombuffer(d.raw_data, dtype=np.uint8).reshape(d.height, d.width, 4)
                        rgb = np.ascontiguousarray(bgra[:, :, 2::-1])
                        if k == "rgb" and fmt == "jpg":
                            path = base + ".jpg"
                            Image.fromarray(rgb).save(path, quality=quality)
                        else:  # depth / segmentation must stay lossless
                            path = base + ".png"
                            Image.fromarray(rgb).save(path)
                    elif k == "lidar":
                        pts = np.frombuffer(d.raw_data, dtype=np.float32).reshape(-1, 4)
                        path = base + (".npy" if pc_fmt == "npy" else ".bin")
                        if pc_fmt == "npy":
                            np.save(path, pts)
                        else:
                            pts.tofile(path)
                    elif k == "radar":
                        pts = np.frombuffer(d.raw_data, dtype=np.float32).reshape(-1, 4)
                        path = base + ".csv"
                        np.savetxt(path, pts, delimiter=",", fmt="%.4f")
                    elif k == "imu":
                        ego["imu"] = {"accelerometer": _vec(d.accelerometer), "gyroscope": _vec(d.gyroscope),
                                      "compass": d.compass}
                        continue
                    elif k == "gnss":
                        ego["gnss"] = {"lat": d.latitude, "lon": d.longitude, "alt": d.altitude}
                        continue
                    else:
                        ego.setdefault("events", []).append({"sensor": s["name"], "frame": d.frame})
                        continue
                    self.bytes += os.path.getsize(path)
                for sub, obj in (("ego", ego), ("labels", labels)):
                    if obj is None:
                        continue
                    path = os.path.join(self.root, sub, "%06d.json" % frame)
                    with open(path, "w") as f:
                        json.dump(obj, f)
                    self.bytes += os.path.getsize(path)
            except Exception as e:
                self.errors.append("写入第 %d 帧失败：%s" % (frame, e))
