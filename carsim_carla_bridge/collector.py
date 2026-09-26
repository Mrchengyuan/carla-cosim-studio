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
        labels/<frame>.json     3D boxes (CARLA ego frame), for the KITTI / nuScenes export
        ego/<frame>.json        CARLA pose, velocity, acceleration, control, imu / gnss
        frames/<frame>.json     CarSim time, the selected CarSim exports and scene keys
                                (ego, obstacles, lane, collisions; CarSim frames and units)
        frames.csv, objects.csv, lane.csv   the same for every sample, one table each

Safety: start() refuses to run when the estimated size exceeds the free disk
space minus a reserve, and stop conditions (frames / seconds / GB) are
enforced every frame. Near a GB limit, queued frames are flushed before
another one is accepted so queued writes cannot overshoot the limit.
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


EVENT_TYPES = ("collision", "lane_invasion")  # fire now and then, not every frame
_WINDOWS_RESERVED = ({"CON", "PRN", "AUX", "NUL"}
                     | {"COM%d" % i for i in range(1, 10)}
                     | {"LPT%d" % i for i in range(1, 10)})
_SESSION_FILES = {"ego", "labels", "frames", "calib.json", "meta.json", "frames.csv", "objects.csv", "lane.csv"}


def _check_name(name, what):
    """Keep session and sensor names as one portable directory component."""
    bad = '/\\:*?"<>|'
    if (not isinstance(name, str) or not name or name in (".", "..")
            or name != name.strip() or name.endswith(".")
            or any(c in bad or ord(c) < 32 for c in name)
            or name.split(".", 1)[0].upper() in _WINDOWS_RESERVED):
        raise ValueError("%s只能是普通文件夹名，不能包含路径或系统保留字符：%r" % (what, name))


class DataCollector:
    def __init__(self, world, ego, sensors, cfg, extra_state=None, emit=None, shared=None,
                 scene=None, exports=None, export_names=(), ref_local=None):
        """sensors: rig sensor dicts; cfg: collect settings; extra_state():
        dict merged into ego/<frame>.json; shared: a scene.SceneProvider that
        already runs these sensors (its .frame / .datas are read instead of
        spawning a second set); scene: the run's SceneProvider, exports():
        the CarSim exports of the step, export_names: their names (for
        frames/ and the CSV files); ref_local: the CarSim reference point in the
        CARLA vehicle frame (sensor mounts are relative to it; front axle when None)."""
        self.ref_local = ref_local
        self.shared = shared
        self.scene, self.exports = scene, exports or (lambda: {})
        self.export_names = list(export_names)
        self.recorder = None
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
        self._frame0 = self._last_frame = None  # simulation time, for max_seconds

    # ----------------------------------------------------------- planning
    def estimate(self):
        if self.cfg.get("session"):
            _check_name(self.cfg["session"], "场景名称")
        seen = set()
        for s in self.sensor_cfgs:
            name = s.get("name")
            _check_name(name, "传感器名称")
            if name.casefold() in _SESSION_FILES:
                raise ValueError("传感器名称与采集文件冲突：%s" % name)
            if name.casefold() in seen:
                raise ValueError("传感器名称重复：%s" % name)
            seen.add(name.casefold())
        fmt = self.cfg.get("image_format", "jpg")
        per_frame = sum(rigmod.bytes_per_frame(s, fmt, self.cfg["frame_dt"]) for s in self.sensor_cfgs) + 20000  # labels + ego
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
        # Never write into an existing session: frames of two runs (and two
        # rigs' calibrations) would mix. A reused name gets a suffix.
        base_root, k = os.path.join(c["out_dir"], name), 1
        self.root = base_root
        while os.path.exists(self.root):
            k += 1
            self.root = "%s_%d" % (base_root, k)
        os.makedirs(os.path.join(self.root, "labels"), exist_ok=True)
        os.makedirs(os.path.join(self.root, "ego"), exist_ok=True)
        if self.scene is not None:
            import scene as scn
            os.makedirs(os.path.join(self.root, "frames"), exist_ok=True)
            self.recorder = scn.Recorder({"main": os.path.join(self.root, "frames.csv"),
                                          "objects": os.path.join(self.root, "objects.csv"),
                                          "lane": os.path.join(self.root, "lane.csv")},
                                         self.scene.s, self.export_names)

        bl = self.world.get_blueprint_library()
        if self.ref_local is None:
            from bridge import front_axle_local
            self.ref_local = front_axle_local(self.ego)
        ref = [float(v) for v in self.ref_local]
        calib = {"ego_frame": "CARLA vehicle frame: origin at ground under the car centre, "
                              "x forward, y right, z up (left-handed), meters (extrinsic_sensor_to_ego)",
                 "mount_frame": "CarSim vehicle frame: origin at the CarSim reference point, x forward, y LEFT, "
                                "z up, meters; yaw + = left, pitch + = nose down, deg (mount)",
                 "reference_point_in_ego_frame": ref,
                 "sensors": {}}
        for s in self.sensor_cfgs:
            bp = bl.find(rigmod.SENSOR_BLUEPRINTS[s["type"]])
            for k, v in s.get("attributes", {}).items():
                if bp.has_attribute(k):
                    bp.set_attribute(k, str(v))
            attrs = dict(s.get("attributes", {}))
            if s["type"] == "lidar" and bp.has_attribute("rotation_frequency"):
                # One full sweep per captured frame.
                bp.set_attribute("rotation_frequency", str(1.0 / c["frame_dt"]))
                attrs["rotation_frequency"] = 1.0 / c["frame_dt"]
            tf = rigmod.mount_transform(s, ref)
            if self.shared is None:
                actor = self.world.spawn_actor(bp, tf, attach_to=self.ego)
                qq = queue.Queue()
                actor.listen(qq.put)
                self.actors.append(actor)
                self.queues.append(qq)
            os.makedirs(os.path.join(self.root, s["name"]), exist_ok=True)
            entry = {"type": s["type"], "blueprint": bp.id, "extrinsic_sensor_to_ego": tf.get_matrix(),
                     "mount": {k: s[k] for k in ("x", "y", "z", "roll", "pitch", "yaw")},
                     "attributes": attrs}
            if s["type"] in rigmod.CAMERA_TYPES:
                a = s["attributes"]
                entry["K"] = camera_K(int(a["image_size_x"]), int(a["image_size_y"]), float(a["fov"]))
            calib["sensors"][s["name"]] = entry
        with open(os.path.join(self.root, "calib.json"), "w", encoding="utf-8") as f:
            json.dump(calib, f, indent=2)
        w = self.world
        meta = {"map": w.get_map().name, "weather": {k: getattr(w.get_weather(), k) for k in
                                                     ("cloudiness", "precipitation", "sun_altitude_angle", "fog_density", "wetness")},
                "ego_blueprint": self.ego.type_id, "collect": c, "rig": self.sensor_cfgs,
                "started": time.strftime("%Y-%m-%d %H:%M:%S"),
                "conventions": "carsim", "units": c.get("units") or {"speed": "km/h", "angle": "deg"},
                "formats": {"lidar_bin": "float32 N x 4 [x forward, y left, z up, intensity] in the lidar frame (CarSim axes)",
                            "depth_npy": "float32 H x W, metres, clipped at the camera's max_distance",
                            "semantic_png": "R channel = CARLA semantic tag",
                            "instance_png": "R = semantic tag, G + B*256 = object id",
                            "radar": "rows of [distance m, azimuth (+ left), elevation, radial velocity], "
                                     "angles and speed in the CarSim units above"}}
        with open(os.path.join(self.root, "meta.json"), "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2, ensure_ascii=False)
        # Raw frames waiting for the writer: at most ~1 GB of them (the nuScenes
        # rig is ~35 MB per frame), however many that is.
        raw = sum(int(s.get("attributes", {}).get("image_size_x", 0) or 0) * int(s.get("attributes", {}).get("image_size_y", 0) or 0) * 4
                  for s in self.sensor_cfgs if s["type"] in rigmod.CAMERA_TYPES) or 1
        self.q = queue.Queue(maxsize=max(4, min(48, int(1e9 // raw))))
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
            # Wait for every queued frame: "finished" must mean "on disk" (the
            # session can be exported or deleted right after).
            self.q.put(None)
            self.writer.join()
            self.writer = None
        if self.recorder is not None:
            self.recorder.close()
            self.recorder = None
        self._emit_stats(force=True)

    # --------------------------------------------------------------- tick
    def on_tick(self, frame):
        """Call right after world.tick(); frame = its return value."""
        if self.done:
            return
        if self._frame0 is None:
            self._frame0 = frame
        every = max(1, int(self.cfg.get("capture_every", 1)))
        if self.cfg.get("max_gb") and frame % every == 0:
            # Queued frames have not counted toward the limit yet. Far from it
            # the writer runs alongside CARLA; near it, wait for the queue so
            # queued writes cannot overshoot the limit.
            cap = float(self.cfg["max_gb"]) * 1e9
            pending = self.q.unfinished_tasks
            written = self.frames - pending
            if written > 0:
                avg = self.bytes / written
            else:
                avg = sum(rigmod.bytes_per_frame(s, self.cfg.get("image_format", "jpg"), self.cfg["frame_dt"])
                          for s in self.sensor_cfgs) + 20000
            if self.bytes + (pending + 1) * avg >= cap:
                self.q.join()
            if self.bytes >= cap:
                self.done, self.stop_reason = True, "达到容量上限"
                self._last_frame = frame
                self._emit_stats()
                return
        datas = []
        if self.shared is not None:
            ok = self.shared.frame == frame and len(self.shared.datas) == len(self.sensor_cfgs)
            datas = list(self.shared.datas) if ok else [None] * len(self.sensor_cfgs)
            if not ok:
                self.errors.append("第 %d 帧传感器数据缺失" % frame)
        for s, qq in zip(self.sensor_cfgs if self.shared is None else [], self.queues):
            if s["type"] in EVENT_TYPES:
                # Event sensors fire rarely: take what is there, never wait.
                events = []
                while True:
                    try:
                        d = qq.get_nowait()
                    except queue.Empty:
                        break
                    if d.frame <= frame:
                        events.append(d)
                datas.append(events or None)
                continue
            d = None
            deadline = time.time() + 5.0
            while time.time() < deadline:
                try:
                    d = qq.get(timeout=max(0.01, deadline - time.time()))
                except queue.Empty:
                    break
                if d.frame >= frame:
                    break
            if d is None or d.frame != frame:
                self.errors.append("%s: 第 %d 帧数据缺失" % (s["name"], frame))
                d = None
            datas.append(d)
        if frame % every == 0:
            sample = (frame, datas, self._ego_state(frame), self._labels() if self.cfg.get("labels", True) else None,
                      self._frame_record(frame))
            self.q.put(sample)       # blocks if the writer falls behind
            self.frames += 1
        self._last_frame = frame
        self._check_limits()
        self._emit_stats()

    def _frame_record(self, frame):
        """frames/<frame>.json: the selected scene keys and CarSim exports, the
        same frame as the sensor data."""
        if self.recorder is None:
            return None
        v = self.scene.view()
        if v is None or v.get("frame") != frame:
            self.errors.append("第 %d 帧场景数据缺失" % frame)
            return None
        rec = {k: v[k] for k in ("t", "frame", "ego", "objects", "lane", "collisions") if k in v}
        rec["exports"] = self.recorder.selected_exports(self.exports())
        return rec

    def _check_limits(self):
        c = self.cfg
        if c.get("max_frames") and self.frames >= int(c["max_frames"]):
            self.done, self.stop_reason = True, "达到帧数上限"
        elif c.get("max_seconds") and (self._last_frame - self._frame0 + 1) * float(c["frame_dt"]) >= float(c["max_seconds"]):
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
        import scene as scn
        units = scn.Units(self.cfg.get("units"))
        fmt = self.cfg.get("image_format", "jpg")
        quality = int(self.cfg.get("jpg_quality", 90))
        pc_fmt = self.cfg.get("pointcloud_format", "bin")
        try:
            from PIL import Image
        except ImportError as e:  # keep draining the queue, or the simulation blocks on it
            Image = None
            self.errors.append("无法保存图像：缺少 Pillow（pip install pillow）：%s" % e)
        while True:
            item = self.q.get()
            if item is None:
                self.q.task_done()
                return
            frame, datas, ego, labels, rec = item
            try:
                for s, d in zip(self.sensor_cfgs, datas):
                    if d is None:
                        continue
                    base = os.path.join(self.root, s["name"], "%06d" % frame)
                    k = s["type"]
                    if k in rigmod.CAMERA_TYPES:
                        bgra = np.frombuffer(d.raw_data, dtype=np.uint8).reshape(d.height, d.width, 4)
                        rgb = np.ascontiguousarray(bgra[:, :, 2::-1])
                        if Image is None:
                            continue
                        if k == "depth":
                            path = base + ".npy"
                            np.save(path, scn.depth_m(bgra, s))
                        elif k == "rgb" and fmt == "jpg":
                            path = base + ".jpg"
                            Image.fromarray(rgb).save(path, quality=quality)
                        else:  # depth / segmentation must stay lossless
                            path = base + ".png"
                            Image.fromarray(rgb).save(path)
                    elif k == "lidar":
                        pts = scn.lidar_iso(d.raw_data)
                        path = base + (".npy" if pc_fmt == "npy" else ".bin")
                        if pc_fmt == "npy":
                            np.save(path, pts)
                        else:
                            pts.tofile(path)
                    elif k == "radar":
                        pts = scn.radar_iso(d.raw_data, units.speed, units.angle)
                        path = base + ".csv"
                        np.savetxt(path, pts, delimiter=",", fmt="%.4f")
                    elif k == "imu":
                        ego["imu"] = {"accelerometer": _vec(d.accelerometer), "gyroscope": _vec(d.gyroscope),
                                      "compass": d.compass}
                        continue
                    elif k == "gnss":
                        ego["gnss"] = {"lat": d.latitude, "lon": d.longitude, "alt": d.altitude}
                        continue
                    else:  # event sensors: every event of this frame
                        for ev in d:
                            rec = {"sensor": s["name"], "frame": ev.frame}
                            other = getattr(ev, "other_actor", None)
                            if other is not None:
                                rec["other_actor"] = {"id": other.id, "type_id": other.type_id}
                            if hasattr(ev, "normal_impulse"):
                                rec["normal_impulse"] = _vec(ev.normal_impulse)
                            if hasattr(ev, "crossed_lane_markings"):
                                rec["crossed_lane_markings"] = [str(m.type) for m in ev.crossed_lane_markings]
                            ego.setdefault("events", []).append(rec)
                        continue
                    self.bytes += os.path.getsize(path)
                if rec is not None:
                    self.recorder.write(rec, rec["exports"])
                for sub, obj in (("ego", ego), ("labels", labels), ("frames", rec)):
                    if obj is None:
                        continue
                    path = os.path.join(self.root, sub, "%06d.json" % frame)
                    with open(path + ".tmp", "w", encoding="utf-8") as f:  # no half-written file after a crash
                        json.dump(obj, f)
                    os.replace(path + ".tmp", path)
                    self.bytes += os.path.getsize(path)
            except Exception as e:
                self.errors.append("写入第 %d 帧失败：%s" % (frame, e))
            del self.errors[:-100]
            self.q.task_done()
