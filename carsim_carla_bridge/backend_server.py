"""GUI backend: exposes CARLA operations and CarSim co-simulation over TCP.

Protocol: newline-delimited JSON (UTF-8) on localhost.
  request : {"id": 1, "cmd": "load_map", "args": {"name": "Town03"}}
  response: {"id": 1, "ok": true, "result": ...}  or  {"id": 1, "ok": false, "error": "..."}
  event   : {"event": "telemetry" | "log" | "cosim_state" | "progress", ...}

All CARLA calls run on one worker thread; the socket thread only queues
requests and writes replies, so a slow map load never blocks the connection.

    python backend_server.py --port 57100
"""

import argparse
import atexit
import base64
import json
import math
import os
import queue
import random
import signal
import sys
import socket
import threading
import time
import traceback

import carla

import collector as coll
import rig as rigmod
import settings as st
from session import CarlaDriveSession, CoSimSession

WEATHER_PRESETS = [n for n in dir(carla.WeatherParameters)
                   if n[0].isupper() and isinstance(getattr(carla.WeatherParameters, n), carla.WeatherParameters)]
WEATHER_FIELDS = ["cloudiness", "precipitation", "precipitation_deposits", "wind_intensity",
                  "sun_azimuth_angle", "sun_altitude_angle", "fog_density", "fog_distance",
                  "wetness", "fog_falloff", "scattering_intensity", "mie_scattering_scale",
                  "rayleigh_scattering_scale", "dust_storm"]
SENSOR_TYPES = {
    "rgb": "sensor.camera.rgb", "depth": "sensor.camera.depth",
    "semantic": "sensor.camera.semantic_segmentation", "lidar": "sensor.lidar.ray_cast",
    "radar": "sensor.other.radar", "imu": "sensor.other.imu", "gnss": "sensor.other.gnss",
    "collision": "sensor.other.collision", "lane_invasion": "sensor.other.lane_invasion",
}


class Backend:
    def __init__(self):
        self.client = self.world = None
        self.tm = None
        self.ego = None
        self.anchor = None
        self.session = None
        self.cosim_state = "stopped"
        self.spectator_mode = "free"
        self.sensors = {}          # id -> {"actor", "type", "save_dir", "count", "file"}
        self.traffic = {"vehicles": [], "walkers": [], "controllers": []}
        self.idle_tick = False     # tick the world ourselves when sync mode is on and idle
        self.frame_dt = 0.05
        self.spec_cache = {}
        self.view = None           # {"actor", "fps", "last", "mode"}
        self.collector = None
        self.emit = lambda msg: None
        self.requests = queue.Queue()

    # ------------------------------------------------------------ utilities
    def _need_world(self):
        if self.world is None:
            raise RuntimeError("未连接 CARLA，请先连接")
        return self.world

    def _log(self, msg, level="info"):
        self.emit({"event": "log", "level": level, "msg": msg})

    def _set_cosim_state(self, s, detail=""):
        self.cosim_state = s
        self.emit({"event": "cosim_state", "state": s, "detail": detail})

    def _alive(self, actor):
        try:
            return actor is not None and actor.is_alive
        except RuntimeError:
            return False

    def _tick_or_wait(self, n=1):
        w = self._need_world()
        for _ in range(n):
            if w.get_settings().synchronous_mode:
                w.tick()
            else:
                w.wait_for_tick(5.0)

    # --------------------------------------------------------------- server
    def cmd_ping(self):
        return "pong"

    def cmd_connect(self, host="localhost", port=2000, timeout=20.0):
        self.client = carla.Client(host, int(port))
        self.client.set_timeout(float(timeout))
        self.world = self.client.get_world()
        self.tm = self.client.get_trafficmanager(8000)
        self.ego = None
        info = self.cmd_world_info()
        info.update({
            "server_version": self.client.get_server_version(),
            "client_version": self.client.get_client_version(),
            "external_api_client": hasattr(carla.Vehicle, "apply_external_state"),
        })
        self._log("已连接 %s:%s，地图 %s" % (host, port, info["map"]))
        return info

    def cmd_world_info(self):
        w = self._need_world()
        s = w.get_settings()
        return {
            "map": w.get_map().name.split("/")[-1],
            "synchronous": s.synchronous_mode,
            "frame_dt": s.fixed_delta_seconds or 0.0,
            "no_rendering": s.no_rendering_mode,
            "weather": self._weather_dict(w.get_weather()),
            "n_actors": len(w.get_actors()),
            "ego_id": self.ego.id if self._alive(self.ego) else 0,
            "spectator_mode": self.spectator_mode,
            "cosim_state": self.cosim_state,
            "idle_tick": self.idle_tick,
        }

    # ---------------------------------------------------------------- world
    def cmd_list_maps(self):
        self._need_world()
        return sorted({m.split("/")[-1] for m in self.client.get_available_maps()})

    def _switch_world(self, load):
        self._need_world()
        self._stop_cosim_if_running()
        self.cmd_clear_traffic()
        self._drop_ego_refs()
        # The traffic manager runs inside this process and keeps its vehicle
        # registry across a world change; load_world() then segfaults in
        # libcarla. Shut it down first and start a fresh one afterwards.
        if self.tm is not None:
            self.tm.shut_down()
            self.tm = None
        self.client.set_timeout(180.0)
        try:
            self.world = load()
        finally:
            self.client.set_timeout(20.0)
            self.tm = self.client.get_trafficmanager(8000)
        return self.cmd_world_info()

    def cmd_load_map(self, name):
        self._log("正在加载地图 %s ..." % name)
        return self._switch_world(lambda: self.client.load_world(name))

    def cmd_reload_world(self):
        return self._switch_world(lambda: self.client.reload_world())

    def cmd_world_settings(self, synchronous=None, frame_dt=None, no_rendering=None, idle_tick=None):
        w = self._need_world()
        s = w.get_settings()
        if synchronous is not None:
            s.synchronous_mode = bool(synchronous)
            self.tm.set_synchronous_mode(bool(synchronous))
        if frame_dt is not None:
            s.fixed_delta_seconds = float(frame_dt) if frame_dt else None
            if frame_dt:
                self.frame_dt = float(frame_dt)
        if no_rendering is not None:
            s.no_rendering_mode = bool(no_rendering)
        w.apply_settings(s)
        if idle_tick is not None:
            self.idle_tick = bool(idle_tick)
        return self.cmd_world_info()

    @staticmethod
    def _weather_dict(wp):
        return {f: getattr(wp, f) for f in WEATHER_FIELDS if hasattr(wp, f)}

    def cmd_list_weathers(self):
        return WEATHER_PRESETS

    def cmd_set_weather(self, preset=None, params=None):
        w = self._need_world()
        wp = getattr(carla.WeatherParameters, preset) if preset else w.get_weather()
        for k, v in (params or {}).items():
            if k in WEATHER_FIELDS:
                setattr(wp, k, float(v))
        w.set_weather(wp)
        # get_weather() only reflects the change after the next server frame.
        return self._weather_dict(wp)

    # ------------------------------------------------------------- vehicles
    def cmd_list_vehicles(self):
        bl = self._need_world().get_blueprint_library().filter("vehicle.*")
        out = []
        def attr(bp, name, default=""):
            # Bool attributes (e.g. has_dynamic_doors) refuse as_str(); only
            # read the string/int ones the GUI shows.
            try:
                return bp.get_attribute(name).as_str() if bp.has_attribute(name) else default
            except RuntimeError:
                return default

        for bp in sorted(bl, key=lambda b: b.id):
            out.append({"id": bp.id, "wheels": int(attr(bp, "number_of_wheels", "4") or 4),
                        "base_type": attr(bp, "base_type"), "generation": attr(bp, "generation"),
                        "colors": bp.get_attribute("color").recommended_values if bp.has_attribute("color") else [],
                        "spec": self.spec_cache.get(bp.id)})
        return out

    def cmd_vehicle_specs(self, ids=None):
        """Spawn each vehicle once far from the road to read wheel geometry."""
        w = self._need_world()
        bl = w.get_blueprint_library()
        ids = ids or [b.id for b in bl.filter("vehicle.*")]
        spots = w.get_map().get_spawn_points()
        base = carla.Transform(carla.Location(spots[0].location.x, spots[0].location.y, 500.0))
        result = {}
        for k, vid in enumerate(ids):
            if vid in self.spec_cache:
                result[vid] = self.spec_cache[vid]
                continue
            self.emit({"event": "progress", "task": "vehicle_specs", "done": k, "total": len(ids), "item": vid})
            tf = carla.Transform(carla.Location(base.location.x + 20.0 * k, base.location.y, 500.0))
            actor = w.try_spawn_actor(bl.find(vid), tf)
            if actor is None:
                continue
            try:
                actor.set_simulate_physics(False)
                pc = actor.get_physics_control()
                inv = actor.get_transform().get_inverse_matrix()
                local = []
                for wh in pc.wheels:
                    p = [wh.position.x / 100.0, wh.position.y / 100.0, wh.position.z / 100.0, 1.0]
                    local.append([sum(inv[r][c] * p[c] for c in range(4)) for r in range(3)])
                bb = actor.bounding_box
                spec = {"wheel_radius_m": [round(wh.radius / 100.0, 3) for wh in pc.wheels],
                        "max_steer_deg": [round(wh.max_steer_angle, 1) for wh in pc.wheels],
                        "mass_kg": round(pc.mass, 1),
                        "length_m": round(2 * bb.extent.x, 3), "width_m": round(2 * bb.extent.y, 3),
                        "height_m": round(2 * bb.extent.z, 3)}
                if len(local) >= 4:
                    spec["wheelbase_m"] = round((local[0][0] + local[1][0]) / 2 - (local[2][0] + local[3][0]) / 2, 3)
                    spec["track_m"] = round(abs(local[1][1] - local[0][1]), 3)
                    spec["front_axle_x_m"] = round((local[0][0] + local[1][0]) / 2, 3)
                self.spec_cache[vid] = result[vid] = spec
            finally:
                actor.destroy()
        self.emit({"event": "progress", "task": "vehicle_specs", "done": len(ids), "total": len(ids), "item": ""})
        return result

    def cmd_list_spawn_points(self):
        pts = self._need_world().get_map().get_spawn_points()
        return [{"index": i, "x": round(p.location.x, 2), "y": round(p.location.y, 2),
                 "z": round(p.location.z, 2), "yaw": round(p.rotation.yaw, 1)} for i, p in enumerate(pts)]

    def _drop_ego_refs(self):
        self.cmd_view_stop()
        for sid in list(self.sensors):
            self.cmd_remove_sensor(sid)
        self.ego = None
        self.anchor = None

    def cmd_spawn_ego(self, blueprint="vehicle.tesla.model3", spawn_index=0, color=""):
        w = self._need_world()
        self._stop_cosim_if_running()
        self.cmd_destroy_ego()
        bp = w.get_blueprint_library().find(blueprint)
        bp.set_attribute("role_name", "hero")
        if color and bp.has_attribute("color"):
            bp.set_attribute("color", color)
        pts = w.get_map().get_spawn_points()
        spawn_index = int(spawn_index) % len(pts)
        self.anchor = pts[spawn_index]
        self.ego = w.spawn_actor(bp, self.anchor)
        self._tick_or_wait(1)
        self._log("已生成主车 %s (id %d) 于 spawn point %d" % (blueprint, self.ego.id, spawn_index))
        return {"id": self.ego.id, "spawn_index": spawn_index}

    def cmd_destroy_ego(self):
        self._stop_cosim_if_running()
        self.cmd_view_stop()
        for sid in list(self.sensors):
            self.cmd_remove_sensor(sid)
        if self._alive(self.ego):
            self.ego.destroy()
        self.ego = None
        return True

    def cmd_ego_autopilot(self, enabled=True):
        if not self._alive(self.ego):
            raise RuntimeError("没有主车")
        if self.cosim_state != "stopped":
            raise RuntimeError("联合仿真运行中，主车由 CarSim 驱动")
        self.ego.set_autopilot(bool(enabled), self.tm.get_port())
        return True

    # ------------------------------------------------------------ spectator
    def cmd_spectator(self, mode="follow"):
        self.spectator_mode = mode
        self._update_spectator()
        return mode

    def _update_spectator(self):
        if self.world is None or self.spectator_mode == "free" or not self._alive(self.ego):
            return
        t = self.ego.get_transform()
        f = t.get_forward_vector()
        if self.spectator_mode == "follow":
            loc = t.location - carla.Location(f.x * 8.0, f.y * 8.0, -3.5)
            rot = carla.Rotation(pitch=-15.0, yaw=t.rotation.yaw)
        elif self.spectator_mode == "top":
            loc = t.location + carla.Location(z=40.0)
            rot = carla.Rotation(pitch=-90.0, yaw=t.rotation.yaw)
        else:  # "side"
            r = t.get_right_vector()
            loc = t.location + carla.Location(-r.x * 6.0, -r.y * 6.0, 1.5)
            rot = carla.Rotation(pitch=-5.0, yaw=t.rotation.yaw + 90.0)
        self.world.get_spectator().set_transform(carla.Transform(loc, rot))

    # -------------------------------------------------------------- traffic
    def cmd_spawn_traffic(self, vehicles=20, walkers=10, seed=0, safe=True):
        w = self._need_world()
        rng = random.Random(int(seed))
        bl = w.get_blueprint_library()
        vbps = [b for b in bl.filter("vehicle.*") if not safe or
                (b.has_attribute("base_type") and b.get_attribute("base_type").as_str() == "car")]
        pts = w.get_map().get_spawn_points()
        rng.shuffle(pts)
        ego_loc = self.ego.get_location() if self._alive(self.ego) else None
        tm_port = self.tm.get_port()
        self.tm.set_synchronous_mode(w.get_settings().synchronous_mode)
        n = 0
        for p in pts:
            if n >= int(vehicles):
                break
            if ego_loc is not None and p.location.distance(ego_loc) < 10.0:
                continue
            bp = rng.choice(vbps)
            if bp.has_attribute("color"):
                bp.set_attribute("color", rng.choice(bp.get_attribute("color").recommended_values))
            bp.set_attribute("role_name", "autopilot")
            a = w.try_spawn_actor(bp, p)
            if a is not None:
                a.set_autopilot(True, tm_port)
                self.traffic["vehicles"].append(a)
                n += 1
        # Walkers: spawn bodies, then AI controllers, then start them.
        wbps = bl.filter("walker.pedestrian.*")
        ctrl_bp = bl.find("controller.ai.walker")
        m = 0
        for _ in range(int(walkers) * 3):
            if m >= int(walkers):
                break
            loc = w.get_random_location_from_navigation()
            if loc is None:
                continue
            bp = rng.choice(wbps)
            if bp.has_attribute("is_invincible"):
                bp.set_attribute("is_invincible", "false")
            body = w.try_spawn_actor(bp, carla.Transform(loc + carla.Location(z=1.0)))
            if body is None:
                continue
            ctrl = w.try_spawn_actor(ctrl_bp, carla.Transform(), body)
            if ctrl is None:
                body.destroy()
                continue
            self.traffic["walkers"].append(body)
            self.traffic["controllers"].append(ctrl)
            m += 1
        self._tick_or_wait(1)
        for c in self.traffic["controllers"]:
            try:
                c.start()
                c.go_to_location(w.get_random_location_from_navigation())
                c.set_max_speed(1.0 + rng.random())
            except RuntimeError:
                pass
        self._log("已生成交通：车辆 %d，行人 %d" % (n, m))
        return {"vehicles": len(self.traffic["vehicles"]), "walkers": len(self.traffic["walkers"])}

    def cmd_clear_traffic(self):
        if self.world is None:
            return True
        for c in self.traffic["controllers"]:
            try:
                c.stop()
            except RuntimeError:
                pass
        ids = [a.id for k in ("controllers", "walkers", "vehicles") for a in self.traffic[k]]
        if ids:
            self.client.apply_batch_sync([carla.command.DestroyActor(i) for i in ids],
                                         self.world.get_settings().synchronous_mode)
        self.traffic = {"vehicles": [], "walkers": [], "controllers": []}
        return True

    # -------------------------------------------------------------- sensors
    def cmd_add_sensor(self, type="rgb", x=1.5, y=0.0, z=2.0, pitch=0.0, yaw=0.0, roll=0.0,
                       attributes=None, save_dir=""):
        w = self._need_world()
        if not self._alive(self.ego):
            raise RuntimeError("请先生成主车")
        bp = w.get_blueprint_library().find(SENSOR_TYPES.get(type, type))
        for k, v in (attributes or {}).items():
            if bp.has_attribute(k):
                bp.set_attribute(k, str(v))
        tf = carla.Transform(carla.Location(float(x), float(y), float(z)),
                             carla.Rotation(float(pitch), float(yaw), float(roll)))
        actor = w.spawn_actor(bp, tf, attach_to=self.ego)
        rec = {"actor": actor, "type": type, "save_dir": save_dir, "count": 0, "file": None,
               "last": "", "spec": {"type": type, "x": x, "y": y, "z": z, "pitch": pitch, "yaw": yaw,
                                    "roll": roll, "attributes": attributes, "save_dir": save_dir}}
        if save_dir:
            os.makedirs(save_dir, exist_ok=True)
        self.sensors[actor.id] = rec
        actor.listen(lambda data, r=rec: self._on_sensor(r, data))
        self._log("已添加传感器 %s (id %d)%s" % (bp.id, actor.id, "，保存到 " + save_dir if save_dir else ""))
        return {"id": actor.id, "blueprint": bp.id}

    def _on_sensor(self, rec, data):
        rec["count"] += 1
        t, d = rec["type"], rec["save_dir"]
        if t in ("rgb", "depth", "semantic"):
            rec["last"] = "%dx%d frame %d" % (data.width, data.height, data.frame)
            if d:
                cc = {"depth": carla.ColorConverter.LogarithmicDepth,
                      "semantic": carla.ColorConverter.CityScapesPalette}.get(t, carla.ColorConverter.Raw)
                data.save_to_disk(os.path.join(d, "%06d.png" % data.frame), cc)
        elif t == "lidar":
            rec["last"] = "%d points frame %d" % (len(data), data.frame)
            if d:
                data.save_to_disk(os.path.join(d, "%06d.ply" % data.frame))
        else:
            if t == "imu":
                row = [data.frame, data.timestamp, data.accelerometer.x, data.accelerometer.y,
                       data.accelerometer.z, data.gyroscope.x, data.gyroscope.y, data.gyroscope.z, data.compass]
                rec["last"] = "acc(%.2f,%.2f,%.2f) gyro(%.3f,%.3f,%.3f)" % tuple(row[2:8])
            elif t == "gnss":
                row = [data.frame, data.timestamp, data.latitude, data.longitude, data.altitude]
                rec["last"] = "lat %.6f lon %.6f" % (data.latitude, data.longitude)
            elif t == "collision":
                row = [data.frame, data.timestamp, data.other_actor.type_id,
                       data.normal_impulse.x, data.normal_impulse.y, data.normal_impulse.z]
                rec["last"] = "撞到 %s" % data.other_actor.type_id
                self._log("碰撞：%s" % data.other_actor.type_id, "warn")
            elif t == "lane_invasion":
                row = [data.frame, data.timestamp, " ".join(str(m.type) for m in data.crossed_lane_markings)]
                rec["last"] = "压线 %s" % row[2]
            else:
                row = [data.frame, data.timestamp]
                rec["last"] = "frame %d" % data.frame
            if d:
                if rec["file"] is None:
                    rec["file"] = open(os.path.join(d, "%s.csv" % t), "a", encoding="utf-8")
                rec["file"].write(",".join(str(x) for x in row) + "\n")

    def cmd_list_sensors(self):
        return [{"id": sid, "type": r["type"], "blueprint": r["actor"].type_id, "count": r["count"],
                 "last": r["last"], "save_dir": r["save_dir"]} for sid, r in self.sensors.items()]

    def cmd_remove_sensor(self, id):
        rec = self.sensors.pop(int(id), None)
        if rec is None:
            return False
        try:
            rec["actor"].stop()
            rec["actor"].destroy()
        except RuntimeError:
            pass
        if rec["file"]:
            rec["file"].close()
        return True

    # ------------------------------------------------------------ live view
    VIEW_MOUNTS = {
        "chase": carla.Transform(carla.Location(x=-6.5, z=2.8), carla.Rotation(pitch=-12)),
        "hood": carla.Transform(carla.Location(x=0.6, z=1.45)),
        "wheel": carla.Transform(carla.Location(x=3.6, y=-2.6, z=0.9), carla.Rotation(pitch=-8, yaw=145)),
        "top": carla.Transform(carla.Location(z=22.0), carla.Rotation(pitch=-90)),
    }

    def cmd_view_start(self, mode="chase", width=640, height=360, fps=15.0, mount=None, fov=90.0):
        """Stream a camera on the ego vehicle to the GUI as raw RGB frames.
        mount: optional {"x","y","z","pitch","yaw","roll"} to preview a rig camera."""
        w = self._need_world()
        if not self._alive(self.ego):
            raise RuntimeError("请先生成主车")
        self.cmd_view_stop()
        bp = w.get_blueprint_library().find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(int(width)))
        bp.set_attribute("image_size_y", str(int(height)))
        bp.set_attribute("fov", str(fov) if mount else ("90" if mode != "wheel" else "60"))
        tf = carla.Transform(carla.Location(mount["x"], mount["y"], mount["z"]),
                             carla.Rotation(pitch=mount.get("pitch", 0.0), yaw=mount.get("yaw", 0.0),
                                            roll=mount.get("roll", 0.0))) if mount else \
            self.VIEW_MOUNTS.get(mode, self.VIEW_MOUNTS["chase"])
        actor = w.spawn_actor(bp, tf, attach_to=self.ego)
        self.view = {"actor": actor, "period": 1.0 / max(1.0, float(fps)), "last": 0.0, "mode": mode,
                     "args": {"mode": mode, "width": width, "height": height, "fps": fps, "mount": mount, "fov": fov}}
        actor.listen(self._on_view_frame)
        return {"id": actor.id, "mode": mode}

    def _on_view_frame(self, img):
        v = self.view
        if v is None:
            return
        now = time.time()
        if now - v["last"] < v["period"]:
            return
        v["last"] = now
        import numpy as np
        rgb = np.frombuffer(img.raw_data, dtype=np.uint8).reshape(img.height, img.width, 4)[:, :, 2::-1]
        self.emit({"event": "frame", "w": img.width, "h": img.height, "frame": img.frame,
                   "rgb": base64.b64encode(np.ascontiguousarray(rgb).tobytes()).decode("ascii")})

    def cmd_view_stop(self):
        v, self.view = self.view, None
        if v is not None:
            try:
                v["actor"].stop()
                v["actor"].destroy()
            except RuntimeError:
                pass
        return True

    # ------------------------------------------------------------- recorder
    def cmd_start_recorder(self, filename="cosim_record.log", additional_data=True):
        self._need_world()
        path = self.client.start_recorder(os.path.abspath(filename), bool(additional_data))
        self._log("开始录制：%s" % filename)
        return path

    def cmd_stop_recorder(self):
        self._need_world()
        self.client.stop_recorder()
        self._log("录制已停止")
        return True

    def cmd_replay(self, filename, start=0.0, duration=0.0, follow_id=0):
        self._need_world()
        self._stop_cosim_if_running()
        return self.client.replay_file(os.path.abspath(filename), float(start), float(duration), int(follow_id))

    def cmd_recorder_info(self, filename):
        self._need_world()
        return self.client.show_recorder_file_info(os.path.abspath(filename), False)[:20000]

    # --------------------------------------------------------------- actors
    def cmd_list_actors(self, filter="*"):
        out = []
        for a in self._need_world().get_actors().filter(filter):
            if a.type_id.startswith(("spectator", "traffic.")):
                continue
            l = a.get_location()
            out.append({"id": a.id, "type_id": a.type_id, "role": a.attributes.get("role_name", ""),
                        "x": round(l.x, 1), "y": round(l.y, 1), "z": round(l.z, 1),
                        "ego": self._alive(self.ego) and a.id == self.ego.id})
        return out

    def cmd_destroy_actor(self, id):
        a = self._need_world().get_actor(int(id))
        if a is None:
            return False
        if self._alive(self.ego) and a.id == self.ego.id:
            return self.cmd_destroy_ego()
        if int(id) in self.sensors:
            return self.cmd_remove_sensor(id)
        return a.destroy()

    # ---------------------------------------------------------------- cosim
    def cmd_default_config(self):
        return st.default_dict()

    def cmd_cosim_start(self, config=None):
        """Start a run: CarSim or CARLA dynamics, any driver, optional collection."""
        w = self._need_world()
        if self.cosim_state in ("running", "paused"):
            raise RuntimeError("已经有仿真在运行")
        d = st.load_dict(None, config or {})
        c = d["carla"]
        col_cfg = dict(d["collect"])
        col_cfg["frame_dt"] = d["sync"]["frame_dt"]
        sensors = d["rig"]["sensors"] or rigmod.build_preset(d["rig"]["preset"], self.spec_cache.get(c["vehicle"]))
        if col_cfg["enabled"]:
            # Validate limits / disk space before touching the world.
            est = coll.DataCollector(w, None, sensors, col_cfg).estimate()
            di = coll.disk_info(col_cfg["out_dir"])
            if est["total_gb"] is None:
                raise RuntimeError("数据采集必须设置停止条件（帧数、时长或容量上限）")
            if est["total_gb"] > di["free_gb"] - coll.DISK_RESERVE_GB:
                raise RuntimeError("预计需要 %.1f GB，磁盘只剩 %.1f GB" % (est["total_gb"], di["free_gb"]))
        # Every run starts with a fresh ego at the chosen spawn point, like a
        # CarSim run starts from its initial conditions. A teleported vehicle
        # keeps stale traffic-manager state (autopilot then brakes forever),
        # so respawn instead, and bring back the user's sensors and view.
        sensor_specs = [dict(r["spec"]) for r in self.sensors.values()]
        view_args = dict(self.view["args"]) if self.view else None
        color = self.ego.attributes.get("color", "") if self._alive(self.ego) else ""
        self.cmd_spawn_ego(c["vehicle"], c["spawn_index"], color)
        for spec in sensor_specs:
            self.cmd_add_sensor(**spec)
        if view_args:
            self.cmd_view_start(**view_args)
        # Settle under PhysX so the bridge reads a resting vehicle.
        self._pre_cosim_settings = w.get_settings()
        s = w.get_settings()
        s.synchronous_mode, s.fixed_delta_seconds = True, d["sync"]["frame_dt"]
        w.apply_settings(s)
        self.tm.set_synchronous_mode(True)
        for _ in range(20):
            w.tick()
        cosim = d["drive"]["dynamics"] == "cosim"
        self.session = CoSimSession(w, self.ego, self.anchor, d) if cosim else \
            CarlaDriveSession(w, self.ego, d, self.tm)
        try:
            info = self.session.start()
            if col_cfg["enabled"]:
                self.collector = coll.DataCollector(w, self.ego, sensors, col_cfg,
                                                    extra_state=self.session.carsim_state, emit=self.emit)
                info["collect"] = self.collector.start()
                self._log("数据采集开始：%d 个传感器 → %s（预计 %.1f MB/s）" % (
                    len(self.collector.sensor_cfgs), info["collect"]["root"], info["collect"]["estimate"]["mb_per_s"]))
        except Exception:
            self._stop_cosim_if_running("error")
            raise
        self._set_cosim_state("running")
        if cosim:
            self._log("联合仿真开始：%s，参考点 %s，每帧 %d 个 CarSim 步，驾驶：%s" % (
                "改版 CARLA 接口" if info["external_api"] else "原版兼容模式",
                info["reference_point"], info["inner_steps"], d["run"]["driver"]))
            if d["run"]["driver"] == "carsim" and info.get("declared_imports"):
                self._log("CarSim 驾驶员模式：模型声明了 %d 个导入变量，Python 不写它们；若是 REPLACE 模式会一直是 0，"
                          "会覆盖 CarSim 自己的驾驶员（车不动或不转向），请在 CarSim 里删掉导入或改成 ADD" % info["declared_imports"], "warn")
            if info["clock_warning"]:
                self._log("frame_dt 不是 CarSim t_step 的整数倍，两边时钟会漂移", "warn")
        else:
            self._log("仿真开始：CARLA 物理，驾驶：%s" % d["drive"]["carla_driver"])
        return info

    def cmd_manual_control(self, throttle=0.0, brake=0.0, steer=0.0):
        drv = getattr(self.session, "command_driver", None) if self.session else None
        if drv is None or not hasattr(drv, "set"):
            return False
        drv.set(float(throttle), float(brake), float(steer), time.time())
        return True

    # ------------------------------------------------------------- rigs
    def cmd_rig_presets(self):
        return [{"id": k, "name": v} for k, v in rigmod.PRESETS.items()]

    def cmd_rig_build(self, preset, blueprint=""):
        spec = None
        if blueprint and self.world is not None:
            spec = self.spec_cache.get(blueprint) or self.cmd_vehicle_specs([blueprint]).get(blueprint)
        return rigmod.build_preset(preset, spec)

    def cmd_rig_estimate(self, sensors, collect=None, frame_dt=0.1):
        cfg = dict(st.default_dict()["collect"])
        cfg.update(collect or {})
        cfg["frame_dt"] = float(frame_dt)
        c = coll.DataCollector(None, None, sensors, cfg)
        est = c.estimate()
        est["per_sensor"] = [{"name": s["name"], "mb_per_frame": rigmod.bytes_per_frame(s, cfg["image_format"]) / 1e6}
                             for s in c.sensor_cfgs]
        est["disk"] = coll.disk_info(cfg["out_dir"])
        return est

    def cmd_disk_info(self, path="."):
        return coll.disk_info(path)

    def cmd_cosim_pause(self):
        if self.cosim_state == "running":
            self._set_cosim_state("paused")
        return self.cosim_state

    def cmd_cosim_resume(self):
        if self.cosim_state == "paused":
            self._set_cosim_state("running")
        return self.cosim_state

    def cmd_cosim_step(self):
        """Single frame while paused."""
        if self.cosim_state != "paused":
            raise RuntimeError("仅在暂停时可单步")
        self._cosim_frame()
        return True

    def cmd_cosim_stop(self):
        self._stop_cosim_if_running("stopped")
        return True

    def _stop_cosim_if_running(self, final="stopped", detail=""):
        if self.collector is not None:
            try:
                self.collector.stop()
            finally:
                self.collector = None
        if self.session is None:
            return
        try:
            self.session.stop(release_vehicle=True)
        finally:
            self.session = None
            # Back to what the user had before co-sim (usually async), so the
            # world does not stay frozen in sync mode with nobody ticking.
            if getattr(self, "_pre_cosim_settings", None) is not None:
                self.world.apply_settings(self._pre_cosim_settings)
                self._pre_cosim_settings = None
            self.tm.set_synchronous_mode(self.world.get_settings().synchronous_mode)
            self._set_cosim_state(final, detail)

    def _cosim_frame(self):
        try:
            tel = self.session.step()
            if self.collector is not None:
                self.collector.on_tick(tel["world_frame"])
                if self.collector.done:
                    self._log("数据采集结束：%s，共 %d 帧，%.1f MB" % (
                        self.collector.stop_reason, self.collector.frames, self.collector.bytes / 1e6))
                    tel["done"] = True
        except Exception as e:
            self._log("仿真出错：%s" % e, "error")
            self._stop_cosim_if_running("error", str(e))
            return
        self._update_spectator()
        if tel["frame"] % 2 == 0 or tel["done"]:
            self.emit({"event": "telemetry", "data": tel})
        if tel["done"]:
            self._log("联合仿真完成：%.1f s，%.2f 倍实时" % (tel["t"], tel["rt_factor"]))
            self._stop_cosim_if_running("finished")

    # ------------------------------------------------------------ shutdown
    def cleanup(self):
        """Leave the CARLA world as we found it: no ego, sensors, traffic."""
        if getattr(self, "_cleaned", False) or self.world is None:
            return
        self._cleaned = True
        for step in (lambda: self._stop_cosim_if_running("stopped"), self.cmd_view_stop,
                     self.cmd_clear_traffic, self.cmd_destroy_ego):
            try:
                step()
            except Exception:
                traceback.print_exc()

    def cmd_shutdown(self):
        self.cleanup()
        threading.Timer(0.2, lambda: os._exit(0)).start()
        return True

    # ----------------------------------------------------------- dispatcher
    def handle(self, req):
        fn = getattr(self, "cmd_" + str(req.get("cmd")), None)
        if fn is None:
            raise RuntimeError("未知命令 %s" % req.get("cmd"))
        return fn(**(req.get("args") or {}))

    def run_worker(self):
        while True:
            if self.cosim_state == "running":
                self._cosim_frame()
                timeout = 0.0
            else:
                timeout = 0.05
            # Serve every queued request between frames.
            while True:
                try:
                    req, reply = self.requests.get(timeout=timeout)
                except queue.Empty:
                    break
                timeout = 0.0
                try:
                    reply({"id": req.get("id"), "ok": True, "result": self.handle(req)})
                except Exception as e:  # report every failure to the GUI
                    traceback.print_exc()
                    reply({"id": req.get("id"), "ok": False, "error": str(e) or e.__class__.__name__})
            if self.cosim_state != "running" and self.world is not None:
                try:
                    if self.idle_tick and self.world.get_settings().synchronous_mode:
                        self.world.tick()
                        time.sleep(self.frame_dt)
                    self._update_spectator()
                except RuntimeError:
                    pass


def serve(port, exit_with_client=False):
    backend = Backend()
    atexit.register(backend.cleanup)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    threading.Thread(target=backend.run_worker, daemon=True).start()
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port))
    srv.listen(1)
    print("backend listening on 127.0.0.1:%d" % port, flush=True)
    while True:
        conn, _ = srv.accept()
        lock = threading.Lock()

        def send(msg, conn=conn, lock=lock):
            data = (json.dumps(msg, ensure_ascii=False) + "\n").encode("utf-8")
            with lock:
                try:
                    conn.sendall(data)
                except OSError:
                    pass

        backend.emit = send
        buf = b""
        try:
            while True:
                chunk = conn.recv(65536)
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if line.strip():
                        backend.requests.put((json.loads(line.decode("utf-8")), send))
        except OSError:
            pass
        finally:
            backend.emit = lambda msg: None
            conn.close()
        if exit_with_client:
            # Started by the GUI: when it disconnects (closed, crashed, killed),
            # clean CARLA up and quit instead of lingering as an orphan.
            print("GUI disconnected, cleaning up and exiting", flush=True)
            backend.cleanup()
            os._exit(0)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=57100)
    ap.add_argument("--exit-with-client", action="store_true",
                    help="clean up and exit when the client disconnects")
    a = ap.parse_args()
    serve(a.port, a.exit_with_client)
