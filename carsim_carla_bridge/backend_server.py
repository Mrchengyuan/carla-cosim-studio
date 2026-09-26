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
import faulthandler
import json
import math
import os
import queue
import random
import re
import shutil
import signal
import socket
import subprocess
import threading
import time
import traceback

import carla

import collector as coll
import dataset as dsmod
import rig as rigmod
import settings as st
from session import CarlaDriveSession, CoSimSession
from views import ViewStreamer

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
# No car of the background traffic drives this fast (180 km/h): one that does
# was thrown by a collision or is falling out of the world.
RUNAWAY_SPEED = 50.0
PROBE_ROLE = "cosim_probe"  # cars vehicle_specs spawns to measure a vehicle model


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
        self.replay_before = None  # ids in the world before a replay started (its actors are the rest)
        self._probe = None         # the car vehicle_specs is measuring right now
        self.views = ViewStreamer(lambda msg: self.emit(msg))  # live views of the GUI viewport
        self.collector = None
        self._unsent_tel = None    # last telemetry frame not sent to the GUI yet
        self._ego_missing = (0, set())  # (ego id, world frames whose snapshot lacks it)
        self.task = None           # (what the worker is doing, since when), for the "busy" heartbeat
        self.ego_autopilot = False  # the ego is driven by the traffic manager outside of a run
        self.carla_addr = None     # (host, port) of the CARLA server we are connected to
        self._alive_check = 0.0    # when we last checked that CARLA still listens
        self._ds = None            # dataset.Session being browsed
        self.exporter = dsmod.Exporter(lambda msg: self.emit(msg))
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

    def _carla_listening(self, now=False):
        """Does the CARLA server still listen? Never by connecting to it:
        CARLA 0.9.16 crashes ("close: Bad file descriptor") after a few hundred
        connections that close right away. Local Linux: the kernel's socket
        table; local Windows: netstat, only right after a call timed out;
        another host: a call that timed out is taken as the answer (None: can't tell)."""
        host, port = self.carla_addr
        if host not in ("localhost", "127.0.0.1", "::1"):
            return None if not now else False
        if os.path.exists("/proc/net/tcp"):
            want = ":%04X" % port
            for table in ("/proc/net/tcp", "/proc/net/tcp6"):
                try:
                    with open(table) as f:
                        for line in f.readlines()[1:]:
                            cols = line.split()
                            if cols[1].endswith(want) and cols[3] == "0A":  # 0A = LISTEN
                                return True
                except OSError:
                    pass
            return False
        if os.name == "nt" and now:
            try:
                out = subprocess.run(["netstat", "-ano", "-p", "TCP"], capture_output=True, text=True, timeout=10).stdout
            except (OSError, subprocess.SubprocessError):
                return None
            return any(":%d " % port in l and "LISTEN" in l for l in out.splitlines())
        return None

    def _check_carla(self, now=False):
        """Every 2 s (or right after a CARLA call timed out): is the server still
        there? Once it is gone (closed, crashed) every call would wait for the
        client time-out (20 s) and the GUI would crawl: stop using it instead."""
        if self.world is None or self.carla_addr is None:
            return
        if not now and time.time() - self._alive_check < 2.0:
            return
        self._alive_check = time.time()
        if self._carla_listening(now) is not False:
            return
        why = "CARLA 服务器已退出或连不上（%s:%d）" % self.carla_addr
        self.emit({"event": "carla_lost", "reason": why})  # first: the GUI stops asking about that world
        self._try(lambda: self.client.set_timeout(0.5))  # the cleanup below must not wait
        self._try(lambda: self._stop_cosim_if_running("error", why))
        self._try(lambda: self.views.stop())
        for sid in list(self.sensors):
            self._try(lambda sid=sid: self.cmd_remove_sensor(sid))
        self.traffic = {"vehicles": [], "walkers": [], "controllers": []}
        self.ego = self.anchor = self._probe = self.replay_before = None
        # Let go of the old connection entirely: its streaming thread reconnects
        # by itself once a new CARLA listens on the same port, and a stale
        # client (or traffic manager) then trips over the new server.
        self._release_tm()
        # Parts of it may live on anyway (e.g. sensor streams): with the original
        # carla package a call of theirs timing out on a CARLA just restarting
        # ends this process, so give them time again.
        self._try(lambda: self.client.set_timeout(60.0))
        self.world = self.client = None
        self._log(why + "。重新启动 CARLA 后点“连接”", "error")

    def _need_tm(self):
        """The traffic manager, started on first use (traffic, autopilot). It runs
        a thread inside this process that queries CARLA all the time, and in
        CARLA 0.9.16 that thread takes the whole process down when CARLA stops
        answering (the modified carla package fixes that): no traffic, no
        traffic manager."""
        if self.tm is None:
            self.tm = self.client.get_trafficmanager(8000)
            self.tm.set_synchronous_mode(self.world.get_settings().synchronous_mode)
        return self.tm

    def _release_tm(self):
        if self.tm is not None:
            tm, self.tm = self.tm, None
            self._try(tm.shut_down)

    def _tm_sync(self, on):
        if self.tm is not None:
            self.tm.set_synchronous_mode(bool(on))

    def _check_ego(self):
        """is_alive only knows about actors this client destroyed. CARLA removes
        a car that fell out of the world (driven off the map) by itself, and
        another client can destroy it too; is_alive then stays True and runs,
        views and sensors carry on with a car that is gone. The world snapshot
        does know: the ego counts as lost once 3 different frames lack it."""
        if self.ego is None or self.world is None:
            return
        try:
            snap = self.world.get_snapshot()
        except RuntimeError:
            return
        if self._ego_missing[0] != self.ego.id:
            self._ego_missing = (self.ego.id, set())
        if snap.find(self.ego.id) is not None:
            self._ego_missing[1].clear()
            return
        self._ego_missing[1].add(snap.frame)
        if len(self._ego_missing[1]) >= 3 and not self._follow_next_replay_hero(snap):
            self._lose_ego()

    def _follow_next_replay_hero(self, snap):
        """A recording holds every ego of that time (a run respawns it): the
        replay removes one and spawns the next. Keep the views on the current one."""
        if self.replay_before is None:
            return False
        hero = next((a for a in self.world.get_actors().filter("vehicle.*")
                     if a.id not in self.replay_before and a.id != self.ego.id
                     and a.attributes.get("role_name") == "hero" and snap.find(a.id) is not None), None)
        if hero is None:
            return False
        view_specs = self.views.specs()
        self._try(self._drop_ego_refs)
        self.ego = hero
        if view_specs:
            self._try(lambda: self.cmd_views_set(view_specs))
        self.emit({"event": "ego_changed", "id": hero.id})
        return True

    def _ego_gone(self, err=None):
        """The ego is not in the latest world snapshot (see _check_ego), or the
        error of a call says so: the modified CARLA refuses the pose of a car
        that is gone before a new snapshot shows it."""
        if self.ego is None:
            return False
        if err is not None and "could not be found" in str(err) and "Actor Id: %d" % self.ego.id in str(err):
            return True
        try:
            return self.world.get_snapshot().find(self.ego.id) is None
        except RuntimeError:
            return False

    def _lose_ego(self):
        why = "主车已不在 CARLA 里：可能开出了地图边界、掉出了世界（CARLA 会删除掉出世界的车），或被其他程序删除"
        running = self.cosim_state in ("running", "paused")
        # Tell the GUI first (it would otherwise reopen the views of the old
        # ego once they close), then forget the car, so everything below sees
        # "no ego".
        self.emit({"event": "ego_lost", "reason": why})
        self._try(self._drop_ego_refs)
        if running:
            self._stop_cosim_if_running("error", why)
        else:
            self._log(why + "。请重新生成主车", "warn")

    def _check_traffic(self):
        """A traffic car thrown by a collision or falling out of the world gets an
        absurd speed, and the traffic manager of CARLA 0.9.16 then loops forever
        (its path horizon grows with speed, beyond the size of the map); in
        synchronous mode world.tick() never returns. Our CARLA patch bounds that
        loop; with an unpatched carla package, take such a car out early."""
        if not self.traffic["vehicles"] or self.world is None:
            return
        try:
            snap = self.world.get_snapshot()
        except RuntimeError:
            return
        bad = []
        for a in self.traffic["vehicles"]:
            sa = snap.find(a.id)
            if sa is not None:
                v = sa.get_velocity()
                if v.x * v.x + v.y * v.y + v.z * v.z > RUNAWAY_SPEED ** 2:
                    bad.append(a)
        if not bad:
            return
        port = self._need_tm().get_port()
        self._try(lambda: self.client.apply_batch(
            [carla.command.SetAutopilot(a.id, False, port) for a in bad] +
            [carla.command.DestroyActor(a.id) for a in bad]))
        ids = {a.id for a in bad}
        self.traffic["vehicles"] = [a for a in self.traffic["vehicles"] if a.id not in ids]
        self._log("移除了 %d 辆被撞飞或掉出世界的交通车（速度超过 %.0f km/h）" % (len(bad), RUNAWAY_SPEED * 3.6), "warn")

    @staticmethod
    def _try(fn):
        """Run one cleanup step; a failure (e.g. CARLA gone) must not stop the next."""
        try:
            fn()
        except Exception:
            traceback.print_exc()

    def _teardown(self):
        """Remove everything this backend put into the world (run, views, sensors,
        traffic, ego), best effort: used before reconnecting and on exit."""
        self._try(lambda: self._stop_cosim_if_running("stopped"))
        self._try(lambda: self._probe.destroy() if self._probe is not None else None)
        self._try(self._clear_replay)
        self._try(self.views.stop)
        for sid in list(self.sensors):
            self._try(lambda sid=sid: self.cmd_remove_sensor(sid))
        self._try(self.cmd_clear_traffic)
        self._try(lambda: self.ego.destroy() if self._alive(self.ego) else None)
        self.ego = self.anchor = None

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

    def cmd_connect(self, host="localhost", port=2000, timeout=20.0, recover=False):
        if self.world is not None:
            # Reconnecting: take our ego, sensors, views and traffic out of the
            # old world first, or they stay behind as orphans.
            self._teardown()
        self.client = carla.Client(host, int(port))
        self.client.set_timeout(float(timeout))
        self.carla_addr = (host, int(port))
        self.world = self.client.get_world()
        s = self.world.get_settings()
        if s.synchronous_mode:
            # A backend killed during a run leaves the world in synchronous mode
            # with nobody ticking it: frozen. Give a client that does tick it a
            # moment; if none does, go back to asynchronous.
            try:
                self.world.wait_for_tick(2.0)
            except RuntimeError:
                s.synchronous_mode, s.fixed_delta_seconds = False, None
                self.world.apply_settings(s)
                self._log("CARLA 停在同步模式、没有程序在推进（上一次后端没有正常退出），已切回异步模式", "warn")
        if recover:
            self._remove_leftovers()
        self._release_tm()
        self.ego = None
        sv, cv = self.client.get_server_version(), self.client.get_client_version()
        # Does the server have the external-dynamics API? A server built from
        # the same tree as the patched client has; a plain release (e.g.
        # "0.9.16") has not; otherwise the first co-sim run finds out.
        self.server_api = None
        if hasattr(carla.Vehicle, "apply_external_state"):
            self.server_api = True if sv == cv else False if re.fullmatch(r"\d+\.\d+\.\d+", sv) else None
        info = self.cmd_world_info()
        info.update({
            "server_version": sv,
            "client_version": cv,
            "external_api_client": hasattr(carla.Vehicle, "apply_external_state"),
        })
        self._log("已连接 %s:%s，地图 %s" % (host, port, info["map"]))
        return info

    def _remove_leftovers(self):
        """After the GUI restarted a hung or crashed backend, which could not clean
        up: remove what it left in the world (the CARLA server started for this
        program is used by it alone): the ego (role hero), the traffic (role
        autopilot), pedestrians with their AI controllers, and sensors."""
        self.world.wait_for_tick(5.0)
        acts = [a for a in self.world.get_actors()
                if (a.type_id.startswith("vehicle.") and a.attributes.get("role_name") in ("hero", "autopilot", PROBE_ROLE))
                or a.type_id.startswith(("walker.pedestrian.", "controller.ai.walker", "sensor."))]
        for a in acts:
            if a.type_id.startswith("controller."):
                self._try(a.stop)
        if acts:
            self.client.apply_batch_sync([carla.command.DestroyActor(a.id) for a in acts], False)
            self._log("已清理上一次后端留在 CARLA 里的 %d 个对象（主车、交通、行人、传感器）" % len(acts), "warn")

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
            "external_api_server": getattr(self, "server_api", None),
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
        self.replay_before = None  # its actors go with the old world
        # The traffic manager runs inside this process and keeps its vehicle
        # registry across a world change; load_world() then segfaults in
        # libcarla. Shut it down first (a fresh one starts when needed).
        self._release_tm()
        self.client.set_timeout(180.0)
        try:
            self.world = load()
        finally:
            self.client.set_timeout(20.0)
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
            self._tm_sync(synchronous)
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
        # A measurement cut short (backend killed or closed meanwhile) leaves its
        # probe car up there, which then blocks that spot for good.
        stale = [a.id for a in w.get_actors().filter("vehicle.*") if a.attributes.get("role_name") == PROBE_ROLE]
        if stale:
            self.client.apply_batch_sync([carla.command.DestroyActor(i) for i in stale], False)
        result = {}
        for k, vid in enumerate(ids):
            if vid in self.spec_cache:
                result[vid] = self.spec_cache[vid]
                continue
            self.emit({"event": "progress", "task": "vehicle_specs", "done": k, "total": len(ids), "item": vid})
            tf = carla.Transform(carla.Location(base.location.x + 20.0 * k, base.location.y, 500.0))
            bp = bl.find(vid)
            bp.set_attribute("role_name", PROBE_ROLE)
            actor = self._probe = w.try_spawn_actor(bp, tf)
            if actor is None:
                continue
            try:
                # Read before turning physics off: CARLA 0.9.16 then reads the wheel
                # list of multi-wheel vehicles (trucks, buses) out of bounds, which
                # crashes development builds such as the modified CARLA.
                pc = actor.get_physics_control()
                actor.set_simulate_physics(False)
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
                self._probe = None
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
        # Validate before destroying the current ego, so a bad request loses nothing.
        bp = next((b for b in w.get_blueprint_library() if b.id == blueprint), None)
        if bp is None:
            raise RuntimeError("这个 CARLA 里没有车型 %s，请在“车辆与视角”页重新选择" % blueprint)
        pts = w.get_map().get_spawn_points()
        if not pts:
            raise RuntimeError("当前地图没有出生点")
        self._stop_cosim_if_running()
        self.cmd_destroy_ego()
        self._clear_replay()
        bp.set_attribute("role_name", "hero")
        if color and bp.has_attribute("color"):
            bp.set_attribute("color", color)
        spawn_index = int(spawn_index) % len(pts)
        self.anchor = pts[spawn_index]
        self.ego = w.try_spawn_actor(bp, self.anchor)
        for _ in range(5):
            # Traffic keeps moving: the next car may roll into the spot just
            # freed, so clear it again before each attempt. A destroyed car
            # leaves the physics scene with the next frame.
            if self.ego is not None or not self._clear_spawn_point(self.anchor, spawn_index):
                break
            self._tick_or_wait(1)
            self.ego = w.try_spawn_actor(bp, self.anchor)
        if self.ego is None:
            self.anchor = None
            raise RuntimeError("出生点 %d 被其他车辆或物体占用，无法生成主车：换一个出生点，或先在“交通流”页清除交通" % spawn_index)
        self._tick_or_wait(1)
        self._log("已生成主车 %s (id %d) 于 spawn point %d" % (blueprint, self.ego.id, spawn_index))
        return {"id": self.ego.id, "spawn_index": spawn_index}

    def _clear_spawn_point(self, tf, spawn_index, radius=8.0):
        """Traffic drives around: one of our cars may just be standing on the
        ego's spawn point, and the run could not start. The ego has priority:
        replace such cars by new ones on spawn points further away (a car
        teleported instead keeps stale traffic-manager state)."""
        near = [a for a in self.traffic["vehicles"] if a.get_location().distance(tf.location) < radius]
        if not near:
            return False
        w = self.world
        ids = {a.id for a in near}
        blueprints = [w.get_blueprint_library().find(a.type_id) for a in near]
        port = self._need_tm().get_port()
        self.client.apply_batch_sync([carla.command.SetAutopilot(i, False, port) for i in ids] +
                                     [carla.command.DestroyActor(i) for i in ids], False)
        self.traffic["vehicles"] = [a for a in self.traffic["vehicles"] if a.id not in ids]
        free = [p for p in w.get_map().get_spawn_points() if p.location.distance(tf.location) > 40.0]
        random.shuffle(free)
        for bp in blueprints:
            bp.set_attribute("role_name", "autopilot")
            for p in free:
                a = w.try_spawn_actor(bp, p)
                if a is not None:
                    free.remove(p)
                    a.set_autopilot(True, port)
                    self.traffic["vehicles"].append(a)
                    break
        self._log("出生点 %d 上停着 %d 辆交通车，已把它挪到别处" % (spawn_index, len(ids)))
        return True

    def _clear_replay(self):
        """The replayer leaves every actor it spawned in the world: remove them
        (everything new since the replay started that is not ours)."""
        if self.replay_before is None or self.world is None:
            return
        before, self.replay_before = self.replay_before, None
        self._try(lambda: self.client.stop_replayer(True))
        ours = {a.id for a in self.traffic["vehicles"] + self.traffic["walkers"] + self.traffic["controllers"]}
        ours |= set(self.sensors) | {v["actor"].id for v in self.views.views.values()}
        if self._alive(self.ego):
            ours.add(self.ego.id)
        acts = [a for a in self.world.get_actors() if a.id not in before and a.id not in ours
                and a.type_id.split(".")[0] in ("vehicle", "walker", "controller", "sensor")]
        for a in acts:
            if a.type_id.startswith("controller."):
                self._try(a.stop)
        if acts:
            self.client.apply_batch_sync([carla.command.DestroyActor(a.id) for a in acts], False)
            self._log("已清除回放留下的 %d 个对象" % len(acts))

    def cmd_destroy_ego(self):
        self._stop_cosim_if_running()
        self.cmd_view_stop()
        for sid in list(self.sensors):
            self.cmd_remove_sensor(sid)
        if self._alive(self.ego):
            self.ego.destroy()
        self.ego = None
        self.ego_autopilot = False
        return True

    def cmd_ego_autopilot(self, enabled=True):
        if not self._alive(self.ego):
            raise RuntimeError("没有主车")
        if self.cosim_state in ("running", "paused"):
            raise RuntimeError("仿真运行中，主车由当前驾驶方式控制")
        if enabled or self.tm is not None:
            self.ego.set_autopilot(bool(enabled), self._need_tm().get_port())
        self.ego_autopilot = bool(enabled)
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
        self._clear_replay()
        rng = random.Random(int(seed))
        bl = w.get_blueprint_library()
        vbps = [b for b in bl.filter("vehicle.*") if not safe or
                (b.has_attribute("base_type") and b.get_attribute("base_type").as_str() == "car")]
        pts = w.get_map().get_spawn_points()
        rng.shuffle(pts)
        ego_loc = self.ego.get_location() if self._alive(self.ego) else None
        tm_port = self._need_tm().get_port()
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
        if not self.ego_autopilot and self.cosim_state not in ("running", "paused"):
            self._release_tm()  # nothing left for it to drive (see _need_tm)
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
    def cmd_views_set(self, views):
        """Stream several sensors on the ego to the GUI at once (see views.py)."""
        w = self._need_world()
        if not self._alive(self.ego):
            raise RuntimeError("请先生成主车")
        return self.views.set(w, self.ego, list(views))

    def cmd_views_stop(self):
        self.views.stop()
        return True

    def cmd_view_start(self, mode="chase", width=640, height=360, fps=15.0, mount=None, fov=90.0):
        """Single camera view (older clients): same as views_set with one view."""
        spec = {"id": "p0", "kind": "rgb", "mode": mode, "width": width, "height": height, "fps": fps,
                "mount": mount, "attrs": {"fov": fov} if mount else {}}
        r = self.cmd_views_set([spec])
        return {"id": r[0]["actor"], "mode": mode}

    def cmd_view_stop(self):
        return self.cmd_views_stop()

    # ------------------------------------------------------------ datasets
    def _session(self, root):
        if self._ds is None or self._ds.root != os.path.abspath(root):
            self._ds = dsmod.Session(root)
        return self._ds

    def cmd_dataset_list(self, out_dir="datasets"):
        return dsmod.list_sessions(out_dir)

    def cmd_dataset_info(self, root):
        self._ds = None  # re-read: the session may have grown
        return self._session(root).summary()

    def cmd_dataset_frame(self, root, frame, sensor, max_w=960, boxes=True):
        img, info = dsmod.render_frame(self._session(root), int(frame), sensor, int(max_w), bool(boxes))
        import numpy as np
        info.update({"w": int(img.shape[1]), "h": int(img.shape[0]), "sensor": sensor, "frame": int(frame),
                     "rgb": base64.b64encode(np.ascontiguousarray(img).tobytes()).decode("ascii")})
        return info

    def cmd_dataset_export(self, root, format="kitti", out="", camera=None, lidar=None, min_lidar_pts=1):
        if format not in ("kitti", "nuscenes"):
            raise ValueError("未知格式 %s" % format)
        out = out or os.path.abspath(root).rstrip("/") + "_" + format
        return dict(self.exporter.start(root, format, out, camera=camera, lidar=lidar, min_lidar_pts=min_lidar_pts), out=out)

    def cmd_dataset_delete(self, root):
        root = os.path.abspath(root)
        if not (os.path.isfile(os.path.join(root, "calib.json")) and os.path.isfile(os.path.join(root, "meta.json"))):
            raise RuntimeError("不是采集生成的数据集目录，拒绝删除：%s" % root)
        if self.collector is not None and os.path.abspath(getattr(self.collector, "root", "")) == root:
            raise RuntimeError("这个数据集正在采集中")
        if self.exporter.busy() and self.exporter.root == root:
            raise RuntimeError("这个数据集正在导出，等导出结束再删除")
        size = dsmod.dir_size(root)
        shutil.rmtree(root)
        if self._ds is not None and self._ds.root == root:
            self._ds = None
        self._log("已删除数据集 %s（%.1f MB）" % (root, size / 1e6))
        return {"deleted": root, "size_mb": size / 1e6}

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
        w = self._need_world()
        if not os.path.isfile(filename):
            raise RuntimeError("找不到录制文件 %s" % filename)
        self._stop_cosim_if_running()
        # The replayer spawns every recorded vehicle and pedestrian again, the
        # ego and our traffic included: with ours still there the copies overlap
        # them and are thrown around. Clear the stage, then show the recorded
        # ego in the viewport.
        view_specs = self.views.specs()
        self._clear_replay()
        self.cmd_clear_traffic()
        self.cmd_destroy_ego()
        self._tick_or_wait(1)
        self.replay_before = {a.id for a in w.get_actors()}
        info = self.client.replay_file(os.path.abspath(filename), float(start), float(duration), int(follow_id))
        for _ in range(40):
            self._tick_or_wait(1)
            hero = next((a for a in w.get_actors().filter("vehicle.*")
                         if a.id not in self.replay_before and a.attributes.get("role_name") == "hero"), None)
            if hero is not None:
                self.ego, self.anchor = hero, None
                if view_specs:
                    self._try(lambda: self.cmd_views_set(view_specs))
                break
        self._log("回放开始：当前的主车和交通已清除，画面跟随录像里的主车")
        return info

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
        view_specs = self.views.specs()
        color = self.ego.attributes.get("color", "") if self._alive(self.ego) else ""
        old = (self.ego.type_id, self.ego.get_transform(), self.anchor) if self._alive(self.ego) else None
        prev_ego = self.ego
        try:
            self.cmd_spawn_ego(c["vehicle"], c["spawn_index"], color)
        except Exception:
            # The run did not start: put the previous ego back where it was, with
            # its sensors and views, so a failed start costs the user nothing.
            # (Nothing to do when the request was refused before the ego was touched.)
            if old is not None and not (self.ego is prev_ego and self._alive(prev_ego)):
                try:
                    self._restore_ego(old, color, sensor_specs, view_specs)
                except Exception:
                    traceback.print_exc()  # report the original error, not this one
            raise
        for spec in sensor_specs:
            self.cmd_add_sensor(**spec)
        if view_specs:
            self.cmd_views_set(view_specs)
        self._unsent_tel = None
        self._pre_cosim_settings = w.get_settings()
        try:
            # Settle under PhysX so the bridge reads a resting vehicle.
            s = w.get_settings()
            s.synchronous_mode, s.fixed_delta_seconds = True, d["sync"]["frame_dt"]
            w.apply_settings(s)
            self._tm_sync(True)
            for _ in range(20):
                w.tick()
            cosim = d["drive"]["dynamics"] == "cosim"
            autopilot = not cosim and d["drive"]["carla_driver"] == "autopilot"
            self.session = CoSimSession(w, self.ego, self.anchor, d) if cosim else \
                CarlaDriveSession(w, self.ego, d, self._need_tm() if autopilot else None)
        except BaseException:
            # Never leave the world in sync mode with nobody ticking it.
            pre, self._pre_cosim_settings = self._pre_cosim_settings, None
            self._try(lambda: w.apply_settings(pre))
            self._try(lambda: self._tm_sync(pre.synchronous_mode))
            raise
        try:
            info = self.session.start()
            if col_cfg["enabled"]:
                self.collector = coll.DataCollector(w, self.ego, sensors, col_cfg,
                                                    extra_state=self.session.carsim_state, emit=self.emit)
                info["collect"] = self.collector.start()
                self._log("数据采集开始：%d 个传感器 → %s（预计 %.1f MB/s）" % (
                    len(self.collector.sensor_cfgs), info["collect"]["root"], info["collect"]["estimate"]["mb_per_s"]))
        except BaseException:  # SystemExit from a user controller included
            self._stop_cosim_if_running("error")
            raise
        self._set_cosim_state("running")
        if info.get("server_api") is not None:
            self.server_api = info["server_api"]
        if cosim:
            self._log("联合仿真开始：%s，参考点 %s，每帧 %d 个 CarSim 步，驾驶：%s" % (
                "改版 CARLA 接口" if info["external_api"] else "原版兼容模式",
                info["reference_point"], info["inner_steps"], d["run"]["driver"]))
            if info["clock_warning"]:
                self._log("frame_dt 不是 CarSim t_step 的整数倍，两边时钟会漂移", "warn")
        else:
            self._log("仿真开始：CARLA 物理，驾驶：%s" % d["drive"]["carla_driver"])
        return info

    def _restore_ego(self, old, color, sensor_specs, view_specs):
        """Put the previous ego (type, transform, anchor) back with its sensors and views."""
        w = self.world
        if self._alive(self.ego):  # spawned, then something after the spawn failed
            self._try(self.cmd_destroy_ego)
        self.ego = None
        bp = w.get_blueprint_library().find(old[0])
        bp.set_attribute("role_name", "hero")
        if color and bp.has_attribute("color"):
            bp.set_attribute("color", color)
        tf = old[1]
        for _ in range(5):
            # The destroyed ego only leaves the physics scene on the next frame.
            self._tick_or_wait(1)
            tf.location.z += 0.1
            self.ego = w.try_spawn_actor(bp, tf)
            if self.ego is not None:
                break
        if self.ego is None:  # is_alive can lag one frame behind the spawn, so test for None
            return
        self.anchor = old[2]
        self._tick_or_wait(1)
        for spec in sensor_specs:
            self.cmd_add_sensor(**spec)
        if view_specs:
            self.cmd_views_set(view_specs)

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
            # Telemetry goes out every 2nd frame: send the frame we stopped on,
            # so the GUI shows exactly the paused state (and a step is +1).
            if self._unsent_tel is not None:
                self.emit({"event": "telemetry", "data": self._unsent_tel})
                self._unsent_tel = None
        return self.cosim_state

    def cmd_cosim_resume(self):
        if self.cosim_state == "paused":
            self._set_cosim_state("running")
        return self.cosim_state

    def cmd_cosim_step(self):
        """Single frame while paused."""
        if self.cosim_state != "paused":
            raise RuntimeError("仅在暂停时可单步")
        self._cosim_frame(always_emit=True)
        return True

    def cmd_cosim_stop(self):
        self._stop_cosim_if_running("stopped")
        # Always report the state, so a GUI that is out of sync (e.g. after a
        # backend restart) stops showing "running".
        self._set_cosim_state(self.cosim_state if self.cosim_state not in ("running", "paused") else "stopped")
        return True

    def _stop_cosim_if_running(self, final="stopped", detail=""):
        """Best effort: every step runs even if an earlier one fails (CARLA may be
        gone), and afterwards there is no session and the state is `final`."""
        col, self.collector = self.collector, None
        if col is not None:
            self._try(col.stop)
        ses, self.session = self.session, None
        self._unsent_tel = None
        if ses is None:
            if self.cosim_state in ("running", "paused"):
                self._set_cosim_state(final, detail)
            return
        self._try(lambda: ses.stop(release_vehicle=True))
        # Back to what the user had before co-sim (usually async), so the
        # world does not stay frozen in sync mode with nobody ticking.
        pre, self._pre_cosim_settings = getattr(self, "_pre_cosim_settings", None), None
        if pre is not None:
            self._try(lambda: self.world.apply_settings(pre))
        self._try(lambda: self._tm_sync(self.world.get_settings().synchronous_mode))
        self._try(self._park_ego)
        self._set_cosim_state(final, detail)

    def _park_ego(self):
        """Stop means stop, like the end of a CarSim run: without this the car
        keeps the last throttle (route / manual / autopilot) or the CarSim
        velocity handed over to PhysX, and drives or rolls on."""
        if not self._alive(self.ego):
            return
        try:
            if self.tm is not None:
                self.ego.set_autopilot(False, self.tm.get_port())
            self.ego_autopilot = False
            self.ego.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
            self.ego.set_target_velocity(carla.Vector3D())
            self.ego.set_target_angular_velocity(carla.Vector3D())
        except RuntimeError:
            pass

    def _cosim_frame(self, always_emit=False):
        if self.session is None:  # state says running but the run is gone
            self._stop_cosim_if_running("error", "仿真会话已丢失")
            return
        try:
            tel = self.session.step()
            if self.collector is not None:
                self.collector.on_tick(tel["world_frame"])
                if self.collector.done:
                    self._log("数据采集结束：%s，共 %d 帧，%.1f MB" % (
                        self.collector.stop_reason, self.collector.frames, self.collector.bytes / 1e6))
                    tel["done"] = True
        except (Exception, SystemExit) as e:  # SystemExit: e.g. argparse in a user controller
            traceback.print_exc()
            if "time-out" in str(e):
                self._check_carla(now=True)
                if self.world is None:
                    return
            if self._ego_gone(e):
                # The modified CARLA refuses the pose of a car that is gone right
                # away: report that, not the RPC error it causes.
                self._lose_ego()
                return
            self._log("仿真出错：%s" % (e or e.__class__.__name__), "error")
            self._stop_cosim_if_running("error", str(e))
            return
        self._update_spectator()
        # Every 2nd frame is enough for the GUI, but a single step must show.
        if always_emit or tel["frame"] % 2 == 0 or tel["done"]:
            self.emit({"event": "telemetry", "data": tel})
            self._unsent_tel = None
        else:
            self._unsent_tel = tel
        if tel["done"]:
            # Tell the GUI exactly why the run ended.
            ses = self.session
            if self.collector is not None and self.collector.done:
                reason = "数据采集%s" % self.collector.stop_reason
            elif ses.n_frames > 0 and ses.frame >= ses.n_frames:
                reason = "达到设定的运行时长 %.0f s" % tel["t"]
            else:
                reason = "CarSim 到达 .sim 里设定的结束时间"
            self._log("运行结束（%s）：%.1f s，%.2f 倍实时" % (reason, tel["t"], tel["rt_factor"]))
            self._stop_cosim_if_running("finished", reason)

    # ------------------------------------------------------------ shutdown
    def cleanup(self):
        """Leave the CARLA world as we found it: no ego, sensors, traffic."""
        if getattr(self, "_cleaned", False) or self.world is None:
            return
        self._cleaned = True
        self._teardown()

    def cmd_shutdown(self):
        threading.Timer(5.0, lambda: os._exit(0)).start()  # even if the cleanup hangs
        self.cleanup()
        threading.Timer(0.2, lambda: os._exit(0)).start()
        return True

    def exit_now(self, limit=5.0):
        """Clean the world up and exit, but never hang on the way out: a stuck
        CARLA call (e.g. a looping traffic manager) would keep the process, and
        the GUI waiting for it, alive forever."""
        t0 = time.time()
        t = threading.Thread(target=self.cleanup, daemon=True)
        t.start()
        t.join(limit)
        print("exit: cleanup %s after %.1f s" % ("done" if not t.is_alive() else "NOT finished", time.time() - t0), flush=True)
        os._exit(0)

    # ----------------------------------------------------------- dispatcher
    def handle(self, req):
        fn = getattr(self, "cmd_" + str(req.get("cmd")), None)
        if fn is None:
            raise RuntimeError("未知命令 %s" % req.get("cmd"))
        return fn(**(req.get("args") or {}))

    def run_worker(self):
        """The one thread that touches CARLA. It must never die: an uncaught
        error here would leave the GUI waiting forever for replies."""
        while True:
            try:
                self._worker_iteration()
            except BaseException:
                traceback.print_exc()
                self.task = None
                self._try(lambda: self._stop_cosim_if_running("error", "后端内部错误"))
                time.sleep(0.1)

    def _worker_iteration(self):
        self._check_carla()
        self._check_ego()
        self._check_traffic()
        if self.cosim_state == "running":
            self.task = ("仿真步进", time.time())
            self._cosim_frame()
            self.task = None
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
            self.task = (str(req.get("cmd")), time.time())
            try:
                reply({"id": req.get("id"), "ok": True, "result": self.handle(req)})
            except BaseException as e:  # report every failure to the GUI (SystemExit too)
                traceback.print_exc()
                reply({"id": req.get("id"), "ok": False, "error": str(e) or e.__class__.__name__})
                if "time-out" in str(e):
                    self._check_carla(now=True)
            finally:
                self.task = None
        if self.cosim_state != "running" and self.world is not None:
            try:
                if self.idle_tick and self.world.get_settings().synchronous_mode:
                    self.task = ("空闲时推进世界", time.time())
                    try:
                        self.world.tick()
                    finally:
                        self.task = None
                    time.sleep(self.frame_dt)
                self._update_spectator()
            except RuntimeError as e:
                if "time-out" in str(e):
                    self._check_carla(now=True)


def _json_safe(o):
    """Replace non-finite floats with None and unknown objects with their str()."""
    if isinstance(o, float):
        return o if math.isfinite(o) else None
    if isinstance(o, dict):
        return {str(k): _json_safe(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_json_safe(v) for v in o]
    if o is None or isinstance(o, (str, int, bool)):
        return o
    try:
        return _json_safe(float(o))  # numpy scalars
    except (TypeError, ValueError):
        return str(o)


def serve(port, exit_with_client=False):
    # A crash inside the carla library (a C++ thread) leaves no Python error:
    # print every thread's stack into the log instead. SIGUSR1 prints them for
    # a backend that hangs (the GUI sends it before restarting one).
    faulthandler.enable(all_threads=True)
    if hasattr(signal, "SIGUSR1"):
        faulthandler.register(signal.SIGUSR1, all_threads=True)
    backend = Backend()
    atexit.register(backend.cleanup)
    signal.signal(signal.SIGTERM, lambda *_: backend.exit_now())
    threading.Thread(target=backend.run_worker, daemon=True).start()

    def heartbeat():
        # Tell the GUI what the worker is busy with, so a CARLA call that never
        # returns shows up as such instead of as a silently frozen GUI.
        while True:
            time.sleep(1.0)
            task = backend.task
            if task is not None and time.time() - task[1] >= 2.0:
                # A CARLA call that waits because CARLA is gone: say so now, and
                # let the calls after it give up quickly.
                gone = backend.world is not None and backend.carla_addr is not None and backend._carla_listening() is False
                if gone:
                    backend._try(lambda: backend.client.set_timeout(0.5))
                backend.emit({"event": "busy", "task": task[0], "seconds": round(time.time() - task[1], 1),
                              "carla_gone": gone})
    threading.Thread(target=heartbeat, daemon=True).start()
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    for attempt in range(50):
        try:
            srv.bind(("127.0.0.1", port))
            break
        except OSError:
            # The previous backend may still be cleaning up (GUI closed and
            # opened again right away): give it a few seconds.
            if attempt == 49:
                raise
            time.sleep(0.2)
    srv.listen(1)
    print("backend listening on 127.0.0.1:%d" % port, flush=True)
    while True:
        conn, _ = srv.accept()
        lock = threading.Lock()

        def send(msg, conn=conn, lock=lock):
            try:
                text = json.dumps(msg, ensure_ascii=False, allow_nan=False)
            except (ValueError, TypeError):
                # NaN / inf (e.g. a diverging CarSim) are not valid JSON and the GUI
                # would drop the whole message: send them as null instead.
                text = json.dumps(_json_safe(msg), ensure_ascii=False, allow_nan=False)
            data = (text + "\n").encode("utf-8")
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
                    if not line.strip():
                        continue
                    try:
                        req = json.loads(line.decode("utf-8"))
                    except ValueError as e:  # one bad line must not end the backend
                        send({"event": "log", "level": "error", "msg": "后端收到无法解析的请求：%s" % e})
                        continue
                    backend.requests.put((req, send))
        except OSError:
            pass
        finally:
            backend.emit = lambda msg: None
            conn.close()
        if exit_with_client:
            # Started by the GUI: when it disconnects (closed, crashed, killed),
            # clean CARLA up and quit instead of lingering as an orphan.
            print("GUI disconnected, cleaning up and exiting", flush=True)
            backend.exit_now()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=57100)
    ap.add_argument("--exit-with-client", action="store_true",
                    help="clean up and exit when the client disconnects")
    a = ap.parse_args()
    serve(a.port, a.exit_with_client)
