"""GUI backend: exposes CARLA operations and CarSim co-simulation over TCP.

Protocol: newline-delimited JSON (UTF-8) on localhost.
  request : {"id": 1, "cmd": "load_map", "args": {"name": "Town03"}}
  response: {"id": 1, "ok": true, "result": ...}  or  {"id": 1, "ok": false, "error": "..."}
  event   : {"event": "telemetry" | "log" | "cosim_state" | "progress", ...}

All CARLA calls run on one worker thread; the socket thread only queues
requests and writes replies, so a slow map load never blocks the connection.
Commands that only touch files (IO_CMDS) have a thread of their own.

    python backend_server.py --port 57100 [--carsim-port 57121]

Remote mode (carsim.remote): the CarSim service on the user's Windows computer
connects to --carsim-port (127.0.0.1) and runs CarSim there (carsim_remote.py).
Only the remote session's backend gives it: by default nothing listens for it.
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
import sys
import threading
import time
import traceback

import carla
import numpy as np

import carsim_remote
import batch_report
import runs as runsmod
import simcheck
import templates
import vehicle_ident
import chrono_local
import collector as coll
import dataset as dsmod
import rig as rigmod
import scenario as scenariomod
import settings as st
from bridge import anchor_frame, front_axle_local
from carsim_local import json_safe as _json_safe
from session import (AlgoOutput, CarlaDriveSession, CoSimSession, browse_controllers, check_run_config, check_run_files,
                     controller_params,
                     control_busy)
from views import ViewStreamer, encode_jpeg

WEATHER_PRESETS = [n for n in dir(carla.WeatherParameters)
                   if n[0].isupper() and isinstance(getattr(carla.WeatherParameters, n), carla.WeatherParameters)]
WEATHER_FIELDS = ["cloudiness", "precipitation", "precipitation_deposits", "wind_intensity",
                  "sun_azimuth_angle", "sun_altitude_angle", "fog_density", "fog_distance",
                  "wetness", "fog_falloff", "scattering_intensity", "mie_scattering_scale",
                  "rayleigh_scattering_scale", "dust_storm"]
# No car of the background traffic drives this fast (180 km/h): one that does
# was thrown by a collision or is falling out of the world.
RUNAWAY_SPEED = 50.0
PROBE_ROLE = "cosim_probe"  # cars vehicle_specs spawns to measure a vehicle model
# Commands that never touch CARLA (dataset browsing / export / deletion, disk
# space, estimates): served by their own thread, so a dataset playback or
# deleting a large dataset never holds up a run's frames on the worker.
IO_CMDS = {"dataset_list", "dataset_info", "dataset_frame", "dataset_export", "dataset_delete",
           "disk_info", "rig_estimate", "path_status", "carsim_service", "controller_browse",
           "runs_list", "run_series", "controller_params", "batch_summary", "vehicle_identify",
           "templates_list", "template_config", "run_output"}
# What "hello" reports; the GUI (kBackendProtocol in cosim_gui/src/app.cpp) warns
# when it was built for another one. Raise both together whenever a command,
# event or config field the GUI relies on changes.
PROTOCOL = 4


class Backend:
    def __init__(self):
        self.client = self.world = None
        self.tm = None
        self.ego = None
        self.anchor = None
        self.session = None
        self.scenario_ids = []  # 测试场景 props of the last run (removed when the next one starts)
        self._replay_cache = {}  # 运行对比 replay: folder -> (ego track, anchor frame)
        self._replay_ref = None  # (ego id, its reference point in its own frame)
        # The 输出 of a run, also into its record folder (output.txt): a list of lines until the
        # folder exists, then the file's path; None between runs.
        self._run_out = None
        self._run_out_lock = threading.Lock()
        self._scn_start = None  # (map name, scenario.find_start of it)
        self.ending = None         # the session in stop(), for the heartbeat: the algorithm's finish()
        self.cosim_state = "stopped"
        self.spectator_mode = "free"
        self.recording = None      # file of the CARLA recording we started (the recorder runs inside CARLA)
        self.traffic = {"vehicles": [], "walkers": [], "controllers": []}
        self.traffic_seed = 0      # seed of the last spawn_traffic (also for cars moved off the ego's spawn point)
        self.idle_tick = False     # tick the world ourselves when sync mode is on and idle
        self.frame_dt = 0.05
        self.spec_cache = {}
        self.replay_before = None  # ids in the world before a replay started (its actors are the rest)
        self._probe = None         # the car vehicle_specs is measuring right now
        self.views = ViewStreamer(lambda msg: self.emit(msg))  # live views of the GUI viewport
        self.collector = None
        self._stopping_collector = None  # still writing its last frames (see _stop_cosim_if_running)
        self._pending_walkers = []  # walker controllers to start after the run's next frame
        self._unsent_tel = None    # last telemetry frame not sent to the GUI yet
        self._ego_missing = (0, set())  # (ego id, world frames whose snapshot lacks it)
        self.task = None           # (what the worker is doing, since when), for the "busy" heartbeat
        self.ego_autopilot = False  # the ego is driven by the traffic manager outside of a run
        self.carla_addr = None     # (host, port) of the CARLA server we are connected to
        self._alive_check = 0.0    # when we last checked that CARLA still listens
        self._gone_hint = False    # the heartbeat saw no CARLA listening (Windows): check at once
        self._ds = None            # dataset.Session being browsed
        self.exporter = dsmod.Exporter(lambda msg: self.emit(msg))
        self.emit = lambda msg: None
        self.requests = queue.Queue()
        self.io_requests = queue.Queue()  # IO_CMDS, see run_io_worker

    # ------------------------------------------------------------ utilities
    def _need_world(self):
        if self.world is None:
            raise RuntimeError("未连接 CARLA，请先连接")
        return self.world

    def _log(self, msg, level="info"):
        self.emit({"event": "log", "level": level, "msg": msg})
        self._run_out_add(msg, level)

    _OUT_TAGS = {"warn": "[警告] ", "error": "[错误] ", "algo": "[算法] "}

    def _run_out_add(self, msg, level="info", folder=None):
        """A line of the running run's 输出 into <record folder>/output.txt (kept in memory
        until the folder is there; folder: the run's, when the session no longer has it)."""
        if self._run_out is None:
            return
        line = "%s %s%s" % (time.strftime("%H:%M:%S"), self._OUT_TAGS.get(level, ""),
                            str(msg).rstrip("\n").replace("\n", "\n    "))  # a traceback: indented under its line
        with self._run_out_lock:
            out = self._run_out
            if isinstance(out, list):
                out.append(line)
                del out[:-5000]
                folder = folder or getattr(self.session, "record_dir", None)
                if not (folder and os.path.isdir(folder)):
                    return
                self._run_out, lines = os.path.join(folder, "output.txt"), out
            else:
                lines = [line]
            try:
                with open(self._run_out, "a", encoding="utf-8") as f:
                    f.write("".join(x + "\n" for x in lines))
            except OSError:
                pass  # (a full disk: the run record says so itself)

    def _algo_output(self, ses=None, end=False):
        """What the user's algorithm printed, and its traceback, to the 输出
        page (level "algo"); backend.log has all of it already."""
        out = getattr(self.session if ses is None else ses, "algo_out", None)
        for line in out.take(end) if isinstance(out, AlgoOutput) else ():
            self._log(line, "algo")

    def _set_cosim_state(self, s, detail=""):
        self.cosim_state, self.cosim_detail = s, detail
        self.emit({"event": "cosim_state", "state": s, "detail": detail})

    def _carsim_service_changed(self, s):
        """Remote mode: the CarSim service on the Windows computer connected or went away."""
        self.emit(dict(s, event="carsim_service"))  # the GUI's status line, without CARLA
        if s["connected"]:
            self._log("Windows 上的 CarSim 服务已连上（%s）" % s["host"])
        else:
            self._log("Windows 上的 CarSim 服务断开了", "warn")

    def _alive(self, actor):
        try:
            return actor is not None and actor.is_alive
        except RuntimeError:
            return False

    def _carla_listening(self, now=False, record=True):
        """Does the CARLA server still listen? Never by connecting to it:
        CARLA 0.9.16 crashes ("close: Bad file descriptor") after a few hundred
        connections that close right away. Local Linux: the kernel's socket
        table; local Windows: netstat, only right after a call timed out (and
        from the heartbeat thread, see _carla_listening_now);
        another host: a call that timed out is taken as the answer (None: can't tell).
        The listening socket (Linux: inode, Windows: process id) seen right after
        connecting is remembered: another one on the port later is not our
        CARLA (it was restarted, or another program took the port). Only the
        worker records it (record=False: the heartbeat thread)."""
        host, port = self.carla_addr
        if host not in ("localhost", "127.0.0.1", "::1"):
            return None if not now else False
        if os.path.exists("/proc/net/tcp") and getattr(self, "_proc_sees_carla", True):
            want = ":%04X" % port
            owners = set()
            for table in ("/proc/net/tcp", "/proc/net/tcp6"):
                try:
                    with open(table) as f:
                        for line in f.readlines()[1:]:
                            cols = line.split()
                            if cols[1].endswith(want) and cols[3] == "0A":  # 0A = LISTEN
                                owners.add(cols[9])  # socket inode
                except (OSError, IndexError):
                    pass
            return self._same_listener(owners, record)
        if os.name == "nt" and now and getattr(self, "_proc_sees_carla", True):
            try:
                out = subprocess.run(["netstat", "-ano", "-p", "TCP"], capture_output=True, text=True, timeout=10).stdout
            except (OSError, subprocess.SubprocessError):
                return None
            # The state column is localized (e.g. "ABHÖREN"); a listening socket is
            # the one without a foreign address.
            owners = set()
            for line in out.splitlines():
                cols = line.split()
                if len(cols) >= 3 and cols[1].endswith(":%d" % port) and cols[2] in ("0.0.0.0:0", "[::]:0", "*:*"):
                    owners.add(cols[-1])  # process id
            return self._same_listener(owners, record)
        return None

    def _same_listener(self, owners, record=True):
        """Listening on our port, by the listener seen when we connected?"""
        if not owners:
            return False
        known = getattr(self, "_carla_listener", None)
        if known is None:  # right after connecting: that is our CARLA
            if not record:  # the heartbeat, while the worker connects: can't tell
                return None
            self._carla_listener = set(owners)
            return True
        return bool(owners & known)

    def _carla_listening_now(self, record=True):
        """_carla_listening with the slow look too (netstat on Windows), for
        the heartbeat thread (record=False) and before a reconnect; None for
        another host, where only a call that timed out tells."""
        addr, known = self.carla_addr, getattr(self, "_carla_listener", None)
        if addr is None:
            return None
        listening = self._carla_listening(now=os.name == "nt" and addr[0] in ("localhost", "127.0.0.1", "::1"),
                                          record=record)
        if not record and (self.carla_addr != addr or getattr(self, "_carla_listener", None) is not known):
            return None  # the worker connected meanwhile: that look was about the old CARLA
        return listening

    def _check_carla(self, now=False):
        """Every 2 s (or right after a CARLA call timed out, or when the
        heartbeat saw no CARLA): is the server still there? Once it is gone
        (closed, crashed) every call would wait for the client time-out (20 s)
        and the GUI would crawl: stop using it instead."""
        if self.world is None or self.carla_addr is None:
            return
        hint, self._gone_hint = self._gone_hint, False
        if not now and not hint and time.time() - self._alive_check < 2.0:
            return
        self._alive_check = time.time()
        listening = self._carla_listening(now) if now or not hint else self._carla_listening_now()
        if listening is not False:
            if getattr(self, "_fast_timeout", False):  # set while CARLA seemed gone
                self._fast_timeout = False
                self._try(lambda: self.client.set_timeout(20.0))
            return
        why = "CARLA 服务器已退出或连不上（%s:%d）" % self.carla_addr
        self.emit({"event": "carla_lost", "reason": why})  # first: the GUI stops asking about that world
        self._forget_carla(why)
        self._log(why + "。重新启动 CARLA 后点“连接”", "error")

    def _forget_carla(self, why):
        """Stop using a CARLA server that is gone, without waiting for it: the
        run ends with `why`, and nothing of its world is kept (its actor ids
        would name other actors in the next world)."""
        self._try(lambda: self.client.set_timeout(0.5))  # the cleanup below must not wait
        self._pending_walkers = []
        self._try(lambda: self._stop_cosim_if_running("error", why))
        self._try(lambda: self.views.stop())
        self.recording = None  # went with that CARLA (or is out of reach)
        self.traffic = {"vehicles": [], "walkers": [], "controllers": []}
        self.ego = self.anchor = self._probe = self.replay_before = None
        self.ego_autopilot = False
        # Let go of the old connection entirely: its streaming thread reconnects
        # by itself once a new CARLA listens on the same port, and a stale
        # client (or traffic manager) then trips over the new server.
        self._release_tm()
        # Parts of it may live on anyway (e.g. sensor streams): with the original
        # carla package a call of theirs timing out on a CARLA just restarting
        # ends this process, so give them time again.
        self._try(lambda: self.client.set_timeout(60.0))
        self.world = self.client = None

    def _need_tm(self):
        """The traffic manager, started on first use (traffic, autopilot). It runs
        a thread inside this process that queries CARLA all the time, and in
        CARLA 0.9.16 that thread takes the whole process down when CARLA stops
        answering (the modified carla package fixes that): no traffic, no
        traffic manager."""
        if self.tm is None:
            # One traffic manager port per CARLA server (2000 -> 8000, 3000 -> 9000):
            # with one shared port, a second backend on the other CARLA cannot start its own.
            port = self.carla_addr[1] + 6000 if self.carla_addr else 8000
            self.tm = self.client.get_trafficmanager(port if port < 65536 else 8000)
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
        """Remove everything this backend put into the world (run, views,
        traffic, ego) and stop its CARLA recording, best effort: used before
        reconnecting and on exit."""
        self._try(lambda: self._stop_cosim_if_running("stopped"))
        self._try(lambda: self._probe.destroy() if self._probe is not None else None)
        self._try(self._clear_replay)
        self._try(self.views.stop)
        if self.recording:
            # The recorder runs inside CARLA: left alone it writes on until CARLA exits.
            self._try(self.cmd_stop_recorder)
            self.recording = None
        self._try(self.cmd_clear_traffic)
        self._try(self._clear_scenario)
        self._try(lambda: self.ego.destroy() if self._alive(self.ego) else None)
        self.ego = self.anchor = None
        self.ego_autopilot = False

    def _clear_scenario(self, everywhere=False):
        """Remove the 测试场景 props of the last run; everywhere: also any other
        in the world (a backend that crashed or was killed leaves its own)."""
        ids, self.scenario_ids = self.scenario_ids, []
        if everywhere and self.world is not None:
            ids = sorted(set(ids) | {a.id for a in scenariomod.leftovers(self.world)})
        if ids and self.world is not None:
            scenariomod.remove(self.client, ids)

    def _run_active(self):
        return self.session is not None and self.cosim_state in ("running", "paused")

    def _tick_or_wait(self, n=1):
        """Let the world advance n frames. False when a run is active: only the
        run may tick then (an extra tick would move CARLA a frame ahead of
        CarSim); what waited for the tick happens with the run's next frame."""
        w = self._need_world()
        for _ in range(n):
            if w.get_settings().synchronous_mode:
                if self._run_active():
                    return False
                w.tick()
            else:
                w.wait_for_tick(5.0)
        return True

    # --------------------------------------------------------------- server
    def cmd_ping(self):
        return "pong"

    def cmd_carsim_service(self):
        """Remote mode: is the CarSim service on the Windows computer connected (CARLA or not)."""
        return carsim_remote.SERVICE.status()

    def cmd_hello(self, jpeg=False):
        # A GUI at the other end of an SSH tunnel (its remote_backend setting)
        # asks for its live views as JPEG instead of raw RGB.
        self.views.jpeg = bool(jpeg)
        return {"protocol": PROTOCOL}

    def cmd_connect(self, host="localhost", port=2000, timeout=20.0, recover=False):
        # This backend kept the world ticking itself: sync mode is no sign of a
        # frozen world then.
        we_ticked = False
        if self.world is not None:
            if self._carla_listening_now() is False:
                # The old CARLA is gone (closed, crashed, restarted) and nothing
                # noticed yet: cleaning up would wait for the time-out at every call.
                self._forget_carla("CARLA 服务器已退出")
                self._log("之前连接的 CARLA 服务器已经退出，不再清理它的世界")
            else:
                we_ticked = self.idle_tick
                # Reconnecting: take our ego, sensors, views and traffic out of the
                # old world first, or they stay behind as orphans. A short time-out:
                # a CARLA on another host may have died without anyone noticing.
                self._try(lambda: self.client.set_timeout(3.0))
                self._teardown()
                self._try(lambda: self.client.set_timeout(20.0))
        self._release_tm()
        self.world = None  # until the new connection works: never half old, half new
        self._fast_timeout = self._gone_hint = False  # about the old connection
        self.client = carla.Client(host, int(port))
        self.client.set_timeout(float(timeout))
        self.carla_addr = (host, int(port))
        try:
            world = self.client.get_world()
        except Exception:
            self.client = None
            raise
        # Docker / WSL2 setups can hide a live CARLA from /proc/net/tcp (or
        # netstat): then only time-outs tell that it is gone. (Before self.world
        # is set: from then on the heartbeat thread looks at the port too.)
        self._proc_sees_carla = True
        self._carla_listener = None  # the next look at the port records this CARLA's socket
        self._proc_sees_carla = self._carla_listening(now=os.name == "nt") is not False
        self.world = world
        s = self.world.get_settings()
        if s.synchronous_mode and not we_ticked:
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
        # A new connection: an "error" / "finished" of an earlier run is not about it.
        self.cosim_state, self.cosim_detail = "stopped", ""
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
        autopilot), pedestrians with their AI controllers, and sensors; and it
        stops a CARLA recording that would otherwise go on until CARLA exits."""
        self.world.wait_for_tick(5.0)
        self._try(self.client.stop_recorder)  # nothing happens when none is running
        acts = [a for a in self.world.get_actors()
                if (a.type_id.startswith("vehicle.") and a.attributes.get("role_name") in ("hero", "autopilot", PROBE_ROLE,
                                                                                            scenariomod.ROLE))
                or a.type_id.startswith(("walker.pedestrian.", "controller.ai.walker", "sensor."))
                or (a.type_id.startswith("static.prop.") and a.attributes.get("role_name") == scenariomod.ROLE)]
        for a in acts:
            if a.type_id.startswith("controller."):
                self._try(a.stop)
        if acts:
            self.client.apply_batch_sync([carla.command.DestroyActor(a.id) for a in acts], False)
            self._log("已清理上一次后端留在 CARLA 里的 %d 个对象（主车、交通、行人、传感器、测试场景的锥桶 / 护栏）" % len(acts), "warn")

    def _scenario_start(self, cmap):
        """scenario.find_start of this map, once per map; None when it fails (world_info
        must not fail over it)."""
        if self._scn_start is None or self._scn_start[0] != cmap.name:
            try:
                start = scenariomod.find_start(cmap)
            except Exception:
                traceback.print_exc()
                start = None
            self._scn_start = (cmap.name, start)
        return self._scn_start[1]

    def cmd_world_info(self):
        w = self._need_world()
        s = w.get_settings()
        cmap = w.get_map()
        return {
            "map": cmap.name.split("/")[-1],
            "synchronous": s.synchronous_mode,
            "frame_dt": s.fixed_delta_seconds or 0.0,
            "no_rendering": s.no_rendering_mode,
            "weather": self._weather_dict(w.get_weather()),
            "n_actors": len(w.get_actors()),
            "ego_id": self.ego.id if self._alive(self.ego) else 0,
            "spectator_mode": self.spectator_mode,
            "cosim_state": self.cosim_state,
            "idle_tick": self.idle_tick,
            "ego_autopilot": self.ego_autopilot and self._alive(self.ego),
            "external_api_server": getattr(self, "server_api", None),
            "recording": self.recording or "",
            # Remote mode (carsim.remote): the CarSim service on the Windows computer.
            "carsim_service": carsim_remote.SERVICE.status(),
            # 测试场景: the start its presets are made for (Town04: the highway), None if the map has none.
            "scenario_start": self._scenario_start(cmap),
        }

    # ---------------------------------------------------------------- world
    def cmd_list_maps(self):
        self._need_world()
        # AnnotationColorLandscape is an internal map of CARLA (no roads): loading it fails.
        return sorted({m.split("/")[-1] for m in self.client.get_available_maps()} - {"AnnotationColorLandscape"})

    def _switch_world(self, load):
        self._need_world()
        self._stop_cosim_if_running()
        self.cmd_clear_traffic()
        ego = self.ego
        self._drop_ego_refs()
        # Not left to the new map: if loading fails, the car stays behind in
        # the old one, untracked, and blocks its spawn point.
        self._try(lambda: ego.destroy() if self._alive(ego) else None)
        self.replay_before = None  # its actors go with the old world
        self.scenario_ids = []     # so do the 测试场景 props
        # The traffic manager runs inside this process and keeps its vehicle
        # registry across a world change; load_world() then segfaults in
        # libcarla. Shut it down first (a fresh one starts when needed).
        self._release_tm()
        self.client.set_timeout(180.0)
        try:
            self.world = load()
        except Exception:
            # The server may have switched after all (a time-out, "failed to
            # connect to newly created map"): the old world then fails every
            # call ("expired episode"). Follow the map the server is on now.
            self.client.set_timeout(20.0)
            self._try(lambda: setattr(self, "world", self.client.get_world()))
            self._log("换地图没有成功，后端改用 CARLA 当前的地图", "warn")
            raise
        finally:
            self.client.set_timeout(20.0)
        if self.recording:  # CARLA ends a recording together with its world
            self.recording = None
            self._log("换地图结束了 CARLA 录制", "warn")
        # (The run was stopped above.) An "error" / "finished" of it is not about this map.
        self.cosim_state, self.cosim_detail = "stopped", ""
        return self.cmd_world_info()

    def cmd_load_map(self, name):
        self._log("正在加载地图 %s ..." % name)
        return self._switch_world(lambda: self.client.load_world(name))

    def cmd_reload_world(self):
        return self._switch_world(lambda: self.client.reload_world())

    def cmd_world_settings(self, synchronous=None, frame_dt=None, no_rendering=None, idle_tick=None):
        w = self._need_world()
        if self.cosim_state in ("running", "paused"):
            # The run owns synchronous mode and the time step; changing them
            # now would silently break the lock-step with CarSim.
            raise RuntimeError("仿真运行中不能改仿真设置，请先停止运行")
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
        result, failed = {}, []
        for k, vid in enumerate(ids):
            if vid in self.spec_cache:
                result[vid] = self.spec_cache[vid]
                continue
            self.emit({"event": "progress", "task": "vehicle_specs", "done": k, "total": len(ids), "item": vid})
            tf = carla.Transform(carla.Location(base.location.x + 20.0 * k, base.location.y, 500.0))
            try:
                bp = bl.find(vid)
            except IndexError:  # not in this CARLA (e.g. a config from another build): measure the rest
                failed.append(vid)
                continue
            bp.set_attribute("role_name", PROBE_ROLE)
            actor = self._probe = w.try_spawn_actor(bp, tf)
            if actor is None:
                failed.append(vid)
                continue
            try:
                # Read before turning physics off: CARLA 0.9.16 then reads the wheel
                # list of multi-wheel vehicles (trucks, buses) out of bounds, which
                # crashes development builds such as the modified CARLA.
                pc = actor.get_physics_control()
                actor.set_simulate_physics(False)
                # The probe stands where it was spawned (physics off, never
                # ticked). Not actor.get_transform(): in synchronous mode the
                # client has no snapshot of the new actor yet and returns zeros,
                # which would leave the wheel positions in world coordinates.
                inv = tf.get_inverse_matrix()
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
                    spec["front_axle_z_m"] = round(bb.location.z - bb.extent.z, 3)  # ground, as bridge.front_axle_local
                self.spec_cache[vid] = result[vid] = spec
            finally:
                self._probe = None
                actor.destroy()
        self.emit({"event": "progress", "task": "vehicle_specs", "done": len(ids), "total": len(ids), "item": ""})
        if failed:
            self._log("以下车型无法测量（这个 CARLA 里没有，或生成失败）：%s" % "、".join(failed), "warn")
        return result

    def cmd_list_spawn_points(self):
        pts = self._need_world().get_map().get_spawn_points()
        return [{"index": i, "x": round(p.location.x, 2), "y": round(p.location.y, 2),
                 "z": round(p.location.z, 2), "yaw": round(p.rotation.yaw, 1)} for i, p in enumerate(pts)]

    def _drop_ego_refs(self):
        self.cmd_view_stop()
        self.ego = None
        self.anchor = None
        self.ego_autopilot = False

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
        random.Random(self.traffic_seed).shuffle(free)  # same seed, same new places
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
        ours |= {v["actor"].id for v in self.views.views.values()}
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
        self.traffic_seed = int(seed)
        rng = random.Random(self.traffic_seed)
        bl = w.get_blueprint_library()
        vbps = [b for b in bl.filter("vehicle.*") if not safe or
                (b.has_attribute("base_type") and b.get_attribute("base_type").as_str() == "car")]
        pts = w.get_map().get_spawn_points()
        rng.shuffle(pts)
        ego_loc = self.ego.get_location() if self._alive(self.ego) else None
        tm_port = self._need_tm().get_port()
        # Everything random in the traffic follows the seed too: the traffic
        # manager's choices (route at junctions, lane changes; setting its seed
        # also resets the traffic lights, so not in the middle of a run) and
        # the pedestrians' spawn points, destinations and road crossings.
        if not self._run_active():
            self.tm.set_random_device_seed(self.traffic_seed % 2 ** 32)
        w.set_pedestrians_seed(self.traffic_seed % 2 ** 32)
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
        # Walker controllers start once their bodies are in the world (next frame).
        new = [(c, 1.0 + rng.random()) for c in self.traffic["controllers"][len(self.traffic["controllers"]) - m:]]
        if self._tick_or_wait(1):
            self._start_walkers(new)
        else:
            self._pending_walkers += new  # during a run: after the run's next frame
        self._log("已生成交通：车辆 %d，行人 %d" % (n, m))
        return {"vehicles": len(self.traffic["vehicles"]), "walkers": len(self.traffic["walkers"])}

    def _start_walkers(self, ctrls):
        w = self.world
        for c, speed in ctrls:
            try:
                c.start()
                c.go_to_location(w.get_random_location_from_navigation())
                c.set_max_speed(speed)
            except RuntimeError:
                pass

    def cmd_clear_traffic(self):
        if self.world is None:
            return True
        try:
            for c in self.traffic["controllers"]:
                try:
                    c.stop()
                except RuntimeError:
                    pass
            ids = [a.id for k in ("controllers", "walkers", "vehicles") for a in self.traffic[k]]
            if ids:
                # During a run the run's next frame applies it (no extra tick).
                self.client.apply_batch_sync([carla.command.DestroyActor(i) for i in ids],
                                             self.world.get_settings().synchronous_mode and not self._run_active())
        finally:
            # Forgotten even when CARLA did not answer: destroyed later, these ids
            # could name other actors (e.g. a new ego) in a new world.
            self.traffic = {"vehicles": [], "walkers": [], "controllers": []}
            self._pending_walkers = []
        if not self.ego_autopilot and self.cosim_state not in ("running", "paused"):
            self._release_tm()  # nothing left for it to drive (see _need_tm)
        return True

    # ------------------------------------------------------------ live view
    def cmd_views_set(self, views):
        """Stream several sensors on the ego to the GUI at once (see views.py)."""
        w = self._need_world()
        if not self._alive(self.ego):
            raise RuntimeError("请先生成主车")
        return self.views.set(w, self.ego, list(views))

    def cmd_view_stop(self):
        self.views.stop()
        return True

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
        info.update({"w": int(img.shape[1]), "h": int(img.shape[0]), "sensor": sensor, "frame": int(frame)})
        if self.views.jpeg:  # a GUI at the other end of an SSH tunnel
            info["jpeg"] = encode_jpeg(img)
        else:
            info["rgb"] = base64.b64encode(np.ascontiguousarray(img).tobytes()).decode("ascii")
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
        # (A collector stopping on the worker still writes its last frames.)
        if any(c is not None and os.path.abspath(getattr(c, "root", "")) == root
               for c in (self.collector, self._stopping_collector)):
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
        # CARLA ends a recording in progress first, and answers "" when it
        # cannot create the file (it writes it itself, on its own machine).
        self.recording = path or None
        if not path:
            raise RuntimeError("CARLA 无法创建录制文件 %s：文件夹不存在或没有写权限" % os.path.abspath(filename))
        self._log("开始录制：%s" % filename)
        return path

    def cmd_stop_recorder(self):
        self._need_world()
        self.client.stop_recorder()
        self.recording = None
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
        if self.recording:  # CARLA ends a recording in progress when a replay starts
            self.recording = None
            self._log("回放会结束正在进行的录制，录制已停止", "warn")
        self.replay_before = {a.id for a in w.get_actors()}
        try:
            info = self.client.replay_file(os.path.abspath(filename), float(start), float(duration), int(follow_id))
        except Exception:
            self.replay_before = None  # nothing to clear later: it never started
            raise
        hero = None
        for _ in range(40):
            self._tick_or_wait(1)
            hero = next((a for a in w.get_actors().filter("vehicle.*")
                         if a.id not in self.replay_before and a.attributes.get("role_name") == "hero"), None)
            if hero is not None:
                self.ego, self.anchor = hero, None
                if view_specs:
                    self._try(lambda: self.cmd_views_set(view_specs))
                break
        self._log("回放开始：当前的主车和交通已清除" + ("，画面跟随录像里的主车" if hero is not None else "（录像里没有主车）"))
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
        if a.type_id.startswith("sensor.") and self._alive(self.ego) and a.parent is not None and a.parent.id == self.ego.id:
            # The live views and the run's sensors (scene, collection): without
            # one, the run would wait for its data every frame.
            raise RuntimeError("这是主车上的实时画面或当前运行用的传感器，不能单独删除：关闭画面或停止运行时会自动删除")
        doomed = [a]
        if a.type_id.startswith("walker."):  # with its AI controller, which would stay behind
            doomed = [c for c in self.world.get_actors().filter("controller.ai.walker")
                      if c.parent is not None and c.parent.id == a.id] + doomed
        ids = {x.id for x in doomed}
        tracked = any(x.id in ids for k in self.traffic for x in self.traffic[k])
        for k in self.traffic:  # traffic ids are destroyed again later (clear_traffic)
            self.traffic[k] = [x for x in self.traffic[k] if x.id not in ids]
        self._pending_walkers = [(c, s) for c, s in self._pending_walkers if c.id not in ids]
        for x in doomed[:-1]:
            self._try(x.stop)
            self._try(x.destroy)
        if a.type_id.startswith("controller."):
            self._try(a.stop)
        done = a.destroy()
        if tracked:  # part of the traffic: what is left of it, as spawn_traffic tells (the GUI shows the count)
            return {"vehicles": len(self.traffic["vehicles"]), "walkers": len(self.traffic["walkers"])}
        return done

    # ---------------------------------------------------------------- cosim
    def cmd_default_config(self):
        return st.default_dict()

    def _fix_world(self, d):
        """world.fixed: the map, weather and traffic of the config before the run (the same
        surroundings every time); what was done, for the output."""
        wd = d.get("world") or {}
        if not wd.get("fixed"):
            return None
        want = str(wd.get("map") or "")
        now = self._need_world().get_map().name.split("/")[-1]
        done = []
        if want and want != now:
            self._log("按配置加载地图 %s（现在是 %s）…" % (want, now))
            self.cmd_load_map(want)
            done.append("地图 %s" % want)
        if wd.get("weather"):
            self.cmd_set_weather(params=wd["weather"])
            done.append("天气")
        # The traffic: cleared now, spawned once the world is synchronous (_fix_world_traffic):
        # spawned in an asynchronous world it would drive on for however long the start takes.
        self.cmd_clear_traffic()
        return done

    def _fix_world_traffic(self, d, done):
        """world.fixed: the traffic of the config, in the run's synchronous world (from its first
        frame every step is a tick of the backend: the same positions every run)."""
        tr = (d.get("world") or {}).get("traffic") or {}
        nv, nw, seed = int(tr.get("vehicles") or 0), int(tr.get("walkers") or 0), int(tr.get("seed") or 0)
        if nv or nw:
            got = self.cmd_spawn_traffic(vehicles=nv, walkers=nw, seed=seed)
            done.append("交通流 %d 辆车、%d 个行人（种子 %d）" % (got["vehicles"], got["walkers"], seed))
        else:
            done.append("没有交通流")
        self._log("按配置重建世界：%s" % "，".join(done))

    def cmd_cosim_start(self, config=None):
        """Start a run: CarSim or CARLA dynamics, any driver, optional collection."""
        w = self._need_world()
        if self.cosim_state in ("running", "paused"):
            raise RuntimeError("已经有仿真在运行")
        d = st.load_dict(None, config or {})
        self._run_out = []  # this run's 输出, for output.txt in its record folder
        for note in check_run_config(d):
            self._log(note)
        world_done = self._fix_world(d)  # world.fixed: the map and weather of the config first (traffic: below)
        w = self._need_world()  # (another map: another world)
        c = d["carla"]
        col_cfg = dict(d["collect"])
        col_cfg["frame_dt"] = d["sync"]["frame_dt"]
        col_cfg["capture_every"] = d["collect"]["capture_every"] = st.sample_every(d)
        col_cfg["units"] = d["carsim"]["units"]  # radar speeds / angles are saved in CarSim's units
        rp = d["sync"]["reference_point"]
        # Sizes only (for the estimate); the mounts are fixed after the spawn below.
        sensors = d["rig"]["sensors"] or rigmod.build_preset(d["rig"]["preset"], self.spec_cache.get(c["vehicle"]), rp)
        if col_cfg["enabled"]:
            # Validate limits / disk space before touching the world.
            est = coll.DataCollector(w, None, sensors, col_cfg).estimate()
            di = coll.disk_info(col_cfg["out_dir"])
            if est["total_gb"] is None:
                raise RuntimeError("数据采集必须设置停止条件（帧数、时长或容量上限）")
            if di["free_gb"] is None:
                raise RuntimeError(di["error"])
            if est["total_gb"] > di["free_gb"] - coll.DISK_RESERVE_GB:
                raise RuntimeError("预计需要 %.1f GB，磁盘只剩 %.1f GB" % (est["total_gb"], di["free_gb"]))
        check_run_files(d)  # controller, .sim, python_carsim_env, CarSim solver
        # 测试场景: planned on the map now (a lane that is not there refuses the run
        # before anything changes), placed once the ego stands at the spawn point.
        scenario_plan = actor_plan = None
        sc = d["scenario"]
        if sc.get("enabled") and (sc.get("closures") or sc.get("actors")):
            pts = w.get_map().get_spawn_points()
            if not pts:
                raise RuntimeError("当前地图没有出生点")
            spawn_tf = pts[int(c["spawn_index"]) % len(pts)]
            if sc.get("closures"):
                scenario_plan = scenariomod.layout(w.get_map(), spawn_tf, sc["closures"])
            if sc.get("actors"):  # 动态目标: their roads worked out now, placed with the closures
                actor_plan = scenariomod.plan_actors(w.get_map(), spawn_tf, sc["actors"])
        # Every run starts with a fresh ego at the chosen spawn point, like a
        # CarSim run starts from its initial conditions. A teleported vehicle
        # keeps stale traffic-manager state (autopilot then brakes forever),
        # so respawn instead, and bring back the user's views.
        view_specs = self.views.specs()
        color = self.ego.attributes.get("color", "") if self._alive(self.ego) else ""
        old = (self.ego.type_id, self.ego.get_transform(), self.anchor) if self._alive(self.ego) else None
        prev_ego = self.ego
        try:
            self.cmd_spawn_ego(c["vehicle"], c["spawn_index"], color)
        except Exception:
            # The run did not start: put the previous ego back where it was, with
            # its views, so a failed start costs the user nothing.
            # (Nothing to do when the request was refused before the ego was touched.)
            if old is not None and not (self.ego is prev_ego and self._alive(prev_ego)):
                try:
                    self._restore_ego(old, color, view_specs)
                except Exception:
                    traceback.print_exc()  # report the original error, not this one
            raise
        try:
            if view_specs:
                self.cmd_views_set(view_specs)
            # Presets and older configs (CARLA frame) relative to this car's
            # measured reference point: the one the sensors are mounted on.
            ego_spec = rigmod.spec_of(self.ego)
            ref = rigmod.preset_reference(ego_spec, rp)
            if not d["rig"]["sensors"]:
                sensors = rigmod.build_preset(d["rig"]["preset"], ego_spec, ref)
            elif d["rig"].get("frame") == "carla":
                sensors = [rigmod.to_carsim(s, ref) for s in d["rig"]["sensors"]]
            d["rig"]["frame"] = "carsim"
            d["rig"]["sensors"] = sensors  # the scene's sensors for the algorithm are the same rig
        except BaseException:
            # Respawning already removed the previous ego and its attachments.
            # A failed attachment must restore them before returning an error.
            if old is not None:
                self._try(lambda: self._restore_ego(old, color, view_specs))
            else:
                self._try(self.cmd_destroy_ego)
            raise
        # The last run's cones go (and any a killed backend left: each run has exactly its
        # own); this run's are placed before the world settles (there at t0).
        self._try(lambda: self._clear_scenario(everywhere=True))
        if scenario_plan is not None:
            items, summary = scenario_plan
            self.scenario_ids = scenariomod.spawn(self.client, w, items)
            if len(self.scenario_ids) < len(items):
                self._log("测试场景：%d 个锥桶 / 护栏中有 %d 个没能放下（那里被别的物体占着）"
                          % (len(items), len(items) - len(self.scenario_ids)), "warn")
            self._log("测试场景：%s" % "；".join(
                "第 %d 处，出生点前方 %g m，%s，渐变段 %g m + 封闭 %g m，%d 个%s" % (
                    c["number"], c["distance_m"], scenariomod.lane_name(c["lane"]), c["taper_m"], c["length_m"],
                    c["props"], scenariomod.KIND_NAMES[c["kind"]]) for c in summary))
            if "static" not in (d["scene"].get("object_types") or []):
                self._log("测试场景的锥桶 / 护栏不会交给控制算法：“场景信息”页障碍物没有勾选“施工锥桶 / 护栏”", "warn")
        movers = None
        if actor_plan:
            movers = scenariomod.Movers(self.client, w, actor_plan, self.ego)
            self.scenario_ids += movers.ids
            placed = movers.summary()
            self._log("测试场景动态目标：%s" % "；".join(
                "第 %d 个%s，出生点前方 %g m%s，%g km/h%s" % (
                    a["number"], a["name"], a["distance_m"],
                    "，" + scenariomod.lane_name(a["lane"]) if a["type"] in ("slow_car", "cut_in") else "", a["speed_kmh"],
                    "" if a["placed"] else "（没能放下：那里被别的物体占着）") for a in placed))
        self._unsent_tel = None
        self._pre_cosim_settings = w.get_settings()
        try:
            # Settle under PhysX so the bridge reads a resting vehicle.
            s = w.get_settings()
            s.synchronous_mode, s.fixed_delta_seconds = True, d["sync"]["frame_dt"]
            w.apply_settings(s)
            self._tm_sync(True)
            if world_done is not None:
                self._fix_world_traffic(d, world_done)
            # The traffic lights start their cycle again with every run, like
            # the ego, instead of wherever the time before the run left them.
            w.reset_all_traffic_lights()
            for _ in range(20):
                w.tick()
            cosim = d["drive"]["dynamics"] == "cosim"
            autopilot = not cosim and d["drive"]["carla_driver"] == "autopilot"
            self.session = CoSimSession(w, self.ego, self.anchor, d) if cosim else \
                CarlaDriveSession(w, self.ego, d, self._need_tm() if autopilot else None, self.anchor)
            # For the run record (run.json): the seed of the traffic around the car, if there is any.
            self.session.run_meta = {"traffic_seed": self.traffic_seed if self.traffic["vehicles"] or
                                     self.traffic["walkers"] else None,
                                     "scenario": None if scenario_plan is None and movers is None else
                                     {"closures": scenario_plan[1] if scenario_plan else [],
                                      "placed": len(self.scenario_ids) - (len(movers.ids) if movers else 0),
                                      "actors": movers.summary() if movers else []}}
            if movers is not None:  # moved before every tick of the run, from its first
                ses = self.session
                ses.pre_tick = lambda: movers.step(ses.d["sync"]["frame_dt"])
                ses.velocity_overrides = movers.velocities
        except BaseException:
            # Never leave the world in sync mode with nobody ticking it.
            pre, self._pre_cosim_settings = self._pre_cosim_settings, None
            self._try(lambda: w.apply_settings(pre))
            self._try(lambda: self._tm_sync(pre.synchronous_mode))
            raise
        req_dt = d["sync"]["frame_dt"]
        try:
            info = self.session.start()
            self._algo_output()  # what the algorithm printed while loading
            # The session aligned frame_dt to CarSim's t_step: collect on that clock.
            col_cfg["frame_dt"] = d["sync"]["frame_dt"]
            col_cfg["capture_every"] = d["collect"]["capture_every"] = st.sample_every(d)
            if col_cfg["enabled"]:
                ses, sc = self.session, self.session.scene
                self.collector = coll.DataCollector(w, self.ego, sensors, col_cfg,
                                                    emit=lambda msg: self.emit(msg),  # the GUI connected now
                                                    # stock CARLA reads 0 for the teleported car
                                                    extra_state=ses.ego_motion if cosim else None,
                                                    shared=sc if sc is not None and sc.sensor_cfgs else None,
                                                    scene=sc, exports=ses.exports, ref_local=sc.ref_local,
                                                    export_names=ses.export_names(),
                                                    action=lambda: ses.last_action, n_actions=ses.n_actions())
                info["collect"] = self.collector.start()
                self.collector.on_tick(sc.frame, 0)  # step 0 (t0): the first sample, like the run record
                self._log("数据采集开始：%d 个传感器 → %s（预计 %.1f MB/s）" % (
                    len(self.collector.sensor_cfgs), info["collect"]["root"], info["collect"]["estimate"]["mb_per_s"]))
        except BaseException as e:  # SystemExit from a user controller included
            self._stop_cosim_if_running("error", str(e) or type(e).__name__)  # the banner says why
            raise
        col = self.collector
        # Step 0 can already end the run (a collision with "stop", collect.max_frames = 1):
        # finish below instead of running one more step.
        ended = self.session.done or (col is not None and col.done)
        if not ended:
            self._set_cosim_state("running")
        if info.get("server_api") is not None:
            self.server_api = info["server_api"]
        if cosim:
            self._log("联合仿真开始%s：%s，参考点 %s，每帧 %d 个 CarSim 步，驾驶：%s" % (
                "（模拟 CarSim）" if info["mock"] else "（Chrono 宝马 E90 代替 CarSim）" if info.get("chrono") else "",
                "改版 CARLA 接口" if info["external_api"] else "原版兼容模式",
                info["reference_point"], info["inner_steps"], d["run"]["driver"]))
            if abs(info["frame_dt"] - req_dt) > 1e-9:
                period, every = float(d["collect"].get("sample_period") or 0.0), col_cfg["capture_every"]
                self._log("仿真步长已对齐到 CarSim t_step（%g s）的整数倍：%g s → %g s%s" % (
                    info["t_step"], req_dt, info["frame_dt"],
                    "；采样周期 %g s → %g s（每 %d 帧）" % (period, every * info["frame_dt"], every)
                    if period > 0 and abs(every * info["frame_dt"] - period) > 1e-9 else ""), "warn")
            if not info["mock"] and not info.get("chrono") and info["t_stop"] > 0:
                self._log("这次运行最晚在 CarSim 的结束时间 t = %.1f s 停止（.sim 里设定）" % info["t_stop"])
            for msg in info.get("warnings", []):
                self._log(msg, "warn")
        else:
            self._log("仿真开始：CARLA 物理，驾驶：%s" % d["drive"]["carla_driver"])
        for hit in info.get("collisions", []):  # touching at the start: counted once, here
            self._log("碰撞：撞到 %s（id %s），t = %.2f s" % (hit["model"], hit["id"], info["t"]), "warn")
        warning = info.pop("warning", "")
        if warning:  # the run record stopped at step 0 (disk full): for the log, once
            self._log(warning, "warn")
        if ended:
            if col is not None and col.done:
                self._log("数据采集结束：%s，共 %d 帧，%.1f MB" % (col.stop_reason, col.frames, col.bytes / 1e6))
            reason = self.session.end_reason or "数据采集%s" % col.stop_reason
            self._log("运行结束（%s）：%.1f s" % (reason, info["t"]))
            self._stop_cosim_if_running("finished", reason)
        return info

    def _restore_ego(self, old, color, view_specs):
        """Put the previous ego (type, transform, anchor) back with its views."""
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

    def cmd_rig_build(self, preset, blueprint="", reference_point="front_axle"):
        """A preset fitted to the vehicle, in CarSim's vehicle frame."""
        spec = None
        if blueprint and self.world is not None:
            spec = self.spec_cache.get(blueprint) or self.cmd_vehicle_specs([blueprint]).get(blueprint)
        return rigmod.build_preset(preset, spec, reference_point)

    def cmd_rig_to_carsim(self, sensors, blueprint="", reference_point="front_axle"):
        """Mounts of an older config (CARLA frame: car centre, y right) in CarSim's
        vehicle frame, with the measured front axle of the vehicle."""
        spec = None
        if blueprint and self.world is not None:
            spec = self.spec_cache.get(blueprint) or self.cmd_vehicle_specs([blueprint]).get(blueprint)
        ref = rigmod.preset_reference(spec, reference_point)
        return [rigmod.to_carsim(s, ref) for s in sensors]

    def cmd_rig_estimate(self, sensors, collect=None, frame_dt=0.1):
        cfg = dict(st.default_dict()["collect"])
        cfg.update(collect or {})
        cfg["frame_dt"] = float(frame_dt)
        cfg["capture_every"] = st.sample_every({"collect": cfg, "sync": {"frame_dt": frame_dt}})
        c = coll.DataCollector(None, None, sensors, cfg)
        est = c.estimate()
        est["per_sensor"] = [{"name": s["name"], "mb_per_frame": rigmod.bytes_per_frame(s, cfg["image_format"], cfg["frame_dt"]) / 1e6}
                             for s in c.sensor_cfgs]
        est["disk"] = coll.disk_info(cfg["out_dir"])
        return est

    def cmd_disk_info(self, path="."):
        return coll.disk_info(path)

    def cmd_path_status(self, paths=()):
        """The GUI's check or cross under a path field, for files on this
        machine when the GUI runs on another one (remote_backend): relative
        paths resolve like a run's (from the bridge directory, the working directory)."""
        out = {}
        for p in paths:
            r = os.path.abspath(p) if p else ""
            out[p] = {"resolved": r, "exists": os.path.exists(r), "is_file": os.path.isfile(r)}
        return out

    def cmd_controller_params(self, path):
        """The algorithm file's tunable constants (session.controller_params: read, never run)."""
        return controller_params(os.path.abspath(path))

    def cmd_batch_summary(self, dir, items):
        """批量测试: the batch's report (report.csv, report.md in its dir) from its runs' run.json."""
        return batch_report.summarize(dir, items)

    def cmd_replay_pose(self, folder, t):
        """运行对比's 在 CARLA 里看这一刻: the ego placed where a recorded run had it at time t
        (the run's CarSim frame at its spawn point, as the run itself did; physics off)."""
        if self.cosim_state in ("running", "paused"):
            raise RuntimeError("仿真运行中不能回放：先停止运行")
        w = self._need_world()
        cache = self._replay_cache.get(folder)
        if cache is None:
            track = runsmod.ego_track(folder)
            cmap = w.get_map()
            now = cmap.name.split("/")[-1]
            if track["map"] and track["map"] != now:
                raise RuntimeError("这次运行在地图 %s 上，当前是 %s：先在“地图与天气”页加载 %s" % (track["map"], now, track["map"]))
            pts = cmap.get_spawn_points()
            anchor = anchor_frame(cmap, pts[int(track["spawn_index"] or 0) % len(pts)])
            cache = self._replay_cache[folder] = (track, anchor)
            if len(self._replay_cache) > 4:
                self._replay_cache.pop(next(iter(self._replay_cache)))
        track, anchor = cache
        if not self._alive(self.ego):
            raise RuntimeError("还没有主车：先在“车辆与视角”页生成主车，再回放")
        t, X, Y, Z, yaw = runsmod.pose_at(track, float(t))
        R = anchor.R
        ref = np.asarray(anchor.origin) + R @ np.array([X, -Y, Z])       # the reference point, CARLA world
        anchor_yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
        yaw_c = anchor_yaw - yaw                                          # CarSim yaw (left +) -> CARLA
        if self._replay_ref is None or self._replay_ref[0] != self.ego.id:
            self._replay_ref = (self.ego.id, front_axle_local(self.ego))
            self.ego.set_simulate_physics(False)  # held where it is put (the next run respawns the ego)
        loc = self._replay_ref[1]
        c, s = math.cos(math.radians(yaw_c)), math.sin(math.radians(yaw_c))
        centre = ref - np.array([c * loc[0] - s * loc[1], s * loc[0] + c * loc[1], loc[2]])
        self.ego.set_transform(carla.Transform(carla.Location(*map(float, centre)), carla.Rotation(yaw=float(yaw_c))))
        return {"t": t}

    def cmd_runs_list(self, path=""):
        """The run records in a record dir (relative: the bridge dir), newest first (runs.list_runs)."""
        return runsmod.list_runs(path or "runs")

    def cmd_check_sim(self, config=None):
        """检查 .sim: CarSim (or its stand-in) started once, its settings against the page's (simcheck.py)."""
        if self.cosim_state in ("running", "paused"):
            raise RuntimeError("仿真运行中不能检查：先停止运行")
        return simcheck.check(st.load_dict(None, config or {}))

    def cmd_templates_list(self):
        """文件 → 从模板新建: the templates (templates.py)."""
        return templates.listing()

    def cmd_template_config(self, id, current=None):
        """A template's config, this machine's settings of current (the open config) kept."""
        return templates.config(id, current)

    def cmd_vehicle_identify(self, folder, m, I, a, b, save_path=""):
        """车辆参数辨识 (vehicle_ident.py) from a run's record; save_path: also written as a KMPPI
        vehicle file (relative: to this folder)."""
        res = vehicle_ident.identify(folder, m, I, a, b)
        if save_path:
            res["saved"] = vehicle_ident.save(save_path, folder, m, I, a, b, res)
        return res

    def cmd_run_output(self, folder):
        """A run's 输出 (output.txt in its record folder) for the 运行对比 page."""
        return runsmod.run_output(folder)

    def cmd_run_series(self, folder, max_points=2000):
        """One run's time series for the GUI's comparison (runs.run_series)."""
        return runsmod.run_series(folder, int(max_points))

    def cmd_controller_browse(self, path=""):
        """The GUI's “浏览…” for the control algorithm when it runs on another
        computer (remote_backend): this machine's folders and .py files, each
        file with its entries and docstring (never run). See session.browse_controllers."""
        return browse_controllers(path)

    def _pause_clock(self, paused):
        """The real-time factor counts running time only."""
        clock = getattr(self.session, "clock", None)
        if clock is not None:
            clock.pause() if paused else clock.resume()

    def cmd_cosim_pause(self):
        if self.cosim_state == "running":
            self._set_cosim_state("paused")
            self._pause_clock(True)
            # Telemetry goes out every 2nd frame: send the frame we stopped on,
            # so the GUI shows exactly the paused state (and a step is +1).
            if self._unsent_tel is not None:
                self.emit({"event": "telemetry", "data": self._unsent_tel})
                self._unsent_tel = None
        return self.cosim_state

    def cmd_cosim_resume(self):
        if self.cosim_state == "paused":
            self._set_cosim_state("running")
            self._pause_clock(False)
        return self.cosim_state

    def cmd_cosim_step(self):
        """Single frame while paused."""
        if self.cosim_state != "paused":
            raise RuntimeError("仅在暂停时可单步")
        self._pause_clock(False)  # the step's own time counts, the pause does not
        self._cosim_frame(always_emit=True)
        self._pause_clock(True)
        return True

    def cmd_cosim_stop(self):
        self._stop_cosim_if_running("stopped")
        # Always report the state, so a GUI that is out of sync (e.g. after a
        # backend restart) stops showing "running".
        if self.cosim_state in ("running", "paused"):
            self._set_cosim_state("stopped")
        else:  # the same state again, with its reason (e.g. why the run ended)
            self._set_cosim_state(self.cosim_state, getattr(self, "cosim_detail", ""))
        return True

    def _stop_cosim_if_running(self, final="stopped", detail=""):
        """Best effort: every step runs even if an earlier one fails (CARLA may be
        gone), and afterwards there is no session and the state is `final`."""
        col, self.collector = self.collector, None
        if col is not None:
            # Until its last frames are on disk, dataset_delete (another thread)
            # must not take the directory away.
            self._stopping_collector = col
            self._try(col.stop)
            self._stopping_collector = None
        ses, self.session = self.session, None
        self._unsent_tel = None
        if ses is None:
            if self.cosim_state in ("running", "paused"):
                self._set_cosim_state(final, detail)
            return
        record_dir = getattr(ses, "record_dir", None)  # (the session forgets it when it stops)
        self._algo_output(ses, end=True)
        n = getattr(ses, "ctrl_n", 0)
        if isinstance(n, int) and n > 0:  # the user's control() ran
            self._log("算法耗时：control() 调用 %d 次，平均 %.2f ms，最长 %.2f ms（t = %.2f s）；仿真步长 %g ms" % (
                n, ses.ctrl_ms_sum / n, ses.ctrl_ms_max, ses.ctrl_ms_max_t, ses.d["sync"]["frame_dt"] * 1000.0))
        # The algorithm's finish(), the run record's run.json: they get how and why the run ended.
        summary = {}
        self.ending = ses  # a slow finish() is the algorithm's, not a hung CARLA (the heartbeat)
        own = self.task is None  # CARLA or the ego lost (no request running): the heartbeat needs a task
        if own:
            self.task = ("结束运行", time.time())
        self._try(lambda: summary.update(ses.stop(release_vehicle=True, end=final, reason=detail) or {}))
        if own:
            self.task = None
        self.ending = None
        self._algo_output(ses, end=True)  # what finish() printed, and where it failed, before its error
        for msg in summary.get("errors", []):  # e.g. an error in the algorithm's finish()
            self._log(msg, "warn")
        if summary.get("record_dir"):
            self._log("运行记录：%s" % summary["record_dir"])
        if summary.get("kpi_text"):
            self._log("运行指标：%s" % summary["kpi_text"])
        self._run_out_add("运行结束（%s）%s" % ({"finished": "完成", "stopped": "停止", "error": "出错"}.get(final, final),
                                            "：" + detail if detail else ""), folder=record_dir)
        self._run_out = None
        # Back to what the user had before co-sim (usually async), so the
        # world does not stay frozen in sync mode with nobody ticking.
        pre, self._pre_cosim_settings = getattr(self, "_pre_cosim_settings", None), None
        if pre is not None:
            self._try(lambda: self.world.apply_settings(pre))
        self._try(lambda: self._tm_sync(self.world.get_settings().synchronous_mode))
        self._try(self._park_ego)
        if self._pending_walkers:  # spawned while paused: no next frame of the run starts them
            pending, self._pending_walkers = self._pending_walkers, []
            self._try(lambda: self._tick_or_wait(1) and self._start_walkers(pending))
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
            self._algo_output()
            warning = tel.pop("warning", "")
            if warning:  # e.g. the run record stopped: disk full
                self._log(warning, "warn")
            if self._pending_walkers:
                pending, self._pending_walkers = self._pending_walkers, []
                self._start_walkers(pending)
            for hit in tel.get("collisions", []):
                self._log("碰撞：撞到 %s（id %s），t = %.2f s" % (hit["model"], hit["id"], tel["t"]), "warn")
            for msg in tel.pop("warnings", []):  # the log has them; not for the telemetry
                self._log(msg, "warn")
            if self.collector is not None:
                self.collector.on_tick(tel["world_frame"], tel["frame"])
                if self.collector.done:
                    self._log("数据采集结束：%s，共 %d 帧，%.1f MB" % (
                        self.collector.stop_reason, self.collector.frames, self.collector.bytes / 1e6))
                    tel["done"] = True
        except (Exception, SystemExit) as e:  # SystemExit: e.g. argparse in a user controller
            traceback.print_exc()
            self._algo_output()  # what the algorithm printed, and where it failed, before the error
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
        self._try(self._update_spectator)
        # Every 2nd frame is enough for the GUI, but a single step must show; the algorithm's
        # new lines (self.draw) of a frame not sent go with the next one.
        if "draw" not in tel and self._unsent_tel is not None and "draw" in self._unsent_tel:
            tel["draw"] = self._unsent_tel["draw"]
        if always_emit or tel["frame"] % 2 == 0 or tel["done"]:
            self.emit({"event": "telemetry", "data": tel})
            self._unsent_tel = None
        else:
            self._unsent_tel = tel
        if tel["done"]:
            # Tell the GUI exactly why the run ended.
            ses = self.session
            if ses.end_reason:
                reason = ses.end_reason
            elif self.collector is not None and self.collector.done:
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
        if self.idle_tick:
            # This backend kept a synchronous world going: nobody ticks it after
            # us. Hand it back asynchronous, not frozen (the next backend would
            # otherwise take it for one that did not exit cleanly).
            def to_async():
                s = self.world.get_settings()
                if s.synchronous_mode:
                    s.synchronous_mode, s.fixed_delta_seconds = False, None
                    self.world.apply_settings(s)
            self._try(to_async)

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
        # All CARLA calls belong to the worker: stop it taking work (queued
        # requests are dropped; a run start or traffic spawn after the cleanup
        # would leave actors behind) and let it finish the current one.
        self.exiting = True
        while True:
            try:
                self.requests.get_nowait()
            except queue.Empty:
                break
        while self.task is not None and time.time() - t0 < 2.0:
            time.sleep(0.05)
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

    def submit(self, req, reply):
        (self.io_requests if req.get("cmd") in IO_CMDS else self.requests).put((req, reply))

    def run_io_worker(self):
        """Serves IO_CMDS beside the worker: they never touch CARLA."""
        while True:
            req, reply = self.io_requests.get()
            try:
                reply({"id": req.get("id"), "ok": True, "result": self.handle(req)})
            except BaseException as e:  # this thread must not die either
                traceback.print_exc()
                reply({"id": req.get("id"), "ok": False, "error": str(e) or e.__class__.__name__})

    def heartbeat_step(self, last_look=0.0):
        """Once a second, from its own thread: tell the GUI what the worker is
        busy with, so a CARLA call that never returns shows up as such instead
        of as a silently frozen GUI. Returns when it last looked for CARLA."""
        task = self.task
        if task is not None and time.time() - task[1] >= 2.0:
            # A CARLA call that waits because CARLA is gone: say so now, and
            # let the calls after it give up quickly.
            gone = self.world is not None and self._carla_listening_now(record=False) is False
            if gone:
                self._fast_timeout = True
                self._try(lambda: self.client.set_timeout(0.5))
            ev = {"event": "busy", "task": task[0], "seconds": round(time.time() - task[1], 1), "carla_gone": gone}
            ses = self.session
            ctl = control_busy(getattr(ses or self.ending, "driver", None))
            if ctl is not None:  # the user's control() / finish() has not returned: not a CARLA problem
                ev.update(task="control" if ses is not None else "finish",
                          seconds=round(time.time() - ctl[0], 1), where=ctl[1])
            self.emit(ev)
        elif os.name == "nt" and self.world is not None and time.time() - last_look >= 5.0:
            # The worker's check every 2 s cannot run netstat (too slow between
            # a run's frames): look here, and have the worker check at once.
            last_look = time.time()
            if self._carla_listening_now(record=False) is False:
                self._gone_hint = True
        return last_look

    def run_worker(self):
        """The one thread that touches CARLA. It must never die: an uncaught
        error here would leave the GUI waiting forever for replies."""
        while True:
            try:
                self._worker_iteration()
            except BaseException as e:
                traceback.print_exc()
                self.task = None
                self._try(lambda: self._stop_cosim_if_running("error", "后端内部错误：%s: %s" % (type(e).__name__, e)))
                time.sleep(0.1)

    def _worker_iteration(self):
        if getattr(self, "exiting", False):  # exit_now() is cleaning up
            time.sleep(0.05)
            return
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
                # Not while paused: a paused run keeps the world where it is
                # (and its collector would pile up sensor data meanwhile).
                sync = self.world.get_settings().synchronous_mode
                if self.idle_tick and self.cosim_state != "paused" and sync:
                    self.task = ("空闲时推进世界", time.time())
                    try:
                        self.world.tick()
                    finally:
                        self.task = None
                    time.sleep(self.frame_dt)
                elif self.traffic["walkers"] and self.cosim_state != "paused" and not sync:
                    # CARLA moves pedestrians in the client that spawned them, on its
                    # world.tick() / wait_for_tick(): with an asynchronous world
                    # nothing else here calls them, and the pedestrians stand still.
                    self.world.wait_for_tick(2.0)
                self._update_spectator()
            except RuntimeError as e:
                if "time-out" in str(e):
                    self._check_carla(now=True)


def _socket_thread_request(req, send):
    """Requests the socket thread answers itself, since the worker may be the
    thread that hangs: "dump_stacks" prints every thread's stack into the log
    (the GUI sends it before restarting a backend where there is no SIGUSR1,
    i.e. on Windows); "restart_backend" does the same and then exits with code
    3 (a GUI whose backend runs on a server, remote_backend: the session script
    there starts a new one). False: a request for the worker."""
    cmd = req.get("cmd")
    if cmd not in ("dump_stacks", "restart_backend"):
        return False
    faulthandler.dump_traceback(all_threads=True)
    send({"id": req.get("id"), "ok": True, "result": True})
    if cmd == "restart_backend":
        print("restart_backend: exiting with code 3", flush=True)
        os._exit(3)
    return True


def serve(port, exit_with_client=False, carsim_port=0):
    # A crash inside the carla library (a C++ thread) leaves no Python error:
    # print every thread's stack into the log instead. SIGUSR1 prints them for
    # a backend that hangs (the GUI sends it before restarting one; on Windows
    # a "dump_stacks" request, see _socket_thread_request).
    faulthandler.enable(all_threads=True)
    if hasattr(signal, "SIGUSR1"):
        faulthandler.register(signal.SIGUSR1, all_threads=True)
    backend = Backend()
    atexit.register(backend.cleanup)
    atexit.register(chrono_local.stop)  # the Chrono BMW's service process, if one was started
    signal.signal(signal.SIGTERM, lambda *_: backend.exit_now())
    # Ctrl+C in the terminal of a backend started by hand: the same bounded
    # clean exit, never a cleanup on this thread racing the worker.
    signal.signal(signal.SIGINT, lambda *_: backend.exit_now())
    threading.Thread(target=backend.run_worker, daemon=True).start()
    threading.Thread(target=backend.run_io_worker, daemon=True).start()

    def heartbeat():
        last_look = 0.0
        while True:
            time.sleep(1.0)
            last_look = backend.heartbeat_step(last_look)
    threading.Thread(target=heartbeat, daemon=True).start()
    if carsim_port:  # remote mode: where the CarSim service connects (carsim_remote.py)
        carsim_remote.SERVICE.on_change = backend._carsim_service_changed
        carsim_remote.SERVICE.listen(carsim_port)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    if os.name == "nt":  # there SO_REUSEADDR lets a second backend listen on the same port
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
    else:
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
    serve_clients(srv, backend, exit_with_client)


def serve_clients(srv, backend, exit_with_client=False):
    """One GUI at a time: a second one is told so and let go (it would
    otherwise wait for replies forever)."""
    in_use = threading.Lock()
    leaving = threading.Event()  # the GUI has gone and this backend exits with it
    parked = []
    # Back in Python every 0.5 s: on Windows a blocking accept() holds off the
    # Ctrl+C handler until the next client connects.
    srv.settimeout(0.5)

    def serve_client(conn):
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
                    if not isinstance(req, dict):
                        send({"event": "log", "level": "error", "msg": "后端收到的请求不是 JSON 对象，已忽略"})
                        continue
                    if _socket_thread_request(req, send):
                        continue
                    backend.submit(req, send)
        except OSError:
            pass
        finally:
            backend.emit = lambda msg: None
            if exit_with_client:
                leaving.set()  # before the close: a GUI that reconnects at once is not turned away
            conn.close()
            if exit_with_client:
                # Started by the GUI: when it disconnects (closed, crashed, killed),
                # clean CARLA up and quit instead of lingering as an orphan.
                print("GUI disconnected, cleaning up and exiting", flush=True)
                backend.exit_now()
            in_use.release()

    while True:
        try:
            conn, _ = srv.accept()
        except socket.timeout:
            continue
        conn.settimeout(None)
        if not in_use.acquire(blocking=False):
            if leaving.is_set() or getattr(backend, "exiting", False) or getattr(backend, "_cleaned", False):
                # On the way out (the GUI closed and opened again right away):
                # left waiting, it is dropped when this process ends, and the
                # GUI then connects to the backend it started.
                parked.append(conn)
                continue
            msg = {"event": "log", "level": "error", "rejected": True,
                   "msg": "这个后端（端口 %d）已被另一个界面窗口使用，请关闭这个窗口" % srv.getsockname()[1]}
            try:
                conn.sendall((json.dumps(msg, ensure_ascii=False) + "\n").encode("utf-8"))
            except OSError:
                pass
            conn.close()
            continue
        threading.Thread(target=serve_client, args=(conn,), daemon=True).start()


if __name__ == "__main__":
    # print() in the user's control() also goes to backend.log. On Windows a
    # redirected stdout uses the ANSI code page (cp936), where printing e.g.
    # "✓" raises inside control() and stops the run.
    for stream in (sys.stdout, sys.stderr):
        if stream is not None and hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="backslashreplace")
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=57100)
    ap.add_argument("--exit-with-client", action="store_true",
                    help="clean up and exit when the client disconnects")
    ap.add_argument("--carsim-port", type=int, default=0,
                    help="where the CarSim service connects in remote mode (127.0.0.1, e.g. 57121); "
                         "0 (default) = nowhere")
    a = ap.parse_args()
    serve(a.port, a.exit_with_client, a.carsim_port)
