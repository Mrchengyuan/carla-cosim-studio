"""One co-simulation run, stepped frame by frame.

Shared by run_cosim.py (CLI loop) and backend_server.py (GUI backend), so
both run exactly the same code path.
"""

import csv
import importlib.util
import inspect
import math
import os
import sys
import time
import traceback

import carla

import rig as rigmod
import settings as st
from bridge import CarlaVehicleSync
from drivers import ManualDriver, RouteFollower
from scene import SceneProvider, gui_view


def demo_driver(t):
    """Accelerate, then a slalom: exercises steer, roll, pitch and spin."""
    throttle = 0.6 if t < 6.0 else 0.25
    brake = 0.4 if 14.0 < t < 15.5 else 0.0
    steer_sw = 0.0 if t < 3.0 else 90.0 * math.sin(2 * math.pi * 0.25 * (t - 3.0))
    return [0.0 if brake else throttle, brake, steer_sw]


def make_env(d):
    c = d["carsim"]
    if c["mock"]:
        from mock_carsim import MockCarSimEnv
        dur = d["sync"]["duration"]
        return MockCarSimEnv(c["export_names"], t_stop=dur + 1.0 if dur > 0 else 1e9)
    if not c["sim_path"]:
        raise ValueError("CarSim .sim file not set (carsim.sim_path)")
    repo = os.path.abspath(c["repo_path"])
    if repo in sys.path:
        sys.path.remove(repo)
    sys.path.insert(0, repo)
    # A different python_carsim_env folder than last run: import that one.
    old = sys.modules.get("carsim_env")
    if old is not None and os.path.dirname(os.path.abspath(getattr(old, "__file__", ""))) != repo:
        for m in ("carsim_env", "vs_solver"):
            sys.modules.pop(m, None)
    from carsim_env import CarSimEnv
    return CarSimEnv(c["sim_path"])


def make_driver(d, ex, n_imports=None, scene=None):
    drv = d["run"]["driver"]
    if drv == "demo":
        return lambda obs, t: demo_driver(t)
    if drv == "custom":
        return load_controller(d, ex, n_imports, scene)
    raise ValueError("unknown CarSim driver '%s'" % drv)


def _user_error(what, e, path):
    """'控制算法出错：ValueError: boom（my_ctrl.py 第 12 行）': the user needs the
    line of their own file, not the backend's."""
    line = next((f.lineno for f in reversed(traceback.extract_tb(e.__traceback__))
                 if os.path.abspath(f.filename) == path), None)
    msg, fname = str(e), path
    if isinstance(e, SyntaxError) and e.lineno:  # possibly in a module the algorithm imports
        line, msg, fname = e.lineno, e.msg, e.filename or path
    where = "（%s 第 %d 行）" % (os.path.basename(fname), line) if line else ""
    return "%s：%s: %s%s" % (what, type(e).__name__, msg, where)


def _wants_scene(fn):
    """control(exports, t, dt, scene): a 4th parameter (or *args) asks for the scene."""
    try:
        ps = list(inspect.signature(fn).parameters.values())
    except (TypeError, ValueError):
        return False
    if any(p.kind == p.VAR_POSITIONAL for p in ps):
        return True
    return len([p for p in ps if p.kind in (p.POSITIONAL_ONLY, p.POSITIONAL_OR_KEYWORD)]) >= 4


def load_controller(d, ex, n_imports=None, scene=None):
    """The user's control algorithm, loaded fresh from its file at every run
    start (edits apply on the next run). The entry is a class (instantiated,
    reset() called if present, then control() every frame) or a function:

        control(exports, t, dt) -> values for the CarSim imports, .sim order
        control(exports, t, dt, scene) -> the same, also given what is around
                                          the car (scene.py; scene() returns it)

    exports is {name: value} of every CarSim export, in CarSim units.
    See controllers/example_controller.py.
    """
    c = d["run"]["controller"]
    path = os.path.abspath(c["path"])
    if not os.path.isfile(path):
        raise ValueError("控制算法文件不存在：%s" % path)
    # Let the algorithm import its neighbours and python_carsim_env modules.
    folder = os.path.dirname(path)
    for p in (os.path.abspath(d["carsim"]["repo_path"]), folder):
        if p not in sys.path:
            sys.path.insert(0, p)
    # "Reloaded every run" also for helper modules next to the algorithm file.
    # (Not when the file sits next to the backend's own modules.)
    if folder != os.path.dirname(os.path.abspath(__file__)):
        for name, m in list(sys.modules.items()):
            f = getattr(m, "__file__", None)
            if f and os.path.dirname(os.path.abspath(f)) == folder:
                del sys.modules[name]
    spec = importlib.util.spec_from_file_location("user_controller", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["user_controller"] = mod  # dataclasses & co. look the module up here
    entry = c.get("entry") or "Controller"
    try:
        spec.loader.exec_module(mod)
        obj = getattr(mod, entry, None)
        if obj is None:
            raise ValueError("%s 里没有找到 %s" % (os.path.basename(path), entry))
        if isinstance(obj, type):
            obj = obj()
            if hasattr(obj, "reset"):
                obj.reset()
            obj = obj.control
    except SystemExit as e:  # e.g. argparse at module level: must not end the backend
        raise RuntimeError("控制算法 %s 在加载时调用了 sys.exit（常见原因：模块顶层用了 argparse）：%s"
                           % (os.path.basename(path), e))
    except ValueError as e:
        if "没有找到" in str(e):
            raise
        raise RuntimeError(_user_error("加载控制算法出错", e, path)) from e
    except Exception as e:
        raise RuntimeError(_user_error("加载控制算法出错", e, path)) from e
    dt = d["sync"]["frame_dt"]
    names = list(ex.index)
    with_scene = _wants_scene(obj)

    def control(obs, t):
        exports = {n: ex.raw(obs, n) for n in names}
        try:
            out = obj(exports, t, dt, scene() if scene else None) if with_scene else obj(exports, t, dt)
        except KeyError as e:
            if e.args and e.args[0] not in exports:
                raise RuntimeError(_user_error("控制算法出错", e, path) +
                                   "——导出变量里没有 %r（导出变量在“CarSim 动力学”页设置，现有：%s）"
                                   % (e.args[0], "、".join(names[:12]) + (" ..." if len(names) > 12 else ""))) from e
            raise RuntimeError(_user_error("控制算法出错", e, path)) from e
        except Exception as e:
            raise RuntimeError(_user_error("控制算法出错", e, path)) from e
        n = n_imports() if n_imports else None
        want = "按 .sim 里导入变量的顺序返回 %s 个数，例如 [油门, 制动, 方向盘角]" % (n or "若干")
        if out is None:
            raise RuntimeError("控制算法的 control() 返回了 None（是不是忘了 return？）；应%s" % want)
        if isinstance(out, (dict, str, bytes)):
            raise RuntimeError("控制算法的 control() 返回了 %s；应%s" % (type(out).__name__, want))
        try:
            vals = [float(v) for v in out]
        except (TypeError, ValueError):
            raise RuntimeError("控制算法的 control() 返回值不是一组数字：%s；应%s" % (repr(out)[:80], want))
        if n and len(vals) != n:
            # CarSim would silently fill missing imports with 0 (e.g. no steering).
            raise RuntimeError("控制算法返回了 %d 个值，但 .sim 里有 %d 个导入变量；应%s" % (len(vals), n, want))
        return vals
    return control


def spawn_chase_camera(world, vehicle, out_dir):
    bp = world.get_blueprint_library().find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", "960")
    bp.set_attribute("image_size_y", "540")
    tf = carla.Transform(carla.Location(x=-6.5, z=3.0), carla.Rotation(pitch=-15))
    cam = world.spawn_actor(bp, tf, attach_to=vehicle)
    os.makedirs(out_dir, exist_ok=True)
    cam.listen(lambda img: img.save_to_disk(os.path.join(out_dir, "%06d.png" % img.frame))
               if img.frame % 10 == 0 else None)
    return cam


def start_scene(world, vehicle, d, ego_velocity=None):
    """The SceneProvider of a run: rig sensors only when the algorithm asked for them."""
    sensors = []
    if d["scene"].get("sensors"):
        sensors = d["rig"]["sensors"] or rigmod.build_preset(d["rig"]["preset"])
    sp = SceneProvider(world, vehicle, d["scene"], sensors, d["sync"]["frame_dt"])
    try:
        sp.start(ego_velocity)
    except BaseException:
        sp.stop()
        raise
    return sp


def scene_step(ses, world_frame, ego_velocity=None):
    """Update the scene after a tick; handle contacts. Returns telemetry fields."""
    scene = ses.scene.update(world_frame, ego_velocity)
    new = [c for c in scene["collisions"] if c["new"]]
    policy = ses.d["scene"].get("collision", "log")
    if policy == "off":
        new = []
    elif new and policy == "stop" and not ses.end_reason:
        c = new[0]
        ses.end_reason = "碰撞：撞到 %s（id %s）" % (c["type_id"], c["id"])
    return {"scene": gui_view(scene), "collisions": new}


class CoSimSession:
    """Lock-step CarSim + CARLA run for one already-spawned vehicle."""

    def __init__(self, world, vehicle, anchor, d):
        self.world, self.vehicle, self.anchor, self.d = world, vehicle, anchor, d
        self.env = self.sync = self.driver = self.camera = None
        self._log_file = self._log = None
        self._original_settings = None
        self.frame = 0
        self.done = False
        self.scene = None
        self.end_reason = ""

    # --------------------------------------------------------------- lifecycle
    def start(self):
        d, w = self.d, self.world
        frame_dt = d["sync"]["frame_dt"]
        self._original_settings = w.get_settings()
        s = w.get_settings()
        s.synchronous_mode = True
        s.fixed_delta_seconds = frame_dt
        w.apply_settings(s)

        ext = d["sync"]["use_external_api"]
        self.sync = CarlaVehicleSync(
            w, self.vehicle, self.anchor,
            use_external_api=None if ext == "auto" else bool(ext),
            settings=st.to_bridge_cfg(d))
        drv = d["run"]["driver"]
        self.command_driver = None
        self._speed = 0.0
        if drv in ("route", "manual"):
            dr = d["drive"]
            if drv == "route":
                dest = int(dr.get("destination_index", -1))
                pts = w.get_map().get_spawn_points()
                self.command_driver = RouteFollower(
                    w, self.vehicle, dr["target_speed_kmh"],
                    destination=pts[dest].location if 0 <= dest < len(pts) else None)
            else:
                self.command_driver = ManualDriver()
            sw_max = float(d["sync"]["steering_wheel_max_deg"])
            scale = float(dr.get("brake_scale", 1.0))
            self.driver = lambda obs, t: self.command_driver.step(
                self.vehicle, self._speed, frame_dt).to_carsim(sw_max, scale)
        else:
            self.driver = make_driver(d, self.sync.ex, lambda: self.env.config.get("n_import"),
                                      lambda: self.scene.latest)
        self.env = make_env(d)
        if d["run"]["record_dir"]:
            self.camera = spawn_chase_camera(w, self.vehicle, d["run"]["record_dir"])

        self.obs = self.env.reset()
        # The .sim defines how many exports / imports there are, in which order.
        n_exp, n_imp = self.env.config.get("n_export"), self.env.config.get("n_import")
        names = d["carsim"]["export_names"]
        if n_exp and len(names) != int(n_exp):
            raise RuntimeError("导出变量个数不一致：界面里列了 %d 个，.sim 里有 %d 个。顺序和个数必须与 .sim 的导出变量一致"
                               "（“CarSim 动力学”页），否则位姿会用错变量" % (len(names), int(n_exp)))
        if d["run"]["driver"] != "custom" and n_imp and int(n_imp) != 3:
            raise RuntimeError("测试用驾驶方式只给 3 个导入变量（油门、制动、方向盘角），.sim 里有 %d 个；"
                               "请用你自己的控制算法按 .sim 的导入顺序返回" % int(n_imp))
        # Put the car where CarSim starts (reference point on the spawn point)
        # before the first control() call, so its scene shows the real start.
        state0 = self.sync.sync(self.obs, self.env.t_current, frame_dt)
        w.tick()
        self.scene = start_scene(w, self.vehicle, d, state0.velocity)
        t_step = self.env.config["t_step"]
        self.inner = max(1, int(round(frame_dt / t_step)))
        self.clock_warning = abs(self.inner * t_step - frame_dt) > 1e-9
        # duration <= 0: run until stopped (or until CarSim reaches t_stop).
        self.n_frames = max(1, int(round(d["sync"]["duration"] / frame_dt))) if d["sync"]["duration"] > 0 else 0

        if d["run"]["log_path"]:
            self._log_file = open(d["run"]["log_path"], "w", newline="", encoding="utf-8")
            self._log = csv.writer(self._log_file)
            self._log.writerow(["t", "carsim_x", "carsim_y", "carsim_yaw", "cmd_x", "cmd_y", "cmd_yaw",
                                "carla_x", "carla_y", "carla_yaw", "steer_fl", "steer_fr", "speed_carla"])
        self._wall0 = time.perf_counter()
        return {"external_api": self.sync.external_api, "server_api": self.sync.server_api,
                "reference_point": [round(float(x), 3) for x in self.sync.ref_local],
                "t_step": t_step, "inner_steps": self.inner, "clock_warning": self.clock_warning}

    def stop(self, release_vehicle=True):
        """Best effort: one failing step (CarSim or CARLA gone) must not skip the rest."""
        def step(fn):
            try:
                fn()
            except Exception as e:
                print("CoSimSession.stop: %s" % e, flush=True)
        if self.env is not None:
            step(self.env.close)
        if self.scene is not None:
            step(self.scene.stop)
        if self.camera is not None:
            step(self.camera.stop)
            step(self.camera.destroy)
            self.camera = None
        if self._log_file is not None:
            step(self._log_file.close)
            self._log_file = None
        if release_vehicle and self.sync is not None:
            step(self.sync.release)
        if self._original_settings is not None:
            orig, self._original_settings = self._original_settings, None
            step(lambda: self.world.apply_settings(orig))

    # -------------------------------------------------------------------- step
    def step(self):
        """Advance one CARLA frame. Returns a telemetry dict."""
        env, frame_dt = self.env, self.d["sync"]["frame_dt"]
        action = self.driver(self.obs, env.t_current)
        if not all(math.isfinite(a) for a in action):
            raise RuntimeError("控制算法输出了无效数值（NaN / 无穷大）：%s" % list(action))
        self.obs, _, done, info = env.control_step(action, self.inner)
        if info.get("error"):
            raise RuntimeError("CarSim error: %s" % info["error"])
        bad = [n for n, i in self.sync.ex.index.items() if i < len(self.obs) and not math.isfinite(float(self.obs[i]))]
        if bad:
            # Never hand NaN / inf to CARLA as a pose; stop with a clear reason.
            raise RuntimeError("CarSim 输出了无效数值（NaN / 无穷大），仿真已停止：%s" % ", ".join(bad[:6]))
        state = self.sync.sync(self.obs, env.t_current, frame_dt)
        world_frame = self.world.tick()
        self.frame += 1
        scene_tel = scene_step(self, world_frame, state.velocity)  # CarSim's velocity, not the teleported actor's
        v = state.velocity
        self._speed = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)

        snap = self.vehicle.get_transform()
        vel = self.vehicle.get_velocity()
        ex = self.sync.ex
        if self._log is not None:
            self._log.writerow([round(env.t_current, 4),
                                ex.raw(self.obs, "Xo"), ex.raw(self.obs, "Yo"), ex.angle(self.obs, "Yaw"),
                                state.transform.location.x, state.transform.location.y,
                                state.transform.rotation.yaw,
                                snap.location.x, snap.location.y, snap.rotation.yaw,
                                state.wheel_steer[0], state.wheel_steer[1],
                                math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)])
        self.done = bool(done) or (self.n_frames > 0 and self.frame >= self.n_frames) or bool(self.end_reason)
        wall = time.perf_counter() - self._wall0
        v = state.velocity
        return {**scene_tel,
            "t": env.t_current,
            "frame": self.frame,
            "n_frames": self.n_frames,
            "rt_factor": env.t_current / wall if wall > 0 else 0.0,
            "speed_kmh": math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2) * 3.6,
            "location": [snap.location.x, snap.location.y, snap.location.z],
            "rotation": [snap.rotation.pitch, snap.rotation.yaw, snap.rotation.roll],
            "wheel_steer": list(state.wheel_steer),
            "wheel_rotation": list(state.wheel_rotation),
            "wheel_suspension_mm": [x * 1000.0 for x in state.wheel_suspension],
            "action": [float(a) for a in action],
            "world_frame": world_frame,
            "dynamics": "CarSim",
            "done": self.done,
        }

    def carsim_state(self):
        """CarSim exports of the current step, for the data collector."""
        ex = self.sync.ex
        return {"carsim": {n: ex.raw(self.obs, n) for n in ex.index}}


class CarlaDriveSession:
    """Same interface as CoSimSession, but CARLA PhysX drives the vehicle."""

    def __init__(self, world, vehicle, d, traffic_manager):
        self.world, self.vehicle, self.d, self.tm = world, vehicle, d, traffic_manager
        self.frame = 0
        self.done = False
        self.command_driver = None
        self._original_settings = None
        self.scene = None
        self.end_reason = ""

    def start(self):
        d, w = self.d, self.world
        dt = d["sync"]["frame_dt"]
        self._original_settings = w.get_settings()
        s = w.get_settings()
        s.synchronous_mode, s.fixed_delta_seconds = True, dt
        w.apply_settings(s)
        if self.tm is not None:  # only the autopilot needs the traffic manager
            self.tm.set_synchronous_mode(True)
        # Front wheel angles for the telemetry are computed the way PhysX does
        # (steer x max angle x speed curve for the inner wheel, Ackermann for
        # the outer one), not read with get_wheel_steer_angle(): that call
        # waits for the server holding the GIL (original carla package), which
        # deadlocks with the sensor callbacks of large live views.
        pc = self.vehicle.get_physics_control()
        self._steer_geo = None
        if len(pc.wheels) >= 4:
            inv = self.vehicle.get_transform().get_inverse_matrix()
            pos = [[sum(inv[r][k] * p[k] for k in range(4)) for r in range(2)]
                   for p in ([w.position.x / 100.0, w.position.y / 100.0, w.position.z / 100.0, 1.0] for w in pc.wheels[:4])]
            wheelbase = (pos[0][0] + pos[1][0]) / 2 - (pos[2][0] + pos[3][0]) / 2
            track = abs(pos[1][1] - pos[0][1])
            curve = sorted((p.x, p.y) for p in pc.steering_curve) or [(0.0, 1.0)]
            self._steer_geo = (pc.wheels[0].max_steer_angle, curve, wheelbase, track)
        dr = d["drive"]
        self.mode = dr["carla_driver"]
        if self.mode == "autopilot":
            self.vehicle.set_autopilot(True, self.tm.get_port())
            self.tm.vehicle_percentage_speed_difference(self.vehicle, float(dr.get("tm_speed_diff_pct", 0.0)))
            self.tm.ignore_lights_percentage(self.vehicle, 100.0 if dr.get("tm_ignore_lights") else 0.0)
        elif self.mode == "route":
            dest = int(dr.get("destination_index", -1))
            pts = w.get_map().get_spawn_points()
            self.command_driver = RouteFollower(w, self.vehicle, dr["target_speed_kmh"],
                                                destination=pts[dest].location if 0 <= dest < len(pts) else None)
        else:
            self.command_driver = ManualDriver()
        self.n_frames = max(1, int(round(d["sync"]["duration"] / dt))) if d["sync"]["duration"] > 0 else 0
        self.scene = start_scene(w, self.vehicle, d)
        self._wall0 = time.perf_counter()
        return {"external_api": False, "server_api": None, "reference_point": [0, 0, 0], "t_step": dt, "inner_steps": 1,
                "clock_warning": False, "dynamics": "CARLA"}

    def _wheel_angles(self, steer, speed_kmh):
        """[FL, FR] steer angle, deg, + = right (as get_wheel_steer_angle)."""
        if self._steer_geo is None or abs(steer) < 1e-4:
            return [0.0, 0.0]
        max_deg, curve, wheelbase, track = self._steer_geo
        k = curve[-1][1]
        for (x0, y0), (x1, y1) in zip(curve, curve[1:]):
            if speed_kmh <= x1:
                k = y0 + (y1 - y0) * max(0.0, speed_kmh - x0) / max(1e-6, x1 - x0)
                break
        if speed_kmh <= curve[0][0]:
            k = curve[0][1]
        inner = min(abs(steer), 1.0) * max_deg * k
        outer = math.degrees(math.atan(wheelbase / (wheelbase / math.tan(math.radians(inner)) + track))) \
            if wheelbase > 0 and inner > 1e-3 else inner
        # Turning right: the right wheel is the inner one.
        return [outer, inner] if steer > 0 else [-inner, -outer]

    def stop(self, release_vehicle=True):
        try:
            if getattr(self, "mode", None) == "autopilot":  # start() may have failed before setting it
                self.vehicle.set_autopilot(False, self.tm.get_port())
        except RuntimeError:
            pass
        if self.scene is not None:
            self.scene.stop()
        if self._original_settings is not None:
            self.world.apply_settings(self._original_settings)
            self._original_settings = None

    def step(self):
        dt = self.d["sync"]["frame_dt"]
        v = self.vehicle.get_velocity()
        speed = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
        if self.command_driver is not None:
            self.vehicle.apply_control(self.command_driver.step(self.vehicle, speed, dt).to_carla())
        world_frame = self.world.tick()
        self.frame += 1
        scene_tel = scene_step(self, world_frame)
        t = self.frame * dt
        tf = self.vehicle.get_transform()
        c = self.vehicle.get_control()
        steer = self._wheel_angles(c.steer, speed * 3.6)
        self.done = (self.n_frames > 0 and self.frame >= self.n_frames) or bool(self.end_reason)
        wall = time.perf_counter() - self._wall0
        try:
            red = self.vehicle.is_at_traffic_light() and \
                self.vehicle.get_traffic_light_state() == carla.TrafficLightState.Red
        except RuntimeError:
            red = False
        return {**scene_tel, "t": t, "frame": self.frame, "n_frames": self.n_frames, "at_red_light": red,
                "rt_factor": t / wall if wall > 0 else 0.0, "speed_kmh": speed * 3.6,
                "location": [tf.location.x, tf.location.y, tf.location.z],
                "rotation": [tf.rotation.pitch, tf.rotation.yaw, tf.rotation.roll],
                "wheel_steer": steer + [0.0, 0.0], "wheel_rotation": [], "wheel_suspension_mm": [],
                "action": [c.throttle, c.brake, c.steer], "world_frame": world_frame,
                "dynamics": "CARLA", "done": self.done}

    def carsim_state(self):
        return {}
