"""One co-simulation run, stepped frame by frame.

Shared by run_cosim.py (CLI loop) and backend_server.py (GUI backend), so
both run exactly the same code path.
"""

import csv
import importlib.util
import math
import os
import sys
import time

import carla

import settings as st
from bridge import CarlaVehicleSync
from drivers import ManualDriver, RouteFollower


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
    sys.path.insert(0, os.path.abspath(c["repo_path"]))
    from carsim_env import CarSimEnv
    return CarSimEnv(c["sim_path"])


def make_driver(d, ex):
    drv = d["run"]["driver"]
    if drv == "demo":
        return lambda obs, t: demo_driver(t)
    if drv == "custom":
        return load_controller(d, ex)
    raise ValueError("unknown CarSim driver '%s'" % drv)


def load_controller(d, ex):
    """The user's control algorithm, loaded fresh from its file at every run
    start (edits apply on the next run). The entry is a class (instantiated,
    reset() called if present, then control() every frame) or a function:

        control(exports, t, dt) -> values for the CarSim imports, .sim order

    exports is {name: value} of every CarSim export, in CarSim units.
    See controllers/example_controller.py.
    """
    c = d["run"]["controller"]
    path = os.path.abspath(c["path"])
    if not os.path.isfile(path):
        raise ValueError("控制算法文件不存在：%s" % path)
    # Let the algorithm import its neighbours and python_carsim_env modules.
    for p in (os.path.abspath(d["carsim"]["repo_path"]), os.path.dirname(path)):
        if p not in sys.path:
            sys.path.insert(0, p)
    spec = importlib.util.spec_from_file_location("user_controller", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    entry = c.get("entry") or "Controller"
    obj = getattr(mod, entry, None)
    if obj is None:
        raise ValueError("%s 里没有找到 %s" % (os.path.basename(path), entry))
    if isinstance(obj, type):
        obj = obj()
        if hasattr(obj, "reset"):
            obj.reset()
        obj = obj.control
    dt = d["sync"]["frame_dt"]
    names = list(ex.index)
    return lambda obs, t: [float(v) for v in obj({n: ex.raw(obs, n) for n in names}, t, dt)]


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


class CoSimSession:
    """Lock-step CarSim + CARLA run for one already-spawned vehicle."""

    def __init__(self, world, vehicle, anchor, d):
        self.world, self.vehicle, self.anchor, self.d = world, vehicle, anchor, d
        self.env = self.sync = self.driver = self.camera = None
        self._log_file = self._log = None
        self._original_settings = None
        self.frame = 0
        self.done = False

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
            self.driver = make_driver(d, self.sync.ex)
        self.env = make_env(d)
        if d["run"]["record_dir"]:
            self.camera = spawn_chase_camera(w, self.vehicle, d["run"]["record_dir"])

        self.obs = self.env.reset()
        t_step = self.env.config["t_step"]
        self.inner = max(1, int(round(frame_dt / t_step)))
        self.clock_warning = abs(self.inner * t_step - frame_dt) > 1e-9
        # duration <= 0: run until stopped (or until CarSim reaches t_stop).
        self.n_frames = int(round(d["sync"]["duration"] / frame_dt)) if d["sync"]["duration"] > 0 else 0

        if d["run"]["log_path"]:
            self._log_file = open(d["run"]["log_path"], "w", newline="")
            self._log = csv.writer(self._log_file)
            self._log.writerow(["t", "carsim_x", "carsim_y", "carsim_yaw", "cmd_x", "cmd_y", "cmd_yaw",
                                "carla_x", "carla_y", "carla_yaw", "steer_fl", "steer_fr", "speed_carla"])
        self._wall0 = time.perf_counter()
        return {"external_api": self.sync.external_api,
                "reference_point": [round(float(x), 3) for x in self.sync.ref_local],
                "t_step": t_step, "inner_steps": self.inner, "clock_warning": self.clock_warning}

    def stop(self, release_vehicle=True):
        if self.env is not None:
            self.env.close()
        if self.camera is not None:
            self.camera.stop()
            self.camera.destroy()
            self.camera = None
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None
        if release_vehicle and self.sync is not None:
            self.sync.release()
        if self._original_settings is not None:
            self.world.apply_settings(self._original_settings)
            self._original_settings = None

    # -------------------------------------------------------------------- step
    def step(self):
        """Advance one CARLA frame. Returns a telemetry dict."""
        env, frame_dt = self.env, self.d["sync"]["frame_dt"]
        action = self.driver(self.obs, env.t_current)
        self.obs, _, done, info = env.control_step(action, self.inner)
        if info.get("error"):
            raise RuntimeError("CarSim error: %s" % info["error"])
        state = self.sync.sync(self.obs, env.t_current, frame_dt)
        world_frame = self.world.tick()
        self.frame += 1
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
        self.done = bool(done) or (self.n_frames > 0 and self.frame >= self.n_frames)
        wall = time.perf_counter() - self._wall0
        v = state.velocity
        return {
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

    def start(self):
        d, w = self.d, self.world
        dt = d["sync"]["frame_dt"]
        self._original_settings = w.get_settings()
        s = w.get_settings()
        s.synchronous_mode, s.fixed_delta_seconds = True, dt
        w.apply_settings(s)
        self.tm.set_synchronous_mode(True)
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
        self.n_frames = int(round(d["sync"]["duration"] / dt)) if d["sync"]["duration"] > 0 else 0
        self._wall0 = time.perf_counter()
        return {"external_api": False, "reference_point": [0, 0, 0], "t_step": dt, "inner_steps": 1,
                "clock_warning": False, "dynamics": "CARLA"}

    def stop(self, release_vehicle=True):
        try:
            if self.mode == "autopilot":
                self.vehicle.set_autopilot(False, self.tm.get_port())
        except RuntimeError:
            pass
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
        t = self.frame * dt
        tf = self.vehicle.get_transform()
        c = self.vehicle.get_control()
        steer = []
        for wl in (carla.VehicleWheelLocation.FL_Wheel, carla.VehicleWheelLocation.FR_Wheel):
            try:
                steer.append(self.vehicle.get_wheel_steer_angle(wl))
            except RuntimeError:
                steer.append(0.0)
        self.done = self.n_frames > 0 and self.frame >= self.n_frames
        wall = time.perf_counter() - self._wall0
        try:
            red = self.vehicle.is_at_traffic_light() and \
                self.vehicle.get_traffic_light_state() == carla.TrafficLightState.Red
        except RuntimeError:
            red = False
        return {"t": t, "frame": self.frame, "n_frames": self.n_frames, "at_red_light": red,
                "rt_factor": t / wall if wall > 0 else 0.0, "speed_kmh": speed * 3.6,
                "location": [tf.location.x, tf.location.y, tf.location.z],
                "rotation": [tf.rotation.pitch, tf.rotation.yaw, tf.rotation.roll],
                "wheel_steer": steer + [0.0, 0.0], "wheel_rotation": [], "wheel_suspension_mm": [],
                "action": [c.throttle, c.brake, c.steer], "world_frame": world_frame,
                "dynamics": "CARLA", "done": self.done}

    def carsim_state(self):
        return {}
