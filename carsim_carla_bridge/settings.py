"""Co-simulation settings: config.py defaults, optionally overridden by JSON.

The JSON file is what the C++ GUI edits; the same file drives run_cosim.py
(--config) and the RL training code, so every entry point sees one config.

JSON layout (all keys optional, missing ones fall back to config.py):
{
  "carla":  {"host": "localhost", "port": 2000,
             "vehicle": "vehicle.tesla.model3", "spawn_index": 0},
  "carsim": {"sim_path": "", "repo_path": "../python_carsim_env", "mock": false, "remote": false,
             "export_names": [...], "units": {"angle": "deg", ...}},
  "sync":   {"frame_dt": 0.02, "duration": 0.0, "reference_point": "front_axle",
             "z_mode": "carsim", "wheel_spin_sign": -1.0,
             "steering_wheel_max_deg": 540.0, "use_external_api": "auto"},
  "run":    {"driver": "custom", "log_path": "runs",
             "controller": {"path": "controllers/example_controller.py", "entry": "Controller"}},
  "drive":  {"dynamics": "cosim", "carla_driver": "route", "target_speed_kmh": 40.0, ...},
  "rig":    {"preset": "front_camera", "sensors": [...]},
  "collect":{"enabled": false, "out_dir": "datasets", "max_frames": 100, "max_gb": 2.0, ...},
  "scene":  {"collision": "log", "objects": ["id", "type", "rel_x", ...], "lane": [...], ...}
}
"""

import copy
import json
from types import SimpleNamespace

import config as _defaults


def default_dict():
    return {
        # host / port: the CARLA server run_cosim.py connects to (the GUI
        # writes the one it is connected to when it saves).
        "carla": {"host": "localhost", "port": 2000,
                  "vehicle": "vehicle.tesla.model3", "spawn_index": 0},
        # remote: CarSim runs on the user's Windows computer (carsim_service.py), and
        # sim_path / repo_path are paths there; mock goes first. chrono: the PyChrono BMW
        # E90 on this machine instead (chrono_local.py; imports [ax m/s^2, front wheel
        # angle rad], starting at chrono_init_speed m/s; chrono_python "" = found).
        "carsim": {"sim_path": "", "repo_path": "../python_carsim_env", "mock": False, "remote": False,
                   "chrono": False, "chrono_init_speed": 20.0, "chrono_python": "",
                   "mock_init_speed": 0.0,  # m/s: the mock's speed at t = 0 (a .sim's initial speed)
                   "export_names": list(_defaults.EXPORT_NAMES),
                   "units": dict(_defaults.UNITS)},
        # duration 0 = run until stopped (or until CarSim reaches t_stop).
        "sync": {"frame_dt": 0.02, "duration": 0.0,
                 "reference_point": _defaults.CARSIM_REFERENCE_POINT,
                 "z_mode": _defaults.Z_MODE,
                 "wheel_spin_sign": _defaults.WHEEL_SPIN_SIGN,
                 "steering_wheel_max_deg": _defaults.STEERING_WHEEL_MAX_DEG,
                 "use_external_api": "auto"},
        # driver: custom = the user's control algorithm (controller.path,
        # relative to carsim_carla_bridge/) | demo. Both drive CarSim.
        # log_path: the run record directory, every run a folder in it ("" = no
        # record; an older config's "cosim_log.csv": its folder, CSV names cosim_log*).
        # params: the 算法参数 changed on the 驾驶模式 page, by algorithm file: {path: {NAME: value}}
        # (set on the loaded module before the algorithm object is made; session.apply_params).
        "run": {"driver": "custom", "log_path": "runs",
                "controller": {"path": "controllers/example_controller.py", "entry": "Controller"}, "params": {},
                # 干扰 (disturb.py): delays, noise on exports, lane dropouts for the user's algorithm.
                "disturb": {"enabled": False, "act_delay": 0.0, "sense_delay": 0.0, "noise": {},
                            "lane_dropout": 0.0, "seed": 0}},
        # dynamics: "cosim" = CarSim drives the car, "carla" = CARLA PhysX.
        # drive.cosim_driver (GUI) = custom | demo | route | manual; carla_driver = route | autopilot | manual
        "drive": {"dynamics": "cosim", "carla_driver": "route", "target_speed_kmh": 40.0,
                  "destination_index": -1, "brake_scale": 1.0,
                  "tm_speed_diff_pct": 0.0, "tm_ignore_lights": False},
        # 测试场景 (scenario.py): lane closures with cones / barriers, placed at a run's
        # start along the road from the spawn point: [{"distance_m", "lane" (0 = the start
        # lane, -1 / 1 the first to the left / right), "taper_m", "length_m", "kind"
        # ("cones" | "barrier")}].
        # actors: 动态目标 [{"type": "slow_car" | "lead_brake" | "cut_in" | "pedestrian", "distance_m",
        # "lane", "speed_kmh", "trigger_m", "param" (lead_brake: m/s^2, cut_in: s)}] (scenario.py).
        "scenario": {"enabled": False, "closures": [], "actors": []},
        # world: with fixed on, every run first rebuilds the CARLA world the config names: the map
        # (loaded if another one is up), the weather (CARLA weather parameters) and the traffic
        # (cleared, then vehicles / walkers spawned with seed): the same surroundings every time.
        "world": {"fixed": False, "map": "", "weather": {}, "traffic": {"vehicles": 0, "walkers": 0, "seed": 0}},
        # 通过标准 (criteria.py): the verdict of every run (run.json "verdict", the batch report).
        "criteria": {"finished": True, "no_collision": True, "on_lane": True,
                     "limits": {"lane_offset_rms": None, "lane_offset_max": None, "ttc_min": None, "min_gap_ahead": None,
                                "accel_max": None, "decel_max": None, "jerk_max": None}},
        # 车辆参数辨识 (运行对比 page, vehicle_ident.py): the car's mass (kg), yaw inertia (kg m^2),
        # CG to front / rear axle (m); path = where the KMPPI vehicle file is written.
        "ident": {"m": 1910.0, "I": 3482.0, "a": 1.371, "b": 1.386, "path": "controllers/kmppi/vehicle_identified.json"},
        # Sensor mounts in CarSim's vehicle frame (origin = the reference
        # point, y left; rig.py). "carla" = an older config (car centre, y right).
        "rig": {"preset": "front_camera", "sensors": [], "frame": "carsim"},
        # Conservative defaults: a run without an explicit limit never starts.
        "collect": {"enabled": False, "out_dir": "datasets", "session": "", "image_format": "jpg",
                    "jpg_quality": 90, "pointcloud_format": "bin", "capture_every": 1, "labels": True,
                    "label_radius": 80.0, "max_frames": 100, "max_seconds": 0.0, "max_gb": 2.0,
                    # Sampling period of collection and run records, s (a multiple of
                    # frame_dt); 0 = every capture_every frames (older configs).
                    "sample_period": 0.0},
        # What the control algorithm gets (ego / objects / lane / sensors) and
        # what the records keep (record, exports_all / exports): the selected
        # keys only (scene.py, docs/场景与数据接口.md). collision: "log" = report
        # and go on, "stop" = end the run, "off" = no check. sensors: rig
        # sensor names handed to the algorithm; the algorithm always gets all
        # CarSim exports.
        "scene": {"collision": "log", "object_types": ["vehicle", "walker", "parked", "static"],
                  "ego": ["X", "Y", "Z", "Yaw", "Vx_global", "Vy_global", "Speed", "length", "width", "height"],
                  "objects": ["id", "type", "rel_x", "rel_y", "rel_vx", "rel_vy", "dist", "gap"],
                  "lane": ["width", "offset", "heading_err", "center_rel"],
                  "sensors": [],
                  "record": {"ego": ["X", "Y", "Z", "Yaw", "Vx_global", "Vy_global", "Speed", "length", "width", "height"],
                             "objects": ["id", "type", "rel_x", "rel_y", "rel_vx", "rel_vy", "dist", "gap"],
                             "lane": ["width", "offset", "heading_err", "center_rel"]},
                  "exports_all": True, "exports": []},
    }


def _merge(base, override):
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _merge(base[k], v)
        else:
            base[k] = v
    return base


def load_dict(path=None, override=None):
    d = default_dict()
    if path:
        with open(path, encoding="utf-8") as f:
            raw = json.load(f)
        rig = raw.get("rig") if isinstance(raw, dict) else None
        if isinstance(rig, dict) and rig.get("sensors") and "frame" not in rig:
            rig["frame"] = "carla"  # written before mounts were in CarSim's frame
        _merge(d, raw)
    if override:
        _merge(d, copy.deepcopy(override))
    return d


def sample_every(d):
    """Run steps between two samples of the records / data collection
    (counted from step 0, the run's start)."""
    c, dt = d["collect"], float(d["sync"]["frame_dt"])
    period = float(c.get("sample_period", 0.0) or 0.0)
    if period > 0 and dt > 0:
        return max(1, int(round(period / dt)))
    return max(1, int(c.get("capture_every", 1) or 1))


def to_bridge_cfg(d):
    """Namespace with the attribute names bridge.py reads (config.py style)."""
    s = d["sync"]
    ref = s["reference_point"]
    return SimpleNamespace(
        EXPORT_NAMES=list(d["carsim"]["export_names"]),
        UNITS=dict(d["carsim"]["units"]),
        CARSIM_REFERENCE_POINT=ref if ref == "front_axle" else [float(x) for x in ref],
        Z_MODE=s["z_mode"],
        WHEEL_SPIN_SIGN=float(s["wheel_spin_sign"]),
        STEERING_WHEEL_MAX_DEG=float(s["steering_wheel_max_deg"]),
    )


def save_dict(d, path):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(d, f, ensure_ascii=False, indent=2)
