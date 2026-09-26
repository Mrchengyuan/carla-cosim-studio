"""Co-simulation settings: config.py defaults, optionally overridden by JSON.

The JSON file is what the C++ GUI edits; the same file drives run_cosim.py
(--config) and the RL training code, so every entry point sees one config.

JSON layout (all keys optional, missing ones fall back to config.py):
{
  "carla":  {"host": "localhost", "port": 2000, "map": "", "weather": "",
             "vehicle": "vehicle.tesla.model3", "spawn_index": 0},
  "carsim": {"sim_path": "", "repo_path": "../python_carsim_env", "mock": false,
             "export_names": [...], "units": {"angle": "deg", ...}},
  "sync":   {"frame_dt": 0.02, "duration": 0.0, "reference_point": "front_axle",
             "z_mode": "carsim", "wheel_spin_sign": -1.0,
             "steering_wheel_max_deg": 540.0, "use_external_api": "auto"},
  "run":    {"driver": "custom", "record_dir": "", "log_path": "cosim_log.csv",
             "controller": {"path": "controllers/example_controller.py", "entry": "Controller"}},
  "drive":  {"dynamics": "cosim", "carla_driver": "route", "target_speed_kmh": 40.0, ...},
  "rig":    {"preset": "front_camera", "sensors": [...]},
  "collect":{"enabled": false, "out_dir": "datasets", "max_frames": 100, "max_gb": 2.0, ...},
  "scene":  {"objects": true, "lane": true, "sensors": false, "range_m": 80.0,
             "collision": "log", ...}
}
"""

import copy
import json
from types import SimpleNamespace

import config as _defaults


def default_dict():
    return {
        "carla": {"host": "localhost", "port": 2000, "map": "", "weather": "",
                  "vehicle": "vehicle.tesla.model3", "spawn_index": 0},
        "carsim": {"sim_path": "", "repo_path": "../python_carsim_env", "mock": False,
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
        "run": {"driver": "custom", "record_dir": "", "log_path": "cosim_log.csv",
                "controller": {"path": "controllers/example_controller.py", "entry": "Controller"}},
        # dynamics: "cosim" = CarSim drives the car, "carla" = CARLA PhysX.
        # drive.cosim_driver (GUI) = custom | demo | route | manual; carla_driver = route | autopilot | manual
        "drive": {"dynamics": "cosim", "carla_driver": "route", "target_speed_kmh": 40.0,
                  "destination_index": -1, "brake_scale": 1.0,
                  "tm_speed_diff_pct": 0.0, "tm_ignore_lights": False},
        "rig": {"preset": "front_camera", "sensors": []},
        # Conservative defaults: a run without an explicit limit never starts.
        "collect": {"enabled": False, "out_dir": "datasets", "session": "", "image_format": "jpg",
                    "jpg_quality": 90, "pointcloud_format": "bin", "capture_every": 1, "labels": True,
                    "label_radius": 80.0, "max_frames": 100, "max_seconds": 0.0, "max_gb": 2.0},
        # What the control algorithm gets each frame (scene.py). collision:
        # "log" = report and go on, "stop" = end the run, "off" = ignore.
        "scene": {"objects": True, "map_objects": True, "lane": True, "sensors": False,
                  "range_m": 80.0, "lane_ahead_m": 60.0, "lane_step_m": 2.0, "collision": "log"},
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
            _merge(d, json.load(f))
    if override:
        _merge(d, copy.deepcopy(override))
    return d


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
