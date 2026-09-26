"""Sensor rigs: presets fitted to the vehicle, plus per-sensor data estimates.

A rig is a list of sensors, each with a mount pose in the CARLA vehicle frame
(origin = actor origin at ground level under the car centre; x forward,
y right, z up; meters and degrees) and blueprint attributes.

Presets are written against the vehicle's bounding box so they land in a
sensible place on any car: "roof" = top of the box, "front" = front bumper.
"""

import math

SENSOR_BLUEPRINTS = {
    "rgb": "sensor.camera.rgb",
    "depth": "sensor.camera.depth",
    "semantic": "sensor.camera.semantic_segmentation",
    "instance": "sensor.camera.instance_segmentation",
    "lidar": "sensor.lidar.ray_cast",
    "radar": "sensor.other.radar",
    "imu": "sensor.other.imu",
    "gnss": "sensor.other.gnss",
    "collision": "sensor.other.collision",
    "lane_invasion": "sensor.other.lane_invasion",
}
CAMERA_TYPES = ("rgb", "depth", "semantic", "instance")


def default_attrs(kind):
    if kind in CAMERA_TYPES:
        return {"image_size_x": 1600, "image_size_y": 900, "fov": 70.0}
    if kind == "lidar":
        return {"channels": 32, "range": 100.0, "points_per_second": 600000, "rotation_frequency": 10.0,
                "upper_fov": 10.0, "lower_fov": -30.0}
    if kind == "radar":
        return {"horizontal_fov": 30.0, "vertical_fov": 10.0, "range": 100.0, "points_per_second": 1500}
    return {}


def sensor(name, kind, x, y, z, yaw=0.0, pitch=0.0, roll=0.0, **attrs):
    a = default_attrs(kind)
    a.update(attrs)
    return {"name": name, "type": kind, "x": round(x, 3), "y": round(y, 3), "z": round(z, 3),
            "roll": roll, "pitch": pitch, "yaw": yaw, "attributes": a, "enabled": True}


def _dims(spec):
    spec = spec or {}
    L = spec.get("length_m", 4.8)
    W = spec.get("width_m", 2.0)
    H = spec.get("height_m", 1.5)
    return L, W, H


PRESETS = {
    "front_camera": "单前视相机",
    "kitti": "KITTI 风格（双目 + 64 线激光雷达 + GNSS/IMU）",
    "nuscenes": "nuScenes 风格（6 环视 + 32 线激光雷达 + 5 毫米波雷达）",
    "production": "量产车风格（8 相机 + 前向毫米波雷达）",
    "perception_gt": "感知真值（RGB + 深度 + 语义 + 实例，同一位置）",
}


def build_preset(name, spec=None):
    """Resolve a preset for a vehicle with the given spec (from vehicle_specs)."""
    L, W, H = _dims(spec)
    roof = H + 0.05            # just above the roof
    front = L / 2.0            # front bumper
    rear = -L / 2.0
    wind_x = L * 0.15          # windshield top, behind the hood
    wind_z = H * 0.92
    s = []
    if name == "front_camera":
        s.append(sensor("cam_front", "rgb", wind_x, 0.0, wind_z, image_size_x=1280, image_size_y=720, fov=90.0))
    elif name == "kitti":
        # KITTI: stereo pair 0.54 m apart on the roof, HDL-64 above it.
        s.append(sensor("cam_left", "rgb", 0.27, -0.27, roof, image_size_x=1242, image_size_y=375, fov=90.0))
        s.append(sensor("cam_right", "rgb", 0.27, 0.27, roof, image_size_x=1242, image_size_y=375, fov=90.0))
        s.append(sensor("lidar_top", "lidar", 0.0, 0.0, roof + 0.25, channels=64, range=120.0,
                        points_per_second=1300000, rotation_frequency=10.0, upper_fov=2.0, lower_fov=-24.8))
        s.append(sensor("gnss", "gnss", 0.0, 0.0, roof))
        s.append(sensor("imu", "imu", 0.0, 0.0, H * 0.5))
    elif name == "nuscenes":
        z = roof
        cams = [("cam_front", 0, 70), ("cam_front_left", -55, 70), ("cam_front_right", 55, 70),
                ("cam_back", 180, 110), ("cam_back_left", -110, 70), ("cam_back_right", 110, 70)]
        for n, yaw, fov in cams:
            r = 0.35
            s.append(sensor(n, "rgb", 0.4 + r * math.cos(math.radians(yaw)), r * math.sin(math.radians(yaw)), z,
                            yaw=float(yaw), image_size_x=1600, image_size_y=900, fov=float(fov)))
        s.append(sensor("lidar_top", "lidar", 0.0, 0.0, roof + 0.3, channels=32, range=70.0,
                        points_per_second=700000, rotation_frequency=10.0, upper_fov=10.0, lower_fov=-30.0))
        radars = [("radar_front", front, 0.0, 0), ("radar_front_left", front - 0.3, -W / 2, -90 + 10),
                  ("radar_front_right", front - 0.3, W / 2, 90 - 10), ("radar_back_left", rear + 0.3, -W / 2, -150),
                  ("radar_back_right", rear + 0.3, W / 2, 150)]
        for n, x, y, yaw in radars:
            s.append(sensor(n, "radar", x, y, 0.5, yaw=float(yaw), horizontal_fov=60.0, range=250.0))
    elif name == "production":
        s.append(sensor("cam_front_main", "rgb", wind_x, 0.0, wind_z, image_size_x=1920, image_size_y=1080, fov=50.0))
        s.append(sensor("cam_front_wide", "rgb", wind_x, 0.1, wind_z, image_size_x=1920, image_size_y=1080, fov=120.0))
        s.append(sensor("cam_front_narrow", "rgb", wind_x, -0.1, wind_z, image_size_x=1920, image_size_y=1080, fov=30.0))
        for n, x, y, yaw in [("cam_left_fwd", L * 0.1, -W / 2, -60), ("cam_right_fwd", L * 0.1, W / 2, 60),
                             ("cam_left_rear", L * 0.25, -W / 2, -135), ("cam_right_rear", L * 0.25, W / 2, 135)]:
            s.append(sensor(n, "rgb", x, y, H * 0.75, yaw=float(yaw), image_size_x=1280, image_size_y=960, fov=90.0))
        s.append(sensor("cam_rear", "rgb", rear, 0.0, H * 0.7, yaw=180.0, pitch=-10.0,
                        image_size_x=1280, image_size_y=960, fov=120.0))
        s.append(sensor("radar_front", "radar", front, 0.0, 0.5, horizontal_fov=45.0, range=200.0))
    elif name == "perception_gt":
        for kind in ("rgb", "depth", "semantic", "instance"):
            s.append(sensor("cam_" + kind, kind, wind_x, 0.0, wind_z, image_size_x=1280, image_size_y=720, fov=90.0))
    else:
        raise ValueError("unknown rig preset %s" % name)
    return s


# ---------------------------------------------------------------- estimates
# Bytes per frame for planning disk space. Ratios measured from a 3-frame
# 640x360 capture in Town10HD (tests/test_features.py): RGB JPEG q90 0.108,
# depth PNG 0.18, semantic 0.016, instance 0.022 (bytes per pixel channel);
# ~50 % of lidar rays return a point in urban scenes. The collector reports
# the real rate while running.
BYTES_PER_PX3 = {"rgb_jpg": 0.108, "rgb_png": 0.45, "depth": 0.18, "semantic": 0.016, "instance": 0.022}
LIDAR_HIT_RATIO = 0.5


def bytes_per_frame(sensor_cfg, image_format="jpg", frame_dt=None):
    """Bytes one captured frame of this sensor writes. With frame_dt: lidar and
    radar as collected (one sweep per frame: points_per_second x frame_dt)."""
    k, a = sensor_cfg["type"], sensor_cfg.get("attributes", {})
    if k in CAMERA_TYPES:
        px3 = int(a.get("image_size_x", 800)) * int(a.get("image_size_y", 600)) * 3
        key = ("rgb_jpg" if image_format == "jpg" else "rgb_png") if k == "rgb" else k
        return px3 * BYTES_PER_PX3[key]
    if k == "lidar":
        pps = float(a.get("points_per_second", 56000))
        pts = pps * frame_dt if frame_dt else pps / max(1.0, float(a.get("rotation_frequency", 10.0)))
        return pts * LIDAR_HIT_RATIO * 16  # float32 x, y, z, intensity
    if k == "radar":
        return float(a.get("points_per_second", 1500)) * (frame_dt or 0.1) * 32  # csv rows
    return 200                             # imu / gnss rows in the ego json
