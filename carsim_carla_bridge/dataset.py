"""Browse and export datasets written by collector.py.

Coordinate frames
-----------------
CARLA (world, ego, sensor actors): x forward, y right, z up -- left-handed.
KITTI velodyne: x forward, y left, z up.  KITTI camera: x right, y down, z forward.
nuScenes global / ego / lidar / radar: right-handed, z up; camera: x right, y down, z forward.
F flips y (CARLA <-> right-handed); C maps CARLA camera axes to OpenCV camera axes.
"""

import glob
import hashlib
import json
import math
import os
import shutil
import struct
import threading

import numpy as np

F = np.diag([1.0, -1.0, 1.0])
C = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])

# CARLA 0.9.14+ semantic tags -> CityScapes colours (same as carla.ColorConverter.CityScapesPalette).
PALETTE = np.array([
    (0, 0, 0), (128, 64, 128), (244, 35, 232), (70, 70, 70), (102, 102, 156), (190, 153, 153), (153, 153, 153),
    (250, 170, 30), (220, 220, 0), (107, 142, 35), (152, 251, 152), (70, 130, 180), (220, 20, 60), (255, 0, 0),
    (0, 0, 142), (0, 0, 70), (0, 60, 100), (0, 80, 100), (0, 0, 230), (119, 11, 32), (110, 190, 160),
    (170, 120, 50), (55, 90, 80), (45, 60, 150), (157, 234, 50), (81, 0, 81), (150, 100, 100), (230, 150, 140),
    (180, 165, 180)] + [(255, 255, 255)] * 227, dtype=np.uint8)

CLASS_COLORS = {"car": (70, 160, 255), "van": (70, 160, 255), "truck": (255, 150, 40), "bus": (255, 150, 40),
                "motorcycle": (190, 110, 255), "bicycle": (190, 110, 255), "pedestrian": (255, 70, 70)}
KITTI_TYPES = {"car": "Car", "van": "Van", "truck": "Truck", "bus": "Truck", "motorcycle": "Cyclist",
               "bicycle": "Cyclist", "pedestrian": "Pedestrian"}
NUSC_CATEGORIES = {"car": "vehicle.car", "van": "vehicle.car", "truck": "vehicle.truck", "bus": "vehicle.bus.rigid",
                   "motorcycle": "vehicle.motorcycle", "bicycle": "vehicle.bicycle", "pedestrian": "human.pedestrian.adult"}


# --------------------------------------------------------------------------- helpers
def rot_matrix(roll, pitch, yaw):
    """Same matrix as carla.Transform.get_matrix() (UE / CARLA convention, degrees)."""
    cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    cr, sr = math.cos(math.radians(roll)), math.sin(math.radians(roll))
    cp, sp = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
    return np.array([[cp * cy, cy * sp * sr - sy * cr, -cy * sp * cr - sy * sr],
                     [cp * sy, sy * sp * sr + cy * cr, -sy * sp * cr + cy * sr],
                     [sp, -cp * sr, cp * cr]])


def tf_matrix(d):
    m = np.eye(4)
    m[:3, :3] = rot_matrix(d.get("roll", 0.0), d.get("pitch", 0.0), d.get("yaw", 0.0))
    m[:3, 3] = [d["x"], d["y"], d["z"]]
    return m


def quat(R):
    """Rotation matrix -> quaternion [w, x, y, z]."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    return (q / np.linalg.norm(q) * (1 if q[0] >= 0 else -1)).tolist()


def box_corners(obj):
    """8 corners of a label box in the CARLA ego frame, shape (8, 3)."""
    ex, ey, ez = obj["extent"]
    c = np.array(obj["center_ego"])
    yaw = math.radians(obj["yaw_ego"])
    local = np.array([[sx * ex, sy * ey, sz * ez] for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)])
    R = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    return local @ R.T + c


BOX_EDGES = [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5), (2, 6), (3, 7)]


def points_in_box(pts_ego, obj):
    """Boolean mask of ego-frame points inside a label box."""
    ex, ey, ez = obj["extent"]
    yaw = math.radians(obj["yaw_ego"])
    d = pts_ego[:, :3] - np.array(obj["center_ego"])
    x = d[:, 0] * math.cos(yaw) + d[:, 1] * math.sin(yaw)
    y = -d[:, 0] * math.sin(yaw) + d[:, 1] * math.cos(yaw)
    return (np.abs(x) <= ex) & (np.abs(y) <= ey) & (np.abs(d[:, 2]) <= ez)


def token(*parts):
    return hashlib.md5("/".join(str(p) for p in parts).encode()).hexdigest()


def dir_size(path):
    total = 0
    for root, _, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return total


# --------------------------------------------------------------------------- session
class Session:
    """A collector session folder."""

    def __init__(self, root):
        self.root = os.path.abspath(root)
        with open(os.path.join(self.root, "calib.json"), encoding="utf-8") as f:
            self.calib = json.load(f)
        with open(os.path.join(self.root, "meta.json"), encoding="utf-8") as f:
            self.meta = json.load(f)
        self.sensors = self.calib["sensors"]
        self.frames = sorted(int(os.path.basename(p)[:-5]) for p in glob.glob(os.path.join(glob.escape(self.root), "ego", "*.json")))
        self.extrinsic = {n: np.array(s["extrinsic_sensor_to_ego"]) for n, s in self.sensors.items()}

    def file(self, sensor, frame):
        hits = glob.glob(os.path.join(glob.escape(self.root), glob.escape(sensor), "%06d.*" % frame))
        return hits[0] if hits else None

    def ego(self, frame):
        with open(os.path.join(self.root, "ego", "%06d.json" % frame), encoding="utf-8") as f:
            return json.load(f)

    def labels(self, frame):
        p = os.path.join(self.root, "labels", "%06d.json" % frame)
        if not os.path.exists(p):
            return []
        with open(p, encoding="utf-8") as f:
            return json.load(f).get("objects", [])

    def lidar_points(self, sensor, frame):
        """Points of a lidar in its own frame (CARLA axes), shape (N, 4)."""
        p = self.file(sensor, frame)
        if p is None:
            return np.zeros((0, 4), np.float32)
        return np.load(p) if p.endswith(".npy") else np.fromfile(p, dtype=np.float32).reshape(-1, 4)

    def radar_points(self, sensor, frame):
        """Radar detections as (x, y, z, velocity) in the radar frame (CARLA axes)."""
        p = self.file(sensor, frame)
        if p is None or os.path.getsize(p) == 0:
            return np.zeros((0, 4), np.float32)
        d = np.loadtxt(p, delimiter=",", ndmin=2)
        vel, az, alt, dep = d[:, 0], d[:, 1], d[:, 2], d[:, 3]
        return np.stack([dep * np.cos(alt) * np.cos(az), dep * np.cos(alt) * np.sin(az), dep * np.sin(alt), vel], 1)

    def to_ego(self, sensor, pts):
        m = self.extrinsic[sensor]
        return pts[:, :3] @ m[:3, :3].T + m[:3, 3]

    def first_of(self, kind):
        for n, s in self.sensors.items():
            if s["type"] == kind:
                return n
        return None

    def summary(self):
        return {"root": self.root, "name": os.path.basename(self.root), "frames": len(self.frames),
                "first": self.frames[0] if self.frames else None, "last": self.frames[-1] if self.frames else None,
                "frame_list": self.frames,
                "map": self.meta.get("map", ""), "started": self.meta.get("started", ""),
                "sensors": [{"name": n, "type": s["type"]} for n, s in self.sensors.items()],
                "has_labels": os.path.isdir(os.path.join(self.root, "labels"))}


def list_sessions(out_dir):
    out = []
    for p in sorted(glob.glob(os.path.join(glob.escape(os.path.abspath(out_dir or ".")), "*", "calib.json"))):
        root = os.path.dirname(p)
        try:
            s = Session(root).summary()
            s.pop("frame_list", None)
        except Exception as e:  # half-written or foreign folder
            s = {"root": root, "name": os.path.basename(root), "error": str(e), "frames": 0, "sensors": []}
        s["size_mb"] = dir_size(root) / 1e6
        out.append(s)
    return out


# --------------------------------------------------------------------------- rendering (browser)
def _lidar_counts(ses, frame, objs):
    lid = ses.first_of("lidar")
    if lid is None:
        return None
    pts = ses.to_ego(lid, ses.lidar_points(lid, frame))
    return [int(points_in_box(pts, o).sum()) for o in objs]


def render_frame(ses, frame, sensor, max_w=960, boxes=True):
    """Image of one sensor at one frame, with the ground-truth boxes drawn on it."""
    from PIL import Image, ImageDraw
    kind = ses.sensors[sensor]["type"]
    objs = ses.labels(frame) if boxes else []
    if kind in ("rgb", "semantic", "depth", "instance"):
        path = ses.file(sensor, frame)
        if path is None:
            raise ValueError("%s 第 %d 帧没有数据" % (sensor, frame))
        img = np.array(Image.open(path).convert("RGB"))
        if kind == "semantic":
            img = PALETTE[img[:, :, 0]]
        elif kind == "depth":
            d = (img[:, :, 0] + img[:, :, 1] * 256.0 + img[:, :, 2] * 65536.0) / (256 ** 3 - 1) * 1000.0
            v = np.clip(np.log(np.maximum(d, 0.1)) / math.log(1000.0), 0, 1)
            img = np.repeat((v * 255).astype(np.uint8)[:, :, None], 3, 2)
        elif kind == "instance":
            oid = img[:, :, 1].astype(np.int64) + img[:, :, 2].astype(np.int64) * 256
            h = (oid * 2654435761) & 0xFFFFFF
            img = np.stack([(h >> 16) & 255, (h >> 8) & 255, h & 255], 2).astype(np.uint8)
            img[oid == 0] = 0
        H, W = img.shape[:2]
        scale = min(1.0, max_w / float(W))
        im = Image.fromarray(img)
        if scale < 1.0:
            im = im.resize((int(W * scale), int(H * scale)), Image.BILINEAR)
        draw = ImageDraw.Draw(im)
        K = np.array(ses.sensors[sensor]["K"]) * np.array([[scale], [scale], [1.0]])
        inv = np.linalg.inv(ses.extrinsic[sensor])
        for o in objs:
            cam = (box_corners(o) @ inv[:3, :3].T + inv[:3, 3]) @ C.T   # OpenCV camera frame
            if (cam[:, 2] <= 0.3).any():
                continue
            uv = cam @ K.T
            uv = uv[:, :2] / uv[:, 2:3]
            if uv[:, 0].max() < 0 or uv[:, 0].min() > im.width or uv[:, 1].max() < 0 or uv[:, 1].min() > im.height:
                continue
            col = CLASS_COLORS.get(o["class"], (255, 255, 255))
            for a, b in BOX_EDGES:
                draw.line([tuple(uv[a]), tuple(uv[b])], fill=col, width=2)
            draw.text((float(uv[:, 0].min()), float(uv[:, 1].min()) - 12), "%s %d" % (o["class"], o["id"]), fill=col)
        out = np.array(im)
    else:
        size = int(min(max_w, 720))
        rng = float(ses.sensors[sensor].get("attributes", {}).get("range", 60.0))
        rng = min(rng, 80.0)
        img = np.full((size, size, 3), (16, 18, 22), dtype=np.uint8)
        yy, xx = np.mgrid[0:size, 0:size]
        r = np.hypot(xx - size / 2, yy - size / 2) / (size / 2) * rng
        for ring in np.arange(10.0, rng + 0.1, 10.0):
            img[np.abs(r - ring) < rng / size * 0.8] = (48, 54, 62)

        def uvs(x, y):
            return size / 2 + y / rng * size / 2, size / 2 - x / rng * size / 2

        if kind == "lidar":
            pts = ses.lidar_points(sensor, frame)
            ego_pts = ses.to_ego(sensor, pts)
            u, v = uvs(ego_pts[:, 0], ego_pts[:, 1])
            z = ego_pts[:, 2]
            col = np.stack([np.interp(z, [-0.5, 1, 3], [60, 60, 255]), np.interp(z, [-0.5, 1, 3], [120, 230, 90]),
                            np.interp(z, [-0.5, 1, 3], [255, 90, 60])], 1).astype(np.uint8)
            m = (u >= 0) & (u < size) & (v >= 0) & (v < size)
            img[v[m].astype(int), u[m].astype(int)] = col[m]
        else:
            det = ses.radar_points(sensor, frame)
            ego_pts = ses.to_ego(sensor, det) if len(det) else np.zeros((0, 3))
            u, v = uvs(ego_pts[:, 0], ego_pts[:, 1])
            for du in (-1, 0, 1):
                for dv in (-1, 0, 1):
                    uu, vv = (u + du).astype(int), (v + dv).astype(int)
                    m = (uu >= 0) & (uu < size) & (vv >= 0) & (vv < size)
                    vel = det[:, 3] if len(det) else np.zeros(0)
                    cc = np.where(vel[:, None] < -0.5, [255, 70, 60], np.where(vel[:, None] > 0.5, [70, 140, 255], [235, 235, 235]))
                    img[vv[m], uu[m]] = cc[m]
        im = Image.fromarray(img)
        draw = ImageDraw.Draw(im)
        ego_box = [uvs(x, y) for x, y in ((2.4, 1.0), (2.4, -1.0), (-2.4, -1.0), (-2.4, 1.0))]
        draw.polygon(ego_box, outline=(230, 232, 236))
        for o in objs:
            c = box_corners(o)[[0, 2, 6, 4]]  # top face, going round
            poly = [uvs(p[0], p[1]) for p in c]
            col = CLASS_COLORS.get(o["class"], (255, 255, 255))
            draw.polygon(poly, outline=col)
            fx, fy = uvs(*((c[0][:2] + c[1][:2]) / 2))  # front edge marker
            cx, cy = uvs(o["center_ego"][0], o["center_ego"][1])
            draw.line([(cx, cy), (fx, fy)], fill=col)
        out = np.array(im)
    counts = _lidar_counts(ses, frame, objs) if objs else None
    ego = ses.ego(frame)
    v = ego.get("velocity", [0, 0, 0])
    objects = []
    for i, o in enumerate(objs):
        objects.append({"id": o["id"], "class": o["class"], "type_id": o.get("type_id", ""),
                        "distance": float(np.linalg.norm(o["center_ego"][:2])),
                        "lidar_pts": counts[i] if counts else None})
    objects.sort(key=lambda x: x["distance"])
    return out, {"objects": objects, "speed_kmh": 3.6 * math.hypot(v[0], v[1]), "timestamp": ego.get("timestamp", 0.0)}


# --------------------------------------------------------------------------- export
def _copy_image(src, dst_noext, want_ext):
    """Copy an image, converting only when the format has to change."""
    from PIL import Image
    ext = os.path.splitext(src)[1].lower()
    if ext == want_ext:
        shutil.copyfile(src, dst_noext + ext)
        return dst_noext + ext
    Image.open(src).convert("RGB").save(dst_noext + want_ext, quality=95)
    return dst_noext + want_ext


def estimate_export(ses, fmt, camera=None):
    """Rough output size (bytes): KITTI stores PNG images, which is ~4x a JPG."""
    total = 0
    camera = camera or ses.first_of("rgb")
    for f in ses.frames[:3] or []:
        for n, s in ses.sensors.items():
            p = ses.file(n, f)
            if p is None:
                continue
            if fmt == "kitti" and s["type"] == "rgb" and n != camera:
                continue
            if fmt == "kitti" and s["type"] not in ("rgb", "lidar"):
                continue
            if fmt == "nuscenes" and s["type"] not in ("rgb", "lidar", "radar"):
                continue
            sz = os.path.getsize(p)
            if fmt == "kitti" and p.lower().endswith(".jpg"):
                sz *= 4
            if s["type"] == "lidar" and fmt == "nuscenes":
                sz = sz * 5 // 4
            total += sz
    n = max(1, min(3, len(ses.frames)))
    return total / n * len(ses.frames)


def export_kitti(ses, out, camera=None, lidar=None, min_lidar_pts=1, progress=None):
    """KITTI 3D object layout: training/{image_2, velodyne, calib, label_2}, ImageSets/."""
    camera = camera or ses.first_of("rgb")
    lidar = lidar or ses.first_of("lidar")
    if camera is None or ses.sensors[camera]["type"] != "rgb":
        raise ValueError("KITTI 需要一个 RGB 相机")
    if lidar is None:
        raise ValueError("KITTI 需要一个激光雷达")
    tr = os.path.join(out, "training")
    for sub in ("image_2", "velodyne", "calib", "label_2"):
        os.makedirs(os.path.join(tr, sub), exist_ok=True)
    os.makedirs(os.path.join(out, "ImageSets"), exist_ok=True)
    K = np.array(ses.sensors[camera]["K"])
    P = np.hstack([K, np.zeros((3, 1))])
    E_cam, E_lid = ses.extrinsic[camera], ses.extrinsic[lidar]
    cam_from_ego = np.linalg.inv(E_cam)
    # velodyne(KITTI) -> lidar(CARLA): flip y; -> ego -> camera(CARLA) -> camera(KITTI)
    T = np.eye(4)
    T[:3, :3] = C @ (cam_from_ego[:3, :3] @ E_lid[:3, :3] @ F)
    T[:3, 3] = C @ (cam_from_ego[:3, :3] @ E_lid[:3, 3] + cam_from_ego[:3, 3])
    imu = np.eye(4)  # IMU = ego frame, right-handed
    velo_from_ego = np.linalg.inv(E_lid)
    imu[:3, :3] = F @ velo_from_ego[:3, :3] @ F
    imu[:3, 3] = F @ velo_from_ego[:3, 3]

    def row(m):
        return " ".join("%.12e" % x for x in np.asarray(m).reshape(-1))

    # The ego "down" direction in KITTI camera axes (camera y only for a level camera).
    down_cam = C @ (cam_from_ego[:3, :3] @ np.array([0.0, 0.0, -1.0]))
    mount = ses.sensors[camera].get("mount", {})
    tilted = abs(float(mount.get("pitch", 0.0))) > 3.0 or abs(float(mount.get("roll", 0.0))) > 3.0
    W = int(ses.sensors[camera]["attributes"]["image_size_x"])
    H = int(ses.sensors[camera]["attributes"]["image_size_y"])
    ids, pairs, n_obj = [], [], 0
    for idx, frame in enumerate(ses.frames):
        name = "%06d" % len(ids)  # contiguous KITTI indices even if a frame is skipped
        src = ses.file(camera, frame)
        if src is None or ses.file(lidar, frame) is None:
            continue  # without its lidar sweep every object would be filtered out: a false "empty road"
        _copy_image(src, os.path.join(tr, "image_2", name), ".png")
        pts = ses.lidar_points(lidar, frame).astype(np.float32).copy()
        pts_ego = ses.to_ego(lidar, pts)
        pts[:, 1] *= -1.0
        pts.tofile(os.path.join(tr, "velodyne", name + ".bin"))
        with open(os.path.join(tr, "calib", name + ".txt"), "w", encoding="utf-8") as f:
            for k in ("P0", "P1", "P2", "P3"):
                f.write("%s: %s\n" % (k, row(P)))
            f.write("R0_rect: %s\n" % row(np.eye(3)))
            f.write("Tr_velo_to_cam: %s\n" % row(T[:3]))
            f.write("Tr_imu_to_velo: %s\n" % row(imu[:3]))
        lines = []
        for o in ses.labels(frame):
            npts = int(points_in_box(pts_ego, o).sum())
            if npts < min_lidar_pts:
                continue
            corners = (box_corners(o) @ cam_from_ego[:3, :3].T + cam_from_ego[:3, 3]) @ C.T
            center = (np.array(o["center_ego"]) @ cam_from_ego[:3, :3].T + cam_from_ego[:3, 3]) @ C.T
            if center[2] <= 0.5:
                continue
            front = corners[corners[:, 2] > 0.1]
            if len(front) < 2:
                continue
            uv = front @ K.T
            uv = uv[:, :2] / uv[:, 2:3]
            x0, y0, x1, y1 = uv[:, 0].min(), uv[:, 1].min(), uv[:, 0].max(), uv[:, 1].max()
            cx0, cy0, cx1, cy1 = max(0.0, x0), max(0.0, y0), min(W - 1.0, x1), min(H - 1.0, y1)
            if cx1 <= cx0 or cy1 <= cy0:
                continue
            full = (x1 - x0) * (y1 - y0)
            trunc = 1.0 - (cx1 - cx0) * (cy1 - cy0) / full if full > 0 else 1.0
            if len(front) < 8:
                trunc = max(trunc, 0.5)
            ex, ey, ez = o["extent"]
            yaw = math.radians(o["yaw_ego"])
            d_cam = (np.array([math.cos(yaw), math.sin(yaw), 0.0]) @ cam_from_ego[:3, :3].T) @ C.T
            ry = math.atan2(-d_cam[2], d_cam[0])
            alpha = ry - math.atan2(center[0], center[2])
            alpha = (alpha + math.pi) % (2 * math.pi) - math.pi
            occ = 0 if npts >= 50 else 1 if npts >= 10 else 2
            bottom = center + ez * down_cam  # KITTI location = bottom centre
            lines.append("%s %.2f %d %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f" % (
                KITTI_TYPES.get(o["class"], "Misc"), min(1.0, max(0.0, trunc)), occ, alpha, cx0, cy0, cx1, cy1,
                2 * ez, 2 * ey, 2 * ex, bottom[0], bottom[1], bottom[2], ry))
        n_obj += len(lines)
        with open(os.path.join(tr, "label_2", name + ".txt"), "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + ("\n" if lines else ""))
        ids.append(name)
        pairs.append((name, frame))
        if progress:
            progress(idx + 1, len(ses.frames))
    for split in ("train", "trainval"):
        with open(os.path.join(out, "ImageSets", split + ".txt"), "w", encoding="utf-8") as f:
            f.write("\n".join(ids) + "\n")
    with open(os.path.join(out, "ImageSets", "val.txt"), "w", encoding="utf-8") as f:
        f.write("")
    with open(os.path.join(out, "carla_frames.txt"), "w", encoding="utf-8") as f:
        f.write("# kitti_index carla_frame\n")
        f.write("".join("%s %d\n" % (i, fr) for i, fr in pairs))
    with open(os.path.join(out, "README_carla.txt"), "w", encoding="utf-8") as f:
        f.write("由 CARLA CoSim Studio 从 %s 导出。\n相机 %s -> image_2，激光雷达 %s -> velodyne（y 轴已翻转为 KITTI 左手->右手约定）。\n"
                "occluded 由框内激光点数估计：>=50 为 0，>=10 为 1，其余为 2；框内点数少于 %d 的目标不导出。\n"
                % (ses.root, camera, lidar, min_lidar_pts))
    res = {"format": "kitti", "out": os.path.abspath(out), "frames": len(ids), "objects": n_obj,
           "camera": camera, "lidar": lidar}
    if tilted:
        res["warning"] = ("相机 %s 有俯仰或侧倾：KITTI 的朝向角 ry 定义在相机 y 轴上，倾斜相机的朝向只是近似"
                          "（位置已按真实竖直方向计算）。训练 KITTI 模型建议用水平安装的相机" % camera)
    return res


def _radar_pcd(path, det, vr_comp=None):
    """nuScenes radar .pcd (binary, 18 fields, as read by RadarPointCloud.from_file)."""
    fields = ["x", "y", "z", "dyn_prop", "id", "rcs", "vx", "vy", "vx_comp", "vy_comp", "is_quality_valid",
              "ambig_state", "x_rms", "y_rms", "invalid_state", "pdh0", "vx_rms", "vy_rms"]
    types = ["F", "F", "F", "I", "I", "F", "F", "F", "F", "F", "I", "I", "I", "I", "I", "I", "I", "I"]
    sizes = [4, 4, 4, 1, 2, 4, 4, 4, 4, 4, 1, 1, 1, 1, 1, 1, 1, 1]
    fmt = "<fffbhfffffbbbbbbbb"
    n = len(det)
    header = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS %s\nSIZE %s\nTYPE %s\nCOUNT %s\n"
              "WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n" % (
                  " ".join(fields), " ".join(map(str, sizes)), " ".join(types), " ".join(["1"] * 18), n, n))
    with open(path, "wb") as f:
        f.write(header.encode())
        for i, (x, y, z, vr) in enumerate(det):
            r = math.hypot(x, y) or 1.0
            vx, vy = vr * x / r, vr * y / r
            vc = vr if vr_comp is None else vr_comp[i]
            f.write(struct.pack(fmt, x, y, z, 0, i, 0.0, vx, vy, vc * x / r, vc * y / r, 1, 3, 0, 0, 0, 0, 0, 0))
        f.write(b"\n")  # the devkit reader asserts end < len(data) for the last field


def export_nuscenes(ses, out, version="v1.0-carla", progress=None):
    """nuScenes layout: <out>/<version>/*.json + samples/<CHANNEL>/…; one scene per session."""
    tables = {k: [] for k in ("category", "attribute", "visibility", "instance", "sensor", "calibrated_sensor",
                              "ego_pose", "log", "scene", "sample", "sample_data", "sample_annotation", "map")}
    cats = sorted(set(NUSC_CATEGORIES.values()))
    for c in cats:
        tables["category"].append({"token": token("cat", c), "name": c, "description": c})
    attrs = ["vehicle.moving", "vehicle.stopped", "cycle.with_rider", "pedestrian.moving", "pedestrian.standing"]
    for a in attrs:
        tables["attribute"].append({"token": token("attr", a), "name": a, "description": a})
    for lvl, desc in (("1", "0-40"), ("2", "40-60"), ("3", "60-80"), ("4", "80-100")):
        tables["visibility"].append({"token": lvl, "level": "v" + desc, "description": "visibility of whole object is between %s%%" % desc})
    name = os.path.basename(ses.root)
    log_tok, scene_tok, map_tok = token(name, "log"), token(name, "scene"), token(name, "map")
    location = ses.meta.get("map", "carla").split("/")[-1]
    tables["log"].append({"token": log_tok, "logfile": name, "vehicle": ses.meta.get("ego_blueprint", ""),
                          "date_captured": ses.meta.get("started", "")[:10], "location": location})
    tables["map"].append({"token": map_tok, "log_tokens": [log_tok], "category": "semantic_prior", "filename": ""})

    channels = {}
    lidar_name = ses.first_of("lidar")
    for n, s in ses.sensors.items():
        mod = {"rgb": "camera", "lidar": "lidar", "radar": "radar"}.get(s["type"])
        if mod is None:
            continue
        # nuScenes tools look the lidar up as LIDAR_TOP, whatever the rig called it.
        ch = "LIDAR_TOP" if n == lidar_name else n.upper()
        channels[n] = ch
        tables["sensor"].append({"token": token("sensor", ch), "channel": ch, "modality": mod})
        E = ses.extrinsic[n]
        R = F @ E[:3, :3] @ (C.T if mod == "camera" else F)
        tables["calibrated_sensor"].append({"token": token(name, "calib", ch), "sensor_token": token("sensor", ch),
                                            "translation": (F @ E[:3, 3]).tolist(), "rotation": quat(R),
                                            "camera_intrinsic": s["K"] if mod == "camera" else []})
        os.makedirs(os.path.join(out, "samples", ch), exist_ok=True)

    # A sample without its lidar sweep breaks the usual converters (they read
    # sample["data"]["LIDAR_TOP"]): leave such frames out.
    frames = [f for f in ses.frames if lidar_name is None or ses.file(lidar_name, f) is not None]
    sample_toks = [token(name, "sample", f) for f in frames]
    last_sd = {}
    inst_anns = {}
    radar_names = [n for n, s in ses.sensors.items() if s["type"] == "radar"]
    for i, frame in enumerate(frames):
        ego = ses.ego(frame)
        ts = int(round(ego.get("timestamp", i * 0.1) * 1e6))
        M = tf_matrix(ego["pose"])
        pose_tok = token(name, "pose", frame)
        tables["ego_pose"].append({"token": pose_tok, "timestamp": ts, "translation": (F @ M[:3, 3]).tolist(),
                                   "rotation": quat(F @ M[:3, :3] @ F)})
        st = sample_toks[i]
        tables["sample"].append({"token": st, "timestamp": ts, "scene_token": scene_tok,
                                 "prev": sample_toks[i - 1] if i else "", "next": sample_toks[i + 1] if i + 1 < len(sample_toks) else ""})
        for n, ch in channels.items():
            src = ses.file(n, frame)
            if src is None:
                continue
            kind = ses.sensors[n]["type"]
            base = os.path.join("samples", ch, "%s__%s__%d" % (name, ch, ts))
            if kind == "rgb":
                rel = os.path.relpath(_copy_image(src, os.path.join(out, base), os.path.splitext(src)[1].lower()), out)
                h, w = int(ses.sensors[n]["attributes"]["image_size_y"]), int(ses.sensors[n]["attributes"]["image_size_x"])
                fmt = rel.rsplit(".", 1)[1]
            elif kind == "lidar":
                p = ses.lidar_points(n, frame).astype(np.float32)
                p5 = np.zeros((len(p), 5), np.float32)
                p5[:, :4] = p
                p5[:, 1] *= -1.0
                p5[:, 3] *= 255.0  # CARLA intensity 0-1, nuScenes 0-255
                rel = base + ".pcd.bin"
                p5.tofile(os.path.join(out, rel))
                h = w = 0
                fmt = "pcd"
            else:
                det = ses.radar_points(n, frame)
                # vx_comp / vy_comp: without the radar's own motion (CARLA measures
                # relative to the sensor): add the ego velocity along each ray.
                v_world = np.array(ego.get("velocity", [0.0, 0.0, 0.0]), dtype=float)
                v_radar = (M[:3, :3] @ ses.extrinsic[n][:3, :3]).T @ v_world
                rng = np.linalg.norm(det[:, :3], axis=1)
                rng[rng == 0] = 1.0
                vr_comp = det[:, 3] + (det[:, :3] @ v_radar) / rng
                det = det * np.array([1, -1, 1, 1])
                rel = base + ".pcd"
                _radar_pcd(os.path.join(out, rel), det, vr_comp)
                h = w = 0
                fmt = "pcd"
            sd_tok = token(name, "sd", ch, frame)
            rec = {"token": sd_tok, "sample_token": st, "ego_pose_token": pose_tok,
                   "calibrated_sensor_token": token(name, "calib", ch), "timestamp": ts, "fileformat": fmt,
                   "is_key_frame": True, "height": h, "width": w, "filename": rel.replace(os.sep, "/"),
                   "prev": last_sd.get(ch, {}).get("token", ""), "next": ""}
            if ch in last_sd:
                last_sd[ch]["next"] = sd_tok
            last_sd[ch] = rec
            tables["sample_data"].append(rec)
        # Annotations in the global frame. Boxes are upright in the world (yaw
        # only), so points are counted there too: with the ego pitched or rolled
        # an upright box in the ego frame would hold different points.
        pts_w = ses.to_ego(lidar_name, ses.lidar_points(lidar_name, frame)) @ M[:3, :3].T + M[:3, 3] if lidar_name else np.zeros((0, 3))
        rad_w = (np.concatenate([ses.to_ego(r, ses.radar_points(r, frame)) for r in radar_names]) @ M[:3, :3].T + M[:3, 3]
                 if radar_names else np.zeros((0, 3)))
        yaw_ego_world = ego["pose"]["yaw"]
        for o in ses.labels(frame):
            cls = o["class"]
            c_world = M[:3, :3] @ np.array(o["center_ego"]) + M[:3, 3]
            box_w = {"center_ego": c_world, "extent": o["extent"], "yaw_ego": yaw_ego_world + o["yaw_ego"]}
            npts = int(points_in_box(pts_w, box_w).sum()) if len(pts_w) else 0
            nrad = int(points_in_box(rad_w, box_w).sum()) if len(rad_w) else 0
            yaw_rh = -math.radians(yaw_ego_world + o["yaw_ego"])
            speed = math.hypot(o["velocity"][0], o["velocity"][1])
            if cls == "pedestrian":
                attr = "pedestrian.moving" if speed > 0.3 else "pedestrian.standing"
            elif cls in ("bicycle", "motorcycle"):
                attr = "cycle.with_rider"
            else:
                attr = "vehicle.moving" if speed > 0.3 else "vehicle.stopped"
            inst = token(name, "inst", o["id"])
            ann_tok = token(name, "ann", o["id"], frame)
            vis = "4" if npts >= 50 else "3" if npts >= 10 else "2" if npts >= 1 else "1"
            ann = {"token": ann_tok, "sample_token": st, "instance_token": inst,
                   "visibility_token": vis, "attribute_tokens": [token("attr", attr)],
                   "translation": (F @ c_world).tolist(), "size": [2 * o["extent"][1], 2 * o["extent"][0], 2 * o["extent"][2]],
                   "rotation": [math.cos(yaw_rh / 2), 0.0, 0.0, math.sin(yaw_rh / 2)],
                   "prev": "", "next": "", "num_lidar_pts": npts, "num_radar_pts": nrad}
            prev = inst_anns.get(inst)
            if prev:
                ann["prev"] = prev[-1]["token"]
                prev[-1]["next"] = ann_tok
                prev.append(ann)
            else:
                inst_anns[inst] = [ann]
                tables["instance"].append({"token": inst, "category_token": token("cat", NUSC_CATEGORIES.get(cls, "vehicle.car")),
                                           "nbr_annotations": 0, "first_annotation_token": ann_tok, "last_annotation_token": ""})
            tables["sample_annotation"].append(ann)
        if progress:
            progress(i + 1, len(frames))
    for inst in tables["instance"]:
        anns = inst_anns[inst["token"]]
        inst["nbr_annotations"] = len(anns)
        inst["last_annotation_token"] = anns[-1]["token"]
    tables["scene"].append({"token": scene_tok, "log_token": log_tok, "nbr_samples": len(sample_toks),
                            "first_sample_token": sample_toks[0] if sample_toks else "",
                            "last_sample_token": sample_toks[-1] if sample_toks else "",
                            "name": name, "description": "CARLA %s · %s" % (location, ses.meta.get("started", ""))})
    vdir = os.path.join(out, version)
    os.makedirs(vdir, exist_ok=True)
    for k, rows in tables.items():
        with open(os.path.join(vdir, k + ".json"), "w", encoding="utf-8") as f:
            json.dump(rows, f, indent=0)
    return {"format": "nuscenes", "out": os.path.abspath(out), "version": version, "frames": len(frames),
            "objects": len(tables["sample_annotation"]), "channels": sorted(channels.values())}


class Exporter:
    """Runs one export in a background thread and reports progress."""

    def __init__(self, emit):
        self.emit = emit
        self.thread = None
        self.root = None

    def busy(self):
        return self.thread is not None and self.thread.is_alive()

    def start(self, root, fmt, out, **opts):
        if self.busy():
            raise RuntimeError("已有导出在进行")
        ses = Session(root)
        if not ses.frames:
            raise ValueError("这个数据集没有帧")
        need = estimate_export(ses, fmt, opts.get("camera"))
        probe = os.path.abspath(out)
        while not os.path.exists(probe):
            parent = os.path.dirname(probe)
            if parent == probe:  # e.g. a drive letter or network share that does not exist
                raise RuntimeError("输出目录所在的磁盘或网络位置不存在：%s" % out)
            probe = parent
        free = shutil.disk_usage(probe).free
        if need > free - 10e9:
            raise RuntimeError("预计需要 %.1f GB，磁盘只剩 %.1f GB（需保留 10 GB）" % (need / 1e9, free / 1e9))
        if os.path.exists(out) and os.listdir(out):
            raise RuntimeError("输出目录不是空的：%s" % out)

        def progress(done, total):
            self.emit({"event": "export_progress", "done": done, "total": total})

        def run():
            try:
                if fmt == "kitti":
                    r = export_kitti(ses, out, opts.get("camera"), opts.get("lidar"), int(opts.get("min_lidar_pts", 1)), progress)
                else:
                    r = export_nuscenes(ses, out, progress=progress)
                r["size_mb"] = dir_size(out) / 1e6
                self.emit({"event": "export_done", "ok": True, "result": r})
            except Exception as e:
                self.emit({"event": "export_done", "ok": False, "error": str(e)})

        self.root = ses.root
        self.thread = threading.Thread(target=run, daemon=True)
        self.thread.start()
        return {"estimate_mb": need / 1e6, "frames": len(ses.frames)}
