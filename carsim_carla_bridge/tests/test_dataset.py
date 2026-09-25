"""Dataset browser + KITTI / nuScenes export, end to end (tiny capture, deleted afterwards).

    python tests/test_dataset.py [--port 2000]

With the nuScenes devkit importable (pip install nuscenes-devkit), the
nuScenes export is also loaded and cross-checked with the official code.
"""
import base64
import glob
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
from test_backend import Conn, check  # noqa: E402
import dataset as dsmod  # noqa: E402

PORT = 57194


def sensor(name, kind, x, y, z, yaw=0.0, pitch=0.0, **attrs):
    import rig
    return rig.sensor(name, kind, x, y, z, yaw=yaw, pitch=pitch, **attrs)


def wait_export(c, timeout=300):
    e = c.wait_event(lambda e: e.get("event") == "export_done", timeout)
    c.events.remove(e)
    return e


def kitti_checks(out, min_pts):
    tr = os.path.join(out, "training")
    n = {sub: len(os.listdir(os.path.join(tr, sub))) for sub in ("image_2", "velodyne", "calib", "label_2")}
    check("kitti files", len(set(n.values())) == 1 and list(n.values())[0] == 6, n)
    bad, total, center_in_2d, untrunc, outside = [], 0, 0, 0, []
    for lab in sorted(glob.glob(os.path.join(tr, "label_2", "*.txt"))):
        idx = os.path.basename(lab)[:-4]
        calib = {}
        for line in open(os.path.join(tr, "calib", idx + ".txt")):
            k, v = line.split(":", 1)
            calib[k] = np.array([float(x) for x in v.split()])
        P2 = calib["P2"].reshape(3, 4)
        Tr = np.vstack([calib["Tr_velo_to_cam"].reshape(3, 4), [0, 0, 0, 1]])
        velo = np.fromfile(os.path.join(tr, "velodyne", idx + ".bin"), dtype=np.float32).reshape(-1, 4)
        cam = (np.hstack([velo[:, :3], np.ones((len(velo), 1))]) @ Tr.T)[:, :3]
        for line in open(lab):
            f = line.split()
            if not f:
                continue
            h, w, l = float(f[8]), float(f[9]), float(f[10])
            x, y, z, ry = float(f[11]), float(f[12]), float(f[13]), float(f[14])
            # KITTI: location = bottom centre, box y from -h to 0, rotation about camera y.
            d = cam - np.array([x, y, z])
            cx = d[:, 0] * math.cos(ry) - d[:, 2] * math.sin(ry)
            cz = d[:, 0] * math.sin(ry) + d[:, 2] * math.cos(ry)
            inside = (np.abs(cx) <= l / 2) & (np.abs(cz) <= w / 2) & (d[:, 1] <= 0) & (d[:, 1] >= -h)
            total += 1
            if inside.sum() < min_pts:
                bad.append((idx, f[0], int(inside.sum())))
            if float(f[1]) > 0.05:
                continue  # truncated: the 2D box is clipped at the image edge
            untrunc += 1
            c3 = P2 @ np.array([x, y - h / 2, z, 1.0])
            u, v = c3[0] / c3[2], c3[1] / c3[2]
            x0, y0, x1, y1 = map(float, f[4:8])
            if x0 - 2 <= u <= x1 + 2 and y0 - 2 <= v <= y1 + 2:
                center_in_2d += 1
            else:
                outside.append((idx, f[0], "trunc %s" % f[1], "centre %.0f,%.0f" % (u, v), "box %s" % " ".join(f[4:8])))
    check("kitti labels have objects", total > 0, total)
    check("kitti boxes contain their lidar points (Tr_velo_to_cam, location, ry)", not bad, "%d boxes, bad %s" % (total, bad[:5]))
    check("kitti 3D centre projects inside the 2D box (untruncated objects)", untrunc > 0 and center_in_2d == untrunc,
          "%d / %d, outside: %s" % (center_in_2d, untrunc, outside[:3]))


def nuscenes_checks(out, ses):
    try:
        from nuscenes.nuscenes import NuScenes
        from nuscenes.utils.data_classes import LidarPointCloud, RadarPointCloud
        from nuscenes.utils.geometry_utils import points_in_box, view_points
    except ImportError:
        print("INFO nuscenes-devkit not installed: skipping the official-devkit checks")
        return
    nusc = NuScenes(version="v1.0-carla", dataroot=out, verbose=False)
    check("nuscenes devkit loads the export", len(nusc.sample) == 6 and len(nusc.scene) == 1,
          "%d samples, %d annotations" % (len(nusc.sample), len(nusc.sample_annotation)))
    # 1) lidar: devkit box (global -> ego -> sensor via quaternions) vs num_lidar_pts
    diffs, n = [], 0
    for sample in nusc.sample:
        sd = sample["data"]["LIDAR_TOP"]
        path, boxes, _ = nusc.get_sample_data(sd)
        pc = LidarPointCloud.from_file(path)
        for b in boxes:
            ann = nusc.get("sample_annotation", b.token)
            cnt = int(points_in_box(b, pc.points[:3]).sum())
            diffs.append(abs(cnt - ann["num_lidar_pts"]) - max(2, 0.05 * ann["num_lidar_pts"]))
            n += ann["num_lidar_pts"] > 0
    check("nuscenes: devkit counts the same lidar points in each box", n > 0 and max(diffs) <= 0,
          "%d boxes with points, worst excess %.1f" % (n, max(diffs)))
    # 2) camera: devkit projection of box centres vs our own CARLA-frame projection
    errs = []
    for sample, frame in zip(sorted(nusc.sample, key=lambda s: s["timestamp"]), ses.frames):
        sd = sample["data"]["CAM_FRONT"]
        _, boxes, K = nusc.get_sample_data(sd)
        inv = np.linalg.inv(ses.extrinsic["cam_front"])
        by_id = {}
        for o in ses.labels(frame):
            by_id[dsmod.token(os.path.basename(ses.root), "inst", o["id"])] = o
        for b in boxes:
            if b.center[2] < 1.0:
                continue
            ann = nusc.get("sample_annotation", b.token)
            o = by_id[ann["instance_token"]]
            ours = (np.array(o["center_ego"]) @ inv[:3, :3].T + inv[:3, 3]) @ dsmod.C.T
            u1 = view_points(b.center.reshape(3, 1), K, normalize=True)[:2, 0]
            u2 = (np.array(K) @ ours)[:2] / ours[2]
            errs.append(float(np.linalg.norm(u1 - u2)))
    check("nuscenes: camera extrinsic / ego pose round trip (pixel error)", errs and max(errs) < 2.0,
          "%d boxes, max %.2f px" % (len(errs), max(errs) if errs else -1))
    # 3) radar files parse with the devkit reader
    RadarPointCloud.disable_filters()
    sd = nusc.sample[0]["data"]["RADAR_FRONT"]
    rp = RadarPointCloud.from_file(os.path.join(out, nusc.get("sample_data", sd)["filename"]))
    csv_rows = len(ses.radar_points("radar_front", ses.frames[0]))
    check("nuscenes: radar .pcd readable by the devkit", rp.points.shape[1] == csv_rows, "%d points" % rp.points.shape[1])


def main():
    carla_port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    tmp = tempfile.mkdtemp(prefix="cc_ds_")
    env = dict(os.environ)
    env.pop("PYTHONPATH", None)  # the backend runs in the project environment, not the devkit's
    proc = subprocess.Popen([sys.executable, os.path.join(HERE, "..", "backend_server.py"), "--port", str(PORT)],
                            cwd=os.path.join(HERE, ".."), env=env)
    try:
        time.sleep(2)
        c = Conn(PORT)
        c.call("connect", host="localhost", port=carla_port)
        check("tf_matrix == carla.Transform.get_matrix", _matrix_check())
        c.call("spawn_ego", blueprint="vehicle.tesla.model3", spawn_index=3)
        c.call("spawn_traffic", vehicles=30, walkers=10, seed=2, safe=True)
        sensors = [sensor("cam_front", "rgb", 1.5, 0.0, 1.6, image_size_x=800, image_size_y=450, fov=90.0),
                   sensor("cam_front_semantic", "semantic", 1.5, 0.0, 1.6, image_size_x=800, image_size_y=450, fov=90.0),
                   sensor("lidar_top", "lidar", 0.0, 0.0, 1.9, channels=32, range=60.0, points_per_second=300000),
                   sensor("radar_front", "radar", 2.3, 0.0, 0.6, range=80.0)]
        cfg = c.call("default_config")
        cfg["drive"].update({"dynamics": "carla", "carla_driver": "autopilot", "tm_ignore_lights": True})
        cfg["sync"].update({"frame_dt": 0.1, "duration": 0.0})
        cfg["rig"]["sensors"] = sensors
        cfg["collect"].update({"enabled": True, "out_dir": tmp, "session": "ds", "max_frames": 6, "max_gb": 0.2,
                               "capture_every": 3, "image_format": "jpg"})
        info = c.call("cosim_start", config=cfg)
        c.wait_event(lambda e: e.get("event") == "cosim_state" and e["state"] in ("finished", "error"), 300)
        root = info["collect"]["root"]

        lst = c.call("dataset_list", out_dir=tmp)
        check("dataset_list", len(lst) == 1 and lst[0]["frames"] == 6, [(s["name"], s["frames"], round(s["size_mb"], 1)) for s in lst])
        ses = dsmod.Session(root)

        # ---- browser rendering ----------------------------------------------
        for sname, kind in (("cam_front", "rgb"), ("cam_front_semantic", "semantic"), ("lidar_top", "lidar"), ("radar_front", "radar")):
            r = c.call("dataset_frame", root=root, frame=ses.frames[-1], sensor=sname, max_w=640)
            px = base64.b64decode(r["rgb"])
            check("dataset_frame %s" % kind, len(px) == r["w"] * r["h"] * 3 and r["w"] > 0, "%dx%d, %d objects" % (r["w"], r["h"], len(r["objects"])))
        check("lidar -> ego convention (semantic lidar: points hit by an actor fall in its box)", _semantic_lidar_check(carla_port))
        c.call("clear_traffic")

        # ---- KITTI -------------------------------------------------------------
        kout = os.path.join(tmp, "export_kitti")
        c.call("dataset_export", root=root, format="kitti", out=kout, camera="cam_front", lidar="lidar_top", min_lidar_pts=5)
        e = wait_export(c)
        check("kitti export", e["ok"], e.get("result") or e.get("error"))
        if e["ok"]:
            kitti_checks(kout, 5)
        try:
            c.call("dataset_export", root=root, format="kitti", out=kout)
            check("export refuses a non-empty output folder", False)
        except RuntimeError as err:
            check("export refuses a non-empty output folder", "不是空的" in str(err))

        # ---- nuScenes ----------------------------------------------------------
        nout = os.path.join(tmp, "export_nuscenes")
        c.call("dataset_export", root=root, format="nuscenes", out=nout)
        e = wait_export(c)
        check("nuscenes export", e["ok"], e.get("result") or e.get("error"))
        if e["ok"]:
            nuscenes_checks(nout, ses)

        # ---- delete --------------------------------------------------------------
        try:
            c.call("dataset_delete", root=kout)
            check("delete refuses a folder that is not a session", False)
        except RuntimeError as err:
            check("delete refuses a folder that is not a session", "拒绝删除" in str(err))
        r = c.call("dataset_delete", root=root)
        check("dataset_delete", not os.path.exists(root), "%.1f MB" % r["size_mb"])
        c.call("destroy_ego")
    finally:
        proc.terminate()
        proc.wait()
        shutil.rmtree(tmp, ignore_errors=True)
    print("ALL DATASET TESTS PASSED")  # check() exits on the first failure


def _semantic_lidar_check(port):
    """The semantic lidar tags each point with the actor it hit. Transformed
    with the same extrinsic as the dataset lidar, those points must land in
    the actor's ground-truth box (built like collector._labels) read at the
    same frame; with the y axis flipped they must not."""
    import carla
    cl = carla.Client("localhost", port)
    cl.set_timeout(20)
    w = cl.get_world()
    ego = [a for a in w.get_actors().filter("vehicle.*") if a.attributes.get("role_name") == "hero"][0]
    orig = w.get_settings()
    s = w.get_settings()
    s.synchronous_mode, s.fixed_delta_seconds = True, 0.1
    w.apply_settings(s)
    bp = w.get_blueprint_library().find("sensor.lidar.ray_cast_semantic")
    for k, v in (("channels", "32"), ("range", "50"), ("points_per_second", "300000"), ("rotation_frequency", "10")):
        bp.set_attribute(k, v)
    mount = {"x": 0.0, "y": 0.0, "z": 1.9, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    lid = w.spawn_actor(bp, carla.Transform(carla.Location(0, 0, 1.9)), attach_to=ego)
    got = []
    lid.listen(got.append)
    try:
        for _ in range(3):
            frame = w.tick()   # one full sweep per tick (10 Hz rotation, 0.1 s step)
        t0 = time.time()
        while not any(m.frame == frame for m in got) and time.time() - t0 < 5:
            time.sleep(0.02)
        snap = w.get_snapshot()
        meas = [m for m in got if m.frame == frame][0]
    finally:
        lid.stop()
        lid.destroy()
        w.apply_settings(orig)
    dt = np.dtype([("x", "f4"), ("y", "f4"), ("z", "f4"), ("cos", "f4"), ("idx", "u4"), ("tag", "u4")])
    pts = np.frombuffer(meas.raw_data, dtype=dt)
    E = dsmod.tf_matrix(mount)
    ego_tf = snap.find(ego.id).get_transform()
    inv = np.array(ego_tf.get_inverse_matrix())
    tested = ok_right = ok_wrong = 0
    for a in w.get_actors().filter("vehicle.*"):
        sel = pts[pts["idx"] == a.id]
        if a.id == ego.id or len(sel) < 20 or snap.find(a.id) is None:
            continue
        t = snap.find(a.id).get_transform()
        bb = a.bounding_box
        cw = t.transform(bb.location)
        ce = inv @ np.array([cw.x, cw.y, cw.z, 1.0])
        if abs(ce[1]) < 1.5:
            continue  # straight ahead / behind looks the same with y flipped
        obj = {"center_ego": ce[:3].tolist(), "extent": [bb.extent.x + 0.1, bb.extent.y + 0.1, bb.extent.z + 0.1],
               "yaw_ego": t.rotation.yaw - ego_tf.rotation.yaw}
        xyz = np.stack([sel["x"], sel["y"], sel["z"]], 1).astype(np.float64)
        right = dsmod.points_in_box(xyz @ E[:3, :3].T + E[:3, 3], obj).mean()
        flip = xyz * np.array([1, -1, 1])
        wrong = dsmod.points_in_box(flip @ E[:3, :3].T + E[:3, 3], obj).mean()
        tested += 1
        ok_right += right > 0.9
        ok_wrong += wrong > 0.9
    print("INFO semantic lidar: %d vehicles hit, %d with >90%% of their points in the box, %d if y were flipped"
          % (tested, ok_right, ok_wrong))
    return tested >= 2 and ok_right == tested and ok_wrong == 0


def _matrix_check():
    import random
    import carla
    for _ in range(20):
        d = {"x": random.uniform(-50, 50), "y": random.uniform(-50, 50), "z": random.uniform(0, 5),
             "roll": random.uniform(-30, 30), "pitch": random.uniform(-30, 30), "yaw": random.uniform(-180, 180)}
        t = carla.Transform(carla.Location(d["x"], d["y"], d["z"]), carla.Rotation(pitch=d["pitch"], yaw=d["yaw"], roll=d["roll"]))
        if np.abs(np.array(t.get_matrix()) - dsmod.tf_matrix(d)).max() > 1e-4:
            return False
    return True


if __name__ == "__main__":
    main()
