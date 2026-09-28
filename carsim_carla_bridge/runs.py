"""Run records for comparing runs (the GUI's 运行对比 page and panel).

Every run writes a folder <record dir>/<YYYYmmdd_HHMMSS>_<algorithm>/ with
run.json (settings, how it ended, key figures), log.csv (ego, CarSim
exports, the algorithm's outputs u1..un), log_lane.csv, log_objects.csv and,
when the algorithm sets self.debug, log_debug.csv (session.run_dir,
scene.Recorder). list_runs() summarises the folders of a record dir;
run_series() reads one run's time series for plotting, thinned to at most
max_points samples. Files only: no CARLA, no CarSim.
"""
import csv
import json
import math
import os

KPI_KEYS = ("lane_offset_rms", "lane_offset_max", "heading_err_max", "time_off_lane", "collisions",
            "min_gap_ahead", "ay_max", "distance")


def _num(v):
    try:
        x = float(v)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def _read_json(path):
    try:
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
        return d if isinstance(d, dict) else None
    except (OSError, ValueError):
        return None


def list_runs(root, base=None, limit=300):
    """The runs in a record dir, newest first: {"root", "runs": [{"folder", "name",
    "controller", "map", "spawn_index", "dynamics", "vehicle", "end", "end_reason",
    "duration", "kpi", "debug"}], "error"}. root relative to base (the bridge dir)."""
    base = os.path.abspath(base or os.getcwd())
    root = os.path.abspath(os.path.join(base, root or "runs"))
    out = {"root": root, "runs": [], "error": ""}
    if not os.path.isdir(root):
        out["error"] = "运行记录目录不存在：%s（第一次运行后才有）" % root
        return out
    names = sorted((n for n in os.listdir(root) if os.path.isfile(os.path.join(root, n, "run.json"))), reverse=True)
    for n in names[:limit]:
        folder = os.path.join(root, n)
        r = _read_json(os.path.join(folder, "run.json")) or {}
        t0, t1 = _num(r.get("t_start")), _num(r.get("t_end"))
        vehicle = "模拟 CarSim" if r.get("carsim_mock") else "Chrono 宝马" if r.get("carsim_chrono") else \
            "CARLA 物理" if r.get("dynamics") == "carla" else "CarSim"
        out["runs"].append({
            "folder": folder, "name": n,
            "controller": os.path.basename(str(r.get("controller") or "")) or str(r.get("driver") or ""),
            "map": r.get("map"), "spawn_index": r.get("spawn_index"), "dynamics": r.get("dynamics"), "vehicle": vehicle,
            "end": r.get("end"), "end_reason": r.get("end_reason") or "",
            "duration": (t1 - t0) if t0 is not None and t1 is not None else None,
            "kpi": {k: (r.get("kpi") or {}).get(k) for k in KPI_KEYS} if r.get("kpi") else {},
            "debug": os.path.isfile(os.path.join(folder, "log_debug.csv")),
            "units": r.get("units") or {},
        })
    return out


def _rows(path):
    try:
        with open(path, newline="", encoding="utf-8") as f:
            return list(csv.DictReader(f))
    except OSError:
        return []


def run_series(folder, max_points=2000):
    """One run's time series: {"name", "t", "x", "y", "speed", "offset", "heading_err",
    "u": [u1, u2, u3], "debug": {name: [...]}, "units", "kpi", "controller"}; None where
    a sample has no value. Thinned evenly to at most max_points samples."""
    folder = os.path.abspath(folder)
    run = _read_json(os.path.join(folder, "run.json"))
    if run is None:
        raise ValueError("不是一次运行的记录（没有 run.json）：%s" % folder)
    main = _rows(os.path.join(folder, "log.csv"))
    if not main:  # an older config's file name: <base>.csv next to its run.json
        cands = [n for n in os.listdir(folder) if n.endswith(".csv") and not n.endswith(("_lane.csv", "_objects.csv", "_debug.csv"))]
        main = _rows(os.path.join(folder, cands[0])) if cands else []
    base = "log"
    lane = {r.get("t"): r for r in _rows(os.path.join(folder, base + "_lane.csv"))}
    dbg_rows = _rows(os.path.join(folder, base + "_debug.csv"))
    dbg = {r.get("t"): r for r in dbg_rows}
    dbg_names = [k for k in (dbg_rows[0].keys() if dbg_rows else []) if k not in ("t", "frame")]
    step = max(1, int(math.ceil(len(main) / float(max_points)))) if main else 1
    pick = main[::step]
    if main and pick[-1] is not main[-1]:
        pick.append(main[-1])  # always the end of the run

    def col(rows, *names):
        for n in names:
            if rows and n in rows[0]:
                return [_num(r.get(n)) for r in rows]
        return [None] * len(rows)
    out = {"name": os.path.basename(folder), "folder": folder,
           "controller": os.path.basename(str(run.get("controller") or "")) or str(run.get("driver") or ""),
           "units": run.get("units") or {}, "kpi": run.get("kpi") or {},
           "t": col(pick, "t"), "x": col(pick, "ego_X", "Xo"), "y": col(pick, "ego_Y", "Yo"),
           "speed": col(pick, "ego_Speed", "Vx"),
           "u": [col(pick, "u%d" % i) for i in (1, 2, 3)],
           "offset": [_num((lane.get(r.get("t")) or {}).get("offset")) for r in pick],
           "heading_err": [_num((lane.get(r.get("t")) or {}).get("heading_err")) for r in pick],
           "debug": {n: [_num((dbg.get(r.get("t")) or {}).get(n)) for r in pick] for n in dbg_names}}
    # What u1..u3 are: the Chrono BMW's [ax, front wheel angle], CarSim's usual [throttle, brake, steering wheel].
    n_u = sum(1 for i in (1, 2, 3) if main and "u%d" % i in main[0])
    if run.get("carsim_chrono"):
        out["u_names"] = ["ax (m/s²)", "前轮转角 (rad)"]
    elif n_u == 3:
        out["u_names"] = ["油门", "制动", "方向盘转角 (°)"]
    else:
        out["u_names"] = ["导入 %d" % i for i in range(1, n_u + 1)]
    return out


def ego_track(folder):
    """The ego's recorded pose over a run, for replaying it in CARLA: {"t", "x", "y", "z",
    "yaw" (deg: the run's angle unit converted), "map", "spawn_index"}; the reference point's
    CarSim global position (m), as the record has it."""
    folder = os.path.abspath(folder)
    run = _read_json(os.path.join(folder, "run.json"))
    if run is None:
        raise ValueError("不是一次运行的记录（没有 run.json）：%s" % folder)
    main = _rows(os.path.join(folder, "log.csv"))
    need = ("ego_X", "ego_Y", "ego_Yaw")
    if not main or any(k not in main[0] for k in need):
        raise ValueError("这次运行的记录里没有主车的位置和航向（log.csv 要有 ego_X、ego_Y、ego_Yaw："
                         "“场景信息”页自车的“写进记录”勾选 X、Y、Yaw）")
    deg = (180.0 / math.pi) if (run.get("units") or {}).get("angle") == "rad" else 1.0
    t, x, y, z, yaw = [], [], [], [], []
    for r in main:
        v = [_num(r.get(k)) for k in ("t", "ego_X", "ego_Y", "ego_Z", "ego_Yaw")]
        if None in (v[0], v[1], v[2], v[4]):
            continue
        t.append(v[0])
        x.append(v[1])
        y.append(v[2])
        z.append(v[3] or 0.0)
        yaw.append(v[4] * deg)
    if not t:
        raise ValueError("这次运行的记录里没有主车的位置")
    return {"t": t, "x": x, "y": y, "z": z, "yaw": yaw, "map": run.get("map"), "spawn_index": run.get("spawn_index")}


def pose_at(track, t):
    """The recorded pose at time t (interpolated, held at the ends): (t, x, y, z, yaw deg)."""
    ts = track["t"]
    if t <= ts[0]:
        i, k = 0, 0.0
    elif t >= ts[-1]:
        i, k = len(ts) - 1, 0.0
    else:
        lo, hi = 0, len(ts) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if ts[mid] <= t:
                lo = mid
            else:
                hi = mid
        i, k = lo, (t - ts[lo]) / (ts[hi] - ts[lo]) if ts[hi] > ts[lo] else 0.0
    j = min(i + 1, len(ts) - 1)

    def lerp(a):
        return a[i] + k * (a[j] - a[i])
    dyaw = (track["yaw"][j] - track["yaw"][i] + 180.0) % 360.0 - 180.0
    return (min(max(t, ts[0]), ts[-1]), lerp(track["x"]), lerp(track["y"]), lerp(track["z"]), track["yaw"][i] + k * dyaw)
