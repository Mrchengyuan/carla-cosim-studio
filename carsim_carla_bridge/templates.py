"""工程模板（界面“文件 → 从模板新建”）：常用的几种工程，各是盖在默认配置上的一层。

新建时这台机器上的设置保留（KEEP）：CARLA 的地址、车型、出生点，CarSim 的 .sim / python_carsim_env 路径、
远程、Chrono 的 Python、导出变量和单位，车辆参数辨识的输入。keep_algorithm 的模板还保留当前的算法文件和
它的参数（换场景、不换算法）。town04_start：界面随后切到 Town04 高速起点（和“测试场景”页的按钮一样）。
"""
import copy

import settings

KEEP = {"carla": ("host", "port", "vehicle", "spawn_index"),
        "carsim": ("sim_path", "repo_path", "remote", "chrono_python", "export_names", "units"),
        "ident": None}   # None: the whole block

TEMPLATES = [
    {"id": "blank", "name": "空白工程",
     "desc": "全部是默认设置：CarSim 联合仿真、示例算法，不开测试场景和数据采集。",
     "overlay": {}},
    {"id": "kmppi_chrono", "name": "KMPPI 路径跟踪（Chrono 宝马，Town04 高速）",
     "desc": "算法 controllers/kmppi/controller.py，动力学用服务器上的 Chrono 宝马 E90（初始车速 20 m/s），"
             "在 Town04 高速起点沿车道中心线跑 40 s；仿真步长由算法设成 0.05 s。有 CarSim 时在“CarSim 动力学”页换回 CarSim。",
     "town04_start": True,
     "overlay": {"run": {"driver": "custom", "controller": {"path": "controllers/kmppi/controller.py", "entry": "Controller"}},
                 "drive": {"dynamics": "cosim"},
                 "carsim": {"mock": False, "chrono": True, "chrono_init_speed": 20.0},
                 "sync": {"duration": 40.0}}},
    {"id": "work_zone", "name": "施工封道避障测试（Town04 高速）",
     "desc": "保留当前的算法文件；在 Town04 高速起点前方 250 m 用锥桶封闭本车道，撞上就停止运行，每次跑 40 s。"
             "封道和动态目标可在“测试场景”页再改。",
     "town04_start": True, "keep_algorithm": True,
     "overlay": {"drive": {"dynamics": "cosim"},
                 "scenario": {"enabled": True, "actors": [],
                              "closures": [{"distance_m": 250, "lane": 0, "taper_m": 40, "length_m": 100, "kind": "cones"}]},
                 "scene": {"collision": "stop"},
                 "sync": {"duration": 40.0}}},
    {"id": "dataset", "name": "数据集采集（nuScenes 传感器，CARLA 自动驾驶）",
     "desc": "CARLA 物理 + 自动驾驶，nuScenes 风格传感器（6 环视 + 32 线激光雷达 + 5 毫米波雷达），固定世界：Town10、"
             "30 辆车 20 个行人（种子 1）；10 Hz 采集，最多 200 帧。点“运行”开始采集。",
     "overlay": {"drive": {"dynamics": "carla", "carla_driver": "autopilot"},
                 "rig": {"preset": "nuscenes", "sensors": [], "frame": "carsim"},
                 "world": {"fixed": True, "map": "Town10HD_Opt", "weather": {},
                           "traffic": {"vehicles": 30, "walkers": 20, "seed": 1}},
                 "sync": {"frame_dt": 0.05},
                 "collect": {"enabled": True, "sample_period": 0.1, "max_frames": 200}}},
]


def listing():
    return [{"id": t["id"], "name": t["name"], "desc": t["desc"], "town04_start": bool(t.get("town04_start"))}
            for t in TEMPLATES]


def config(tid, current=None):
    """The template's config: the defaults, the template over them, this machine's settings of
    current (the config open now) kept."""
    t = next((t for t in TEMPLATES if t["id"] == tid), None)
    if t is None:
        raise ValueError("没有这个模板：%s" % tid)
    d = settings.load_dict(override=t["overlay"])
    cur = current if isinstance(current, dict) else {}
    for block, keys in KEEP.items():
        src = cur.get(block)
        if not isinstance(src, dict):
            continue
        if keys is None:
            d[block] = copy.deepcopy(src)
            continue
        for k in keys:
            if k in src:
                d[block][k] = copy.deepcopy(src[k])
    if t.get("keep_algorithm") and isinstance(cur.get("run"), dict):
        for k in ("driver", "controller", "params"):
            if k in cur["run"]:
                d["run"][k] = copy.deepcopy(cur["run"][k])
    return d
