"""不需要 CARLA：工程模板（templates.py，界面“文件 → 从模板新建”）。

四个模板各有名字和说明、id 不重复；每个模板的配置都是完整的（默认配置的每一项都在）、通过运行前的检查
（session.check_run_config，算法文件存在、封道和动态目标合法）；本机设置保留（CARLA 车型和出生点、CarSim 路径、
导出变量、车辆参数辨识的输入）；KMPPI 模板用 Chrono 宝马、施工封道模板保留当前算法、数据集模板开采集和固定世界；
没有的模板说清；后端命令走文件线程。

    python tests/test_offline_templates.py
"""
import copy
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)
os.chdir(ROOT)

import rig  # noqa: E402
import scenario  # noqa: E402
import session  # noqa: E402
import settings  # noqa: E402
import templates  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + (("  " + str(detail)) if detail != "" else ""), flush=True)
    if not cond:
        FAILS.append(name)


def keys(d, prefix=""):
    out = set()
    for k, v in d.items():
        out.add(prefix + k)
        if isinstance(v, dict) and v:
            out |= keys(v, prefix + k + ".")
    return out


def main():
    lst = templates.listing()
    ids = [t["id"] for t in lst]
    check("four templates, each with a name and a description, ids unique",
          len(lst) == 4 and len(set(ids)) == 4 and all(t["name"] and t["desc"] for t in lst), ids)
    current = settings.default_dict()
    current["carla"].update(vehicle="vehicle.lincoln.mkz_2020", spawn_index=7, port=3000)
    exports = list(reversed(current["carsim"]["export_names"]))   # this machine's .sim order
    current["carsim"].update(sim_path="C:/CarSim/my.sim", repo_path="D:/python_carsim_env", remote=True,
                             export_names=exports, mock=True)
    current["ident"].update(m=1500.0, path="x.json")
    current["run"]["controller"] = {"path": "controllers/example_controller.py", "entry": "Controller"}
    current["run"]["params"] = {"controllers/example_controller.py": {"TARGET_KMH": 30}}
    base_keys = keys(settings.default_dict())
    cfgs = {}
    for tid in ids:
        d = templates.config(tid, copy.deepcopy(current))
        cfgs[tid] = d
        missing = sorted(k for k in base_keys if k not in keys(d) and not k.startswith("world.weather"))
        try:
            dd = copy.deepcopy(d)
            session.check_run_config(dd)
            err = ""
        except Exception as e:  # noqa: BLE001
            err = str(e)
        ctl = d["run"]["controller"]["path"]
        check("%s: a complete config, passes the run checks, its algorithm file exists" % tid,
              not missing and not err and os.path.isfile(ctl), (missing[:3], err, ctl))
        scenario.check_closures(d["scenario"]["closures"])
        scenario.check_actors(d["scenario"]["actors"])
        kept = (d["carla"]["vehicle"] == "vehicle.lincoln.mkz_2020" and d["carla"]["spawn_index"] == 7 and d["carla"]["port"] == 3000
                and d["carsim"]["sim_path"] == "C:/CarSim/my.sim" and d["carsim"]["repo_path"] == "D:/python_carsim_env"
                and d["carsim"]["remote"] is True and d["carsim"]["export_names"] == exports
                and d["ident"]["m"] == 1500.0)
        check("%s: this machine's settings kept (vehicle, spawn point, CarSim paths, exports, 车辆参数辨识)" % tid, kept)
    b = cfgs["blank"]
    check("blank: the defaults (the example algorithm, no mock, no Chrono, no test scene, no collection)",
          b["run"]["controller"]["path"] == settings.default_dict()["run"]["controller"]["path"] and not b["carsim"]["mock"]
          and not b["carsim"]["chrono"] and not b["scenario"]["enabled"] and not b["collect"]["enabled"])
    k = cfgs["kmppi_chrono"]
    check("kmppi_chrono: KMPPI on the Chrono BMW at 20 m/s, 40 s, the Town04 start",
          k["run"]["controller"]["path"] == "controllers/kmppi/controller.py" and k["carsim"]["chrono"] and not k["carsim"]["mock"]
          and k["carsim"]["chrono_init_speed"] == 20.0 and k["sync"]["duration"] == 40.0 and k["drive"]["dynamics"] == "cosim"
          and next(t for t in lst if t["id"] == "kmppi_chrono")["town04_start"])
    w = cfgs["work_zone"]
    check("work_zone: the current algorithm and its params kept, a cone closure 250 m ahead, a collision stops the run",
          w["run"]["controller"]["path"] == "controllers/example_controller.py"
          and w["run"]["params"] == {"controllers/example_controller.py": {"TARGET_KMH": 30}} and w["scenario"]["enabled"]
          and w["scenario"]["closures"][0]["distance_m"] == 250 and w["scene"]["collision"] == "stop")
    ds = cfgs["dataset"]
    check("dataset: CARLA physics + autopilot, nuScenes rig, a fixed world with traffic, 10 Hz collection",
          ds["drive"]["dynamics"] == "carla" and ds["drive"]["carla_driver"] == "autopilot" and ds["rig"]["preset"] in rig.PRESETS
          and ds["rig"]["preset"] == "nuscenes" and ds["world"]["fixed"] and ds["world"]["traffic"]["vehicles"] == 30
          and ds["collect"]["enabled"] and settings.sample_every(ds) == 2)
    check("the defaults are left alone (a template is a copy)", settings.default_dict() == settings.load_dict())
    try:
        templates.config("nope")
        err = ""
    except ValueError as e:
        err = str(e)
    check("an unknown template: said", "没有这个模板" in err, err)
    check("without a current config: the template alone", templates.config("kmppi_chrono")["carla"]["spawn_index"] == 0)
    import backend_server
    check("the backend commands run on the file thread", {"templates_list", "template_config"} <= backend_server.IO_CMDS
          and hasattr(backend_server.Backend, "cmd_template_config"))
    print("FAILED: %s" % FAILS if FAILS else "ALL TEMPLATE TESTS PASSED")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
