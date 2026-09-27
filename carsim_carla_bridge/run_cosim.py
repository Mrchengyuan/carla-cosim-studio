"""CarSim + CARLA lock-step co-simulation (command line).

Each CARLA frame (fixed_delta_seconds = frame_dt):
  1. controller computes [throttle, brake, steering_wheel_deg]
  2. CarSim integrates frame_dt / t_step solver steps (control_step; frame_dt
     is aligned to a whole number of t_step)
  3. bridge pushes the resulting state to the CARLA vehicle
  4. world.tick() renders the frame / runs the sensors
CARLA runs in synchronous mode, so both simulators share one clock.

Settings come from config.py, then --config JSON (the file the GUI edits),
then the command-line flags below. Only the CarSim co-simulation runs here: a
config with drive.dynamics = "carla" (CARLA physics) is refused.

Examples
  python run_cosim.py --mock --duration 20                       # no CarSim needed
  python run_cosim.py --config cosim.json                        # GUI-saved settings
  python run_cosim.py --sim C:/CarSim/simfile.sim --carsim-repo ../python_carsim_env --duration 0
  python run_cosim.py --sim C:/CarSim/simfile.sim --controller my_ctrl.py --duration 0   # your control algorithm
Without --duration (or --config) a run lasts 20 s; --duration 0 runs until the .sim's end time.
"""

import argparse

import carla

import settings as st
from session import CoSimSession, check_run_config, check_run_files


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", help="JSON settings file (as saved by the GUI)")
    ap.add_argument("--host")
    ap.add_argument("--port", type=int)
    ap.add_argument("--mock", action="store_true", help="use MockCarSimEnv instead of CarSim")
    ap.add_argument("--sim", help="CarSim .sim file")
    ap.add_argument("--carsim-repo", help="path to python_carsim_env")
    ap.add_argument("--frame-dt", type=float, help="CARLA frame period [s]")
    ap.add_argument("--duration", type=float, help="simulated seconds")
    ap.add_argument("--spawn-index", type=int, help="CARLA spawn point used as CarSim origin")
    ap.add_argument("--vehicle")
    ap.add_argument("--driver", choices=("custom", "demo"))
    ap.add_argument("--controller", help="your control algorithm (.py), see controllers/example_controller.py")
    ap.add_argument("--log")
    ap.add_argument("--no-external-api", action="store_true", help="force the stock-CARLA fallback")
    args = ap.parse_args()

    o = {"carla": {}, "carsim": {}, "sync": {}, "run": {}}
    for key, sect, name in (("host", "carla", "host"), ("port", "carla", "port"),
                            ("spawn_index", "carla", "spawn_index"), ("vehicle", "carla", "vehicle"),
                            ("sim", "carsim", "sim_path"), ("carsim_repo", "carsim", "repo_path"),
                            ("frame_dt", "sync", "frame_dt"), ("duration", "sync", "duration"),
                            ("driver", "run", "driver"), ("log", "run", "log_path")):
        if getattr(args, key) is not None:
            o[sect][name] = getattr(args, key)
    if args.controller:
        o["run"]["controller"] = {"path": args.controller}
        o["run"]["driver"] = "custom"
    if args.mock:
        o["carsim"]["mock"] = True
    if args.no_external_api:
        o["sync"]["use_external_api"] = False
    if args.duration is None and not args.config:
        o["sync"]["duration"] = 20.0   # a CLI run without --duration is a short demo
    d = st.load_dict(args.config, o)
    if d["drive"]["dynamics"] != "cosim":
        ap.error("配置里 drive.dynamics = %r：命令行只支持 CarSim 联合仿真（\"cosim\"），"
                 "CARLA 物理请在界面里运行" % d["drive"]["dynamics"])
    if not d["carsim"]["mock"] and not d["carsim"]["sim_path"]:
        ap.error("--sim (or carsim.sim_path in --config) is required unless --mock is given")
    try:
        # The GUI backend's pre-flight, before CARLA is touched.
        check_run_config(d)
        check_run_files(d)
    except (ValueError, RuntimeError) as e:
        ap.error(str(e))

    client = carla.Client(d["carla"]["host"], d["carla"]["port"])
    client.set_timeout(30.0)
    world = client.get_world()

    original_settings = world.get_settings()
    anchor = world.get_map().get_spawn_points()[d["carla"]["spawn_index"]]
    vehicle = world.spawn_actor(world.get_blueprint_library().find(d["carla"]["vehicle"]), anchor)
    session = CoSimSession(world, vehicle, anchor, d)
    try:
        # Let the vehicle land under PhysX before the bridge reads its geometry.
        s = world.get_settings()
        s.synchronous_mode, s.fixed_delta_seconds = True, d["sync"]["frame_dt"]
        world.apply_settings(s)
        for _ in range(30):
            world.tick()
        req_dt = d["sync"]["frame_dt"]
        info = session.start()
        print("external-dynamics API:", "yes (modified CARLA)" if info["external_api"] else "no (stock fallback)")
        print("CarSim reference point in vehicle frame [m]:", info["reference_point"])
        if abs(info.get("frame_dt", req_dt) - req_dt) > 1e-9:
            print("frame_dt %g s -> %g s (a whole number of CarSim t_step)" % (req_dt, info["frame_dt"]))
            period, every = float(d["collect"].get("sample_period") or 0.0), st.sample_every(d)
            if period > 0 and abs(every * info["frame_dt"] - period) > 1e-9:
                print("sample period %g s -> %g s (every %d frames)" % (period, every * info["frame_dt"], every))
        print("CarSim t_step=%g s, %d solver steps per CARLA frame" % (info["t_step"], info["inner_steps"]))
        for w in info.get("warnings", []):
            print("warning:", w)
        for hit in info.get("collisions", []):  # touching at the start: counted once, here
            print("collision at t=%.2f s with %s (id %s)" % (info["t"], hit["model"], hit["id"]))
        if info.get("warning"):
            print("warning:", info["warning"])
        tel = None
        while not session.done:
            tel = session.step()
            for hit in tel.get("collisions", []):
                print("collision at t=%.2f s with %s (id %s)" % (tel["t"], hit["model"], hit["id"]))
            if tel.get("warning"):
                print("warning:", tel["warning"])
            for msg in tel.get("warnings", []):
                print("warning: t=%.2f s: %s" % (tel["t"], msg))
        if getattr(session, "end_reason", ""):
            print("stopped:", session.end_reason)
        if tel:
            print("ran %.1f s simulated (%.2fx real time)" % (tel["t"], tel["rt_factor"]))
    finally:
        try:
            session.stop(release_vehicle=False)
        finally:
            try:
                vehicle.destroy()
            finally:
                world.apply_settings(original_settings)


if __name__ == "__main__":
    main()
