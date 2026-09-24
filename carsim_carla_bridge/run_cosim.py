"""CarSim + CARLA lock-step co-simulation (command line).

Each CARLA frame (fixed_delta_seconds = frame_dt):
  1. controller computes [throttle, brake, steering_wheel_deg]
  2. CarSim integrates frame_dt / t_step solver steps (control_step)
  3. bridge pushes the resulting state to the CARLA vehicle
  4. world.tick() renders the frame / runs the sensors
CARLA runs in synchronous mode, so both simulators share one clock.

Settings come from config.py, then --config JSON (the file the GUI edits),
then the command-line flags below.

Examples
  python run_cosim.py --mock --duration 20 --record out/        # no CarSim needed
  python run_cosim.py --config cosim.json                        # GUI-saved settings
  python run_cosim.py --sim C:/CarSim/simfile.sim --carsim-repo ../python_carsim_env
  python run_cosim.py --sim C:/CarSim/simfile.sim --driver pid   # your SimplePathFollower
"""

import argparse

import carla

import settings as st
from session import CoSimSession


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
    ap.add_argument("--driver", choices=("demo", "pid"))
    ap.add_argument("--record", help="save chase-camera frames here")
    ap.add_argument("--log")
    ap.add_argument("--no-external-api", action="store_true", help="force the stock-CARLA fallback")
    args = ap.parse_args()

    o = {"carla": {}, "carsim": {}, "sync": {}, "run": {}}
    for key, sect, name in (("host", "carla", "host"), ("port", "carla", "port"),
                            ("spawn_index", "carla", "spawn_index"), ("vehicle", "carla", "vehicle"),
                            ("sim", "carsim", "sim_path"), ("carsim_repo", "carsim", "repo_path"),
                            ("frame_dt", "sync", "frame_dt"), ("duration", "sync", "duration"),
                            ("driver", "run", "driver"), ("record", "run", "record_dir"),
                            ("log", "run", "log_path")):
        if getattr(args, key) is not None:
            o[sect][name] = getattr(args, key)
    if args.mock:
        o["carsim"]["mock"] = True
    if args.no_external_api:
        o["sync"]["use_external_api"] = False
    d = st.load_dict(args.config, o)
    if not d["carsim"]["mock"] and not d["carsim"]["sim_path"]:
        ap.error("--sim (or carsim.sim_path in --config) is required unless --mock is given")

    client = carla.Client(d["carla"]["host"], d["carla"]["port"])
    client.set_timeout(30.0)
    world = client.get_world()

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
        info = session.start()
        print("external-dynamics API:", "yes (modified CARLA)" if info["external_api"] else "no (stock fallback)")
        print("CarSim reference point in vehicle frame [m]:", info["reference_point"])
        if info["clock_warning"]:
            print("warning: frame_dt is not a multiple of CarSim t_step; clocks will drift")
        print("CarSim t_step=%g s, %d solver steps per CARLA frame" % (info["t_step"], info["inner_steps"]))
        tel = None
        while not session.done:
            tel = session.step()
        if tel:
            print("ran %.1f s simulated (%.2fx real time)" % (tel["t"], tel["rt_factor"]))
    finally:
        session.stop(release_vehicle=False)
        vehicle.destroy()
        s = world.get_settings()
        s.synchronous_mode, s.fixed_delta_seconds = False, None
        world.apply_settings(s)


if __name__ == "__main__":
    main()
