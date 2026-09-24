"""Close-up render of the co-simulated car in a tight left turn.

Front-quarter camera, so the Ackermann steer of both front wheels and the
body roll are visible. Writes steer_left.png.
"""
import os
import sys

import carla

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import config as cfg  # noqa: E402
from bridge import CarlaVehicleSync  # noqa: E402
from mock_carsim import MockCarSimEnv  # noqa: E402

c = carla.Client("localhost", 2000); c.set_timeout(30)
w = c.get_world()
orig = w.get_settings(); s = w.get_settings(); s.synchronous_mode = True; s.fixed_delta_seconds = 0.02; w.apply_settings(s)
anchor = w.get_map().get_spawn_points()[0]
v = w.spawn_actor(w.get_blueprint_library().find("vehicle.tesla.model3"), anchor)
cam = None
try:
    for _ in range(30):
        w.tick()
    sync = CarlaVehicleSync(w, v, anchor)
    print("ref point:", sync.ref_local.round(3).tolist())
    env = MockCarSimEnv(cfg.EXPORT_NAMES); obs = env.reset()
    for _ in range(150):  # 3 s: creep forward with full left lock
        obs, *_ = env.control_step([0.15, 0.0, 480.0], 20)
        sync.sync(obs, env.t_current, 0.02); w.tick()
    bp = w.get_blueprint_library().find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", "960"); bp.set_attribute("image_size_y", "540"); bp.set_attribute("fov", "70")
    cam = w.spawn_actor(bp, carla.Transform(carla.Location(x=4.2, y=-3.2, z=1.3), carla.Rotation(yaw=145, pitch=-12)), attach_to=v)
    shots = []
    cam.listen(shots.append)
    for _ in range(3):
        obs, *_ = env.control_step([0.15, 0.0, 480.0], 20)
        st = sync.sync(obs, env.t_current, 0.02); w.tick()
    shots[-1].save_to_disk("steer_left.png")
    print("wheel steer FL/FR (CARLA deg):", [round(a, 1) for a in st.wheel_steer[:2]],
          "roll:", round(st.transform.rotation.roll, 2), "speed m/s:", round(env.v, 2))
finally:
    if cam: cam.stop(); cam.destroy()
    v.destroy(); w.apply_settings(orig)
