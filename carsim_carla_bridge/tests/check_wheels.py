"""Measure how CARLA's wheel animation reacts to steer / pitch commands.

Reads wheel bone transforms so the sign conventions are checked numerically
instead of by eye. Needs a running CARLA server.
"""
import math

import numpy as np
import carla

c = carla.Client("localhost", 2000)
c.set_timeout(30)
w = c.get_world()
s = w.get_settings(); s.synchronous_mode = True; s.fixed_delta_seconds = 0.05; w.apply_settings(s)
v = w.spawn_actor(w.get_blueprint_library().find("vehicle.tesla.model3"), w.get_map().get_spawn_points()[0])
try:
    for _ in range(20):
        w.tick()
    v.set_simulate_physics(False)
    w.tick()
    names = v.get_bone_names()
    wheel_bones = [i for i, n in enumerate(names) if "wheel" in n.lower()]
    print("wheel bones:", [names[i] for i in wheel_bones])

    def wheel_axes():
        tfs = v.get_bone_world_transforms()
        inv = np.array(v.get_transform().get_inverse_matrix())[:3, :3]
        out = {}
        for i in wheel_bones:
            R = inv @ np.array(tfs[i].get_matrix())[:3, :3]
            out[names[i]] = R
        return out

    base = wheel_axes()
    FL = carla.VehicleWheelLocation.FL_Wheel
    v.set_wheel_steer_direction(FL, 20.0)
    w.tick(); w.tick()
    after = wheel_axes()
    for n in base:
        d = after[n] @ base[n].T
        yaw = math.degrees(math.atan2(d[1, 0], d[0, 0]))
        if abs(yaw) > 0.5:
            print("steer +20 on FL -> bone %s yaw change %+.1f deg (CARLA + = nose right)" % (n, yaw))
    v.set_wheel_steer_direction(FL, 0.0)

    base = wheel_axes()
    v.set_wheel_pitch_angle(FL, 30.0)
    w.tick(); w.tick()
    after = wheel_axes()
    for n in base:
        d = after[n] @ base[n].T
        # rotation about the wheel axle (local y): top of the wheel moves fwd?
        top = d @ np.array([0, 0, 1.0])
        if np.linalg.norm(top - [0, 0, 1]) > 1e-3:
            print("pitch +30 on FL -> bone %s: wheel top moves %s (x>0 = forward = forward rolling)"
                  % (n, np.round(top, 3)))
finally:
    v.destroy()
    s.synchronous_mode = False; s.fixed_delta_seconds = None; w.apply_settings(s)
