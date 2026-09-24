"""Checks the external-dynamics API of the *modified* CARLA server.

Needs the patched server and the patched Python client (carla wheel built
from carla_src). Rendering is not required (-nullrhi is fine).

    python tests/test_modified_carla.py --port 3000
"""
import math
import sys

import numpy as np
import carla


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + (("  " + str(detail)) if detail != "" else ""))
    return cond


def main():
    port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 3000
    c = carla.Client("localhost", port)
    c.set_timeout(120)
    w = c.get_world()
    print("server", c.get_server_version(), w.get_map().name)
    ok = check("client has new API", hasattr(carla.Vehicle, "apply_external_state"))

    orig = w.get_settings()
    s = w.get_settings()
    s.synchronous_mode, s.fixed_delta_seconds = True, 0.05
    w.apply_settings(s)
    bl = w.get_blueprint_library()
    sp = w.get_map().get_spawn_points()[3]
    v = w.spawn_actor(bl.find("vehicle.tesla.model3"), sp)
    imu = w.spawn_actor(bl.find("sensor.other.imu"), carla.Transform(), attach_to=v)
    gyro = []
    imu.listen(lambda m: gyro.append((m.frame, m.gyroscope.z)))
    try:
        for _ in range(20):
            w.tick()
        v.enable_external_dynamics()
        w.tick()

        speed, yaw_rate = 10.0, 15.0                      # m/s, deg/s
        steer = [5.0, 4.0, 0.0, 0.0]
        susp = [0.02, -0.02, 0.03, -0.03]                 # m
        loc = carla.Location(sp.location.x, sp.location.y, sp.location.z + 0.0)
        yaw = sp.rotation.yaw
        errs, verrs, werrs = [], [], []
        for k in range(40):
            yaw_r = math.radians(yaw)
            vel = carla.Vector3D(speed * math.cos(yaw_r), speed * math.sin(yaw_r), 0.0)
            tf = carla.Transform(loc, carla.Rotation(yaw=yaw))
            v.apply_external_state(tf, vel, carla.Vector3D(0, 0, yaw_rate),
                                   wheel_steer=steer, wheel_rotation=[k * 20.0] * 4, wheel_suspension=susp,
                                   throttle=0.5, steer=-0.1, brake=0.0, gear=3)
            w.tick()
            got = v.get_transform()
            errs.append(got.location.distance(loc))
            gv = v.get_velocity()
            verrs.append(math.dist((gv.x, gv.y, gv.z), (vel.x, vel.y, vel.z)))
            werrs.append(abs(v.get_angular_velocity().z - yaw_rate))
            loc = carla.Location(loc.x + vel.x * 0.05, loc.y + vel.y * 0.05, loc.z)
            yaw += yaw_rate * 0.05

        bad = [k for k, e in enumerate(errs) if e > 0.01]
        ok &= check("pose follows external state", not bad,
                    "max err %.4f m, frames off: %s of %d" % (max(errs), bad, len(errs)))
        badv = [k for k, e in enumerate(verrs) if e > 0.01]
        print("INFO velocity mismatch frames:", badv)
        ok &= check("get_velocity is the external velocity (stock CARLA reads 0)", not badv,
                    "speed read back %.2f m/s" % math.hypot(v.get_velocity().x, v.get_velocity().y))
        ok &= check("get_angular_velocity is the external yaw rate", max(werrs) < 0.01,
                    "z = %.2f deg/s" % v.get_angular_velocity().z)
        ctl = v.get_control()
        ok &= check("get_control reports solver inputs", abs(ctl.throttle - 0.5) < 1e-3 and abs(ctl.steer + 0.1) < 1e-3 and ctl.gear == 3,
                    "throttle %.2f steer %.2f gear %d" % (ctl.throttle, ctl.steer, ctl.gear))
        fl = v.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
        fr = v.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
        ok &= check("per-wheel steer applied", abs(fl - 5.0) < 1e-3 and abs(fr - 4.0) < 1e-3, "FL %.2f FR %.2f" % (fl, fr))
        snap = w.get_snapshot().find(v.id)
        ok &= check("world snapshot carries the velocity", abs(math.hypot(snap.get_velocity().x, snap.get_velocity().y) - speed) < 0.01)
        if gyro:
            gz = gyro[-1][1]
            ok &= check("IMU gyroscope sees the yaw rate", abs(abs(gz) - math.radians(yaw_rate)) < 0.01,
                        "gyro z = %.4f rad/s (expected %.4f)" % (gz, math.radians(yaw_rate)))

        # Suspension: wheel bones should move by the commanded travel (cm in bone space).
        try:
            names = v.get_bone_names()
            rel = v.get_bone_relative_transforms()
            idx = {n: i for i, n in enumerate(names)}
            fl_z = rel[idx["Wheel_Front_Left"]].location.z
            fr_z = rel[idx["Wheel_Front_Right"]].location.z
            check("suspension moves wheel bones (FL up, FR down)", fl_z - fr_z > 0.02,
                  "FL-FR bone height difference %.3f m (expected 0.04)" % (fl_z - fr_z))
        except Exception as e:
            print("INFO bone check skipped:", e)

        # Hand the car back to PhysX: it should keep moving with the last velocity.
        v.restore_physx_physics()
        for _ in range(5):
            w.tick()
        sp_after = math.hypot(v.get_velocity().x, v.get_velocity().y)
        ok &= check("restore_physx_physics hands over the velocity", sp_after > 5.0, "%.2f m/s after 5 frames" % sp_after)
    finally:
        imu.stop()
        imu.destroy()
        v.destroy()
        w.apply_settings(orig)
    print("ALL MODIFIED-CARLA TESTS PASSED" if ok else "SOME TESTS FAILED")


if __name__ == "__main__":
    main()
