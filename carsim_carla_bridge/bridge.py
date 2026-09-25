"""CarSim -> CARLA vehicle synchronisation.

CarSim is the single source of truth for vehicle dynamics. Every frame the
CarSim export vector is converted to CARLA conventions and pushed to the
CARLA vehicle: body pose, linear/angular velocity, per-wheel road steer angle
and per-wheel spin angle. With the modified CARLA (apply_external_state) this
is one atomic RPC per frame; on stock CARLA it falls back to set_transform +
per-wheel animation calls, which looks the same but reports zero velocity.
"""

import math
from dataclasses import dataclass, field

import numpy as np
import carla

import config as cfg
from coords import AnchorFrame, euler_from_rot, iso_to_carla_angular, iso_to_carla_vector

KMH_TO_MS = 1.0 / 3.6
RPM_TO_DEGS = 360.0 / 60.0

# CarSim wheel suffix -> CARLA wheel index (FL, FR, RL, RR)
WHEEL_SUFFIXES = ("L1", "R1", "L2", "R2")
CARLA_WHEELS = (
    carla.VehicleWheelLocation.FL_Wheel,
    carla.VehicleWheelLocation.FR_Wheel,
    carla.VehicleWheelLocation.BL_Wheel,
    carla.VehicleWheelLocation.BR_Wheel,
)


class CarSimExports:
    """Name-based access to the CarSim export vector, in SI + degrees."""

    def __init__(self, names, units=None):
        self.index = {n: i for i, n in enumerate(names)}
        units = units or cfg.UNITS
        self._angle = 1.0 if units["angle"] == "deg" else math.degrees(1.0)
        self._speed = KMH_TO_MS if units["speed"] == "km/h" else 1.0
        self._rate = 1.0 if units["rate"] == "deg/s" else math.degrees(1.0)
        self._spin = RPM_TO_DEGS if units["wheel_spin"] == "rpm" else math.degrees(1.0)
        self._jounce = 1e-3 if units.get("jounce", "mm") == "mm" else 1.0
        missing = [n for n in ("Xo", "Yo", "Zo", "Yaw", "Pitch", "Roll", "Steer_L1", "Steer_R1")
                   if n not in self.index]
        if missing:
            raise ValueError("EXPORT_NAMES is missing required CarSim variables: %s" % missing)

    def has(self, name):
        return name in self.index

    def raw(self, obs, name, default=0.0):
        i = self.index.get(name)
        return float(obs[i]) if i is not None else default

    def angle(self, obs, name):          # -> deg
        return self.raw(obs, name) * self._angle

    def speed(self, obs, name):          # -> m/s
        return self.raw(obs, name) * self._speed

    def rate(self, obs, name):           # -> deg/s
        return self.raw(obs, name) * self._rate

    def spin(self, obs, name):           # -> deg/s
        return self.raw(obs, name) * self._spin

    def jounce(self, obs, name):         # -> m, + = compression
        return self.raw(obs, name) * self._jounce


@dataclass
class SyncedState:
    transform: carla.Transform
    velocity: carla.Vector3D
    angular_velocity: carla.Vector3D
    wheel_steer: list = field(default_factory=list)
    wheel_rotation: list = field(default_factory=list)
    wheel_suspension: list = field(default_factory=list)
    throttle: float = 0.0
    steer: float = 0.0
    brake: float = 0.0
    gear: int = 0


class CarlaVehicleSync:
    """Drives one CARLA vehicle from CarSim exports."""

    def __init__(self, world, vehicle, anchor, export_names=None, use_external_api=None, settings=None):
        """settings: namespace with config.py attribute names (see
        settings.to_bridge_cfg); defaults to config.py itself."""
        self.cfg = settings or cfg
        self.world = world
        self.vehicle = vehicle
        self._map = world.get_map()
        self.ex = CarSimExports(export_names or self.cfg.EXPORT_NAMES, self.cfg.UNITS)
        a = anchor
        self.anchor = AnchorFrame((a.location.x, a.location.y, a.location.z),
                                  a.rotation.yaw, a.rotation.pitch, a.rotation.roll)
        self.ref_local = self._reference_point_local()
        self.wheel_radius_m = self._wheel_radii()
        self.wheel_angle = [0.0] * 4
        self._prev = None  # (time, position, R) for finite differences

        auto = use_external_api is None
        self.external_api = hasattr(vehicle, "apply_external_state") if auto else bool(use_external_api)
        self.server_api = None  # does the server have the API: True / False / not tried
        if self.external_api:
            try:
                vehicle.enable_external_dynamics()
                self.server_api = True
            except RuntimeError as e:
                # A patched client talks to an original server: the call is unknown there.
                if not auto:
                    raise RuntimeError("这个 CARLA 服务器不是改版，不支持外部动力学接口；"
                                       "请把“CARLA 接口”设为“自动”或“强制原版兼容”（%s）" % e)
                self.external_api, self.server_api = False, False
        if not self.external_api:
            vehicle.set_simulate_physics(False)

    # ------------------------------------------------------------------ setup
    def _reference_point_local(self):
        ref = self.cfg.CARSIM_REFERENCE_POINT
        if ref != "front_axle":
            return np.asarray(ref, dtype=float)
        # Wheel positions come back in world cm; bring them to vehicle-local m.
        inv = np.array(self.vehicle.get_transform().get_inverse_matrix())
        wheels = self.vehicle.get_physics_control().wheels
        local = [inv @ np.array([w.position.x / 100.0, w.position.y / 100.0, w.position.z / 100.0, 1.0])
                 for w in wheels[:2]]
        front_x = (local[0][0] + local[1][0]) / 2.0
        # Ground level from the bounding box bottom, which is pure geometry;
        # wheel heights depend on where the suspension happens to be.
        bb = self.vehicle.bounding_box
        # CarSim origin: front axle centerline, at ground level.
        return np.array([front_x, 0.0, bb.location.z - bb.extent.z])

    def _wheel_radii(self):
        return [w.radius / 100.0 for w in self.vehicle.get_physics_control().wheels[:4]]

    # ------------------------------------------------------------------ core
    def compute(self, obs, sim_time, dt):
        ex = self.ex
        p_iso = (ex.raw(obs, "Xo"), ex.raw(obs, "Yo"), ex.raw(obs, "Zo"))
        ref_world, R = self.anchor.pose_to_world(
            p_iso, ex.angle(obs, "Yaw"), ex.angle(obs, "Pitch"), ex.angle(obs, "Roll"))
        # CarSim reports the reference point; CARLA wants the actor origin.
        actor_pos = ref_world - R @ self.ref_local
        yaw, pitch, roll = euler_from_rot(R)

        if self.cfg.Z_MODE == "ground":
            # Road height from the OpenDRIVE map. A raycast would hit our own
            # car body, so do not use world.ground_projection() here.
            wp = self._map.get_waypoint(
                carla.Location(float(ref_world[0]), float(ref_world[1]), float(ref_world[2])))
            if wp is not None:
                # Put the CarSim reference point (ground level) on the road.
                actor_pos[2] = wp.transform.location.z - R[2] @ self.ref_local

        velocity, angular = self._velocities(obs, R, actor_pos, sim_time)

        # Road-wheel steer: CarSim + = left, CARLA + = right.
        steer = [-ex.angle(obs, "Steer_" + s) if ex.has("Steer_" + s) else 0.0 for s in WHEEL_SUFFIXES]

        # Wheel spin: integrate CarSim wheel speed (keeps slip visible);
        # fall back to rolling without slip.
        v_fwd = float(np.dot(velocity, R[:, 0]))
        for i, s in enumerate(WHEEL_SUFFIXES):
            if ex.has("AVy_" + s):
                w_degs = ex.spin(obs, "AVy_" + s)
            else:
                w_degs = math.degrees(v_fwd / self.wheel_radius_m[i])
            self.wheel_angle[i] = (self.wheel_angle[i] + self.cfg.WHEEL_SPIN_SIGN * w_degs * dt) % 360.0

        # Suspension travel: same sign as CarSim jounce and PhysX suspJounce
        # (+ = compression, wheel moves up towards the body).
        suspension = [ex.jounce(obs, "Jnc_" + s) for s in WHEEL_SUFFIXES] \
            if all(ex.has("Jnc_" + s) for s in WHEEL_SUFFIXES) else []

        gear = int(round(ex.raw(obs, "GearStat"))) if ex.has("GearStat") else 0
        return SyncedState(
            transform=carla.Transform(
                carla.Location(float(actor_pos[0]), float(actor_pos[1]), float(actor_pos[2])),
                carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)),
            velocity=carla.Vector3D(*map(float, velocity)),
            angular_velocity=carla.Vector3D(*map(float, angular)),
            wheel_steer=steer,
            wheel_rotation=list(self.wheel_angle),
            wheel_suspension=suspension,
            throttle=ex.raw(obs, "Throttle") if ex.has("Throttle") else 0.0,
            steer=max(-1.0, min(1.0, -ex.angle(obs, "Steer_SW") / self.cfg.STEERING_WHEEL_MAX_DEG))
            if ex.has("Steer_SW") else 0.0,
            gear=gear,
        )

    def _velocities(self, obs, R, actor_pos, t):
        ex = self.ex
        if all(ex.has(n) for n in ("Vx", "Vy")):
            # CarSim Vx/Vy are body-frame; vertical speed is small, use 0.
            v_body = iso_to_carla_vector((ex.speed(obs, "Vx"), ex.speed(obs, "Vy"), 0.0))
            velocity = R @ v_body
        elif self._prev is not None and t > self._prev[0]:
            velocity = (actor_pos - self._prev[1]) / (t - self._prev[0])
        else:
            velocity = np.zeros(3)

        if all(ex.has(n) for n in ("AVx", "AVy", "AVz")):
            w_body = iso_to_carla_angular((ex.rate(obs, "AVx"), ex.rate(obs, "AVy"), ex.rate(obs, "AVz")))
            angular = R @ w_body
        elif self._prev is not None and t > self._prev[0]:
            # Small-angle rotation vector from R_prev -> R, Unreal components.
            dR = R @ self._prev[2].T
            rotvec = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) / 2.0
            angular = np.degrees(rotvec) / (t - self._prev[0])
        else:
            angular = np.zeros(3)

        self._prev = (t, actor_pos.copy(), R.copy())
        return velocity, angular

    def apply(self, state):
        v = self.vehicle
        if self.external_api:
            v.apply_external_state(
                state.transform, state.velocity, state.angular_velocity,
                state.wheel_steer, state.wheel_rotation, state.wheel_suspension,
                state.throttle, state.steer, state.brake, state.gear)
            return
        # Stock CARLA fallback: same visuals except suspension travel (no
        # Python API for wheel height), and get_velocity()/IMU read zero.
        v.set_transform(state.transform)
        for loc, ang in zip(CARLA_WHEELS, state.wheel_steer):
            v.set_wheel_steer_direction(loc, ang)
        for loc, ang in zip(CARLA_WHEELS, state.wheel_rotation):
            v.set_wheel_pitch_angle(loc, ang)

    def sync(self, obs, sim_time, dt):
        state = self.compute(obs, sim_time, dt)
        self.apply(state)
        return state

    def release(self):
        """Hand the vehicle back to CARLA's own physics."""
        if self.external_api:
            self.vehicle.restore_physx_physics()
        else:
            self.vehicle.set_simulate_physics(True)
