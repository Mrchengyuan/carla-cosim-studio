"""CarSim (ISO 8855) <-> CARLA (Unreal) coordinate conversion.

CarSim / ISO 8855 : right-handed, x forward, y LEFT,  z up.
                    Yaw   + = nose to the left  (counter-clockwise from above)
                    Pitch + = nose DOWN
                    Roll  + = right side down
CARLA / Unreal    : left-handed,  x forward, y RIGHT, z up.
                    Yaw   + = nose to the right (clockwise from above)
                    Pitch + = nose UP
                    Roll  + = right side down

The two frames differ by the reflection S = diag(1, -1, 1). Conjugating a
rotation by S keeps the Z-Y-X Euler order and gives, per angle:
    yaw_carla = -yaw_iso,  pitch_carla = -pitch_iso,  roll_carla = roll_iso
The signs were checked against carla.Transform.transform() on 0.9.16
(see tests/test_coords.py).

Angular velocity is a pseudo-vector, so it maps as  w_carla = -S @ w_iso.
Unreal/PhysX report angular velocity with the right-hand rule applied to
Unreal's own axis components, which is exactly what this produces
(yaw rate: w_z > 0 means yaw increasing, i.e. turning right).
"""

import math

import numpy as np

S = np.diag([1.0, -1.0, 1.0])


def rot_zyx(yaw_deg, pitch_deg, roll_deg):
    """Rotation matrix (body -> world) for CARLA Euler angles.

    Matches carla.Transform.get_matrix()[:3][:3] and Unreal's FRotator:
    R = Rz(yaw) @ Ry(-pitch) @ Rx(-roll) in right-hand-rule matrix terms,
    applied to Unreal axis components.
    """
    y, p, r = (math.radians(a) for a in (yaw_deg, pitch_deg, roll_deg))
    cy, sy = math.cos(y), math.sin(y)
    cp, sp = math.cos(p), math.sin(p)
    cr, sr = math.cos(r), math.sin(r)
    # Same closed form Unreal uses in FRotationTranslationMatrix.
    return np.array([
        [cp * cy, sr * sp * cy - cr * sy, -(cr * sp * cy + sr * sy)],
        [cp * sy, sr * sp * sy + cr * cy, cy * sr - cr * sp * sy],
        [sp,      -sr * cp,               cr * cp],
    ])


def euler_from_rot(m):
    """Inverse of rot_zyx: returns (yaw, pitch, roll) in degrees."""
    pitch = math.degrees(math.atan2(m[2, 0], math.hypot(m[0, 0], m[1, 0])))
    yaw = math.degrees(math.atan2(m[1, 0], m[0, 0]))
    roll = math.degrees(math.atan2(-m[2, 1], m[2, 2]))
    return yaw, pitch, roll


def iso_to_carla_angles(yaw_iso, pitch_iso, roll_iso):
    """CarSim attitude (deg) -> CARLA attitude (deg), same local frame."""
    return -yaw_iso, -pitch_iso, roll_iso


def iso_to_carla_vector(v_iso):
    """Polar vector (position, velocity) ISO -> CARLA components."""
    return S @ np.asarray(v_iso, dtype=float)


def iso_to_carla_angular(w_iso):
    """Pseudo-vector (angular velocity) ISO -> CARLA components."""
    return -S @ np.asarray(w_iso, dtype=float)


class AnchorFrame:
    """Places the CarSim global frame at a CARLA transform (the 'anchor').

    CarSim (0, 0, 0) with yaw 0 maps onto the anchor location and heading,
    so a CarSim run starting at the origin starts exactly on a CARLA spawn
    point.
    """

    def __init__(self, location_xyz, yaw_deg, pitch_deg=0.0, roll_deg=0.0):
        self.origin = np.asarray(location_xyz, dtype=float)
        self.R = rot_zyx(yaw_deg, pitch_deg, roll_deg)

    def pose_to_world(self, p_iso, yaw_iso, pitch_iso, roll_iso):
        """CarSim global pose -> CARLA world (position m, R body->world)."""
        p_local = iso_to_carla_vector(p_iso)
        R_local = rot_zyx(*iso_to_carla_angles(yaw_iso, pitch_iso, roll_iso))
        return self.origin + self.R @ p_local, self.R @ R_local

    def vector_to_world(self, v_iso_global):
        return self.R @ iso_to_carla_vector(v_iso_global)

    def angular_to_world(self, w_iso_global):
        return self.R @ iso_to_carla_angular(w_iso_global)
