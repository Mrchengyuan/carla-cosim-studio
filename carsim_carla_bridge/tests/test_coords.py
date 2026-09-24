"""Offline checks of coords.py against carla.Transform (no server needed).

    python -m pytest tests/test_coords.py   (or just: python tests/test_coords.py)
"""
import math
import os
import random
import sys

import numpy as np
import carla

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from coords import AnchorFrame, S, euler_from_rot, iso_to_carla_angles, rot_zyx  # noqa: E402


def carla_R(yaw, pitch, roll):
    m = np.array(carla.Transform(carla.Location(), carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)).get_matrix())
    return m[:3, :3]


def test_rot_matches_carla():
    rng = random.Random(0)
    for _ in range(500):
        y, p, r = rng.uniform(-180, 180), rng.uniform(-89, 89), rng.uniform(-180, 180)
        assert np.allclose(rot_zyx(y, p, r), carla_R(y, p, r), atol=1e-5)


def test_euler_roundtrip():
    rng = random.Random(1)
    for _ in range(500):
        y, p, r = rng.uniform(-179, 179), rng.uniform(-89, 89), rng.uniform(-179, 179)
        y2, p2, r2 = euler_from_rot(rot_zyx(y, p, r))
        assert np.allclose((y2, p2, r2), (y, p, r), atol=1e-6)


def iso_R(yaw, pitch, roll):
    """Right-handed ISO rotation Rz(yaw) Ry(pitch) Rx(roll), degrees."""
    y, p, r = map(math.radians, (yaw, pitch, roll))
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    return Rz @ Ry @ Rx


def test_iso_angles_equal_reflected_rotation():
    """The per-angle sign rule is exactly S R_iso S, for any attitude."""
    rng = random.Random(2)
    for _ in range(500):
        y, p, r = rng.uniform(-180, 180), rng.uniform(-80, 80), rng.uniform(-180, 180)
        assert np.allclose(rot_zyx(*iso_to_carla_angles(y, p, r)), S @ iso_R(y, p, r) @ S, atol=1e-9)


def test_physical_meaning():
    # ISO yaw +90 (nose to the LEFT) -> CARLA forward axis points to -y (left).
    fwd = rot_zyx(*iso_to_carla_angles(90, 0, 0))[:, 0]
    assert np.allclose(fwd, (0, -1, 0), atol=1e-9)
    # ISO pitch +10 (nose DOWN) -> forward axis has negative z.
    assert rot_zyx(*iso_to_carla_angles(0, 10, 0))[2, 0] < 0
    # ISO roll +10 (right side DOWN) -> CARLA right axis (+y) has negative z.
    assert rot_zyx(*iso_to_carla_angles(0, 0, 10))[2, 1] < 0


def test_anchor_places_origin_on_spawn():
    a = AnchorFrame((10.0, 20.0, 0.5), yaw_deg=90.0)
    p, R = a.pose_to_world((5.0, 0.0, 0.0), 0.0, 0.0, 0.0)
    assert np.allclose(p, (10.0, 25.0, 0.5))         # 5 m ahead along +90 deg heading
    p, _ = a.pose_to_world((0.0, 2.0, 0.0), 0.0, 0.0, 0.0)
    assert np.allclose(p, (12.0, 20.0, 0.5))         # 2 m to the LEFT of that heading


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("PASS", name)
