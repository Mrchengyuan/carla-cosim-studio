"""Driving modes shared by CARLA-physics runs and CarSim co-simulation.

Every driver returns a normalised command (throttle 0..1, brake 0..1,
steer -1..1 with + = right, CARLA convention). to_carsim() maps it to the
CarSim import vector [throttle, brake, steering_wheel_deg (+ = left)].

RouteFollower plans a route on the OpenDRIVE road graph (CARLA's
GlobalRoutePlanner) and tracks it with pure pursuit + a PI speed loop. It
only needs the vehicle pose and a speed value, so it works on stock CARLA
with CarSim (where vehicle.get_velocity() reads 0) as well.
"""

import math
import time
import os
import random
import sys

import carla


def _import_route_planner(pythonapi_dir):
    if pythonapi_dir and pythonapi_dir not in sys.path:
        sys.path.insert(0, pythonapi_dir)
    from agents.navigation.global_route_planner import GlobalRoutePlanner
    return GlobalRoutePlanner


def find_pythonapi_dir():
    """Folder that contains CARLA's 'agents' package (not part of the wheel)."""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [os.environ.get("CARLA_PYTHONAPI", ""),
                  os.path.join(os.environ.get("CARLA_ROOT", ""), "PythonAPI", "carla"),
                  os.path.join(here, "..", "CARLA_0.9.16", "PythonAPI", "carla"),
                  os.path.join(here, "..", "CARLA_0.9.16", "WindowsNoEditor", "PythonAPI", "carla"),
                  os.path.join(here, "..", "carla_src", "PythonAPI", "carla")]
    for c in candidates:
        if c and os.path.isdir(os.path.join(c, "agents", "navigation")):
            return os.path.abspath(c)
    return ""


class Command:
    __slots__ = ("throttle", "brake", "steer")

    def __init__(self, throttle=0.0, brake=0.0, steer=0.0):
        self.throttle, self.brake, self.steer = throttle, brake, steer

    def to_carla(self):
        return carla.VehicleControl(throttle=float(self.throttle), brake=float(self.brake),
                                    steer=float(max(-1.0, min(1.0, self.steer))))

    def to_carsim(self, steering_wheel_max_deg=540.0, brake_scale=1.0):
        # CarSim steering wheel angle is + to the left; CARLA steer + is right.
        return [float(self.throttle), float(self.brake) * brake_scale, -float(self.steer) * steering_wheel_max_deg]


class ManualDriver:
    """Holds the latest command sent by the GUI keyboard."""

    def __init__(self):
        self.cmd = Command()
        self.stamp = 0.0

    def set(self, throttle, brake, steer, stamp):
        self.cmd = Command(throttle, brake, steer)
        self.stamp = stamp

    def step(self, vehicle, speed_ms, dt):
        # Dead man's switch: the GUI sends the keys 20 times a second. If that
        # stops (GUI stalled or gone), release the throttle instead of driving on.
        if self.stamp and time.time() - self.stamp > 0.5:
            return Command(0.0, 0.3, 0.0)
        return self.cmd


class RouteFollower:
    def __init__(self, world, vehicle, target_speed_kmh=40.0, destination=None, max_steer_deg=70.0,
                 wheelbase=2.9, loop=True, pythonapi_dir=None, seed=0):
        GRP = _import_route_planner(pythonapi_dir or find_pythonapi_dir())
        self.world, self.vehicle, self.map = world, vehicle, world.get_map()
        self.grp = GRP(self.map, 2.0)
        self.target = target_speed_kmh / 3.6
        self.max_steer = math.radians(max_steer_deg)
        self.L = wheelbase
        self.loop = loop
        self.rng = random.Random(seed)
        self.destination = destination
        self.route = []
        self.i = 0
        self.integ = 0.0
        self.replans = 0
        self._plan()

    def _plan(self):
        start = self.vehicle.get_location()
        pts = self.map.get_spawn_points()
        dest = self.destination
        if dest is None or self.replans > 0:
            # Random far-away destination; loop forever when the route ends.
            far = [p.location for p in pts if p.location.distance(start) > 150.0] or [p.location for p in pts]
            dest = self.rng.choice(far)
        route = []
        for wp, _ in self.grp.trace_route(start, dest):
            loc = wp.transform.location
            # The planner repeats points at lane-segment joins; drop them.
            if not route or loc.distance(route[-1]) > 0.5:
                route.append(loc)
        self.route = route
        self.i = 0
        self.replans += 1

    def remaining(self):
        return max(0, len(self.route) - self.i)

    def step(self, vehicle, speed_ms, dt):
        tf = vehicle.get_transform()
        loc = tf.location
        # Track progress: nearest route point within a window ahead of the
        # current index (never jumps back, never skips to a far part of a loop).
        window = self.route[self.i:self.i + 40]
        if window:
            self.i += min(range(len(window)), key=lambda k: loc.distance(window[k]))
        if self.remaining() < 5:
            if not self.loop:
                return Command(0.0, 1.0, 0.0)
            self._plan()
        lookahead = max(5.0, 0.8 * speed_ms + 3.0)
        j = self.i
        while j + 1 < len(self.route) and loc.distance(self.route[j]) < lookahead:
            j += 1
        tgt = self.route[j]
        # Pure pursuit in the vehicle frame (CARLA: y to the right).
        yaw = math.radians(tf.rotation.yaw)
        dx, dy = tgt.x - loc.x, tgt.y - loc.y
        x_v = math.cos(yaw) * dx + math.sin(yaw) * dy
        y_v = -math.sin(yaw) * dx + math.cos(yaw) * dy
        ld2 = max(1.0, x_v * x_v + y_v * y_v)
        delta = math.atan2(2.0 * self.L * y_v, ld2)       # + = right
        steer = max(-1.0, min(1.0, delta / self.max_steer))
        # Slow down for sharp curves, then PI on speed.
        target = self.target * (1.0 - 0.5 * min(1.0, abs(delta) / 0.5))
        err = target - speed_ms
        self.integ = max(-5.0, min(5.0, self.integ + err * dt))
        u = 0.35 * err + 0.05 * self.integ
        if u >= 0:
            return Command(min(1.0, u), 0.0, steer)
        return Command(0.0, min(1.0, -u * 0.5), steer)
