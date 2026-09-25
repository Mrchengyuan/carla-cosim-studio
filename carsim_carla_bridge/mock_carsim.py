"""Stand-in for CarSimEnv when no CarSim license is available (e.g. on Linux).

Same interface as python_carsim_env.carsim_env.CarSimEnv and produces the
same export variables, in CarSim conventions (ISO axes, deg, km/h, rpm), so
the bridge can be exercised end to end. The dynamics are a kinematic bicycle
with Ackermann steering and quasi-static roll/pitch - enough to check signs,
not a vehicle model.
"""

import math

import numpy as np


class MockCarSimEnv:
    WHEELBASE = 2.9      # m
    TRACK = 1.6          # m
    WHEEL_RADIUS = 0.35  # m
    STEER_RATIO = 16.0   # steering wheel deg / road wheel deg

    def __init__(self, export_names, t_step=0.001, t_stop=60.0):
        self.export_names = list(export_names)
        self.config = {"t_start": 0.0, "t_stop": t_stop, "t_step": t_step,
                       "n_import": 3, "n_export": len(self.export_names)}
        self.t_step = t_step
        self.t_stop = t_stop

    def reset(self):
        self.t_current = 0.0
        self.x = self.y = self.psi = 0.0
        self.v = 0.0
        self.ax = self.ay = 0.0
        self.roll = self.pitch = 0.0
        self.prev_roll = self.prev_pitch = 0.0
        self.delta_l = self.delta_r = 0.0
        self.steer_sw = self.throttle = 0.0
        self.done = False
        return self._exports()

    def own_driver(self):
        """Stands in for CarSim's built-in driver model (action None): hold
        ~40 km/h and weave gently, like a CarSim procedure would."""
        throttle = max(0.0, min(1.0, 0.1 * (40.0 / 3.6 - self.v)))
        steer_sw = 0.0 if self.t_current < 4.0 else 45.0 * math.sin(2 * math.pi * 0.1 * (self.t_current - 4.0))
        return [throttle, 0.0, steer_sw]

    def control_step(self, action, inner_steps):
        if action is None:
            action = self.own_driver()
        throttle, brake, steer_sw = (list(action) + [0.0, 0.0, 0.0])[:3]
        self.throttle, self.steer_sw = throttle, steer_sw
        delta = math.radians(steer_sw / self.STEER_RATIO)  # + = left (ISO)
        dt = self.t_step
        for _ in range(inner_steps):
            a = 3.0 * throttle - 8.0 * brake - 0.02 * self.v
            if self.v <= 0.0 and a < 0.0:
                a = 0.0
            self.v = max(0.0, self.v + a * dt)
            r = self.v * math.tan(delta) / self.WHEELBASE
            self.x += self.v * math.cos(self.psi) * dt
            self.y += self.v * math.sin(self.psi) * dt
            self.psi += r * dt
            self.ax, self.ay = a, self.v * r
            self.prev_roll, self.prev_pitch = self.roll, self.pitch
            self.roll = 0.6 * self.ay      # deg; left turn -> body rolls right (+)
            self.pitch = -0.4 * self.ax    # deg; braking -> nose down (+)
            self.t_current += dt
        # Ackermann: inner wheel steers more.
        if abs(delta) > 1e-6:
            R = self.WHEELBASE / math.tan(abs(delta))
            inner = math.atan(self.WHEELBASE / (R - self.TRACK / 2))
            outer = math.atan(self.WHEELBASE / (R + self.TRACK / 2))
            s = math.copysign(1.0, delta)
            self.delta_l, self.delta_r = (s * inner, s * outer) if delta > 0 else (s * outer, s * inner)
        else:
            self.delta_l = self.delta_r = 0.0
        self.done = self.t_current >= self.t_stop
        return self._exports(), 0.0, self.done, {"return_code": 0}

    def close(self):
        self.done = True

    def _exports(self):
        spin_rpm = self.v / self.WHEEL_RADIUS * 60.0 / (2 * math.pi)
        yaw_rate = self.v * math.tan(math.radians(self.steer_sw / self.STEER_RATIO)) / self.WHEELBASE
        vals = {
            "Xo": self.x, "Yo": self.y, "Zo": 0.0,
            "Yaw": math.degrees(self.psi), "Pitch": self.pitch, "Roll": self.roll,
            "Vx": self.v * 3.6, "Vy": 0.0,
            "AVx": 0.0, "AVy": 0.0, "AVz": math.degrees(yaw_rate),
            "Steer_SW": self.steer_sw,
            "Steer_L1": math.degrees(self.delta_l), "Steer_R1": math.degrees(self.delta_r),
            "Steer_L2": 0.0, "Steer_R2": 0.0,
            "AVy_L1": spin_rpm, "AVy_R1": spin_rpm, "AVy_L2": spin_rpm, "AVy_R2": spin_rpm,
            "Throttle": self.throttle, "GearStat": 1.0 if self.v > 0 else 0.0,
            # Quasi-static jounce, mm: outer wheels compress in a turn
            # (left turn: ay > 0, right side is outer), front compresses on braking.
            "Jnc_L1": -4.0 * self.ay - 3.0 * self.ax, "Jnc_R1": 4.0 * self.ay - 3.0 * self.ax,
            "Jnc_L2": -4.0 * self.ay + 3.0 * self.ax, "Jnc_R2": 4.0 * self.ay + 3.0 * self.ax,
        }
        return tuple(float(vals.get(n, 0.0)) for n in self.export_names)
