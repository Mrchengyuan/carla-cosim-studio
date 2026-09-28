"""PyChrono BMW E90 in place of CarSim (no CarSim here: the user's
~/Desktop/rl/chrono_bmw car, as kmppi_chrono uses it).

Same interface as python_carsim_env's CarSimEnv (config, reset,
control_step, close, t_current), so the platform runs it like CarSim
(carsim_service.py --chrono): the exports in CarSim's names, frames and
units (the CarSim page's), and the imports of the KMPPI project's plant:

    imports  [ax (m/s^2), delta (rad, front road wheel, left +)]
             through chrono_plant.py's command adapter unchanged: torque
             on the four drive shafts, the steering calibration table
             (bmw_e90_identified.json, from kmppi_chrono/identify_plant.py)
    exports  Xo / Yo: the front axle centre at ground level (CarSim's
             default origin), in a frame with its origin where the front
             axle is at t = 0 and x along the car's heading then; Zo 0 (the
             Chrono ground is flat). Vx / Vy: the velocity of that point in
             the body frame, AVx / AVy / AVz the body rates, Yaw (unwrapped)
             / Pitch / Roll ISO like CarSim, Steer_L1 / Steer_R1 the front
             road wheel angles from straight ahead (the E90's static toe,
             about 1.26 deg per wheel, taken off: measured at t = 0 driving
             straight), AVy_* wheel spin (+ rolling forward), Jnc_*
             suspension travel from t = 0 (+ = compression), Throttle the ax
             command. Steer_SW 0: the model has no steering-wheel angle.

reset() builds the car at init_speed and lets it settle straight for
settle_time (chrono_plant.settle, as the KMPPI project starts), then t = 0.
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pychrono as chrono  # noqa: E402
import pychrono.vehicle as veh  # noqa: E402

from chrono_plant import ChronoBMWPlant, _cardan_zyx  # noqa: E402
from kmppi_config import KMPPIConfig  # noqa: E402

STEP = 1e-3            # s, the multibody step (chrono_plant's step_size): CarSim's t_step
TERRAIN_HALF = 3000.0  # m, the flat ground reaches this far from the start (chrono_plant sizes it from R_traj)


class ChronoCarSimEnv:
    def __init__(self, export_names, t_stop=60.0, units=None, init_speed=20.0):
        self.export_names = list(export_names)
        self.t_step = STEP
        self.t_stop = t_stop
        self.init_speed = float(init_speed)
        self.config = {"t_start": 0.0, "t_stop": t_stop, "t_step": STEP,
                       "n_import": 2, "n_export": len(self.export_names)}
        self.sim_path = ""
        u = units or {}
        k = {"angle": 1.0 if u.get("angle", "deg") == "deg" else math.radians(1.0),
             "speed": 1.0 if u.get("speed", "km/h") == "km/h" else 1.0 / 3.6,
             "rate": 1.0 if u.get("rate", "deg/s") == "deg/s" else math.radians(1.0),
             "wheel_spin": 1.0 if u.get("wheel_spin", "rpm") == "rpm" else 2 * math.pi / 60.0,
             "jounce": 1.0 if u.get("jounce", "mm") == "mm" else 1e-3}
        kind = {"Yaw": "angle", "Pitch": "angle", "Roll": "angle", "Steer_SW": "angle",
                "Vx": "speed", "Vy": "speed", "AVx": "rate", "AVy": "rate", "AVz": "rate"}
        for s in ("L1", "R1", "L2", "R2"):
            kind.update({"Steer_" + s: "angle", "AVy_" + s: "wheel_spin", "Jnc_" + s: "jounce"})
        self._unit = {n: k[g] for n, g in kind.items()}  # deg, km/h, deg/s, rpm, mm -> the page's units
        self.plant = None
        self.t_current = 0.0
        self.done = False

    # ------------------------------------------------------------------ CarSimEnv
    def reset(self):
        cfg = KMPPIConfig(plant="chrono")
        cfg.chrono.load_identified()
        cfg.R_traj = (2.0 * TERRAIN_HALF - 400.0) / 4.0  # chrono_plant: ground size 4 * R_traj + 400
        self.plant = ChronoBMWPlant(cfg, init_yaw=0.0, init_speed=self.init_speed)
        self.plant.settle()
        p = self.plant
        self._sides = [(a, s) for a in (0, 1) for s in (veh.LEFT, veh.RIGHT)]
        fx, fy = self._front_axle_xy()
        self._origin = (fx, fy)
        self._yaw0 = _cardan_zyx(p.body.GetRot())[2]
        self._yaw = 0.0
        self._last_yaw = self._yaw0
        self._jnc0 = [self._spindle_z(a, s) for a, s in self._sides]
        self._toe = (self._wheel_angle(veh.LEFT), self._wheel_angle(veh.RIGHT))  # straight ahead after settle
        self._ax = 0.0
        self.t_current = 0.0
        self.done = False
        return self._exports()

    def control_step(self, action, inner_steps):
        ax, delta = (list(action) + [0.0, 0.0])[:2]
        self._ax = float(ax)
        p = self.plant
        p.last_cmd = (float(ax), float(delta))
        p._apply_command(float(ax), float(delta))  # zero-order hold for the frame, as plant.step does per period
        for _ in range(int(inner_steps)):
            p._advance_one()
        p.time += int(inner_steps) * STEP
        self.t_current += int(inner_steps) * STEP
        self.done = self.t_current >= self.t_stop - 1e-9
        return self._exports(), 0.0, self.done, {"return_code": 0}

    def close(self):
        if self.plant is not None:
            self.plant.close()
        self.plant = None
        self.done = True

    # ------------------------------------------------------------------ measurements
    def _front_axle_xy(self):
        v = self.plant.vehicle
        a = v.GetSpindlePos(0, veh.LEFT)
        b = v.GetSpindlePos(0, veh.RIGHT)
        return 0.5 * (a.x + b.x), 0.5 * (a.y + b.y)

    def _spindle_z(self, axle, side):
        """The spindle's height in the chassis frame, m."""
        body = self.plant.body
        d = self.plant.vehicle.GetSpindlePos(axle, side) - body.GetPos()
        return body.GetRot().RotateBack(d).z

    def _wheel_angle(self, side):
        """A front road wheel's steer angle, rad, left + (chrono_plant.front_wheel_angle for one wheel)."""
        qc = self.plant.body.GetRot().GetConjugate()
        y = (qc * self.plant.vehicle.GetSpindleRot(0, side)).GetAxisY()
        return math.atan2(-y.x, y.y)

    def _exports(self):
        p = self.plant
        body, v = p.body, p.vehicle
        q = body.GetRot()
        roll, pitch, yaw = _cardan_zyx(q)
        self._yaw += math.atan2(math.sin(yaw - self._last_yaw), math.cos(yaw - self._last_yaw))  # unwrapped
        self._last_yaw = yaw
        fx, fy = self._front_axle_xy()
        c0, s0 = math.cos(self._yaw0), math.sin(self._yaw0)
        dx, dy = fx - self._origin[0], fy - self._origin[1]
        X, Y = c0 * dx + s0 * dy, -s0 * dx + c0 * dy
        # The front axle centre's velocity: the body's at its CG plus omega x (front axle - CG).
        pc = body.GetPos()
        w = body.GetAngVelParent()
        rf = np.array([fx - pc.x, fy - pc.y, 0.0])
        vw = np.array([body.GetPosDt().x, body.GetPosDt().y, body.GetPosDt().z]) + np.cross([w.x, w.y, w.z], rf)
        vloc = q.RotateBack(chrono.ChVector3d(*vw))
        wl = body.GetAngVelLocal()
        spin = {}
        for (a, s), name in zip(self._sides, ("L1", "R1", "L2", "R2")):
            spin[name] = -v.GetSpindleOmega(a, s) * 60.0 / (2 * math.pi)  # forward rolling is about -y
        jnc = {name: (self._spindle_z(a, s) - z0) * 1000.0
               for (a, s), name, z0 in zip(self._sides, ("L1", "R1", "L2", "R2"), self._jnc0)}
        vals = {
            "Xo": X, "Yo": Y, "Zo": 0.0,
            "Yaw": math.degrees(self._yaw), "Pitch": math.degrees(pitch), "Roll": math.degrees(roll),
            "Vx": vloc.x * 3.6, "Vy": vloc.y * 3.6,
            "AVx": math.degrees(wl.x), "AVy": math.degrees(wl.y), "AVz": math.degrees(wl.z),
            "Steer_SW": 0.0,
            "Steer_L1": math.degrees(self._wheel_angle(veh.LEFT) - self._toe[0]),
            "Steer_R1": math.degrees(self._wheel_angle(veh.RIGHT) - self._toe[1]),
            "Steer_L2": 0.0, "Steer_R2": 0.0,
            "AVy_L1": spin["L1"], "AVy_R1": spin["R1"], "AVy_L2": spin["L2"], "AVy_R2": spin["R2"],
            "Throttle": self._ax, "GearStat": 1.0,
            "Jnc_L1": jnc["L1"], "Jnc_R1": jnc["R1"], "Jnc_L2": jnc["L2"], "Jnc_R2": jnc["R2"],
        }
        return tuple(float(vals.get(n, 0.0)) * self._unit.get(n, 1.0) for n in self.export_names)
