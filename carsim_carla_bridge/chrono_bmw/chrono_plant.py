"""
PyChrono BMW_E90 高保真被控对象封装 (对应 MATLAB 工程里的 official_14dof_plant.slx 接口包装层)。

对外接口与 3DOF 基准一致:
    plant.get_state() -> [X, Y, yaw, vx, vy, r]   (X, Y 为质心在参考坐标系下的位置)
    plant.step(ax, delta) -> 施加一个控制周期 (dt) 的零阶保持命令, 返回新状态
    plant.audit() -> dict(roll, pitch, heave, Fz_FL, Fz_FR, Fz_RL, Fz_RR)

命令适配层 (与 Simulink 的 Acceleration_To_WheelTorque / RoadWheel_To_SteeringWheel 对应):
    drive_mode="torque":  断开传动系, 每个车轮驱动轴上直接施加
                          T = -(ax*accel_gain + rolling_comp) * m * r_w / 4     (负号: 轴上向前驱动为负)
    drive_mode="throttle": 保留发动机/变速箱, 用速度环把 ax 换成油门/刹车 (实验性)
    转向: steering_input = delta / steer_gain, 或按标定查表 (delta -> u)

坐标系: ISO 车辆坐标, x 向前, y 向左, z 向上。参考轨迹的原点定义为起步稳定 (settle) 结束时刻的质心位置。
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh

from kmppi_config import KMPPIConfig

veh.SetVehicleDataPath(chrono.GetChronoDataPath() + "vehicle/")


def _yaw_quat(yaw: float) -> chrono.ChQuaterniond:
    return chrono.QuatFromAngleZ(yaw)


def _cardan_zyx(q: chrono.ChQuaterniond):
    """返回 (roll, pitch, yaw)。注意 Chrono 10 的 GetCardanAnglesZYX() 分量顺序是 (x=绕Y的俯仰, y=绕X的侧倾, z=横摆),
    已用 QuatFromAngleX/Y/Z 实测验证 (2026-09-05)。"""
    ang = q.GetCardanAnglesZYX()
    return ang.y, ang.x, ang.z


class ChronoBMWPlant:
    def __init__(self, cfg: KMPPIConfig, init_yaw: float, init_speed: float,
                 path_xy: Optional[np.ndarray] = None, vis: Optional[bool] = None):
        self.cfg = cfg
        self.pc = cfg.chrono
        self.dt = cfg.dt
        self.step_size = self.pc.step_size
        self.n_inner = int(round(self.dt / self.step_size))
        if abs(self.n_inner * self.step_size - self.dt) > 1e-9:
            raise ValueError("dt 必须是 step_size 的整数倍")
        self.vis_enabled = self.pc.vis if vis is None else vis
        self.init_yaw = init_yaw
        self.init_speed = init_speed

        # ---------------- 车辆 ----------------
        car = veh.BMW_E90()
        car.SetContactMethod(chrono.ChContactMethod_NSC)
        car.SetChassisFixed(False)
        car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.5), _yaw_quat(init_yaw)))
        car.SetInitFwdVel(init_speed)
        car.SetTireType(veh.TireModelType_TMEASY)
        car.SetTireStepSize(self.pc.tire_step_size)
        car.SetInitWheelAngVel([init_speed / self.pc.wheel_radius] * 4)
        car.Initialize()
        # SetInitFwdVel 只给车身设速度; 把所有刚体的平动速度都设成初速度, 避免第一步动量平均掉速
        v_init = chrono.ChVector3d(init_speed * math.cos(init_yaw), init_speed * math.sin(init_yaw), 0.0)
        for b in car.GetSystem().GetBodies():
            b.SetPosDt(v_init)
        if self.vis_enabled:
            car.SetChassisVisualizationType(chrono.VisualizationType_MESH)
            car.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
            car.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
            car.SetWheelVisualizationType(chrono.VisualizationType_MESH)
            car.SetTireVisualizationType(chrono.VisualizationType_MESH)
        else:
            car.SetChassisVisualizationType(chrono.VisualizationType_NONE)
            car.SetSuspensionVisualizationType(chrono.VisualizationType_NONE)
            car.SetSteeringVisualizationType(chrono.VisualizationType_NONE)
            car.SetWheelVisualizationType(chrono.VisualizationType_NONE)
            car.SetTireVisualizationType(chrono.VisualizationType_NONE)
        car.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
        self.car = car
        self.vehicle = car.GetVehicle()
        self.system = car.GetSystem()
        self.body = self.vehicle.GetChassisBody()
        self.mass = float(self.vehicle.GetMass())

        # ---------------- 地面 ----------------
        self.terrain = veh.RigidTerrain(self.system)
        mat = chrono.ChContactMaterialNSC()
        mat.SetFriction(self.pc.friction)
        mat.SetRestitution(0.01)
        size = 4.0 * cfg.R_traj + 400.0
        self.patch = self.terrain.AddPatch(mat, chrono.CSYSNORM, size, size)
        if self.vis_enabled:
            self.patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), size / 4, size / 4)
            self.patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
        self.terrain.Initialize()

        # ---------------- 命令适配 ----------------
        self.sides = (veh.LEFT, veh.RIGHT)
        self.axle_shafts = [(a, s, self.vehicle.GetSuspension(a).GetAxle(s)) for a in (0, 1) for s in self.sides]
        if self.pc.drive_mode == "torque":
            self.vehicle.GetDriveline().Disconnect()
        elif self.pc.drive_mode != "throttle":
            raise ValueError("drive_mode must be 'torque' or 'throttle'")
        self.driver_inputs = veh.DriverInputs()
        self._v_target = init_speed     # throttle 模式用
        self._speed_int = 0.0
        if self.pc.steer_table_u is not None:
            self._steer_u = np.asarray(self.pc.steer_table_u, dtype=float)
            self._steer_delta = np.asarray(self.pc.steer_table_delta, dtype=float)
        else:
            self._steer_u = None

        # ---------------- 可视化 ----------------
        self.vis = None
        if self.vis_enabled:
            import pychrono.irrlicht as irr  # noqa: F401
            vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
            vis.SetWindowTitle("KMPPI on PyChrono BMW E90")
            vis.SetWindowSize(1280, 800)
            vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.5), 8.0, 0.8)
            vis.Initialize()
            vis.AddLightDirectional()
            vis.AddSkyBox()
            vis.AttachVehicle(self.vehicle)
            self.vis = vis
            self._path_xy = path_xy
        self._vis_counter = 0

        # ---------------- 参考原点 ----------------
        self.origin = np.zeros(3)
        self.z0 = float(self.body.GetPos().z)
        self.time = 0.0
        self.last_cmd = (0.0, 0.0)

    # ------------------------------------------------------------------ 内部工具
    def _steer_input_from_delta(self, delta: float) -> float:
        if self._steer_u is None:
            u = delta / self.pc.steer_gain
        else:
            mag_d = abs(delta)
            if mag_d <= self._steer_delta[-1]:
                mag = np.interp(mag_d, self._steer_delta, self._steer_u)
            else:  # 超出标定表: 用表末段斜率线性外推
                slope = (self._steer_u[-1] - self._steer_u[-2]) / max(self._steer_delta[-1] - self._steer_delta[-2], 1e-6)
                mag = self._steer_u[-1] + slope * (mag_d - self._steer_delta[-1])
            u = math.copysign(mag, delta)
        return float(np.clip(u, -1.0, 1.0))

    def _apply_command(self, ax: float, delta: float):
        di = self.driver_inputs
        di.m_steering = self._steer_input_from_delta(delta)
        if self.pc.drive_mode == "torque":
            torque = -(ax * self.pc.accel_gain + self.pc.rolling_comp) * self.mass * self.pc.wheel_radius / 4.0
            for _, _, shaft in self.axle_shafts:
                shaft.SetAppliedLoad(torque)
            di.m_throttle = 0.0
            di.m_braking = 0.0
        else:
            # throttle 模式: 由 ax 积分出目标速度, PI 环跟踪
            self._v_target += ax * self.step_size
            err = self._v_target - self.vehicle.GetSpeed()
            self._speed_int = float(np.clip(self._speed_int + err * self.step_size, -5.0, 5.0))
            u = 0.4 * err + 0.1 * self._speed_int
            di.m_throttle = float(np.clip(u, 0.0, 1.0))
            di.m_braking = float(np.clip(-u, 0.0, 1.0))

    def _advance_one(self):
        t = self.system.GetChTime()
        if self.vis is not None:
            self._vis_counter += 1
            if self._vis_counter % self.pc.vis_every == 0:
                if not self.vis.Run():
                    raise KeyboardInterrupt("visualization window closed")
                self.vis.BeginScene()
                self.vis.Render()
                self.vis.EndScene()
            self.vis.Synchronize(t, self.driver_inputs)
        self.terrain.Synchronize(t)
        self.car.Synchronize(t, self.driver_inputs, self.terrain)
        self.terrain.Advance(self.step_size)
        self.car.Advance(self.step_size)
        if self.vis is not None:
            self.vis.Advance(self.step_size)

    def draw_path(self):
        if self.vis is None or self._path_xy is None:
            return
        pts = chrono.vector_ChVector3d()
        for x, y in self._path_xy:
            pts.push_back(chrono.ChVector3d(float(x) + self.origin[0], float(y) + self.origin[1], 0.05))
        curve = chrono.ChBezierCurve(pts, True)
        shape = chrono.ChVisualShapeLine()
        shape.SetLineGeometry(chrono.ChLineBezier(curve))
        shape.SetColor(chrono.ChColor(0.0, 0.8, 0.0))
        shape.SetNumRenderPoints(2000)
        self.patch.GetGroundBody().AddVisualShape(shape)
        self.vis.BindAll()

    # ------------------------------------------------------------------ 公共接口
    def settle(self, duration: Optional[float] = None):
        """起步稳定: 直线匀速行驶 duration 秒, 让悬架/轮胎载荷收敛。结束时刻定义参考原点与 t=0。"""
        duration = self.pc.settle_time if duration is None else duration
        n = int(round(duration / self.step_size))
        for _ in range(n):
            v = self.vehicle.GetSpeed()
            ax = float(np.clip(1.0 * (self.init_speed - v), -self.cfg.ax_max, self.cfg.ax_max))
            self._apply_command(ax, 0.0)
            self._advance_one()
        p = self.body.GetPos()
        self.origin = np.array([p.x, p.y, 0.0])
        self.z0 = float(p.z)
        self.time = 0.0
        self._v_target = self.vehicle.GetSpeed()
        self._speed_int = 0.0
        self.draw_path()
        return self.get_state()

    def get_state(self) -> np.ndarray:
        p = self.body.GetPos()
        q = self.body.GetRot()
        _, _, yaw = _cardan_zyx(q)
        vloc = q.RotateBack(self.body.GetPosDt())
        r = self.body.GetAngVelLocal().z
        return np.array([p.x - self.origin[0], p.y - self.origin[1], yaw, vloc.x, vloc.y, r])

    def step(self, ax: float, delta: float) -> np.ndarray:
        self.last_cmd = (float(ax), float(delta))
        self._apply_command(float(ax), float(delta))
        for _ in range(self.n_inner):
            self._advance_one()
        self.time += self.dt
        return self.get_state()

    def audit(self) -> dict:
        roll, pitch, _ = _cardan_zyx(self.body.GetRot())
        out = dict(roll=roll, pitch=pitch, heave=float(self.body.GetPos().z - self.z0))
        for name, (a, s) in zip(("Fz_FL", "Fz_FR", "Fz_RL", "Fz_RR"), [(0, veh.LEFT), (0, veh.RIGHT), (1, veh.LEFT), (1, veh.RIGHT)]):
            tf = self.vehicle.GetTire(a, s).ReportTireForce(self.terrain)
            out[name] = float(tf.force.z)
        out["steer_input"] = float(self.driver_inputs.m_steering)
        out["wheel_angle"] = self.front_wheel_angle()
        return out

    def front_wheel_angle(self) -> float:
        """前轮平均转角 [rad]: 主销 y 轴在车身 xy 平面上的投影角 (不受轮子自转影响)。"""
        qc = self.body.GetRot().GetConjugate()
        angles = []
        for s in self.sides:
            y = (qc * self.vehicle.GetSpindleRot(0, s)).GetAxisY()
            angles.append(math.atan2(-y.x, y.y))
        return 0.5 * (angles[0] + angles[1])

    def close(self):
        if self.vis is not None:
            self.vis.Quit()
