"""用 python_carsim_env 里的 SimplePathFollower 作为控制算法（原来的 PID 模式）。

需要在导出变量里有横向误差 LatErr 和车速 Vx；名字不同就改下面两行。
"""
from simple_controller import SimplePathFollower   # 来自 python_carsim_env

LATERAL_ERROR = "LatErr"
SPEED = "Vx"            # km/h
TARGET_KMH = 50.0


class Controller:
    def reset(self):
        self.ctrl = SimplePathFollower()
        self.ctrl.reset()

    def control(self, exports, t, dt):
        return list(self.ctrl.control(exports[SPEED], TARGET_KMH, exports[LATERAL_ERROR], dt=dt))
