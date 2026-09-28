class SpeedPI:
    """定速 PI：输入车速 km/h，输出 u（正 = 油门，负 = 制动）。"""

    def __init__(self, target_kmh, kp=0.08, ki=0.02):
        self.target, self.kp, self.ki, self.integral = target_kmh, kp, ki, 0.0

    def step(self, v_kmh, dt):
        err = self.target - v_kmh
        self.integral = min(max(self.integral + err * dt, -100.0), 100.0)
        return self.kp * err + self.ki * self.integral
