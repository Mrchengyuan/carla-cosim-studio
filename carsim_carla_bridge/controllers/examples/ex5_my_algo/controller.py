"""例 5：拆成多个文件 + 从文件读参数。界面“算法文件”填 controllers/examples/ex5_my_algo/controller.py。"""
import json
import os

from speed import SpeedPI      # 同一个文件夹里的 speed.py，直接 import

HERE = os.path.dirname(os.path.abspath(__file__))           # 这个 .py 所在的文件夹


class Controller:
    def reset(self):
        with open(os.path.join(HERE, "params.json"), encoding="utf-8") as f:
            self.p = json.load(f)                            # {"target_kmh": 30, "brake_max": 8}
        self.speed = SpeedPI(self.p["target_kmh"])
        print("参数：", self.p)

    def control(self, exports, t, dt, scene):
        u = self.speed.step(exports["Vx"], dt)
        return [min(max(u, 0.0), 1.0), min(max(-u, 0.0), 1.0) * self.p["brake_max"], 0.0]
