"""控制算法示例：复制这个文件，把 control() 换成你的算法。

界面「驾驶模式」页选“我的控制算法”，文件选这个 .py。每次点“运行”都会重新
加载文件，改完代码直接再点运行即可，不用重启界面。

约定
    class Controller:
        reset(self)                          可选，每次运行开始调用一次
        control(self, exports, t, dt) -> list 每帧调用一次

    exports  dict，CarSim 全部导出变量，按名字取值，CarSim 单位
             （如 exports["Vx"] km/h，exports["Yaw"] deg）。
             变量名就是“CarSim 动力学”页导出变量列表里填的名字。
    t, dt    CarSim 当前时间、控制周期（= 界面里的“仿真步长”），秒
    返回值   按 .sim 里导入变量（REPLACE）的顺序给出的数值。
             python_carsim_env 的默认顺序：[油门 0~1, 制动, 方向盘转角 deg（左正）]

入口也可以是普通函数 control(exports, t, dt)，在界面“入口”里填函数名。
"""
import math


class Controller:
    TARGET_KMH = 40.0

    def reset(self):
        self.integral = 0.0

    def control(self, exports, t, dt):
        # 纵向：PI 定速
        err = self.TARGET_KMH - exports["Vx"]
        self.integral = max(-50.0, min(50.0, self.integral + err * dt))
        u = 0.08 * err + 0.02 * self.integral
        throttle = max(0.0, min(1.0, u))
        brake = max(0.0, min(1.0, -u))
        # 横向：3 s 后缓慢蛇形，只是为了看到转向同步；换成你的横向控制
        steer_sw = 0.0 if t < 3.0 else 30.0 * math.sin(2 * math.pi * 0.2 * (t - 3.0))
        return [throttle, brake, steer_sw]
