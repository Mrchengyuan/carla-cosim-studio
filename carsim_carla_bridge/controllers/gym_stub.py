"""gym_env.CoSimEnv 用的占位算法：session.start() 要加载一个算法文件，动作马上被 env.step() 给的换掉，这里的 control() 不会被调用。"""


class Controller:
    def control(self, exports, t, dt):
        raise RuntimeError("gym_stub 的 control() 不该被调用：动作来自 CoSimEnv.step()")
