"""学习的价值函数 V：MPC 4 s 时域之外的长远收益（TD-MPC 的思路：时域末端加一个学出来的价值）。

V(特征) ≈ 从这个状态起、按控制器自己的策略往后开，折扣回报之和（γ = GAMMA，每帧 0.05 s，约 10 s 视野）。
回报（每帧）：车速 / 期望车速（跑得快）− 侧向冲击（舒适）− 离出发车道（每条）的小罚 − 碰撞的大罚。
训练见 train_value.py：离线仿真里随机场景（弯道、交通车、封道、切入、红灯），32 核并行采集，n 步 TD + 目标网络。

控制器里怎么用：每个方案（本车道 / 换道 / 让行）预测 4 s 的终点状态算一次 V，方案代价减去 VALUE_WEIGHT × V
（替代手写的“车道价值”）。V 只影响选哪个方案，方案内部的连续优化不变（4 s 内的安全约束照旧是硬约束）。

特征（FEATURES 个，都已归一化到大约 −2 ~ 2）：
    自车   车速 / 期望车速、期望车速 / 30、横向偏差 / 车道宽、航向差、侧向加速度 / 5
    道路   前方 0 / 20 / 40 / 60 m 处的弯道限速 / 期望车速（√(A_LAT / |κ|)，最多 1）
    车道   左、本、右三条：能不能开、前方最近的东西（间距 / 50、相对车速 / 10、是不是静止的）、
           后方最近的车（间距 / 50、相对车速 / 10）；没有时间距记 1、相对车速记 0
    其他   到停止线的距离 / 50（没有要停的灯记 1）、离出发车道几条 / 2
推理只用 numpy（不需要 torch）。
"""
import math
import os

import numpy as np

FEATURES = 29
GAMMA = 0.995          # 每帧（0.05 s）的折扣：有效视野 1 / (1 − γ) = 200 帧 = 10 s
LOOK = 50.0            # m，车道上前 / 后看多远（场景信息给 50 m 以内）
A_LAT_FEAT = 3.0       # m/s²，特征里弯道限速用的侧向加速度


def features(vx, ey, epsi, ay, kap_preview, lanes, obs, stop_dist, home, vt, width):
    """一个状态的特征向量（numpy，长度 FEATURES）。

    vx, ey, epsi, ay   自车车速 m/s、离所在车道中心的横向距离 m（左正）、航向差 rad、侧向加速度 m/s²
    kap_preview        前方 0 / 20 / 40 / 60 m 处的曲率 1/m
    lanes              {−1: 右边能开?, 0: True, 1: 左边能开?}
    obs                [(纵向间距 m（包围盒之间，前正后负）, 横向位置 m（相对所在车道中心）, 沿车道车速 m/s, 静止?)]
    stop_dist          到停止线的距离 m（车头），没有要停的灯为 None
    home               离出发车道几条（左正）
    vt                 期望车速 m/s；width 车道宽 m
    """
    vt = max(vt, 1.0)
    f = [vx / vt, vt / 30.0, ey / width, epsi, ay / 5.0]
    for k in kap_preview:
        f.append(min(1.0, math.sqrt(A_LAT_FEAT / max(abs(k), 1e-4)) / vt))
    for r in (1, 0, -1):
        center = r * width
        front, rear = None, None
        for gap, d, vs, static in obs:
            if abs(d - center) > width / 2.0:
                continue
            if gap >= 0.0:
                if gap <= LOOK and (front is None or gap < front[0]):
                    front = (gap, vs, static)
            elif -gap <= LOOK and (rear is None or gap > rear[0]):
                rear = (gap, vs)
        f.append(1.0 if lanes.get(r, False) else 0.0)
        if front is None:
            f += [1.0, 0.0, 0.0]
        else:
            f += [front[0] / LOOK, max(-2.0, min(1.0, (front[1] - vx) / 10.0)), 1.0 if front[2] else 0.0]
        if rear is None:
            f += [1.0, 0.0]
        else:
            f += [-rear[0] / LOOK, max(-1.0, min(2.0, (rear[1] - vx) / 10.0))]
    f.append(1.0 if stop_dist is None else max(0.0, min(1.0, stop_dist / LOOK)))
    f.append(max(-2.0, min(2.0, home)) / 2.0)
    return np.asarray(f, dtype=np.float64)


def reward(vx, vt, ay_excess, home, collided):
    """每帧（0.05 s）的回报。"""
    r = min(vx, vt + 2.0) / max(vt, 1.0) - 0.01 * ay_excess ** 2 - 0.05 * abs(home)
    if collided:
        r -= 50.0
    return r


class ValueNet:
    """多层感知机 V(特征)，numpy 推理。权重文件：train_value.py 存的 .npz。"""

    def __init__(self, path):
        z = np.load(path)
        self.layers = []
        i = 0
        while "W%d" % i in z:
            self.layers.append((z["W%d" % i], z["b%d" % i]))
            i += 1
        self.mean, self.std = z["mean"], z["std"]
        self.info = {k: z[k].item() for k in ("gamma", "rounds", "samples") if k in z}
        if self.layers[0][0].shape[0] != FEATURES:
            raise ValueError("价值网络的输入是 %d 个特征，代码里是 %d 个：要重新训练" % (self.layers[0][0].shape[0], FEATURES))

    def __call__(self, f):
        """f: (..., FEATURES) → (...)"""
        x = (np.asarray(f, dtype=np.float64) - self.mean) / self.std
        for W, b in self.layers[:-1]:
            x = np.maximum(x @ W + b, 0.0)
        W, b = self.layers[-1]
        return (x @ W + b)[..., 0]


def load(path):
    if not path or not os.path.isfile(path):
        return None
    return ValueNet(path)
