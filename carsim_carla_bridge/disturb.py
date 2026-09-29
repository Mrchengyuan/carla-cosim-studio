"""干扰注入（驾驶模式页“干扰”，config run.disturb）：检验控制算法的鲁棒性。只作用在你的算法看到的和
它发出的东西上，运行记录（log.csv）记的还是 CarSim 的真值：

    act_delay     s    执行器延迟：算法的输出晚这么久才到 CarSim（按帧取整；最初几帧给 CarSim 的是第一次的输出）
    sense_delay   s    测量延迟：算法拿到的导出变量和场景是这么久以前的（最初几帧是第一帧的）
    noise         {导出变量名: 标准差}  高斯噪声，单位同导出变量（CarSim 动力学页的单位）
    lane_dropout  0~1  每帧以这个概率拿不到车道信息（scene["lane"] 为 None，像没识别到车道线）
    seed               随机种子：同样的设置每次运行的噪声和丢失都一样

最长延迟 1 s。
"""
import collections
import copy

import numpy as np

MAX_DELAY = 1.0


def config(d):
    """The run's disturbance settings, None when off (or everything is 0)."""
    x = (d.get("run") or {}).get("disturb") or {}
    if not x.get("enabled"):
        return None
    noise = {str(k): float(v) for k, v in (x.get("noise") or {}).items() if float(v) > 0}
    out = {"act_delay": float(x.get("act_delay") or 0.0), "sense_delay": float(x.get("sense_delay") or 0.0),
           "noise": noise, "lane_dropout": float(x.get("lane_dropout") or 0.0), "seed": int(x.get("seed") or 0)}
    if not (out["act_delay"] or out["sense_delay"] or noise or out["lane_dropout"]):
        return None
    return out


def check(d):
    """Refused in plain words (check_run_config)."""
    x = config(d)
    if x is None:
        return
    for k, name in (("act_delay", "执行器延迟"), ("sense_delay", "测量延迟")):
        if not 0.0 <= x[k] <= MAX_DELAY:
            raise ValueError("干扰：%s %g s 不在 0 ~ %g s 之间（驾驶模式页）" % (name, x[k], MAX_DELAY))
    if not 0.0 <= x["lane_dropout"] <= 1.0:
        raise ValueError("干扰：车道信息丢失概率 %g 不在 0 ~ 1 之间" % x["lane_dropout"])
    names = set(d["carsim"]["export_names"])
    unknown = [n for n in x["noise"] if n not in names]
    if unknown:
        raise ValueError("干扰：噪声加在 %s 上，但导出变量里没有（“CarSim 动力学”页）" % "、".join(unknown))


def describe(x, dt):
    parts = []
    if x["act_delay"]:
        parts.append("执行器延迟 %g s（%d 帧）" % (x["act_delay"], frames(x["act_delay"], dt)))
    if x["sense_delay"]:
        parts.append("测量延迟 %g s（%d 帧）" % (x["sense_delay"], frames(x["sense_delay"], dt)))
    if x["noise"]:
        parts.append("噪声 " + "、".join("%s ±%g" % kv for kv in x["noise"].items()))
    if x["lane_dropout"]:
        parts.append("车道信息丢失 %g%%" % (x["lane_dropout"] * 100))
    return "，".join(parts) + "（种子 %d）" % x["seed"]


def frames(delay, dt):
    return int(round(delay / dt)) if dt > 0 else 0


class Disturb:
    """Between the run and the algorithm: sense() on what it is given, act() on what it returns."""

    def __init__(self, x, dt):
        self.x = x
        self.n_act, self.n_sense = frames(x["act_delay"], dt), frames(x["sense_delay"], dt)
        self.rng = np.random.default_rng(x["seed"])
        self.acts = collections.deque(maxlen=self.n_act + 1)
        self.seen = collections.deque(maxlen=self.n_sense + 1)
        self.dropped = 0

    def sense(self, exports, scene):
        """(exports, scene) the algorithm gets this frame."""
        self.seen.append((exports, scene))
        exports, scene = self.seen[0]  # n_sense frames old (the first one until there are that many)
        if self.x["noise"]:
            exports = dict(exports)
            for n, sd in self.x["noise"].items():
                if n in exports:
                    exports[n] = float(exports[n]) + float(self.rng.normal(0.0, sd))
        if self.x["lane_dropout"] and scene is not None and self.rng.random() < self.x["lane_dropout"]:
            scene = copy.copy(scene)
            scene["lane"] = None
            self.dropped += 1
        return exports, scene

    def act(self, values):
        """What reaches CarSim this frame: the output of n_act frames ago."""
        self.acts.append(list(values))
        return list(self.acts[0])
