"""Gymnasium 环境：CARLA 联合仿真平台（CoSimSession）当强化学习的训练环境。

    import gymnasium as gym, gym_env
    env = gym_env.CoSimEnv(carla_port=2100)          # 连一个已经在跑的 CARLA 服务器（原版或改版都行）
    obs, info = env.reset(seed=0)
    obs, reward, terminated, truncated, info = env.step(env.action_space.sample())

和运行界面 / run_cosim.py 是同一套仿真（CoSimSession：被控对象 + CARLA + 场景信息），区别只在谁给动作：
那里是控制算法的 control() 每帧被调用，这里是你的训练循环调用 step(action)。一次 step = 一个 CARLA 帧
（frame_dt，默认 0.05 s，Chrono 宝马的控制周期）：动作交给被控对象积分一帧 → 位姿推给 CARLA → world.tick() → 新的场景。
被控对象默认是服务器上的 PyChrono 宝马 E90（“CarSim 动力学”页的“Chrono 宝马”），动作是它的导入变量 [ax (m/s²),
前轮转角 delta (rad，左为正)]；config={"carsim": {"mock": True}} 换成模拟 CarSim（导入 [油门, 制动, 方向盘角 deg]）。

    观测   默认 = 全部导出变量（CarSim 单位，顺序同 export_names）+ 车道 [offset (m), heading_err (deg), curvature (1/m)]
           （不在车道上时这三个是 0）。要别的观测：obs_fn(exports, scene, t) → 一维数组，并给 observation_space。
           info 里有 exports 字典（CarSim 单位）和几个常用的数（t、lane_offset、speed_kmh、n_collisions ...），类型固定，
           所以 AsyncVectorEnv 能把各个环境的 info 合起来。完整的 scene（障碍物、车道、传感器数据，见 docs/场景与数据接口.md）
           是环境的属性 env.scene（向量环境里 envs.call("scene")）：它的结构随场景变（不在车道上时 lane 是 None ...），
           放进 info 会让向量环境合并 info 时出错。
    奖励   reward_fn(info, action) → float，你来写；这里的 info 多两项：info["scene"]（完整的 scene）和 info["collisions"]
           （这一步新撞上的目标）。默认只是个例子：沿车道中心行驶得 1 - |offset|/max_offset，撞到东西 -10。
    终止   terminated：撞到东西（terminate_on_collision）、离开车道（不在车道上，或 |offset| > max_offset）。
           truncated：跑满 episode_seconds（或被控对象自己到了结束时间）。原因在 info["end_reason"]。
    重置   reset(options={"spawn_index": i, "init_speed": m/s})：每个回合重新放一辆车（上一辆销毁）、被控对象重新开始。
    并行   一个进程一个环境、一个 CARLA 服务器（端口不同）；用 gymnasium.vector.AsyncVectorEnv 把几个环境放进各自的进程
           （scripts/carla_pool.sh 起 / 关几个私有端口的原版 CARLA）。
    速度   一个环境约 10 步/秒（= 0.5 倍实时）：时间几乎都花在被控对象上——Chrono 宝马积分 0.05 s 要 ~76 ms（这台服务器的 CPU 上，
           和线程数无关），CARLA 的 tick 和场景更新加起来 ~4 ms，所以 no_render 对速度几乎没有影响（默认 True：不渲染，
           不占 GPU；要相机数据就设 False，并在 config 里勾选传感器）。要更快只能并行（上面）。
    崩溃   训练进程崩溃后车会留在 CARLA 里：下一次 reset() 发现出生点上有上次留下的 hero 车会清掉它；别的车占着则报错、不动它。
"""
import copy
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import carla  # noqa: E402
import gymnasium as gym  # noqa: E402
from gymnasium import spaces  # noqa: E402

import scenario as scenariomod  # noqa: E402
import settings as st  # noqa: E402
from session import CoSimSession, check_run_config, check_run_files  # noqa: E402

STUB = os.path.join(HERE, "controllers", "gym_stub.py")   # session.start() 要一个能加载的算法文件；动作马上被换成 step() 给的
LANE_KEYS = ["width", "offset", "heading_err", "curvature", "center_rel"]


class _Driver:
    """session.driver 的替身：每帧返回 step() 最近给的动作。"""

    def __init__(self):
        self.action = None
        self.path = ""

    def __call__(self, obs, t):
        return self.action


def _deep_update(base, new):
    for k, v in new.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _deep_update(base[k], v)
        else:
            base[k] = copy.deepcopy(v)
    return base


class CoSimEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, config=None, *, carla_host="localhost", carla_port=2000, map_name="Town04_Opt",
                 spawn_index=None, init_speed=20.0, episode_seconds=40.0, frame_dt=0.05,
                 action_low=(-4.0, -0.09), action_high=(4.0, 0.09), max_offset=3.5, terminate_on_collision=True,
                 no_render=True, obs_fn=None, observation_space=None, reward_fn=None, timeout=60.0):
        """config：和界面保存的 JSON 一样的设置（只需要写要改的键），叠在默认之上、环境自己定的键之下；
        spawn_index：CARLA 出生点，None = 地图上有四条同向车道、前方直路最长的那个（Town04 的高速起点）；
        init_speed：被控对象的初始车速（Chrono 宝马，m/s）；action_low / action_high：动作范围，step() 会裁剪。"""
        super().__init__()
        if (obs_fn is None) != (observation_space is None):
            raise ValueError("obs_fn 和 observation_space 要一起给")
        self.obs_fn, self.reward_fn = obs_fn, reward_fn
        self.max_offset, self.terminate_on_collision = float(max_offset), bool(terminate_on_collision)
        self.init_speed = float(init_speed)
        self.no_render = bool(no_render)
        base = {"carsim": {"mock": False, "remote": False, "chrono": True, "chrono_init_speed": self.init_speed},
                "run": {"driver": "custom", "log_path": "", "controller": {"path": STUB, "entry": "Controller"}},
                "sync": {"frame_dt": float(frame_dt), "duration": float(episode_seconds)},
                "collect": {"enabled": False},
                "scene": {"collision": "log"}}
        d = st.load_dict(None, _deep_update(base, config or {}))
        d["scene"]["lane"] = list(dict.fromkeys(list(d["scene"]["lane"]) + LANE_KEYS))   # 默认观测要的车道量一定给
        self._d = d
        self.client = carla.Client(carla_host, int(carla_port))
        self.client.set_timeout(float(timeout))
        self.world = self.client.get_world()
        if map_name and map_name.split("/")[-1] not in self.world.get_map().name:
            self.world = self.client.load_world(map_name)
        self._world_settings = self.world.get_settings()   # close() 放回去
        cmap = self.world.get_map()
        if spawn_index is None:
            found = scenariomod.find_start(cmap)
            spawn_index = found["index"] if found else 0
        self.spawn_index = int(spawn_index)
        self._names = list(d["carsim"]["export_names"])
        low, high = np.asarray(action_low, dtype=np.float32), np.asarray(action_high, dtype=np.float32)
        self.action_space = spaces.Box(low, high, dtype=np.float32)
        if observation_space is not None:
            self.observation_space = observation_space
        else:
            n = len(self._names) + 3
            self.observation_space = spaces.Box(-np.inf, np.inf, (n,), dtype=np.float32)
        self._driver = _Driver()
        self._session = self._vehicle = None
        self.scene = {}   # 最近一步的完整场景（给算法的那份，见 docs/场景与数据接口.md）

    # ------------------------------------------------------------------ 回合
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        options = options or {}
        self._end_episode("stopped", "reset")
        d = copy.deepcopy(self._d)
        d["carla"]["spawn_index"] = int(options.get("spawn_index", self.spawn_index))
        d["carsim"]["chrono_init_speed"] = float(options.get("init_speed", self.init_speed))
        check_run_config(d)
        check_run_files(d)
        w = self.world
        pts = w.get_map().get_spawn_points()
        anchor = pts[d["carla"]["spawn_index"] % len(pts)]
        bp = w.get_blueprint_library().find(d["carla"]["vehicle"])
        bp.set_attribute("role_name", "hero")
        s = w.get_settings()
        s.synchronous_mode, s.fixed_delta_seconds = True, d["sync"]["frame_dt"]
        s.no_rendering_mode = self.no_render
        w.apply_settings(s)
        w.tick()   # 客户端先收到一帧，get_actors() 才是真的
        vehicle = w.try_spawn_actor(bp, anchor)
        if vehicle is None:
            # 出生点上有车：可能是上一次崩溃的训练进程留下的 hero 车（环境自己每个回合结束都会销毁自己的车）：清掉再试一次
            stale = [a for a in w.get_actors().filter("vehicle.*")
                     if a.attributes.get("role_name") == "hero" and a.get_location().distance(anchor.location) < 3.0]
            for a in stale:
                a.destroy()
            if stale:
                print("CoSimEnv: 出生点 %d 上有 %d 辆上次运行留下的 hero 车，已清除" % (d["carla"]["spawn_index"], len(stale)), flush=True)
                w.tick()
                vehicle = w.try_spawn_actor(bp, anchor)
        if vehicle is None:
            raise RuntimeError("出生点 %d 被别的车辆或物体占用，无法生成主车：换一个 spawn_index" % d["carla"]["spawn_index"])
        try:
            for _ in range(30):   # 让车在 PhysX 里落稳，桥接读到的是静止的车
                w.tick()
            session = CoSimSession(w, vehicle, anchor, d)
            session.start()
            session.driver = self._driver
        except BaseException:
            vehicle.destroy()
            raise
        n = session.n_actions()
        if n != self.action_space.shape[0]:
            session.stop(release_vehicle=False, end="error", reason="动作个数不对")
            vehicle.destroy()
            raise RuntimeError("被控对象有 %d 个导入变量，action_space 是 %d 维：用 action_low / action_high 给 %d 维的范围"
                               % (n, self.action_space.shape[0], n))
        self._session, self._vehicle = session, vehicle
        self._driver.action = [0.0] * n
        self._terminal = None
        info, rich = self._make_info([])
        return self._observation(rich), info

    def step(self, action):
        if self._session is None or self._terminal is not None:
            raise RuntimeError("回合已经结束（或还没 reset()）：先调用 reset()")
        a = np.clip(np.asarray(action, dtype=np.float64).reshape(-1), self.action_space.low, self.action_space.high)
        self._driver.action = [float(x) for x in a]
        tel = self._session.step()
        hits = tel.get("collisions", [])
        info, rich = self._make_info(hits)
        reason = None
        if self.terminate_on_collision and hits:
            reason = "碰撞：撞到 %s（id %s）" % (hits[0]["model"], hits[0]["id"])
        elif math.isnan(info["lane_offset"]):
            reason = "离开了车道"
        elif abs(info["lane_offset"]) > self.max_offset:
            reason = "偏离车道中心 %.1f m（超过 %.1f m）" % (abs(info["lane_offset"]), self.max_offset)
        terminated = reason is not None
        truncated = bool(self._session.done) and not terminated
        if truncated:
            reason = self._session.end_reason or "跑满了 %.0f s" % self._d["sync"]["duration"]
        info["end_reason"] = rich["end_reason"] = reason or ""
        reward = float(self.reward_fn(rich, a)) if self.reward_fn is not None else self._default_reward(rich, terminated)
        if terminated or truncated:
            self._terminal = reason
        return self._observation(rich), reward, terminated, truncated, info

    def close(self):
        self._end_episode("stopped", "close")
        if self._world_settings is not None:
            try:
                self.world.apply_settings(self._world_settings)
            except Exception:
                pass
            self._world_settings = None

    # ------------------------------------------------------------------ 内部
    def _end_episode(self, end, reason):
        ses, veh, self._session, self._vehicle = self._session, self._vehicle, None, None
        if ses is not None:
            try:
                ses.stop(release_vehicle=False, end=end, reason=reason)
            except Exception as e:  # noqa: BLE001
                print("CoSimEnv: 结束上一个回合时：%s" % e, flush=True)
        if veh is not None:
            try:
                veh.destroy()
            except Exception:
                pass

    def _make_info(self, collisions):
        """返回 (info, rich)：info 给 step() / reset() 返回（类型固定：向量环境要把各环境的 info 按键合并），
        rich = info + 完整的 scene 和这一步的碰撞（给 reward_fn 和观测）。"""
        ses = self._session
        # scene["frame"] 是 CARLA 世界的帧号（服务器一直往上数，每个回合不同）：不放进去，同样的动作才得到同样的结果
        scene = {k: v for k, v in (ses.scene.view() or {}).items() if k != "frame"}
        self.scene = scene
        lane = scene.get("lane") or {}
        nan = float("nan")
        off, herr = lane.get("offset"), lane.get("heading_err")
        info = {"t": float(ses.env.t_current), "frame": int(ses.frame), "exports": ses.exports(),
                "lane_offset": nan if off is None else float(off), "lane_heading_err": nan if herr is None else float(herr),
                "speed_kmh": float((scene.get("ego") or {}).get("Speed", nan)), "n_collisions": len(collisions),
                "end_reason": ""}
        return info, {**info, "scene": scene, "collisions": list(collisions)}

    def _observation(self, info):
        if self.obs_fn is not None:
            return self.obs_fn(info["exports"], info["scene"], info["t"])
        lane = info["scene"].get("lane") or {}
        v = [float(info["exports"][n]) for n in self._names]
        v += [float(lane.get("offset", 0.0)), float(lane.get("heading_err", 0.0)), float(lane.get("curvature", 0.0))]
        return np.asarray(v, dtype=np.float32)

    def _default_reward(self, info, terminated):
        """例子：沿车道中心行驶得 1 - |offset| / max_offset（0 ~ 1），撞到东西 -10，离开车道 -10。自己的奖励用 reward_fn。"""
        if info["collisions"] and self.terminate_on_collision:
            return -10.0
        off = info["lane_offset"]
        if math.isnan(off) or (terminated and abs(off) > self.max_offset):
            return -10.0
        return 1.0 - min(abs(off) / self.max_offset, 1.0)
