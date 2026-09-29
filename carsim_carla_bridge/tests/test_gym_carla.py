"""gym_env.CoSimEnv（Gymnasium 环境）在 CARLA 服务器上：被控对象是 PyChrono 宝马 E90，Town04 高速起点。

检查：空间和第一个观测（形状、有限、在 observation_space 里）；info 里有 exports 和 scene；用你的算法当策略
（controllers/kmppi_learned 的 Controller：control(exports, t, dt, scene) → [ax, delta] → step）跑一个回合，
车道中心偏差和平台运行的同量级、不终止、最后 truncated；两次 reset 后同样的动作序列得到同样的观测（可复现）；
猛打方向 → terminated（离开车道），回合结束后 step() 要求先 reset()；跑满 episode_seconds → truncated；
reset(options) 的初始车速和出生点生效；自定义 reward_fn 被用；动作被裁剪；超出范围的 obs_fn 不给 observation_space 被拒绝；
导入变量个数和 action_space 不一致时说清楚、并且不留下车；gymnasium 自带的 check_env 通过；close() 把 CARLA 的设置放回去；
--speed 时量一下每秒多少步。

    python tests/test_gym_carla.py [--port 2000] [--speed]
"""
import contextlib
import io
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "controllers", "kmppi_learned"))

import carla  # noqa: E402
import gymnasium as gym  # noqa: E402
from gymnasium.utils.env_checker import check_env  # noqa: E402

import gym_env  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    shown = "" if isinstance(detail, str) and detail == "" else "  " + str(detail)
    print(("PASS " if cond else "FAIL ") + name + shown, flush=True)
    if not cond:
        FAILS.append(name)


def quiet(fn, *a, **k):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        out = fn(*a, **k)
    return out


def vehicles(world):
    """世界里有几辆车。新连接的客户端在同步模式下还没收到过一帧，读到的是 0：用已经在 tick 的那个 world；
    否则先 wait_for_tick（异步模式）。"""
    return len(world.get_actors().filter("vehicle.*"))


def vehicles_settled(world, want):
    """异步模式下等到车数是 want（销毁的车下一帧才离开世界），最多 3 s；返回最后数到的。"""
    n = vehicles(world)
    for _ in range(30):
        if n == want:
            break
        time.sleep(0.1)
        world.wait_for_tick()
        n = vehicles(world)
    return n


def main():
    port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 2000
    env = gym_env.CoSimEnv(carla_port=port, episode_seconds=20.0)
    try:
        n_exp = len(env._names)
        check("action_space Box(2,) [ax, delta]，observation_space Box(%d,)（导出变量 + 车道 3 个）" % (n_exp + 3),
              env.action_space.shape == (2,) and env.observation_space.shape == (n_exp + 3,),
              (env.action_space, env.observation_space.shape))
        obs, info = env.reset(seed=0)
        check("第一个观测：形状对、有限、在 observation_space 里", obs.shape == env.observation_space.shape
              and np.all(np.isfinite(obs)) and env.observation_space.contains(obs), obs[:6])
        check("info 里有 exports（含 Vx）和常用的数、t = 0；完整的场景在 env.scene（含 lane、ego、units）",
              "Vx" in info["exports"] and info["t"] < 1e-6 and {"lane_offset", "speed_kmh", "n_collisions", "end_reason"} <= set(info)
              and {"lane", "ego", "units"} <= set(env.scene), (sorted(info), sorted(env.scene)))
        check("info 的类型固定（数、文字、exports 字典），AsyncVectorEnv 才能合并各环境的 info（scene 不在里面）",
              "scene" not in info and all(isinstance(v, (int, float, str)) for k, v in info.items() if k != "exports")
              and all(isinstance(v, float) for v in info["exports"].values()), {k: type(v).__name__ for k, v in info.items()})
        check("初始车速 20 m/s = 72 km/h", abs(info["exports"]["Vx"] - 72.0) < 1.0, info["exports"]["Vx"])
        check("在车道里（offset 在 1 m 以内）", abs(info["lane_offset"]) < 1.0, info["lane_offset"])

        # ---- 你的算法当策略
        import controller as kmppi
        kmppi.DEVICE = "auto"
        kmppi.DRAW_CANDIDATES = 0
        pol = kmppi.Controller()
        quiet(pol.reset)
        offs, t0, steps, terminated, truncated = [], time.time(), 0, False, False
        obs, info = env.reset(seed=1)
        while not (terminated or truncated):
            a = quiet(pol.control, info["exports"], info["t"], 0.05, env.scene)
            obs, r, terminated, truncated, info = env.step(np.asarray(a, dtype=np.float32))
            steps += 1
            if not math.isnan(info["lane_offset"]):
                offs.append(abs(info["lane_offset"]))
        rms = math.sqrt(float(np.mean(np.square(offs))))
        wall = time.time() - t0
        print("INFO 你的算法当策略：%d 步（%.0f s 仿真）用 %.0f s，车道偏差 rms %.3f max %.3f m，结束原因：%s" % (
            steps, info["t"], wall, rms, max(offs), info["end_reason"]))
        check("你的算法当策略：跑满 20 s（400 步），不终止、最后 truncated", steps == 400 and truncated and not terminated, (steps, terminated, truncated))
        check("... 车道中心偏差 rms < 0.1 m、最大 < 0.5 m（平台运行里 0.022 / 0.17）", rms < 0.1 and max(offs) < 0.5, (round(rms, 3), round(max(offs), 3)))
        check("... 结束原因写清楚", info["end_reason"] == "跑满了 20 s", info["end_reason"])
        try:
            env.step(env.action_space.sample())
            msg = ""
        except RuntimeError as e:
            msg = str(e)
        check("回合结束后 step() 要求先 reset()", "reset" in msg, msg)

        # ---- 可复现
        def rollout(n=30):
            o, _ = env.reset(seed=5)
            seq = [o]
            for i in range(n):
                o, *_ = env.step(np.array([0.5 * math.sin(0.2 * i), 0.02 * math.sin(0.15 * i)], dtype=np.float32))
                seq.append(o)
            return np.stack(seq)
        a, b = rollout(), rollout()
        check("两次 reset 后同样的动作序列 → 同样的观测（30 步，差 < 1e-4）", np.max(np.abs(a - b)) < 1e-4, float(np.max(np.abs(a - b))))
        check("... 而且车动了（Vx 变了）", np.max(np.abs(a[:, 0] - a[0, 0])) > 0.1 or np.ptp(a[:, 3]) > 0.1, float(np.ptp(a[:, 0])))

        # ---- 猛打方向：离开车道 → terminated
        env.reset(seed=2)
        terminated = truncated = False
        n = 0
        while not (terminated or truncated) and n < 300:
            _, r, terminated, truncated, info = env.step(np.array([0.0, 0.09], dtype=np.float32))
            n += 1
        check("猛打方向：终止（不是 truncated），原因是离开车道 / 偏离中心，最后一步的奖励是 -10",
              terminated and not truncated and ("车道" in info["end_reason"]) and r == -10.0, (n, info["end_reason"], r))

        # ---- 跑满 episode_seconds → truncated（短回合）
        env2 = gym_env.CoSimEnv(carla_port=port, episode_seconds=2.0)
        env2.reset()
        outs = [env2.step(np.zeros(2, dtype=np.float32)) for _ in range(40)]
        env2.close()
        check("episode_seconds = 2：第 40 步 truncated（前面都不是）", outs[-1][3] and not outs[-1][2] and not any(o[3] or o[2] for o in outs[:-1]),
              [(o[2], o[3]) for o in outs[-2:]])

        # ---- reset(options)
        _, info = env.reset(options={"init_speed": 10.0})
        check("reset(options={'init_speed': 10}) → 36 km/h", abs(info["exports"]["Vx"] - 36.0) < 1.0, info["exports"]["Vx"])
        loc0 = env._vehicle.get_location()
        pts = env.world.get_map().get_spawn_points()
        other = next(i for i, p in enumerate(pts) if p.location.distance(loc0) > 30.0)
        _, info = env.reset(options={"spawn_index": other})
        loc1 = env._vehicle.get_location()
        check("reset(options={'spawn_index': ...}) → 车换了出生点", loc1.distance(loc0) > 30.0, round(loc1.distance(loc0), 1))
        check("... 每个回合只有一辆主车（上一辆销毁了）", vehicles(env.world) == 1, vehicles(env.world))

        # ---- 上次崩溃留下的 hero 车在出生点上：清掉；别的车占着：说清楚、不动它
        env._end_episode("stopped", "test")
        pts = env.world.get_map().get_spawn_points()
        anchor = pts[env.spawn_index]
        lib = env.world.get_blueprint_library()
        bp = lib.find("vehicle.tesla.model3")
        bp.set_attribute("role_name", "hero")
        ghost = env.world.spawn_actor(bp, anchor)
        env.world.tick()
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            env.reset()
        check("出生点上有上次崩溃留下的 hero 车：清除后照常开始，说一声", "已清除" in buf.getvalue() and env.world.get_snapshot().find(ghost.id) is None
              and vehicles(env.world) == 1, (buf.getvalue().strip(), vehicles(env.world)))
        env._end_episode("stopped", "test")
        bp = lib.find("vehicle.tesla.model3")
        bp.set_attribute("role_name", "autopilot")
        other_car = env.world.spawn_actor(bp, anchor)
        env.world.tick()
        try:
            env.reset()
            msg = ""
        except RuntimeError as e:
            msg = str(e)
        alive = env.world.get_snapshot().find(other_car.id) is not None
        other_car.destroy()
        env.world.tick()
        check("出生点被别的车占着（不是 hero）：说清楚、不动它", "被别的车辆或物体占用" in msg and alive, (msg, alive))
        env.reset()

        # ---- 动作被裁剪
        env.reset()
        _, r, term, trunc, info = env.step(np.array([100.0, 5.0], dtype=np.float32))
        check("动作被裁剪：给 [100, 5] 不出错", np.isfinite(r), r)

        # ---- gymnasium 自带的检查
        with contextlib.redirect_stdout(io.StringIO()):
            check_env(env, skip_render_check=True)
        check("gymnasium 的 check_env 通过", True)
    finally:
        env.close()
    client = carla.Client("localhost", port)
    client.set_timeout(30.0)
    w = client.get_world()
    w.wait_for_tick()   # 异步模式：等来一帧，之后 get_actors() 才是真的
    check("close()：CARLA 放回异步模式、有渲染", not w.get_settings().synchronous_mode and not w.get_settings().no_rendering_mode,
          (w.get_settings().synchronous_mode, w.get_settings().no_rendering_mode))
    check("... 也没有留下车", vehicles_settled(w, 0) == 0, vehicles(w))

    # ---- 自定义奖励 / 观测的检查 / 导入变量个数
    seen = {}

    def my_reward(info, action):
        seen.update(scene="scene" in info and "lane" in info["scene"], collisions="collisions" in info, exports="exports" in info)
        return 42.0
    env3 = gym_env.CoSimEnv(carla_port=port, reward_fn=my_reward, episode_seconds=2.0)
    env3.reset()
    _, r, _t, _u, info3 = env3.step(np.zeros(2, dtype=np.float32))
    env3.close()
    check("自定义 reward_fn 被用，拿到 info + 完整的 scene 和这一步的碰撞；返回的 info 里没有它们",
          r == 42.0 and all(seen.values()) and "scene" not in info3 and "collisions" not in info3, (r, seen))
    try:
        gym_env.CoSimEnv(carla_port=port, obs_fn=lambda e, s, t: np.zeros(3))
        msg = ""
    except ValueError as e:
        msg = str(e)
    check("obs_fn 不给 observation_space：说清楚", "observation_space" in msg, msg)
    env4 = gym_env.CoSimEnv(carla_port=port, config={"carsim": {"mock": True, "chrono": False, "mock_init_speed": 20.0}}, episode_seconds=2.0)
    try:
        env4.reset()
        msg = ""
    except RuntimeError as e:
        msg = str(e)
    finally:
        env4.close()
    check("导入变量个数和 action_space 不一致：说清楚是几个 / 几维", "3 个导入变量" in msg and "2 维" in msg, msg)
    w.wait_for_tick()
    check("... 没有留下车", vehicles_settled(w, 0) == 0, vehicles(w))

    if "--speed" in sys.argv:   # 零动作会很快出线：回合结束就 reset()，只给 step() 计时（reset 另外量一次）
        for nr in (True, False):
            e = gym_env.CoSimEnv(carla_port=port, episode_seconds=60.0, no_render=nr)
            try:
                t0 = time.time()
                e.reset()
                t_reset = time.time() - t0
                spent, n = 0.0, 0
                while n < 100:
                    t1 = time.time()
                    _obs, _r, term, trunc, _info = e.step(np.zeros(2, dtype=np.float32))
                    spent += time.time() - t1
                    n += 1
                    if term or trunc:
                        e.reset()
                print("INFO no_render=%s：step() %.1f 步/秒（一步 = 0.05 s 仿真），reset() %.1f s" % (nr, n / spent, t_reset))
            finally:
                e.close()
    print("FAILED: %s" % FAILS if FAILS else "ALL GYM CARLA TESTS PASSED")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
