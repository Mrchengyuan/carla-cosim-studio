"""几个 CoSimEnv 并行（gymnasium.vector.AsyncVectorEnv，每个环境一个进程、一个 CARLA 服务器）。

检查：两个环境在各自的端口、各自的进程里跑，观测堆成 (2, n)；两个环境用不同的初始车速（reset 的 options 逐个给不了，
所以各自的构造参数不同）互不影响；动作逐个环境生效；一个环境的回合结束后，下一次 step 里它自动 reset（gymnasium 向量环境的行为），
另一个不受影响；close() 后两个 CARLA 都放回异步模式；--speed 时量总的步数 / 秒。

需要两个私有端口的 CARLA（scripts/carla_pool.sh start 2200 2300）。

    python tests/test_gym_vector_carla.py [--ports 2200,2300] [--speed]
"""
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

import carla  # noqa: E402
from gymnasium.vector import AsyncVectorEnv  # noqa: E402

import gym_env  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    shown = "" if isinstance(detail, str) and detail == "" else "  " + str(detail)
    print(("PASS " if cond else "FAIL ") + name + shown, flush=True)
    if not cond:
        FAILS.append(name)


def make(port, speed):
    return lambda: gym_env.CoSimEnv(carla_port=port, init_speed=speed, episode_seconds=60.0)


def main():
    ports = [int(p) for p in sys.argv[sys.argv.index("--ports") + 1].split(",")] if "--ports" in sys.argv else [2200, 2300]
    speeds = [20.0, 10.0]
    envs = AsyncVectorEnv([make(p, s) for p, s in zip(ports, speeds)])
    failed = True
    try:
        obs, info = envs.reset(seed=0)
        n = envs.single_observation_space.shape[0]
        check("reset：观测堆成 (2, %d)、有限" % n, obs.shape == (2, n) and np.all(np.isfinite(obs)), obs.shape)
        ex = info.get("exports")
        check("info 里逐个环境的 exports（Vx 是各自的初始车速：72 和 36 km/h）",
              ex is not None and abs(float(ex["Vx"][0]) - 72.0) < 1.5 and abs(float(ex["Vx"][1]) - 36.0) < 1.5,
              None if ex is None else [round(float(v), 1) for v in ex["Vx"]])
        # 两个环境给不同的动作：第一个直行，第二个猛打方向 → 第二个先出线、结束，第一个不受影响
        acts = np.array([[0.0, 0.0], [0.0, 0.09]], dtype=np.float32)
        ended, steps = [False, False], 0
        while steps < 300 and not ended[1]:
            obs, r, term, trunc, info = envs.step(acts)
            steps += 1
            ended = [bool(a or b) for a, b in zip(term, trunc)]
        check("第二个环境猛打方向：先结束（终止），第一个没有结束", ended[1] and not ended[0] and bool(term[1]), (steps, ended, list(term)))
        scenes = envs.call("scene")
        check("envs.call('scene')：逐个环境的完整场景（第二个已经不在车道上：lane 是 None，第一个还在）",
              len(scenes) == 2 and scenes[0].get("lane") is not None and scenes[1].get("lane") is None,
              [None if s.get("lane") is None else "lane" for s in scenes])
        t_before = float(info["t"][0])
        # gymnasium 1.x 的向量环境在结束的下一次 step 里 reset 那个环境（它的动作被忽略），其余环境照常前进
        obs, r, term, trunc, info = envs.step(acts)
        check("下一次 step：结束的那个环境重新开始（t 回到 0、奖励 0、没有终止），另一个照常前进",
              float(info["t"][1]) < 1e-6 and r[1] == 0.0 and not term[1] and float(info["t"][0]) > t_before,
              ([round(float(x), 2) for x in info["t"]], list(r), list(term), t_before))
        if "--speed" in sys.argv:
            envs.reset(seed=1)
            t1, k = time.time(), 0
            for _ in range(60):
                envs.step(np.zeros((2, 2), dtype=np.float32))
                k += 1
            print("INFO %d 个环境并行：%.1f 步/秒（合计，一步 = 0.05 s 仿真）" % (len(ports), len(ports) * k / (time.time() - t1)))
        failed = False
    except BaseException:
        import traceback
        traceback.print_exc()
        FAILS.append("exception")
    finally:
        envs.close(terminate=failed)   # 出了错就直接结束各个进程，不等它们
    for p in ports:
        client = carla.Client("localhost", p)
        client.set_timeout(30.0)
        w = client.get_world()
        check("close() 后端口 %d 的 CARLA 放回异步模式" % p, not w.get_settings().synchronous_mode, w.get_settings().synchronous_mode)
    print("FAILED: %s" % FAILS if FAILS else "ALL GYM VECTOR TESTS PASSED")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
