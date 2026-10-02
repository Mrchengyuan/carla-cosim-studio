"""训练多拓扑 NMPC 的价值函数（nmpc_value.py）：离线仿真里随机场景，多进程并行采集，GPU 上拟合。

    python controllers/topo_nmpc/train_value.py [--rounds 3] [--episodes 96] [-j 32] [--out value_net.npz]
    python controllers/topo_nmpc/train_value.py --eval [--episodes 64] [-j 32]   # 只评估：手写车道价值 vs 学习价值
    python controllers/topo_nmpc/train_value.py --replay 种子 [--learned]        # 重放一个场景（看碰撞原因）

采集（每轮）：每个进程开若干个随机场景（tests/topo_nmpc_sim.py 的道路和被控车辆），用控制器本身开 25 s，
    每帧记下当前状态的特征（控制器算好的 value_features）和回报（nmpc_value.reward：车速、舒适、离出发车道、碰撞）。
    探索：每 3 s 给“换到左 / 右边车道”的方案随机加减一些代价（σ = EXPLORE），换道的时机和选择因此多样。
    第 0 轮用手写的车道价值，之后各轮用上一轮学到的价值函数（策略改进）。数据只在内存里，不写盘。
随机场景：2~5 条同向车道、出发车道随机，路由直道和 R 80~400 的弯随机拼成；1~8 辆交通车（各自车道、
    6~30 m/s，用 IDM 跟车：前面有车（包括自车）会减速，不会无脑追尾）；25% 封一段车道（锥桶），20% 有车切入，
    15% 有红灯（红 4~10 s 再绿），10% 前车急刹；自车初速 10~25 m/s、期望车速 60~100 km/h。
拟合：n 步 TD（n = NSTEP 帧）+ 目标网络，多次外循环；跑满时长的回合在末尾用价值自举，碰撞的回合到此为止。
    网络 29 → 128 → 128 → 1（ReLU），Adam，在 GPU 上（没有 GPU 用 CPU）。
保存：.npz（W0/b0 ...、特征均值方差、γ、轮数、样本数），几十 KB。
"""
import contextlib
import importlib.util
import io
import math
import multiprocessing as mp
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.abspath(os.path.join(HERE, "..", "..", "tests"))
EPISODE_S = 25.0
DT = 0.05
EXPLORE = 150.0       # 探索：换道方案代价的随机偏置标准差（每 3 s 重抽）
NSTEP = 40            # n 步 TD（2 s）
TRAIN_SEED0 = 1000    # 训练场景的随机种子从这里起；评估用 900000 起（不重叠）
EVAL_SEED0 = 900000


def load_controller(value_path):
    sys.path.insert(0, HERE)
    sys.path.insert(0, TESTS)
    spec = importlib.util.spec_from_file_location("topo_nmpc_controller_train", os.path.join(HERE, "controller.py"))
    C = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(C)
    C.VALUE_NET = value_path or ""
    C.DRAW = False
    C.PRINT_EVERY = 1e9
    return C


# ---------------------------------------------------------------------- random scenarios
def make_sim(rng):
    from topo_nmpc_sim import Actor, Road, Sim, W
    n_left, n_right = int(rng.integers(0, 3)), int(rng.integers(0, 3))
    if n_left + n_right == 0:
        n_left = 1
    segs, total = [], 0.0
    while total < 1400.0:
        L = float(rng.uniform(80, 250))
        k = 0.0 if rng.random() < 0.5 else float(rng.choice([-1, 1]) / rng.uniform(80, 400))
        segs.append((L, k))
        total += L
    road = Road(segs, lanes=(-n_right, n_left))
    lanes = list(range(-n_right, n_left + 1))
    actors = []
    for _ in range(int(rng.integers(1, 9))):
        ln = int(rng.choice(lanes))
        s0 = float(rng.uniform(-50, 250))
        if ln == 0 and abs(s0 - 5.0) < 25.0:
            s0 += 40.0
        if any(a.d == ln * W and abs(a.s - s0) < 15.0 for a in actors):
            continue
        actors.append(Actor(s0, ln, float(rng.uniform(6, 30))))
    if rng.random() < 0.25:   # 封一段车道
        ln = int(rng.choice(lanes))
        s0 = float(rng.uniform(100, 250))
        for i in range(int(rng.uniform(60, 150) / 4.0)):
            frac = min(1.0, i * 4.0 / 30.0)
            actors.append(Actor(s0 + 4.0 * i, d=ln * W - W / 2 + frac * (W - 0.3), kind="static", length=0.4, width=0.4))
    if rng.random() < 0.2 and len(lanes) > 1:   # 切入
        side = int(rng.choice([l for l in lanes if l != 0 and abs(l) == 1] or [lanes[0]]))
        actors.append(Actor(float(rng.uniform(25, 60)), side, float(rng.uniform(10, 20)), cut=(25.0, 0, 2.0)))
    if rng.random() < 0.1:    # 前车急刹
        actors.append(Actor(float(rng.uniform(40, 80)), 0, float(rng.uniform(12, 22)), brake_at=50.0,
                            brake=float(rng.uniform(3, 7))))
    light = None
    if rng.random() < 0.15:
        red = float(rng.uniform(4, 10))
        light = (float(rng.uniform(150, 300)), [(0.0, red, "red"), (red, 1e9, "green")])
    sim = Sim(road, start_lane=0, v0=float(rng.uniform(10, 25)), actors=actors, light=light)
    return sim, float(rng.uniform(60, 100))


def step_actors_idm(sim, dt):
    """交通车按 IDM 跟车（前方同车道最近的车，包括自车）；锥桶、行人、急刹 / 切入中的车按它们自己的规则。"""
    from topo_nmpc_sim import W
    s_e, d_e = sim.ego_frenet()
    for a in sim.actors:
        if a.kind != "vehicle" or a.braking or (a.cut_t is not None):
            continue
        if not hasattr(a, "v0"):
            a.v0 = max(a.v, 1.0)
        lead_gap, lead_v = None, None
        for b in sim.actors:
            if b is a or abs(b.d - a.d) > W / 2 or b.s <= a.s:
                continue
            g = b.s - a.s - (a.length + b.length) / 2
            if lead_gap is None or g < lead_gap:
                lead_gap, lead_v = g, b.v
        if abs(d_e - a.d) < W / 2 and s_e - 1.4 > a.s:
            g = s_e - 1.4 - a.s - (a.length + 4.6) / 2
            if lead_gap is None or g < lead_gap:
                lead_gap, lead_v = g, sim.plant.vx
        acc = 1.5 * (1 - (a.v / a.v0) ** 4)
        if lead_gap is not None:
            s_star = 2.0 + a.v * 1.2 + a.v * (a.v - lead_v) / (2 * math.sqrt(1.5 * 3.0))
            acc -= 1.5 * (max(s_star, 0.0) / max(lead_gap, 0.5)) ** 2
        a.v = max(0.0, a.v + max(acc, -8.0) * dt)


def episode(args):
    seed, value_path, explore = args
    rng = np.random.default_rng(seed)
    C = load_controller(value_path)
    sim, vt_kmh = make_sim(rng)
    C.TARGET_KMH = vt_kmh
    vt = vt_kmh / 3.6
    ctl = C.Controller()
    F, R = [], []
    collided = False
    home0 = 0
    next_bias = 0.0
    lanes_seen, ay_sq, n = set(), 0.0, 0
    min_clear = float("inf")
    with contextlib.redirect_stdout(io.StringIO()):
        ctl.reset()
        while sim.t < EPISODE_S - 1e-9:
            if explore and sim.t >= next_bias:
                next_bias = sim.t + 3.0
                ctl.explore_bias = {("lane", ctl.lane_idx + r): float(rng.normal(0.0, EXPLORE)) for r in (-1, 1)}
            sc = sim.scene()
            out = ctl.control(sim.plant.exports(), sim.t, DT, sc)
            f = ctl.value_features
            sim.plant.step(out[0], out[1], DT)
            step_actors_idm(sim, DT)
            for a in sim.actors:
                a.step(DT, sim.s_ego)
            sim.t += DT
            s, d = sim.ego_frenet()
            clear = sim.clearance()
            min_clear = min(min_clear, clear)
            kap = float(np.interp(s, sim.road.s, sim.road.kap))
            p = sim.plant
            ay_ex = p.vx * p.r - p.vx ** 2 * kap
            lane = sim.lane_of(d)
            lanes_seen.add(lane)
            collided = clear <= 0.0
            if f is not None:
                F.append(f)
                R.append(nmpc_value_reward(p.vx, vt, ay_ex, lane - home0, collided))
            ay_sq += ay_ex ** 2
            n += 1
            if collided:
                break
    v_mean = float(np.mean([x[0] for x in F])) if F else 0.0
    return {"seed": seed, "F": np.asarray(F), "R": np.asarray(R), "collided": collided,
            "v_ratio": v_mean, "min_clear": min_clear, "lane_changes": ctl.lane_changes,
            "ay_rms": math.sqrt(ay_sq / max(n, 1)), "solve_ms": 1000 * ctl.solve_s / max(ctl.n_calls, 1)}


def nmpc_value_reward(*a):
    sys.path.insert(0, HERE)
    import nmpc_value
    return nmpc_value.reward(*a)


# ---------------------------------------------------------------------- fitting
Y_SCALE = 100.0        # 回报之和约 0 ~ 200：拟合时除以它（网络输出在 ±2 左右），存权重时乘回去


def fit(episodes, out_path, rounds_done, outer=8, epochs=10, first_epochs=40, log=print):
    import torch
    sys.path.insert(0, HERE)
    import nmpc_value
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    g = nmpc_value.GAMMA
    F = np.concatenate([e["F"] for e in episodes])
    mean, std = F.mean(0), F.std(0) + 1e-3
    net = torch.nn.Sequential(torch.nn.Linear(nmpc_value.FEATURES, 128), torch.nn.ReLU(),
                              torch.nn.Linear(128, 128), torch.nn.ReLU(), torch.nn.Linear(128, 1)).to(dev).double()
    opt = torch.optim.Adam(net.parameters(), lr=1e-3)
    tm, ts = torch.tensor(mean, device=dev), torch.tensor(std, device=dev)

    def V(x):
        with torch.no_grad():
            return Y_SCALE * net((torch.tensor(x, device=dev) - tm) / ts)[:, 0].cpu().numpy()

    # 起点：蒙特卡洛回报（跑满的回合末尾按“一直保持最后的回报”估计）
    targets = []
    for e in episodes:
        r = e["R"]
        G = np.zeros(len(r))
        acc = 0.0 if e["collided"] else r[-1] / (1 - g)
        for t in range(len(r) - 1, -1, -1):
            acc = r[t] + g * acc
            G[t] = acc
        targets.append(G)
    for it in range(outer):
        X = torch.tensor((F - mean) / std, device=dev)
        Y = torch.tensor(np.concatenate(targets) / Y_SCALE, device=dev)
        n = len(X)
        for ep in range(first_epochs if it == 0 else epochs):
            perm = torch.randperm(n, device=dev)
            tot = 0.0
            for i in range(0, n, 2048):
                idx = perm[i:i + 2048]
                loss = torch.mean((net(X[idx])[:, 0] - Y[idx]) ** 2)
                opt.zero_grad()
                loss.backward()
                opt.step()
                tot += float(loss) * len(idx)
        log("  拟合 第 %d/%d 次：均方根误差 %.2f（目标均值 %.1f）" % (
            it + 1, outer, Y_SCALE * math.sqrt(tot / n), Y_SCALE * float(Y.mean())))
        # n 步 TD 目标（目标网络 = 这一次拟合完的网络）
        targets = []
        for e in episodes:
            r, Fe = e["R"], e["F"]
            v = V(Fe)
            m = len(r)
            disc = g ** np.arange(NSTEP)
            G = np.zeros(m)
            for t in range(m):
                end = min(t + NSTEP, m)
                G[t] = float(np.dot(disc[:end - t], r[t:end]))
                if end < m:
                    G[t] += g ** NSTEP * v[end]
                elif not e["collided"]:
                    G[t] += g ** (end - t) * v[m - 1]
            targets.append(G)
    Ws = [l for l in net if isinstance(l, torch.nn.Linear)]
    arrays = {}
    for i, l in enumerate(Ws):
        k = Y_SCALE if i == len(Ws) - 1 else 1.0     # 最后一层乘回 Y_SCALE：推理直接得到回报之和
        arrays["W%d" % i] = k * l.weight.detach().cpu().numpy().T.copy()
        arrays["b%d" % i] = k * l.bias.detach().cpu().numpy().copy()
    np.savez(out_path, mean=mean, std=std, gamma=np.array(g), rounds=np.array(rounds_done), samples=np.array(len(F)),
             **arrays)
    log("  保存 %s（%d 个样本）" % (out_path, len(F)))


def summarize(res, name, log=print):
    col = sum(1 for r in res if r["collided"])
    log("%s：%d 回合，碰撞 %d，平均 车速/期望 %.3f，最小间距中位数 %.2f m，换道 平均 %.2f 次，侧向冲击 rms %.2f m/s²，"
        "求解 %.0f ms" % (name, len(res), col, np.mean([r["v_ratio"] for r in res]),
                          float(np.median([r["min_clear"] for r in res])), np.mean([r["lane_changes"] for r in res]),
                          np.mean([r["ay_rms"] for r in res]), np.mean([r["solve_ms"] for r in res])))
    if col:
        log("  碰撞的场景（种子，可单独重放：--replay 种子）：%s" % ", ".join(str(r["seed"]) for r in res if r["collided"]))
    return col


def replay(seed, value_path):
    """重放一个场景，打印控制器的决策和碰撞前后的情况。"""
    rng = np.random.default_rng(seed)
    C = load_controller(value_path)
    sim, vt_kmh = make_sim(rng)
    C.TARGET_KMH = vt_kmh
    C.PRINT_EVERY = 2.0
    ctl = C.Controller()
    ctl.reset()
    print("场景 %d：期望 %.0f km/h，车道 %s，交通 %s" % (seed, vt_kmh, sim.road.lanes, [
        (round(a.s), round(a.d, 1), round(a.v, 1), a.kind) for a in sim.actors if a.kind != "static"]))
    while sim.t < EPISODE_S - 1e-9:
        out = ctl.control(sim.plant.exports(), sim.t, DT, sim.scene())
        sim.plant.step(out[0], out[1], DT)
        step_actors_idm(sim, DT)
        for a in sim.actors:
            a.step(DT, sim.s_ego)
        sim.t += DT
        c = sim.clearance()
        if c <= 0.0:
            s, d = sim.ego_frenet()
            near = sorted(sim.actors, key=lambda a: abs(a.s - s) + abs(a.d - d))[:3]
            print("t = %.2f s 碰撞：自车 s %.1f d %.2f v %.1f；附近 %s" % (
                sim.t, s, d, sim.plant.vx, [(round(a.s - s, 1), round(a.d, 1), round(a.v, 1), a.kind) for a in near]))
            break


def main():
    jobs = int(sys.argv[sys.argv.index("-j") + 1]) if "-j" in sys.argv else (os.cpu_count() or 1)
    eps = int(sys.argv[sys.argv.index("--episodes") + 1]) if "--episodes" in sys.argv else 96
    rounds = int(sys.argv[sys.argv.index("--rounds") + 1]) if "--rounds" in sys.argv else 3
    out = sys.argv[sys.argv.index("--out") + 1] if "--out" in sys.argv else os.path.join(HERE, "value_net.npz")
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    ctx = mp.get_context("spawn")
    if "--replay" in sys.argv:
        replay(int(sys.argv[sys.argv.index("--replay") + 1]), out if "--learned" in sys.argv else "")
        return
    if "--eval" in sys.argv:
        seeds = [EVAL_SEED0 + i for i in range(eps)]
        with ctx.Pool(jobs) as pool:
            base = pool.map(episode, [(s, "", False) for s in seeds])
            learned = pool.map(episode, [(s, out, False) for s in seeds])
        summarize(base, "手写车道价值")
        summarize(learned, "学习价值函数")
        diff = [l["v_ratio"] - b["v_ratio"] for b, l in zip(base, learned)]
        print("同一场景两者对比：学习价值函数的 车速/期望 平均 %+.3f（更快 %d 个、更慢 %d 个、差不多 %d 个）" % (
            float(np.mean(diff)), sum(d > 0.01 for d in diff), sum(d < -0.01 for d in diff),
            sum(abs(d) <= 0.01 for d in diff)))
        return
    data = []
    t0 = time.time()
    with ctx.Pool(jobs) as pool:
        for rd in range(rounds):
            vp = out if rd > 0 else ""
            seeds = [TRAIN_SEED0 + rd * 10000 + i for i in range(eps)]
            res = pool.map(episode, [(s, vp, True) for s in seeds])
            print("第 %d 轮采集（%s，%.0f s）：" % (rd, "学习价值函数" if vp else "手写车道价值", time.time() - t0), flush=True)
            summarize(res, "  本轮")
            data += [r for r in res if len(r["R"]) > NSTEP]
            fit(data, out, rd + 1)
            print("  已用 %.0f s" % (time.time() - t0), flush=True)


if __name__ == "__main__":
    main()
