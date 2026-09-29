"""
学习世界模型: 用 Chrono BMW_E90 的数据训练一个神经网络, 替代 3DOF 物理模型给 KMPPI 做 rollout ("做梦")。

模型结构 (有限历史监督动力学模型，用于在线规划中的未来推演；不是 Dreamer):
  * 只学车身系速度动力学 [vx, vy, r] 的一步增量, 位姿 [x, y, yaw] 用已知运动学公式做数值积分;
  * 输入 = 最近 H 步的 [vx, vy, r, ax_cmd, delta_cmd] (H>1 时模型可从历史推断侧倾/轮胎弛豫/转向柔度等隐状态);
  * 训练用多步展开损失 (rollout_len 步), 抑制 33 步 rollout 的误差累积;
  * 全部归一化, MLP + SiLU。

KMPPI 用法: LearnedDynamics 是有状态的 dynamics: 每次 rollout 开始前用真实历史初始化 (K, H-1, 5) 的历史缓冲,
之后用模型自己的预测填充历史。
"""
from __future__ import annotations

import os
import time
from typing import List, Optional, Sequence, Tuple

import numpy as np
import torch
import torch.nn as nn

from dream_config import WorldModelConfig

FEAT = 5   # vx, vy, r, ax, delta
OUT = 3    # dvx, dvy, dr


class WorldModelNet(nn.Module):
    def __init__(self, history: int, hidden: int, layers: int):
        super().__init__()
        dims = [history * FEAT] + [hidden] * layers + [OUT]
        mods = []
        for i in range(len(dims) - 1):
            mods.append(nn.Linear(dims[i], dims[i + 1]))
            if i < len(dims) - 2:
                mods.append(nn.SiLU())
        self.net = nn.Sequential(*mods)

    def forward(self, x):
        return self.net(x)


def episode_to_arrays(states: np.ndarray, actions: np.ndarray):
    """states (n+1,6), actions (n,2) -> pairs P (n,5) = [vel_k, a_k], V (n+1,3)。"""
    P = np.concatenate([states[:-1, 3:6], actions], axis=1).astype(np.float32)
    V = states[:, 3:6].astype(np.float32)
    return P, V


class WorldModel:
    def __init__(self, cfg: WorldModelConfig, dt: float, device: str = "cpu"):
        self.cfg = cfg
        self.dt = dt
        self.H = cfg.history
        self.device = torch.device(device)
        torch.manual_seed(cfg.seed)
        self.net = WorldModelNet(cfg.history, cfg.hidden, cfg.layers).to(self.device)
        self.in_mean = torch.zeros(FEAT, device=self.device)
        self.in_std = torch.ones(FEAT, device=self.device)
        self.out_std = torch.ones(OUT, device=self.device)
        self.train_log: List[dict] = []

    def to(self, device: str) -> "WorldModel":
        self.device = torch.device(device)
        self.net.to(self.device)
        self.in_mean, self.in_std, self.out_std = self.in_mean.to(self.device), self.in_std.to(self.device), self.out_std.to(self.device)
        return self

    # ------------------------------------------------------------------ 数据
    def _build_dataset(self, episodes: Sequence[Tuple[np.ndarray, np.ndarray]], L: int):
        P_list, Vn_list, starts = [], [], []
        offset = 0
        for S, A in episodes:
            P, V = episode_to_arrays(S, A)
            n = P.shape[0]
            P_list.append(P)
            Vn_list.append(V[1:])
            lo, hi = self.H - 1, n - L          # 窗口起点 i: 需要 i-(H-1) >= 0 且 i+L-1 <= n-1
            if hi >= lo:
                starts.append(np.arange(lo, hi + 1) + offset)
            offset += n
        P_all = torch.from_numpy(np.concatenate(P_list, axis=0)).to(self.device)
        Vn_all = torch.from_numpy(np.concatenate(Vn_list, axis=0)).to(self.device)
        starts = torch.from_numpy(np.concatenate(starts, axis=0)).to(self.device)
        return P_all, Vn_all, starts

    def _normalize_in(self, x_pairs):      # (..., H, 5) -> (..., H*5)
        z = (x_pairs - self.in_mean) / self.in_std
        return z.reshape(*z.shape[:-2], self.H * FEAT)

    def _predict_dvel(self, vel, act, hist):
        """vel (B,3), act (B,2), hist (B,H-1,5) -> dvel (B,3) 实际单位。"""
        cur = torch.cat([vel, act], dim=-1).unsqueeze(1)          # (B,1,5)
        pairs = torch.cat([hist, cur], dim=1) if self.H > 1 else cur  # 时间顺序: 旧 -> 新
        return self.net(self._normalize_in(pairs)) * self.out_std

    @staticmethod
    def _push_hist(hist, vel, act):
        if hist.shape[1] == 0:
            return hist
        new = torch.cat([vel, act], dim=-1).unsqueeze(1)
        return torch.cat([hist[:, 1:], new], dim=1)

    # ------------------------------------------------------------------ 训练
    def fit(self, episodes, epochs: Optional[int] = None, lr: Optional[float] = None, verbose=True, val_episodes=None):
        """Train after planning; restore the caller's mode and gradient flags."""
        flags = [p.requires_grad for p in self.net.parameters()]
        was_training = self.net.training
        try:
            self.net.train()
            for p in self.net.parameters():
                p.requires_grad_(True)
            with torch.enable_grad():
                return self._fit(episodes, epochs, lr, verbose, val_episodes)
        finally:
            for p, flag in zip(self.net.parameters(), flags):
                p.requires_grad_(flag)
            self.net.train(was_training)

    def _fit(self, episodes, epochs=None, lr=None, verbose=True, val_episodes=None):
        cfg = self.cfg
        L = cfg.rollout_len
        epochs = cfg.epochs if epochs is None else epochs
        lr = cfg.lr if lr is None else lr
        P_all, Vn_all, starts = self._build_dataset(episodes, L)
        # 归一化统计 (只在首次训练时设定, 之后保持不变以便增量微调)
        if not self.train_log:
            self.in_mean = P_all.mean(0)
            self.in_std = P_all.std(0) + 1e-6
            dV = Vn_all - P_all[:, :3]
            self.out_std = dV.std(0) + 1e-6
        opt = torch.optim.Adam(self.net.parameters(), lr=lr, weight_decay=cfg.weight_decay)
        n_win = starts.shape[0]
        iters_per_epoch = max(1, n_win // cfg.batch)
        sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=epochs * iters_per_epoch)
        g = torch.Generator().manual_seed(cfg.seed + len(self.train_log))
        t0 = time.time()
        for ep in range(epochs):
            perm = starts[torch.randperm(n_win, generator=g).to(self.device)]
            tot = 0.0
            for it in range(iters_per_epoch):
                idx = perm[it * cfg.batch:(it + 1) * cfg.batch]
                B = idx.shape[0]
                hist = torch.stack([P_all[idx - self.H + 1 + j] for j in range(self.H - 1)], dim=1) if self.H > 1 else torch.zeros(B, 0, FEAT, device=self.device)
                vel = P_all[idx, :3]
                loss = 0.0
                for j in range(L):
                    act = P_all[idx + j, 3:5]
                    dvel = self._predict_dvel(vel, act, hist)
                    hist = self._push_hist(hist, vel, act)
                    vel = vel + dvel
                    target = Vn_all[idx + j]
                    loss = loss + torch.mean(((vel - target) / self.out_std) ** 2)
                loss = loss / L
                opt.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self.net.parameters(), 5.0)
                opt.step()
                sched.step()
                tot += float(loss.detach())
            rec = dict(epoch=ep, train_loss=tot / iters_per_epoch, wall=time.time() - t0)
            if val_episodes is not None and (ep % 10 == 9 or ep == epochs - 1):
                rec.update(self.evaluate(val_episodes, horizons=(1, 33)))
            self.train_log.append(rec)
            if verbose and (ep % 10 == 9 or ep == epochs - 1):
                msg = f"  epoch {ep+1:3d}/{epochs}  loss {rec['train_loss']:.4f}"
                if "pos_err_33" in rec:
                    msg += f"  val 1-step vel RMSE [{rec['vel_rmse_1'][0]:.4f} {rec['vel_rmse_1'][1]:.4f} {rec['vel_rmse_1'][2]:.5f}]  33-step open-loop pos err {rec['pos_err_33']:.3f} m"
                print(msg + f"  ({rec['wall']:.0f}s)")
        return self.train_log[-1]

    # ------------------------------------------------------------------ 评估
    @torch.no_grad()
    def evaluate(self, episodes, horizons=(1, 33), max_windows=4000):
        """开环多步预测误差: 从真实状态出发, 用真实动作展开 h 步, 比较速度 RMSE 和位置误差。"""
        out = {"position_error_definition": "actual held-out pose displacement in initial yaw frame", "kinematic_drift_definition": "both poses integrated from velocities (legacy metric)"}
        pose_all = torch.from_numpy(np.concatenate([S[:-1, :3] for S, _ in episodes]).astype(np.float32)).to(self.device)
        pose_next = torch.from_numpy(np.concatenate([S[1:, :3] for S, _ in episodes]).astype(np.float32)).to(self.device)
        for h in horizons:
            P_all, Vn_all, starts = self._build_dataset(episodes, h)
            if starts.shape[0] > max_windows:
                starts = starts[torch.linspace(0, starts.shape[0] - 1, max_windows).long().to(self.device)]
            idx = starts
            B = idx.shape[0]
            hist = torch.stack([P_all[idx - self.H + 1 + j] for j in range(self.H - 1)], dim=1) if self.H > 1 else torch.zeros(B, 0, FEAT, device=self.device)
            vel = P_all[idx, :3]
            # 位姿: 模型与真值都从原点/零航向出发, 只比较相对位移
            pose_m = torch.zeros(B, 3, device=self.device)
            pose_t = torch.zeros(B, 3, device=self.device)
            for j in range(h):
                act = P_all[idx + j, 3:5]
                dvel = self._predict_dvel(vel, act, hist)
                hist = self._push_hist(hist, vel, act)
                vel_new = vel + dvel
                pose_m = self._integrate_pose(pose_m, vel, vel_new)
                vt, vt_new = (P_all[idx + j, :3], Vn_all[idx + j])
                pose_t = self._integrate_pose(pose_t, vt, vt_new)
                vel = vel_new
            err_v = vel - Vn_all[idx + h - 1]
            out[f"vel_rmse_{h}"] = torch.sqrt((err_v ** 2).mean(0)).tolist()
            legacy_error = torch.linalg.vector_norm(pose_m[:, :2] - pose_t[:, :2], dim=1)
            start_pose, end_pose = pose_all[idx], pose_next[idx + h - 1]
            dx, dy = end_pose[:, 0] - start_pose[:, 0], end_pose[:, 1] - start_pose[:, 1]
            c, sn = torch.cos(start_pose[:, 2]), torch.sin(start_pose[:, 2])
            displacement = torch.stack([c * dx + sn * dy, -sn * dx + c * dy], dim=1)
            errors = torch.linalg.vector_norm(pose_m[:, :2] - displacement, dim=1)
            out[f"pos_err_{h}"] = float(errors.mean())
            out[f"pos_err_{h}_max"] = float(errors.max())
            # quantile on CPU also supports MPS releases lacking this operation.
            out[f"pos_err_{h}_p95"] = float(torch.quantile(errors.cpu(), .95))
            out[f"kinematic_drift_err_{h}"] = float(legacy_error.mean())
        return out

    def _integrate_pose(self, pose, vel0, vel1):
        """梯形积分: x += dt*(vx cos yaw - vy sin yaw) ..., 速度取步初/步末平均, 航向取中点。"""
        dt = self.dt
        vx = 0.5 * (vel0[:, 0] + vel1[:, 0])
        vy = 0.5 * (vel0[:, 1] + vel1[:, 1])
        r = 0.5 * (vel0[:, 2] + vel1[:, 2])
        yaw_mid = pose[:, 2] + 0.5 * dt * r
        x = pose[:, 0] + dt * (vx * torch.cos(yaw_mid) - vy * torch.sin(yaw_mid))
        y = pose[:, 1] + dt * (vx * torch.sin(yaw_mid) + vy * torch.cos(yaw_mid))
        yaw = pose[:, 2] + dt * r
        return torch.stack([x, y, yaw], dim=1)

    # ------------------------------------------------------------------ 保存/加载
    def save(self, path: Optional[str] = None):
        path = path or self.cfg.path
        os.makedirs(os.path.dirname(path), exist_ok=True)
        torch.save(dict(cfg=self.cfg.__dict__, dt=self.dt, state_dict={k: v.cpu() for k, v in self.net.state_dict().items()},
                        in_mean=self.in_mean.cpu(), in_std=self.in_std.cpu(), out_std=self.out_std.cpu(), train_log=self.train_log), path)

    @classmethod
    def load(cls, path: str, device: str = "cpu") -> "WorldModel":
        blob = torch.load(path, weights_only=False, map_location="cpu")
        cfg = WorldModelConfig(**blob["cfg"])
        wm = cls(cfg, blob["dt"], device=device)
        wm.net.load_state_dict(blob["state_dict"])
        wm.in_mean, wm.in_std, wm.out_std = blob["in_mean"].to(wm.device), blob["in_std"].to(wm.device), blob["out_std"].to(wm.device)
        wm.train_log = blob.get("train_log", [])
        wm.net.eval()
        return wm

    # ------------------------------------------------------------------ 稳态表 (给参考 vy 用)
    @torch.no_grad()
    def steady_state_vy_table(self, kappa_grid: np.ndarray, speed: float, delta_max: float, ax_max: float, T_settle: float = 4.0):
        """对每个曲率, 用二分法找到使稳态 r = v*kappa 的恒定 delta, 记录稳态 vy。全部曲率批量并行。"""
        dev = self.device
        kappa = torch.tensor(np.asarray(kappa_grid, dtype=np.float32), device=dev)
        B = kappa.shape[0]
        n = int(round(T_settle / self.dt))
        lo = torch.full((B,), -delta_max, device=dev)
        hi = torch.full((B,), delta_max, device=dev)
        r_target = speed * kappa

        def simulate(delta):
            vel = torch.tensor([[speed, 0.0, 0.0]], device=dev).repeat(B, 1)
            hist = torch.zeros(B, self.H - 1, FEAT, device=dev)
            if self.H > 1:
                hist[:, :, 0] = speed
            for _ in range(n):
                ax = torch.clamp(0.5 * (speed - vel[:, 0]), -ax_max, ax_max)
                act = torch.stack([ax, delta], dim=1)
                dvel = self._predict_dvel(vel, act, hist)
                hist = self._push_hist(hist, vel, act)
                vel = vel + dvel
            return vel

        for _ in range(18):
            mid = 0.5 * (lo + hi)
            vel = simulate(mid)
            too_small = vel[:, 2] < r_target
            lo = torch.where(too_small, mid, lo)
            hi = torch.where(too_small, hi, mid)
        delta = 0.5 * (lo + hi)
        vel = simulate(delta)
        return delta.cpu().numpy(), vel[:, 1].cpu().numpy(), vel[:, 2].cpu().numpy()


    @torch.no_grad()
    def steady_state_vy_table_2d(self, v_grid, kappa_grid, delta_max: float, ax_max: float, T_settle: float = 4.0):
        """二维稳态表 vy[v, kappa]: 对每个 (v, kappa) 组合批量二分 delta 使稳态 r = v*kappa。"""
        dev = self.device
        V, Kp = np.meshgrid(np.asarray(v_grid, dtype=np.float32), np.asarray(kappa_grid, dtype=np.float32), indexing="ij")
        v = torch.tensor(V.reshape(-1), device=dev)
        kappa = torch.tensor(Kp.reshape(-1), device=dev)
        B = v.shape[0]
        n = int(round(T_settle / self.dt))
        lo = torch.full((B,), -delta_max, device=dev)
        hi = torch.full((B,), delta_max, device=dev)
        r_target = v * kappa

        def simulate(delta):
            vel = torch.stack([v, torch.zeros_like(v), torch.zeros_like(v)], dim=1)
            hist = torch.zeros(B, self.H - 1, FEAT, device=dev)
            if self.H > 1:
                hist[:, :, 0] = v.unsqueeze(1)
            for _ in range(n):
                ax = torch.clamp(0.5 * (v - vel[:, 0]), -ax_max, ax_max)
                act = torch.stack([ax, delta], dim=1)
                dvel = self._predict_dvel(vel, act, hist)
                hist = self._push_hist(hist, vel, act)
                vel = vel + dvel
            return vel

        for _ in range(18):
            mid = 0.5 * (lo + hi)
            vel = simulate(mid)
            too_small = vel[:, 2] < r_target
            lo = torch.where(too_small, mid, lo)
            hi = torch.where(too_small, hi, mid)
        vel = simulate(0.5 * (lo + hi))
        return vel[:, 1].cpu().numpy().reshape(V.shape)


class LearnedDynamics:
    """KMPPI 用的有状态 dynamics: step(state (K,6), action (K,2)) -> (K,6)。"""

    def __init__(self, wm: WorldModel, K: int):
        self.wm = wm
        # Forward rollouts use no_grad; polishing uses autograd.grad on actions.
        # Do not freeze a shared model needed by the next lap training pass.
        self.K = K
        self.H = wm.H
        self.dt = wm.dt
        self._hist_seed = np.zeros((max(self.H - 1, 0), FEAT), dtype=np.float32)   # 真实最近历史 (旧->新)
        self.hist = None

    def set_history(self, recent_pairs: np.ndarray):
        """recent_pairs: (H-1, 5) 最近 H-1 步的真实 [vel, action] (旧->新)。不足时用第一行填充。"""
        if self.H <= 1:
            return
        rp = np.asarray(recent_pairs, dtype=np.float32).reshape(-1, FEAT)
        if rp.shape[0] < self.H - 1:
            pad = np.repeat(rp[:1], self.H - 1 - rp.shape[0], axis=0)
            rp = np.concatenate([pad, rp], axis=0)
        self._hist_seed = rp[-(self.H - 1):]

    @property
    def device(self):
        return self.wm.device

    def begin_rollout(self, K: int):
        dev = self.device
        self.hist = torch.from_numpy(np.repeat(self._hist_seed[None], K, axis=0)).to(dev) if self.H > 1 else torch.zeros(K, 0, FEAT, device=dev)

    def _step_torch(self, st, act):
        """st (K,6), act (K,2) 张量 (设备上) -> 下一状态张量。"""
        vel = st[:, 3:6]
        dvel = self.wm._predict_dvel(vel, act, self.hist)
        self.hist = self.wm._push_hist(self.hist, vel, act)
        vel_new = vel + dvel
        vel_new = torch.cat([torch.clamp(vel_new[:, :1], min=0.5), vel_new[:, 1:]], dim=1)
        pose = self.wm._integrate_pose(st[:, 0:3], vel, vel_new)
        yaw = torch.atan2(torch.sin(pose[:, 2]), torch.cos(pose[:, 2]))
        return torch.cat([pose[:, :2], yaw.unsqueeze(1), vel_new], dim=1)

    @torch.no_grad()
    def step(self, state: np.ndarray, action: np.ndarray) -> np.ndarray:
        if self.hist is None or self.hist.shape[0] != state.shape[0]:
            self.begin_rollout(state.shape[0])
        st = torch.from_numpy(np.asarray(state, dtype=np.float32)).to(self.device)
        act = torch.from_numpy(np.asarray(action, dtype=np.float32)).to(self.device)
        return self._step_torch(st, act).cpu().numpy().astype(np.float64)

    @staticmethod
    def _lift(act, u, st, dt, a_min, a_max, ss):
        """导数动作积分 a_{t+1} = clip(a_t + dt*u_t)。ss (KMPPI 改进 A1) 给出时: 转向导数按 delta_lim(v)/delta_lim(v_ref) 缩放,
        转角按当前车速逐步限幅 (刹车进弯时时域内能转得动, 固定限幅做不到)。"""
        if ss is None:
            return torch.minimum(torch.maximum(act + dt * u, a_min), a_max)
        v = torch.clamp(st[:, 3], min=3.0)
        dlim = torch.clamp(ss["c"] / (v * v), ss["dmin"], ss["dmax"])
        ax = torch.clamp(act[:, 0] + dt * u[:, 0], a_min[0], a_max[0])
        g = torch.clamp(dlim / ss["ref"], max=float(ss.get("gmax", 1e9)))
        de = torch.clamp(act[:, 1] + dt * u[:, 1] * g, -dlim, dlim)
        return torch.stack([ax, de], dim=1)

    @torch.no_grad()
    def rollout_cost(self, perturbed_sequence: np.ndarray, *args, **kw) -> np.ndarray:
        U = torch.from_numpy(np.asarray(perturbed_sequence, dtype=np.float32)).to(self.device)
        return self.rollout_cost_torch(U, *args, **kw).cpu().numpy().astype(np.float64)

    def rollout_cost_torch(self, U, state0: np.ndarray, action0: np.ndarray, action_dt: float,
                           action_min: np.ndarray, action_max: np.ndarray, rate_weight: np.ndarray, input_lifting: bool,
                           ref_horizon: np.ndarray, Q: np.ndarray, R: np.ndarray, Q_boundary: float = 0.0, boundary_margin: float = 0.0,
                           frenet: bool = False, Q_lon: float = 0.0, Q_lat: float = 0.0, v_cap: float = 0.0, Q_vcap: float = 0.0,
                           steer_scale=None):
        """整段 KMPPI rollout 留在设备上: 输入 (K,T,nu) 导数动作序列, 返回 (K,) 累计代价 (与 numpy 路径同一公式)。
        ref_horizon 若有 8 列, 第 7/8 列是到左/右赛道边界的距离, 超出 (宽度 - margin) 的部分按 Q_boundary 二次惩罚。"""
        dev = self.device
        K, T, nu = U.shape
        st = torch.from_numpy(np.asarray(state0, dtype=np.float32)).to(dev).reshape(1, -1).repeat(K, 1)
        act = torch.from_numpy(np.asarray(action0, dtype=np.float32)).to(dev).reshape(1, -1).repeat(K, 1)
        a_min = torch.from_numpy(np.asarray(action_min, dtype=np.float32)).to(dev)
        a_max = torch.from_numpy(np.asarray(action_max, dtype=np.float32)).to(dev)
        rw = torch.from_numpy(np.asarray(rate_weight, dtype=np.float32)).to(dev)
        ref = torch.from_numpy(np.asarray(ref_horizon, dtype=np.float32)).to(dev)
        Qt = torch.from_numpy(np.asarray(Q, dtype=np.float32)).to(dev)
        Rt = torch.from_numpy(np.asarray(R, dtype=np.float32)).to(dev)
        use_boundary = ref.shape[1] >= 8 and Q_boundary > 0
        self.begin_rollout(K)
        cost = torch.zeros(K, device=dev)
        for t in range(T):
            u = U[:, t, :]
            if input_lifting:
                act = self._lift(act, u, st, action_dt, a_min, a_max, steer_scale)
                cost = cost + (u * u * rw).sum(1)
            else:
                act = u
            st = self._step_torch(st, act)
            r = ref[t]
            dyaw = st[:, 2] - r[2]
            yaw_err = torch.atan2(torch.sin(dyaw), torch.cos(dyaw))
            err = torch.stack([st[:, 0] - r[0], st[:, 1] - r[1], yaw_err, st[:, 3] - r[3], st[:, 4] - r[4], st[:, 5] - r[5]], dim=1)
            cost = cost + (err * err * Qt).sum(1) + (act * act * Rt).sum(1)
            if frenet or use_boundary:
                e_lat = -(st[:, 0] - r[0]) * torch.sin(r[2]) + (st[:, 1] - r[1]) * torch.cos(r[2])   # 左正
            if frenet:   # 位置代价改在参考 Frenet 系: 横向重, 纵向轻 (Qt[0:2] 此时应为 0)
                e_lon = (st[:, 0] - r[0]) * torch.cos(r[2]) + (st[:, 1] - r[1]) * torch.sin(r[2])
                cost = cost + Q_lon * e_lon * e_lon + Q_lat * e_lat * e_lat
            if Q_vcap > 0:
                over_v = torch.relu(st[:, 3] - v_cap)
                cost = cost + Q_vcap * over_v * over_v
            if use_boundary:
                over_l = torch.relu(e_lat - (r[6] - boundary_margin))
                over_r = torch.relu(-e_lat - (r[7] - boundary_margin))
                cost = cost + Q_boundary * (over_l * over_l + over_r * over_r)
        return cost

    @torch.no_grad()
    def rollout_cost_race(self, perturbed_sequence, *args, **kw) -> np.ndarray:
        U = torch.from_numpy(np.asarray(perturbed_sequence, dtype=np.float32)).to(self.device)
        return self.rollout_cost_race_torch(U, *args, **kw).cpu().numpy().astype(np.float64)

    def rollout_cost_race_torch(self, U, state0, action0, action_dt, action_min, action_max, rate_weight, input_lifting, win, rp,
                                steer_scale=None):
        """竞速模式 (MPCC 式): 代价 = -Q_prog*Δs + Q_lat*e_lat^2 + 边界 + 速度上限 + Q_vy*vy^2 + Q_psi*Δψ^2 + 动作代价。
        win: track_reference.window() 的张量字典 (在设备上); rp: 参数 dict。"""
        dev = self.device
        K, T, nu = U.shape
        st = torch.from_numpy(np.asarray(state0, dtype=np.float32)).to(dev).reshape(1, -1).repeat(K, 1)
        act = torch.from_numpy(np.asarray(action0, dtype=np.float32)).to(dev).reshape(1, -1).repeat(K, 1)
        a_min = torch.from_numpy(np.asarray(action_min, dtype=np.float32)).to(dev)
        a_max = torch.from_numpy(np.asarray(action_max, dtype=np.float32)).to(dev)
        rw = torch.from_numpy(np.asarray(rate_weight, dtype=np.float32)).to(dev)
        Rt = torch.from_numpy(np.asarray(rp["R"], dtype=np.float32)).to(dev)
        wx, wy, wtx, wty, ws, wpsi, wl, wr, wv = win["x"], win["y"], win["tx"], win["ty"], win["s"], win["psi"], win["w_left"], win["w_right"], win["v_safe"]

        def project(px, py):
            d2 = (px.unsqueeze(1) - wx.unsqueeze(0)) ** 2 + (py.unsqueeze(1) - wy.unsqueeze(0)) ** 2   # (K, W)
            idx = torch.argmin(d2, dim=1)
            dx, dy = px - wx[idx], py - wy[idx]
            s = ws[idx] + dx * wtx[idx] + dy * wty[idx]
            e_lat = -dx * wty[idx] + dy * wtx[idx]
            return s, e_lat, idx

        self.begin_rollout(K)
        s_prev, _, _ = project(st[:, 0], st[:, 1])
        cost = torch.zeros(K, device=dev)
        for t in range(T):
            u = U[:, t, :]
            if input_lifting:
                act = self._lift(act, u, st, action_dt, a_min, a_max, steer_scale)
                cost = cost + (u * u * rw).sum(1)
            else:
                act = u
            st = self._step_torch(st, act)
            s_now, e_lat, idx = project(st[:, 0], st[:, 1])
            ds = torch.clamp(s_now - s_prev, -5.0, 10.0)
            s_prev = s_now
            dpsi = st[:, 2] - wpsi[idx]
            dpsi = torch.atan2(torch.sin(dpsi), torch.cos(dpsi))
            over_l = torch.relu(e_lat - (wl[idx] - rp["margin"]))
            over_r = torch.relu(-e_lat - (wr[idx] - rp["margin"]))
            over_v = torch.relu(st[:, 3] - rp["v_cap"])
            over_vs = torch.relu(st[:, 3] - rp["safe_slack"] * wv[idx])          # 时域中允许超离线剖面 slack 倍
            cost = (cost - rp["Q_prog"] * ds + rp["Q_lat"] * e_lat * e_lat + rp["Q_boundary"] * (over_l * over_l + over_r * over_r)
                    + rp["Q_vcap"] * over_v * over_v + rp["Q_vsafe"] * over_vs * over_vs
                    + rp["Q_vy"] * st[:, 4] * st[:, 4] + rp["Q_psi"] * dpsi * dpsi + (act * act * Rt).sum(1))
        # 终端速度软惩罚参考离线刹车包络；该代价项本身不保证制动可行性或安全。
        over_T = torch.relu(st[:, 3] - wv[idx])
        cost = cost + rp["Q_term"] * over_T * over_T
        return cost

    def polish_support(self, support: np.ndarray, interp: np.ndarray, u_min: np.ndarray, u_max: np.ndarray, sigma: np.ndarray,
                       n_steps: int, lr: float, trust: float, cost_fn):
        """KMPPI 改进 B1: 对一批支撑点值 (E,P,nu) 沿可微世界模型的代价梯度精修。变量 z 以噪声 sigma 为单位, |z| <= trust。
        每步: 一次反传得到梯度, 取归一化方向 -g/|g|, 批量试 lr*[1, 1/2, 1/4, 1/10] 四个步长 (一次前向), 每条样本取最优 (含不动)。
        (第一版用固定步长 Adam: 闭环里接受率只有 4-7%, 步长对已近似最优的精英样本太粗, 已弃用。)
        cost_fn(U (B,T,nu) tensor) -> (B,) tensor。返回 (精修后的支撑点值, 其 rollout 代价) numpy。"""
        dev = self.device
        sup0 = torch.from_numpy(np.asarray(support, dtype=np.float32)).to(dev)
        E = sup0.shape[0]
        M = torch.from_numpy(np.asarray(interp, dtype=np.float32)).to(dev)
        sig = torch.from_numpy(np.asarray(sigma, dtype=np.float32)).to(dev).reshape(1, 1, -1)
        umin = torch.from_numpy(np.asarray(u_min, dtype=np.float32)).to(dev)
        umax = torch.from_numpy(np.asarray(u_max, dtype=np.float32)).to(dev)
        alphas = torch.tensor([1.0, 0.5, 0.25, 0.1], device=dev) * float(lr)
        A = alphas.numel()

        def sequences(zz):
            sup = torch.minimum(torch.maximum(sup0.repeat(zz.shape[0] // E, 1, 1) + sig * zz, umin), umax)
            U = torch.einsum("tp,epu->etu", M, sup)
            return sup, torch.minimum(torch.maximum(U, umin), umax)

        z = torch.zeros_like(sup0)
        with torch.no_grad():
            _, U = sequences(z)
            best = cost_fn(U)
        for _ in range(int(n_steps)):
            zg = z.clone().requires_grad_(True)
            with torch.enable_grad():
                _, U = sequences(zg)
                cost = cost_fn(U)
                (g,) = torch.autograd.grad(cost.sum(), zg)
            if not torch.isfinite(g).all():
                break
            d = g / (g.reshape(E, -1).norm(dim=1).reshape(E, 1, 1) + 1e-9)
            with torch.no_grad():
                cand = torch.clamp(z.unsqueeze(0) - alphas.reshape(A, 1, 1, 1) * d.unsqueeze(0), -trust, trust)   # (A,E,P,nu)
                _, Uc = sequences(cand.reshape(A * E, *z.shape[1:]))
                cc = cost_fn(Uc).reshape(A, E)
                cbest, ai = cc.min(dim=0)
                improved = cbest < best
                if not bool(improved.any()):
                    break
                z = torch.where(improved.reshape(E, 1, 1), cand[ai, torch.arange(E, device=dev)], z)
                best = torch.where(improved, cbest, best)
        with torch.no_grad():
            sup, U = sequences(z)
            cost = cost_fn(U)
        return sup.cpu().numpy().astype(np.float64), cost.cpu().numpy().astype(np.float64)
