"""增广拉格朗日 iLQR（AL-iLQR，Gauss-Newton 形式），几个方案（拓扑类）一起批量求解。

问题（每个方案 h 各自独立）：
    min  ½ Σ_k ‖r(x_k, u_k, k)‖² + ½ ‖r_T(x_N)‖²
    s.t. x_{k+1} = f(x_k, u_k),   c(x_k, u_k, k) ≤ 0（k = 0..N−1），c_T(x_N) ≤ 0
不等式约束用增广拉格朗日（PHR）并入残差：每个约束一项 √μ · max(0, c + λ/μ)，
这样整个代价都是“残差平方和”，Hessian 用 Gauss-Newton 近似 JᵀJ（恒半正定，不用另加处理）。
雅可比用有限差分，所有方案、所有时刻、所有扰动方向一次批量算（numpy 向量化）。
实时迭代：每个控制周期只迭代 ITERS 次、更新一次乘子，热启动用上一周期的解（平移一个周期）。

problem 要提供：
    nx, nu, N, H
    dyn(x, u)            -> x_next                 形状 (..., nx), (..., nu)
    res(x, u, k, h)      -> (..., nr)  运行代价残差   k, h：与 x 同形状前缀的整数数组
    res_T(x, h)          -> (..., nrT) 终端代价残差
    con(x, u, k, h)      -> (..., nc)  运行约束（≤ 0 满足），已按量纲归一化
    con_T(x, h)          -> (..., ncT) 终端约束
"""
import numpy as np

EPS = 1e-5
ALPHAS = np.array([1.0, 0.5, 0.25, 0.1])


def _fd_jac(fun, x, u, eps=EPS):
    """fun(x, u) 对 (x, u) 的雅可比，有限差分：返回 f0 (..., m)，Jx (..., m, nx)，Ju (..., m, nu)。"""
    nx, nu = x.shape[-1], u.shape[-1]
    n = nx + nu
    pert = np.eye(n) * eps                                         # (n, n)
    xs = np.concatenate([x[..., None, :], x[..., None, :] + pert[None, :, :nx].reshape((1,) * (x.ndim - 1) + (n, nx))], axis=-2)
    us = np.concatenate([u[..., None, :], u[..., None, :] + pert[None, :, nx:].reshape((1,) * (u.ndim - 1) + (n, nu))], axis=-2)
    f = fun(xs, us)                                                # (..., n+1, m)
    f0 = f[..., 0, :]
    J = (f[..., 1:, :] - f0[..., None, :]) / eps                   # (..., n, m)
    J = np.swapaxes(J, -1, -2)                                     # (..., m, n)
    return f0, J[..., :nx], J[..., nx:]


class ALiLQR:
    def __init__(self, problem, mu0=10.0, mu_max=1e4, mu_grow=4.0, iters=2, reg0=1e-4):
        self.p = problem
        self.mu0, self.mu_max, self.mu_grow = mu0, mu_max, mu_grow
        self.iters, self.reg0 = iters, reg0

    # ------------------------------------------------------------------ helpers
    def _idx(self, H, N):
        k = np.broadcast_to(np.arange(N)[None, :], (H, N))
        h = np.broadcast_to(np.arange(H)[:, None], (H, N))
        return k, h

    def _al_res(self, c, lam, mu):
        """PHR 增广拉格朗日项写成残差：√μ · max(0, c + λ/μ)。"""
        return np.sqrt(mu) * np.maximum(0.0, c + lam / mu)

    def rollout(self, x0, U):
        """x0 (H, nx)，U (H, N, nu) -> X (H, N+1, nx)。"""
        H, N = U.shape[:2]
        X = np.empty((H, N + 1, x0.shape[-1]))
        X[:, 0] = x0
        for k in range(N):
            X[:, k + 1] = self.p.dyn(X[:, k], U[:, k])
        return X

    def total_cost(self, X, U, lam, mu, lamT, muT):
        """每个方案的总代价（含增广拉格朗日项），以及不含约束项的“原始代价”和最大约束违反量。"""
        p = self.p
        H, N = U.shape[:2]
        k, h = self._idx(H, N)
        r = p.res(X[:, :N], U, k, h)
        rT = p.res_T(X[:, N], np.arange(H))
        c = p.con(X[:, :N], U, k, h)
        cT = p.con_T(X[:, N], np.arange(H))
        base = 0.5 * (np.sum(r ** 2, axis=(1, 2)) + np.sum(rT ** 2, axis=1))
        al = 0.5 * (np.sum(self._al_res(c, lam, mu) ** 2 - lam ** 2 / mu, axis=(1, 2))
                    + np.sum(self._al_res(cT, lamT, muT) ** 2 - lamT ** 2 / muT, axis=1))
        viol = np.maximum(np.max(np.maximum(c[:, 1:], 0.0), axis=(1, 2)), np.max(np.maximum(cT, 0.0), axis=1))
        return base + al, base, viol, c, cT

    # ------------------------------------------------------------------ one solve
    def solve(self, x0, U, lam, mu, lamT, muT, reg=None):
        """从热启动 U 迭代 self.iters 次，再更新乘子。返回 dict（X, U, lam, mu, lamT, muT, cost, base, viol, reg）。"""
        p = self.p
        H, N, nu = U.shape
        nx = x0.shape[-1]
        x0 = np.broadcast_to(x0, (H, nx)).copy()
        reg = np.full(H, self.reg0) if reg is None else reg.copy()
        X = self.rollout(x0, U)
        J, base, viol, c, cT = self.total_cost(X, U, lam, mu, lamT, muT)
        k_idx, h_idx = self._idx(H, N)
        hT = np.arange(H)
        for _ in range(self.iters):
            # ---- 线性化（有限差分，全部批量）
            Xk = X[:, :N]
            _, A, B = _fd_jac(p.dyn, Xk, U)                                         # (H,N,nx,nx), (H,N,nx,nu)
            kk = np.broadcast_to(k_idx[..., None], (H, N, nx + nu + 1))
            hh = np.broadcast_to(h_idx[..., None], (H, N, nx + nu + 1))
            r0, Rx, Ru = _fd_jac(lambda x, u: p.res(x, u, kk, hh), Xk, U)
            c0, Cx, Cu = _fd_jac(lambda x, u: p.con(x, u, kk, hh), Xk, U)
            act = (c0 + lam / mu) > 0.0                                             # 起作用的约束
            sm = np.sqrt(mu) * act
            ra = sm * (c0 + lam / mu)
            Cx, Cu = Cx * sm[..., None], Cu * sm[..., None]
            lx = np.einsum("hkri,hkr->hki", Rx, r0) + np.einsum("hkri,hkr->hki", Cx, ra)
            lu = np.einsum("hkri,hkr->hki", Ru, r0) + np.einsum("hkri,hkr->hki", Cu, ra)
            lxx = np.einsum("hkri,hkrj->hkij", Rx, Rx) + np.einsum("hkri,hkrj->hkij", Cx, Cx)
            luu = np.einsum("hkri,hkrj->hkij", Ru, Ru) + np.einsum("hkri,hkrj->hkij", Cu, Cu)
            lux = np.einsum("hkri,hkrj->hkij", Ru, Rx) + np.einsum("hkri,hkrj->hkij", Cu, Cx)
            # 终端
            xT = X[:, N]
            zu = np.zeros((H, 1))
            hT1 = np.broadcast_to(hT[:, None], (H, nx + 2))
            rT0, RTx, _ = _fd_jac(lambda x, u: p.res_T(x, hT1), xT, zu)
            cT0, CTx, _ = _fd_jac(lambda x, u: p.con_T(x, hT1), xT, zu)
            smT = np.sqrt(muT) * ((cT0 + lamT / muT) > 0.0)
            raT = smT * (cT0 + lamT / muT)
            CTx = CTx * smT[..., None]
            Vx = np.einsum("hri,hr->hi", RTx, rT0) + np.einsum("hri,hr->hi", CTx, raT)
            Vxx = np.einsum("hri,hrj->hij", RTx, RTx) + np.einsum("hri,hrj->hij", CTx, CTx)
            # ---- 反向递推（Riccati）
            Kfb = np.empty((H, N, nu, nx))
            kff = np.empty((H, N, nu))
            eye_u = np.eye(nu)
            for k in range(N - 1, -1, -1):
                Ak, Bk = A[:, k], B[:, k]
                Qx = lx[:, k] + np.einsum("hji,hj->hi", Ak, Vx)
                Qu = lu[:, k] + np.einsum("hji,hj->hi", Bk, Vx)
                VA = Vxx @ Ak
                VB = Vxx @ Bk
                Qxx = lxx[:, k] + np.swapaxes(Ak, 1, 2) @ VA
                Quu = luu[:, k] + np.swapaxes(Bk, 1, 2) @ VB + reg[:, None, None] * eye_u
                Qux = lux[:, k] + np.swapaxes(Bk, 1, 2) @ VA
                Qinv = np.linalg.inv(Quu)
                K = -Qinv @ Qux
                kf = -np.einsum("hij,hj->hi", Qinv, Qu)
                Kfb[:, k], kff[:, k] = K, kf
                KT = np.swapaxes(K, 1, 2)
                Vx = Qx + np.einsum("hij,hj->hi", KT @ Quu, kf) + np.einsum("hij,hj->hi", KT, Qu) \
                    + np.einsum("hji,hj->hi", Qux, kf)
                Vxx = Qxx + KT @ Quu @ K + KT @ Qux + np.swapaxes(Qux, 1, 2) @ K
                Vxx = 0.5 * (Vxx + np.swapaxes(Vxx, 1, 2))
            # ---- 前向（所有步长一起推演，每个方案取代价下降最多的）
            nA = len(ALPHAS)
            Xn = np.empty((nA, H, N + 1, nx))
            Un = np.empty((nA, H, N, nu))
            Xn[:, :, 0] = x0
            al = ALPHAS[:, None, None]
            for k in range(N):
                dx = Xn[:, :, k] - X[None, :, k]
                Un[:, :, k] = U[None, :, k] + al * kff[None, :, k] + np.einsum("hij,ahj->ahi", Kfb[:, k], dx)
                Xn[:, :, k + 1] = p.dyn(Xn[:, :, k], Un[:, :, k])
            costs = np.empty((nA, H))
            for a in range(nA):
                costs[a] = self.total_cost(Xn[a], Un[a], lam, mu, lamT, muT)[0]
            best = np.argmin(costs, axis=0)
            improved = costs[best, hT] < J
            for h in range(H):
                if improved[h]:
                    X[h], U[h] = Xn[best[h], h], Un[best[h], h]
                    reg[h] = max(self.reg0, reg[h] / 3.0)
                else:
                    reg[h] = min(1e3, reg[h] * 10.0)
            J, base, viol, c, cT = self.total_cost(X, U, lam, mu, lamT, muT)
        # ---- 乘子更新（PHR）；没满足的约束加大罚因子
        lam = np.maximum(0.0, lam + mu * c)
        lamT = np.maximum(0.0, lamT + muT * cT)
        mu = np.where(c > 1e-3, np.minimum(mu * self.mu_grow, self.mu_max), mu)
        muT = np.where(cT > 1e-3, np.minimum(muT * self.mu_grow, self.mu_max), muT)
        return {"X": X, "U": U, "lam": lam, "mu": mu, "lamT": lamT, "muT": muT,
                "cost": J, "base": base, "viol": viol, "reg": reg}
