"""LearnedDynamics.rollout_cost_torch（world_model.py，你的整段 KMPPI 推演 + 代价）的 CUDA graph 版。

算的东西和 rollout_cost_torch 逐行一样（同一个 _lift、_step_torch、同样的代价项和加法顺序）：
    for t in range(T):  a = lift(a, u_t);  cost += rate 代价;  s = 一步世界模型预测;  cost += 状态代价 + 动作代价
只是把整段推演录成一个 CUDA graph，之后每次只把输入拷进固定的显存、重放一次。
原来的写法一步要发出几十个很小的 GPU 算子，CPU 侧发射开销把 T = 33 步的推演拖到 ~70 ms（RTX 3090 上
GPU 本身几乎是空的）；录成 graph 后一次推演约几 ms。
同一次重放还顺带留下每步的位置 (K, T, 2)，画候选轨迹不用再推一遍。

只覆盖 kmppi_dream 车道跟踪用到的代价（没有边界 / Frenet / 速度上限项、没有 A1 转向缩放）；
其它配置在 __init__ 里直接拒绝（调用方回退到原来的 rollout_cost）。
核对：tests/test_offline_kmppi_learned.py 里用随机动作序列比较两者的代价（float32 舍入误差以内）。
"""
import numpy as np
import torch


class GraphRollout:
    def __init__(self, dyn, ctrl, ref_box, cfg):
        cp = ctrl.cost_params
        if cp is None or cp.get("race") is not None or cp.get("frenet") or cp.get("Q_boundary") or cp.get("Q_vcap") \
                or ctrl.steer_scale is not None or not ctrl.input_lifting:
            raise ValueError("CUDA graph 版只覆盖车道跟踪的代价（没有边界 / Frenet / 速度上限 / A1 转向缩放）")
        dev = dyn.device
        if dev.type != "cuda":
            raise ValueError("CUDA graph 只能在 NVIDIA 显卡上用")
        self.dyn, self.ctrl, self.ref_box = dyn, ctrl, ref_box
        self.K, self.T, self.dt = int(cfg.K), int(cfg.T), float(ctrl.action_dt)
        f32 = dict(dtype=torch.float32, device=dev)
        self.U = torch.zeros(self.K, self.T, 2, **f32)
        self.s0 = torch.zeros(6, **f32)
        self.a0 = torch.zeros(2, **f32)
        self.ref = torch.zeros(self.T, 6, **f32)
        self.hseed = torch.zeros(max(dyn.H - 1, 0), 5, **f32)
        t = lambda x: torch.from_numpy(np.asarray(x, dtype=np.float32)).to(dev)
        self.a_min, self.a_max = t(ctrl.actual_action_min), t(ctrl.actual_action_max)
        self.rw, self.Q, self.R = t(ctrl.rate_weight), t(cp["Q"]), t(cp["R"])
        self._graph = torch.cuda.CUDAGraph()
        with torch.no_grad():
            side = torch.cuda.Stream()
            side.wait_stream(torch.cuda.current_stream())
            with torch.cuda.stream(side):
                for _ in range(3):   # 热身：cuBLAS / 内存池在录制前先建好
                    self._run()
            torch.cuda.current_stream().wait_stream(side)
            torch.cuda.synchronize()
            with torch.cuda.graph(self._graph):
                self._cost, self._xy = self._run()

    def _run(self):
        dyn, K = self.dyn, self.K
        st = self.s0.reshape(1, -1).repeat(K, 1)
        act = self.a0.reshape(1, -1).repeat(K, 1)
        dyn.hist = self.hseed.unsqueeze(0).repeat(K, 1, 1)   # begin_rollout(K)：每条候选从同一份真实历史开始
        cost = torch.zeros(K, device=self.U.device)
        xy = []
        for t in range(self.T):
            u = self.U[:, t, :]
            act = dyn._lift(act, u, st, self.dt, self.a_min, self.a_max, None)
            cost = cost + (u * u * self.rw).sum(1)
            st = dyn._step_torch(st, act)
            r = self.ref[t]
            dyaw = st[:, 2] - r[2]
            yaw_err = torch.atan2(torch.sin(dyaw), torch.cos(dyaw))
            err = torch.stack([st[:, 0] - r[0], st[:, 1] - r[1], yaw_err, st[:, 3] - r[3], st[:, 4] - r[4], st[:, 5] - r[5]], dim=1)
            cost = cost + (err * err * self.Q).sum(1) + (act * act * self.R).sum(1)
            xy.append(st[:, :2])
        return cost, torch.stack(xy, dim=1)

    def __call__(self, seq):
        """KMPPIController._rollout_cost 的替身：(K, T, 2) 导数动作序列 → (K,) 代价（float64 numpy）。"""
        dyn, ctrl = self.dyn, self.ctrl
        f32 = lambda x: torch.from_numpy(np.ascontiguousarray(x, dtype=np.float32))
        self.U.copy_(f32(seq))
        self.s0.copy_(f32(ctrl.current_state))
        self.a0.copy_(f32(ctrl.applied_action))
        self.ref.copy_(f32(self.ref_box.val))
        if dyn.H > 1:
            self.hseed.copy_(f32(dyn._hist_seed))
        self._graph.replay()
        return self._cost.cpu().numpy().astype(np.float64)

    def positions(self):
        """最近一次重放里每条候选每步的位置 (K, T, 2)，float64 numpy。"""
        return self._xy.cpu().numpy().astype(np.float64)
