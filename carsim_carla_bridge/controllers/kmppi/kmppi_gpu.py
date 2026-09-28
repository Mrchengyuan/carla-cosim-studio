"""KMPPI 的 GPU 推演：K 条候选的 T 步推演和代价（KMPPIController._rollout_cost，占每次计算的绝大部分时间）
放到一个 CUDA 核里，一个线程一条候选。其余（采样、RBF 插值、权重和自适应温度）还是原来的 numpy 代码，
只是 RBF 插值 einsum("tp,kpu->ktu") 换成等价的矩阵乘（快十几倍，差 ~1e-15）。

核里的预测模型和代价与 prediction_model.BicycleModel.step、kmppi_controller.build_kmppi 的 time_cost 逐行对应，
双精度、不做乘加合并（--fmad=false）、运算顺序相同：和 CPU 版的差别只来自 GPU 的 sin / cos / atan2
（与 CPU 的数学库差最后一两位），代价的相对差 ~1e-15（tests/test_offline_kmppi_gpu.py 核对）。

需要 NVIDIA 显卡和 CuPy（pip install cupy-cuda12x）；没有时 available() 给出原因，控制器用 CPU 版。
"""
import os

import numpy as np

_KERNEL = r"""
__device__ double pac(double alpha, double B, double C, double D, double E) {
  double Ba = B * alpha;
  return D * sin(C * atan(Ba - E * (Ba - atan(Ba))));
}
__device__ double clip(double v, double lo, double hi) { return fmin(fmax(v, lo), hi); }

extern "C" __global__ void rollout(const double* seq, const double* x0, const double* a0, const double* ref,
                                   const double* p, double* cost, double* xy, int K, int T, int record) {
  int k = blockDim.x * blockIdx.x + threadIdx.x;
  if (k >= K) return;
  // p: action_dt, act_min0, act_min1, act_max0, act_max1, rate_w0, rate_w1,
  //    dt, ax_max, delta_max, a, b, m, I, fls, rls, By, Cy, Dy, Ey, Q0..Q5, R0, R1
  const double adt = p[0], dt = p[7], ax_max = p[8], delta_max = p[9], a = p[10], b = p[11], m = p[12], I = p[13];
  double X = x0[0], Y = x0[1], yaw = x0[2], vx0 = x0[3], vy = x0[4], r = x0[5];
  double act0 = a0[0], act1 = a0[1], c = 0.0;
  for (int t = 0; t < T; ++t) {
    const double u0 = seq[(k * T + t) * 2], u1 = seq[(k * T + t) * 2 + 1];
    // input lifting (KMPPIController._rollout_cost)
    act0 = clip(act0 + adt * u0, p[1], p[3]);
    act1 = clip(act1 + adt * u1, p[2], p[4]);
    c = c + (u0 * u0 * p[5] + u1 * u1 * p[6]);
    // BicycleModel.step
    const double vx = fmax(vx0, 0.5);
    const double ax = clip(act0, -ax_max, ax_max), delta = clip(act1, -delta_max, delta_max);
    const double alpha_f = delta - atan2(vy + a * r, vx);
    const double alpha_r = -atan2(vy - b * r, vx);
    const double Fy_f = 2.0 * p[14] * pac(alpha_f, p[16], p[17], p[18], p[19]);
    const double Fy_r = 2.0 * p[15] * pac(alpha_r, p[16], p[17], p[18], p[19]);
    const double Fx = m * ax;
    const double cd = cos(delta), sd = sin(delta);
    const double vx_dot = (Fx - Fy_f * sd + m * vy * r) / m;
    const double vy_dot = (Fy_f * cd + Fy_r - m * vx * r) / m;
    const double r_dot = (a * Fy_f * cd - b * Fy_r) / I;
    const double cy = cos(yaw), sy = sin(yaw);
    const double x_dot = vx * cy - vy * sy;
    const double y_dot = vx * sy + vy * cy;
    X = X + dt * x_dot;
    Y = Y + dt * y_dot;
    const double yw = yaw + dt * r;
    yaw = atan2(sin(yw), cos(yw));
    vx0 = fmax(vx + dt * vx_dot, 0.0);
    vy = vy + dt * vy_dot;
    r = r + dt * r_dot;
    if (record) { xy[(k * T + t) * 2] = X; xy[(k * T + t) * 2 + 1] = Y; }
    // time_cost (build_kmppi)
    const double* rf = ref + t * 6;
    const double d2 = yaw - rf[2];
    const double e0 = X - rf[0], e1 = Y - rf[1], e2 = atan2(sin(d2), cos(d2)), e3 = vx0 - rf[3], e4 = vy - rf[4], e5 = r - rf[5];
    const double sq = e0 * e0 * p[20] + e1 * e1 * p[21] + e2 * e2 * p[22] + e3 * e3 * p[23] + e4 * e4 * p[24] + e5 * e5 * p[25];
    c = c + (sq + (act0 * act0 * p[26] + act1 * act1 * p[27]));
  }
  cost[k] = c;
}
"""

_cp = None
_why = None


def available():
    """(True, 显卡名) 或 (False, 原因)。"""
    global _cp, _why
    if _cp is None and _why is None:
        if not os.environ.get("CUDA_PATH") and os.path.isdir("/usr/local/cuda"):
            os.environ["CUDA_PATH"] = "/usr/local/cuda"   # CuPy 编译核时要找 CUDA 头文件
        try:
            import cupy
            n = cupy.cuda.runtime.getDeviceCount()
            if n < 1:
                raise RuntimeError("没有 NVIDIA 显卡")
            _cp = cupy
        except Exception as e:   # noqa: BLE001 (没装 CuPy、没有驱动、没有显卡)
            _why = "%s: %s" % (type(e).__name__, str(e).splitlines()[0] if str(e) else "")
    if _cp is None:
        return False, _why
    name = _cp.cuda.runtime.getDeviceProperties(0)["name"]
    return True, name.decode() if isinstance(name, bytes) else name


class GPURollout:
    """替换 ctrl._rollout_cost（和 _sample_perturbed_sequences，见上）：同样的输入输出（每条候选的代价 (K,)）。recorder.on 时把各步位置
    交给 recorder.xy（和 CPU 版包在预测模型外的 _Recorder 一样：T 个 (K, 2)）。"""

    def __init__(self, ctrl, model, cfg, reference_box, recorder=None):
        ok, why = available()
        if not ok:
            raise RuntimeError("GPU 不可用：%s" % why)
        cp = _cp
        if not ctrl.input_lifting:
            raise ValueError("GPU 推演只实现了 input lifting（build_kmppi 的设置）")
        self.cp, self.ctrl, self.box, self.rec = cp, ctrl, reference_box, recorder
        pv = model.p
        prm = [ctrl.action_dt, *ctrl.actual_action_min, *ctrl.actual_action_max, *ctrl.rate_weight,
               model.dt, model.ax_max, model.delta_max, pv.a, pv.b, pv.m, pv.I, pv.front_lateral_scale,
               pv.rear_lateral_scale, pv.pac_By, pv.pac_Cy, pv.pac_Dy, pv.pac_Ey, *cfg.Qdiag, *cfg.Rdiag]
        assert len(prm) == 28, len(prm)
        self.p = cp.asarray(np.asarray(prm, dtype=np.float64))
        self.kernel = cp.RawKernel(_KERNEL, "rollout", options=("--fmad=false",))
        K, T = ctrl.K, ctrl.T
        self.cost = cp.empty(K, dtype=np.float64)
        self.xy = cp.empty((K, T, 2), dtype=np.float64)
        ctrl._sample_perturbed_sequences = self._sample

    def _sample(self):
        """KMPPIController._sample_perturbed_sequences，插值用 matmul（BLAS）代替 einsum。"""
        c = self.ctrl
        raw = c._sample_noise()                                     # (K,P,nu)
        center = c.support_values.reshape(1, c.P, c.nu)
        perturbed_support = c._bound_optimizer(center + raw)
        support_noise = perturbed_support - center
        perturbed_sequence = c._bound_optimizer(np.matmul(c.interpolation_matrix, perturbed_support))  # (K,T,nu)
        sequence_noise = perturbed_sequence - c.nominal_sequence.reshape(1, c.T, c.nu)
        return perturbed_sequence, support_noise, sequence_noise

    def __call__(self, perturbed_sequence):
        cp, c = self.cp, self.ctrl
        K, T = c.K, c.T
        record = bool(self.rec is not None and self.rec.on)
        args = (cp.asarray(np.ascontiguousarray(perturbed_sequence, dtype=np.float64)),
                cp.asarray(np.asarray(c.current_state, dtype=np.float64)),
                cp.asarray(np.asarray(c.applied_action, dtype=np.float64)),
                cp.asarray(np.ascontiguousarray(self.box.val[:T], dtype=np.float64)),
                self.p, self.cost, self.xy, np.int32(K), np.int32(T), np.int32(record))
        threads = 128
        self.kernel(((K + threads - 1) // threads,), (threads,), args)
        if record:
            xy = cp.asnumpy(self.xy)
            self.rec.xy = [xy[:, t, :] for t in range(T)]
        return cp.asnumpy(self.cost)
